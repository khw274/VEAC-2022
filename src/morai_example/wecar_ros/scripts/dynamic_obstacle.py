#!/usr/bin/env python3
# -*- coding: utf-8 -*-

# 필요한 라이브러리와 메시지 타입을 import 합니다.
import rospy
import math
import numpy as np
from morai_msgs.msg import GetTrafficLightStatus, EgoVehicleStatus
from std_msgs.msg import Float64, Int32
from sensor_msgs.msg import PointCloud
from geometry_msgs.msg import Point32, PointStamped, Point
from nav_msgs.msg import Path
import time

''' 장애물 종류에 대한 설명:
    dynamic = 움직임 있는 장애물, 넓이가 크다.
    static = 움직임 없는 장애물, 넓이가 작다.
    cone = 움직임 없는 장애물, 넓이가 크다. '''

class DynamicStop:
    def __init__(self):
        # ROS 노드를 초기화합니다.
        rospy.init_node("obstacle_mission", anonymous=False)

        # 각 토픽에 대한 Subscriber를 정의합니다.
        rospy.Subscriber("/laser2pcd_map", PointCloud, self.map_callback)
        rospy.Subscriber('/laser2pcd', PointCloud, self.callback)
        rospy.Subscriber("/lattice_path_2", Path, self.local_cb)
        rospy.Subscriber("/Ego_topic", EgoVehicleStatus, self.ego_cb)
        rospy.Subscriber("/global_path", Path, self.global_cb)
        rospy.Subscriber("/current_waypoint", Int32, self.waypoint_cb)

        # 각 토픽에 대한 Publisher를 정의합니다.
        self.vel_pub = rospy.Publisher("/dynamic_vel", Float64, queue_size=1)
        self.static_obstacle = rospy.Publisher("/static_obstacle", PointCloud, queue_size=1)
        self.selected_pub = rospy.Publisher("/selected_lane", Int32, queue_size=1)
        self.str_pub = rospy.Publisher('/cone_str', Float64, queue_size=1)

        # 변수를 초기화합니다.
        '''callback boolean'''
        self.is_status = False  # Ego 차량 상태 콜백 여부
        self.is_local_path = False  # Local path 콜백 여부
        '''static mission parameter'''
        self.tic_tok = 0  # 정적 장애물 카운터
        self.static_mission = False  # 정적 장애물 미션 여부
        self.static_obj_mean = None  # 정적 장애물의 평균 위치
        '''cone mission'''
        self.cone_mission = False  # 콘 미션 여부
        self.obj_width = 0  # 장애물의 너비

    # 주어진 포인트 리스트에서 x, y의 평균 위치를 계산하는 함수
    def get_mean(self, points):
        points_np = np.array(points)
        x_mean = np.mean(points_np.T[0])
        y_mean = np.mean(points_np.T[1])
        return [x_mean, y_mean]

    # 두 점 사이의 거리를 계산하는 함수
    def get_dst(self, p1, p2):
        dx = p1[0] - p2[0]
        dy = p1[1] - p2[1]
        dst = math.sqrt((dx*dx)+(dy*dy))
        return dst

    # 레이더 맵 데이터를 처리하는 콜백 함수
    def map_callback(self, msg):
        # 포인트 클라우드 데이터를 리스트로 변환합니다.
        pcd_list = []
        for i in msg.points:
            pcd = [i.x, i.y]
            pcd_list.append(pcd)

        ''' 알고리즘 부분 '''
        if self.is_status and self.is_local_path:  # 데이터가 모두 콜백된 상태에서 알고리즘을 실행합니다.
            if self.cone_mission is False:  # 콘 미션 제어값과 겹치지 않기 위한 조건문
                # 로컬 파라미터 초기화
                stop_distance = 1.35  # 정지 거리 설정
                dynamic_vel = 999  # 초기 동적 속도
                selected_lane = 1  # 초기 선택된 차선
                if self.static_mission is False:  # 주행 차선에 장애물이 있다면 장애물 위치를 특정하기 위한 조건문
                    local_obstacle_list = []
                    '''로컬 경로에 장애물이 존재하는지 검사
                    로컬 경로 길이는 속도가 클수록 늘어남'''
                    for i in pcd_list:
                        for j in self.local_path_list:
                            dst = self.get_dst(i, j)
                            if dst < 0.15:  # 차선 폭은 대략 0.36m, 주행 차선 중앙 기준 양쪽 0.15m 장애물 여부 확인
                                local_obstacle_list.append(i)
                                print('전방에 장애물')
                            elif dst < 1.25:  # 주행 차선 근처에 장애물이 감지되면 서행 -> 동적 장애물이 주행 차로에 다가올 때를 대비
                                dynamic_vel = 2.0
                                print('근처에 장애물')

                    for i in local_obstacle_list:  # local_obstacle_list는 차선 위에 존재하는 장애물 포인트
                        dst = self.get_dst(i, self.ego_pos)
                        if dst < stop_distance:  # 주행 차선에 위치한 장애물이 ego 차량과 가깝고 콘 미션이 아니라면 정지
                            if 215 <= self.current_waypoint <= 236:  # 로타리 구간이라면 dynamic_vel 비활성화
                                dynamic_vel = 999
                            else:
                                dynamic_vel = 0  # 로타리 구간이 아니라면 정지
                                print('충돌 대비 정지')

                    # 정적, 동적 구분 -> 동적 장애물이라면 주행 차선에 머물지 않음
                    if dynamic_vel == 0:
                        self.tic_tok += 1
                    else:
                        self.tic_tok = 0

                    # 10동안 장애물이 주행 차선에 머물기 때문에 정적 장애물로 판단
                    if self.tic_tok >= 30:
                        if self.obj_width > 0.7:  # 정적 장애물 중에서 장애물 간격이 크다면 콘 미션
                            self.cone_mission = True
                            self.static_mission = False
                        else:
                            self.cone_mission = False
                            self.static_mission = True
                            self.static_obj_mean = self.get_mean(local_obstacle_list)  # 회피 여부를 결정하기 위해 정지해있을때의 장애물 위치 기억

                # 정적 장애물 회피
                elif self.static_mission:
                    print('정적 장애물 미션 중')
                    # 왼쪽으로 이동 == select lane = 0
                    dynamic_vel = 2
                    selected_lane = 0
                    # print("지역 경로 내 장애물 감지 : 회피 시작")

                    # 기억한 장애물 위치가 현재 차량 위치와의 거리에서 멀어지면 정적 미션 종료
                    dst = self.get_dst(self.ego_pos, self.static_obj_mean)
                    if dst < 0.5:  # 장애물을 회피하면서 가까워지면 stop_distance를 줄여 원경로로 빨리 복귀할 수 있도록 함
                        stop_distance = 0.5

                    if dst > stop_distance:
                        # print(dst)
                        self.static_mission = False

                # 속도와 선택된 차선을 퍼블리시합니다.
                self.vel_pub.publish(dynamic_vel)
                self.selected_pub.publish(selected_lane)

    # 라이더 데이터를 처리하는 콜백 함수
    def callback(self, msg):
        lfd = 0.5  # look forward distance
        track_width = 1.2  # 트랙의 폭
        offset = track_width / 2  # 오프셋 계산
        if self.is_status and self.is_local_path:  # 데이터가 모두 콜백된 상태에서 알고리즘을 실행합니다.
            around_obj_list = []
            for i in msg.points:
                dis = self.get_dst([i.x, i.y], [0,0])
                if dis < 1.3:
                    around_obj_list.append([i.x, i.y])  # ego 주변의 장애물만 검출

            left_objs = []
            right_objs = []
            for i in around_obj_list:  # 라이더 좌표계 y축을 기준으로 좌우 구분
                if i[1] > 0:
                    left_objs.append(i)
                else:
                    right_objs.append(i)

            curr_right, curr_left = [0,0], [0,0]  # ego와 가장 가까운 좌우 장애물 하나씩 검출
            min_dst = 999
            for obj in left_objs:
                dst = self.get_dst(obj, self.ego_pos)
                if min_dst > dst:
                    min_dst = dst
                    curr_left = obj
            min_dst = 999
            for obj in right_objs:
                dst = self.get_dst(obj, self.ego_pos)
                if min_dst > dst:
                    min_dst = dst
                    curr_right = obj

            self.obj_width = self.get_dst(curr_left, curr_right)  # 좌우 장애물 간격이 일정 수치보다 크다면 콘 미션으로 인지
            print(self.obj_width)

            if self.cone_mission:
                print('콘 미션 중 : {}'.format(self.obj_width))
                min_dst = 999
                cur_cone = [0,0]
                for cone in right_objs:
                    dst = self.get_dst(cone, self.ego_pos)
                    if min_dst> dst - lfd and dst - lfd > 0:
                        min_dst=dst
                        cur_cone = cone  # lfd 만큼 앞에 있는 오른쪽 콘 중 가장 가까운 콘 검출

                str_p = [cur_cone[0], cur_cone[1] + offset]  # 오른쪽 가장 가까운 콘 기준 왼쪽으로 트랙 폭의 반만큼 오프셋하여 조향점 선정
                str_rad = self.get_str_angle(str_p)
                str_deg = math.degrees(str_rad)

                if self.obj_width < 0.7:
                    self.cone_mission = False  # cone mission 종료

                self.str_pub.publish(str_deg)
                self.vel_pub.publish(Float64(1.5))

    # 조향각을 계산하는 함수
    def get_str_angle(self, steer_p, wheel_base=0.39):
        steer_x = steer_p[1]
        steer_y = steer_p[0]

        if steer_x == 0:
            r = 9999999999
        else:
            r = -(steer_x / 2) - steer_y**2 / (2 * steer_x)

        str_rad = wheel_base / r

        return str_rad

    # 로컬 경로를 받아오는 콜백 함수
    def local_cb(self, msg):
        self.local_path_list = []
        for i in msg.poses:
            path = []
            path.append(i.pose.position.x)
            path.append(i.pose.position.y)

            self.local_path_list.append(path)

        self.is_local_path = True

    # 글로벌 경로를 받아오는 콜백 함수
    def global_cb(self, msg):
        self.global_path_list = []
        for i in msg.poses:
            path = []
            path.append(i.pose.position.x)
            path.append(i.pose.position.y)

            self.global_path_list.append(path)

    # Ego 차량의 상태를 받아오는 콜백 함수
    def ego_cb(self, msg):
        self.ego_pos = [msg.position.x, msg.position.y]

        self.is_status = True

    # 현재 웨이포인트를 받아오는 콜백 함수
    def waypoint_cb(self, msg):
        self.current_waypoint = msg.data

# 노드를 실행합니다.
if __name__ == "__main__":
    ts = DynamicStop()
    rospy.spin()
