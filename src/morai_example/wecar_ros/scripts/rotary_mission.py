#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import math
from std_msgs.msg import Float64, Int32
from sensor_msgs.msg import PointCloud
from nav_msgs.msg import Path

# 회전 교차로의 경로 좌표 설정
rotary_waypoints = [
    [12.793862342834473, 1.3633406162261963],
    [12.487863540649414, 1.3097057342529297]
]

class RotaryVel:
    def __init__(self):
        rospy.init_node("rotary_mission", anonymous=False)

        # subscriber 설정
        rospy.Subscriber("/laser2pcd_map", PointCloud, self.callback)
        rospy.Subscriber("/local_path", Path, self.local_cb)
        rospy.Subscriber("/Ego_topic", EgoVehicleStatus, self.ego_cb)
        rospy.Subscriber("/current_waypoint", Int32, self.waypoint_cb)

        # publisher 설정
        self.vel_pub = rospy.Publisher("/rotary_vel", Float64, queue_size=10)
        self.stopvel_pub = rospy.Publisher("/stop_vel", Float64, queue_size=10)

        # 초기값 설정
        self.rotary_vel = 999
        self.stop_vel = 999

    def callback(self, msg):
        checking = True

        self.pcd_list = []
        for i in msg.points:
            pcd = [i.x, i.y]
            self.pcd_list.append(pcd)

        # rotary_waypoints와의 거리 계산하여 장애물 검출
        for i in self.pcd_list:
            for j in rotary_waypoints:
                dx = i[0] - j[0]
                dy = i[1] - j[1]
                dst = math.sqrt(dx * dx + dy * dy)
                
                if dst < 1:
                    checking = False
                else:
                    checking = True
                    
                # 현재 waypoint가 213 또는 214일 때, 장애물 여부에 따라 정지 속도 설정
                if 213 <= self.current_waypoint <= 214:
                    if checking == False:
                        self.stop_vel = 0
                    elif checking == True:
                        self.stop_vel = 999
                else:
                    self.stop_vel = 999

        self.stopvel_pub.publish(self.stop_vel)

        # 로컬 경로에서 장애물 여부 확인하여 rotary_vel 설정
        obstacle_list = []
        for i in self.pcd_list:
            for j in self.local_path_list:
                dx = i[0] - j[0]
                dy = i[1] - j[1]
                dst = math.sqrt(dx * dx + dy * dy)
                if dst < 0.4:
                    obstacle_list.append(i)

        for i in obstacle_list:
            dx = i[0] - self.ego_x
            dy = i[1] - self.ego_y
            dst = math.sqrt(dx * dx + dy * dy)
            
            # 현재 waypoint가 215부터 236일 때, 거리에 따라 rotary_vel 설정
            if 215 <= self.current_waypoint <= 236:
                if dst < 0.5:
                    self.rotary_vel = 0
            else:
                self.rotary_vel = 999

        self.vel_pub.publish(self.rotary_vel)

    def local_cb(self, msg):
        self.local_path_list = []
        for i in msg.poses:
            path = [i.pose.position.x, i.pose.position.y]
            self.local_path_list.append(path)

    def ego_cb(self, msg):
        self.ego_x = msg.position.x
        self.ego_y = msg.position.y
    
    def waypoint_cb(self, msg):
        self.current_waypoint = msg.data

if __name__ == "__main__":
    ts = RotaryVel()
    rospy.spin()
