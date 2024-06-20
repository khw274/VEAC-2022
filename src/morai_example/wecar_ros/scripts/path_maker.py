#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import sys
import rospy
import rospkg
from morai_msgs.msg import EgoVehicleStatus  # morai_msgs 패키지에서 EgoVehicleStatus 메시지를 임포트합니다.
from math import pi, cos, sin, sqrt, pow
from nav_msgs.msg import Path  # nav_msgs 패키지에서 Path 메시지를 임포트합니다.
import tf
from geometry_msgs.msg import PoseStamped  # geometry_msgs 패키지에서 PoseStamped 메시지를 임포트합니다.

class test:
    
    def __init__(self):
        rospy.init_node('path_maker', anonymous=True)  # 'path_maker'라는 이름의 ROS 노드를 초기화합니다.

        arg = rospy.myargv(argv=sys.argv)
        self.path_folder_name = arg[1]  # 첫 번째 인자는 경로 폴더 이름입니다.
        self.make_path_name = arg[2]   # 두 번째 인자는 생성할 경로 이름입니다.

        rospy.Subscriber("/Ego_topic", EgoVehicleStatus, self.status_callback)  # '/Ego_topic' 토픽에서 EgoVehicleStatus 메시지를 구독하고, 수신될 때마다 status_callback 메소드를 호출합니다.
        self.global_path_pub = rospy.Publisher('/global_path', Path, queue_size=1)  # '/global_path' 토픽으로 Path 메시지를 발행할 Publisher를 설정합니다.

        self.is_status = False  # 차량 상태 수신 여부를 나타내는 플래그입니다.
        self.prev_x = 0  # 이전 x 위치를 저장하는 변수입니다.
        self.prev_y = 0  # 이전 y 위치를 저장하는 변수입니다.

        rospack = rospkg.RosPack()  # rospack을 사용하여 ROS 패키지의 경로를 가져옵니다.
        pkg_path = rospack.get_path('wecar_ros')  # 'wecar_ros' 패키지의 경로를 가져옵니다.
        full_path = pkg_path + '/' + self.path_folder_name + '/' + self.make_path_name + '.txt'  # 저장할 파일의 전체 경로를 설정합니다.
        self.f = open(full_path, 'w')  # 파일을 쓰기 모드로 엽니다.

        rate = rospy.Rate(30)  # 주기를 30Hz로 설정합니다.
        while not rospy.is_shutdown():
            if self.is_status == True:
                self.path_make()  # 차량 상태를 받은 후 경로를 생성하는 메소드를 호출합니다.
            rate.sleep()

        self.f.close()  # 파일을 닫습니다.

    def path_make(self):
        x = self.status_msg.position.x  # 현재 차량의 x 위치를 가져옵니다.
        y = self.status_msg.position.y  # 현재 차량의 y 위치를 가져옵니다.
        z = self.status_msg.position.z  # 현재 차량의 z 위치를 가져옵니다.
        distance = sqrt(pow(x - self.prev_x, 2) + pow(y - self.prev_y, 2))  # 이전 위치에서 현재 위치까지의 거리를 계산합니다.
        if distance > 0.3:  # 이동 거리가 0.3보다 크면
            data = '{0}\t{1}\t{2}\n'.format(x, y, z)  # 파일에 저장할 데이터를 포맷합니다.
            self.f.write(data)  # 데이터를 파일에 씁니다.
            self.prev_x = x  # 이전 x 위치를 현재 위치로 업데이트합니다.
            self.prev_y = y  # 이전 y 위치를 현재 위치로 업데이트합니다.
            print(x, y)  # 현재 위치를 출력합니다.

    def status_callback(self, msg):
        self.is_status = True  # 차량 상태 수신을 확인합니다.
        self.status_msg = msg  # 수신된 메시지를 저장합니다.
        br = tf.TransformBroadcaster()  # TransformBroadcaster 객체를 생성합니다.
        br.sendTransform((self.status_msg.position.x, self.status_msg.position.y, self.status_msg.position.z),  # 현재 차량의 위치를 기준으로 변환을 방송합니다.
                         tf.transformations.quaternion_from_euler(0, 0, (self.status_msg.heading + 90) / 180 * pi),  # 현재 차량의 방향을 기준으로 변환을 방송합니다.
                         rospy.Time.now(),  # 현재 시간을 가져와서 방송합니다.
                         "gps",  # 차량의 위치를 나타내는 프레임입니다.
                         "map")  # 맵 프레임입니다.

if __name__ == '__main__':
    try:
        test_track = test()  # test 클래스의 인스턴스를 생성합니다.
    except rospy.ROSInterruptException:
        pass
