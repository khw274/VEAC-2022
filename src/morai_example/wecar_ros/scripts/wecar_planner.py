#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import sys,os
import rospy
import rospkg
import numpy as np
from nav_msgs.msg import Path
from std_msgs.msg import Float64,Int32
from geometry_msgs.msg import PointStamped
from morai_msgs.msg import EgoVehicleStatus
from lib.utils import pathReader, findLocalPath,purePursuit,velocityPlanning,latticePlanner
import tf
from math import cos,sin,sqrt,pow,atan2,pi


class wecar_planner():
    def __init__(self):
        rospy.init_node('wecar_planner', anonymous=True)

        arg = rospy.myargv(argv=sys.argv)
        self.path_name=arg[1]

        #publisher
        global_path_pub= rospy.Publisher('/global_path',Path, queue_size=1) ## global_path publisher
        local_path_pub= rospy.Publisher('/local_path',Path, queue_size=1) ## local_path publisher
        motor_pub = rospy.Publisher('commands/motor/speed',Float64, queue_size=1)
        servo_pub = rospy.Publisher('commands/servo/position',Float64, queue_size=1)
        wayp_pub = rospy.Publisher('/current_waypoint', Int32, queue_size=1)
        steer_point_pub = rospy.Publisher('/steer_point', PointStamped, queue_size = 1)

        ########################  lattice  ########################
        for i in range(1,8):            
            globals()['lattice_path_{}_pub'.format(i)]=rospy.Publisher('lattice_path_{}'.format(i),Path,queue_size=1)  
        ########################  lattice  ########################
        
        #subscriber
        rospy.Subscriber("/Ego_topic", EgoVehicleStatus, self.statusCB) ## Vehicl Status Subscriber 

        # mission velocity subscriber
        rospy.Subscriber("/traffic_vel", Float64, self.traffic_set)
        rospy.Subscriber("/dynamic_vel", Float64, self.dynamic_set)
        rospy.Subscriber("/rotary_vel", Float64, self.rotary_set)
        rospy.Subscriber("/stop_vel", Float64, self.stop_set)

        rospy.Subscriber("/selected_lane", Int32, self.selected_set)
        rospy.Subscriber("/cone_str", Float64, self.str_set)

        #def
        self.is_status=False ## 차량 상태 점검
        self.is_obj=False ## 장애물 상태 점검

        # 속도 초기화
        self.traffic_vel = 999   ## 신호등
        self.dynamic_vel = 999   ## 동적장애물
        self.rotary_vel = 999 ## 로타리장애물
        self.stop_vel = 999 ## 로타리 진입

        # 조향 초기화
        self.cone_str = 0

        #값 초기화
        self.selected_lane = 1
        self.steering_angle_to_servo_offset=0.500 ## servo moter offset
        self.rpm_gain = 1000
        self.motor_msg=Float64()
        self.servo_msg=Float64()
        
        #class
        path_reader=pathReader('wecar_ros') ## 경로 파일의 위치
        pure_pursuit=purePursuit() ## purePursuit import

        #read path
        self.global_path=path_reader.read_txt(self.path_name+".txt") ## 출력할 경로의 이름
        vel_planner=velocityPlanning(8, 0.15) ## 속도 계획
        vel_profile=vel_planner.curveBasedVelocity(self.global_path, 20)

        #time var
        count=0
        rate = rospy.Rate(30) # 30hz
        lattice_current_lane=1
        while not rospy.is_shutdown():   
            if self.is_status: ## 차량의 상태, 장애물 상태 점검
                ## global_path와 차량의 status_msg를 이용해 현재 waypoint와 local_path를 생성
                local_path, self.current_waypoint=findLocalPath(self.global_path,self.status_msg) 
                wayp_pub.publish(self.current_waypoint)

                ## 장애물의 숫자와 Type 위치 속도 (object_num, object type, object pose_x, object pose_y, object velocity)
                global_obj = []
                ########################  lattice  ########################
                current_pos = self.global_path.poses[self.current_waypoint]
                vehicle_status=[current_pos.pose.position.x, current_pos.pose.position.y, (self.status_msg.heading)/180*pi, self.status_msg.velocity.x]
                lattice_path,selected_lane=latticePlanner(local_path,global_obj,vehicle_status,lattice_current_lane)
                lattice_current_lane=selected_lane

                if selected_lane != -1: 
                    local_path=lattice_path[self.selected_lane]                

                if len(lattice_path)==2:                    
                    for i in range(1,3):
                        globals()['lattice_path_{}_pub'.format(i)].publish(lattice_path[i-1])
                ########################  lattice  ########################

                pure_pursuit.getPath(local_path) ## pure_pursuit 알고리즘에 Local path 적용
                pure_pursuit.getEgoStatus(self.status_msg) ## pure_pursuit 알고리즘에 차량의 status 적용

                # 조향점 rviz에 출력
                steer_point = PointStamped()
                steer_point.header.frame_id = '/map'
                steer_point.point = pure_pursuit.forward_point

                self.steering=pure_pursuit.steering_angle()
                
                self.cc_vel = vel_profile[self.current_waypoint] ## advanced cruise control 적용한 속도 계획
 
                # 속도 Filtering
                spd_list = []
                spd_list.append(self.cc_vel) # wecar planner 계획한 현재 속도
                spd_list.append(self.traffic_vel)
                spd_list.append(self.dynamic_vel)
                spd_list.append(self.rotary_vel)
                spd_list.append(self.stop_vel)


                # 속도 리스트 중에서 "최솟값"
                min_vel = min(spd_list)

                # print("waypoint : {:0f}".format(self.current_waypoint))
                # print("min_vel {:.2f} / cc_vel : {:.2f} traffic_vel : {:.2f} dynamic_vel : {:.2f} rotary_vel : {:.2f}".format(min_vel, self.cc_vel, self.traffic_vel, self.dynamic_vel, self.rotary_vel))
                # print("min_vel {:.2f} / cc_vel : {:.2f} traffic_vel : {:.2f} dynamic_vel : {:.2f}".format(min_vel, self.cc_vel, self.traffic_vel, self.dynamic_vel))
                print("min_vel {:.2f} / cc_vel : {:.2f} traffic_vel : {:.2f} dynamic_vel : {:.2f} rotary_vel : {:.2f} stop_vel : {:.2f}".format(min_vel, self.cc_vel, self.traffic_vel,self.dynamic_vel, self.rotary_vel, self.stop_vel))
            
                if self.dynamic_vel == 1.5: # 콘 미션에서 보내는 속도값은 1.5km/h로 해당 값을 보낸다면 콘 미션 중이기에 콘 스티어링으로 변경
                    self.steering = self.cone_str

                self.servo_msg = self.steering*0.016 + self.steering_angle_to_servo_offset
                self.motor_msg = min_vel *self.rpm_gain /3.6
    
                local_path_pub.publish(local_path) ## Local Path 출력
                steer_point_pub.publish(steer_point)
                servo_pub.publish(self.servo_msg)
                motor_pub.publish(self.motor_msg)
            
            if count==300 : ## global path 출력
                global_path_pub.publish(self.global_path)
                count=0
            count+=1

            
            rate.sleep()

    def statusCB(self,data): ## Vehicl Status Subscriber 
        self.status_msg=data
        br = tf.TransformBroadcaster()
        br.sendTransform((self.status_msg.position.x, self.status_msg.position.y, self.status_msg.position.z),
                        tf.transformations.quaternion_from_euler(0, 0, (self.status_msg.heading)/180*pi),
                        rospy.Time.now(),
                        "gps",
                        "map")
        self.is_status=True
        #print('x : {:.2f}, y : {:.2f}'.format(self.status_msg.position.x, self.status_msg.position.y))

    def traffic_set(self, msg):
        self.traffic_vel = msg.data
        #print(self.traffic_vel))

    def dynamic_set(self, msg):
        self.dynamic_vel = msg.data
        # print(self.dynamic_vel)

    def rotary_set(self, msg):
        self.rotary_vel = msg.data
        #print(self.rotary_vel)

    def stop_set(self, msg):
        self.stop_vel = msg.data
        #print(self.stop_vel)

    def selected_set(self, msg):
        self.selected_lane = msg.data

    def str_set(self, msg):
        self.cone_str = msg.data
    
if __name__ == '__main__':
    kcity_pathtracking=wecar_planner()