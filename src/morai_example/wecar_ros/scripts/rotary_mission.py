#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import sys
import rospy
import math
import numpy as np
from morai_msgs.msg import GetTrafficLightStatus, EgoVehicleStatus
from std_msgs.msg import Float64, Int32
from sensor_msgs.msg import PointCloud
from nav_msgs.msg import Path
from lib.utils import pathReader


# rotary_waypoints = ([14.194157600402832, 0.290998250246048]
# ,[14.029155731201172, 0.5643513202667236]
# ,[13.850486755371094, 0.8175596594810486]
# ,[13.641944885253906, 1.0377745628356934]
# ,[13.378286361694336, 1.2046408653259277]
# ,[13.098857879638672, 1.331963300704956]
# ,[12.793862342834473, 1.3633406162261963]
# ,[12.487863540649414, 1.3097057342529297])

# rotary_waypoints = ([13.641944885253906, 1.0377745628356934]
# ,[13.378286361694336, 1.2046408653259277]
# ,[13.098857879638672, 1.331963300704956])

rotary_waypoints = ([12.793862342834473, 1.3633406162261963]
,[12.487863540649414, 1.3097057342529297])




class RotaryVel:
    def __init__(self):
        rospy.init_node("rotary_mission", anonymous=False)

        # subscriber
        rospy.Subscriber("/laser2pcd_map", PointCloud, self.callback)
        rospy.Subscriber("/local_path", Path, self.local_cb)
        rospy.Subscriber("/obstacle_path", Path, self.path_cb) #진입 장애물 경로 좌표
        rospy.Subscriber("/Ego_topic", EgoVehicleStatus, self.ego_cb)
        rospy.Subscriber("/current_waypoint", Int32, self.waypoint_cb)

        # publisher
        self.vel_pub = rospy.Publisher("/rotary_vel", Float64, queue_size=10)
        self.stopvel_pub = rospy.Publisher("/stop_vel", Float64, queue_size=10)

        #초기값 설정
        self.rotary_vel = 999
        self.local_path_list = []
        self.obstacle_path_list = []
        self.stop_vel = 999
    def callback(self, msg):
        checking = True
        set_speed = Float64()
        # self.stop_vel = 10

        self.pcd_list = []

        for i in msg.points:
            pcd = []
            pcd.append(i.x)
            pcd.append(i.y)

            self.pcd_list.append(pcd)
            
        '''알고리즘 부분'''
        
        # rotary path에 장애물이 존재하는지 검사
        # rotary_obstacle_list = []
        for i in (self.pcd_list):
            for j in rotary_waypoints:
                dx = ((i[0])-j[0])
                dy = ((i[1])-j[1])
                dst = math.sqrt((dx*dx)+(dy*dy))
                if dst <1:
                    # print(dst)
                
                    # 장애물 발견
                    # rotary_obstacle_list.append(i)
                    checking = False
                    # print("서라")
                else:
                    checking == True  
                # print("rotary Entering : {}".format(checking))
                
                if 213 <= self.current_waypoint <= 214:
                    if checking == False:
                        self.stop_vel = 0
                        
                    elif checking == True:
                        self.stop_vel = 999
                else:
                    self.stop_vel = 999

        self.stopvel_pub.publish(self.stop_vel)
        
            
        #     if checking == False:
        #         set_speed.data = 0
        #         break
        # if checking == True:
        #     set_speed.data = 5
            
            # for i in rotary_obstacle_list:
            #     dx = (i[0]-self.ego_x)
            #     dy = (i[1]-self.ego_y)
            #     dst = math.sqrt((dx*dx)+(dy*dy))
            #     # print(dst)
            #     if dst < 3.5:
            #             print("서라")
            #             # checking == False
            #     else :
            #         checking == True      
                # print("rotary Entering : {}".format(checking))

                        # if checking == False:
                        # set_speed.data = 0
                        #     # break
                        # if checking == True:
                        #     set_speed.data = 5
            
        #Local path 에 장애물이 존재하는지 검사
        obstacle_list = []
        for i in self.pcd_list:
            for j in self.local_path_list:
                dx = (i[0]-j[0])
                dy = (i[1]-j[1])
                dst = math.sqrt((dx*dx)+(dy*dy))
                if dst < 0.4:
                    # 장애물 발견
                    obstacle_list.append(i)
                    # print(i)
                else:
                    self.rotary_vel = 999

        for i in obstacle_list:
            dx = (i[0]-self.ego_x)
            dy = (i[1]-self.ego_y)
            dst = math.sqrt((dx*dx)+(dy*dy))
            # print(dst)
            if 215 <= self.current_waypoint <= 236:
                if dst < 0.5:
                    #print("서라")
                    self.rotary_vel = 0
            else :
                self.rotary_vel = 999

            
        
        self.vel_pub.publish(self.rotary_vel)


    def local_cb(self, msg):
        # Local path 를 받아와서 processing
        self.local_path_list = []
        for i in msg.poses:
            path = []
            path.append(i.pose.position.x)
            path.append(i.pose.position.y)
            # path.append(i.pose.position.z)

            self.local_path_list.append(path)
        # print(self.local_path_list)

    def path_cb(self, msg):
        # 장애물 path 를 받아와서 processing
        self.obstacle_path_list = []
        for i in msg.poses:
            path = []
            path.append(i.pose.position.x)
            path.append(i.pose.position.y)
            # path.append(i.pose.position.z)

            self.obstacle_path_list.append(path)
        print(self.obstacle_path_list)

    def ego_cb(self, msg):
        self.ego_x = msg.position.x
        self.ego_y = msg.position.y
    
    def waypoint_cb(self, msg):
        self.current_waypoint = msg.data


if __name__ == "__main__":
    ts = RotaryVel()
    rospy.spin()