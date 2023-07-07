#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy

from morai_msgs.msg import GetTrafficLightStatus
from std_msgs.msg import Float64, Int32

class TrafficStop:
    def __init__(self):
        rospy.init_node("traffic_mission", anonymous=False)
        # subscriber
        rospy.Subscriber("/GetTrafficLightStatus", GetTrafficLightStatus, self.callback)
        rospy.Subscriber("/current_waypoint", Int32, self.waypoint_cb)
        
        # publisher
        self.vel_pub = rospy.Publisher("traffic_vel", Float64, queue_size=10)

    def callback(self, msg):
        # 신호등 상태
        traffic_status = msg.trafficLightStatus

        # 신호등 속도 초기화
        traffic_vel = 999

        if 171 <= self.current_waypoint <= 172:
            if traffic_status != 16: # 신호등이 파란불이 아니면
                traffic_vel = 0 # 속도가 0
                

        self.vel_pub.publish(traffic_vel)
        # print("-----------------------------------------------")
        # print("traffic index : {}".format(data.trafficLightIndex))
        # print("traffic Type : {}".format(data.trafficLightType))
        # print("traffic Status : {}".format(data.trafficLightStatus))

    def waypoint_cb(self, msg):
        self.current_waypoint = msg.data
        # print(self.current_waypoint)
        

if __name__ == "__main__":
    ts = TrafficStop()
    rospy.spin()