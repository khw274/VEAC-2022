#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from std_msgs.msg import Float64
from sensor_msgs.msg import LaserScan,PointCloud
from morai_msgs.msg import EgoVehicleStatus
from math import cos,sin,pi,floor
from geometry_msgs.msg import Point32

class lidarParser :

    def __init__(self):
        rospy.init_node('lidar_parser', anonymous=True)
        rospy.Subscriber("/lidar2D", LaserScan, self.laser_callback)
        rospy.Subscriber("/Ego_topic", EgoVehicleStatus, self.ego_status_cb)

        self.pcd_pub = rospy.Publisher('/laser2pcd', PointCloud, queue_size=1)
        self.pcd_map_pub = rospy.Publisher('/laser2pcd_map', PointCloud, queue_size=1)
        self.ego_heading_rad=0
        self.is_status = False
        rospy.spin()

    # 차량의 헤딩과 x, y 위치를 받아옴
    def ego_status_cb(self, msg):
        self.ego_x, self.ego_y = msg.position.x, msg.position.y
        n = floor(msg.heading / 180)
        self.ego_heading_rad = -(msg.heading - n * 360)*pi/180

        self.is_status = True

    def laser_callback(self,msg):
        if self.is_status:
            pcd=PointCloud()
            pcd.header.frame_id=msg.header.frame_id
            angle=0

            pcd_map=PointCloud()
            pcd_map.header.frame_id="map"

            for r in msg.ranges :
                
                # Lidar 직각 좌표계 변환 
                tmp_point=Point32()
                tmp_point.x=r*cos(angle)
                tmp_point.y=r*sin(angle)
                # print(angle,tmp_point.x,tmp_point.y)
                angle=angle+(1.0/180*pi)

                # Map 좌표계 변환
                map_point = Point32()
                # 회전 
                rotate_x = r*cos(angle)*cos(self.ego_heading_rad)+r*sin(angle)*sin(self.ego_heading_rad)
                rotate_y = -r*cos(angle)*sin(self.ego_heading_rad)+r*sin(angle)*cos(self.ego_heading_rad)
                # 차량의 x, y 위치만큼 더해줌
                map_point.x = rotate_x + self.ego_x 
                map_point.y = rotate_y + self.ego_y

                if 0.2<r<9.9 and tmp_point.x > 0:
                    pcd.points.append(tmp_point)
                    pcd_map.points.append(map_point)
                    
            self.pcd_pub.publish(pcd)
            self.pcd_map_pub.publish(pcd_map)

if __name__ == '__main__':
    try:
        test=lidarParser()
    except rospy.ROSInterruptException:
        pass