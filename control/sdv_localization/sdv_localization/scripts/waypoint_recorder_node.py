#!/usr/bin/env python3

import os
import csv

from ament_index_python.packages import get_package_share_directory

import rclpy
from rclpy.node import Node

from sdv_msgs.msg import Encoder
from std_msgs.msg import UInt8
from geometry_msgs.msg import Point
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from vectornav_msgs.msg import CommonGroup, InsGroup, ImuGroup

#include "vectornav_msgs/msg/attitude_group.hpp"
#include "vectornav_msgs/msg/common_group.hpp"
#include "vectornav_msgs/msg/gps_group.hpp"
#include "vectornav_msgs/msg/imu_group.hpp"
#include "vectornav_msgs/msg/ins_group.hpp"
#include "vectornav_msgs/msg/time_group.hpp"


class Waypoint_recorder_node(Node):

    def __init__(self):
        super().__init__('waypoint_recorder_node')

        self.waypoint_list_ = []
        self.x = 0.0
        self.y = 0.0
        self.z = 0.0

        self.last = False
        self.pose_sub_ = self.create_subscription( Odometry, '/odom_lidar',
                                                      self.save_pos, 1)
        self.new_rec_sub_ = self.create_subscription( UInt8, '/new_waypoint',
                                                      self.new_rec, 1)

        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

        self.writting_permisions = 0

    def timer_callback(self):
        print(self.x, self.y, self.z)
        if self.writting_permisions == 1:
            self.waypoint_list_.append([self.x,self.y,self.z])

    def save_pos(self, msg):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        self.z = msg.pose.pose.position.z
                
    def new_rec(self, msg):
        self.writting_permisions = msg.data
        if self.writting_permisions == 0:
            print("Not recording!")
        elif self.writting_permisions == 1:
            print("Recording...")
        elif self.writting_permisions == 2:
            print("Writting")
            with open('/home/max/vanttec_sdv/workspace/src/sdv_control/config/new_waypoints.csv', 'w', encoding='UTF8', newline='') as f:
                writer = csv.writer(f)
                writer.writerows(self.waypoint_list_)

def main(args=None):
    rclpy.init(args=args)

    waypoint_rec = Waypoint_recorder_node()

    rclpy.spin(waypoint_rec)

    waypoint_rec.destroy_node()
    waypoint_rec.close()
    rclpy.shutdown()


if __name__ == '__main__':
    main()