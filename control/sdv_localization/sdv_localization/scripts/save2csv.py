#!/usr/bin/env python3

import os
import csv

import rclpy
from rclpy.node import Node

from sdv_msgs.msg import Encoder
from std_msgs.msg import UInt8
from vectornav_msgs.msg import CommonGroup, InsGroup, ImuGroup

#include "vectornav_msgs/msg/attitude_group.hpp"
#include "vectornav_msgs/msg/common_group.hpp"
#include "vectornav_msgs/msg/gps_group.hpp"
#include "vectornav_msgs/msg/imu_group.hpp"
#include "vectornav_msgs/msg/ins_group.hpp"
#include "vectornav_msgs/msg/time_group.hpp"


class IMU2CSV(Node):

    def __init__(self):
        super().__init__('to_csv_node')

        self.common_sub_ = self.create_subscription( CommonGroup, 'vectornav/raw/common',
                                                      self.save_common, 1)
        self.ins_sub_ = self.create_subscription( InsGroup, 'vectornav/raw/ins',
                                                      self.save_ins, 1)
        self.wheel_encoder_sub_ = self.create_subscription( Encoder, 'ifm_encoder',
                                                      self.save_wheel_encoder, 1)
        self.pot_step_sub_ = self.create_subscription( UInt8, 'potentiometer_step',
                                                      self.pot_step, 1)
        # self.imu_sub_ = self.create_subscription( String, 'vectornav/raw/imu',
        #                                               self.save_imu, 1)
        # self.gps2_sub_ = self.create_subscription( String, 'vectornav/raw/gps2',
        #                                               self.save_gps, 1)
        # self.attitude_sub_ = self.create_subscription( String, 'vectornav/raw/attitude',
        #                                               self.save_attitude, 1)

        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.pot_callback)
        self.pot_val = 0

        self.is_msg_arrived_ = False
        self.pot_step_val = 0

        rosbag_path = '/home/ws/src/tests/'
        test = 'pruebas1ago/throttle/step95/data'

        self.accel_file_path_ = rosbag_path + test + '/accel_data.csv'
        self.accel_file_ = open(self.accel_file_path_, 'w')
        self.accel_file_writer_ = csv.writer(self.accel_file_)
        self.accel_file_writer_.writerow(['Time', 'AccelBody(x)','AccelBody(y)','AccelBody(z)', 'Theta', 'Psi', 'Psi_dot'])

        self.vel_file_path_ = rosbag_path + test + '/vel_data.csv'
        self.vel_file_ = open(self.vel_file_path_, 'w')
        self.vel_file_writer_ = csv.writer(self.vel_file_)
        self.vel_file_writer_.writerow(['Time', 'VelBody(x)','VelBody(y)','VelBody(z)'])

        self.encoder_file_path_ = rosbag_path + test + '/encoder_data.csv'
        self.encoder_file_ = open(self.encoder_file_path_, 'w')
        self.encoder_file_writer_ = csv.writer(self.encoder_file_)
        self.encoder_file_writer_.writerow(['Time', 'WheelAngle', 'SteeringAngle'])

        self.pot_file_path_ = rosbag_path + test + '/pot_data.csv'
        self.pot_file_ = open(self.pot_file_path_, 'w')
        self.pot_file_writer_ = csv.writer(self.pot_file_)
        self.pot_file_writer_.writerow(['Time', 'Pot_Step'])

    def pot_callback(self):
        if self.is_msg_arrived_:
            elapsed_time = self.get_clock().now() - self.start_time_
            self.pot_file_writer_.writerow([elapsed_time.nanoseconds / 1e9, self.pot_val])

    def pot_step(self, msg):
        if not self.is_msg_arrived_:
            self.is_msg_arrived_ = True
            self.start_time_ = self.get_clock().now()
            
        if self.pot_val != msg.data:
            self.pot_val = msg.data

    def save_common(self, msg):
        # if not self.is_msg_arrived_:
        #     self.is_msg_arrived_ = True
        #     self.start_time_ = self.get_clock().now()
        
        if self.is_msg_arrived_:
            elapsed_time = self.get_clock().now() - self.start_time_
            self.accel_file_writer_.writerow([elapsed_time.nanoseconds / 1e9, msg.accel.x, msg.accel.y, msg.accel.z, msg.yawpitchroll.y, msg.yawpitchroll.x, msg.angularrate.z])

    def save_ins(self, msg):
        # if not self.is_msg_arrived_:
        #     self.is_msg_arrived_ = True
        #     self.start_time_ = self.get_clock().now()
        
        if self.is_msg_arrived_:
            elapsed_time = self.get_clock().now() - self.start_time_
            self.vel_file_writer_.writerow([elapsed_time.nanoseconds / 1e9, msg.velbody.x, msg.velbody.y, msg.velbody.z])

    def save_wheel_encoder(self, msg):
        # if not self.is_msg_arrived_:
        #     self.is_msg_arrived_ = True
        #     self.start_time_ = self.get_clock().now()
        
        if self.is_msg_arrived_:
            elapsed_time = self.get_clock().now() - self.start_time_
            MAX_WHEEL_ANGLE = 715.78
            MIN_WHEEL_ANGLE = -577

            MAX_DELTA_R = 44
            MIN_DELTA_R = -44 #TO CHECK

            MAX_DELTA_L = 38 #TO CHECK
            MIN_DELTA_L = -38
            
            delta_r = msg.abs_angle * MAX_DELTA_R/MAX_WHEEL_ANGLE if msg.abs_angle >= 0 else msg.abs_angle * MIN_DELTA_R/MIN_WHEEL_ANGLE
            delta_l = msg.abs_angle * MAX_DELTA_L/MAX_WHEEL_ANGLE if msg.abs_angle >= 0 else msg.abs_angle * MIN_DELTA_L/MIN_WHEEL_ANGLE
            delta = (delta_l + delta_r) / 2
            self.encoder_file_writer_.writerow([elapsed_time.nanoseconds / 1e9, msg.abs_angle, delta])

    def close(self):
        self.accel_file_.close()
        self.vel_file_.close()
        self.encoder_file_.close()

def main(args=None):
    rclpy.init(args=args)

    to_csv = IMU2CSV()

    rclpy.spin(to_csv)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    to_csv.destroy_node()
    to_csv.close()
    rclpy.shutdown()


if __name__ == '__main__':
    main()