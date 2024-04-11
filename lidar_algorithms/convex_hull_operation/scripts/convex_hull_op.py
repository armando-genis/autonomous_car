#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from std_msgs.msg import String, Float64
from rclpy.timer import Timer
from visualization_msgs.msg import Marker

import matplotlib.pyplot as plt
import numpy as np


class PublisherNode(Node):
    def __init__(self):
        super().__init__('publisher_node')

        # Variables
        self.string_value = "Hello World!"
        self.float_value = 0.0

        # Matplotlib setup for a blank graph
        self.fig, self.ax = plt.subplots()  # Create a figure and an axes
        self.ax.plot([], [], 'r')  # Plot some data on the axes (empty for now)
        self.ax.set_title('Blank Graph')  # Set a title for the axes
        self.ax.set_xlabel('x')  # Add an x-label to the axes
        self.ax.set_ylabel('y')  # Add a y-label to the axes
        plt.show(block=False)  # Show the plot non-blocking

        # Timers, publishers & subscribers
        self.string_publisher = self.create_publisher(String, 'string_topic', 10)
        self.float_publisher = self.create_publisher(Float64, 'float_topic', 10)

        self.hull_subscriber = self.create_subscription(Marker, 'convex_hull_marker', self.hull_marker_callback, 10)

        self.timer = self.create_timer(0.2, self.timer_callback)  # in seconds

        self.get_logger().info("publisher_node initialized")

    def timer_callback(self):
        string_msg = String()
        string_msg.data = self.string_value
        self.string_publisher.publish(string_msg)

        float_msg = Float64()
        float_msg.data = self.float_value
        self.float_publisher.publish(float_msg)

        self.get_logger().info("I am publishing string: '%s', float: '%f'" % (self.string_value, self.float_value))

        self.ax.clear() 
        self.ax.plot([], [], 'r')  
        plt.draw()  
        plt.pause(0.001)  


    def hull_marker_callback(self, msg):
        self.get_logger().info(f'Received Marker: {msg.id}')
        


def main(args=None):
    rclpy.init(args=args)
    node = PublisherNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()