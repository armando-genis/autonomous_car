#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from std_msgs.msg import String, Float64
from rclpy.timer import Timer
from visualization_msgs.msg import Marker
from lidar_msgs.msg import ObstacleData 


import matplotlib.pyplot as plt
import numpy as np


class PublisherNode(Node):
    def __init__(self):
        super().__init__('publisher_node')

        # Variables to store the hull vectors
        self.hull_vectors = []

        # Matplotlib setup for a blank graph
        self.fig, self.ax = plt.subplots()  # Create a figure and an axes
        self.ax.plot([], [], 'r')  # Plot some data on the axes (empty for now)
        self.ax.set_title('Blank Graph')  # Set a title for the axes
        self.ax.set_xlabel('x')  # Add an x-label to the axes
        self.ax.set_ylabel('y')  # Add a y-label to the axes
        plt.show(block=False)  # Show the plot non-blocking


        self.obstacle_data_subscriber = self.create_subscription(ObstacleData, 'obstacle_data',  self.obstacle_data_callback,  10)
        
        self.get_logger().info("publisher_node initialized")



    def obstacle_data_callback(self, msg):
        # Clear the existing data
        self.hull_vectors.clear()

        # Clear the plot
        self.ax.clear()
        self.ax.set_title('Hull Clusters')
        self.ax.set_xlabel('x')
        self.ax.set_ylabel('y')
        # Set the axis limits
        self.ax.set_xlim(-7, 7)  # Set x-axis limits from 0 to 15
        self.ax.set_ylim(-3, 17)  # Set y-axis limits from 0 to 20

        # Process each PointArray in the received ObstacleData message
        for point_array in msg.cluster_points:
            x_vals = [point.x for point in point_array.points]
            y_vals = [point.y for point in point_array.points]

            # Plot the points for this cluster
            self.ax.plot(x_vals, y_vals, 'o-', label='Cluster')  # 'o-' for points connected by lines

        # Update plot legends and redraw
        self.ax.legend()
        plt.draw()
        plt.pause(0.001)  # Pause to allow update


    # def obstacle_data_callback(self, msg):
    #     # Clear the previous points to store the latest ones or remove this line to accumulate over time
    #     self.hull_vectors.clear() 

    #     # Process each PointArray in the received ObstacleData message
    #     for point_array in msg.cluster_points:
    #         # Create a list to store points from the current cluster
    #         cluster_points = []
    #         for point in point_array.points:
    #             cluster_points.append((point.x, point.y, point.z))  # Append each point as a tuple

    #         # Store the list of points for this cluster
    #         self.hull_vectors.append(cluster_points)

    #     # Log the total number of points received in this message
    #     self.get_logger().info(f'Received {len(msg.cluster_points)} clusters; storing points.')

    

def main(args=None):
    rclpy.init(args=args)
    node = PublisherNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()