import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

	# This yaml is a modified copy of the one found in the sbg driver
	config = os.path.join(
		get_package_share_directory('sdv_localization'),
		'config',
		'sbg',
		'ellipse_D_uart_car.yaml'
	)

	return LaunchDescription([
		Node(
			package='sbg_driver',
			executable = 'sbg_device',
			output = 'screen',
			parameters = [config],
			remappings=[
				('/imu/data', '/sbg/imu/data'),
				('/imu/temp', '/sbg/temp'),
				('/imu/velocity', '/sbg/velocity'),
				# ('/imu/mag', '/sbg/mag'),
				('/imu/pres', '/sbg/pres'),
				('/imu/pos_ecef', '/sbg/pos_ecef'),
				('/imu/utc_ref', '/sbg/utc_ref'),
				('/imu/nav_sat_fix', '/sbg/nav_sat_fix'),
				('/imu/odometry', '/sbg/odom'),
			]
		)
	])