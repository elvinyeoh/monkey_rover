import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

PACKAGE_NAME = 'monkey_rover'

def generate_launch_description():

    motor_config = os.path.join(
        get_package_share_directory(PACKAGE_NAME), 'config', 'motor_config.yaml'
    )
	
    return LaunchDescription([
            
        Node(
            package = PACKAGE_NAME,
            executable = 'motor_controller',
            name = 'motor_controller',
            parameters = [motor_config])
    ])
