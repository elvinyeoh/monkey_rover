import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

PACKAGE_NAME = 'monkey_rover'

def generate_launch_description():
#Tseting
    return LaunchDescription([
            
        Node(
            package = 'joy',
            executable = 'joy_node',
            name = 'joy_node'),
        Node(
            package = PACKAGE_NAME,
            executable = 'ps4_controller_converter',
            name = 'ps4_controller_converter')
    ])
