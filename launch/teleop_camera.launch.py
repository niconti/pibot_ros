import os

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import ThisLaunchFileDir,LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    
    teleop_camera = Node(package='pibot_ros', node_executable='teleop_camera',
                         output='screen')              
    
    return LaunchDescription([
        teleop_camera
    ])