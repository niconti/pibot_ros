import os
# ROS2
from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import ThisLaunchFileDir, LaunchConfiguration, TextSubstitution
from launch_ros.actions import Node


def generate_launch_description():

    joy_config_arg = DeclareLaunchArgument('joy_config', default_value='logitech')

    joy_config_file_arg = DeclareLaunchArgument('joy_config_file', default_value=[
        TextSubstitution(text=os.path.join(get_package_share_directory('zumo_ros'), 'config', '')), LaunchConfiguration('joy_config'), TextSubstitution(text='.config.yaml')
    ])

    motors_config_file_arg = DeclareLaunchArgument('motors_config_file', default_value=[
        TextSubstitution(text=os.path.join(get_package_share_directory('zumo_ros'), 'config', '')), 'zumo', TextSubstitution(text='.config.yaml')
    ])

    motors_controller = Node(package='zumo_ros', executable='motors_nvidia',
                             parameters=[LaunchConfiguration('motors_config_file')],
                             output='screen', emulate_tty=True)
    
    teleop_camera = Node(package='zumo_ros', executable='teleop_camera',
                         output='screen')              
    
    teleop_robot = Node(package='teleop_twist_joy', executable='teleop_node',
                        name='teleop_robot',
                        parameters=[LaunchConfiguration('joy_config_file')],
                        remappings=[
                            ("/cmd_vel", "/zumo/cmd_vel"),
                        ],
                        output='screen', emulate_tty=True)
    
    return LaunchDescription([
        joy_config_arg,
        joy_config_file_arg,
        motors_config_file_arg,
        motors_controller,
        teleop_camera,
        teleop_robot
    ])
