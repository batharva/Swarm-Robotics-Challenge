#!/usr/bin/env python3
import os
import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    pkg_name = 'hb_description'
    pkg_holo_share = get_package_share_directory(pkg_name)

    xacro_file = os.path.join(pkg_holo_share, 'models', 'holonomic_bot', 'hb_crystal.xacro')
    robot_desc = xacro.process_file(xacro_file).toxml()

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time' ,
            default_value='true',
            description='Use simulation (Gazebo) clock if true'),
        
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='crystal_rsp',
            namespace='crystal',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'robot_description': robot_desc
            }]
        ),
    ])
