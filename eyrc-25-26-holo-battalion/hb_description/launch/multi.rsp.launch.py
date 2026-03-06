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
    crystal_xacro_file = os.path.join(pkg_holo_share, 'models', 'holonomic_bot', 'hb_crystal.xacro')
    glacio_xacro_file = os.path.join(pkg_holo_share, 'models', 'holonomic_bot', 'hb_glacio.xacro')
    frostbite_xacro_file = os.path.join(pkg_holo_share, 'models', 'holonomic_bot', 'hb_frostbite.xacro')
    crystal_robot_desc = xacro.process_file(crystal_xacro_file).toxml()
    glacio_robot_desc = xacro.process_file(glacio_xacro_file).toxml()
    frostbite_robot_desc = xacro.process_file(frostbite_xacro_file).toxml()

    # Create three robot_state_publisher nodes for three robots
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
                'robot_description': crystal_robot_desc
            }]
        ),
        
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='glacio_rsp',
            namespace='glacio',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'robot_description': glacio_robot_desc
            }]
        ),
        
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='frostbite_rsp',
            namespace='frostbite',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'robot_description': frostbite_robot_desc
            }]
        ),
    ])