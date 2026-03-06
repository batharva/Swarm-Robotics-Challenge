import os
import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_aruco_marker():
    return

def generate_launch_description():
    ld = LaunchDescription()
    use_sim_time_arg = DeclareLaunchArgument('use_sim_time', default_value='True', description='Use simulation (Gazebo) clock')
    ld.add_action(use_sim_time_arg)
    spawn_crystal = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-name', 'hb_crystal', '-topic', '/crystal/robot_description', '-x', '1.0', '-y', '0.0', '-z', '0.03','-R', "0.0",
        '-P', "0.0",'-Y', "3.14"],
        output='screen',
    )
    ld.add_action(spawn_crystal)
    return ld
