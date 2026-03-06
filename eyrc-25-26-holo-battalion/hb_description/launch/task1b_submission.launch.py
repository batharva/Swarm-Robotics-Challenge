#!/usr/bin/env python3
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import AppendEnvironmentVariable
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.actions import ExecuteProcess

def generate_launch_description():
    ros_gz_sim = get_package_share_directory('ros_gz_sim')
    launch_file_dir = os.path.join(get_package_share_directory('hb_description'), 'launch')

    world = os.path.join(
        get_package_share_directory('hb_description'),
        'worlds',
        'task_1b.world'
    )

    gui_config = os.path.join(
        get_package_share_directory('hb_description'),
        'config',
        'gui.config'
    )

    set_env_vars_resources = AppendEnvironmentVariable(
        'GZ_SIM_RESOURCE_PATH',
        os.path.join(get_package_share_directory('hb_description'), 'models'))
    
    gzserver_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': ['-r -s -v4 ', world], 'on_exit_shutdown': 'true'}.items()
    )
    
    gzclient_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={
            'gz_args': f'-g --gui-config {gui_config} -v4'
        }.items()
    )

    gz_sim_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
            "/camera_sensor@sensor_msgs/msg/Image[gz.msgs.Image",
            "/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo"
        ],
        remappings=[
            ("/camera_sensor", "/camera/image_raw"),
            ("/camera_info", "/camera/camera_info"),
        ],
        output="screen",
    )

    box_spawner = Node(
            namespace='eyantrasim', package='hb_description',
            executable='box_placer_1b', output='screen')

    start_autoevaluate = Node(
            namespace='eyantrasim', package='hb_description',
            executable='autoeval_1b', output='screen')


    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    robot_state_publisher_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_file_dir, 'rsp.launch.py')
        ),
        launch_arguments={'use_sim_time': use_sim_time}.items()
    )

    start_gazebo_ros_spawner_cmd = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-name', 'hb_crystal', '-topic', 'robot_description', '-x', '1.0', '-y', '0.0', '-z', '0.1','-R', "0.0",
        '-P', "0.0",'-Y', "-1.5708"],
        output='screen',
    )

    autoevaluate = ExecuteProcess(
    cmd=[
        'ros2', 'bag', 'record', '-o', 'task_1b',
        '/autoeval_data',
        '/crate_pose',
        '/bot_pose'
    ],
    output='screen'
    )

    ld = LaunchDescription()
    ld.add_action(set_env_vars_resources)
    ld.add_action(gzserver_cmd)
    ld.add_action(gzclient_cmd)
    ld.add_action(gz_sim_bridge)
    ld.add_action(robot_state_publisher_cmd)
    ld.add_action(start_gazebo_ros_spawner_cmd)
    ld.add_action(start_autoevaluate)
    ld.add_action(box_spawner)
    ld.add_action(autoevaluate)
    return ld