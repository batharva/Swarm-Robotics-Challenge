#!/usr/bin/env python3
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import AppendEnvironmentVariable
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.actions import TimerAction
import xacro
from launch.actions import ExecuteProcess

def generate_launch_description():
    pkg_project_bringup = get_package_share_directory('hb_description')
    ros_gz_sim = get_package_share_directory('ros_gz_sim')
    launch_file_dir = os.path.join(get_package_share_directory('hb_description'), 'launch')
    pkg_name = 'hb_description'
    pkg_holo_share = get_package_share_directory(pkg_name)

    world = os.path.join(
        get_package_share_directory('hb_description'),
        'worlds',
        'task2.world'
    )
    gui_config = os.path.join(
        get_package_share_directory('hb_description'),
        'config',
        'world.config'
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
        package='ros_gz_bridge',
        executable='parameter_bridge',
        parameters=[{
            'config_file': os.path.join(pkg_project_bringup, 'config', 'ros_gz_bridge.yaml'),
        }],
        output='screen'
    )

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    robot_state_publisher_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_file_dir, 'multi.rsp.launch.py')
        ),
        launch_arguments={'use_sim_time': use_sim_time}.items()
    )

    spawn_holonomic_bot = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_file_dir, 'spawn_multi_hb_bot.launch.py')
        )
    )

    box_spawner = Node(
            namespace='eyantrasim', package='hb_description',
            executable='box_placer_2b', output='screen')

    hb_bridge = Node(
            package='hb_description',
            executable='hb_bridge', output='screen')

    hb_perception_node = Node(
        package='hb_description',
        executable='/usr/bin/python3',
        name='hb_perception',
        output='screen',
        arguments=[os.path.join(get_package_share_directory('hb_description'), 'config', 'holonomic_perception.py')]
    )

    autoevaluate = ExecuteProcess(
    cmd=[
        'ros2', 'bag', 'record', '-o', 'task_2b',
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
    ld.add_action(spawn_holonomic_bot)
    # ld.add_action(hb_perception_node)
    ld.add_action(hb_bridge)
    ld.add_action(box_spawner)
    ld.add_action(autoevaluate)
    return ld