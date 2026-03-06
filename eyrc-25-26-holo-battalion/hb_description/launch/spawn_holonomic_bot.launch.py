import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()
    pkg_name = 'hb_description'
    pkg_holo_share = get_package_share_directory(pkg_name)
    controllers_file = os.path.join(pkg_holo_share, 'config', 'controllers.yaml')
    use_sim_time_arg = DeclareLaunchArgument('use_sim_time', default_value='True', description='Use simulation (Gazebo) clock')
    ld.add_action(use_sim_time_arg)

    # controller_manager (ros2_control) node
    controller_manager_node = Node(
        package='controller_manager', executable='ros2_control_node',
        name='controller_manager', output='screen',
        parameters=[
            controllers_file
        ]
    )

    # spawn effort controllers
    spawn_effort_controller = Node(
        package='controller_manager', executable='spawner', output='screen',
        arguments=['forward_velocity_controller']
    )

    start_gazebo_ros_spawner_cmd = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-name', 'holonomic_bot', '-topic', 'robot_description', '-x', '0.0', '-y', '0.0', '-z', '0.1','-R', "0.0",
        '-P', "0.0",'-Y', "-1.5708"],
        output='screen',
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

    ld.add_action(controller_manager_node)
    ld.add_action(spawn_effort_controller)
    ld.add_action(start_gazebo_ros_spawner_cmd)
    ld.add_action(gz_sim_bridge)  
    return ld
