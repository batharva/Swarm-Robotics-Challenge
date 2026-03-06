Overview

ROS2-based multi-robot swarm warehouse system using holonomic robots, OpenCV ArUco pose estimation, and Gazebo simulation.
Three robots coordinate to detect, navigate, pick, and place crates.

Requirements

Ubuntu 22.04

ROS2 Humble

Gazebo

Python 3.10

Install ROS dependencies:

sudo apt install python3-colcon-common-extensions \
ros-humble-xacro \
ros-humble-ros2-control \
ros-humble-controller-manager \
ros-humble-ros-gz-sim \
ros-humble-ros-gz-bridge \
ros-humble-gz-ros2-control \
ros-humble-velocity-controllers \
ros-humble-ros2-controllers

Install OpenCV:

pip install opencv-python==4.11.0.86
Installation

Clone repository:

cd ~/Desktop
git clone https://github.com/<your-username>/hb_ws2.git

Setup workspace:

mkdir -p ~/hb_ws/src
cd ~/hb_ws/src
cp -r ~/Desktop/hb_ws2/* .

Build workspace:

cd ~/hb_ws
colcon build
source install/setup.bash
Run Simulation (Task 2A – Swarm)
ros2 launch hb_description task_2a.launch.py
Run Perception Node

Open new terminal:

cd ~/hb_ws
source install/setup.bash
ros2 run hb_control holonomic_perception.py
Run Controller (Swarm Control)

Open another terminal:

cd ~/hb_ws
source install/setup.bash
ros2 run hb_control task_2a_controller.py
ROS Topics
/bot_pose
/crate_pose
Implemented

ArUco-based pose estimation

OpenCV vision pipeline

Holonomic robot motion control

PID control

3-robot swarm navigation

Crate pick & place

Limitation

System tested only in simulation (no hardware deployment).
