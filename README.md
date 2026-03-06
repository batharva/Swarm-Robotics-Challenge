# 🤖 HB Swarm — ROS2 Multi-Robot Warehouse System

<p align="center">
  <img src="https://img.shields.io/badge/ROS2-Humble-blue?style=for-the-badge&logo=ros" />
  <img src="https://img.shields.io/badge/Gazebo-Simulation-orange?style=for-the-badge" />
  <img src="https://img.shields.io/badge/OpenCV-4.11.0-green?style=for-the-badge&logo=opencv" />
  <img src="https://img.shields.io/badge/Python-3.10-yellow?style=for-the-badge&logo=python" />
  <img src="https://img.shields.io/badge/Ubuntu-22.04-E95420?style=for-the-badge&logo=ubuntu" />
</p>

<p align="center">
  A ROS2-based multi-robot swarm system for autonomous warehouse operations — featuring holonomic robots, ArUco marker pose estimation, and coordinated crate pick-and-place in Gazebo simulation.
</p>

---

## 📖 Overview

Three holonomic robots coordinate autonomously to:
1. **Detect** crates using ArUco marker-based pose estimation
2. **Navigate** to target positions using PID-controlled holonomic motion
3. **Pick and place** crates in designated warehouse locations

The perception pipeline uses OpenCV to process ArUco markers, publishing robot and crate poses over ROS2 topics. A swarm controller node then assigns tasks and coordinates movement across all three robots simultaneously.

---

## 🗂️ Repository Structure

```
hb_ws/src/
├── hb_description/          # Robot URDF/Xacro, launch files
│   └── launch/
│       └── task_2a.launch.py
├── hb_control/              # Perception & control nodes
│   ├── holonomic_perception.py
│   └── task_2a_controller.py
└── ...
```

---

## ⚙️ Requirements

| Dependency | Version |
|---|---|
| Ubuntu | 22.04 |
| ROS2 | Humble |
| Gazebo | (ros-gz-sim) |
| Python | 3.10 |
| OpenCV | 4.11.0.86 |

### Install ROS2 Dependencies

```bash
sudo apt install \
  python3-colcon-common-extensions \
  ros-humble-xacro \
  ros-humble-ros2-control \
  ros-humble-controller-manager \
  ros-humble-ros-gz-sim \
  ros-humble-ros-gz-bridge \
  ros-humble-gz-ros2-control \
  ros-humble-velocity-controllers \
  ros-humble-ros2-controllers
```

### Install Python Dependencies

```bash
pip install opencv-python==4.11.0.86
```

---

## 🚀 Installation

**1. Clone the repository**

```bash
cd ~/Desktop
git clone https://github.com/<your-username>/hb_ws2.git
```

**2. Set up the workspace**

```bash
mkdir -p ~/hb_ws/src
cd ~/hb_ws/src
cp -r ~/Desktop/hb_ws2/* .
```

**3. Build**

```bash
cd ~/hb_ws
colcon build
source install/setup.bash
```

---

## ▶️ Running the System

Each step requires a separate terminal. Source the workspace in each one:

```bash
cd ~/hb_ws && source install/setup.bash
```

### Terminal 1 — Launch Simulation

```bash
ros2 launch hb_description task_2a.launch.py
```

### Terminal 2 — Start Perception Node

```bash
ros2 run hb_control holonomic_perception.py
```

### Terminal 3 — Start 1 Bot Controller

```bash
ros2 run hb_control task_2a_controller.py
```

---

## 📡 ROS2 Topics

| Topic | Type | Description |
|---|---|---|
| `/bot_pose` | `geometry_msgs/Pose` | Estimated pose of each robot |
| `/crate_pose` | `geometry_msgs/Pose` | Detected pose of crates |

---

## ✅ Features

- **ArUco Pose Estimation** — Real-time marker detection using OpenCV
- **Holonomic Motion Control** — Full 3-DOF movement (x, y, θ) for each robot
- **PID Controller** — Smooth, stable navigation to target poses
- **3-Robot Swarm Coordination** — Task assignment and simultaneous execution
- **Crate Pick & Place** — Coordinated manipulation in simulation

---

## ⚠️ Limitations

- Tested in **Gazebo simulation only** — no hardware deployment validated
- Swarm task allocation is centralized (single controller node)

---

## 🛠️ Tech Stack

`ROS2 Humble` · `Gazebo` · `OpenCV` · `Python 3.10` · `ArUco Markers` · `PID Control` · `URDF/Xacro` · `ros2-control`
