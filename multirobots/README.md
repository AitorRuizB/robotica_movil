# ROS 2 Scalable Multi-Robot System Simulation 🤖🚀

<p align="center">
  <img src="https://img.shields.io/badge/ROS_2-Jazzy-0A0FF9?style=for-the-badge&logo=ros&logoColor=white" alt="ROS 2 Jazzy">
  <img src="https://img.shields.io/badge/Gazebo-Harmonic-FF7F00?style=for-the-badge" alt="Gazebo Harmonic">
  <img src="https://img.shields.io/badge/Python-3670A0?style=for-the-badge&logo=python&logoColor=ffdd54" alt="Python">
</p>

## 📌 Overview
This repository contains the full development of a scalable multi-robot simulation system from scratch using **ROS 2 (Jazzy)** and **Gazebo Harmonic**. The architecture allows for the dynamic instantiation of multiple differential-drive robots equipped with LiDAR and RGB cameras, managing complex TF trees, namespaces, and dynamic ROS-Gazebo bridges.

## ⚙️ System Architecture & Packages

The workspace is modularized into three distinct ROS 2 packages:

1. **`practica1` (Robot Description):** Contains the core URDF/Xacro descriptions. The robot is modeled with a differential drive base, caster wheel for stability, and macro-based modular sensors (`lidar.xacro`, `camara.xacro`). Physics parameters (`inercias.xacro`) and visual properties (`materials.xacro`) are strictly separated.
2. **`robot_bringup` (Single Agent Instance):** Manages the simulation of a single robot. Includes the static `bridge.yaml` for ROS-Gazebo topic routing, the base `world.sdf`, and RViz2 configuration profiles (`config.rviz`).
3. **`multirobot_bringup` (Multi-Agent Orchestrator):** The core orchestrator for multi-robot deployment. It utilizes Xacro arguments to dynamically inject `prefix` tags into all links, joints, and Gazebo references. It features a Python launch file (`multirobot.launch.py`) that generates dynamic ROS-Gazebo bridges "on the fly" and spaces out the spawning poses (`--y`) of an arbitrary number of robots.

## 📂 Repository Structure

    ├── src/
    │   ├── practica1/            # Core URDF/Xacro models, sensors, and inertia macros
    │   ├── robot_bringup/        # Single-robot launch, static bridge, and RViz config
    │   └── multirobot_bringup/   # Multi-agent dynamic launcher, namespaces, and worlds
    └── docs/                     # Technical documentation and architecture report

## 💻 How to Build and Run

### Prerequisites
* **OS:** Ubuntu 24.04 LTS (Noble)
* **Middleware:** ROS 2 Jazzy
* **Simulator:** Gazebo Harmonic

### 1. Build the Workspace
Clone the repository into your ROS 2 workspace `src` folder, navigate to the root of your workspace, and compile the packages:

    source /opt/ros/jazzy/setup.bash
    colcon build
    source install/setup.bash

### 2. 🏃 Single Robot Simulation
To launch the Gazebo environment and RViz2 with a single robot instance:

    ros2 launch robot_bringup robot_bringup.launch.py

### 3. 🏃‍♂️🤖 Multi-Robot Simulation
To spawn multiple robots, dynamically generating their isolated TF trees and ROS-Gazebo bridges, run the following command. You can specify the number of robots using the `num_robots` argument (it defaults to 2 if omitted):

    ros2 launch multirobot_bringup multirobot.launch.py num_robots:=4

*Note: Each robot's topics (e.g., `/robot1/cmd_vel`, `/robot2/scan`, `/robot1/camera/image_raw`) and TF frames (`robot1/odom` -> `robot1/base_footprint` -> `robot1/base_link`) are properly isolated using namespaces to prevent cross-interference.*
