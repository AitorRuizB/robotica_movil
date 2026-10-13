# ROS 2 Scalable Multi-Robot System Simulation 🤖🚀

<p align="center">
  <img src="https://img.shields.io/badge/ROS_2-Jazzy-0A0FF9?style=for-the-badge&logo=ros&logoColor=white" alt="ROS 2 Jazzy">
  <img src="https://img.shields.io/badge/Gazebo-Harmonic-FF7F00?style=for-the-badge" alt="Gazebo Harmonic">
  <img src="https://img.shields.io/badge/Python-3670A0?style=for-the-badge&logo=python&logoColor=ffdd54" alt="Python">
</p>

## 📌 Overview
This repository contains the full development of a scalable multi-robot simulation system from scratch using **ROS 2 (Jazzy)** and **Gazebo Harmonic**[cite: 21]. The architecture allows for the dynamic instantiation of multiple differential-drive robots equipped with LiDAR and RGB cameras, managing complex TF trees, namespaces, and dynamic ROS-Gazebo bridges[cite: 27, 29].

## ⚙️ System Architecture & Packages

The workspace is modularized into three distinct ROS 2 packages:

1. **`practica1` (Robot Description):** Contains the core URDF/Xacro descriptions. The robot is modeled with a differential drive base, caster wheel for stability, and macro-based modular sensors (`lidar.xacro`, `camara.xacro`)[cite: 24, 27]. Physics parameters (`inercias.xacro`) and visual properties are strictly separated[cite: 25].
2. **`robot_bringup` (Single Agent Instance):** Manages the simulation of a single robot. Includes the static `bridge.yaml` for ROS-Gazebo topic routing, the base `world.sdf`, and RViz2 configuration profiles[cite: 25].
3. **`multirobot_bringup` (Multi-Agent Orchestrator):** The core orchestrator for multi-robot deployment. It utilizes Xacro arguments to dynamically inject `prefix` tags into all links, joints, and Gazebo references[cite: 28, 29]. It features a Python launch file that generates dynamic ROS-Gazebo bridges "on the fly" and spaces out the spawning poses (`--y`) of an arbitrary number of robots.

## 💻 How to Build and Run

### Prerequisites
* Ubuntu 24.04 LTS
* ROS 2 Jazzy
* Gazebo Harmonic

### Build the Workspace
Clone the repository into your ROS 2 workspace `src` folder and build:
```bash
colcon build
source install/setup.bash
