# 🤖 ROS 2 Autonomous Mobile Robotics

<p align="center">
  <img src="https://img.shields.io/badge/ROS_2-Jazzy%20%2F%20Humble-0A0FF9?style=for-the-badge&logo=ros&logoColor=white" alt="ROS 2">
  <img src="https://img.shields.io/badge/Python-3670A0?style=for-the-badge&logo=python&logoColor=ffdd54" alt="Python">
  <img src="https://img.shields.io/badge/Nav2-Navigation-green?style=for-the-badge" alt="Nav2">
  <img src="https://img.shields.io/badge/OpenCV-5C3EE8?style=for-the-badge&logo=opencv&logoColor=white" alt="OpenCV">
</p>

## 📌 About This Repository
This repository contains a comprehensive suite of autonomous mobile robotics projects developed as part of the Robotics Engineering degree at Universidad de Alicante. 

The projects span from low-level coordinate transformations and sensor processing to high-level task orchestration, demonstrating a complete pipeline for autonomous mobile robot navigation and behavior generation using the **ROS 2 ecosystem**.

## 🚀 Projects Overview

The repository is structured into three progressive modules, each contained within its own directory with detailed instructions:

### 1. [Simple Mapping & Landmarks (`/p1`)](./p1)
* **Focus:** Spatial data processing, `tf2` coordinate transformations, and environmental feature extraction.
* **Key Implementations:** 
  * Projection of 2D LiDAR scans (`/scan_raw`) into global `PointCloud2` maps.
  * Spatial voxelization for memory optimization.
  * Landmark extraction (cones, bins) published as RViz `MarkerArrays`.
  * Reactive obstacle avoidance based on laser segment averaging.

### 2. [SLAM & Autonomous Navigation (`/p2`)](./p2)
* **Focus:** Advanced environment mapping, probabilistic localization, and path planning using standard industry stacks.
* **Key Implementations:**
  * **SLAM Toolbox:** Generation of 2D occupancy grids (`.pgm`/`.yaml`) in online asynchronous mode.
  * **AMCL:** Adaptive Monte Carlo Localization implementation, analyzing particle filter convergence and covariance.
  * **Nav2 Stack:** Full parameterization of global/local costmaps and planner servers for collision-free autonomous waypoint navigation.

### 3. [Task Orchestration & Computer Vision (`/p3`)](./p3)
* **Focus:** High-level decision making, state machines, and visual perception.
* **Key Implementations:**
  * **YASMIN FSM:** Design of finite state machines to coordinate complex, multi-step robotic behaviors.
  * **OpenCV Integration:** Processing 2D RGB camera streams to detect specific color thresholds (green target identification).
  * **Autonomous Logic:** A complete routine where the robot explores reactively, detects a visual target, and automatically triggers a Nav2 `NavigateToPose` action to return to its origin.

## 🛠️ Environment & Tools
* **OS:** Ubuntu 24.04 LTS
* **Middleware:** ROS 2
* **Simulators:** `mvsim`
* **Visualization:** RViz2
* **Libraries:** `tf2_ros`, `sensor_msgs`, `nav2`, `slam_toolbox`, `yasmin`, `cv2`

---
*Developed by Aitor Ruiz Bautista as part of the Mobile Robotics coursework.*
