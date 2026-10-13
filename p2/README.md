# ROS 2 Nav2: Mapping, Localization & Autonomous Navigation 🗺️📍

<p align="center">
  <!-- Reemplaza este enlace con un GIF extraído de Navegacion.mp4 -->
  <img src="URL_AL_GIF_DE_NAVEGACION" alt="Nav2 Autonomous Navigation" width="600">
</p>

## 📌 Overview
This project demonstrates the integration and configuration of the **ROS 2 Navigation Stack (Nav2)** and **SLAM Toolbox** for autonomous mobile robots[cite: 7]. The implementation covers the full pipeline from environment mapping to probabilistic localization and autonomous waypoint navigation in simulated environments (`mvsim`)[cite: 7].

## ⚙️ Core Implementations

* **SLAM & Mapping:** Configured `slam_toolbox` in online asynchronous mode to generate 2D occupancy grid maps (`.pgm`/`.yaml`) from 2D LiDAR data (`/scan_raw`)[cite: 8, 9].
* **AMCL Localization:** Deployed Adaptive Monte Carlo Localization (AMCL) to track the robot's pose against the static map, analyzing the particle filter behavior and covariance convergence during movement[cite: 10, 11].
* **Autonomous Navigation:** Parameterized the Nav2 stack (local/global costmaps, planner server, controller) to enable collision-free path planning and robust obstacle avoidance[cite: 11, 12].

## 📂 Repository Structure

* `config/`: Tuning and parameterization files (`slam.yaml`, `amcl.yaml`, `nav2.yaml`) defining the behavior of the mapping and navigation servers[cite: 8, 10, 11].
* `maps/`: Exported occupancy grids generated via `nav2_map_server`[cite: 9].
* `docs/`: Video demonstrations of the particle cloud convergence and autonomous goal reaching, alongside technical parameter analysis[cite: 11, 12].

## 💻 How to Run

1. Launch the simulation environment:
   ```bash
   ros2 launch mvsim demo_turtlebot_world.launch.py do_fake_localization:=false use_rviz:=false
