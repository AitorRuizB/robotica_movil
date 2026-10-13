# ROS 2 Simple Mapping: Point Clouds & Landmarks 🗺️🤖

<p align="center">
  <!-- Reemplaza este enlace con un GIF extraído de tu Practica1_video.mp4 -->
  <img src="URL_AL_GIF_DE_TU_VIDEO" alt="Mapping Simulation" width="600">
</p>

## 📌 Overview
This repository contains ROS 2 packages implementing simplified 2D mapping algorithms for mobile robots using the `mvsim` simulator[cite: 1]. The primary focus is handling coordinate frame transformations (`tf2`) to project local sensor data into a fixed global odometry frame (`odom`) without real-time odometric error correction[cite: 1, 2, 3].

## ⚙️ Core Features

* **Global Point Cloud Generation:** Fuses 2D laser scans (`scan_raw` frame) into a persistent global map[cite: 1, 3]. 
* **Spatial Voxelization:** Implements voxel-based downsampling on the point cloud to optimize memory and reduce computational load as the map expands[cite: 5].
* **Landmark Extraction:** Processes environmental data to detect distinctive features (e.g., cones and bins) based on spatial distance[cite: 5, 6].
* **Marker Visualization:** Publishes detected landmarks as `visualization_msgs/MarkerArray` for rendering in RViz2, utilizing different marker types (Cube, Sphere, Cylinder) for distinct objects[cite: 5, 6].
* **Autonomous Exploration:** Replaces manual teleoperation with a reactive obstacle-avoidance algorithm, analyzing average laser readings on both sides of the robot to determine safe turning directions[cite: 6].

## 🛠️ Tech Stack
* **Framework:** ROS 2 (Python `rclpy`)
* **Simulator:** `mvsim`
* **Key Libraries:** `tf2_ros`, `sensor_msgs` (`PointCloud2`), `laser_geometry`[cite: 4, 5].

## 📂 Repository Structure
* `src/`: Core Python nodes for landmark detection, point cloud mapping, and autonomous navigation.
* `worlds/`: Custom XML environment definitions (`prac1.world.xml`)[cite: 1].
* `data/`: CSV pose and wheel telemetry logs extracted from the simulation sessions.
* `docs/`: Technical reports and demonstration videos.

## 💻 How to Run

1. Launch the `mvsim` environment with the custom world:
   ```bash
   ros2 launch mvsim launch_world.launch.py world_file:=worlds/prac1.world.xml
