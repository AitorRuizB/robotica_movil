# Autonomous Task Execution with FSM & Computer Vision 🤖🚦

<p align="center">
  <!-- Replace this with a GIF of your robot detecting the green object and returning home -->
  <img src="URL_AL_GIF_DE_TU_ROBOT" alt="Robot Task Execution" width="600">
</p>

## 📌 Overview
This repository implements high-level task coordination for an autonomous mobile robot using **Finite State Machines (YASMIN)**[cite: 17]. The robot explores an unknown environment, avoids obstacles reactively[cite: 18], processes visual data to detect a specific green target using OpenCV[cite: 16], and autonomously navigates back to its home origin (`x:-10.0, y:10.0`)[cite: 15].

## ⚙️ Core Capabilities

* **Finite State Machines:** Designed and orchestrated complex behaviors using YASMIN (Yet Another State Machine in ROS), defining custom action states and state transitions[cite: 17, 18].
* **Computer Vision (OpenCV):** Integrated a ROS 2 node to process 2D RGB camera streams (`/camera1_image`), extracting specific color thresholds (green) and counting matching pixels to trigger state changes[cite: 16].
* **Reactive Obstacle Avoidance:** Analyzed 2D LiDAR scans (`/scan`) at specific angles (-30° and 30°) to dynamically steer the robot toward the furthest obstacle-free path[cite: 18].
* **Autonomous Return-to-Home:** Utilized the ROS 2 Navigation Stack (Nav2) `NavigateToPose` action server to autonomously compute and execute a path back to the starting waypoint upon task completion[cite: 18].

## 📂 Repository Structure

* `practica3/`: Core ROS 2 Python package containing the YASMIN state machine nodes, computer vision algorithms (`color_detector.py`), and reactive movement logic[cite: 16, 17, 18].
* `config/` & `maps/`: Tuning parameters (`nav2.yaml`, `amcl.yaml`) and occupancy grids required for the Nav2 return-to-home sequence.
* `worlds/`: Simulated environment files for `mvsim`.
* `srv/`: Custom ROS 2 service definitions (`GoToPose.srv`) used for node communication.

## 💻 How to Run

1. Launch the `mvsim` environment and RViz:
   ```bash
   ros2 launch mvsim launch_world.launch.py world_file:=worlds/prac3.world.xml do_fake_localization:=false
