# RSFastLivo

The **RSFastLivo** module integrates the RoboSense AC1 with the 
Fast-LIVO-based SLAM framework. It enables real-time LiDAR odometry and mapping, 
providing accurate localization and dense 3D maps for MAVISS.

---

## Repository

- <a href="https://github.com/RoboSense-Robotics/robosense_ac_slam" target="_blank" rel="noopener noreferrer">RSFastLivo (RoboSense SLAM)</a>

---

## Features

- LiDAR odometry and mapping based on the Fast-LIVO algorithm.  
- Real-time processing for UAV navigation and mapping.  
- Optimized for RoboSense AC-series sensors.  
- ROS 2 integration for visualization in **RViz2**.  

---

## Installation & Usage

Clone, build, and launch the SLAM node in your ROS 2 workspace:

```bash
cd ~/ros2_ws/src
git clone https://github.com/RoboSense-Robotics/robosense_ac_slam.git
cd ~/ros2_ws
colcon build --packages-select robosense_ac_slam
source install/setup.bash

# Launch the SLAM node
ros2 launch robosense_ac_slam demo.launch.py
