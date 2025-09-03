# RSFastLivo
==========

The **RSFastLivo** module integrates the RoboSense AC1 with the 
Fast-LIVO-based SLAM framework. It enables real-time LiDAR odometry and mapping, 
providing accurate localization and dense 3D maps for MAVISS.

Repository
----------

- `RSFastLivo (RoboSense SLAM) <https://github.com/RoboSense-Robotics/robosense_ac_slam>`

Features
--------

- LiDAR odometry and mapping based on the Fast-LIVO algorithm.  
- Real-time processing for UAV navigation and mapping.  
- Optimized for RoboSense AC-series sensors.  
- ROS 2 integration for visualization in **RViz2**.  

Installation
------------

Clone the repository into your ROS 2 workspace and build:

 Terminal -

   cd ~/ros2_ws/src
   git clone https://github.com/RoboSense-Robotics/robosense_ac_slam.git
   cd ~/ros2_ws
   colcon build --packages-select robosense_ac_slam
   source install/setup.bash

Usage
-----

To launch the SLAM node:

 Terminal -

   ros2 launch robosense_ac_slam demo.launch.py

Verify that **RViz2** displays both the LiDAR point clouds and the SLAM 
trajectory.

Integration Notes
-----------------

- Requires the **AC1 ROS2 Driver** to publish 
  point clouds.  
- Provides odometry output that can be fused with Pixhawk state estimates 
  (via ROS 2 → MAVROS/PX4 Ros2 bridge).  

