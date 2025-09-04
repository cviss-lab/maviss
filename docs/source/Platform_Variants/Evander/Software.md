# Software (BB + AC1)

The **Evander** software configuration builds directly on the {ref}`Barebone Software <barebone-software>` setup.

In addition to the core flight software, the following is included for LiDAR
integration:

RoboSense AC1 SDK
-----------------

- Repository: `AC1 ROS2 Driver <https://github.com/RoboSense-Robotics/robosense_ac_ros2_sdk_infra>`  
 
- Features:
  - Publishes raw LiDAR point clouds to ROS 2 topics.  
  - Includes sample launch files for quick bring-up.  
  - Compatible with RViz2 for real-time visualization.  

Installation & Usage
--------------------

1. Clone the SDK into your ROS 2 workspace:

   Terminal -

      cd ~/ros2_ws/src
      git clone https://github.com/RoboSense-Robotics/robosense_ac_ros2_sdk_infra.git
      cd ~/ros2_ws
      colcon build --packages-select robosense_ac_ros2_sdk_infra
      source install/setup.bash

2. Launch the LiDAR driver:

   Terminal -

      ros2 launch robosense_ac_ros2_sdk_infra rviz.launch.py

3. Verify data stream in **RViz2** (point cloud should appear).  


