Perception Stack
================

The **Perception Stack** provides real-time environment awareness for MAVISS, 
enabling mapping, localization, and autonomous navigation. It integrates sensor 
drivers with ROS 2 modules for LiDAR and visual-inertial odometry.

At a high level, this includes:

- **LiDAR-based SLAM** – using RoboSense AC1 with RSFastLivo for 
  generating dense point clouds and performing real-time 3D mapping.  
- **Visual-Inertial Odometry (VIO)** – combining onboard cameras and IMU data 
  to achieve accurate state estimation, especially in GPS-denied environments.  

These components are designed to work alongside the companion computer and 
Pixhawk, forming the basis for advanced autonomy.

----

.. toctree::
   :maxdepth: 1
   :hidden:

   RSFastLivo
   VIO
   FastLivo2

