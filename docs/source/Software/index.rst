Software
========

The MAVISS software stack brings together **flight control firmware**, 
**companion computer setup**, and **perception modules** to enable both 
manual and autonomous operation.  

At a high level, the software covers:

- **Flight Control Unit (FCU) Setup**  
  Configuration of the Pixhawk autopilot with PX4 firmware, including sensor 
  calibration, parameter tuning, and integration with QGroundControl.  

- **Companion Board Setup**  
  Preparing the NVIDIA Jetson for ROS 2-based processing, middleware setup, 
  and network configuration to communicate with the FCU and external devices.  

- **Perception Stack**  
  Integration of additional sensors (LiDAR, cameras, VIO) with ROS 2 drivers 
  and visualization tools, forming the foundation for mapping, SLAM, and 
  higher-level autonomy.  

----

.. toctree::
   :maxdepth: 2
   :hidden:

   FCU_Setup/index
   Companion_Board_Setup/index
   Perception_Stack/index

