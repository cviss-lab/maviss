Companion Board Setup
=====================

The **companion board** (NVIDIA Jetson) handles high-level processing for
perception, mapping, and autonomy. It runs the ROS 2 middleware, sensor drivers,
and communicates with the Pixhawk via MAVLink.

At a high level, this setup includes:

- **Flashing the Jetson** with a clean Ubuntu 22.04 LTS image.  
- **Configuring peripherals** such as USB devices (LiDAR, cameras) and Ethernet
  networking.  
- **Installing ROS 2** and required dependencies for perception and autonomy
  stacks.  
- **Setting up MAVLink communication** between Jetson and Pixhawk.  

----

.. toctree::
   :maxdepth: 1
   :hidden:

   Jetson_Flashing

