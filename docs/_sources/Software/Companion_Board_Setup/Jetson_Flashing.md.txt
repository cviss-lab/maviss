Jetson Flashing
===============

This section explains how to set up the **NVIDIA Jetson companion computer** 
for MAVISS. The Jetson handles high-level tasks such as perception, mapping, 
and autonomy, while maintaining communication with the Pixhawk.

Reference Guide
---------------

For hardware wiring and additional details, see the official PX4 guide:  
`Pixhawk + Jetson Baseboard Setup <https://docs.px4.io/main/en/companion_computer/holybro_pixhawk_jetson_baseboard.html>`

Flashing the Jetson
-------------------

1. **Download JetPack** (Ubuntu-based image for Jetson) from the NVIDIA SDK Manager.  
2. Connect the Jetson to your host PC via USB and put it in **recovery mode**.  
3. Use the **SDK Manager** to flash the OS onto the Jetson.  
4. Once complete, boot the Jetson and create a default user.  

Initial Configuration
---------------------

After flashing, perform the following setup:

.. code-block:: bash

   **Update system**
   sudo apt update && sudo apt upgrade -y

   **Install ROS 2 Humble**
   sudo apt install ros-humble-desktop -y

   **Source ROS 2 in bashrc**
   echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
   source ~/.bashrc

MAVLink Communication
---------------------

- The Jetson connects to the Pixhawk via **UART/USB** using the Holybro baseboard.  
- Configure PX4 to enable a MAVLink instance on the appropriate serial port.  
- On the Jetson, verify the link using `mavlink-router` or direct ROS 2 PX4 bridge.  

Integration Notes
-----------------

- Ensure the Jetson is on a **stable power supply** (e.g., UBEC).  
- Configure a **static IP** if using Ethernet for remote visualization or control.  
- This setup provides the foundation for running perception stacks such as 
  LiDAR drivers and VIO.  

