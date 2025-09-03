VIO
===

This guide describes **indoor flight without GPS** using a **VIO/SLAM odometry
source** (from RSFastLivo) and two integration paths to PX4:

- **Path A — MAVROS (serial TELEM2)**: simplest and fully offline.
- **Path B — uXRCE-DDS agent/bridge**: native PX4–ROS 2 DDS pipeline.

The pipeline at a glance
------------------------

1. Create the **ROS 2 workspace** on the Jetson.
2. Start the **AC1 ROS2 Driver** (LiDAR).
3. Start **RSFastLivo** (produces odometry).
4. Feed odometry to PX4 via **MAVROS** *or* **uXRCE-DDS**.
5. Fly indoors in **Position/Offboard** without GPS.

Prerequisites
-------------

- Pixhawk flashed with PX4.
- Jetson flashed and ROS 2 installed.
- TELEM2 UART wired between **Pixhawk ↔ Jetson** (no network required).
- AC1 powered via UBEC and USB to Jetson.

Workspace & drivers
-------------------

 Terminal -

   **Create a ROS 2 workspace on Jetson**
   mkdir -p ~/ros2_ws/src && cd ~/ros2_ws

   **AC1 driver (publishes point clouds)**
   cd ~/ros2_ws/src
   git clone https://github.com/RoboSense-Robotics/robosense_ac_ros2_sdk_infra.git

   **RSFastLivo (produces odometry)**
   git clone https://github.com/RoboSense-Robotics/robosense_ac_slam.git

   **Build**
   cd ~/ros2_ws
   colcon build --symlink-install
   source install/setup.bash

Run sensors & SLAM
------------------

 Terminal -

 **1) Start AC1 driver**
   ros2 launch robosense_ac_ros2_sdk_infra rviz.launch.py

 **2) Start RSFastLivo (publishes odometry)**
   ros2 launch robosense_ac_slam demo.launch.py

Choose your integration path
----------------------------

**Path A — MAVROS over TELEM2 (recommended, fully offline)**

1) Install MAVROS on Jetson:

 Terminal -

   sudo apt update
   sudo apt install -y ros-humble-mavros ros-humble-mavros-extras
   sudo /opt/ros/humble/lib/mavros/install_geographiclib_datasets.sh

2) Start MAVROS with serial FCU URL (adjust port/baud to your wiring):

 Terminal -

   ros2 launch mavros px4.launch fcu_url:=/dev/ttyTHS1:921600

3) Feed RSFastLivo odometry to MAVROS vision topics:

- **Odometry** input:

  Terminal -

     Publish nav_msgs/Odometry to:
      /mavros/odometry/out

**Path B — uXRCE-DDS (PX4 ↔ ROS 2 DDS bridge)**

1) Start the **Micro XRCE-DDS Agent** on Jetson:

 Terminal -

   **Adjust serial device & baud to match TELEM2**
   MicroXRCEAgent serial --dev /dev/ttyTHS1 -b 921600

2) On Pixhawk (via QGC MAVLink console), start the client:

 Terminal -

   uxrce_dds_client start -t serial -d /dev/ttyS2 -b 921600

3) Publish RSFastLivo odometry as a **px4_msgs/VehicleVisualOdometry** topic
   into `/fmu/in/vehicle_visual_odometry` (use your ROS 2 node or example
   publisher). PX4 will fuse this as external vision.

PX4/QGC parameter checklist (indoor EV)
---------------------------------------

Set these in **QGroundControl → Parameters** (names may vary slightly by PX4 version):

- **Arming without GPS**: ``COM_ARM_WO_GPS = 1``
- **External vision aiding**:
  - ``EKF2_AID_MASK`` → enable **Vision position**, **Vision yaw** (optionally **Vision velocity**)
  - ``EKF2_HGT_MODE = Vision`` (use vision as primary height)
- **MAVLink on TELEM2** (if using MAVROS):
  - ``MAV_1_CONFIG = TELEM2``
  - ``MAV_1_MODE = Normal``
  - ``MAV_1_BAUD = 921600``  (or your chosen baud)
- **Indoor flight**:
  - Disable GPS fusion (leave GPS disconnected or ensure EKF does not require GPS)
  - Set appropriate **failsafe** behavior for GPS-denied operation

Safety & frame conventions
--------------------------

- Bench-test first with **props off**; confirm EKF is **using vision** in QGC.
- Ensure correct **frame alignment**:
  - RSFastLivo output (usually **ENU**) vs PX4 (**NED**). MAVROS handles ENU↔NED,
    but verify orientation and yaw sign.
- Secure all cables; avoid LiDAR motion to reduce distortion.

Arming & indoor flight
----------------------

- With MAVROS: arm/disarm and mode change via QGC or MAVROS:

  Terminal -

     # Example (ensure safety is observed!)
     ros2 service call /mavros/cmd/arming mavros_msgs/srv/CommandBool "{value: true}"
     ros2 topic pub /mavros/set_mode mavros_msgs/msg/State "mode: 'OFFBOARD'"

- With uXRCE-DDS: use QGC or your ROS 2 flight app to send mode/arming commands.
- Start in a **wide, texture-rich** indoor area; verify steady pose before takeoff.

Troubleshooting quick tips
--------------------------

- No odometry in PX4:
  - Check RSFastLivo is publishing, and topic remaps to MAVROS/uXRCE inputs.
  - Verify TELEM2 baud and serial device.
- EKF rejecting vision:
  - Recheck ``EKF2_AID_MASK`` and ``EKF2_HGT_MODE``.
  - Confirm timestamp continuity and reasonable covariance on vision data.
- Jittery hover:
  - Increase LiDAR mounting rigidity; reduce vibrations; tune vision noise params.


