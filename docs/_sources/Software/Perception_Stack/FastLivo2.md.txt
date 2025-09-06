# FastLivo2

The **FAST-LIVO2** module integrates **LiDAR–Inertial–Visual Odometry** for UAV navigation  
and mapping. In the MAVISS stack, FAST-LIVO2 is configured for the **Livox Mid-360**,  
providing robust state estimation and mapping in GPS-denied or cluttered environments.  
This module plays a similar role as **RSFastLivo** (for the RoboSense AC1),  
but optimized for Livox sensors.

---

## Repository

- [FAST-LIVO2](https://github.com/hku-mars/FAST-LIVO2)

---

## Features

- LiDAR–Inertial–Visual fusion for odometry.  
- Compatible with the **Livox Mid-360** sensor.  
- Real-time mapping suitable for UAV navigation and SLAM research.  
- Robust in low-texture and GPS-denied environments.  
- ROS/ROS2 integration for visualization in **RViz**.  

---

## Installation & Usage

Clone, build, and launch FAST-LIVO2 in your ROS 2 workspace:

```bash
# Create a workspace
mkdir -p ~/ros2_ws/src && cd ~/ros2_ws/src

# Clone the FAST-LIVO2 repository
git clone https://github.com/hku-mars/FAST-LIVO2.git

# Build
cd ~/ros2_ws
colcon build --packages-select fast_livo2
source install/setup.bash

# Launch FAST-LIVO2 with example launch file
ros2 launch fast_livo2 run.launch.py
```

## Integration Notes

- Requires Livox Mid-360 + IMU + camera for full VIO pipeline.

- Ensure proper calibration between LiDAR, camera, and IMU frames.

- Provides odometry output that can be fused with PX4 via MAVROS or uXRCE-DDS.

- Recommended for indoor and outdoor UAV missions where GPS may be unreliable.