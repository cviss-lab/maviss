# Software (BB + Livox)

The **Penelope** software configuration builds directly on the {ref}`Barebone Software <barebone-software>` setup.  
In this variant, the focus is on **integrating and visualizing data from the Livox Mid-360 LiDAR**.

---

## Livox ROS Driver 2

For ROS 2 integration, the <a href="https://github.com/Livox-SDK/livox_ros_driver2" target="_blank" rel="noopener noreferrer">Livox ROS Driver 2</a> is used.  
This driver allows the Penelope platform to publish **point cloud data** directly into ROS 2 topics, which can be visualized and processed in **RViz2**.

### Installation & Launch

Clone, build, and launch the driver in your ROS 2 workspace:

```bash
cd ~/ros2_ws/src
git clone https://github.com/Livox-SDK/livox_ros_driver2.git
cd ~/ros2_ws
colcon build --packages-select livox_ros_driver2
source install/setup.bash

# Launch the LiDAR driver with RViz2
ros2 launch livox_ros_driver2 rviz_MID360_launch.py
