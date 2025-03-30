<p align="center">
  <img src="misc/cviss_logo.png" alt="Lab Logo" width="200"/>
</p>

# MAVISS
MAVISS is an open-source Micro Aerial Vehicle (MAV) for precision infrastructure inspection, featuring high-resolution cameras, LiDAR, GPS, and NVIDIA Orin board for autonomous operations. This repository, `maviss`, contains the full software stack required to operate the MAVISS drone, including:

- ROS 2 packages
- 3D description packages
- Sensor integration packages
- Flight control configurations

## Repository Structure

```
maviss/
 maviss_description/   # URDF and robot model
 .
 .
 (More to be added)
```

## Status

| Component               | ROS 2 Humble Compatibility | Status  |
|-------------------------|--------------------------|---------|
| maviss_description      | 🚧 Under      | 🚧 Under  |

- ✅ **Tested & Working**: Fully functional with ROS 2 Humble.
- 🛠️ **In Progress**: Partially tested, might need refinements.
- ❌ **Not Yet Tested**: No testing has been performed yet.
- ⚠️ **Experimental**: Works but might be unstable.
- 🚧 **Under Development**: Not ready for deployment.



## Installation
1. Clone the repository:
    ```bash
    git clone https://github.com/your-username/maviss.git
    cd maviss
    ```

2. Install dependencies:
    ```bash
    sudo apt update && sudo apt install -y ros-${ROS_DISTRO}-ros-base
    rosdep install --from-paths src --ignore-src -r -y
    ```
3. Build the workspace:
    ```bash
    colcon build
    source install/setup.bash
    ```