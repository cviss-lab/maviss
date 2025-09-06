# Jetson Flashing

This section explains how to set up the **NVIDIA Jetson companion computer** 
for MAVISS. The Jetson handles high-level tasks such as perception, mapping, 
and autonomy, while maintaining communication with the Pixhawk.

---

## Reference Guide

For hardware wiring and additional details, see the official PX4 guide: <a href="https://docs.px4.io/main/en/companion_computer/holybro_pixhawk_jetson_baseboard.html" target="_blank" rel="noopener noreferrer">Pixhawk + Jetson Baseboard Setup</a>.

---

## Flashing the Jetson

1. **Download JetPack** (Ubuntu-based image for Jetson) from the <a href="https://developer.nvidia.com/nvidia-sdk-manager" target="_blank" rel="noopener noreferrer">NVIDIA SDK Manager</a>.  
2. Connect the Jetson to your host PC via USB and put it in **recovery mode**.  
3. Use the **SDK Manager** to flash the OS onto the Jetson.  
4. Once complete, boot the Jetson and create a default user.  

---

## Initial Configuration

After flashing, perform the following setup:

```bash
# Update system
sudo apt update && sudo apt upgrade -y

# Install ROS 2 Humble
sudo apt install ros-humble-desktop -y

# Source ROS 2 in bashrc
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc
