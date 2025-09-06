(pixhawk-flashing)=
# Pixhawk Flashing

This guide explains how to install the **PX4 firmware** onto the Pixhawk flight
controller as part of the MAVISS platform setup.

---

## Firmware Download

The latest stable PX4 firmware can be downloaded from the official release
page:

- <a href="https://github.com/PX4/PX4-Autopilot/releases" target="_blank" rel="noopener noreferrer">PX4 Releases</a>

Choose the firmware version appropriate for your hardware (e.g., Pixhawk 4, 
Pixhawk 6X). It is recommended to use the latest **stable** release unless a
specific feature requires the **development** version.

---

## Flashing with QGroundControl

The firmware is flashed using **QGroundControl (QGC)**, which provides a simple
interface for managing the Pixhawk.

1. Connect the Pixhawk to your computer using a USB cable.  
2. Open <a href="https://github.com/mavlink/qgroundcontrol/releases" target="_blank" rel="noopener noreferrer">QGroundControl</a>.  
3. Navigate to:

   **Vehicle Setup → Firmware**  

4. Select **PX4 Autopilot** when prompted.  
5. QGC will automatically detect the connected Pixhawk and begin flashing.  
6. Once complete, QGC will reboot the Pixhawk with the newly installed firmware.  

---

## Verification

- After flashing, QGC should display the vehicle type and firmware version.  
- Perform an initial parameter reset to ensure defaults are applied:  

  ```bash
  param reset-all
