# Hardware (BB + Livox Mid-360)

The **Penelope** hardware configuration builds directly on the {ref}`Barebone Hardware <barebone-hardware>` setup, extending it with the **Livox Mid-360 LiDAR** for high-resolution 3D perception and mapping.

## Livox Mid-360 Integration

The Livox Mid-360 provides a **360° horizontal field of view** with dense point cloud output, making it highly suitable for aerial robotics applications such as **SLAM, obstacle detection, and mapping**.  
For technical specifications, see the [Livox Mid-360 Product Page](https://www.livoxtech.com/mid-360).

### Connection Details

The Livox Mid-360 is connected to the MAVISS Penelope platform using a **dedicated 3-in-1 splitter cable** provided by Livox.  
This cable handles **all sensor I/O**, broken into three branches:

- **Power Cable** → connected to the **3-in-1 UBEC** (replacing the AC1 connection) to provide stable and regulated power.  
- **Ethernet Cable** → connected directly to the **NVIDIA Jetson Orin NX**, enabling LiDAR point cloud communication and data transfer.  
- **Function Cable** → reserved for **time synchronization** with other sensors (IMU, GPS, or additional LiDARs), ensuring multi-sensor fusion accuracy.  

### Wiring Diagram

The following diagram illustrates the wiring layout for Penelope:

![Penelope Wiring Diagram](../../_static/mav_3.png)

---

## Integration Notes

- Ensure the **UBEC output voltage** matches the Livox Mid-360 input requirements before connecting.  
- Verify the Jetson detects the LiDAR on Ethernet (`ifconfig` or `ip addr` for network interface check).  
- If using multi-sensor fusion, connect the **function/sync cable** to your time synchronization unit.  
- Maintain vibration isolation and secure cable routing to prevent disconnections during flight.  

---

**Result:**  
The Penelope platform achieves **barebone stability (BB) + Livox perception (Mid-360)**, enabling **high-resolution 3D mapping and autonomous navigation** for MAVISS.
