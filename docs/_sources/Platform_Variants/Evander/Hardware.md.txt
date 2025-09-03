# Hardware (BB + AC1) 

The **Evander** hardware configuration builds directly on the {ref}`Barebone Hardware <barebone-hardware>` setup.

In addition to the core barebone components, the following hardware is added:

RoboSense AC1
-------------------

- **Connection to Jetson**: Single USB cable directly from AC1 to the NVIDIA
  Jetson companion computer.  
- **Power Supply**: Powered through the **3-in-1 UBEC**, ensuring a regulated
  and stable voltage source.  
- **Mounting**: Fixed on the top plate of the airframe with vibration-damping
  support.  

Integration Notes
-----------------

- Verify that the **UBEC output** matches the AC1 input requirements before
  connecting.  
- Ensure the **USB link** is stable and recognized by the Jetson (check with
  **lsusb**).  
- Maintain secure cabling to prevent disconnections during flight.  

