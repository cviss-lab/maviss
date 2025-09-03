(barebone-software)=
# Software

This section describes the software used for the **barebone GPS-only MAVISS platform**.  
At this stage, the focus is on configuring the flight controller, verifying GPS-assisted stability, and ensuring reliable communication with the ground station.

The barebone platform relies on **PX4 Autopilot** firmware running on the Pixhawk 6X Pro, managed through **QGroundControl (QGC)**.

- For firmware installation and setup, please refer to the {ref}`Pixhawk Flashing <pixhawk-flashing>`.  
- Below, we cover only the **QGroundControl workflow** for the barebone configuration.

---

### QGroundControl Setup

**Main Functions:**
- **Sensor Calibration**  
  - Accelerometer  
  - Magnetometer  
  - Radio (RC link)  
  - GPS lock verification  

- **Telemetry Monitoring**  
  - Real-time GPS position and altitude  
  - Flight mode switching (Manual, Stabilized, Position Hold)  
  - Battery voltage monitoring  

- **Parameter Tuning**  
  - Adjust PID values for stability if needed  
  - Configure failsafes (low battery, GPS loss, RC link loss)  

- **Mission Setup (Optional)**  
  - Upload simple waypoint missions for testing  
  - Verify position hold before autonomous routines  

---

**Workflow:**
1. Connect Pixhawk to QGroundControl (via USB or telemetry).  
2. Complete the {ref}`Pixhawk Flashing <pixhawk-flashing>` if not already done.  
3. Perform sensor calibrations inside QGC.  
4. Confirm GPS lock before flight.  
5. Conduct a tethered hover test in **Position Hold** mode.  

---

With PX4 + QGroundControl, the barebone platform achieves **stable GPS-assisted manual flight**, forming the baseline for future sensor and autonomy integration.

