Barebone
========

The **Barebone** variant is the baseline GPS-only MAVISS platform, intended for
manual piloting and flight testing. It provides a lightweight and reliable
configuration without additional perception sensors or autonomy modules.

This variant is best suited for:

- **Fundamental flight testing** – verifying motors, ESCs, and GPS lock.  
- **Manual GPS-assisted flight** – stable outdoor flying with position hold.  
- **Research foundations** – serving as a minimal setup before adding
  perception or autonomy stacks.  

**Key Features -**


- **GPS-based positioning only** – no onboard VIO, LiDAR, or cameras.  
- **Manual control** – flown via RC transmitter or ground control station.  
- **Low system overhead** – minimal compute and power requirements.  
- **Expandable** – can be upgraded later to Evander or Penelope variants.  

**Limitations -**


- No obstacle detection or avoidance.  
- Requires reliable GPS coverage for stable flight.  
- Does not support autonomous navigation.  

.. toctree::
   :maxdepth: 1
   :hidden:

   Hardware
   Software

