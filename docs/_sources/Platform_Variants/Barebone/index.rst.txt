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

.. raw:: html

   <div style="text-align: center; margin: 30px 0;">
     <a href="https://youtube.com/shorts/rPJGv6g07m0" target="_blank" rel="noopener noreferrer" 
        style="display: inline-block; position: relative; max-width: 480px; border-radius: 12px; overflow: hidden; box-shadow: 0 4px 14px rgba(0,0,0,0.25);">
       
       <!-- Custom Thumbnail -->
       <img src="../../_static/barebone_thumb.jpg" 
            alt="Barebone MAVISS Demo"
            style="width: 100%; height: auto; display: block; border-radius: 12px;">
       
       <!-- Play Button Overlay -->
       <svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 68 48" width="68" height="48"
            style="position: absolute; top: 50%; left: 50%; transform: translate(-50%, -50%);">
         <path d="M66.52,7.74a8,8,0,0,0-5.63-5.65C56.91,1,34,1,34,1S11.09,1,7.11,2.09A8,8,0,0,0,1.48,7.74,83.4,83.4,0,0,0,0,24a83.4,83.4,0,0,0,1.48,16.26,8,8,0,0,0,5.63,5.65C11.09,47,34,47,34,47s22.91,0,26.89-1.09a8,8,0,0,0,5.63-5.65A83.4,83.4,0,0,0,68,24,83.4,83.4,0,0,0,66.52,7.74Z" fill="#f00"/>
         <path d="M45,24 27,14 27,34 45,24Z" fill="#fff"/>
       </svg>
     </a>
   </div>

.. toctree::
   :maxdepth: 1
   :hidden:

   Hardware
   Software
