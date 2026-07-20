# Engineering Portfolio

Electrical Engineering student at the University of Canterbury with an interest in embedded systems, RF, aerospace, and power electronics.

## Projects

### GNSS Navigator (STM32 + OpenStreetMaps)
Low-power embedded navigation system built around an STM32 MCU.

- Custom four-layer PCB integrating GNSS, IMU and magnetometer
- C firmware for waypoint navigation and map rendering
- Python tools to compress OpenStreetMap data into binary tiles suitable for MCU storage
- SD card tile streaming based on live NMEA position
- Future revision will incorporate a low-power e-paper display

**Technologies:** STM32, C, Python, KiCad, OpenStreetMaps

---

### Monitor Rotator (ESP32 + BLE)
Wireless monitor orientation sensor powered directly from an HDMI connection.

- Designed a four-layer ESP32 PCB with inertial sensing and BLE
- Developed firmware using ESP-IDF
- Desktop Python application receives orientation data and rotates the display automatically
- Characterised and tuned the integrated 2.4 GHz PCB antenna using a VNA

**Technologies:** ESP32, ESP-IDF, BLE, Python, RF Design

---

### Portable Trailer Light Tester
Battery-powered trailer light tester for automotive diagnostics.

- High-current synchronous buck converter
- Automated lighting test sequences
- Designed for portable operation from an internal battery
- Custom PCB and embedded control firmware

**Technologies:** Power Electronics, Embedded C, KiCad

---

### High-Efficiency LED Array
Custom circular LED lighting module.

- Python-generated PCB layout for uniform LED spacing
- Constant-current buck driver achieving approximately 93% estimated efficiency
- Designed for future integration into IoT lighting systems

**Technologies:** Python, PCB Design, Power Electronics

---

### Artix-7 System-on-Module *(In Progress)*
High-performance FPGA module based on a Xilinx Artix-7.

- Eight-rail power architecture with sequencing
- DDR3 routing and length matching
- Python routing optimisation tools
- High-speed PCB layout
