1. GNSS Navigator:
  Rough proof of concept firmware for low resource map storage and display, written in C using the STM32CubeIDE HAL. Python was used to extract main roads/ways from OpenStreetMaps data and arranged them in binary tiles capable of being stored in MCU memory. Tiles of a desired region are stored on an SD card, the correct tiles are read and displayed according to NMEA data received from a GNSS receiver.
2. LED Array
  High efficiency LED array driven by a CC buck converter IC. The circular nature required the development of Python layout scripts for consistent diode spacing. DC input voltage was chosen for future integration with an IOT project. Given formula estimate the driver efficiency to be around 93% at 100% power.
3. Monitor Rotator
   External accelerometer powered from HDMI sink transceivers. Accelerometer data is received by a host-side Python program, which is used to determine and set monitor orientation. Device firmware is written in C as a modification to an Espressif IDA BLE demo program.  
