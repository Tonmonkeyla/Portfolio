<img width="4096" height="3072" alt="1785923376882" src="https://github.com/user-attachments/assets/75e6f662-0637-4e8b-9e8f-e482aae7e2a1" />
For this project I thought it would be neat to have a minimal, ultra-long battery life, and very small form factor, navigation device. 
Phones don't last forever and the thought of being stranded in an unknown area with a dead phone is very scary indeed. What if another device could be passively carried in a wallet or the pocket of a bag for years, drawing practically 0 quiescent current, and when needed provide minimal navigation for hours or days?

My first though was to store a large database of streetmaps and terrain data, then fetch and display the required streetmap data depending on the location of the device. This would require the location of the device obviously- and the only versatile option in that space is GNSS (ignoring WIFI MAC address databases etc). The other problem is navigational data- how do you store a meaningful amount of map on an embedded system, which often has storage constrained to hundreds of kilobytes? Cellular/network access is not guaranteed in situations that might neccessitate emergency navigation, which leaves storing map data locally. 

Streetmap data is available globally from openstreetmap in the PBF format. Effectively, this format defines ways using delta-encoded global coordinates (2xint64 per way point), and contains a large quantity of data not relevant to a navigational renderer application. This includes string names for each way, n number of flags or way specific variables, and metadata such as contributor ids.

This data can be extracted and formatted into a more compressed, embedded renderer friendly binary format. This is implemented in osm_to_tiles.py, which as the name suggests creates a set of map tiles from a single osm set. These tiles have a variable unsigned "extent" for example 4096=2^12=12 bits. Tiles can be extracted with variable size, for example 10km, with 0-extent defined points within that 10km. Each extracted point is local to the tile; with x/y between 0 and extent. Like PBF way points are assigned to a higher order way, which defines connected roads, pathways, or trails etc. A limited number of flags are extracted for rendering, such as "road" type ways being grouped into 2 bits of size class for rendering, with highways having the highest size value and trails etc. having the lowest. The binary format also supports closed polygons for displaying water features, with potential for displaying other area features (swamp? sand?)

It was necessary to be able to view the extracted binary files to determine the level of simplification that could be tolerated on embedded-type displays. This was accomplished using a vibe-coded web app.
This was beneficial as it allowed the rendering on embedded type displays to be previewed. IE. in TFT RGB or 2 bit grayscale as supported by some B/W epaper displays. Additionally, it displayed the total map binary loaded size to determine MCU resource usage.
![Auckland_Full_detail](Images/auckland.png)
Displaying all of Auckland CBD on a 256x256 display required loading 9 tiles with a total size of 213 KB. This roughly means that at the zoom level of 14 any map could be loaded into a high performance, low power, MCU SRAM, like a STM32U575. Because the maximum tile size at Auckland's very high detail level is ~40 KB, it also means that individual tiles could be loaded and rendered on commodity level MCUs. 
![Deep_Cove](Images/deep_cove.png)
On the other side of the spectrum very sparse tiles still present the relevant details; like Deep Cove which displays the coverage of waterways and any roads present in the OSM dataset. This example only loads 1.6 KB total. 
![grayscale_auckland](Images/256x256_epd_4grayscale.png)
Rendered in 4 grayscale the Auckland map is still usable, which suggests that a simple B/W EPD is sufficient for further developments. Storing all of Oceania at this level of simplification requires only 106 MB down from 1.3 GB, indicating that simple commodity SPI flash could store a significant area of map. 

While it was tempting to immediately start designing hardware to implement this, it would have been difficult to implement the required GNSS receiver, EMMC, EPD, and high performance MCU on the first revision. Nonetheless, I still laid out a concept of what it could look like:

![epd_concept](Images/epd_concept.png)

I thought it would be more prudent to focus on 1 or 2 subsystems on a less complex design, giving an opportunity to debug the hardware of more complex subsystems like the GNSS receiver before moving to the compact design. I decided that the subsystems tested would be the GNSS receiver and accelerometer/magnetometer. As a unit these systems can provide bearing and position, which can still be used for interesting things even without map rendering. 

The resulting prototype focused on validating the two highest-risk subsystems: the GNSS receiver and the accelerometer/magnetometer. Although this significantly reduced the functionality compared to the original concept, it provided a practical platform for developing and debugging the RF front end, antenna matching network, sensor interfaces, and embedded firmware before attempting a far more densely integrated design.

It features an STM32C071RB MCU, an AT6558R GNSS receiver with a custom RF front end comprising an antenna matching network, LNA, and SAW filter, LSM6DS3 IMU, and MMC5603 magnetometer. Information can be displayed on a 20x20 GPIO-driven led matrix.

![gnss_card](Images/min_gnss_card.png)

Despite being a lower risk design there were still some flaws with this revision. Primarily it was designed for the STM32C071RBT6-GP variant, but was fitted with an STM32C071RBT6-N, which has a second set of VDD/GND pins. This required the board to be reworked significantly:

<img width="517" height="539" alt="image" src="https://github.com/user-attachments/assets/1ca52ee3-ef86-4d0c-a9d2-429033ff9006" />
<img width="1413" height="1235" alt="image" src="https://github.com/user-attachments/assets/8c7285e1-5ce3-41b0-a3b3-730a992d93e2" />

Basically the series resistors R7 and R6 were removed, and the MCU side pins of those 0402 footprints were jumpered to 3v3 and ground testpoints respectively, with a new 100nF decoupling capacitor between them. The led matrix side of the 0402 footprints were jumpered to reassigned button GPIOs through series resistors, restoring function. 



https://github.com/user-attachments/assets/14781895-5877-41f6-9fe3-2d7c14a21f9f

As the matrix must be continuously scanned you can see rows visibly flickering, likely due to interrupt servicing for the USB CDC process required to load sequences onto the flash. This could be solved by using an RTOS or similar, or getting rid of the matrix altogether. A more refined row scanning procedure could be to use timer triggered DMA to automatically write to the GPIO registers, but this wasn't possible on this board because the row GPIO were spread across 3 GPIO banks. 

Next most problematic was that the chip antenna keepout area was violated by the internal layer 1 ground plane, which was missed because I usually keep the ground planes hidden. 

<img width="693" height="459" alt="image" src="https://github.com/user-attachments/assets/87cb4bd8-e0a7-4067-9c88-ffbe5b9bd702" />

In the revision 2 version this was fixed, and a optional UFL connector was added. This connector can be selected through an 0402 jumper to either feed the LNA from an external antenna, characterize the chip antenna, or be bypassed entirely with minimal parasitics.

<img width="930" height="832" alt="antenna_rev1_rev2" src="https://github.com/user-attachments/assets/1e1a4e4b-0baa-4c75-ba59-7668e2f56783" />

Next I worked on testing the mmc5603 magnetometer/LSM6 IMU. The magnetometer hard/soft iron offsets and correction matrices were determined by collecting raw magnetometer values as USB CDC ACM packets to a host PC, while the board was rotated through various orientations. These values were plotted live into a cloud to give feedback on captured orientations. Once the magnetometer sphere was sufficiently populated, ellipsoids describing the correction matrices were fit, and offsets determined using a python script. 
<img width="1001" height="971" alt="image" src="https://github.com/user-attachments/assets/bc378541-a4a6-4598-baeb-1599cbbd9013" />
<img width="925" height="393" alt="image" src="https://github.com/user-attachments/assets/5916b8b9-51c6-4408-888e-c85cc6210831" />

This method largely worked but the corrected magnetometer output was still slightly malformed:



Likely due to calibration around external hard iron sources (phone with big neodynium magnets etc). Although relatively isolated on the board the placement of the magnetometer could be improved in future revisions. The performance is still sufficient for rough heading in its current implementation, however.


Actually update:
<img width="943" height="647" alt="image" src="https://github.com/user-attachments/assets/3e642f12-e64f-4093-a5c1-636c4ae178dc" />

<img width="1060" height="786" alt="image" src="https://github.com/user-attachments/assets/e4799776-748a-40f6-a85f-fca0eef27b60" />

Replaced chip antenna with patch antenna 18mm>25mm, RX circuitry now canned on bottom copper with LDO power supply

Replaced LED matrix with TFT LCD, 70MHz single SPI control (note: FPC modelled inverted in solidworks accidentally, but plenty of clearance in actuality)

Replaced stm32c071 with stm32c552 with, 144MHz M33 with FPU, 3x RAM. Cross compatible with STM32F411, WCH32V203, many other QFN48 MCUs.

Isolated magnetometer

fixed errata

works pretty well, GNSS gets best SNR of ~30 dB*Hz indoors
Hagley park and center city rendered from SPI flash (firmware in progress):

<img width="1217" height="916" alt="image" src="https://github.com/user-attachments/assets/874846cc-c729-4e4b-9931-8d3223731f84" />

Entire tile set for South Island fits within 16 MB flash, with complete water/coastline geometry and all navigable ways extracted:

<img width="621" height="100" alt="image" src="https://github.com/user-attachments/assets/92c46870-6e32-4e1b-b986-17b129e4b61f" />

