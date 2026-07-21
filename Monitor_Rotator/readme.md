The main idea behind this project was that the internet is becoming increasingly "vertical"; many apps/websites are designed primarily for mobile usage, with desktops being a faint afterthought. 
However, many monitors can be physically rotated, vertically<->horizontally, easily and quickly. The issue lies with manually setting the orientation of the monitor in the operating system; ie. in windows this can only be changed in the display settings. 
Monitors do not "know" what orientation they are in. They simply display images, and do not have the hardware to detect orientation.
A solution to this is to couple an accelerometer to the orientation of the monitor. The easy, and boring, way to do this is to simply attach an externally powered accelerometer board to the monitor (tape).
However this gets messy fast, the power and data cables get tangled with the existing monitor connections, making it a temporary solution at best. 
Powering the accelerometer board directly from the monitor and transmitting the accelerometer data back to the host PC wirelessly solves this, with exactly 0 additional cables.

The complex problem is receiving power from the monitor. While some monitors have USB passthrough and therefore a nice 5V DC supply, most do not. In fact, almost none of the connectors on a standard monitor have any power supply whatsoever, the exception being 500mA DP_PWR 3.3V on DisplayPort inputs.
Unfortunately, most monitors have a single DisplayPort, which is often the main display input. This leaves exactly 0 conventional, viable power supplies. 

HDMI uses a lot of power to receive signals from the host PC (up to ~130 mW). This is largely due to transceiver TMDS terminations being present on the input side, which are directly tied to the exposed connector transceiver pairs. These terminations are implemented as 50 ohm parallel terminations to aVCC, a 3.3v source. A logical zero from the host PC sinks around 10mA of current per termination resistor, which for a typical data pattern (~50% low) is substantial across the 8 terminations available. This effectively means we can sink 5mA*8=~40mA of 3.05V across the 50 ohm terminations, while technically remaining within HDMI design specification. 
<img width="881" height="471" alt="image" src="https://github.com/user-attachments/assets/da8ae4bc-47de-4ec8-8634-7bbb3ecb85eb" />

This leads to another challenge- how do you consistently sink no greater than 5mA per termination while maximising the available power budget. Connecting the terminations directly to a device side 3.3V net is clearly a bad idea; RF MCUs can draw up to 200mA while transmitting, and even if that were possible within the voltage drop of the terminations it would fry the HDMI aVCC supply instantly. And it isn't possible because it would result in a voltage drop brownout+the catastrophic overcurrent of the HDMI circuit. Initially, my idea was to use a discrete linear current regulator- tie all terminations together through a shunt resistor, V=IR, hold I at 40mA etc. This would use a low VGS PFET in conjunction with a current sense OP-amp to limit the input current to 40mA through to a large capacitor or supercapacitor. Given a (very) low average current draw of the MCU, the voltage of the capacitor net would eventually charge to ~3.3V, which could then be regulated down to an MCU operating voltage of 1.8V or 3V.

The version 2 hardware instead uses a buck converter with op-amp regulated voltage foldback. 6 of the available 8 terminations are used as power terminations, with variable voltage across the termination according to current draw. The remaining 2 are used as reference voltages with minimal current draw; and remain at ~3.3v. The differential voltage between the "reference" and "power" nets is used by the differential op amp to sense average current through the power terminations; and raise the feedback voltage of the buck converter high when input current is exceeded. This effectively uses the 50 ohm terminations themselves as current sense resistors. This current foldback protects the HDMI transceivers from any possible damage.
<img width="1262" height="535" alt="image" src="https://github.com/user-attachments/assets/b9581231-90e9-4d8d-a172-5e49a2c0ea89" />

The power net voltage can approach the reference/VDD "3.3v", so a relatively specialized over the rails OP amp (MCP6021) was required. It was initially designed for the LMV321, however as it turns out the LMV321 does not have rail to rail inputs, as most modern OP-amps have. Additionally the LMV321 has a nasty characteristic when the input differential drops below specification, where the output inverts, effectively dropping the output voltage to the negative rail. As you can imagine buck regulator feedback loops do not tolerate this.

This current regulation feeds into a large (5.7 mF) electrolytic polymer capacitor, which over 100s of mS charges to slightly less than 3.3v. This capacitor net is then regulated to 3.3v exactly using a low quiescent, ultra low input voltage boost converter (TPS61299). This provides stable and low noise power for the MCU, accelerometer, and associated RF circuitry. 

Probing the assembled version 2 revealed that current through the terminations could spike outside of specifications:
A. During startup, when there is significant inrush current into the VIN bulk capacitors of the buck converter.
B. During current spikes on the load side.

The solution to this was to add an additional linear current regulator immediately after the termination nets on the device side. This uses one op-amp unit of a two unit device to control a PFET gate. When the device side termination voltage drops below a set threshold the PFET gate is pulled high to maintain the set current limit, with a very fast response time.
<img width="859" height="513" alt="image" src="https://github.com/user-attachments/assets/4d7ffd00-81a7-4cff-826e-d32a26ee51e6" />
<img width="2456" height="586" alt="image" src="https://github.com/user-attachments/assets/273ec4ac-4488-419a-8ad3-b58615258a96" />
The red trace shows current through R18 into a 2x47uF input bank during startup. Current peaks at 60 mA for a few nanoseconds, before being arrested and maintained at 45 mA until the capacitors are charged and inrush current drops, at which point the switching circuit takes over current regulation. Without the linear regulator it would effectively be 3.3V->50 ohms->ground at startup, drawing ~60 mA (per termination) so 360 mA total through the HDMI transceiver, which would probably fry it.

The "hard" linear current limit and "soft" switching current limit are both user adjustable through potentiometers on the finished board:
<img width="798" height="469" alt="image" src="https://github.com/user-attachments/assets/cdddf9c0-8353-4b8a-a1fe-a5554163d04f" />

This allows for differences in voltage drop through the summing diodes and op-amp differences to be accounted for experimentally.

The MCU chosen was the ESP32C3. In hindsight, this was a poor decision as the BLE power consumption is diminished by sharing circuitry with WIFI. During transmission this can spike to >300 mA at 3.3v, and even in the lowest power, connection maintaining, state still uses ~20 mA on average. This stresses the power budget at best and causes intermittent brownouts when transmissions are not optimized. Eg. to even establish a BLE connection the buffering capacitance needed to be doubled to 11.4 mF. The in-progress version 3 uses a NRF54 series MCU, with less than a tenth of the power consumption, at 1v8.

The version 2 uses a 3216 chip antenna for BLE/WIFI, while the version 2 rev 2 uses a TI AN043 inverted F pcb antenna. 
<img width="385" height="211" alt="image" src="https://github.com/user-attachments/assets/11265a98-20e4-4987-842f-de176d83f06e" /><img width="875" height="493" alt="image" src="https://github.com/user-attachments/assets/ed8c31db-50b4-46dd-990e-13932fe70d42" />

Surprisingly, both held a stable connection within operating parameters (ie. BLE, within 5 meters) without any tuning, fitting only a 0 ohm as the series element of the PI network. However, the performance was subpar beyond that. As modern RF socs generally adjust the transmission power according to RSSI, it was desirable to increase the antenna efficiency to reduce the power consumption of each packet. 

This required assaying the antenna(s) with a VNA. This was challenging as an RF test port was neglected in both designs. The first attempt was simply soldering a RG316 pigtail to the feedpoint of the antenna:
<img width="774" height="1034" alt="image" src="https://github.com/user-attachments/assets/2bf4d29f-3a2f-4296-89c3-68368cb5e7d3" />

Unsurprisingly this didn't give a good measurement; with the resonant frequency of the antenna measuring ~1.9 GHz showing that it was significantly detuned by the test fixture:
<img width="1481" height="807" alt="image" src="https://github.com/user-attachments/assets/71236390-e7ad-44a3-a152-a5e2c203cd88" />

For the second attempt I instead removed some solder mask, exposing the ground plane into a U.FL-like footprint, then soldered an IPEX-1 connector to it:
<img width="875" height="1151" alt="image" src="https://github.com/user-attachments/assets/dfb0f637-b926-4956-8163-e79f4d434ff9" />

This allowed the coaxial shield to be rotated away from the radiating area:
<img width="867" height="1167" alt="image" src="https://github.com/user-attachments/assets/ec85d72d-495c-4aad-9432-8ea72bedd459" />

And gave slightly more believable S11, with resonance now measuring around 2.1 GHz:
<img width="851" height="466" alt="image" src="https://github.com/user-attachments/assets/6665f54a-7973-4a9b-a3ec-7d4df056dde7" />

Which is likely still altered by the large U.FL connector directly adjacent to the radiating area.

Interestingly, rotating the U.FL connector back to parallel with the radiating area detuned the antenna similarly to the pigtail, with S11 resonance dropping back to 1.9 GHz.
<img width="852" height="450" alt="image" src="https://github.com/user-attachments/assets/e81d4755-c7cc-4b0e-85be-99aba876c4b3" />

Despite the measurements likely still being altered by the U.FL connector, the Smith Chart of the configurating with the coaxial rotated away from the radiating area was used to determine matching component values.
<img width="887" height="531" alt="image" src="https://github.com/user-attachments/assets/df97c57d-5d2f-4cc1-a548-be57dfb6d0a0" />

Which, if naively believed, shows that the antenna is very poorly matched with an impedance of 40.7-76.6j ohms at 2.4GHz, and 16.4-35.8j at 2.44Ghz. This suggests a very preliminary matching network on the order of 2-5 nH of series inductance and 1-2 pF of shunt capacitance.

A better takeaway is to always include RF test feedpoints that are sufficiently isolated from the antenna, like a jumper connected U.FL connector (with minimal routed stub) at the opposite end of the transmission line.

### Version 3
Uses STM32WB05 with orders of magnitude lower BLE power consumption over ESP32C3. Larger power budget enables far more robust resistive power supply:

<img width="1615" height="1090" alt="image" src="https://github.com/user-attachments/assets/1c54b035-cd7d-4d72-8213-abf43d9e80fb" />

Every TMDS string is individually ORed into a >40F LIC hybrid supercapacitor operating between 3v3 and 2v5, each string is in series with a 100 ohm resistor, current through each termination physically cannot exceed 6 mA, albeit with acceptable resistive losses. 
Individual current limiting resistors makes device compatible with larger range of monitors. Not all monitors power on all transceiver pairs at all times, and some completely power off all transceivers until HPD or source 5V is asserted. Testing shows that when 5V or HPD is asserted a significant subset of monitors raise all transceiver pairs, while some only raise the CLK pair. The power scheme passively handles these cases, and can assert 5V lines through a low quiescent boost 5v net. Low supercapacitor leakage and very low total device current draw (~40 uA) means that the device should be able to operate for ~weeks without any external power at all.   

<img width="1055" height="635" alt="image" src="https://github.com/user-attachments/assets/eb681f45-cda8-4d80-85d2-44083f390323" />

Dedicated antenna test port/alternate feed through IPX-1 footprint, placed away from antenna on counterbalance side, away from main radiating area. Could use more clearance.

<img width="1585" height="419" alt="image" src="https://github.com/user-attachments/assets/83b20911-95e3-4825-8a2a-65e8534b16e1" />




