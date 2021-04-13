
This folder contains KiCad (5.0.0) design files for the BalancingRobot PCB. 

# Features
* Supports both DRV8825 and A4988 based stepper drivers. Should also be compatible with TMC2100 (with settings configurable via ESP32), not tested yet though.
* Supported voltage: ~7 - 24V (2-6S LiPo/Li-ion)
* Digital current control of steppers (you'll need to desolder the potmeter and add a wire)
* Dynamic microstep switching, allowing for insane speeds
* Connections: 
	- Digital RC receiver (I use IBus / FlySky protocol), J5 upper row
	- Servo, J5 middle row. In case you want to add an arm for self righting :).
	- Digital LED (Neopixel / WS2812B etc), J5, lower row
	- All unused ESP32 pins are broken out (including I2C), for further use and development (J2)
	- Power input (J3). Allows for both a screw clamp and direct wire soldering. I prefer the last, given the high power output of a LiPo in case something comes loose.
* Wireless parameter tuning and data plotting. Makes tuning the controllers a lot easier
* Websocket communication for real-time communication between ESP32 and browser
* Web interface via access point / WiFi network. SSID and key can be set via web interface.

![Top side](/PCB/pictures/pcb_v1-1_top.png)
![Top side](/PCB/pictures/pcb_v1-1_bottom.png)

# Assembly
Assembly is quite straightforward. For those who like some instructions, here goes. For the ones that don't like to follow instructions: that's ok, but please read the last instruction about adjusting the buck converter voltage.

1. Start with the SMD components on the bottom side. First, pre-tin one pad, then place the SMD component while reheating the first pad. Finally, solder the second pad. C5 is not mounted.
2. Next, mount the voltage regulator module U3. Use some wires to connect the 4 pads to the PCB.
3. Place the IMU (U2) with the angled male and female headers. Make sure that the IMU is rigidly mounted. If you don't, the IMU might for example vibrate during operation, causing unstable behaviour. See below pictures for clarification.
4. Add all female headers (J1, J4, J7, and for the stepper driver and ESP32 modules). Optionally, place some headers on J5 and J2 if you like.
5. Finally, place C2, C4 and F1.

IMPORTANT: adjust the voltage regulator to 5V BEFORE inserting any modules. By default, the voltage regulator is set to a higher voltage, and if you don't adjust it, you'll fry some components. 

IMU mounting
<p float="left">
  <img src="/PCB/pictures/DSCN4923.JPG" width="300" />
  <img src="/PCB/pictures/DSCN4924.JPG" width="300" /> 
</p>

In the end, the PCB should look something like this
![Stepper driver improvements](/PCB/pictures/DSCN4937.JPG)

## Advanced
Some optional improvements are listed here, which are not required for basic operation. 

You can de-solder the current adjustment potentiometer on the stepper driver breakout boards. Add a wire to the wiper terminal, and connect the (two) wires to J4. Current can then be adjusted via the web interface. Or, get creative, and implement some current saving functionality.

For better performance, you can tie the decay mode pin of the DRV8825 to VCC, setting it in fast mode. The DRV8825 sometimes skips steps in slow/mixed decay mode. Below picture illustrates both improvements.

![Stepper driver improvements](/PCB/pictures/DSCN4931.JPG)

## Auxiliary header
Connector J2 has a grid of 12 pins (3x4), with some general pins broken out, together with all unused ESP32 IO pins. See the close-up of the header below. Pin numbering in this picture is used in the table below to identify pins.

//![Auxiliary header pinout](/PCB/pictures/auxHeader.PNG)
<img src="/PCB/pictures/auxHeader.PNG" width="400" />

| Pin # | Name    | ESP32 pin |
|-------|---------|-----------|
| 1     | 5V      | Vin       |
| 2     | IMU_SCL | D22       |
| 3     | IMU_SDA | D21       |
| 4     | GND     | GND       |
| 5     | AUX1    | D32       |
| 6     | AUX2    | D33       |
| 7     | GND     | GND       |
| 8     | AUX3    | D35       |
| 9     | AUX4    | D26       |
| 10    | VCC     | 3V3       |
| 11    | AUX5    | D39       |
| 12    | AUX6    | D36       |

# BOM
Applicable for v1.1. v1.0 doesn't have R7.

| Qty | Reference(s)           | Value                                    |
|-----|------------------------|------------------------------------------|
| 2   | A1, A2                 | DRV8825 / A4988 stepper driver breakout  |
| 2   | C1, C3                 | 1u capacitor, 0805 SMD                   |
| 2   | C2, C4                 | 100u elco                                |
| 1   | C5                     | 100n capacitor, not placed               |
| 1   | F1                     | Polyfuse                                 |
| 1   | J1                     | Female header, cut to size               |
| 1   | J2                     | N/A, solder whatever you like here       |
| 1   | J3                     | Screw terminal, or solder wires directly |
| 2   | J4, J7                 | 4 pin (fe)male header, cut to size       |
| 1   | J5                     | Male header, cut to size                 |
| 6   | R1, R2, R3, R4, R6, R7 | 3k3 resistor, 0805 SMD                   |
| 1   | R5                     | 100k resistor, 0805 SMD                  |
| 1   | U1                     | ESP32 devKit module, 30 pins             |
| 1   | U2                     | MPU6050 breakout, 8 pins                 |
| 1   | U3                     | Buck converter module                    |

Additionally, you'll need a pair of 8 pin male/female angled headers, for mounting the IMU. For mounting the ESP32 and stepper driver modules, you'll need 2 times 15 and 16 pin female headers (4 in total).

Some examples of the modules:
* [ESP32 Devkit V1 module, 30 pins](https://www.aliexpress.com/item/ESP32-Development-Board-WiFi-Bluetooth-Ultra-Low-Power-Consumption-Dual-Core-ESP-32-ESP-32S/32802431728.html?spm=a2g0s.9042311.0.0.26604c4dM62q0I)
* [DRV8825 stepper driver](https://www.aliexpress.com/item/Free-shipping-10pcs-lot-3D-Printer-StepStick-DRV8825-Stepper-Motor-Drive-Carrier-Reprap-4-layer-PCB/32292074706.html?spm=a2g0s.9042311.0.0.26604c4dM62q0I)
* [MPU6050 IMU](http://www.aliexpress.com/item/GY-521-MPU-6050-MPU6050-Module-3-Axis-analog-gyro-sensors-3-Axis-Accelerometer-Module/32340949017.html?spm=a2g0s.9042311.0.0.26604c4dM62q0I)
* [Buck converter](https://www.aliexpress.com/item/10PCS-Mini-3A-DC-DC-Converter-Step-Down-Module-Adjustable-3V-5V-16V-Power-for-RC/32639738406.html?spm=a2g0s.9042311.0.0.66ef4c4duolobi)

# Issues
For PCB version 1.0, the + and - signs of the RX/Servo/LED indicator are swapped. The pinout is correct though. To be clear, the order (from right to left) is -, +, and signal. This has been solved in v1.1.

# Change history
## v1.0 
Initial version
## v1.1
* Increased track width of 5V and 24V traces
* Added second via in 24V trace, or rather added trace to second via which wasn't there in v1.0
* Swapped + and - symbols on silkscreen close to RX header 
* Added a 10k pull-up on the stepper driver enable. In v1.0, no pull-up is present, meaning the steppers are enabled if the ESP32 isn't running. No big deal, but somewhat neater to disable steppers if the ESP32 isn't running.
## v1.2
* Added optional 100k RX pulldown resistor
* Added optional 5V filtering cap (might be useful if for example some Neopixels are connected to 5V line)
* Added some markings for the buck converter, clarifying the mounting direction
* Added connections for (fused) battery voltage (J6)

