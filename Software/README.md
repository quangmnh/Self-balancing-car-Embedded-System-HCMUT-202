This folder contains the software source files, written for an ESP32 using the Arduino framework. PlatformIO is used as environment, as it is very easy to use, and provides somewhat more advanced functions, compared to the very basic Arduino IDE. 

# Build environment
For setting up PlatformIO, see https://docs.platformio.org/en/latest/ide/atom.html or https://docs.platformio.org/en/latest/ide/vscode.html. I use Atom myself, but both IDEs will work fine. 

Clone the repository. I strongly advice to not download the source files as a zip package, but to use Git properly. The code is very experimental, and will be often updated.

Once installed, go to PlatformIO home in the IDE. In there, click "Open Project", and navigate to the location where you cloned the repository. Select the "Software" folder, and then click "Open "Software"". 

Open src/main.cpp. You should now be able to combile (ctrl+alt+B).

# Uploading firmware
For uploading, I strongly prefer OTA, as this works very nice in PlatformIO.  

## Serial port
For the first upload, you'll need to upload via USB / the serial port. In platformio.ini, state the COM port under which the ESP32 module is connected (uncomment the line with an IP address / host name), for example 
upload_port = COM3. 
; upload_port = balancingRobot.local

Hit the upload button. Currently, the auto reset functionality of the ESP32 module doesn't seem to work, see Issue #10. So, once the message "Serial port COMx" appears, press the enable and boot buttons on the module. First release the boot button, then the enable button. You might have to try a few times. Luckily, once succeeded with the serial upload, you can use OTA upload.

## OTA upload
When the ESP32 boots, it's IP address is printed. In platformio.ini, fill in this IP address under upload_port. Or, even better, use the hostname balancingRobot.local. This means you don't have to mess around with IP addresses. Make sure to be connected to the same WiFi network as the ESP32, and hit upload.

## File system
Initially, or when changing the web page files, you need to upload the file system. To do so, open a terminal within the PlatformIO environment, and run:
platformio run --target uploadfs

This will upload all content of the data folder to the ESP32 flash memory (either via the serial port or OTA).

# WiFi connection
After flashing, the ESP32 will start an access point (AP), named balancingRobot. The default key is "turboturbo". Once connected, open the balancingRobot web configuration page. Here, among others, you can change the WiFi options. Under "WiFi configuration", enter the SSID and key of your home network, change the selector from "AP" to "SSID", and click the "set" button, followed by the "reboot" button. If all goes well, the ESP32 will now connect to your home network. 

If you are using an IP address instead of host name, don't forget to check the ESP32's IP address, and update this in platformio.ini. It is probably wise to assign a fixed IP address to the ESP32 (via the router).

If the home network cannot be found, the AP will be started.

# Web page editor
hostName.local/edit

User/pass: admin, admin

Be aware though that changes here are not kept on your PC, so make sure to copy and paste everything.

# Plotting signals
hostName.local/plotTest.htm

To be merged into index.htm

For this to work, first load plotTest.htm, then index.htm. This allows to view signals, while simultaneously adjusting parameters. 

# Wifi control
For a slider based web page to control your robot

hostName.local/control.htm

