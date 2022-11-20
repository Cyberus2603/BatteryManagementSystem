# Battery Management System

This is Battery Management System program for [ESP32S3R8 with display](https://github.com/Xinyuan-LilyGO/T-Display-S3).
ESP reads BQ76940 registers, converts values received from it and prints them on display.
It also implements web communication via HTTP and MQTT.
Wi-Fi connection and MQTT broker is configured on first launch.

# Setup

This project should run out of the box in Arduino IDE after installing libraries and changing src/main.cpp into .ino file. I Personally used PlatformIO with **esp32s3box** config. BQ76940 driver is implemented in main.cpp file due to some ESP32 issues with the code when it was moved into separate files.

# Detailed functionality

#### Configuration via WiFi
On initial device startup ESP32 creates AP to connect to. After accessing its IP via browser you can configure your production/home Wi-Fi SSID and Password as well as MQTT server and port.

#### HTTP based communication via JSON:
- data reading (http://<IP>/data) - voltage of the entire battery, voltage on individual cells, current
- management (http://<IP>/command) - reading and setting overvoltage protection, undervoltage protection, cells balancing
- alarms (http://<IP>/alarms) - reading alarms log


#### MQTT based data collection
Voltage of the entire battery, voltage on individual cells, current send to MQTT Broker in one JSON formatted string.

### Information displayed on screen 1:
- Voltage of the entire battery pack
- Maximal voltage difference between cells
- Battery charge in %
- Unread alert icon
- Charging/discharging arrow
- Charging/discharging current
- Maximal value of the connected thermistors

### Information displayed on screen 2:
- List with the voltage of the individual cells
- List with the temperatures read from thermistors (last row)

### Information displayed on screen 3:
- Connection state
- SSID of the connected Wi-Fi
- IP of the device
- MAC address of the device
- HTTP status
- MQTT status

### Information displayed on screen 4:
- Alert log with most recent notifications

# Tips & Tricks
- If your display doesn't work, it might be due to specific TFT_eSPI library version. You must use the one from [LilyGo](https://github.com/Xinyuan-LilyGO/T-Display-S3) and don't update it!
- If you don't use all 15 cells supported by BQ76940 in your battery, then you can define which ones are connected in **"CONNECTED_CELL"** array at the **"BQ76940 DEFINITIONS"** section.
- Keep in mind that BQ76940 has several subversions ([official documentation page 3](https://www.ti.com/lit/ds/symlink/bq76940.pdf)) some require CRC (like the one I use), some do not. They also differ in terms of I2C address and voltage.
- If you are using [spark fun board V2](https://github.com/nseidle/BMS) than keep in mind that there is a bug on the board that prevents coulomb counter from working, onboard capacitors between Rsns and SRP/SRN should be grounded. Check [official documentation page 44](https://www.ti.com/lit/ds/symlink/bq76940.pdf) for further details.
