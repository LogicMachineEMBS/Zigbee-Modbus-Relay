# Zigbee-Modbus-Relay

Project is built using: <br>
**esp-idf-v5.3.2** (for Linux) <br>

Go to directory: <br>
**cd** zigbee\zigbee_modbus\zigbee_modbus_relay <br>

Commads: <br>
**idf.py set-target esp32c6** <br>
**idf.py build** <br>


Binary files and addresses: <br>
**bootloader.bin 0x0** <br>
**partition-table.bin 0x8000** <br>
**zigbee_modbu_relay.bin 0x10000**

<br>
ESP32 device flashing: <br>

Download [Flash Download Tools](https://www.espressif.com/en/support/download/other-tools) <br>
Connect device to PC via USB while holding the programming button <br>
Run **flash_download_tool.exe** and select the following values <br>

* ChipType = ESP32-C6
* WorkMode = Develop
* LoadMode = USB

![setup_1](pictures/flash_download_tool_1.png) <br>

Select the binary files

Click **ERASE** button, to erase ESP32-C6 memory <br>
Click **START** button, to upload the firmware
