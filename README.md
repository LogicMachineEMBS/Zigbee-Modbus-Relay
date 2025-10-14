# Zigbee-Modbus-Relay

Project is built using: <br>
**esp-idf-v5.3.2** (for Linux) <br>

Export path: <br>
**cd esp-idf** <br>
**. ./export.sh**

Go to directory: <br>
**cd zigbee\zigbee_modbus\zigbee_modbus_relay** <br>

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
![setup_2](pictures/flash_download_tool_2.png) <br>

* Tick the checkboxes
* Choose binary files and set the addresses
* Select correct COM port as detected by the OS
* Click **ERASE** button, to erase ESP32-C6 memory
* Click **START** button, to upload the firmware

<br>
**!WARNING**

**Intermittent Zigbee Initialization Failure on Cold Boot**

There is a known, hard-to-reproduce issue where the Zigbee stack may fail to initialize on a cold boot (i.e., when power is first connected). This occurs intermittently, approximately once in every 50 power cycles.

**Workaround:** A simple power cycle (unplugging and replugging the device) or a manual reset resolves the issue.

