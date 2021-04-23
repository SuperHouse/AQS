Air Quality Sensor
==================

Build a DIY Air Quality Sensor to monitor the condition of the air in
your home or office, with options for measuring airborne particulate
matter, temperature, humidity, barometric pressure, and volatile
organic compounds.

This project is documented in an ongoing video series on SuperHouse:

* [DIY Air Quality Sensor, part 1: “Basic” version](https://www.superhouse.tv/38-diy-air-quality-sensor-part-1-basic-model/)
* [DIY Air Quality Sensor, part 2: “Display” version](https://www.superhouse.tv/39-diy-air-quality-sensor-part-2-display-version/)
* [DIY Air Quality Sensor, part 3: source code walkthrough](https://www.superhouse.tv/40-diy-air-quality-sensor-part-3-software/)

There are 3 different versions of this project:

 * The "Basic" version which requires only a Wemos D1 Mini and a
   Plantower PMS5003 laser particulate matter sensor.
 * The "Display" version which adds a 128x32 OLED display to the
   Basic version.
 * The "Pro" version which is based on a fully custom PCB with an ESP32
   MCU, a Plantower PMS7003 sensor, a Bosch BME680 environmental
   sensor, and a 240x240 colour LCD.

Measurements can be shown locally if a display is fitted, and can also
be reported to MQTT over WiFi so that data can be accessed by a home
automation system or external database.

![Air Quality Sensor PCB](Images/AQS-v1_0-oblique-render.jpg)

"Basic" and "Display" version features:

 * PMS5003 particulate matter sensor
 * ESP8266 processor with WiFi
 * Micro-USB connection for power or reflashing
 * 3D-printable case

"Pro" version features:

 * PMS7003 particulate matter sensor
 * BME680 VOC sensor
 * ESP32 processor with WiFi and Bluetooth
 * USB-C connection for power or reflashing
 * 3D-printable case

More information is available at:

  https://www.superhouse.tv/aqs


Hardware
--------
The "Hardware" directory contains the PCB design as an EAGLE project.


Firmware
--------
The "Firmware" directory contains example firmware as an Arduino
project.


Enclosure
---------
The "Enclosure" directory contains STLs for a case that can be
3D-printed.

The two halves of the case should be printed with the flat face down to
the printer bed.

The case can be held shut by M3 bolts.


Credits
-------
 * Jonathan Oxer <jon@oxer.com.au>


License
-------
Copyright 2020-2021 SuperHouse Automation Pty Ltd  www.superhouse.tv  

The hardware portion of this project is licensed under the TAPR Open
Hardware License (www.tapr.org/OHL). The "license" folder within this
repository contains a copy of this license in plain text format.

The software portion of this project is licensed under the Simplified
BSD License. The "licence" folder within this project contains a
copy of this license in plain text format.

