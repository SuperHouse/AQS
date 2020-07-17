Air Quality Sensor Pro
======================

Combines a PMS7003 particulate matter sensor with a BME680 temperature
/ humidity / barometric pressure / VOC sensor, to measure a variety of
potential airborne contaminants.

Includes a 240x240 LCD so that values can be displayed locally, and
also supports MQTT over WiFi so that values can be reported to a home
automation system or external database.

![Air Quality Sensor PCB](Images/AQS-v1_0-oblique-render.jpg)

Features:

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
EAGLE PCB design software is available from Autodesk free for
non-commercial use.


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

The case can be held shut by 8mm M3 bolts.


Credits
-------
 * Jonathan Oxer <jon@oxer.com.au>


License
-------
Copyright 2020 SuperHouse Automation Pty Ltd  www.superhouse.tv  

The hardware portion of this project is licensed under the TAPR Open
Hardware License (www.tapr.org/OHL). The "license" folder within this
repository contains a copy of this license in plain text format.

The software portion of this project is licensed under the Simplified
BSD License. The "licence" folder within this project contains a
copy of this license in plain text format.

