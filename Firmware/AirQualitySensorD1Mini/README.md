"Basic" & "Display" Air Quality Sensor firmware for D1 Mini
============================================================

NOTE: This version of the firmware is for the "Basic" and "Display"
versions of the SuperHouse Air Quality Sensor project. It does not
work on the "Pro" version of the project.

Read from a Plantower PMS5003 particulate matter sensor using a Wemos D1
Mini (or other ESP8266-based board) and report the values to an MQTT
broker and show them on a 128x32 OLED display.

The mode button and OLED are optional. You can build this device with
nothing but a PMS5003 and a D1 Mini, and it will work just fine.

On startup, the device reads its own ESP8266 chip ID and uses this as the
basis for its MQTT topics and client ID, to ensure that they will be
unique for each device.

It displays this ID on the OLED display so you can read it off the
display if you have one installed. In case you don't, it also reports
the ID via the serial console at 9600bps, and publishes a startup
message to the topic "events" including its own ID. You can watch that
topic while the device is booting to see what ID it reports.

This ID is then used to generate topics for it to report its readings.
The topics take the following form, with its unique ID substituted:

 * tele/d9616f/PM1
 * tele/d9616f/PM2.5
 * tele/d9616f/PM10
 * tele/d9616f/PPD0.3
 * tele/d9616f/PPD0.5
 * ...etc

The values are also reported in a combined format as JSON, in the same
structure used by Tasmota, at a topic similar to:

 * tele/d9616f/SENSOR

If you install the OLED and the mode button, you can use the button to
toggle the display between different screens. The first screen shows the
most recent readings in micrograms per cubic meter. The second screen
shows the most recent readings in particles per deciliter. The third
screen shows network information including the MQTT ID, IP address, WiFi
SSID, WiFi connection status, and uptime.

Dependencies
------------

These dependencies can all be fulfilled in the Arduino IDE using
"Sketch -> Include Library -> Manage Libraries..."

 * GFX library by Adafruit https://github.com/adafruit/Adafruit-GFX-Library
 * SSD1306 library by Adafruit https://github.com/adafruit/Adafruit_SSD1306
 * PubSubClient library by Nick O'Leary https://pubsubclient.knolleary.net

It also uses a version of Mariusz Kacki's "PMS" library that was forked by
SwapBap. You do not need to install this library separately, because it's
included in the project.

Connections
-----------

For particulate matter sensor:
 * PMS5003 pin 1 (VCC) to D1 Mini "5V" pin
 * PMS5003 pin 2 (GND) to D1 Mini "GND" pin
 * PMS5003 pin 4 (RX) to D1 Mini "D8" pin
 * PMS5003 pin 5 (TX) to D1 Mini "D4" pin

For mode button:
 * Button connected between D1 Mini "D7" pin (GPIO13) and GND

For 128x32 I2C OLED:
 * OLED VCC to D1 Mini 3.3V
 * OLED GND to D1 Mini GND
 * OLED SCL to D1 Mini "D1" pin
 * OLED SDA to D1 Mini "D2" pin

To do
-----

 * 

Credits
-------

Written by Jonathan Oxer for www.superhouse.tv  
https://github.com/superhouse/AQS

Inspired by https://github.com/SwapBap/WemosDustSensor
