Particulate Matter Sensor firmware
==================================

Read from a Plantower PMS5003 particulate matter sensor using a Wemos D1
Mini (or other ESP8266-based board) and report the values to an MQTT
broker and show them on a 128x32 OLED display.

The mode button and OLED are optional. You can build this device with
nothing but a PMS5003 and a D1 Mini, and it will work just fine.

On startup, the device reads its own ESP8266 chip ID and uses this as the
basis for its MQTT topics and client ID, to ensure that they will be
unique for each device.

It displays this ID on the OLED display so you can read it off the
display if you have one installed. In case you don't, it also publishes
a startup message to the topic "events" including its own ID. You can
watch that topic while the device is booting to see what ID it reports.

This ID is then used to generate topics for it to report its readings.
The topics take the following form, with its unique ID substituted:

 * device/d9616f/pm1
 * device/d9616f/pm2p5
 * device/d9616f/pm10
 * device/d9616f/pm10raw

The main topics report a rolling average of the last 50 readings from the
sensor. The "...raw" topic shows the most recent reading. The PMS5003
usually reports at 10 Hz, so these values can change rapidly.

If you install the OLED and the mode button, you can use the button to
toggle the display between different screens. The default screen shows the
most recent PMS values (updated multiple times per second) and the second
screen shows network information including the MQTT ID, IP address, WiFi
SSID, and WiFi connection status.

When the TX output from the PMS5003 is connected to the RX pin on the D1
Mini, it blocks the D1 Mini from communicating via USB. This includes
updating the firmware! Use a switch or jumper to make this link, so that
you can easily disable it whenever you want to upload new firmware.

Dependencies
------------

These dependencies can all be fulfilled in the Arduino IDE using
"Sketch -> Include Library -> Manage Libraries..."

 * GFX library by Adafruit https://github.com/adafruit/Adafruit-GFX-Library
 * SSD1306 library by Adafruit https://github.com/adafruit/Adafruit_SSD1306
 * PMS library by Mariusz Kacki https://github.com/fu-hsi/pms
 * PubSubClient library by Nick O'Leary https://pubsubclient.knolleary.net

Connections
-----------

For particulate matter sensor:
 * PMS5003 VCC to D1 Mini 5V
 * PMS5003 GND to D1 Mini GND
 * PMS5003 TX to D1 Mini RX pin (via a switch or jumper)

For mode button:
 * Button connected between D1 Mini "D7" pin (GPIO13) and GND

 For 128x32 I2C OLED:
 * OLED VCC to D1 Mini 3.3V
 * OLED GND to D1 Mini GND
 * OLED SCL to D1 Mini D1
 * OLED SDA to D1 Mini D2

To do
-----

 * Refactor bulk code in loop() into separate functions.
 * Don't make display dependent on successful read from PMS. Still display
     network info if possible, and an error message on grams screen.
 * Ignore all PMS values for 30 seconds after booting, to give the sensor
     time to settle.
 * Make ring buffer size configurable.

Credits
-------

Written by Jonathan Oxer for www.superhouse.tv  https://github.com/superhouse/PM25SensorD1Mini

Inspired by https://github.com/SwapBap/WemosDustSensor
