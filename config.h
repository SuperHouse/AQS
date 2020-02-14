/* WiFi config */
const char* ssid     = "YOUR SSID";          // Your WiFi SSID
const char* password = "YOUR PASSWORD";           // Your WiFi password

/* MQTT setup */
const char* mqtt_broker  = "192.168.1.111";  // IP address of your MQTT broker
const char* statusTopic  = "events";         // MQTT topic to publish status reports
uint32_t report_interval = 120;              // Report interval in seconds

/* Mode button connection (momentary between this pin and GND) */
const int mode_button_pin = 16;              // Pushbutton pin
