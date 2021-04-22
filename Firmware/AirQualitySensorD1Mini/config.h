/* ----------------- General config -------------------------------- */
/* WiFi */
const char* ssid                  = "YOUR SSID";     // Your WiFi SSID
const char* password              = "YOUR PASS";     // Your WiFi password

/* MQTT */
const char* mqtt_broker           = "192.168.1.185"; // IP address of your MQTT broker
const char* mqtt_username         = "";              // Your MQTT username
const char* mqtt_password         = "";              // Your MQTT password
#define     REPORT_MQTT_SEPARATE  true               // Report each value to its own topic
#define     REPORT_MQTT_JSON      true               // Report all values in a JSON message
const char* status_topic          = "events";        // MQTT topic to report startup

/* Particulate Matter Sensor */
uint32_t    g_pms_warmup_period   =  30;             // Seconds to warm up PMS before reading
uint32_t    g_pms_report_period   = 120;             // Seconds between reports

/* Serial */
#define     SERIAL_BAUD_RATE    115200               // Speed for USB serial console

/* ----------------- Hardware-specific config ---------------------- */
#define     MODE_BUTTON_PIN         D3               // GPIO0 Pushbutton to GND
#define     ESP_WAKEUP_PIN          D0               // To reset ESP8266 after deep sleep
#define     PMS_RX_PIN              D4               // Rx from PMS (== PMS Tx)
#define     PMS_TX_PIN              D8               // Tx to PMS (== PMS Rx)
#define     PMS_BAUD_RATE         9600               // PMS5003 uses 9600bps

#define     SCREEN_WIDTH           128               // OLED display width (pixels)
#define     SCREEN_HEIGHT           32               // OLED display height (pixels)
#define     OLED_RESET              -1               // Reset pin (or -1 if sharing Arduino reset pin)
