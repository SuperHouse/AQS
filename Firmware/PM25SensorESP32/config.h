/* ----------------- General Config -------------------------------- */
/* WiFi config */
const char* ssid                  = "YOUR SSID";     // Your WiFi SSID
const char* password              = "YOUR PASS";     // Your WiFi password

/* MQTT */
const char* mqtt_broker           = "192.168.1.111"; // IP address of your MQTT broker
#define     REPORT_MQTT_SEPARATE  true               // Report each value to its own topic
#define     REPORT_MQTT_JSON      true               // Report all values in a JSON message
const char* status_topic          = "events";        // MQTT topic to report startup

/* Particulate Matter Sensor */
uint32_t    g_pms_warmup_period   =  30;             // Seconds to warm up PMS before reading
uint32_t    g_pms_report_period   = 120;             // Seconds between reports

/* Use WiFi. If this is off, MQTT won't run */
#define ENABLE_WIFI       true

/* Serial */
#define     SERIAL_BAUD_RATE       9600              // Speed for USB serial console

/* Environmental sensor */
#define SEA_LEVEL_PRESSURE_HPA (1013.25)

/* ----------------- Hardware-specific Config ---------------------- */
/* Mode button connection (momentary between this pin and GND) */
#define MODE_BUTTON_PIN    0

/* RGB LEDs */
#define WS2812B_PIN       13
#define NUM_RGB_LEDS       2

/* LCD */
#define TFT_DC_PIN         2
#define TFT_RST_PIN        4
#define TFT_CS_PIN        26
#define TFT_SCLK_PIN      18
#define TFT_MISO_PIN      19
#define TFT_MOSI_PIN      23

/* I2C */
#define I2C_SDA_PIN       21
#define I2C_SCL_PIN       22
#define BME680_I2C_ADDR 0x76

/* Particulate Matter Sensor */
#define PMS_BAUD_RATE   9600
#define PMS_SET_PIN       25
#define PMS_RESET_PIN     27
#define PMS_RX_PIN        17 //  Tx to the PMS SHOULD BE 16
#define PMS_TX_PIN        16 //  Rx from the PMS SHOULD BE 17
