/* ----------------- General Config -------------------------------- */
/* WiFi config */
const char* ssid     = "YOUR SSID";          // Your WiFi SSID
const char* password = "YOUR PASSWORD";      // Your WiFi password

/* MQTT setup */
const char* mqtt_broker   = "192.168.1.111"; // IP address of your MQTT broker
const char* status_topic  = "events";        // MQTT topic to report startup
uint32_t telemetry_period = 120;             // Report interval in seconds

/* Use WiFi. If this is off, MQTT won't run */
#define ENABLE_WIFI       true

#define SEA_LEVEL_PRESSURE_HPA (1013.25)

/* ----------------- Hardware-specific Config ---------------------- */
/* Mode button connection (momentary between this pin and GND) */
#define MODE_BUTTON_PIN   35

/* RGB LEDs */
#define WS2812B_PIN       16

/* LCD */
#define TFT_DC_PIN         2
#define TFT_RST_PIN        4
#define TFT_CS_PIN        15
#define TFT_SCLK_PIN      18
#define TFT_MISO_PIN      19
#define TFT_MOSI_PIN      23

/* I2C */
#define I2C_SDA_PIN       25
#define I2C_SCL_PIN       26
#define BME680_I2C_ADDR 0x76

/* Particulate Matter Sensor */
#define PMS_BAUD_RATE   9600
#define PMS_SET_PIN       14
#define PMS_RESET_PIN     27
#define PMS_TX_PIN        31   // Tx to the PMS (Rx at PMS)
#define PMS_RX_PIN        34   // Rx from the PMS (Tx at PMS)
