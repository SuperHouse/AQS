/*
  Particulate Matter Sensor firmware

  Read from a Plantower PMS7003 particulate matter sensor and a BME680
  environmental sensor using an ESP32, and report the values to an MQTT
  broker and show them on a 240x240 LCD.

  Written by Jonathan Oxer for www.superhouse.tv
   https://github.com/superhouse/PM25SensorESP32

  Inspired by https://github.com/SwapBap/WemosDustSensor/blob/master/WemosDustSensor.ino
*/

/*--------------------------- Configuration ------------------------------*/
// Config values are changed by editing this file:
#include "config.h"

/*--------------------------- Libraries ----------------------------------*/
#include <Wire.h>                      // I2C for BME680
#include <Adafruit_GFX.h>              // For OLED
#include <WiFi.h>                      // ESP32 WiFi driver
#include <PMS.h>                       // Particulate Matter Sensor driver
#include <PubSubClient.h>              // For MQTT
#include <TFT_eSPI.h>                  // For LCD
#include <Adafruit_NeoPixel.h>         // For RGB LEDs
#include <Adafruit_Sensor.h>           // Dependency for BME680 driver
#include <Adafruit_BME680.h>           // For BME680 environmental sensor

/* -------------------------- Resources ----------------------------------*/
#include "superhouselogo.h"            // SuperHouse logo encoded in XBM format

/*--------------------------- Global Variables ---------------------------*/
// Particulate matter sensor
#define SAMPLE_COUNT 50                // Number of samples to average
uint8_t g_pm1_latest_value   = 0;      // Latest pm1.0 reading from sensor
uint8_t g_pm2p5_latest_value = 0;      // Latest pm2.5 reading from sensor
uint8_t g_pm10_latest_value  = 0;      // Latest pm10.0 reading from sensor
uint16_t g_pm1_ring_buffer[SAMPLE_COUNT];   // Ring buffer for averaging pm1.0 values
uint16_t g_pm2p5_ring_buffer[SAMPLE_COUNT]; // Ring buffer for averaging pm2.5 values
uint16_t g_pm10_ring_buffer[SAMPLE_COUNT];  // Ring buffer for averaging pm10.0 values
uint8_t  g_ring_buffer_index = 0;      // Current position in the ring buffers
uint32_t g_pm1_average_value   = 0;    // Average pm1.0 reading from buffer
uint32_t g_pm2p5_average_value = 0;    // Average pm2.5 reading from buffer
uint32_t g_pm10_average_value  = 0;    // Average pm10.0 reading from buffer

// MQTT
String g_device_id = "default";        // This is replaced later with a unique value
uint32_t g_last_report_time = 0;       // Timestamp of last report to MQTT
char g_mqtt_message_buffer[75];        // General purpose buffer for MQTT messages
char g_pm1_mqtt_topic[50];             // MQTT topic for reporting averaged pm1.0 values
char g_pm2p5_mqtt_topic[50];           // MQTT topic for reporting averaged pm2.5 values
char g_pm10_mqtt_topic[50];            // MQTT topic for reporting averaged pm10.0 values
char g_pm1_raw_mqtt_topic[50];         // MQTT topic for reporting raw pm1.0 values
char g_pm2p5_raw_mqtt_topic[50];       // MQTT topic for reporting raw pm2.5 values
char g_pm10_raw_mqtt_topic[50];        // MQTT topic for reporting raw pm10.0 values
char g_temperature_mqtt_topic[50];     // MQTT topic for reporting temperature
char g_humidity_mqtt_topic[50];        // MQTT topic for reporting humidity
char g_pressure_mqtt_topic[50];        // MQTT topic for reporting pressure
char g_voc_mqtt_topic[50];             // MQTT topic for reporting VOC
char g_command_topic[50];              // MQTT topic for receiving commands

// OLED Display
#define STATE_GRAMS   1                // Display PMS values in grams on screen
#define STATE_INFO    2                // Display network status on screen
#define NUM_OF_STATES 2                // Number of possible states
uint8_t g_display_state = STATE_GRAMS; // Display values in grams by default

// Mode Button
uint8_t  g_current_mode_button_state  = 1;   // Pin is pulled high by default
uint8_t  g_previous_mode_button_state = 1;
uint32_t g_last_debounce_time         = 0;
uint32_t g_debounce_delay             = 200;

// Wifi
#define WIFI_CONNECT_INTERVAL      500 // Wait 500ms intervals for wifi connection
#define WIFI_CONNECT_MAX_ATTEMPTS   10 // Number of attempts/intervals to wait

// BME680
uint32_t bme680_last_sample_time = 0;  //
#define BME680_READING_INTERVAL   1000 // Wait 1000ms between sensor readings

// General
uint32_t chipId;

/*--------------------------- Function Signatures ---------------------------*/
bool initWifi();
void checkModeButton();
void reconnectMqtt();
void setColor(uint32_t c);
void updateEnvironmentalReadings();
void updatePmsReadings();
void renderScreen();

/*--------------------------- Instantiate Global Objects --------------------*/
// Particulate matter sensor
PMS pms(Serial2);
PMS::DATA data;

// LCD
TFT_eSPI tft = TFT_eSPI();

// MQTT
#if ENABLE_WIFI
WiFiClient esp_client;
PubSubClient client(esp_client);
#endif

// BME680 sensor
Adafruit_BME680 bme680;  // Using I2C mode

// RGB LEDs
Adafruit_NeoPixel strip = Adafruit_NeoPixel(4, WS2812B_PIN, NEO_GRB + NEO_KHZ800);

/*--------------------------- Program ---------------------------------------*/
/*
   This callback is invoked when an MQTT message is received. It's not important
   right now for this project because we don't receive commands via MQTT. You
   can modify this function to make the device act on commands that you send it.
*/
void callback(char* topic, byte* message, unsigned int length) {
  //Serial.print("Message arrived [");
  //Serial.print(topic);
  //Serial.print("] ");
  //for (int i = 0; i < length; i++) {
  //  Serial.print((char)payload[i]);
  //}
  //Serial.println();
}

/**
   Setup
*/
void setup()   {
  Serial.begin(9600);   // GPIO1, GPIO3 (TX/RX pin on ESP-12E Development Board)
  Serial.println();
  Serial.println("Air Quality Sensor starting up");

  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);

  strip.begin();                       // Start RGB LEDs
  strip.show();                        // Initialize all LEDs to "off"
  setColour(strip.Color(12, 0, 0));    // Set all to red

  Serial2.begin(PMS_BAUD_RATE, SERIAL_8N1, PMS_RX_PIN, PMS_TX_PIN);

  uint64_t macAddress = ESP.getEfuseMac();
  uint64_t macAddressTrunc = macAddress << 40;
  chipId = macAddressTrunc >> 40;

  tft.begin();                  // Initialise the display
  tft.fillScreen(TFT_BLACK);    // Black screen fill
  tft.drawXBitmap(0, 0, superhouse_logo, logo_width, logo_height, TFT_WHITE);

  // Start the environmental sensor
  if (bme680.begin()) {
    Serial.println("BME680 initialised");
    // Set up oversampling and filter initialization
    bme680.setTemperatureOversampling(BME680_OS_8X);
    bme680.setHumidityOversampling(BME680_OS_2X);
    bme680.setPressureOversampling(BME680_OS_4X);
    bme680.setIIRFilterSize(BME680_FILTER_SIZE_3);
    bme680.setGasHeater(320, 150); // 320*C for 150 ms
  } else {
    Serial.println("BME680 not found");
  }

  // We need a unique device ID for our MQTT client connection
  g_device_id = String(chipId, HEX);  // Get the unique ID of the ESP8266 chip in hex
  Serial.println();
  Serial.print("Device ID: ");
  Serial.println(g_device_id);

  // Set up the topics for publishing sensor readings. By inserting the unique ID,
  // the result is of the form: "device/d9616f/pm1" etc
  sprintf(g_pm1_mqtt_topic,         "device/%x/pm1",         chipId);  // From PMS
  sprintf(g_pm2p5_mqtt_topic,       "device/%x/pm2p5",       chipId);  // From PMS
  sprintf(g_pm10_mqtt_topic,        "device/%x/pm10",        chipId);  // From PMS
  sprintf(g_pm1_raw_mqtt_topic,     "device/%x/pm1raw",      chipId);  // From PMS
  sprintf(g_pm2p5_raw_mqtt_topic,   "device/%x/pm2p5raw",    chipId);  // From PMS
  sprintf(g_pm10_raw_mqtt_topic,    "device/%x/pm10raw",     chipId);  // From PMS

  sprintf(g_temperature_mqtt_topic, "device/%x/temperature", chipId);  // From BME680
  sprintf(g_humidity_mqtt_topic,    "device/%x/humidity",    chipId);  // From BME680
  sprintf(g_pressure_mqtt_topic,    "device/%x/pressure",    chipId);  // From BME680
  sprintf(g_voc_mqtt_topic,         "device/%x/voc",         chipId);  // From BME680

  sprintf(g_command_topic,          "device/%x/command",     chipId);  // For receiving messages

  Serial.println("MQTT topics:");
  Serial.println(g_pm1_mqtt_topic);         // From PMS
  Serial.println(g_pm2p5_mqtt_topic);       // From PMS
  Serial.println(g_pm10_mqtt_topic);        // From PMS
  Serial.println(g_pm1_raw_mqtt_topic);     // From PMS
  Serial.println(g_pm2p5_raw_mqtt_topic);   // From PMS
  Serial.println(g_pm10_raw_mqtt_topic);    // From PMS

  Serial.println(g_temperature_mqtt_topic); // From BME680
  Serial.println(g_humidity_mqtt_topic);    // From BME680
  Serial.println(g_pressure_mqtt_topic);    // From BME680
  Serial.println(g_voc_mqtt_topic);         // From BME680

  Serial.println(g_command_topic);          // For receiving messages

  // Connect to WiFi
#if ENABLE_WIFI
  if (initWifi()) {
    setColour(strip.Color(0, 0, 12));    // Set all to blue
    //OLED.println("Wifi [CONNECTED]");
  } else {
    //OLED.println("Wifi [FAILED]");
  }
  //OLED.display();
  delay(1000);
#else
  //OLED.println("WiFi disabled");
  Serial.println("WiFi disabled");
#endif

  pinMode(MODE_BUTTON_PIN , INPUT_PULLUP); // Pin for switching screens button

  /* Set up the MQTT client */
#if ENABLE_WIFI
  client.setServer(mqtt_broker, 1883);
  client.setCallback(callback);
#endif

  tft.fillScreen(TFT_BLACK);
}

/**
   Main loop
*/
void loop() {
#if ENABLE_WIFI
  if (WiFi.status() == WL_CONNECTED)
  {
    if (!client.connected()) {
      reconnectMqtt();
    }
  }
  client.loop();  // Process any outstanding MQTT messages
#endif

  checkModeButton();
  updateEnvironmentalReadings();
  updatePmsReadings();
  renderScreen();

  uint32_t time_now = millis();
  if (time_now - g_last_report_time > (report_interval * 1000))
  {
    g_last_report_time = time_now;
    // Generate averages for the samples in the ring buffers

    uint8_t i;

    String message_string;

    for (i = 0; i < sizeof(g_pm1_ring_buffer) / sizeof( g_pm1_ring_buffer[0]); i++)
    {
      g_pm1_average_value += g_pm1_ring_buffer[i];
    }
    g_pm1_average_value = (int)((g_pm1_average_value / SAMPLE_COUNT) + 0.5);

    for (i = 0; i < sizeof(g_pm2p5_ring_buffer) / sizeof( g_pm2p5_ring_buffer[0]); i++)
    {
      g_pm2p5_average_value += g_pm2p5_ring_buffer[i];
    }
    g_pm2p5_average_value = (int)((g_pm2p5_average_value / SAMPLE_COUNT) + 0.5);

    for (i = 0; i < sizeof(g_pm10_ring_buffer) / sizeof( g_pm10_ring_buffer[0]); i++)
    {
      g_pm10_average_value += g_pm10_ring_buffer[i];
    }
    g_pm10_average_value = (int)((g_pm10_average_value / SAMPLE_COUNT) + 0.5);

    /* Report PM1 value */
    message_string = String(g_pm1_average_value);
    Serial.print("PM1_AVERAGE:");
    Serial.println(message_string);
#if ENABLE_WIFI
    message_string.toCharArray(g_mqtt_message_buffer, message_string.length() + 1);
    client.publish(g_pm1_mqtt_topic, g_mqtt_message_buffer);
#endif

    /* Report PM2.5 value */
    message_string = String(g_pm2p5_average_value);
    Serial.print("PM2P5_AVERAGE:");
    Serial.println(message_string);
#if ENABLE_WIFI
    message_string.toCharArray(g_mqtt_message_buffer, message_string.length() + 1);
    client.publish(g_pm2p5_mqtt_topic, g_mqtt_message_buffer);
#endif

    /* Report PM10 value */
    message_string = String(g_pm10_average_value);
    Serial.print("PM10_AVERAGE:");
    Serial.println(message_string);
#if ENABLE_WIFI
    message_string.toCharArray(g_mqtt_message_buffer, message_string.length() + 1);
    client.publish(g_pm10_mqtt_topic, g_mqtt_message_buffer);
#endif

    /* Report raw PM1 value for comparison with averaged value */
    message_string = String(g_pm1_latest_value);
    Serial.print("PM1_LATEST:");
    Serial.println(message_string);
#if ENABLE_WIFI
    message_string.toCharArray(g_mqtt_message_buffer, message_string.length() + 1);
    client.publish(g_pm1_raw_mqtt_topic, g_mqtt_message_buffer);
#endif

    /* Report raw PM2.5 value for comparison with averaged value */
    message_string = String(g_pm2p5_latest_value);
    Serial.print("PM2P5_LATEST:");
    Serial.println(message_string);
#if ENABLE_WIFI
    message_string.toCharArray(g_mqtt_message_buffer, message_string.length() + 1);
    client.publish(g_pm2p5_raw_mqtt_topic, g_mqtt_message_buffer);
#endif

    /* Report raw PM10 value for comparison with averaged value */
    message_string = String(g_pm10_latest_value);
    Serial.print("PM10_LATEST:");
    Serial.println(message_string);
#if ENABLE_WIFI
    message_string.toCharArray(g_mqtt_message_buffer, message_string.length() + 1);
    client.publish(g_pm10_raw_mqtt_topic, g_mqtt_message_buffer);
#endif

    /* Report temperature */
    message_string = String(bme680.temperature);
    Serial.print("TEMP:");
    Serial.println(message_string);
#if ENABLE_WIFI
    message_string.toCharArray(g_mqtt_message_buffer, message_string.length() + 1);
    client.publish(g_temperature_mqtt_topic, g_mqtt_message_buffer);
#endif

    /* Report humidity */
    message_string = String(bme680.humidity);
    Serial.print("RHEL:");
    Serial.println(message_string);
#if ENABLE_WIFI
    message_string.toCharArray(g_mqtt_message_buffer, message_string.length() + 1);
    client.publish(g_humidity_mqtt_topic, g_mqtt_message_buffer);
#endif

    /* Report pressure */
    message_string = String(bme680.pressure);
    Serial.print("BARO:");
    Serial.println(message_string);
#if ENABLE_WIFI
    message_string.toCharArray(g_mqtt_message_buffer, message_string.length() + 1);
    client.publish(g_pressure_mqtt_topic, g_mqtt_message_buffer);
#endif

    /* Report VOC */
    message_string = String(bme680.gas_resistance);
    Serial.print("VOC:");
    Serial.println(message_string);
#if ENABLE_WIFI
    message_string.toCharArray(g_mqtt_message_buffer, message_string.length() + 1);
    client.publish(g_voc_mqtt_topic, g_mqtt_message_buffer);
#endif
  }
}

/**
   Render the correct screen based on the display state
*/
void renderScreen()
{
  switch (g_display_state) {
    case STATE_GRAMS:
      //tft.fillScreen(TFT_BLACK);
      //tft.setTextSize(1);
      //tft.setTextColor(TFT_YELLOW, TFT_BLACK);
      //tft.setCursor(0, 0, 2);
      tft.setTextSize(1);
      //tft.println("PM 1.0:");
      tft.drawNumber(g_pm1_average_value, 0, 0, 7);
      tft.drawNumber(g_pm1_latest_value, 120, 0, 7);
      tft.drawNumber(g_pm2p5_average_value, 0, 60, 7);
      tft.drawNumber(g_pm2p5_latest_value, 120, 60, 7);
      tft.drawNumber(g_pm10_average_value, 0, 120, 7);
      tft.drawNumber(g_pm10_latest_value, 120, 120, 7);
      tft.drawNumber(bme680.temperature, 0, 180, 7);
      tft.drawNumber(bme680.humidity, 120, 180, 7);
      /*
        OLED.setTextWrap(false);
        OLED.println("ug/m^3 (Atmos.)");
        OLED.print("PM 1.0 (ug/m3): ");
        OLED.println(data.PM_AE_UG_1_0);

        OLED.print("PM 2.5 (ug/m3): ");
        OLED.println(data.PM_AE_UG_2_5);

        OLED.print("PM 10.0 (ug/m3): ");
        OLED.println(data.PM_AE_UG_10_0);
      */
      break;

    case STATE_INFO:
      char mqtt_client_id[20];
      sprintf(mqtt_client_id, "esp32-%x", chipId);
      //tft.fillScreen(TFT_BLACK);
      //tft.print(mqtt_client_id);
      tft.setTextSize(3);
      tft.drawString(mqtt_client_id, 10,0);
      /*
        OLED.setTextWrap(false);
        OLED.println(mqtt_client_id);
        OLED.print("IP: ");
        OLED.println(WiFi.localIP());
        OLED.print("SSID: ");
        OLED.println(ssid);
        OLED.print("WiFi: ");
        if (WiFi.status() == WL_CONNECTED)
        {
        OLED.println("Connected");
        } else {
        OLED.println("FAILED");
        }
      */
      break;

    /* Fallback helps with debugging if you call a state that isn't defined */
    default:
      //OLED.println(g_display_state);
      break;
  }
}

/**
   Update environmental sensor values
*/
void updateEnvironmentalReadings()
{
  uint32_t time_now = millis();
  if (time_now - bme680_last_sample_time > BME680_READING_INTERVAL)
  {
    bme680.performReading();
    bme680_last_sample_time = time_now;
  }
}

/**
   Update particulate matter sensor values
*/
void updatePmsReadings()
{
  if (pms.read(data))
  {
    g_pm1_latest_value   = data.PM_AE_UG_1_0;
    g_pm2p5_latest_value = data.PM_AE_UG_2_5;
    g_pm10_latest_value  = data.PM_AE_UG_10_0;

    g_pm1_ring_buffer[g_ring_buffer_index]   = g_pm1_latest_value;
    g_pm2p5_ring_buffer[g_ring_buffer_index] = g_pm2p5_latest_value;
    g_pm10_ring_buffer[g_ring_buffer_index]  = g_pm10_latest_value;
    g_ring_buffer_index++;
    if (g_ring_buffer_index > SAMPLE_COUNT)
    {
      g_ring_buffer_index = 0;
    }
  }
}

/**
  Read the display mode button and switch the display mode if necessary
*/
void checkModeButton() {

  g_previous_mode_button_state = g_current_mode_button_state;
  g_current_mode_button_state = digitalRead(MODE_BUTTON_PIN);

  // Check if button is now pressed and it was previously unpressed
  if (g_current_mode_button_state == LOW && g_previous_mode_button_state == HIGH)
  {
    // We haven't waited long enough so ignore this press
    if (millis() - g_last_debounce_time <= g_debounce_delay) {
      return;
    }
    Serial.println("Button pressed");
    tft.fillScreen(TFT_BLACK);

    // Increment display state
    g_last_debounce_time = millis();
    if (g_display_state >= NUM_OF_STATES) {
      g_display_state = 1;
      return;
    }
    else {
      g_display_state++;
      return;
    }
  }
}

/**
  Connect to Wifi. Returns false if it can't connect.
*/
bool initWifi() {
  // Clean up any old auto-connections
  if (WiFi.status() == WL_CONNECTED) {
    WiFi.disconnect();
  }
  WiFi.setAutoConnect(false);

  // RETURN: No SSID, so no wifi!
  if (sizeof(ssid) == 1) {
    return false;
  }

  // Connect to wifi
  WiFi.begin(ssid, password);

  // Wait for connection set amount of intervals
  int num_attempts = 0;
  while (WiFi.status() != WL_CONNECTED && num_attempts <= WIFI_CONNECT_MAX_ATTEMPTS)
  {
    delay(WIFI_CONNECT_INTERVAL);
    num_attempts++;
  }

  if (WiFi.status() != WL_CONNECTED) {
    return false;
  } else {
    return true;
  }
}

/**
  Reconnect to MQTT broker, and publish a notification to the status topic
*/
#if ENABLE_WIFI
void reconnectMqtt() {
  char mqtt_client_id[20];
  sprintf(mqtt_client_id, "esp32-%x", chipId);

  // Loop until we're reconnected
  while (!client.connected()) {
    //Serial.print("Attempting MQTT connection...");
    // Attempt to connect
    if (client.connect(mqtt_client_id)) {
      //Serial.println("connected");
      // Once connected, publish an announcement
      sprintf(g_mqtt_message_buffer, "Device %s starting up", mqtt_client_id);
      client.publish(statusTopic, g_mqtt_message_buffer);
      // Resubscribe
      //client.subscribe(g_command_topic);
    } else {
      //Serial.print("failed, rc=");
      //Serial.print(client.state());
      //Serial.println(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}
#endif

/**
   Set all LEDs to the specified colour
*/
void setColour(uint32_t c) {
  for (uint16_t i = 0; i < strip.numPixels(); i++) {
    strip.setPixelColor(i, c);
  }
  strip.show();
}
