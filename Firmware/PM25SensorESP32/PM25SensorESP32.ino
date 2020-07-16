/*
  Particulate Matter Sensor firmware (ESP32 version)

  Written by Jonathan Oxer for www.superhouse.tv
   https://github.com/superhouse/PM25SensorESP32

  Read from a Plantower PMS7003 particulate matter sensor and a BME680
  environmental sensosr using an ESP32, and report the values to an MQTT
  broker and show them on a 240x240 LCD, with a mode button to change
  between display modes.

  External dependencies. Install using the Arduino library manager:
     "Adafruit GFX Library" by Adafruit
     TFT_eSPI.h
     Adafruit_NeoPixel.h
     Adafruit_Sensor.h
     Adafruit_BME680.h
     "PubSubClient" by Nick O'Leary

  Bundled dependencies. No need to install separately:
     "PMS Library" by Mariusz Kacki, forked by SwapBap

  Inspired by https://github.com/SwapBap/WemosDustSensor/
*/
#define VERSION "2.1"
/*--------------------------- Configuration ------------------------------*/
// Configuration should be done in the included file:
#include "config.h"

/*--------------------------- Libraries ----------------------------------*/
#include <Wire.h>                     // I2C for BME680
#include <Adafruit_GFX.h>             // For OLED
#include <WiFi.h>                     // ESP32 WiFi driver
#include <PubSubClient.h>             // For MQTT
#include <TFT_eSPI.h>                 // For LCD
#include <Adafruit_NeoPixel.h>        // For RGB LEDs
#include <Adafruit_Sensor.h>          // Dependency for BME680 driver
#include <Adafruit_BME680.h>          // For BME680 environmental sensor
#include "PMS.h"                      // Particulate Matter Sensor driver (embedded)

/* -------------------------- Resources ----------------------------------*/
#include "superhouselogo.h"           // SuperHouse logo encoded in XBM format

/*--------------------------- Global Variables ---------------------------*/
// Particulate matter sensor
#define   PMS_STATE_ASLEEP        0   // Low power mode, laser and fan off
#define   PMS_STATE_WAKING_UP     1   // Laser and fan on, not ready yet
#define   PMS_STATE_READY         2   // Warmed up, ready to give data
uint8_t   g_pms_state           = PMS_STATE_WAKING_UP;
uint32_t  g_pms_state_start     = 0;  // Timestamp when PMS state last changed
uint8_t   g_pms_readings_taken  = 0;  // 0/1: whether any readings have been taken
uint8_t   g_pms_ppd_readings_taken = 0;  // 0/1: whether PPD readings have been taken

uint16_t  g_pm1p0_sp_value      = 0;  // Standard Particle calibration pm1.0 reading
uint16_t  g_pm2p5_sp_value      = 0;  // Standard Particle calibration pm2.5 reading
uint16_t  g_pm10p0_sp_value     = 0;  // Standard Particle calibration pm10.0 reading

uint16_t  g_pm1p0_ae_value      = 0;  // Atmospheric Environment pm1.0 reading
uint16_t  g_pm2p5_ae_value      = 0;  // Atmospheric Environment pm2.5 reading
uint16_t  g_pm10p0_ae_value     = 0;  // Atmospheric Environment pm10.0 reading

uint32_t  g_pm0p3_ppd_value     = 0;  // Particles Per Deciliter pm0.3 reading
uint32_t  g_pm0p5_ppd_value     = 0;  // Particles Per Deciliter pm0.5 reading
uint32_t  g_pm1p0_ppd_value     = 0;  // Particles Per Deciliter pm1.0 reading
uint32_t  g_pm2p5_ppd_value     = 0;  // Particles Per Deciliter pm2.5 reading
uint32_t  g_pm5p0_ppd_value     = 0;  // Particles Per Deciliter pm5.0 reading
uint32_t  g_pm10p0_ppd_value    = 0;  // Particles Per Deciliter pm10.0 reading

// MQTT
char g_mqtt_message_buffer[150];      // General purpose buffer for MQTT messages
char g_command_topic[50];             // MQTT topic for receiving commands

#if REPORT_MQTT_SEPARATE
char g_pm1p0_ae_mqtt_topic[50];       // MQTT topic for reporting pm1.0 AE value
char g_pm2p5_ae_mqtt_topic[50];       // MQTT topic for reporting pm2.5 AE value
char g_pm10p0_ae_mqtt_topic[50];      // MQTT topic for reporting pm10.0 AE value
char g_pm0p3_ppd_mqtt_topic[50];      // MQTT topic for reporting pm0.3 PPD value
char g_pm0p5_ppd_mqtt_topic[50];      // MQTT topic for reporting pm0.5 PPD value
char g_pm1p0_ppd_mqtt_topic[50];      // MQTT topic for reporting pm1.0 PPD value
char g_pm2p5_ppd_mqtt_topic[50];      // MQTT topic for reporting pm2.5 PPD value
char g_pm5p0_ppd_mqtt_topic[50];      // MQTT topic for reporting pm5.0 PPD value
char g_pm10p0_ppd_mqtt_topic[50];     // MQTT topic for reporting pm10.0 PPD value
char g_temperature_mqtt_topic[50];    // MQTT topic for reporting temperature
char g_humidity_mqtt_topic[50];       // MQTT topic for reporting humidity
char g_pressure_mqtt_topic[50];       // MQTT topic for reporting pressure
char g_voc_mqtt_topic[50];            // MQTT topic for reporting VOC
char g_altitude_mqtt_topic[50];       // MQTT topic for reporting altitude
#endif
#if REPORT_MQTT_JSON
char g_mqtt_json_topic[50];           // MQTT topic for reporting all values using JSON
#endif

// OLED Display
#define DISPLAY_STATE_GRAMS   1       // Display values in micrograms/m^3 on screen
#define DISPLAY_STATE_PPD     2       // Display values in parts per deciliter on screen
#define DISPLAY_STATE_INFO    3       // Display network status on screen
#define NUM_OF_STATES 3               // Number of possible states
uint8_t g_display_state = DISPLAY_STATE_GRAMS;  // Display values in micrograms/m^3 by default

// Mode Button
uint8_t  g_current_mode_button_state  =  1;  // Pin is pulled high by default
uint8_t  g_previous_mode_button_state =  1;
uint32_t g_last_debounce_time         =  0;
uint32_t g_debounce_delay             = 50;

// Wifi
#define WIFI_CONNECT_INTERVAL          500   // Wait 500ms intervals for wifi connection
#define WIFI_CONNECT_MAX_ATTEMPTS       10   // Number of attempts/intervals to wait

// BME680
uint32_t g_bme680_last_sample_time     = 0;  //
#define BME680_READING_INTERVAL       1000   // Wait 1000ms between sensor readings

// General
uint32_t g_device_id;                        // Unique ID from ESP chip ID

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
PMS::DATA g_data;

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
Adafruit_NeoPixel strip = Adafruit_NeoPixel(NUM_RGB_LEDS, WS2812B_PIN, NEO_GRB + NEO_KHZ800);

/*--------------------------- Program ---------------------------------------*/
/**
  Setup
*/
void setup()
{
  Serial.begin(SERIAL_BAUD_RATE);
  Serial.println();
  Serial.print("Air Quality Sensor starting up, v");
  Serial.println(VERSION);

  // Open a connection to the PMS and put it into passive mode    
  Serial2.begin(PMS_BAUD_RATE, SERIAL_8N1, PMS_RX_PIN, PMS_TX_PIN);  // Connection for PMS5003
  pms.passiveMode();                // Tell PMS to stop sending data automatically
  delay(100);
  pms.wakeUp();                     // Tell PMS to wake up (turn on fan and laser)

  // We need a unique device ID for our MQTT client connection
  uint64_t macAddress = ESP.getEfuseMac();
  uint64_t macAddressTrunc = macAddress << 40;
  g_device_id = macAddressTrunc >> 40;
  Serial.print("Device ID: ");
  Serial.println(g_device_id, HEX);

  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);

  strip.begin();                       // Start RGB LEDs
  strip.show();                        // Initialize all LEDs to "off"
  setColour(strip.Color(20, 0, 0));    // Set all to red while starting
  delay(500);
  //strip.show();                        // Initialize all LEDs to "off"
  setColour(strip.Color(0, 20, 0));    // Set all to red while starting
  delay(500);
  //strip.show();                        // Initialize all LEDs to "off"
  setColour(strip.Color(0, 0, 20));    // Set all to red while starting
  delay(500);

  Serial2.begin(PMS_BAUD_RATE, SERIAL_8N1, PMS_RX_PIN, PMS_TX_PIN);

  tft.begin();                         // Initialise the display
  //tft.fillScreen(TFT_BLACK);           // Black screen fill
  tft.drawXBitmap(0, 0, superhouse_logo, logo_width, logo_height, TFT_WHITE);

  // Start the environmental sensor
  if (bme680.begin(BME680_I2C_ADDR))
  {
    Serial.println("BME680 initialised");
    // Set up oversampling and filter initialization
    bme680.setTemperatureOversampling(BME680_OS_8X);
    bme680.setHumidityOversampling(BME680_OS_2X);
    bme680.setPressureOversampling(BME680_OS_4X);
    bme680.setIIRFilterSize(BME680_FILTER_SIZE_3);
    bme680.setGasHeater(320, 150);     // 320*C for 150 ms
  } else {
    Serial.println("BME680 not found");
  }

  // Set up the topics for publishing sensor readings. By inserting the unique ID,
  // the result is of the form: "device/d9616f/PM1P0" etc
  sprintf(g_command_topic,         "cmnd/%x/COMMAND",     g_device_id);  // For receiving commands
#if REPORT_MQTT_SEPARATE
  sprintf(g_pm1p0_ae_mqtt_topic,   "tele/%x/AE1P0",       g_device_id);  // Data from PMS
  sprintf(g_pm2p5_ae_mqtt_topic,   "tele/%x/AE2P5",       g_device_id);  // Data from PMS
  sprintf(g_pm10p0_ae_mqtt_topic,  "tele/%x/AE10P0",      g_device_id);  // Data from PMS
  sprintf(g_pm0p3_ppd_mqtt_topic,  "tele/%x/PPD0P3",      g_device_id);  // Data from PMS
  sprintf(g_pm0p5_ppd_mqtt_topic,  "tele/%x/PPD0P5",      g_device_id);  // Data from PMS
  sprintf(g_pm1p0_ppd_mqtt_topic,  "tele/%x/PPD1P0",      g_device_id);  // Data from PMS
  sprintf(g_pm2p5_ppd_mqtt_topic,  "tele/%x/PPD2P5",      g_device_id);  // Data from PMS
  sprintf(g_pm5p0_ppd_mqtt_topic,  "tele/%x/PPD5P0",      g_device_id);  // Data from PMS
  sprintf(g_pm10p0_ppd_mqtt_topic, "tele/%x/PPD10P0",     g_device_id);  // Data from PMS
  sprintf(g_temperature_mqtt_topic, "tele/%x/temperature", g_device_id); // From BME680
  sprintf(g_humidity_mqtt_topic,   "tele/%x/humidity",    g_device_id);  // From BME680
  sprintf(g_pressure_mqtt_topic,   "tele/%x/BARO",        g_device_id);  // From BME680
  sprintf(g_voc_mqtt_topic,        "tele/%x/VOC",         g_device_id);  // From BME680
  sprintf(g_altitude_mqtt_topic,   "tele/%x/ALT",         g_device_id);  // From BME680
#endif
#if REPORT_MQTT_JSON
  sprintf(g_mqtt_json_topic,       "tele/%x/SENSOR",      g_device_id);  // Data from PMS
#endif

  // Report the MQTT topics to the serial console
  Serial.println(g_command_topic);          // For receiving messages
#if REPORT_MQTT_SEPARATE
  Serial.println("MQTT topics:");
  Serial.println(g_pm1p0_ae_mqtt_topic);    // From PMS
  Serial.println(g_pm2p5_ae_mqtt_topic);    // From PMS
  Serial.println(g_pm10p0_ae_mqtt_topic);   // From PMS
  Serial.println(g_pm0p3_ppd_mqtt_topic);   // From PMS
  Serial.println(g_pm0p5_ppd_mqtt_topic);   // From PMS
  Serial.println(g_pm1p0_ppd_mqtt_topic);   // From PMS
  Serial.println(g_pm2p5_ppd_mqtt_topic);   // From PMS
  Serial.println(g_pm5p0_ppd_mqtt_topic);   // From PMS
  Serial.println(g_pm10p0_ppd_mqtt_topic);  // From PMS

  Serial.println(g_temperature_mqtt_topic); // From BME680
  Serial.println(g_humidity_mqtt_topic);    // From BME680
  Serial.println(g_pressure_mqtt_topic);    // From BME680
  Serial.println(g_voc_mqtt_topic);         // From BME680
  Serial.println(g_altitude_mqtt_topic);    // From BME680
#endif
#if REPORT_MQTT_JSON
  Serial.println(g_mqtt_json_topic);        // From PMS
#endif

  // Connect to WiFi
#if ENABLE_WIFI
  if (initWifi()) {
    setColour(strip.Color(0, 0, 20));    // Set all to blue
    //OLED.println("Wifi [CONNECTED]");
  } else {
    //OLED.println("Wifi [FAILED]");
  }
  //OLED.display();
  //delay(1000);
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
  setColour(strip.Color(0, 12, 0));    // Set all to green
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
}

/**
  Render the correct screen based on the display mode
*/
void renderScreen()
{
  // Render our displays
  switch (g_display_state)
  {
    case DISPLAY_STATE_GRAMS:
      //tft.fillScreen(TFT_BLACK);
      //tft.setTextSize(3);
      tft.setTextColor(TFT_YELLOW, TFT_BLACK);
      //tft.setCursor(0, 0, 2);
      tft.setTextSize(1);
      tft.setTextFont(6);
      //tft.println("PM 1.0:");
      tft.drawNumber(g_pm1p0_ae_value,      0,   0);
      tft.drawNumber(g_pm1p0_ppd_value,   120,   0);
      tft.drawNumber(g_pm2p5_ae_value,      0,  45);
      tft.drawNumber(g_pm2p5_ppd_value,   120,  45);
      tft.drawNumber(g_pm10p0_ae_value,     0,  90);
      tft.drawNumber(g_pm10p0_ppd_value,  120,  90);
      tft.drawNumber(bme680.temperature,    0, 135);
      tft.drawNumber(bme680.humidity,     120, 135);
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

    case DISPLAY_STATE_PPD:
      //tft.fillScreen(TFT_BLACK);
      //tft.setTextSize(3);
      tft.setTextColor(TFT_YELLOW, TFT_BLACK);
      //tft.setCursor(0, 0, 2);
      tft.setTextSize(1);
      tft.setTextFont(6);
      //tft.println("PM 1.0:");
      tft.drawNumber(g_pm1p0_ae_value,      0,   0);
      tft.drawNumber(g_pm1p0_ppd_value,   120,   0);
      tft.drawNumber(g_pm2p5_ae_value,      0,  45);
      tft.drawNumber(g_pm2p5_ppd_value,   120,  45);
      tft.drawNumber(g_pm10p0_ae_value,     0,  90);
      tft.drawNumber(g_pm10p0_ppd_value,  120,  90);
      tft.drawNumber(bme680.temperature,    0, 135);
      tft.drawNumber(bme680.humidity,     120, 135);
      break;

    case DISPLAY_STATE_INFO:
      char mqtt_client_id[20];
      sprintf(mqtt_client_id, "esp32-%X", g_device_id);
      //tft.fillScreen(TFT_BLACK);
      //tft.print(mqtt_client_id);
      tft.setCursor(0, 0);
      tft.setTextSize(1);
      tft.setTextFont(4);

      tft.print("IP: ");
      tft.setTextColor(TFT_YELLOW);
      tft.println(WiFi.localIP());
      tft.setTextColor(TFT_WHITE);
      tft.print("ID: ");
      tft.setTextColor(TFT_YELLOW);
      tft.println(mqtt_client_id);
      tft.setTextColor(TFT_WHITE);
      tft.print("SSID: ");
      tft.setTextColor(TFT_YELLOW);
      tft.println(WiFi.SSID());

      tft.setTextColor(TFT_WHITE);
      tft.print("WiFi: ");
      if (WiFi.status() == WL_CONNECTED)
      {
        tft.setTextColor(TFT_GREEN);
        tft.println("Connected");
        tft.setTextColor(TFT_WHITE);
      } else {
        tft.setTextColor(TFT_RED);
        tft.println("FAILED");
        tft.setTextColor(TFT_WHITE);
      }
      break;

    /* Fallback helps with debugging if you call a state that isn't defined */
    default:
      tft.setCursor(0, 0);
      tft.setTextSize(1);
      tft.setTextFont(6);
      tft.print(g_display_state);
      break;
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
    Serial.println(g_display_state);
    tft.fillScreen(TFT_BLACK);

    // Increment display state
    g_last_debounce_time = millis();
    if (g_display_state >= NUM_OF_STATES) {
      g_display_state = 1;
      return;
    } else {
      g_display_state++;
      return;
    }
  }
}

/**
  Update environmental sensor values
*/
void updateEnvironmentalReadings()
{
  uint32_t time_now = millis();
  if (time_now - g_bme680_last_sample_time > BME680_READING_INTERVAL)
  {
    bme680.performReading();
    g_bme680_last_sample_time = time_now;
  }
}

/**
  Update particulate matter sensor values
*/
void updatePmsReadings()
{
  uint32_t time_now = millis();

  // Check if we've been in the sleep state for long enough
  if (PMS_STATE_ASLEEP == g_pms_state)
  {
    if (time_now - g_pms_state_start
        >= ((g_pms_report_period * 1000) - (g_pms_warmup_period * 1000)))
    {
      // It's time to wake up the sensor
      //Serial.println("Waking up sensor");
      pms.wakeUp();
      g_pms_state_start = time_now;
      g_pms_state = PMS_STATE_WAKING_UP;
    }
  }

  // Check if we've been in the waking up state for long enough
  if (PMS_STATE_WAKING_UP == g_pms_state)
  {
    if (time_now - g_pms_state_start
        >= (g_pms_warmup_period * 1000))
    {
      g_pms_state_start = time_now;
      g_pms_state = PMS_STATE_READY;
    }
  }

  // Put the most recent values into globals for reference elsewhere
  if (PMS_STATE_READY == g_pms_state)
  {
    //pms.requestRead();
    if (pms.readUntil(g_data))
    {
      g_pm1p0_sp_value   = g_data.PM_SP_UG_1_0;
      g_pm2p5_sp_value   = g_data.PM_SP_UG_2_5;
      g_pm10p0_sp_value  = g_data.PM_SP_UG_10_0;

      g_pm1p0_ae_value   = g_data.PM_AE_UG_1_0;
      g_pm2p5_ae_value   = g_data.PM_AE_UG_2_5;
      g_pm10p0_ae_value  = g_data.PM_AE_UG_10_0;

      // This condition below should NOT be required, but currently I get all
      // 0 values for the PPD results every second time. This check only updates
      // the global values if there is a non-zero result for any of the values:
      if (g_data.PM_TOTALPARTICLES_0_3 + g_data.PM_TOTALPARTICLES_0_5
          + g_data.PM_TOTALPARTICLES_1_0 + g_data.PM_TOTALPARTICLES_2_5
          + g_data.PM_TOTALPARTICLES_5_0 + g_data.PM_TOTALPARTICLES_10_0
          != 0)
      {
        g_pm0p3_ppd_value  = g_data.PM_TOTALPARTICLES_0_3;
        g_pm0p5_ppd_value  = g_data.PM_TOTALPARTICLES_0_5;
        g_pm1p0_ppd_value  = g_data.PM_TOTALPARTICLES_1_0;
        g_pm2p5_ppd_value  = g_data.PM_TOTALPARTICLES_2_5;
        g_pm5p0_ppd_value  = g_data.PM_TOTALPARTICLES_5_0;
        g_pm10p0_ppd_value = g_data.PM_TOTALPARTICLES_10_0;
        g_pms_ppd_readings_taken = 1;
      }
      pms.sleep();

      // Report the new values
      reportToMqtt();
      reportToSerial();

      g_pms_readings_taken = 1;
      g_pms_state_start = time_now;
      g_pms_state = PMS_STATE_ASLEEP;
    }
  }
}

/**
  Report the most recent values to MQTT if enough time has passed
*/
void reportToMqtt()
{
  String message_string;

#if REPORT_MQTT_SEPARATE
  /* Report PM1.0 AE value */
  message_string = String(g_pm1p0_ae_value);
  message_string.toCharArray(g_mqtt_message_buffer, message_string.length() + 1);
  client.publish(g_pm1p0_ae_mqtt_topic, g_mqtt_message_buffer);

  /* Report PM2.5 AE value */
  message_string = String(g_pm2p5_ae_value);
  message_string.toCharArray(g_mqtt_message_buffer, message_string.length() + 1);
  client.publish(g_pm2p5_ae_mqtt_topic, g_mqtt_message_buffer);

  /* Report PM10.0 AE value */
  message_string = String(g_pm10p0_ae_value);
  message_string.toCharArray(g_mqtt_message_buffer, message_string.length() + 1);
  client.publish(g_pm10p0_ae_mqtt_topic, g_mqtt_message_buffer);

  if (1 == g_pms_ppd_readings_taken)
  {
    /* Report PM0.3 PPD value */
    message_string = String(g_pm0p3_ppd_value);
    message_string.toCharArray(g_mqtt_message_buffer, message_string.length() + 1);
    client.publish(g_pm0p3_ppd_mqtt_topic, g_mqtt_message_buffer);

    /* Report PM0.5 PPD value */
    message_string = String(g_pm0p5_ppd_value);
    message_string.toCharArray(g_mqtt_message_buffer, message_string.length() + 1);
    client.publish(g_pm0p5_ppd_mqtt_topic, g_mqtt_message_buffer);

    /* Report PM1.0 PPD value */
    message_string = String(g_pm1p0_ppd_value);
    message_string.toCharArray(g_mqtt_message_buffer, message_string.length() + 1);
    client.publish(g_pm1p0_ppd_mqtt_topic, g_mqtt_message_buffer);

    /* Report PM2.5 PPD value */
    message_string = String(g_pm2p5_ppd_value);
    message_string.toCharArray(g_mqtt_message_buffer, message_string.length() + 1);
    client.publish(g_pm2p5_ppd_mqtt_topic, g_mqtt_message_buffer);

    /* Report PM5.0 PPD value */
    message_string = String(g_pm5p0_ppd_value);
    message_string.toCharArray(g_mqtt_message_buffer, message_string.length() + 1);
    client.publish(g_pm5p0_ppd_mqtt_topic, g_mqtt_message_buffer);

    /* Report PM10.0 PPD value */
    message_string = String(g_pm10p0_ppd_value);
    message_string.toCharArray(g_mqtt_message_buffer, message_string.length() + 1);
    client.publish(g_pm10p0_ppd_mqtt_topic, g_mqtt_message_buffer);
  }
  /* Report temperature */
  message_string = String(bme680.temperature);
  message_string.toCharArray(g_mqtt_message_buffer, message_string.length() + 1);
  client.publish(g_temperature_mqtt_topic, g_mqtt_message_buffer);

  /* Report humidity */
  message_string = String(bme680.humidity);
  message_string.toCharArray(g_mqtt_message_buffer, message_string.length() + 1);
  client.publish(g_humidity_mqtt_topic, g_mqtt_message_buffer);

  /* Report pressure */
  message_string = String(bme680.pressure);
  message_string.toCharArray(g_mqtt_message_buffer, message_string.length() + 1);
  client.publish(g_pressure_mqtt_topic, g_mqtt_message_buffer);

  /* Report VOC */
  message_string = String(bme680.gas_resistance);
  message_string.toCharArray(g_mqtt_message_buffer, message_string.length() + 1);
  client.publish(g_voc_mqtt_topic, g_mqtt_message_buffer);

  /* Report altitude */
  message_string = String(bme680.readAltitude(SEA_LEVEL_PRESSURE_HPA));
  message_string.toCharArray(g_mqtt_message_buffer, message_string.length() + 1);
  client.publish(g_altitude_mqtt_topic, g_mqtt_message_buffer);
#endif
#if REPORT_MQTT_JSON
  /* Report all values combined into one JSON message */
  // Note: The PubSubClient library limits MQTT message size to 128 bytes, which is
  // set inside the library as MQTT_MAX_PACKET_SIZE. This prevents us reporting all
  // the values as a single JSON message, unless the library is edited. See:
  //  https://github.com/knolleary/pubsubclient/issues/110
  // The library has a method for sending large messages. Perhaps modify to use
  // this technique:
  //  https://github.com/knolleary/pubsubclient/blob/master/examples/mqtt_large_message/mqtt_large_message.ino

  // This is an example message generated by Tasmota, to match the format:
  // {"Time":"2020-02-27T03:27:22","PMS5003":{"CF1":0,"CF2.5":1,"CF10":1,"PM1":0,"PM2.5":1,"PM10":1,"PB0.3":0,"PB0.5":0,"PB1":0,"PB2.5":0,"PB5":0,"PB10":0}}
  // This is the source code from Tasmota:
  //ResponseAppend_P(PSTR(",\"PMS5003\":{\"CF1\":%d,\"CF2.5\":%d,\"CF10\":%d,\"PM1\":%d,\"PM2.5\":%d,\"PM10\":%d,\"PB0.3\":%d,\"PB0.5\":%d,\"PB1\":%d,\"PB2.5\":%d,\"PB5\":%d,\"PB10\":%d}"),
  //    pms_g_data.pm10_standard, pms_data.pm25_standard, pms_data.pm100_standard,
  //    pms_data.pm10_env, pms_data.pm25_env, pms_data.pm100_env,
  //    pms_data.particles_03um, pms_data.particles_05um, pms_data.particles_10um, pms_data.particles_25um, pms_data.particles_50um, pms_data.particles_100um);

  // This is the full message that I want to send, but it exceeds the message size limit:
  if (1 == g_pms_ppd_readings_taken)
  {
    sprintf(g_mqtt_message_buffer,  "{\"PMS5003\":{\"CF1\":%i,\"CF1\":%i,\"CF1\":%i,\"PM1\":%i,\"PM2.5\":%i,\"PM10\":%i,\"PB0.3\":%i,\"PB0.5\":%i,\"PB1\":%i,\"PB2.5\":%i,\"PB5\":%i,\"PB10\":%i}}",
            g_pm1p0_sp_value, g_pm2p5_sp_value, g_pm10p0_sp_value,
            g_pm1p0_ae_value, g_pm2p5_ae_value, g_pm10p0_ae_value,
            g_pm0p3_ppd_value, g_pm0p3_ppd_value, g_pm1p0_ppd_value,
            g_pm2p5_ppd_value, g_pm5p0_ppd_value, g_pm10p0_ppd_value);
  } else {
    sprintf(g_mqtt_message_buffer,  "{\"PMS5003\":{\"CF1\":%i,\"CF1\":%i,\"CF1\":%i,\"PM1\":%i,\"PM2.5\":%i,\"PM10\":%i}}",
            g_pm1p0_sp_value, g_pm2p5_sp_value, g_pm10p0_sp_value,
            g_pm1p0_ae_value, g_pm2p5_ae_value, g_pm10p0_ae_value);
  }

  // Reduced message to fit within the size limit:
  if (1 == g_pms_ppd_readings_taken)
  {
    sprintf(g_mqtt_message_buffer,  "{\"PMS5003\":{\"PM1\":%i,\"PM2.5\":%i,\"PM10\":%i,\"PB0.3\":%i,\"PB0.5\":%i,\"PB1\":%i,\"PB2.5\":%i,\"PB5\":%i,\"PB10\":%i}}",
            g_pm1p0_ae_value, g_pm2p5_ae_value, g_pm10p0_ae_value,
            g_pm0p3_ppd_value, g_pm0p3_ppd_value, g_pm1p0_ppd_value,
            g_pm2p5_ppd_value, g_pm5p0_ppd_value, g_pm10p0_ppd_value);
  } else {
    sprintf(g_mqtt_message_buffer,  "{\"PMS5003\":{\"PM1\":%i,\"PM2.5\":%i,\"PM10\":%i}}",
            g_pm1p0_ae_value, g_pm2p5_ae_value, g_pm10p0_ae_value);
  }
  client.publish(g_mqtt_json_topic, g_mqtt_message_buffer);
#endif
}

/**
  Report the latest values to the serial console
*/
void reportToSerial()
{
  /* Report PM1.0 AE value */
  Serial.print("PM1:");
  Serial.println(String(g_pm1p0_ae_value));

  /* Report PM2.5 AE value */
  Serial.print("PM2.5:");
  Serial.println(String(g_pm2p5_ae_value));

  /* Report PM10.0 AE value */
  Serial.print("PM10:");
  Serial.println(String(g_pm10p0_ae_value));

  if (1 == g_pms_ppd_readings_taken)
  {
    /* Report PM0.3 PPD value */
    Serial.print("PB0.3:");
    Serial.println(String(g_pm0p3_ppd_value));

    /* Report PM0.5 PPD value */
    Serial.print("PB0.5:");
    Serial.println(String(g_pm0p5_ppd_value));

    /* Report PM1.0 PPD value */
    Serial.print("PB1:");
    Serial.println(String(g_pm1p0_ppd_value));

    /* Report PM2.5 PPD value */
    Serial.print("PB2.5:");
    Serial.println(String(g_pm2p5_ppd_value));

    /* Report PM5.0 PPD value */
    Serial.print("PB5:");
    Serial.println(String(g_pm5p0_ppd_value));

    /* Report PM10.0 PPD value */
    Serial.print("PB10:");
    Serial.println(String(g_pm10p0_ppd_value));
  }
  
  /* Report temperature */
  Serial.print("TEMP:");
  Serial.println(String(bme680.temperature));

  /* Report humidity */
  Serial.print("RHEL:");
  Serial.println(String(bme680.humidity));

  /* Report pressure */
  Serial.print("BARO:");
  Serial.println(String(bme680.pressure));

  /* Report VOC */
  Serial.print("VOC:");
  Serial.println(String(bme680.gas_resistance));

  /* Report altitude */
  Serial.print("ALT:");
  Serial.println(String(bme680.readAltitude(SEA_LEVEL_PRESSURE_HPA)));
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
  sprintf(mqtt_client_id, "esp32-%x", g_device_id);

  // Loop until we're reconnected
  while (!client.connected()) {
    //Serial.print("Attempting MQTT connection...");
    // Attempt to connect
    if (client.connect(mqtt_client_id)) {
      //Serial.println("connected");
      // Once connected, publish an announcement
      sprintf(g_mqtt_message_buffer, "Device %s starting up", mqtt_client_id);
      client.publish(status_topic, g_mqtt_message_buffer);
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
   Set all LEDs to the specified colour
*/
void setColour(uint32_t c) {
  for (uint8_t i = 0; i < strip.numPixels(); i++) {
    strip.setPixelColor(i, c);
  }
  strip.show();
}
