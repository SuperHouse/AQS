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
#include <Adafruit_GFX.h>      // Required for OLED
#include <Adafruit_SSD1306.h>  // Required for OLED
#include <ESP8266WiFi.h>       // ESP8266 WiFi driver
#include "PMS.h"               // Particulate Matter Sensor driver
#include <PubSubClient.h>      // Required for MQTT

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
#define WIFI_CONNECT_INTERVAL 500      // Wait 500ms intervals for wifi connection
#define WIFI_CONNECT_MAX_ATTEMPTS 10   // Number of attempts/intervals to wait

/*--------------------------- Instantiate Global Objects --------------------*/
// Particulate matter sensor
PMS pms(Serial);
PMS::DATA data;

// OLED
Adafruit_SSD1306 OLED(0);  // GPIO0 = OLED reset pin

// MQTT
WiFiClient esp_client;
PubSubClient client(esp_client);

/*--------------------------- Program ---------------------------------------*/
/*
   This callback is invoked when an MQTT message is received. It's not important
   right now for this project because we don't receive commands via MQTT. You
   can modify this function to make the device act on commands that you send it.
*/
void callback(char* topic, byte* payload, uint8_t length) {
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
  // We need a unique device ID for our MQTT client connection
  g_device_id = String(ESP.getChipId(), HEX);  // Get the unique ID of the ESP8266 chip in hex
  //Serial.print("Device ID: ");
  //Serial.println(g_device_id);

  // Set up the topics for publishing sensor readings. By inserting the unique ID,
  // the result is of the form: "device/d9616f/pm1" etc
  sprintf(g_pm1_mqtt_topic,      "device/%x/pm1",     ESP.getChipId());  // From PMS
  sprintf(g_pm2p5_mqtt_topic,    "device/%x/pm2p5",   ESP.getChipId());  // From PMS
  sprintf(g_pm10_mqtt_topic,     "device/%x/pm10",    ESP.getChipId());  // From PMS
  sprintf(g_pm1_raw_mqtt_topic, "device/%x/pm1raw", ESP.getChipId());  // From PMS
  sprintf(g_pm2p5_raw_mqtt_topic, "device/%x/pm2p5raw", ESP.getChipId());  // From PMS
  sprintf(g_pm10_raw_mqtt_topic, "device/%x/pm10raw", ESP.getChipId());  // From PMS
  sprintf(g_command_topic,       "device/%x/command", ESP.getChipId());  // For receiving messages

  // Set up display
  OLED.begin();
  OLED.clearDisplay();
  OLED.setTextWrap(false);
  OLED.setTextSize(1);
  OLED.setTextColor(WHITE);
  OLED.setCursor(0, 0);
  OLED.println("Booting...");
  OLED.println(g_device_id);
  OLED.display();

  // Connect to WiFi
  if (initWifi()) {
    OLED.println("Wifi [CONNECTED]");
  } else {
    OLED.println("Wifi [FAILED]");
  }
  OLED.display();
  delay(1000);

  Serial.begin(9600);   // GPIO1, GPIO3 (TX/RX pin on ESP-12E Development Board)

  pinMode( mode_button_pin , INPUT_PULLUP ); // Pin for switching screens button

  /* Set up the MQTT client */
  client.setServer(mqtt_broker, 1883);
  client.setCallback(callback);
}

/**
   Main loop
*/
void loop() {
  if (WiFi.status() == WL_CONNECTED)
  {
    if (!client.connected()) {
      reconnectMqtt();
    }
  }
  client.loop();  // Process any outstanding MQTT messages

  toggleDisplay();

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

    OLED.clearDisplay();
    OLED.setCursor(0, 0);

    // Render our displays
    switch (g_display_state) {
      case STATE_GRAMS:
        OLED.setTextWrap(false);
        OLED.println("ug/m^3 (Atmos.)");
        OLED.print("PM 1.0 (ug/m3): ");
        OLED.println(data.PM_AE_UG_1_0);

        OLED.print("PM 2.5 (ug/m3): ");
        OLED.println(data.PM_AE_UG_2_5);

        OLED.print("PM 10.0 (ug/m3): ");
        OLED.println(data.PM_AE_UG_10_0);
        break;

      case STATE_INFO:
        char mqtt_client_id[20];
        sprintf(mqtt_client_id, "esp8266-%x", ESP.getChipId());
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
        break;

      /* Fallback helps with debugging if you call a state that isn't defined */
      default:
        OLED.println(g_display_state);
        break;
    }

    OLED.display();
    delay(0.25);
  }

  uint32_t time_now = millis();
  if (time_now - g_last_report_time > (report_interval * 1000))
  {
    g_last_report_time = time_now;
    // Generate averages for the samples in the ring buffers

    uint8_t i;
    uint32_t pm1_average_value   = 0;
    uint32_t pm2p5_average_value = 0;
    uint32_t pm10_average_value  = 0;

    String message_string;

    for (i = 0; i < sizeof(g_pm1_ring_buffer) / sizeof( g_pm1_ring_buffer[0]); i++)
    {
      pm1_average_value += g_pm1_ring_buffer[i];
    }
    pm1_average_value = (int)((pm1_average_value / SAMPLE_COUNT) + 0.5);

    for (i = 0; i < sizeof(g_pm2p5_ring_buffer) / sizeof( g_pm2p5_ring_buffer[0]); i++)
    {
      pm2p5_average_value += g_pm2p5_ring_buffer[i];
    }
    pm2p5_average_value = (int)((pm2p5_average_value / SAMPLE_COUNT) + 0.5);

    for (i = 0; i < sizeof(g_pm10_ring_buffer) / sizeof( g_pm10_ring_buffer[0]); i++)
    {
      pm10_average_value += g_pm10_ring_buffer[i];
    }
    pm10_average_value = (int)((pm10_average_value / SAMPLE_COUNT) + 0.5);

    /* Report PM1 value */
    message_string = String(pm1_average_value);
    //Serial.println(message_string);
    message_string.toCharArray(g_mqtt_message_buffer, message_string.length() + 1);
    client.publish(g_pm1_mqtt_topic, g_mqtt_message_buffer);

    /* Report PM2.5 value */
    message_string = String(pm2p5_average_value);
    //Serial.println(message_string);
    message_string.toCharArray(g_mqtt_message_buffer, message_string.length() + 1);
    client.publish(g_pm2p5_mqtt_topic, g_mqtt_message_buffer);

    /* Report PM10 value */
    message_string = String(pm10_average_value);
    //Serial.println(message_string);
    message_string.toCharArray(g_mqtt_message_buffer, message_string.length() + 1);
    client.publish(g_pm10_mqtt_topic, g_mqtt_message_buffer);

    /* Report raw PM1 value for comparison with averaged value */
    message_string = String(g_pm1_latest_value);
    //Serial.println(message_string);
    message_string.toCharArray(g_mqtt_message_buffer, message_string.length() + 1);
    client.publish(g_pm1_raw_mqtt_topic, g_mqtt_message_buffer);

    /* Report raw PM2.5 value for comparison with averaged value */
    message_string = String(g_pm2p5_latest_value);
    //Serial.println(message_string);
    message_string.toCharArray(g_mqtt_message_buffer, message_string.length() + 1);
    client.publish(g_pm2p5_raw_mqtt_topic, g_mqtt_message_buffer);

    /* Report raw PM10 value for comparison with averaged value */
    message_string = String(g_pm10_latest_value);
    //Serial.println(message_string);
    message_string.toCharArray(g_mqtt_message_buffer, message_string.length() + 1);
    client.publish(g_pm10_raw_mqtt_topic, g_mqtt_message_buffer);
  }
}

/**
  Read the display mode button and switch the display mode if necessary
*/
void toggleDisplay() {

  g_previous_mode_button_state = g_current_mode_button_state;
  g_current_mode_button_state = digitalRead(mode_button_pin);

  // Check if button is now pressed and it was previously unpressed
  if (g_current_mode_button_state == LOW && g_previous_mode_button_state == HIGH)
  {
    // We haven't waited long enough so ignore this press
    if (millis() - g_last_debounce_time <= g_debounce_delay) {
      return;
    }

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
void reconnectMqtt() {
  char mqtt_client_id[20];
  sprintf(mqtt_client_id, "esp8266-%x", ESP.getChipId());

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
