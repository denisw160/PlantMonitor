#include <Wire.h>
#include <SPI.h>
#include <ESP8266WiFi.h>
#include <PubSubClient.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <Arduino.h>
#include <U8g2lib.h>

#include "plantsensor_icons.h"
#include "plantsensor_settings.h"


// PlantSensor for the PlantMonitor
// Used ESP8266 with 0.91" OLED Display (128x32)
// Use CPU frequency 160MHz and Flash Size 4M (3M SPIFFS)
//
// For MQTT change PubSubClient.h MQTT_MAX_PACKET_SIZE to 256!
//
// *** Wireing plan ***
// Capacitive Soil Moisture Sensor -> ESP8266
// VCC -> +5V
// GND -> GND
// A0 -> A0
//
// BME280 -> ESP8266
// VCC -> +3.3V
// GND -> GND
// SDA -> D2
// SCL -> D1

// TODO
//  - Add SSL Support
//  - Improve configuration via WiFiManager


// U8g2 Contructor List (Frame Buffer)
// The complete list is available here: https://github.com/olikraus/u8g2/wiki/u8g2setupcpp
U8G2_SSD1306_128X32_UNIVISION_F_HW_I2C u8g2(U8G2_R0, /* reset=*/ U8X8_PIN_NONE);  // Adafruit ESP8266/32u4/ARM Boards + FeatherWing OLED

// Assign the Moisture Sensor to pins
#define MOISTURE_ANALOG A0

// Parameter for Moisture Sensor (max. 1023)
const float MOISTURE_MAX = 833.0;

// Assign the BME to pins
#define BME_SDA D2
#define BME_SCL D1

// Parameter for BME Sensor
const float TEMPERATURE_OFFSET = 0.86;
const float HUMIDITY_OFFSET = -0.91;
const float PRESSURE_OFFSET = 0.0;

// BME Sensor
Adafruit_BME280 bme; // I2C

// Page selection
int page = 0;

// Sensor values
int mV;
float m, t, tV, h, hV, p, pV;

// MQTT client
WiFiClient wiFiClient;
PubSubClient mqttClient(wiFiClient);

// MQTT values
String mqttId = "PlantSensor";

void setup() {
  // Initalize serial communicaton at 115200 bits per second
  Serial.begin(115200);
  delay(10);

  // Initalize oled display
  u8g2.begin();
  u8g2.enableUTF8Print();

  // Show startup message
  u8g2.clearBuffer();                 // clear the internal memory
  u8g2.setFont(u8g2_font_crox4hb_tf); // choose a suitable font
  u8g2.drawStr(0, 23, "Booting...");  // write something to the internal memory
  u8g2.sendBuffer();                  // transfer internal memory to the display
  delay(1000);

  // Initalize bme pins
  Wire.begin(BME_SDA, BME_SCL);
  Wire.setClock(100000);
  Serial.println("Starting BME280 sensor ...");
  if (!bme.begin(0x76)) {
    Serial.println("Could not find a valid BME280 sensor, check wiring!");
    while (1);
  }

  // Connecting to WiFi
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  Serial.print("Connecting to WiFi...");

  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_crox4hb_tf);
  u8g2.drawStr(0, 23, "Connecting...");
  u8g2.sendBuffer();
  int c = 0;
  while (WiFi.status() != WL_CONNECTED && c < 50)
  {
    delay(500);
    Serial.print(".");
    c = c + 1;
  }
  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("done!");
    Serial.print("Hostname: ");
    Serial.println(WiFi.hostname());
    Serial.print("WiFi IP-Adress: ");
    Serial.println(WiFi.localIP());

    // Setup MQTT
    mqttId = mqttId + "_" + WiFi.hostname();
    mqttClient.setServer(MQTT_SERVER, MQTT_PORT);
  }
  else {
    Serial.println("failed!");
  }
}

void loop() {
  // Read and updates the data from the sensors
  readSensors();

  // Show one of the sensor values on the display
  Serial.print("Show next page: ");
  Serial.println(page);
  u8g2.firstPage();
  do {
    if (page == 0) {
      showPageMoisture();
      page = 1;
    } else if (page == 1) {
      showPageTemperature();
      page = 2;
    } else if (page == 2) {
      showPageHumidity();
      page = 3;
    } else {
      showPagePressure();
      page = 0;
    }
  } while ( u8g2.nextPage() );

  // Send the sensor data to the server
  Serial.print("WiFi connection status: ");
  Serial.println(WiFi.status());
  if (WiFi.status() == WL_CONNECTED) {
    sendSensorData();
  }

  //delay(30000);   // wait 30 sec (production)
  delay(5000);   // wait 5 sec (for testing)
}

void readSensors() {
  // Moisture Sensor
  // Read the input on analog pin 0
  mV = analogRead(MOISTURE_ANALOG);
  m = max(100.0 - (mV / MOISTURE_MAX) * 100.0, 0.0);
  Serial.print("Moisture: ");
  Serial.println(mV);
  Serial.print("Moisture (%): ");
  Serial.println(m);

  // BME280
  tV = bme.readTemperature();
  t = tV - TEMPERATURE_OFFSET;
  Serial.print("Temperature: ");
  Serial.println(tV);
  Serial.print("Temperature (offset): ");
  Serial.println(t);
  hV = bme.readHumidity();
  h = hV - HUMIDITY_OFFSET;
  Serial.print("Humidity: ");
  Serial.println(hV);
  Serial.print("Humidity (offeset): ");
  Serial.println(h);
  pV = bme.readPressure() / 100.F;
  p = pV - PRESSURE_OFFSET;
  Serial.print("Pressure: ");
  Serial.println(pV);
  Serial.print("Pressure (offset): ");
  Serial.println(p);
}

void sendSensorData() {
  // Connect or reconnect to broker
  if (!mqttClient.connected()) {
    Serial.print("Connecting to MQTT...");
    if (mqttClient.connect((char*)mqttId.c_str(), MQTT_USER, MQTT_PASSWORD)) {
      Serial.println("connected");
    } else {
      Serial.print("failed with state ");
      Serial.println(mqttClient.state());
      return;
    }
  }

  // Sending MQTT message
  // Original structure: {"temperature": 25.89, "humidity": 42.25, "pressure": 1004.061, "moisture": 35.65, "moisture_raw": 658}
  const String message = "{\"temperature\": " + String(t)
                         + ", \"humidity\": " + String(h)
                         + ", \"pressure\": " + String(p)
                         + ", \"moisture\": " + String(m)
                         + ", \"moisture_raw\": " + String(mV)
                         + "}";
  const char *buf = (char*)message.c_str();
  mqttClient.publish(MQTT_TOPIC, buf);
  Serial.print("Message send: ");
  Serial.println(message);
  Serial.print("Max size: ");
  Serial.println(MQTT_MAX_PACKET_SIZE);
}

void showPageMoisture() {
  // Show Icon
  u8g2.drawXBMP(
    0,
    (u8g2.getDisplayHeight() - ICON_HEIGHT) / 2,
    ICON_WIDTH,
    ICON_HEIGHT,
    MOISTURE_ICON);

  // Show Text
  char buf[128];
  char value[10];
  u8g2.setFont(u8g2_font_helvB12_tf);
  strcpy(buf, "");
  dtostrf(m, 6, 2, value);
  strcat(buf,  value);
  strcat(buf, " %");
  u8g2.drawStr(32, 22, buf);
}

void showPageTemperature() {
  // Show Icon
  u8g2.drawXBMP(
    0,
    (u8g2.getDisplayHeight() - ICON_HEIGHT) / 2,
    ICON_WIDTH,
    ICON_HEIGHT,
    TEMPERATURE_ICON);

  // Show Text
  char buf[128];
  char value[10];
  u8g2.setFont(u8g2_font_helvB12_tf);
  strcpy(buf, "");
  dtostrf(t, 6, 2, value);
  strcat(buf,  value);
  strcat(buf, " °C");
  u8g2.drawUTF8(32, 22, buf);
}

void showPageHumidity() {
  // Show Icon
  u8g2.drawXBMP(
    0,
    (u8g2.getDisplayHeight() - ICON_HEIGHT) / 2,
    ICON_WIDTH,
    ICON_HEIGHT,
    HUMIDITY_ICON);

  // Show Text
  char buf[128];
  char value[10];
  u8g2.setFont(u8g2_font_helvB12_tf);
  strcpy(buf, "");
  dtostrf(h, 6, 2, value);
  strcat(buf,  value);
  strcat(buf, " %");
  u8g2.drawStr(32, 22, buf);
}

void showPagePressure() {
  // Show Icon
  u8g2.drawXBMP(
    0,
    (u8g2.getDisplayHeight() - ICON_HEIGHT) / 2,
    ICON_WIDTH,
    ICON_HEIGHT,
    PRESSURE_ICON);

  // Show Text
  char buf[128];
  char value[10];
  u8g2.setFont(u8g2_font_helvB12_tf);
  strcpy(buf, "");
  dtostrf(p, 6, 1, value);
  strcat(buf,  value);
  strcat(buf, " hPa");
  u8g2.drawStr(32, 22, buf);
}
