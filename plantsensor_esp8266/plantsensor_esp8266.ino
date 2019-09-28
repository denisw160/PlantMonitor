/**************************************************************************

  PlantSensor ESP8266 NodeMCU with Capacitive Soil Moisture Sensor, BME280
  and BH1750.

  ** Important **
   - For MQTT change PubSubClient.h MQTT_MAX_PACKET_SIZE to 256!

  ** Wireing plan **
    Capacitive Soil Moisture Sensor -> ESP8266
      VCC     -> +5V
      GND     -> GND
      A0      -> A0

    BME280   -> ESP8266
      VCC     -> +3.3V
      GND     -> GND
      SDA     -> D2
      SCL     -> D1

    BH1750   -> ESP8266
      VCC     -> 3V3
      GND     -> GND
      SDA     -> D2
      SCL     -> D1
      ADDR    -> RX

   ** TODO **
      - Add SSL Support
      - Improve configuration via WiFiManager and remove settings

 **************************************************************************/


// Common
#include <ESP8266WiFi.h>

// MQTT
#include <PubSubClient.h>

// Sensors
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <BH1750FVI.h>
//#include <Arduino.h>

#include "plantsensor_settings.h"


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

// BH1750 Sensor
uint8_t ADDRESSPIN = 13;
BH1750FVI::eDeviceAddress_t DEVICEADDRESS = BH1750FVI::k_DevAddress_H;
BH1750FVI::eDeviceMode_t DEVICEMODE = BH1750FVI::k_DevModeContHighRes;
BH1750FVI lightSensor(ADDRESSPIN, DEVICEADDRESS, DEVICEMODE);

// Sensor values
int mV;
uint16_t l;
float m, t, tV, h, hV, p, pV;

// MQTT client
#if MQTT_SSL
BearSSL::WiFiClientSecure wiFiClient; // for encrypted with SSL
#else
WiFiClient wiFiClient; // for unencrypted
#endif
PubSubClient mqttClient(wiFiClient);

// MQTT values
String mqttId = "PlantSensor";

// Generally, you should use "unsigned long" for variables that hold time
// The value will quickly become too large for an int to store
unsigned long previousMillis = 0;        // will store last time a message was send

// constants won't change:
const long interval = 30000;           // interval for sending a message (milliseconds)


void setup() {
  // Initalize serial communicaton at 115200 bits per second
  Serial.begin(115200);
  delay(10);

  // Initalize bme280
  Wire.begin(BME_SDA, BME_SCL);
  Wire.setClock(100000);
  Serial.print("Starting BME280 sensor ...");
  if (!bme.begin(0x76)) {
    Serial.println("Could not find a valid BME280 sensor, check wiring!");
    while (1);
  } else {
    Serial.println("ready!");
  }

  // Initialize bh1750
  Serial.print("Starting BH1750 sensor ...");
  lightSensor.begin();
  Serial.println("ready!");

  // Connecting to WiFi
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  Serial.print("Connecting to WiFi...");
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
  }
  else {
    Serial.println("failed!");
  }

  // Setup MQTT
  mqttId = mqttId + "_" + WiFi.hostname();
  mqttClient.setServer(MQTT_SERVER, MQTT_PORT);
}

void loop() {
  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis >= interval) {
    previousMillis = currentMillis;

    // Read and updates the data from the sensors
    readSensors();

    // Send the sensor data to the server
    Serial.print("WiFi connection status: ");
    Serial.println(WiFi.status());
    if (WiFi.status() == WL_CONNECTED) {
#if MQTT_SSL
      verifyConnection(); // only for encrypted with SSL
#endif
      sendSensorData();
    }
  }
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

  // BH1750
  l = lightSensor.GetLightIntensity();
  Serial.print("Light = ");
  Serial.print(l);
  Serial.println(" lux");
  Serial.println();
}

#if MQTT_SSL
void verifyConnection() {
  if (mqttClient.connected() || wiFiClient.connected()) return; // Already connected

  Serial.print("Checking SSL@");
  Serial.print(MQTT_SERVER);
  Serial.print("...");

  wiFiClient.setInsecure();
  if (!wiFiClient.connect(MQTT_SERVER, MQTT_PORT)) {
    Serial.println("failed.");
    return;
  } else {
    Serial.println("ok.");
  }
  if (wiFiClient.verify(MQTT_FPRINT, MQTT_SERVER)) {
    Serial.println("Connection is secure.");
  } else {
    Serial.println("Connection insecure! Rebooting.");
    Serial.flush();
    ESP.restart();
  }

  wiFiClient.stop();
  delay(100);
}
#endif

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
  // Original structure: {"temperature": 25.89, "humidity": 42.25, "pressure": 1004.061, "moisture": 35.65, "moisture_raw": 658, "light": 152}
  const String message = "{\"temperature\": " + String(t)
                         + ", \"humidity\": " + String(h)
                         + ", \"pressure\": " + String(p)
                         + ", \"moisture\": " + String(m)
                         + ", \"moisture_raw\": " + String(mV)
                         + ", \"light\": " + String(l)
                         + "}";
  const char *buf = (char*)message.c_str();
  mqttClient.publish(MQTT_TOPIC, buf);
  Serial.print("Message send: ");
  Serial.println(message);
  Serial.print("Max size: ");
  Serial.println(MQTT_MAX_PACKET_SIZE);
  if (MQTT_MAX_PACKET_SIZE < 256) {
    Serial.println("ERROR: package size too small, check PubSubClient.h for this setting");
  }
}
