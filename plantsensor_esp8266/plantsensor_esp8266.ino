#include <Wire.h>
#include <SPI.h>
#include <ESP8266WiFi.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <Arduino.h>
#include <U8g2lib.h>

#include "plantsensor_icons.h"
#include "plantsensor_settings.h"


// PlantSensor for the PlantMonitor
// Used ESP8266 with 0.91" OLED Display (128x32)
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
// - Improve configuration via WiFiManager


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

void setup() {
  // Initalize serial communicaton at 115200 bits per second
  Serial.begin(115200);
  delay(10);

  // Initalize oled display
  u8g2.begin();
  u8g2.enableUTF8Print();
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

  u8g2.clearDisplay();
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
    Serial.print("WiFi IP-Adress: ");
    Serial.println(WiFi.localIP());
  }
  else {
    Serial.println("failed!");
  }
}

void loop() {
  readSensors();

  Serial.print("Show next page: ");
  Serial.println(page);
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

  Serial.print("WiFi connection status: ");
  Serial.println(WiFi.status());
  if (WiFi.status() == WL_CONNECTED) {
    sendSensorData();
  }

  delay(30000);   // wait 30 sec (production)
  //delay(1000);   // wait 1 sec (for testing)
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

}

void showPageMoisture() {
  // Clear display
  u8g2.clearDisplay();

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

  // Update display
  u8g2.sendBuffer();
}

void showPageTemperature() {
  // Clear display
  u8g2.clearDisplay();

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

  // Update display
  u8g2.sendBuffer();
}

void showPageHumidity() {
  // Clear display
  u8g2.clearDisplay();

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

  // Update display
  u8g2.sendBuffer();
}

void showPagePressure() {
  // Clear display
  u8g2.clearDisplay();

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

  // Update display
  u8g2.sendBuffer();
}
