#include <Wire.h>
#include <SPI.h>
#include <ESP8266WiFi.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <Arduino.h>
#include <U8g2lib.h>


// PlantSensor for the PlantMonitor
// Used ESP8266 with 0.91" OLED Display
//
// *** Wireing plan ***
// Hygrometer -> ESP8266
// VCC -> +5V
// GND -> GND
// A0 -> A0
//
// BME280 -> ESP8266
// VCC -> +3.3V
// GND -> GND
// SDA -> D2
// SCL -> D1

// U8g2 Contructor List (Frame Buffer)
// The complete list is available here: https://github.com/olikraus/u8g2/wiki/u8g2setupcpp
U8G2_SSD1306_128X32_UNIVISION_F_HW_I2C u8g2(U8G2_R0, /* reset=*/ U8X8_PIN_NONE);  // Adafruit ESP8266/32u4/ARM Boards + FeatherWing OLED

// assign the hygrometer to pins
#define HYGROMETER_ANALOG A0

// assign the bme to pins
#define BME_SDA D2
#define BME_SCL D1


Adafruit_BME280 bme; // I2C

void setup() {
  // Initalize serial communicaton at 115200 bits per second:
  Serial.begin(115200);
  delay(10);

  // Initalize oled display
  u8g2.begin();
  u8g2.clearBuffer();                  // clear the internal memory
  u8g2.setFont(u8g2_font_ncenB08_tr);  // choose a suitable font
  u8g2.drawStr(0, 10, "Booting...");   // write something to the internal memory
  u8g2.sendBuffer();                   // transfer internal memory to the display
  delay(1000);

  // Initalize bme pins
  Wire.begin(BME_SDA, BME_SCL);
  Wire.setClock(100000);
  Serial.println("Starting BME280 sensor ...");
  if (!bme.begin(0x76)) {
    Serial.println("Could not find a valid BME280 sensor, check wiring!");
    while (1);
  }
}

void loop() {
  // Hygrometer
  // Read the input on analog pin 0
  int mV = analogRead(HYGROMETER_ANALOG);
  float m = (100.0 - (mV / 1024.0) * 100.0);
  Serial.print("Moisture: ");
  Serial.println(m);

  // BME280
  float h, t, p;
  h = bme.readHumidity();
  t = bme.readTemperature();
  p = bme.readPressure() / 100.F;

  Serial.print("Temperature: ");
  Serial.println(t);
  Serial.print("Humidity: ");
  Serial.println(h);
  Serial.print("Pressure: ");
  Serial.println(p);

  // Output to display
  char buf[128];
  char value[10];
  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_ncenB08_tr);

  strcpy(buf, "Moisture: ");
  dtostrf(m, 6, 2, value);
  strcat(buf,  value);
  strcat(buf, "%");
  u8g2.drawStr(0, 10, buf);

  strcpy(buf, "Temperature: ");
  dtostrf(t, 6, 2, value);
  strcat(buf,  value);
  strcat(buf, "°C");
  u8g2.drawStr(0, 20, buf);

  strcpy(buf, "H:");
  dtostrf(h, 6, 2, value);
  strcat(buf,  value);
  strcat(buf, "%/P:");
  dtostrf(p, 6, 2, value);
  strcat(buf,  value);
  strcat(buf, "hPa");
  u8g2.drawStr(0, 30, buf);
  u8g2.sendBuffer();

  delay(30000);   // wait 30 sec
}
