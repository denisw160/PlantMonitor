#include <Wire.h>
#include <ESP8266WiFi.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>


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
// SDA -> D6
// SCL -> D7


// assign the hygrometer to pins
#define HYGROMETER_ANALOG A0

// assign the bme to pins
#define BME_SDA D6
#define BME_SCL D7


Adafruit_BME280 bme; // I2C

void setup() {
  // Initalize serial communicaton at 115200 bits per second:
  Serial.begin(115200);
  delay(10);

  // Initialize bme pins
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
  h = bme.readHumidity(); // d
  t = bme.readTemperature();
  p = bme.readPressure() / 100.F;

  Serial.print("Temperature: ");
  Serial.println(t);
  Serial.print("Humidity: ");
  Serial.println(h);
  Serial.print("Pressure: ");
  Serial.println(p);

  delay(1000); // delay in between reads for stability
  // TODO increse delay min. 10 minutes
}
