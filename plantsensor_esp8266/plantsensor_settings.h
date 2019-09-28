//
// This file contains the settings for your connection.
// Please update to your environment.
//

// Set Wi-Fi SSID and password
const char *WIFI_SSID     = "your-wifi";
const char *WIFI_PASSWORD = "your-password";

// Set your MQTT parameters / MQTT_USER = "", then no login
#define     MQTT_SSL          false
const char *MQTT_SERVER    = "your mqtt server";
const int   MQTT_PORT      = 1883;
const char *MQTT_USER      = "";
const char *MQTT_PASSWORD  = "";
const char *MQTT_TOPIC     = "me/wirries/plantsensor";
const char *MQTT_FPRINT    = "36:6C:A5:A4:A4:3C:CC:56:C4:08:4E:D0:58:A6:9A:A0:44:18:9B:BE";
