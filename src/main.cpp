#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include <api/LedController.hpp>
#include <api/DHTController.hpp>
#include <WiFiManager.hpp>

#define DHT_PIN  4
#define DHT_TYPE DHT22

WiFiManager    wifiManager;
LedController  ledController(LED_PIN);
DHTController  dhtController(DHT_PIN, DHT_TYPE);

void setup() {
  Serial.begin(115200);
  wifiManager.begin();
  ledController.registerRoutes(wifiManager.getServer());
  dhtController.registerRoutes(wifiManager.getServer());
}

void loop() {
  wifiManager.handleClient();
  wifiManager.checkResetButton();
}
