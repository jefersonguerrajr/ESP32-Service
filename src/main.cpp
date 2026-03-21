#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include <api/LedController.hpp>
#include <api/DHTController.hpp>
#include <api/IRController.hpp>
#include <WiFiManager.hpp>

#define DHT_PIN  4
#define DHT_TYPE DHT22
#define IR_RECEIVE_PIN 15
#define IR_SEND_PIN 16

WiFiManager    wifiManager;
LedController  ledController(LED_PIN);
DHTController  dhtController(DHT_PIN, DHT_TYPE);
IRController   irController(IR_RECEIVE_PIN, IR_SEND_PIN);

void setup() {
  Serial.begin(115200);
  wifiManager.begin();
  irController.begin();
  ledController.registerRoutes(wifiManager.getServer());
  dhtController.registerRoutes(wifiManager.getServer());
  irController.registerRoutes(wifiManager.getServer());
}

void loop() {
  wifiManager.handleClient();
  wifiManager.checkResetButton();
}
