#ifndef WIFIMANAGER_HPP
#define WIFIMANAGER_HPP

#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include <ArduinoJson.h>
#include <Preferences.h>

#define LED_PIN 2
#define RESET_BUTTON 0  // GPIO0 (BOOT button)

class WiFiManager {
public:
    WiFiManager();
    void begin();
    void handleClient();
    void checkResetButton();
    WebServer& getServer() { return server; }

private:
    WebServer server;
    Preferences preferences;
    bool isAPMode;
    unsigned long resetButtonPressTime;

    // Handler functions
    void handleRoot();
    void handleSave();
    void handleReset();

    // Mode functions
    void startAPMode();
    void startStationMode(String ssid, String password);

    // Static wrapper functions for server callbacks
    static void handleRootWrapper();
    static void handleSaveWrapper();
    static void handleResetWrapper();

    // Static instance pointer for callbacks
    static WiFiManager* instance;
};

#endif // WIFIMANAGER_HPP
