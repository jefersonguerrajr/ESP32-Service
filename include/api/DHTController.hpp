#ifndef DHT_CONTROLLER_HPP
#define DHT_CONTROLLER_HPP

#include <Arduino.h>
#include <WebServer.h>
#include <Adafruit_Sensor.h>
#include <DHT_U.h>

class DHTController {
public:
    DHTController(uint8_t pin, uint8_t type);
    void registerRoutes(WebServer& server);

private:
    DHT_Unified _dht;
    void handleGetSensors(WebServer& server);
};

#endif // DHT_CONTROLLER_HPP
