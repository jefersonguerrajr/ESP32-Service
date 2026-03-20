#ifndef LED_CONTROLLER_HPP
#define LED_CONTROLLER_HPP

#include <Arduino.h>
#include <WebServer.h>

class LedController {
public:
    explicit LedController(uint8_t pin);
    void registerRoutes(WebServer& server);

private:
    uint8_t _pin;

    void handleGetStatus(WebServer& server);
    void handleSetState(WebServer& server);
    void handleToggle(WebServer& server);

    bool isOn() const;
    void sendStateResponse(WebServer& server);
};

#endif // LED_CONTROLLER_HPP
