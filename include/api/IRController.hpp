#ifndef IR_CONTROLLER_HPP
#define IR_CONTROLLER_HPP

#include <Arduino.h>
#include <ArduinoJson.h>
#include <WebServer.h>

class IRController {
public:
    IRController(uint8_t receivePin, uint8_t sendPin);
    void begin();
    void registerRoutes(WebServer& server);

private:
    static constexpr uint8_t MAX_FRAMES = 12;
    static constexpr uint16_t MAX_RAW_LEN = 750;
    static constexpr uint32_t CAPTURE_TIMEOUT_MS = 10000;
    static constexpr uint32_t SESSION_END_GAP_MS = 500;
    

    struct IRFrame {
        uint16_t raw[MAX_RAW_LEN];
        uint16_t rawLen;
        uint32_t gapBefore;
        uint8_t kHz;
        uint8_t protocolId;
        uint16_t address;
        uint16_t command;
        uint16_t extra;
        uint16_t numberOfBits;
        uint8_t flags;
        bool hasDecodedFields;
        char protocolName[24];
        char decodedRawDataHex[24];
    };

    uint8_t _receivePin;
    uint8_t _sendPin;
    IRFrame _session[MAX_FRAMES];
    uint8_t _sessionLen;
    bool _hasSession;

    void clearSession();
    bool captureFrame();
    bool waitForSession();
    bool loadSessionFromJson(JsonDocument& doc, String& errorMessage);
    void populateFrameDebugData(IRFrame& frame);
    void transmitSession();

    void handleReceive(WebServer& server);
    void handleSend(WebServer& server);
    void sendSessionResponse(WebServer& server);
};

#endif // IR_CONTROLLER_HPP