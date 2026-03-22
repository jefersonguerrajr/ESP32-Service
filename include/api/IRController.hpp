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
    static constexpr uint8_t DEFAULT_KHZ = 38;
    static constexpr uint32_t DEFAULT_INTER_FRAME_GAP_MICROS = 40000;

    struct IRFrame {
        uint16_t raw[MAX_RAW_LEN];
        uint16_t rawLen;
        uint32_t gapBefore;
        uint8_t kHz;
    };

    uint8_t _receivePin;
    uint8_t _sendPin;
    IRFrame _session[MAX_FRAMES];
    uint8_t _sessionLen;
    bool _hasSession;
    uint32_t _lastFrameStartedAtMicros;
    uint32_t _lastFrameDurationMicros;

    void clearSession();
    bool captureFrame();
    bool waitForSession();
    bool loadSessionFromJson(JsonDocument& doc, String& errorMessage);
    uint32_t getFrameDurationMicros(const IRFrame& frame) const;
    uint32_t getSessionDurationMicros() const;
    void appendDiagnostics(JsonDocument& doc) const;
    void transmitSession();
    bool isContinuationChunk(const IRFrame& frame) const;
    void waitGapMicros(uint32_t gapMicros);

    void handleReceive(WebServer& server);
    void handleSend(WebServer& server);
    void handleDiagnostics(WebServer& server);
    void sendSessionResponse(WebServer& server);
};

#endif // IR_CONTROLLER_HPP