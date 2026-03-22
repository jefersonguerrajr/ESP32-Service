#include "api/IRController.hpp"
#include <ArduinoJson.h>
#include <IRremote.hpp>
#include <stdio.h>

constexpr uint8_t IRController::MAX_FRAMES;
constexpr uint16_t IRController::MAX_RAW_LEN;
constexpr uint32_t IRController::CAPTURE_TIMEOUT_MS;
constexpr uint32_t IRController::SESSION_END_GAP_MS;
constexpr uint8_t IRController::DEFAULT_KHZ;
constexpr uint32_t IRController::DEFAULT_INTER_FRAME_GAP_MICROS;

IRController::IRController(uint8_t receivePin, uint8_t sendPin)
    : _receivePin(receivePin),
      _sendPin(sendPin),
      _sessionLen(0),
    _hasSession(false),
    _lastFrameStartedAtMicros(0),
    _lastFrameDurationMicros(0) {}

void IRController::begin() {
    IrReceiver.begin(_receivePin, DISABLE_LED_FEEDBACK);
    IrSender.begin(_sendPin);
    clearSession();
}

void IRController::registerRoutes(WebServer& server) {
    server.on("/api/ir/receive", HTTP_GET, [this, &server]() {
        handleReceive(server);
    });

    server.on("/api/ir/send", HTTP_POST, [this, &server]() {
        handleSend(server);
    });

    server.on("/api/ir/diagnostics", HTTP_GET, [this, &server]() {
        handleDiagnostics(server);
    });
}

void IRController::clearSession() {
    _sessionLen = 0;
    _hasSession = false;
    _lastFrameStartedAtMicros = 0;
    _lastFrameDurationMicros = 0;
}

bool IRController::captureFrame() {
    uint32_t captureStartedAtMicros = micros();

    if ((IrReceiver.decodedIRData.flags & IRDATA_FLAGS_IS_REPEAT) || IrReceiver.irparams.rawlen < 10) {
        IrReceiver.resume();
        return false;
    }

    if (_sessionLen >= MAX_FRAMES) {
        IrReceiver.resume();
        return true;
    }

    IRFrame& frame = _session[_sessionLen];
    frame.gapBefore = 0;
    frame.kHz = DEFAULT_KHZ;

    uint16_t count = IrReceiver.irparams.rawlen - 1;
    if (count > MAX_RAW_LEN) {
        count = MAX_RAW_LEN;
    }

    for (uint16_t i = 0; i < count; i++) {
        uint32_t microsValue = (uint32_t)IrReceiver.irparams.rawbuf[i + 1] * MICROS_PER_TICK;
        frame.raw[i] = microsValue > 65535U ? 65535U : (uint16_t)microsValue;
    }

    frame.rawLen = count;
    uint32_t currentFrameDurationMicros = getFrameDurationMicros(frame);
    uint32_t reportedGapMicros = (uint32_t)IrReceiver.irparams.initialGapTicks * MICROS_PER_TICK;
    if (_sessionLen > 0 && _lastFrameStartedAtMicros > 0) {
        uint32_t elapsedSinceLastStart = captureStartedAtMicros - _lastFrameStartedAtMicros;
        uint32_t measuredGapMicros = elapsedSinceLastStart > _lastFrameDurationMicros
            ? elapsedSinceLastStart - _lastFrameDurationMicros
            : 0;
        frame.gapBefore = measuredGapMicros > 0 ? measuredGapMicros : reportedGapMicros;
    }

    _sessionLen++;
    _hasSession = true;
    _lastFrameStartedAtMicros = captureStartedAtMicros;
    _lastFrameDurationMicros = currentFrameDurationMicros;

    IrReceiver.resume();
    return true;
}

bool IRController::waitForSession() {
    clearSession();

    unsigned long startTime = millis();
    unsigned long lastFrameTime = 0;

    while (millis() - startTime < CAPTURE_TIMEOUT_MS) {
        if (IrReceiver.decode()) {
            if (captureFrame()) {
                lastFrameTime = millis();
            }
        }

        if (_hasSession && lastFrameTime > 0 && millis() - lastFrameTime >= SESSION_END_GAP_MS) {
            return true;
        }

        delay(2);
    }

    return _hasSession;
}

bool IRController::loadSessionFromJson(JsonDocument& doc, String& errorMessage) {
    JsonVariantConst sessionKHzVariant = doc["session"]["kHz"];
    bool sessionHasKHz = !sessionKHzVariant.isNull();
    uint8_t sessionKHz = sessionKHzVariant | DEFAULT_KHZ;
    JsonArrayConst frames = doc["session"]["frames"].as<JsonArrayConst>();
    if (frames.isNull() || frames.size() == 0) {
        errorMessage = "Campo 'session.frames' obrigatorio";
        return false;
    }

    if (frames.size() > MAX_FRAMES) {
        errorMessage = "Quantidade de frames excede o limite suportado";
        return false;
    }

    clearSession();

    for (JsonObjectConst frameJson : frames) {
        JsonArrayConst rawValues = frameJson["raw"].as<JsonArrayConst>();
        if (rawValues.isNull() || rawValues.size() == 0) {
            clearSession();
            errorMessage = "Cada frame precisa conter o array 'raw'";
            return false;
        }

        if (rawValues.size() > MAX_RAW_LEN) {
            clearSession();
            errorMessage = "Tamanho do array raw excede o limite suportado";
            return false;
        }

        IRFrame& frame = _session[_sessionLen];
        frame.gapBefore = frameJson["gapBefore"] | 0U;
        frame.kHz = sessionHasKHz ? sessionKHz : (frameJson["kHz"] | DEFAULT_KHZ);
        frame.rawLen = rawValues.size();

        uint16_t index = 0;
        for (JsonVariantConst rawValue : rawValues) {
            frame.raw[index++] = rawValue.as<uint16_t>();
        }

        _sessionLen++;
    }

    _hasSession = _sessionLen > 0;
    return _hasSession;
}

uint32_t IRController::getFrameDurationMicros(const IRFrame& frame) const {
    uint32_t durationMicros = 0;
    for (uint16_t i = 0; i < frame.rawLen; i++) {
        durationMicros += frame.raw[i];
    }

    return durationMicros;
}

uint32_t IRController::getSessionDurationMicros() const {
    uint32_t totalMicros = 0;
    for (uint8_t i = 0; i < _sessionLen; i++) {
        totalMicros += _session[i].gapBefore;
        totalMicros += getFrameDurationMicros(_session[i]);
    }

    return totalMicros;
}

void IRController::appendDiagnostics(JsonDocument& doc) const {
    JsonObject diagnostics = doc["diagnostics"].to<JsonObject>();
    diagnostics["frameCount"] = _sessionLen;
    diagnostics["sessionDurationMicros"] = getSessionDurationMicros();
    diagnostics["sessionDurationMillis"] = getSessionDurationMicros() / 1000.0;

    JsonArray frames = diagnostics["frames"].to<JsonArray>();
    uint32_t replayOffsetMicros = 0;
    for (uint8_t i = 0; i < _sessionLen; i++) {
        const IRFrame& frame = _session[i];
        uint32_t frameDurationMicros = getFrameDurationMicros(frame);

        JsonObject frameDiag = frames.add<JsonObject>();
        frameDiag["index"] = i;
        frameDiag["gapBefore"] = frame.gapBefore;
        frameDiag["durationMicros"] = frameDurationMicros;
        frameDiag["durationMillis"] = frameDurationMicros / 1000.0;
        frameDiag["replayStartOffsetMicros"] = replayOffsetMicros + frame.gapBefore;
        frameDiag["carrierKHz"] = frame.kHz;
        frameDiag["rawLen"] = frame.rawLen;

        replayOffsetMicros += frame.gapBefore + frameDurationMicros;
    }
}

bool IRController::isContinuationChunk(const IRFrame& frame) const {
    // A continuation chunk has no proper protocol header.
    // Real headers have long marks (>3000us, e.g. NEC ~9000us).
    // Continuation chunks start with data-bit-sized marks (~500-700us).
    if (frame.rawLen < 2) return false;
    return frame.raw[0] < 3000;
}

void IRController::waitGapMicros(uint32_t gapMicros) {
    if (gapMicros == 0) {
        return;
    }

    uint32_t gapMillis = gapMicros / 1000;
    uint32_t remainingMicros = gapMicros % 1000;

    if (gapMillis > 0) {
        delay(gapMillis);
    }

    if (remainingMicros > 0) {
        delayMicroseconds(remainingMicros);
    }
}

void IRController::transmitSession() {
    uint16_t combinedRaw[MAX_RAW_LEN];

    for (uint8_t i = 0; i < _sessionLen; ) {
        const IRFrame& frame = _session[i];

        if (i > 0) {
            waitGapMicros(frame.gapBefore > 0 ? frame.gapBefore : DEFAULT_INTER_FRAME_GAP_MICROS);
        }

        // Copy first frame's raw data into combined buffer
        uint16_t combinedLen = 0;
        for (uint16_t j = 0; j < frame.rawLen && combinedLen < MAX_RAW_LEN; j++) {
            combinedRaw[combinedLen++] = frame.raw[j];
        }

        uint8_t lastIndex = i;

        // Append any continuation chunks (frames without a proper long header)
        while ((lastIndex + 1) < _sessionLen && isContinuationChunk(_session[lastIndex + 1])) {
            const IRFrame& nextFrame = _session[lastIndex + 1];

            // Insert the inter-frame gap as a space between the trailing mark
            // of the previous frame and the first mark of the continuation
            if (combinedLen < MAX_RAW_LEN) {
                uint16_t gapSpace;
                if (nextFrame.gapBefore > 0 && nextFrame.gapBefore <= 65535) {
                    gapSpace = (uint16_t)nextFrame.gapBefore;
                } else if (nextFrame.gapBefore > 65535) {
                    gapSpace = 65535;
                } else {
                    // gapBefore unknown (0) - use RECORD_GAP_MICROS as minimum
                    // since separate captures require gap > RECORD_GAP_MICROS
                    gapSpace = RECORD_GAP_MICROS;
                }
                combinedRaw[combinedLen++] = gapSpace;
            }

            // Append continuation frame's raw data
            for (uint16_t j = 0; j < nextFrame.rawLen && combinedLen < MAX_RAW_LEN; j++) {
                combinedRaw[combinedLen++] = nextFrame.raw[j];
            }

            lastIndex++;
        }

        IrSender.sendRaw(combinedRaw, combinedLen, frame.kHz);
        i = lastIndex + 1;
    }

    IrReceiver.restartAfterSend();
}

void IRController::handleReceive(WebServer& server) {
    if (!waitForSession()) {
        server.send(408, "application/json", "{\"message\":\"Nenhum dado foi recebido pelo sensor infravermelho em 10 segundos\"}");
        return;
    }

    sendSessionResponse(server);
}

void IRController::handleSend(WebServer& server) {
    if (!server.hasArg("plain") || server.arg("plain").isEmpty()) {
        server.send(400, "application/json", "{\"error\":\"Body JSON obrigatorio\"}");
        return;
    }

    JsonDocument doc;
    DeserializationError error = deserializeJson(doc, server.arg("plain"));
    if (error) {
        server.send(400, "application/json", "{\"error\":\"JSON invalido\"}");
        return;
    }

    String errorMessage;
    if (!loadSessionFromJson(doc, errorMessage)) {
        JsonDocument errorDoc;
        errorDoc["error"] = errorMessage;

        String output;
        serializeJson(errorDoc, output);
        server.send(400, "application/json", output);
        return;
    }

    transmitSession();

    JsonDocument responseDoc;
    responseDoc["message"] = "Sinal infravermelho enviado";
    JsonObject session = responseDoc["session"].to<JsonObject>();
    session["frameCount"] = _sessionLen;
    session["kHz"] = _sessionLen > 0 ? _session[0].kHz : DEFAULT_KHZ;

    String output;
    serializeJson(responseDoc, output);
    server.send(200, "application/json", output);
}

void IRController::handleDiagnostics(WebServer& server) {
    if (!_hasSession || _sessionLen == 0) {
        server.send(404, "application/json", "{\"error\":\"Nenhuma sessao IR carregada\"}");
        return;
    }

    JsonDocument doc;
    appendDiagnostics(doc);

    String output;
    serializeJson(doc, output);
    server.send(200, "application/json", output);
}

void IRController::sendSessionResponse(WebServer& server) {
    JsonDocument doc;
    JsonObject session = doc["session"].to<JsonObject>();
    session["frameCount"] = _sessionLen;
    session["kHz"] = _sessionLen > 0 ? _session[0].kHz : DEFAULT_KHZ;

    JsonArray frames = session["frames"].to<JsonArray>();
    for (uint8_t i = 0; i < _sessionLen; i++) {
        const IRFrame& frame = _session[i];
        JsonObject frameObject = frames.add<JsonObject>();
        frameObject["gapBefore"] = frame.gapBefore;

        JsonArray raw = frameObject["raw"].to<JsonArray>();
        for (uint16_t j = 0; j < frame.rawLen; j++) {
            raw.add(frame.raw[j]);
        }
    }

    String output;
    serializeJson(doc, output);
    server.send(200, "application/json", output);
}