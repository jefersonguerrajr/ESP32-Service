#include "api/IRController.hpp"
#include <ArduinoJson.h>
#include <IRremote.hpp>
#include <stdio.h>

IRController::IRController(uint8_t receivePin, uint8_t sendPin)
    : _receivePin(receivePin),
      _sendPin(sendPin),
      _sessionLen(0),
      _hasSession(false) {}

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
}

void IRController::clearSession() {
    _sessionLen = 0;
    _hasSession = false;
}

void IRController::populateFrameDebugData(IRFrame& frame) {
    const char* protocolName = getProtocolString(IrReceiver.decodedIRData.protocol);

    frame.protocolId = static_cast<uint8_t>(IrReceiver.decodedIRData.protocol);
    frame.address = IrReceiver.decodedIRData.address;
    frame.command = IrReceiver.decodedIRData.command;
    frame.extra = IrReceiver.decodedIRData.extra;
    frame.numberOfBits = IrReceiver.decodedIRData.numberOfBits;
    frame.flags = IrReceiver.decodedIRData.flags;
    frame.hasDecodedFields = IrReceiver.decodedIRData.protocol != UNKNOWN;

    snprintf(frame.protocolName, sizeof(frame.protocolName), "%s", protocolName != nullptr ? protocolName : "UNKNOWN");
    snprintf(
        frame.decodedRawDataHex,
        sizeof(frame.decodedRawDataHex),
        "0x%llX",
        static_cast<unsigned long long>(IrReceiver.decodedIRData.decodedRawData)
    );
}

bool IRController::captureFrame() {
    if ((IrReceiver.decodedIRData.flags & IRDATA_FLAGS_IS_REPEAT) || IrReceiver.irparams.rawlen < 10) {
        IrReceiver.resume();
        return false;
    }

    if (_sessionLen >= MAX_FRAMES) {
        IrReceiver.resume();
        return true;
    }

    IRFrame& frame = _session[_sessionLen];
    frame.gapBefore = (uint32_t)IrReceiver.irparams.rawbuf[0] * MICROS_PER_TICK;
    frame.kHz = 38;
    populateFrameDebugData(frame);

    uint16_t count = IrReceiver.irparams.rawlen - 1;
    if (count > MAX_RAW_LEN) {
        count = MAX_RAW_LEN;
    }

    for (uint16_t i = 0; i < count; i++) {
        uint32_t microsValue = (uint32_t)IrReceiver.irparams.rawbuf[i + 1] * MICROS_PER_TICK;
        frame.raw[i] = microsValue > 65535U ? 65535U : (uint16_t)microsValue;
    }

    frame.rawLen = count;
    _sessionLen++;
    _hasSession = true;

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
        frame.kHz = frameJson["kHz"] | 38;
        frame.rawLen = rawValues.size();
        frame.protocolId = 0;
        frame.address = 0;
        frame.command = 0;
        frame.extra = 0;
        frame.numberOfBits = 0;
        frame.flags = 0;
        frame.hasDecodedFields = false;
        frame.protocolName[0] = '\0';
        frame.decodedRawDataHex[0] = '\0';

        uint16_t index = 0;
        for (JsonVariantConst rawValue : rawValues) {
            frame.raw[index++] = rawValue.as<uint16_t>();
        }

        _sessionLen++;
    }

    _hasSession = _sessionLen > 0;
    return _hasSession;
}

void IRController::transmitSession() {
    for (uint8_t i = 0; i < _sessionLen; i++) {
        IRFrame& frame = _session[i];

        if (i > 0) {
            uint32_t gapMs = frame.gapBefore / 1000;
            delay((gapMs > 5 && gapMs < 300) ? gapMs : 40);
        }

        IrSender.sendRaw(frame.raw, frame.rawLen, frame.kHz);
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
    responseDoc["framesSent"] = _sessionLen;

    String output;
    serializeJson(responseDoc, output);
    server.send(200, "application/json", output);
}

void IRController::sendSessionResponse(WebServer& server) {
    JsonDocument doc;
    JsonObject session = doc["session"].to<JsonObject>();
    session["frameCount"] = _sessionLen;

    JsonArray frames = session["frames"].to<JsonArray>();
    for (uint8_t i = 0; i < _sessionLen; i++) {
        const IRFrame& frame = _session[i];
        JsonObject frameObject = frames.add<JsonObject>();
        frameObject["gapBefore"] = frame.gapBefore;
        frameObject["kHz"] = frame.kHz;
        frameObject["rawLen"] = frame.rawLen;

        JsonObject debug = frameObject["debug"].to<JsonObject>();
        debug["protocol"] = frame.protocolName;
        debug["protocolId"] = frame.protocolId;
        debug["numberOfBits"] = frame.numberOfBits;
        debug["flags"] = frame.flags;
        debug["decodedRawDataHex"] = frame.decodedRawDataHex;
        if (frame.hasDecodedFields) {
            debug["address"] = frame.address;
            debug["command"] = frame.command;
            debug["extra"] = frame.extra;
        }

        JsonArray raw = frameObject["raw"].to<JsonArray>();
        for (uint16_t j = 0; j < frame.rawLen; j++) {
            raw.add(frame.raw[j]);
        }
    }

    String output;
    serializeJson(doc, output);
    server.send(200, "application/json", output);
}