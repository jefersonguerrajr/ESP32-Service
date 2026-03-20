#include "api/LedController.hpp"
#include <ArduinoJson.h>

/** 
 * LedController.cpp - Controlador de LED via API REST
 * Este arquivo implementa a classe LedController, que gerencia um LED conectado a um pino GPIO do ESP32.
 * 
 */

LedController::LedController(uint8_t pin) : _pin(pin) {
    pinMode(_pin, OUTPUT);
    digitalWrite(_pin, HIGH); // começa apagado (LED ativo em LOW)
}

void LedController::registerRoutes(WebServer& server) {
    // GET /api/led → retorna estado atual
    server.on("/api/led", HTTP_GET, [this, &server]() {
        handleGetStatus(server);
    });

    // PUT /api/led → define estado via body JSON {"state": "on"} ou {"state": "off"}
    server.on("/api/led", HTTP_PUT, [this, &server]() {
        handleSetState(server);
    });

    // POST /api/led/toggle → alterna estado
    server.on("/api/led/toggle", HTTP_POST, [this, &server]() {
        handleToggle(server);
    });
}

// ── Handlers ────────────────────────────────────────────────

void LedController::handleGetStatus(WebServer& server) {
    sendStateResponse(server);
}

void LedController::handleSetState(WebServer& server) {
    if (!server.hasArg("plain") || server.arg("plain").isEmpty()) {
        server.send(400, "application/json", "{\"error\":\"Body JSON obrigatorio\"}");
        return;
    }

    StaticJsonDocument<64> doc;
    DeserializationError err = deserializeJson(doc, server.arg("plain"));

    if (err || !doc.containsKey("state")) {
        server.send(400, "application/json", "{\"error\":\"Campo 'state' obrigatorio\"}");
        return;
    }

    const char* state = doc["state"];

    if (strcmp(state, "on") == 0) {
        digitalWrite(_pin, LOW);   // LED ativo em LOW
    } else if (strcmp(state, "off") == 0) {
        digitalWrite(_pin, HIGH);
    } else {
        server.send(400, "application/json", "{\"error\":\"state deve ser 'on' ou 'off'\"}");
        return;
    }

    sendStateResponse(server);
}

void LedController::handleToggle(WebServer& server) {
    digitalWrite(_pin, isOn() ? HIGH : LOW);
    sendStateResponse(server);
}

// ── Helpers ─────────────────────────────────────────────────

bool LedController::isOn() const {
    return digitalRead(_pin) == LOW; // LED ativo em LOW
}

void LedController::sendStateResponse(WebServer& server) {
    StaticJsonDocument<128> doc;
    doc["gpio"]  = _pin;
    doc["state"] = isOn() ? "on" : "off";

    String output;
    serializeJson(doc, output);
    server.send(200, "application/json", output);
}
