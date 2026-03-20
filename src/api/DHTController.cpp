#include "api/DHTController.hpp"
#include <ArduinoJson.h>

/**
 * DHTController.cpp - Controlador do sensor DHT22 via API REST
 * Expõe um endpoint GET /api/sensors que retorna temperatura e umidade.
 */

DHTController::DHTController(uint8_t pin, uint8_t type) : _dht(pin, type) {}

void DHTController::registerRoutes(WebServer& server) {
    _dht.begin();
    // GET /api/sensors → retorna temperatura e umidade do DHT22
    server.on("/api/sensors", HTTP_GET, [this, &server]() {
        handleGetSensors(server);
    });
}

// ── Handlers ────────────────────────────────────────────────

void DHTController::handleGetSensors(WebServer& server) {
    sensors_event_t event;
    float temperature = NAN;
    float humidity = NAN;

    _dht.temperature().getEvent(&event);
    if (!isnan(event.temperature)) {
        temperature = event.temperature;
    }

    _dht.humidity().getEvent(&event);
    if (!isnan(event.relative_humidity)) {
        humidity = event.relative_humidity;
    }

    if (isnan(temperature) || isnan(humidity)) {
        server.send(503, "application/json", "{\"error\":\"DHT22 read failed\"}");
        return;
    }

    JsonDocument doc;
    JsonObject sensor = doc["sensor"].to<JsonObject>();
    JsonObject dht22 = sensor["dht22"].to<JsonObject>();
    dht22["temperature"] = roundf(temperature * 10.0f) / 10.0f;
    dht22["humidity"] = roundf(humidity * 10.0f) / 10.0f;

    String output;
    serializeJson(doc, output);
    server.send(200, "application/json", output);
}
