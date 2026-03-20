#include "WiFiManager.hpp"
#include "webpages.hpp"

// Initialize static instance pointer
WiFiManager* WiFiManager::instance = nullptr;

WiFiManager::WiFiManager() : server(80), isAPMode(false), resetButtonPressTime(0) {
    instance = this;
}

void WiFiManager::begin() {
    delay(100);
    pinMode(LED_PIN, OUTPUT);
    pinMode(RESET_BUTTON, INPUT_PULLUP);
    digitalWrite(LED_PIN, HIGH);
    
    Serial.println("\n\nESP32 WiFi Manager");
    
    // Verifica se há credenciais salvas
    preferences.begin("wifi-config", true);
    String savedSSID = preferences.getString("ssid", "");
    String savedPassword = preferences.getString("password", "");
    preferences.end();
    
    if (savedSSID.length() > 0) {
        Serial.println("Credenciais encontradas: " + savedSSID);
        startStationMode(savedSSID, savedPassword);
    } else {
        Serial.println("Nenhuma credencial salva. Iniciando modo AP");
        startAPMode();
    }
}

void WiFiManager::handleClient() {
    server.handleClient();
}

void WiFiManager::checkResetButton() {
    // Verifica botão de reset (pressionar GPIO0 por 3 segundos)
    if (digitalRead(RESET_BUTTON) == LOW) {
        if (resetButtonPressTime == 0) {
            resetButtonPressTime = millis();
        } else if (millis() - resetButtonPressTime > 3000) {
            Serial.println("Botão de reset pressionado - Limpando WiFi");
            for (int c = 0; c < 10; c++) {
                digitalWrite(LED_PIN, !digitalRead(LED_PIN));
                delay(200);
            }
            preferences.begin("wifi-config", false);
            preferences.clear();
            preferences.end();
            delay(500);
            ESP.restart();
        }
    } else {
        resetButtonPressTime = 0;
    }
}

void WiFiManager::handleRoot() {
    if (isAPMode) {
        server.send_P(200, "text/html", config_html);
    } else {
        String html = String(status_html);
        html.replace("%SSID%", WiFi.SSID());
        html.replace("%MAC%", WiFi.macAddress());
        html.replace("%RSSI%", String(WiFi.RSSI()));
        server.send(200, "text/html", html);
    }
}

void WiFiManager::handleSave() {
    if (!isAPMode) {
        server.send(403, "text/plain", "Not in AP mode");
        return;
    }
    
    String ssid = server.arg("ssid");
    String password = server.arg("password");
    
    Serial.println("Recebidas credenciais:");
    Serial.println("SSID: " + ssid);
    
    // Envia página de processamento
    server.send_P(200, "text/html", processing_html);
    
    delay(1000);
    
    // Tenta conectar
    WiFi.mode(WIFI_STA);
    WiFi.begin(ssid.c_str(), password.c_str());
    
    int attempts = 0;
    while (WiFi.status() != WL_CONNECTED && attempts < 40) {
        delay(500);
        Serial.print(".");
        digitalWrite(LED_PIN, !digitalRead(LED_PIN));
        attempts++;
    }
    
    if (WiFi.status() == WL_CONNECTED) {
        Serial.println("\nConexão bem-sucedida!");
        Serial.println("IP: " + WiFi.localIP().toString());
        
        // Salva credenciais
        preferences.begin("wifi-config", false);
        preferences.putString("ssid", ssid);
        preferences.putString("password", password);
        preferences.end();
        
        digitalWrite(LED_PIN, LOW);
        isAPMode = false;
        
        // Reconfigura servidor
        server.stop();
        delay(100);
        server.begin();
    } else {
        Serial.println("\nFalha na conexão. Retornando ao modo AP");
        WiFi.mode(WIFI_AP);
        WiFi.softAP("ESP32-Config");
    }
}

void WiFiManager::handleReset() {
    preferences.begin("wifi-config", false);
    preferences.clear();
    preferences.end();
    
    server.send(200, "text/html", 
        "<html><head><meta charset='utf-8'></head><body style='font-family:Arial;text-align:center;margin:3rem;'>"
        "<h1>Configurações WiFi limpas!</h1><p>O ESP32 será reiniciado...</p></body></html>");
    
    delay(2000);
    ESP.restart();
}

void WiFiManager::startAPMode() {
    Serial.println("Iniciando modo AP...");
    isAPMode = true;
    
    WiFi.mode(WIFI_AP);
    WiFi.softAP("ESP32-Config");
    
    IPAddress IP = WiFi.softAPIP();
    Serial.print("AP IP: ");
    Serial.println(IP);
    
    server.on("/", handleRootWrapper);
    server.on("/save", HTTP_POST, handleSaveWrapper);
    server.begin();
    Serial.println("Servidor AP iniciado na porta 80");
}

void WiFiManager::startStationMode(String ssid, String password) {
    Serial.println("Conectando a rede WiFi...");
    isAPMode = false;
    
    WiFi.mode(WIFI_STA);
    WiFi.begin(ssid.c_str(), password.c_str());
    
    int attempts = 0;
    while (WiFi.status() != WL_CONNECTED && attempts < 40) {
        delay(500);
        Serial.print(".");
        digitalWrite(LED_PIN, !digitalRead(LED_PIN));
        attempts++;
    }
    
    if (WiFi.status() == WL_CONNECTED) {
        Serial.println("\nWiFi conectado!");
        Serial.println("IP: " + WiFi.localIP().toString());
        digitalWrite(LED_PIN, LOW);
        
        server.on("/", handleRootWrapper);
        server.on("/reset", handleResetWrapper);
        server.begin();
        Serial.println("Servidor iniciado na porta 80");
    } else {
        Serial.println("\nFalha ao conectar. Limpando credenciais e iniciando AP");
        preferences.begin("wifi-config", false);
        preferences.clear();
        preferences.end();
        startAPMode();
    }
}

// Static wrapper functions
void WiFiManager::handleRootWrapper() {
    if (instance) instance->handleRoot();
}

void WiFiManager::handleSaveWrapper() {
    if (instance) instance->handleSave();
}

void WiFiManager::handleResetWrapper() {
    if (instance) instance->handleReset();
}
