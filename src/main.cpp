#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include <ArduinoJson.h>
#include <Preferences.h>

#define LED_WIFI_STATUS 2
#define RESET_BUTTON 0  // GPIO0 (BOOT button)

WebServer server(80);
Preferences preferences;

bool isAPMode = false;
unsigned long resetButtonPressTime = 0;

// Página de configuração WiFi (modo AP)
const char config_html[] PROGMEM = R"rawliteral(
<!DOCTYPE HTML>
<html>
  <head>
    <meta charset="utf-8">
    <meta name="viewport" content="width=device-width, initial-scale=1">
    <title>ESP32 - Configurar WiFi</title>
    <style>
      body{font-family:Arial; max-width:400px; margin:2rem auto; padding:1rem;}
      h1{text-align:center; color:#0066cc;}
      form{background:#f0f0f0; padding:1.5rem; border-radius:8px;}
      label{display:block; margin-top:1rem; font-weight:bold;}
      input{width:100%; padding:0.5rem; margin-top:0.3rem; border:1px solid #ccc; border-radius:4px; box-sizing:border-box;}
      button{width:100%; padding:0.7rem; margin-top:1.5rem; background:#0066cc; color:white; border:none; border-radius:4px; font-size:16px; cursor:pointer;}
      button:hover{background:#0052a3;}
      .info{text-align:center; color:#666; margin-top:1rem; font-size:14px;}
    </style>
  </head>
  <body>
    <h1>🛜 Configuração WiFi</h1>
    <form action="/save" method="POST">
      <label for="ssid">Nome da Rede (SSID):</label>
      <input type="text" id="ssid" name="ssid" required>
      
      <label for="password">Senha:</label>
      <input type="password" id="password" name="password" required>
      
      <button type="submit">Conectar</button>
    </form>
    <div class="info">
      <p>Conecte-se ao AP: <strong>ESP32-Config</strong></p>
      <p>IP: 192.168.4.1</p>
    </div>
  </body>
</html>
)rawliteral";

// Página de status (após conexão bem-sucedida)
const char status_html[] PROGMEM = R"rawliteral(
<!DOCTYPE HTML>
<html>
  <head>
    <meta charset="utf-8">
    <meta name="viewport" content="width=device-width, initial-scale=1">
    <title>ESP32 - Status</title>
    <style>
      body{font-family:Arial; max-width:500px; margin:2rem auto; padding:1rem; text-align:center;}
      h1{color:#00aa00;}
      .status{background:#e8f5e9; padding:1.5rem; border-radius:8px; margin:1rem 0;}
      .info{margin:0.5rem 0; font-size:16px;}
      .ip{font-size:24px; font-weight:bold; color:#0066cc; margin:1rem 0;}
      button{padding:0.7rem 1.5rem; margin:0.5rem; background:#0066cc; color:white; border:none; border-radius:4px; cursor:pointer;}
      button:hover{background:#0052a3;}
      .reset-btn{background:#cc0000;}
      .reset-btn:hover{background:#a30000;}
    </style>
  </head>
  <body>
    <h1>✅ WiFi Conectado</h1>
    <div class="status">
      <div class="info">Rede: <strong>%SSID%</strong></div>
      <div class="ip">IP: %IP%</div>
    </div>
    <button onclick="fetch('/api/led').then(()=>alert('LED alternado!'))">Toggle LED</button>
    <br>
    <button class="reset-btn" onclick="if(confirm('Limpar configurações WiFi?'))location.href='/reset'">Resetar WiFi</button>
  </body>
</html>
)rawliteral";

// Página de processamento
const char processing_html[] PROGMEM = R"rawliteral(
<!DOCTYPE HTML>
<html>
  <head>
    <meta charset="utf-8">
    <meta name="viewport" content="width=device-width, initial-scale=1">
    <title>Conectando...</title>
    <style>
      body{font-family:Arial; text-align:center; margin:3rem auto; max-width:400px;}
      .spinner{border:8px solid #f3f3f3; border-top:8px solid #0066cc; border-radius:50%; width:60px; height:60px; animation:spin 1s linear infinite; margin:2rem auto;}
      @keyframes spin{0%{transform:rotate(0deg)} 100%{transform:rotate(360deg)}}
    </style>
    <script>
      setTimeout(()=>window.location.href='/', 8000);
    </script>
  </head>
  <body>
    <h1>Conectando ao WiFi...</h1>
    <div class="spinner"></div>
    <p>Aguarde enquanto conectamos à rede.</p>
  </body>
</html>
)rawliteral";

void handleRoot() {
  if (isAPMode) {
    server.send_P(200, "text/html", config_html);
  } else {
    String html = String(status_html);
    html.replace("%SSID%", WiFi.SSID());
    html.replace("%IP%", WiFi.localIP().toString());
    server.send(200, "text/html", html);
  }
}

void handleSave() {
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
    digitalWrite(LED_WIFI_STATUS, !digitalRead(LED_WIFI_STATUS));
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
    
    digitalWrite(LED_WIFI_STATUS, HIGH);
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

void handleReset() {
  preferences.begin("wifi-config", false);
  preferences.clear();
  preferences.end();
  
  server.send(200, "text/html", 
    "<html><head><meta charset='utf-8'></head><body style='font-family:Arial;text-align:center;margin:3rem;'>"
    "<h1>Configurações WiFi limpas!</h1><p>O ESP32 será reiniciado...</p></body></html>");
  
  delay(2000);
  ESP.restart();
}

void handleToggleLED() {
  int state = digitalRead(LED_WIFI_STATUS);
  digitalWrite(LED_WIFI_STATUS, !state);
  
  StaticJsonDocument<100> doc;
  doc["led"] = (digitalRead(LED_WIFI_STATUS) == LOW) ? "on" : "off";
  String output;
  serializeJson(doc, output);
  server.send(200, "application/json", output);
}

void startAPMode() {
  Serial.println("Iniciando modo AP...");
  isAPMode = true;
  
  WiFi.mode(WIFI_AP);
  WiFi.softAP("ESP32-Config");
  
  IPAddress IP = WiFi.softAPIP();
  Serial.print("AP IP: ");
  Serial.println(IP);
  
  server.on("/", handleRoot);
  server.on("/save", HTTP_POST, handleSave);
  server.begin();
  Serial.println("Servidor AP iniciado na porta 80");
}

void startStationMode(String ssid, String password) {
  Serial.println("Tentando conectar ao WiFi salvo...");
  isAPMode = false;
  
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid.c_str(), password.c_str());
  
  int attempts = 0;
  while (WiFi.status() != WL_CONNECTED && attempts < 40) {
    delay(500);
    Serial.print(".");
    digitalWrite(LED_WIFI_STATUS, !digitalRead(LED_WIFI_STATUS));
    attempts++;
  }
  
  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("\nWiFi conectado!");
    Serial.println("IP: " + WiFi.localIP().toString());
    digitalWrite(LED_WIFI_STATUS, HIGH);
    
    server.on("/", handleRoot);
    server.on("/reset", handleReset);
    server.on("/api/led", handleToggleLED);
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

void setup() {
  Serial.begin(115200);
  delay(100);
  
  pinMode(LED_WIFI_STATUS, OUTPUT);
  pinMode(RESET_BUTTON, INPUT_PULLUP);
  digitalWrite(LED_WIFI_STATUS, HIGH);
  
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

void loop() {
  server.handleClient();
  
  // Verifica botão de reset (pressionar GPIO0 por 3 segundos)
  if (digitalRead(RESET_BUTTON) == LOW) {
    if (resetButtonPressTime == 0) {
      resetButtonPressTime = millis();
    } else if (millis() - resetButtonPressTime > 3000) {
      Serial.println("Botão de reset pressionado - Limpando WiFi");
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
