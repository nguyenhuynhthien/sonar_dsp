#include "ComManager.h"

ComManager::ComManager(const char* ssid, const char* password, const char* hostName, uint16_t port)
    : _ssid(ssid), _password(password), _hostName(hostName), _port(port), _server(port), _onMessageReceived(nullptr) {
}

void ComManager::begin() {
    Serial.print("Connecting to WiFi: ");
    Serial.println(_ssid);
    WiFi.begin(_ssid, _password);

    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
    }

    Serial.println("\nWiFi Connected!");
    Serial.print("IP Address: ");
    Serial.println(WiFi.localIP());

    _server.begin();

    if (!MDNS.begin(_hostName)) {
        Serial.println("Error setting up MDNS!");
    } else {
        Serial.printf("mDNS started. Connect via: %s.local:%d\n", _hostName, _port);
    }
}

void ComManager::update() {
    handleClient();
}

void ComManager::handleClient() {
    // Kiem tra neu co client moi trong khi client cu da mat ket noi
    if (!_client || !_client.connected()) {
        WiFiClient newClient = _server.available();
        if (newClient) {
            if (_client) _client.stop(); // Don dep client cu neu co
            _client = newClient;
            
            Serial.print("Client connected from: ");
            Serial.println(_client.remoteIP());
            _client.println("Connected to ESP32 ComManager");
        }
    }

    while (_client && _client.connected() && _client.available()) {
        String msg = _client.readStringUntil('\n');
        msg.trim();
        if (_onMessageReceived) {
            _onMessageReceived(msg);
        }
    }
}

void ComManager::sendMessage(String message) {
    if (_client && _client.connected()) {
        _client.println(message);
    }
}

bool ComManager::isConnected() {
    return _client && _client.connected();
}

void ComManager::setOnMessageReceived(OnMessageReceivedCallback callback) {
    _onMessageReceived = callback;
}
