#ifndef COM_MANAGER_H
#define COM_MANAGER_H

#include <Arduino.h>
#include <WiFi.h>
#include <ESPmDNS.h>

typedef void (*OnMessageReceivedCallback)(String message);

class ComManager {
public:
    ComManager(const char* ssid, const char* password, const char* hostName, uint16_t port = 8080);
    
    void begin();
    void update();
    void sendMessage(String message);
    bool isConnected();
    
    void setOnMessageReceived(OnMessageReceivedCallback callback);

private:
    const char* _ssid;
    const char* _password;
    const char* _hostName;
    uint16_t _port;
    
    WiFiServer _server;
    WiFiClient _client;
    OnMessageReceivedCallback _onMessageReceived;

    void handleWiFi();
    void handleClient();
};

#endif
