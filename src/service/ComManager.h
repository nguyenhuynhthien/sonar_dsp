#ifndef COM_MANAGER_H
#define COM_MANAGER_H

#include <Arduino.h>
#include <Constant.hpp>
#include <WiFi.h>
#include <WiFiUdp.h>
#include <ESPmDNS.h>

class ComManager {
public:
    ComManager(const char* ssid, const char* password, const char* hostName, uint16_t port = Constant::DEFAULT_PORT);
    
    void begin();
    void update();
    bool isStreaming();
    void sendFrame(uint16_t frameId, const uint16_t* samples, size_t size);

private:
    const char* _ssid;
    const char* _password;
    const char* _hostName;
    uint16_t _port;
    
    WiFiUDP _udp;
    IPAddress _remoteIp;
    uint16_t _remotePort;
    bool _isStreaming;
};

#endif // COM_MANAGER_H
