#ifndef COM_MANAGER_H
#define COM_MANAGER_H

#include <Arduino.h>
#include <Constant.hpp>
#include <WiFi.h>
#include <WiFiUdp.h>
#include <ESPmDNS.h>

class ComManager {
public:
    enum PulseType {
        PULSE_SINGLE,
        PULSE_BARKER13
    };

    enum StreamMode {
        STREAM_RAW = 0,
        STREAM_DEMOD = 1,
        STREAM_COMPRESSED = 2
    };

    ComManager(const char* ssid, const char* password, const char* hostName, uint16_t port = Constant::DEFAULT_PORT);
    
    void begin();
    void update();
    bool isStreaming();
    void sendFrame(uint16_t frameId, const uint16_t* samples, size_t size, uint16_t angle);
    void sendAngle(uint16_t angle);
    void sendTarget(float range, uint16_t angle, float strength);
    PulseType getPulseType() const { return _pulseType; }
    bool isServoEnabled() const { return _isServoEnabled; }
    StreamMode getStreamMode() const { return _streamMode; }

private:
    const char* _ssid;
    const char* _password;
    const char* _hostName;
    uint16_t _port;
    
    WiFiUDP _udp;
    IPAddress _remoteIp;
    uint16_t _remotePort;
    bool _isStreaming;
    PulseType _pulseType;
    bool _isServoEnabled;
    StreamMode _streamMode;
};

#endif // COM_MANAGER_H
