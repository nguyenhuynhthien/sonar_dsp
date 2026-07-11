#include "ComManager.h"

ComManager::ComManager(const char* ssid, const char* password, const char* hostName, uint16_t port)
    : _ssid(ssid), _password(password), _hostName(hostName), _port(port), _remotePort(0), _isStreaming(false) {
}

void ComManager::begin() {
    Serial.println("Starting WiFi STA mode...");
    WiFi.mode(WIFI_STA);
    
    // Connect to WiFi router
    WiFi.begin(_ssid, _password);
    
    Serial.print("Connecting to router ");
    Serial.println(_ssid);

    // Wait up to timeout limit for connection to router
    int timeout = 0;
    while (WiFi.status() != WL_CONNECTED && timeout < (int)Constant::WIFI_CONNECT_TIMEOUT_LIMIT) {
        delay(Constant::WIFI_CONNECT_DELAY_MS);
        Serial.print(".");
        timeout++;
    }

    if (WiFi.status() == WL_CONNECTED) {
        Serial.println("\nWiFi Connected to Router!");
        Serial.print("STA IP Address: ");
        Serial.println(WiFi.localIP());
    } else {
        Serial.println("\nCould not connect to router.");
    }

    // Initialize mDNS
    if (!MDNS.begin(_hostName)) {
        Serial.println("Error setting up MDNS!");
    } else {
        Serial.printf("mDNS started. Connect via: %s.local:%d\n", _hostName, _port);
        MDNS.addService("http", "tcp", _port);
    }

    // Start UDP listening
    _udp.begin(_port);
    Serial.printf("UDP Listening on port %d\n", _port);
}

void ComManager::update() {
    int packetSize = _udp.parsePacket();
    if (packetSize > 0) {
        // Serial.printf("Received UDP packet of size %d from %s:%d\n", 
        //               packetSize, _udp.remoteIP().toString().c_str(), _udp.remotePort());
        char packetBuffer[Constant::UDP_BUFFER_SIZE];
        int len = _udp.read(packetBuffer, sizeof(packetBuffer) - 1);
        if (len > 0) {
            packetBuffer[len] = '\0';
            String command = String(packetBuffer);
            command.trim();
            // Serial.printf("Command content: '%s'\n", command.c_str());

            if (command == "start") {
                _remoteIp = _udp.remoteIP();
                _remotePort = _udp.remotePort();
                
                // Send a tiny dummy packet immediately to trigger ARP resolution in lwIP
                _udp.beginPacket(_remoteIp, _remotePort);
                _udp.write((const uint8_t*)"ping", 4);
                _udp.endPacket();

                // Wait for the router/MAC ARP table to resolve before starting high-speed stream
                delay(Constant::WIFI_ARP_DELAY_MS);
                
                _isStreaming = true;
                Serial.printf("Start streaming registered to %s:%d\n", _remoteIp.toString().c_str(), _remotePort);
            } else if (command == "stop") {
                _isStreaming = false;
                Serial.println("Stop streaming registered.");
            }
        }
    }
}

bool ComManager::isStreaming() {
    return _isStreaming;
}

void ComManager::sendFrame(uint16_t frameId, const uint16_t* samples, size_t size) {
    if (!_isStreaming || size != Constant::ADC_SAMPLES) {
        return;
    }

    const size_t CHUNK_SAMPLES = Constant::CHUNK_SAMPLES;
    const size_t CHUNKS_PER_FRAME = Constant::CHUNKS_PER_FRAME;

    struct __attribute__((packed)) ChunkHeader {
        uint16_t frameId;
        uint8_t chunkIdx;
    };

    for (size_t i = 0; i < CHUNKS_PER_FRAME; ++i) {
        ChunkHeader header;
        header.frameId = frameId;
        header.chunkIdx = (uint8_t)i;

        _udp.beginPacket(_remoteIp, _remotePort);
        _udp.write((const uint8_t*)&header, sizeof(header));
        _udp.write((const uint8_t*)(samples + i * CHUNK_SAMPLES), CHUNK_SAMPLES * sizeof(uint16_t));
        _udp.endPacket();

        // Pace transmission using configured delay to avoid WiFi buffer overflow
        delayMicroseconds(Constant::UDP_PACE_DELAY_US);
    }
}
