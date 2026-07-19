#include "ComManager.h"

ComManager::ComManager(const char *ssid, const char *password,
                       const char *hostName, uint16_t port)
    : _ssid(ssid), _password(password), _hostName(hostName), _port(port),
      _remotePort(0), _isStreaming(false), _pulseType(PULSE_SINGLE),
      _isServoEnabled(false), _streamMode(STREAM_RAW), _txGain(1.0f), 
      _isTxEnabled(false), _targetServoAngle(-1) {
  for (int i = 0; i < 3; ++i) {
    _queuedFrames[i].samples = nullptr;
    _queuedFrames[i].ready = false;
  }
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
  while (WiFi.status() != WL_CONNECTED &&
         timeout < (int)Constant::WIFI_CONNECT_TIMEOUT_LIMIT) {
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

  // Allocate dynamic buffers on the heap
  for (int i = 0; i < 3; ++i) {
    if (_queuedFrames[i].samples == nullptr) {
      _queuedFrames[i].samples = (int16_t*)malloc(Constant::ADC_SAMPLES * sizeof(int16_t));
    }
  }
}

void ComManager::update() {
  int packetSize = _udp.parsePacket();
  if (packetSize > 0) {
    char packetBuffer[Constant::UDP_BUFFER_SIZE];
    int len = _udp.read(packetBuffer, sizeof(packetBuffer) - 1);
    if (len > 0) {
      _remoteIp = _udp.remoteIP();
      _remotePort = _udp.remotePort();
      packetBuffer[len] = '\0';
      String command = String(packetBuffer);
      command.trim();

      if (command == "start") {
        // Send a tiny dummy packet immediately to trigger ARP resolution in
        // lwIP
        _udp.beginPacket(_remoteIp, _remotePort);
        _udp.write((const uint8_t *)"ping", 4);
        _udp.endPacket();

        // Wait for the router/MAC ARP table to resolve before starting
        // high-speed stream
        delay(Constant::WIFI_ARP_DELAY_MS);

        _isStreaming = true;
        Serial.printf("Start streaming registered to %s:%d\n",
                      _remoteIp.toString().c_str(), _remotePort);
      } else if (command == "stop") {
        _isStreaming = false;
        Serial.println("Stop streaming registered.");
      } else if (command == "cfg:single") {
        _pulseType = PULSE_SINGLE;
        Serial.println("Pulse config changed: Single");
      } else if (command == "cfg:barker13") {
        _pulseType = PULSE_BARKER13;
        Serial.println("Pulse config changed: Barker13");
      } else if (command == "servo:on") {
        _isServoEnabled = true;
        Serial.println("Servo control enabled.");
      } else if (command == "servo:off") {
        _isServoEnabled = false;
        Serial.println("Servo control disabled.");
      } else if (command == "mode:raw") {
        _streamMode = STREAM_RAW;
        Serial.println("Streaming mode: Raw Signal");
      } else if (command == "mode:demod") {
        _streamMode = STREAM_DEMOD;
        Serial.println("Streaming mode: Demodulated");
      } else if (command == "mode:compressed") {
        _streamMode = STREAM_COMPRESSED;
        Serial.println("Streaming mode: Pulse Compressed");
      } else if (command.startsWith("tx_atten:")) {
        String valStr = command.substring(9);
        if (valStr == "mute") {
          _txGain = 0.0f;
          Serial.println("Tx attenuation: Mute");
        } else {
          int attenDb = valStr.toInt();
          _txGain = powf(10.0f, -attenDb / 20.0f);
          Serial.printf("Tx attenuation: -%d dB (gain: %.4f)\n", attenDb, _txGain);
        }
      } else if (command == "tx:on") {
        _isTxEnabled = true;
        Serial.println("Tx Enabled");
      } else if (command == "tx:off") {
        _isTxEnabled = false;
        Serial.println("Tx Disabled");
      } else if (command.startsWith("servo:")) {
        if (command == "servo:on") {
          _isServoEnabled = true;
          Serial.println("Servo control enabled.");
        } else if (command == "servo:off") {
          _isServoEnabled = false;
          Serial.println("Servo control disabled.");
        } else {
          int angle = command.substring(6).toInt();
          if (angle >= 0 && angle <= 180) {
            _targetServoAngle = angle;
            Serial.printf("Servo command: Move to %d degrees\n", angle);
          }
        }
      }
    }
  }
}

int ComManager::getTargetServoAngle() {
  int angle = _targetServoAngle;
  _targetServoAngle = -1;
  return angle;
}

bool ComManager::isStreaming() { return _isStreaming; }

void ComManager::sendFrame(uint16_t frameId, const int16_t *samples,
                           size_t size, uint8_t receiverId) {
  if (!_isStreaming || size != Constant::ADC_SAMPLES) {
    return;
  }

  const size_t CHUNK_SAMPLES = Constant::CHUNK_SAMPLES;
  const size_t CHUNKS_PER_FRAME = Constant::CHUNKS_PER_FRAME;

  struct __attribute__((packed)) ChunkHeader {
    uint16_t frameId;
    uint8_t chunkIdx;
    uint8_t receiverId;
  };

  for (size_t i = 0; i < CHUNKS_PER_FRAME; ++i) {
    ChunkHeader header;
    header.frameId = frameId;
    header.chunkIdx = (uint8_t)i;
    header.receiverId = receiverId;

    _udp.beginPacket(_remoteIp, _remotePort);
    _udp.write((const uint8_t *)&header, sizeof(header));
    _udp.write((const uint8_t *)(samples + i * CHUNK_SAMPLES),
               CHUNK_SAMPLES * sizeof(int16_t));
    _udp.endPacket();

    // Pace transmission using configured delay to avoid WiFi buffer overflow
    delayMicroseconds(Constant::UDP_PACE_DELAY_US);
  }
}

void ComManager::sendAngle(uint16_t angle) {
  if (_remotePort == 0) {
    return;
  }
  char buf[16];
  int len = snprintf(buf, sizeof(buf), "ang:%d", angle);

  _udp.beginPacket(_remoteIp, _remotePort);
  _udp.write((const uint8_t *)buf, len);
  _udp.endPacket();
}

void ComManager::sendTarget(int32_t rangeBin, uint16_t angle, int32_t amplitude,
                            int32_t velocityBin, uint8_t receiverId) {
  if (_remotePort == 0) {
    return;
  }
  char buf[64];
  int len = snprintf(buf, sizeof(buf), "target:%d,%d,%d,%d,%d", rangeBin, angle,
                     amplitude, velocityBin, receiverId);

  _udp.beginPacket(_remoteIp, _remotePort);
  _udp.write((const uint8_t *)buf, len);
  _udp.endPacket();
}

void ComManager::sendFrameAsync(uint16_t frameId, const int16_t* samples, size_t size, uint8_t receiverId) {
  if (!_isStreaming || size != Constant::ADC_SAMPLES) {
    return;
  }
  // Determine queue slot based on receiverId (Rx0 Sum -> slot 2, Rx1 -> slot 0, Rx2 -> slot 1)
  int slot = (receiverId == 0) ? 2 : (receiverId - 1);
  if (_queuedFrames[slot].samples == nullptr) return;
  if (_queuedFrames[slot].ready) return; // Drop frame if previous one is still sending to avoid corruption

  _queuedFrames[slot].frameId = frameId;
  _queuedFrames[slot].receiverId = receiverId;
  memcpy(_queuedFrames[slot].samples, samples, Constant::ADC_SAMPLES * sizeof(int16_t));
  _queuedFrames[slot].ready = true;
}

bool ComManager::processAsyncSends() {
  bool sentAny = false;
  for (int i = 0; i < 3; ++i) {
    if (_queuedFrames[i].ready && _queuedFrames[i].samples != nullptr) {
      sendFrame(_queuedFrames[i].frameId, _queuedFrames[i].samples, Constant::ADC_SAMPLES, _queuedFrames[i].receiverId);
      _queuedFrames[i].ready = false;
      sentAny = true;
      // Yield to let the WiFi stack process and prevent LwIP lockup
      vTaskDelay(pdMS_TO_TICKS(1));
    }
  }
  return sentAny;
}
