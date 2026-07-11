#include <Arduino.h>
#include "service/ComManager.h"

// Thông tin WiFi
const char* ssid = "Noel";
const char* password = "hongthanh2110";
const char* hostName = "esp32";

ComManager com(ssid, password, hostName);

// Hàm callback xử lý tin nhắn nhận được
void onMessage(String msg) {
    Serial.println("Received: " + msg);
    
    if (msg == "on") {
        digitalWrite(2, HIGH);
        com.sendMessage(">>> LED ON");
    } else if (msg == "off") {
        digitalWrite(2, LOW);
        com.sendMessage(">>> LED OFF");
    } else {
        com.sendMessage("ESP32 received: " + msg);
    }
}

void setup() {
    Serial.begin(115200);
    pinMode(2, OUTPUT);

    com.setOnMessageReceived(onMessage);
    com.begin();
}

void loop() {
    com.update();

    // Gửi bản tin cứng định kỳ
    static unsigned long lastSend = 0;
    if (com.isConnected() && millis() - lastSend > 2000) {
        com.sendMessage("Heartbeat - Millis: " + String(millis()));
        lastSend = millis();
    }
    delay(2000); // Giảm tải CPU
}



