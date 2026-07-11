#include <Arduino.h>
#include <Constant.hpp>
#include "service/ComManager.h"
#include "app/TransmitterApp.h"
#include "app/ReceiverApp.hpp"

// Shared data block between Core 0 and Core 1
SharedSonarData sharedData = {
    .triggerTx = false,
    .processingDone = false,
    .adcReady = false,
    .adcBuffer = {0},
    .spinlock = portMUX_INITIALIZER_UNLOCKED,
    .rxTaskHandle = nullptr
};

// WiFi SSID, password and MDNS hostname
const char* ssid = "Noel";
const char* password = "hongthanh2110";
const char* hostName = "esp32";

ComManager com(ssid, password, hostName);
TransmitterApp txApp(com, sharedData);
ReceiverApp rxApp(sharedData);

void setup() {
    Serial.begin(Constant::SERIAL_BAUD_RATE);
    delay(Constant::SETUP_DELAY_MS);
    Serial.println("System starting up...");

    // Initialize drivers & local applications (networking is offloaded to Core 0)
    txApp.begin();
    rxApp.begin();

    Serial.println("Drivers and Services initialized.");

    // Pin Task0 to Core 0: High-priority hardware Pulse Generation and Wi-Fi UDP networking
    xTaskCreatePinnedToCore(
        [](void* param) {
            Serial.println("Transmitter Task (Core 0) started.");
            com.begin(); // Initialize UDP socket and WiFi on Core 0 for thread safety
            while (true) {
                txApp.run();
            }
        },
        "TxTask",
        Constant::TASK_STACK_SIZE,
        nullptr,
        Constant::TASK_PRIORITY,
        nullptr,
        0   // Pinned to Core 0
    );

    // Pin Task1 to Core 1: Critical real-time DSP pipelines and high-speed ADC sampling
    xTaskCreatePinnedToCore(
        [](void* param) {
            Serial.println("Receiver Task (Core 1) started.");
            while (true) {
                rxApp.run();
            }
        },
        "RxTask",
        Constant::TASK_STACK_SIZE,
        nullptr,
        Constant::TASK_PRIORITY,
        &sharedData.rxTaskHandle,
        1   // Pinned to Core 1
    );
}

void loop() {
    // Main loop does nothing as the work is executed asynchronously by Core 0/1 tasks.
    vTaskDelay(pdMS_TO_TICKS(Constant::LOOP_DELAY_MS));
}
