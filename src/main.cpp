#include "app/ReceiverApp.hpp"
#include "app/ScannerApp.h"
#include "app/TransmitterApp.h"
#include "app/SyncSignalApp.h"
#include "service/ComManager.h"
#include "service/ServoService.h"
#include <Arduino.h>
#include <Constant.hpp>

// Shared data block between Core 0 and Core 1
SharedSonarData sharedData = {.triggerTx = false,
                              .processingDone = false,
                              .adcReady = false,
                              .adcBuffer = {0},
                              .spinlock = portMUX_INITIALIZER_UNLOCKED,
                              .rxTaskHandle = nullptr,
                              .servoTaskHandle = nullptr,
                              .txBuffer = {0},
                              .txPulseLen = 0,
                              .simDelaySamples = 400, // Default to 400 samples delay (~2.5ms delay, simulating ~1.8 meters range)
                              .simEnabled = true,     // Simulator enabled by default
                              .servoAngle = 0,
                              .angleUpdated = false,
                              .targetRange = 0.0f,
                              .targetStrength = 0.0f,
                              .targetDetected = false,
                              .streamMode = 0};

// WiFi SSID, password and MDNS hostname
const char *ssid = "Noel";
const char *password = "hongthanh2110";
const char *hostName = "esp32";

DacService dac1(DAC_CHANNEL_1);
DacService dac2(DAC_CHANNEL_2);

ComManager com(ssid, password, hostName);
ServoService servoService;
ScannerApp scannerApp(servoService, sharedData);
TransmitterApp txApp(com, sharedData);
SimulatorApp simulatorApp(sharedData);
ReceiverApp rxApp(sharedData);
SyncSignalApp syncApp(sharedData, dac1, dac2, simulatorApp);

void setup() {
  Serial.begin(Constant::SERIAL_BAUD_RATE);
  delay(Constant::SETUP_DELAY_MS);
  Serial.println("System starting up...");

  // Initialize drivers & local applications (networking is offloaded to Core 0)
  scannerApp.begin();
  txApp.begin();
  rxApp.begin();
  simulatorApp.begin();
  syncApp.begin();

  Serial.println("Drivers and Services initialized.");

  // Pin Task0 to Core 0: High-priority hardware Pulse Generation and Wi-Fi UDP
  // networking
  xTaskCreatePinnedToCore(
      [](void *param) {
        Serial.println("Transmitter Task (Core 0) started.");
        com.begin(); // Initialize UDP socket and WiFi on Core 0 for thread
                     // safety
        while (true) {
          txApp.run();
        }
      },
      "TxTask", Constant::TASK_STACK_SIZE, nullptr, Constant::TASK_PRIORITY,
      nullptr,
      0 // Pinned to Core 0
  );

  // Pin Scanner Task to Core 0 (Priority 5, lower than TxTask and RxTask)
  xTaskCreatePinnedToCore(
      [](void *param) {
        Serial.println("Scanner Task (Core 0) started.");
        while (true) {
          if (com.isServoEnabled()) {
            scannerApp.step();
          }
          vTaskDelay(pdMS_TO_TICKS(50)); // Step every 50ms independently
        }
      },
      "ScannerTask", 4096, nullptr,
      5, // Lower priority
      &sharedData.servoTaskHandle,
      0 // Pinned to Core 0
  );

  // Pin Task1 to Core 1: Critical real-time DSP pipelines and high-speed ADC
  // sampling
  xTaskCreatePinnedToCore(
      [](void *param) {
        Serial.println("Sync and DSP Task (Core 1) started.");
        while (true) {
          // 1. SyncSignalApp waits for trigger, prepares buffers, runs ADC/DAC sync loop
          syncApp.run();
          // 2. Once syncApp completes sampling, ReceiverApp processes DSP pipeline
          rxApp.run();
        }
      },
      "RxTask", Constant::TASK_STACK_SIZE, nullptr, Constant::TASK_PRIORITY,
      &sharedData.rxTaskHandle,
      1 // Pinned to Core 1
  );
}

void loop() {
  // Main loop does nothing as the work is executed asynchronously by Core 0/1
  // tasks.
  vTaskDelay(pdMS_TO_TICKS(Constant::LOOP_DELAY_MS));
}
