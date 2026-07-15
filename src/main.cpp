#include <SharedSonarData.h>
#include "app/ReceiverApp.hpp"
#include "app/ScannerApp.h"
#include "app/TransmitterApp.h"
#include "app/SyncSignalApp.h"
#include "service/ComManager.h"
#include "service/ServoService.h"
#include <Arduino.h>
#include <Constant.hpp>

// Shared data block between Core 0 and Core 1
SharedSonarData sharedData = {
    .triggerTx = false,
    .processingDone = false,
    .adcReady = false,
    .adcBuffer1 = {0},
    .spinlock = portMUX_INITIALIZER_UNLOCKED,
    .rxTaskHandle = nullptr,
    .servoTaskHandle = nullptr,
    .rxCore0TaskHandle = nullptr,
    .waveSendTaskHandle = nullptr,
    .txBuffer = {0},
    .txPulseLen = 0,
    .simDelaySamples = 400, // Default to 400 samples delay (~2.5ms delay, simulating ~1.8 meters range)
    .simEnabled = true,     // Simulator enabled by default
    .servoAngle = 0,
    .angleUpdated = false,
    .targetRange = 0,
    .targetStrength = 0,
    .targetDetected = false,
    .streamMode = 0,
    .accumulatedDataReady = false,
    .requestServoStep = false,
    .waveSendBuffer = {0},
    .waveSendAngle = 0,
    .waveSendReady = false,
    .peakIndexForVelocity = -1,
    .velocityRequested = false,
    .pulseIndex = 0
};

// WiFi SSID, password and MDNS hostname
const char *ssid = "Noel";
const char *password = "hongthanh2110";
const char *hostName = "esp32";

DacService dac1(DAC_CHANNEL_1);
DacService dac2(DAC_CHANNEL_2);

AdcSignal adc1(ADC1_CHANNEL_4);
AdcSignal adc2(ADC1_CHANNEL_5);

ComManager com(ssid, password, hostName);
ServoService servoService;
ScannerApp scannerApp(servoService, sharedData);
ReceiverApp rxApp1(sharedData, sharedData.adcBuffer1, 0);
ReceiverApp rxApp2(sharedData, sharedData.adcBuffer2, 1);
TransmitterApp txApp(com, sharedData);
SimulatorApp simulatorApp(sharedData);
SyncSignalApp syncApp(sharedData, dac1, dac2, simulatorApp, adc1, adc2);

void setup() {
  Serial.begin(Constant::SERIAL_BAUD_RATE);
  delay(Constant::SETUP_DELAY_MS);
  Serial.println("System starting up...");

  // Initialize drivers & local applications (networking is offloaded to Core 0)
  scannerApp.begin();
  txApp.begin();
  rxApp1.begin();
  rxApp1.setComManager(com); // Set ComManager reference on rxApp1 for Core 1 sending
  rxApp2.begin();
  rxApp2.setComManager(com);
  simulatorApp.begin();
  syncApp.begin();

  Serial.println("Drivers and Services initialized.");

  // Pin Task0 to Core 0: High-priority hardware Pulse Generation and UDP command polling
  xTaskCreatePinnedToCore(
      [](void *param) {
        Serial.println("Transmitter Task (Core 0) started.");
        com.begin(); // Initialize UDP socket and WiFi on Core 0 for thread safety
        while (true) {
          txApp.run();
        }
      },
      "TxTask", 4096, nullptr, Constant::TASK_PRIORITY,
      nullptr,
      0 // Pinned to Core 0
  );

  // Pin Scanner Task to Core 0 (Priority 5, lower than TxTask and RxTask)
  xTaskCreatePinnedToCore(
      [](void *param) {
        Serial.println("Scanner Task (Core 0) started.");
        while (true) {
          bool stepRequested = false;
          bool streaming = com.isStreaming();

          if (streaming) {
            taskENTER_CRITICAL(&sharedData.spinlock);
            if (sharedData.requestServoStep) {
              stepRequested = true;
            }
            taskEXIT_CRITICAL(&sharedData.spinlock);
          }

          if (stepRequested) {
            if (com.isServoEnabled()) {
              scannerApp.step();
            }
            taskENTER_CRITICAL(&sharedData.spinlock);
            sharedData.requestServoStep = false;
            taskEXIT_CRITICAL(&sharedData.spinlock);
            vTaskDelay(pdMS_TO_TICKS(5));
          } else {
            if (com.isServoEnabled() && !streaming) {
              // When not streaming, step independently on a timer (e.g. every 40ms)
              scannerApp.step();
              vTaskDelay(pdMS_TO_TICKS(40));
            } else {
              vTaskDelay(pdMS_TO_TICKS(10)); // Poll delay when idle or waiting
            }
          }
        }
      },
      "ScannerTask", 4096, nullptr,
      5, // Lower priority
      &sharedData.servoTaskHandle,
      0 // Pinned to Core 0
  );

  // Pin Task1 to Core 1: Critical real-time DSP pipelines, sampling, and UDP transmission
  xTaskCreatePinnedToCore(
      [](void *param) {
        Serial.println("Sync and DSP Task (Core 1) started.");
        while (true) {
          // 1. SyncSignalApp waits for trigger, prepares buffers, runs ADC/DAC sync loop
          syncApp.run();
          // 2. Once syncApp completes sampling, ReceiverApp processes DSP pipeline and sends UDP
          rxApp1.run();
          rxApp2.run();
        }
      },
      "RxTask", 4096, nullptr, Constant::TASK_PRIORITY,
      &sharedData.rxTaskHandle,
      1 // Pinned to Core 1
  );
}

void loop() {
  // Main loop does nothing as the work is executed asynchronously by Core 0/1 tasks.
  vTaskDelay(pdMS_TO_TICKS(Constant::LOOP_DELAY_MS));
}
