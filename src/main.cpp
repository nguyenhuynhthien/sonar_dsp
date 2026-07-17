#include <SharedSonarData.h>
#include "app/ReceiverApp.hpp"
#include "app/ScannerApp.h"
#include "app/TransmitterApp.h"
#include "app/SyncSignalApp.h"
#include "app/CombineReceiverApp.h"
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
    .pulseIndex = 0,
    .txPeriodMs = 12,
    .channelL_I = {nullptr},
    .channelL_Q = {nullptr},
    .channelR_I = {nullptr},
    .channelR_Q = {nullptr},
    .sharedDemodI = nullptr,
    .sharedDemodQ = nullptr
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
CombineReceiverApp combineRxApp(sharedData);

void setup() {
  Serial.begin(Constant::SERIAL_BAUD_RATE);
  delay(Constant::SETUP_DELAY_MS);
  Serial.println("System starting up...");
  Serial.printf("Free heap at startup: %d bytes\n", ESP.getFreeHeap());

  // Initialize WiFi/UDP first so that the network driver gets priority on heap allocations
  com.begin();
  Serial.printf("Free heap after WiFi init: %d bytes\n", ESP.getFreeHeap());

  // Dynamically allocate shared matrices and buffers on the heap to prevent DRAM overflow
  bool allocSuccess = true;
  for (int i = 0; i < 8; ++i) {
    sharedData.channelL_I[i] = new (std::nothrow) int16_t[Constant::ADC_SAMPLES];
    sharedData.channelL_Q[i] = new (std::nothrow) int16_t[Constant::ADC_SAMPLES];
    sharedData.channelR_I[i] = new (std::nothrow) int16_t[Constant::ADC_SAMPLES];
    sharedData.channelR_Q[i] = new (std::nothrow) int16_t[Constant::ADC_SAMPLES];
    if (!sharedData.channelL_I[i] || !sharedData.channelL_Q[i] || !sharedData.channelR_I[i] || !sharedData.channelR_Q[i]) {
      allocSuccess = false;
    }
  }
  sharedData.sharedDemodI = new (std::nothrow) int16_t[Constant::ADC_SAMPLES];
  sharedData.sharedDemodQ = new (std::nothrow) int16_t[Constant::ADC_SAMPLES];
  if (!sharedData.sharedDemodI || !sharedData.sharedDemodQ) {
    allocSuccess = false;
  }
  Serial.printf("Free heap after matrix allocation: %d bytes (Allocation success: %s)\n", ESP.getFreeHeap(), allocSuccess ? "YES" : "NO");

  // Initialize drivers & local applications (networking is offloaded to Core 0)
  scannerApp.begin();
  txApp.begin();
  rxApp1.begin();
  rxApp1.setComManager(com); // Set ComManager reference on rxApp1 for Core 1 sending
  rxApp2.begin();
  rxApp2.setComManager(com);
  simulatorApp.begin();
  syncApp.begin();
  combineRxApp.begin();
  combineRxApp.setComManager(com);

  Serial.println("Drivers and Services initialized.");

  // Pin Task0 to Core 0: High-priority hardware Pulse Generation and UDP command polling
  BaseType_t t1 = xTaskCreatePinnedToCore(
      [](void *param) {
        Serial.println("Transmitter Task (Core 0) started.");
        while (true) {
          txApp.run();
        }
      },
      "TxTask", 4096, nullptr, Constant::TASK_PRIORITY,
      nullptr,
      0 // Pinned to Core 0
  );
  Serial.printf("TxTask creation: %s\n", (t1 == pdPASS) ? "SUCCESS" : "FAILED");

  // Pin Scanner Task to Core 0 (Priority 5, lower than TxTask and RxTask)
  BaseType_t t2 = xTaskCreatePinnedToCore(
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
  Serial.printf("ScannerTask creation: %s\n", (t2 == pdPASS) ? "SUCCESS" : "FAILED");

  // Pin Task1 to Core 1: Critical real-time DSP pipelines, sampling, and UDP transmission
  BaseType_t t3 = xTaskCreatePinnedToCore(
      [](void *param) {
        Serial.println("Sync and DSP Task (Core 1) started.");
        while (true) {
          // 1. SyncSignalApp waits for trigger, prepares buffers, runs ADC/DAC sync loop
          syncApp.run();
          // 2. Once syncApp completes sampling, ReceiverApp processes DSP pipeline and sends UDP
          rxApp1.run();
          rxApp2.run();
          combineRxApp.run();
        }
      },
      "RxTask", 4096, nullptr, Constant::TASK_PRIORITY,
      &sharedData.rxTaskHandle,
      1 // Pinned to Core 1
  );
  Serial.printf("RxTask creation: %s\n", (t3 == pdPASS) ? "SUCCESS" : "FAILED");
}

void loop() {
  // Main loop does nothing as the work is executed asynchronously by Core 0/1 tasks.
  vTaskDelay(pdMS_TO_TICKS(Constant::LOOP_DELAY_MS));
}
