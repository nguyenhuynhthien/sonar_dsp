#ifndef CONSTANT_HPP
#define CONSTANT_HPP

#include <Arduino.h>

namespace Constant {
    // Sonar & DSP parameters
    constexpr size_t ADC_SAMPLES = 2048;
    constexpr size_t FILTER_COEFFS_LEN = 32;
    constexpr double CENTER_FREQ = 40000.0;     // 40 kHz
    constexpr double SAMPLE_RATE = 160000.0;    // 160 kHz
    constexpr uint16_t ADC_RESOLUTION_MAX = 4095;
    constexpr float ADC_DC_OFFSET = 2048.0f;
    constexpr uint8_t DAC_DC_BIAS = 127;
    constexpr size_t SLOW_TIME_LEN = 16;
    constexpr float CPU_CYCLES_PER_SAMPLE_FACTOR = 6.25f;
    constexpr float SAMPLING_CALIBRATION_FACTOR = 1.10f;

    // Pre-calculated single pulse waveform (8 cycles of 4 samples/cycle at 40kHz center frequency / 160kHz sample rate)
    constexpr uint8_t SINGLE_PULSE_WAVE[FILTER_COEFFS_LEN] = {
        127, 254, 127, 0,   127, 254, 127, 0,
        127, 254, 127, 0,   127, 254, 127, 0,
        127, 254, 127, 0,   127, 254, 127, 0,
        127, 254, 127, 0,   127, 254, 127, 0
    };

    // Timing & Timeouts
    constexpr uint32_t RX_POLL_DELAY_US = 50;
    constexpr uint32_t TX_RESPONSE_TIMEOUT_MS = 100;
    constexpr uint32_t TX_PERIOD_MS = 45;
    constexpr uint32_t TX_IDLE_DELAY_MS = 20;
    constexpr uint32_t WIFI_CONNECT_TIMEOUT_LIMIT = 20; // 20 iterations of 500ms = 10 seconds
    constexpr uint32_t WIFI_CONNECT_DELAY_MS = 500;
    constexpr uint32_t WIFI_ARP_DELAY_MS = 300;
    constexpr uint32_t UDP_PACE_DELAY_US = 1500;

    // Network
    constexpr uint16_t DEFAULT_PORT = 8080;
    constexpr size_t UDP_BUFFER_SIZE = 32;
    constexpr size_t CHUNK_SAMPLES = 512;
    constexpr size_t CHUNKS_PER_FRAME = 4;
    
    // Tasks & System
    constexpr uint32_t SERIAL_BAUD_RATE = 115200;
    constexpr uint32_t SETUP_DELAY_MS = 1000;
    constexpr uint32_t TASK_STACK_SIZE = 8192;
    constexpr uint32_t TASK_PRIORITY = 10;
    constexpr uint32_t LOOP_DELAY_MS = 1000;
}

#endif // CONSTANT_HPP
