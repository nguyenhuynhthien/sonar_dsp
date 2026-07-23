#ifndef CONSTANT_HPP
#define CONSTANT_HPP

#include <Arduino.h>

namespace Constant {
// Sonar & DSP parameters
constexpr size_t ADC_SAMPLES = 2048;
constexpr int FFT_WINDOW_SIZE = 15;
constexpr size_t FILTER_COEFFS_LEN = 8;
constexpr int ADC_CHANNEL_RX1 = 4;       // ADC1 Channel 4 (GPIO 32)
constexpr int ADC_CHANNEL_RX2 = 5;       // ADC1 Channel 5 (GPIO 33)
constexpr double CENTER_FREQ = 40000.0;  // 40 kHz
constexpr double SAMPLE_RATE = 160000.0; // 160 kHz
constexpr uint16_t ADC_RESOLUTION_MAX = 4095;
constexpr float ADC_DC_OFFSET = 2048.0f;
constexpr uint8_t DAC_DC_BIAS = 127;
constexpr size_t SLOW_TIME_LEN = 16;
constexpr size_t DOPPLER_FFT_LEN = 8;
constexpr float CPU_CYCLES_PER_SAMPLE_FACTOR = 6.25f;
constexpr float SAMPLING_CALIBRATION_FACTOR = 1.10f;
constexpr float SPEED_OF_SOUND = 343.0f; // Speed of sound in air in m/s

// Q15 fixed-point DSP constants
constexpr int Q15_SHIFT = 15;
constexpr int32_t COS_PI_4_Q15 = 23170; // cos(pi/4) = 0.7071 in Q15 format
constexpr int DEMOD_SAMPLE_PERIOD = 4; // Period of I/Q carrier modulation (4 samples per period)
constexpr int DEMOD_AVG_LEN = 4; // Moving average filter length
constexpr int16_t Q15_MAX = 32767;
constexpr int16_t Q15_MIN = -32768;
constexpr int16_t Q15_DAC_SCALE = 256;
constexpr int32_t MATCHED_FILTER_ROUND_OFFSET = 4096;
constexpr int MATCHED_FILTER_SHIFT = 13;
constexpr int64_t TARGET_THRESHOLD_SCALE_SQ = 1048576LL;


constexpr int32_t BASE_BARKER13_THRESHOLD = 120;
constexpr int32_t BASE_SINGLE_PULSE_THRESHOLD = 300;
constexpr int TX_LEAKAGE_BLANK_SAMPLES = 220;
constexpr int COMPRESSED_STREAM_SCALE = 3;

// Pre-calculated single pulse waveform (1 chip = 2 cycles of 4 samples/cycle = 8 samples)
constexpr uint8_t SINGLE_PULSE_WAVE[FILTER_COEFFS_LEN] = {
    127, 254, 127, 0, 127, 254, 127, 0};


// Pre-calculated Barker 13 waveform (13 chips, 2 cycles of 4 samples/cycle per
// chip = 8 samples/chip, total 104 samples)
constexpr size_t BARKER13_PULSE_LEN = 104;
constexpr uint8_t BARKER13_PULSE_WAVE[BARKER13_PULSE_LEN] = {
    127, 254, 127, 0,   127, 254, 127, 0,   127, 254, 127, 0,   127, 254, 127,
    0,   127, 254, 127, 0,   127, 254, 127, 0,   127, 254, 127, 0,   127, 254,
    127, 0,   127, 254, 127, 0,   127, 254, 127, 95,  127, 95,  127, 254, 127,
    0,   127, 254, 127, 0,   127, 254, 127, 0,   127, 159, 127, 159, 127, 0,
    127, 254, 127, 0,   127, 254, 127, 0,   127, 254, 127, 95,  127, 95,  127,
    254, 127, 0,   127, 159, 127, 159, 127, 0,   127, 254, 127, 95,  127, 95,
    127, 254, 127, 0,   127, 159, 127, 159, 127, 0,   127, 254, 127, 0};

// Timing & Timeouts
constexpr uint32_t RX_POLL_DELAY_US = 50;
constexpr uint32_t TX_RESPONSE_TIMEOUT_MS = 80;
constexpr int SERVO_STEP_DEG = 10;
constexpr uint32_t TX_IDLE_DELAY_MS = 20;
constexpr uint32_t PRI_SINGLE_MS = 30;
constexpr uint32_t PRI_BARKER13_MS = 30;
constexpr uint32_t TX_BUSY_WAIT_US = 100;
constexpr uint32_t TX_YIELD_THRESHOLD_US = 2000;
constexpr uint32_t WIFI_CONNECT_TIMEOUT_LIMIT =
    20; // 20 iterations of 500ms = 10 seconds
constexpr uint32_t WIFI_CONNECT_DELAY_MS = 500;
constexpr uint32_t WIFI_ARP_DELAY_MS = 300;
constexpr uint32_t UDP_PACE_DELAY_US = 250;

// DMA Configuration (I2S ADC)
constexpr size_t DMA_BUF_COUNT = 2;
constexpr size_t DMA_BUF_LEN = 512; // 2 x 512 = 1024 frames (or 512 for half-pulse)

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
} // namespace Constant

#endif // CONSTANT_HPP
