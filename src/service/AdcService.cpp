#include "AdcService.h"
#include "../driver/AdcSignal.h"
#include <Constant.hpp>

// Inline assembly to read CPU cycles (ccount register)
static inline uint32_t get_ccount() {
    uint32_t ccount;
    asm volatile("rsr %0, ccount" : "=r"(ccount));
    return ccount;
}

void AdcService::init() {
    AdcSignal::init();
}

void AdcService::sampleBuffer(uint16_t* buffer, size_t size) {
    uint32_t cpu_freq_mhz = ESP.getCpuFreqMHz();
    // Cycles per sample = CPU frequency * cycle factor
    uint32_t cycles_per_sample = (uint32_t)(cpu_freq_mhz * Constant::CPU_CYCLES_PER_SAMPLE_FACTOR);

    uint32_t start_cycles = get_ccount();
    bool stuck = false;
    
    for (size_t i = 0; i < size; ++i) {
        uint32_t target_cycles = start_cycles + i * cycles_per_sample;
        while ((int32_t)(get_ccount() - target_cycles) < 0) {
            // Spin until timing is reached (overflow-safe)
        }
        
        uint16_t val = AdcSignal::readRaw();
        if (val == 0xFFFF) {
            stuck = true;
            // Pad the remaining samples with 0 to avoid garbage data
            for (size_t j = i; j < size; ++j) {
                buffer[j] = 0;
            }
            break;
        }
        
        // Digital calibration: Scale to align baseline to 1.65V,
        // while preserving 0V minimum and scaling maximum cleanly.
        uint32_t calibrated = (uint32_t)(val * Constant::SAMPLING_CALIBRATION_FACTOR);
        if (calibrated > Constant::ADC_RESOLUTION_MAX) calibrated = Constant::ADC_RESOLUTION_MAX;
        buffer[i] = (uint16_t)calibrated;
    }
}
