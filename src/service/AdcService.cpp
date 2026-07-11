#include "AdcService.h"
#include "DacService.h"
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

void AdcService::sampleBuffer(uint16_t* buffer, size_t size, const uint8_t* txBuffer, size_t txPulseLen, volatile bool& adcReady) {
    uint32_t cpu_freq_mhz = ESP.getCpuFreqMHz();
    // Cycles per sample = CPU frequency * cycle factor
    uint32_t cycles_per_sample = (uint32_t)(cpu_freq_mhz * Constant::CPU_CYCLES_PER_SAMPLE_FACTOR);

    // Read start cycle count first to set the absolute reference
    uint32_t start_cycles = get_ccount();

    // Start the first conversion for i = 0
    AdcSignal::startConversion();

    // Signal Core 0 that we have started the first conversion and are ready!
    adcReady = true;

    bool stuck = false;
    
    for (size_t i = 0; i < size; ++i) {
        uint32_t target_cycles = start_cycles + i * cycles_per_sample;
        while ((int32_t)(get_ccount() - target_cycles) < 0) {
            // Spin until timing is reached (overflow-safe)
        }
        
        // 1. Read result of the conversion that was triggered in the previous step
        uint16_t val = AdcSignal::readResult();
        
        // 2. Trigger the next conversion immediately so it runs in parallel with the rest of the loop
        if (i < size - 1) {
            AdcSignal::startConversion();
        }

        // 3. Write to DAC for synchronous pulse output
        if (i < txPulseLen) {
            DacService::writeSample(txBuffer[i]);
        } else if (i == txPulseLen) {
            // Immediately restore bias level right after pulse transmission
            DacService::writeDCBias();
        }

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
