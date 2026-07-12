#include "SyncSignalService.h"
#include "../driver/AdcSignal.h"
#include <Constant.hpp>

// Inline assembly to read CPU cycles (ccount register)
static inline uint32_t get_ccount() {
    uint32_t ccount;
    asm volatile("rsr %0, ccount" : "=r"(ccount));
    return ccount;
}

void SyncSignalService::init() {
    AdcSignal::init();
}

void SyncSignalService::sampleAndPlay(uint16_t* adcDestBuffer, size_t size, 
                                     const uint8_t* dac1SrcBuffer, const uint8_t* dac2SrcBuffer, 
                                     DacService& dac1, DacService& dac2) {
    uint32_t cpu_freq_mhz = ESP.getCpuFreqMHz();
    // Cycles per sample = CPU frequency * cycle factor
    uint32_t cycles_per_sample = (uint32_t)(cpu_freq_mhz * Constant::CPU_CYCLES_PER_SAMPLE_FACTOR);

    // Read start cycle count first to set the absolute reference
    uint32_t start_cycles = get_ccount();

    // Start the first conversion for i = 0
    AdcSignal::startConversion();

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

        // 3. Write to DAC1 if source buffer is provided
        if (dac1SrcBuffer != nullptr) {
            dac1.writeSample(dac1SrcBuffer[i]);
        }

        // 4. Write to DAC2 if source buffer is provided
        if (dac2SrcBuffer != nullptr) {
            dac2.writeSample(dac2SrcBuffer[i]);
        }

        if (val == 0xFFFF) {
            stuck = true;
            // Pad the remaining samples with 0 to avoid garbage data
            for (size_t j = i; j < size; ++j) {
                adcDestBuffer[j] = 0;
            }
            break;
        }
        
        // Digital calibration: Scale to align baseline to 1.65V
        uint32_t calibrated = (uint32_t)(val * Constant::SAMPLING_CALIBRATION_FACTOR);
        if (calibrated > Constant::ADC_RESOLUTION_MAX) calibrated = Constant::ADC_RESOLUTION_MAX;
        adcDestBuffer[i] = (uint16_t)calibrated;
    }
}
