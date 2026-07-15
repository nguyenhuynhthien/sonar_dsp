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
}

void SyncSignalService::sampleAndPlay(uint16_t* adcDestBuffer1, uint16_t* adcDestBuffer2, size_t size, 
                                     const uint8_t* dac1SrcBuffer, const uint8_t* dac2SrcBuffer, 
                                     DacService& dac1, DacService& dac2,
                                     AdcSignal& adc1, AdcSignal& adc2) {
    uint32_t cpu_freq_mhz = ESP.getCpuFreqMHz();
    uint32_t cycles_per_sample = (uint32_t)(cpu_freq_mhz * Constant::CPU_CYCLES_PER_SAMPLE_FACTOR);

    uint32_t start_cycles = get_ccount();

    // Start conversion on Channel 4 first
    adc1.startConversion();

    for (size_t i = 0; i < size; ++i) {
        uint32_t target_cycles = start_cycles + i * cycles_per_sample;
        while ((int32_t)(get_ccount() - target_cycles) < 0) {
            // Spin until timing is reached
        }
        
        // 1. Read result of Channel 4 conversion
        uint16_t val1 = adc1.readResult();
        
        // 2. Start conversion on Channel 5 immediately
        adc2.startConversion();
        
        // 3. Write to DAC1 and DAC2 while Channel 5 is converting
        if (dac1SrcBuffer != nullptr) {
            dac1.writeSample(dac1SrcBuffer[i]);
        }
        if (dac2SrcBuffer != nullptr) {
            dac2.writeSample(dac2SrcBuffer[i]);
        }
        
        // 4. Read result of Channel 5 conversion
        uint16_t val2 = adc2.readResult();
        
        // 5. Start conversion on Channel 4 for the next loop iteration (if not at the end)
        if (i < size - 1) {
            adc1.startConversion();
        }

        // Calibrate and store channel 1 (val1)
        if (val1 == 0xFFFF) {
            val1 = 0;
        }
        uint32_t calibrated1 = (uint32_t)(val1 * Constant::SAMPLING_CALIBRATION_FACTOR);
        if (calibrated1 > Constant::ADC_RESOLUTION_MAX) calibrated1 = Constant::ADC_RESOLUTION_MAX;
        adcDestBuffer1[i] = (uint16_t)calibrated1;

        // Calibrate and store channel 2 (val2)
        if (val2 == 0xFFFF) {
            val2 = 0;
        }
        uint32_t calibrated2 = (uint32_t)(val2 * Constant::SAMPLING_CALIBRATION_FACTOR);
        if (calibrated2 > Constant::ADC_RESOLUTION_MAX) calibrated2 = Constant::ADC_RESOLUTION_MAX;
        adcDestBuffer2[i] = (uint16_t)calibrated2;
    }
}
