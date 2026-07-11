#include "AdcSignal.h"
#include <driver/adc.h>
#include <soc/sens_reg.h>
#include <soc/syscon_reg.h>

void AdcSignal::init() {
    // Configure GPIO 32 as ADC1 Channel 4 (12-bit resolution)
    adc1_config_width(ADC_WIDTH_BIT_12);
    
    // Fix deprecation: Use ADC_ATTEN_DB_12 (equivalent to 11dB in older SDKs)
    adc1_config_channel_atten(ADC1_CHANNEL_4, ADC_ATTEN_DB_12);

    // CRITICAL: Call standard driver read once to ensure ADC is powered on, 
    // clocks are initialized, and internal hardware routes are fully configured.
    adc1_get_raw(ADC1_CHANNEL_4);

    // ADC Sample-and-Hold Timing Optimization:
    // Maximize the acquisition sample cycles to ensure the S&H capacitor fully tracks the 1.65V bias.
    #ifdef SENS_SAR1_SAMPLE_CYCLE
    REG_SET_FIELD(SENS_SAR_READ_CTRL_REG, SENS_SAR1_SAMPLE_CYCLE, 9);
    #endif

    // Configure the RTC controller to route conversion triggers to ADC1 Channel 4.
    REG_SET_FIELD(SENS_SAR_MEAS_START1_REG, SENS_SAR1_EN_PAD, (1 << 4)); // Channel 4
    REG_SET_BIT(SENS_SAR_MEAS_START1_REG, SENS_SAR1_EN_PAD_FORCE);       // Force pad routing
}

uint16_t AdcSignal::readRaw() {
    // Manually trigger the ADC1 conversion on channel 4 via SENS registers (RTC controller)
    // Clear start bit
    REG_CLR_BIT(SENS_SAR_MEAS_START1_REG, SENS_MEAS1_START_SAR);
    // Set start bit to initiate conversion
    REG_SET_BIT(SENS_SAR_MEAS_START1_REG, SENS_MEAS1_START_SAR);
    
    // Wait for the done bit to go high, with a safety timeout to prevent infinite hangs
    uint32_t spinCount = 0;
    while (!(REG_READ(SENS_SAR_MEAS_START1_REG) & SENS_MEAS1_DONE_SAR)) {
        spinCount++;
        if (spinCount > 5000) {
            // Log once in a while or handle stuck hardware
            return 0xFFFF; 
        }
    }
    
    // Return the 12-bit data (bits 0-11)
    return REG_READ(SENS_SAR_MEAS_START1_REG) & 0xFFF;
}

void AdcSignal::startConversion() {
    REG_CLR_BIT(SENS_SAR_MEAS_START1_REG, SENS_MEAS1_START_SAR);
    REG_SET_BIT(SENS_SAR_MEAS_START1_REG, SENS_MEAS1_START_SAR);
}

uint16_t AdcSignal::readResult() {
    // Check if the done bit is high. It should be high since conversion is fast,
    // but in case it's not, we do a very short spin wait.
    uint32_t spinCount = 0;
    while (!(REG_READ(SENS_SAR_MEAS_START1_REG) & SENS_MEAS1_DONE_SAR)) {
        spinCount++;
        if (spinCount > 5000) {
            return 0xFFFF;
        }
    }
    return REG_READ(SENS_SAR_MEAS_START1_REG) & 0xFFF;
}
