#include "AdcSignal.h"
#include <driver/adc.h>
#include <soc/sens_reg.h>
#include <soc/syscon_reg.h>

AdcSignal::AdcSignal(adc1_channel_t channel) : _channel(channel) {}

void AdcSignal::init() {
    // Configure resolution for ADC1
    adc1_config_width(ADC_WIDTH_BIT_12);
    
    // Configure attenuation for this specific channel
    adc1_config_channel_atten(_channel, ADC_ATTEN_DB_12);

    // Call standard driver read once to ensure this channel's routing & power are initialized
    adc1_get_raw(_channel);

    // ADC Sample-and-Hold Timing Optimization
    #ifdef SENS_SAR1_SAMPLE_CYCLE
    REG_SET_FIELD(SENS_SAR_READ_CTRL_REG, SENS_SAR1_SAMPLE_CYCLE, 9);
    #endif

    REG_SET_BIT(SENS_SAR_MEAS_START1_REG, SENS_SAR1_EN_PAD_FORCE);       // Force pad routing
}

uint16_t AdcSignal::readRaw() {
    startConversion();
    return readResult();
}

void AdcSignal::startConversion() {
    REG_SET_FIELD(SENS_SAR_MEAS_START1_REG, SENS_SAR1_EN_PAD, (1 << _channel));
    REG_CLR_BIT(SENS_SAR_MEAS_START1_REG, SENS_MEAS1_START_SAR);
    REG_SET_BIT(SENS_SAR_MEAS_START1_REG, SENS_MEAS1_START_SAR);
}

uint16_t AdcSignal::readResult() {
    uint32_t spinCount = 0;
    while (!(REG_READ(SENS_SAR_MEAS_START1_REG) & SENS_MEAS1_DONE_SAR)) {
        spinCount++;
        if (spinCount > 5000) {
            return 0xFFFF;
        }
    }
    return REG_READ(SENS_SAR_MEAS_START1_REG) & 0xFFF;
}
