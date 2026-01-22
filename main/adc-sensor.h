#pragma once

#include "esp_err.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"

/**
 * @brief Initializes the ADC for voltage measurement.
 */
void adc_sensor_init(void);

/**
 * @brief Reads the battery voltage from the ADC or returns a simulated value.
 *
 * @return The calculated or simulated battery voltage as a float.
 */
float adc_sensor_read_voltage(void);
