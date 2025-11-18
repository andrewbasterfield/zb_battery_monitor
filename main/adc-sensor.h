#pragma once

#include "esp_err.h"

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
