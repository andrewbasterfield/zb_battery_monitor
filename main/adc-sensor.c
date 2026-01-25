#include "esp_log.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"

#include "adc-sensor.h"

#define TAG "ADC_SENSOR"

// --- Simulation Configuration ---
// Set to 1 to use simulated voltage for testing, 0 to use the real ADC.
#define USE_SIMULATED_VOLTAGE 1

// --- Hardware Configuration ---
#define VOLTAGE_DIVIDER_CHANNEL ADC_CHANNEL_0  // ADC channel connected to the voltage divider (GPIO0)
#define SAMPLE_COUNT 64                        // Number of ADC samples to average for each measurement

// --- Voltage Divider Configuration ---
// Adjust these values based on the resistors used in the voltage divider circuit.
// The goal is to scale the battery voltage (e.g., 0-16V) down to the ESP32's ADC input range (e.g., 0-3.3V).
// Example: R1=47kΩ, R2=10kΩ -> Ratio = (R1+R2)/R2 = 57k/10k = 5.7
#define VOLTAGE_DIVIDER_RATIO 5.7f // The division ratio of the voltage divider circuit
#define ADC_VREF 3300              // ADC reference voltage in millivolts

// --- Global Variables ---
static adc_oneshot_unit_handle_t adc1_handle;      // Handle for the ADC one-shot unit
static adc_cali_handle_t adc1_cali_handle = NULL;  // Handle for ADC calibration data
static bool do_calibration = false;                // Flag indicating if ADC calibration is available and should be used

#if USE_SIMULATED_VOLTAGE
static float simulated_voltage = 14.0f;            // Global variable for voltage simulation
#endif

/**
 * @brief Initializes the ADC calibration scheme.
 *
 * This function attempts to create a calibration scheme for the ADC to improve
 * measurement accuracy. It tries different calibration methods supported by the hardware
 * (Curve Fitting, Line Fitting).
 *
 * @param unit The ADC unit to calibrate.
 * @param channel The ADC channel to calibrate.
 * @param atten The ADC attenuation to use for calibration.
 * @param out_handle Pointer to store the resulting calibration handle.
 * @return true if calibration was successful, false otherwise.
 */
static bool adc_calibration_init(adc_unit_t unit, adc_channel_t channel, adc_atten_t atten, adc_cali_handle_t *out_handle)
{
    adc_cali_handle_t handle = NULL;
    esp_err_t ret = ESP_FAIL;
    bool calibrated = false;

    // Attempt to use Curve Fitting calibration if supported.
#if ADC_CALI_SCHEME_CURVE_FITTING_SUPPORTED
    if (!calibrated) {
        ESP_LOGI(TAG, "calibration scheme version is %s", "Curve Fitting");
        adc_cali_curve_fitting_config_t cali_config = {
            .unit_id = unit,
            .chan = channel,
            .atten = atten,
            .bitwidth = ADC_BITWIDTH_DEFAULT,
        };
        ret = adc_cali_create_scheme_curve_fitting(&cali_config, &handle);
        if (ret == ESP_OK) {
            calibrated = true;
        }
    }
#endif

    // Attempt to use Line Fitting calibration if supported and Curve Fitting failed.
#if ADC_CALI_SCHEME_LINE_FITTING_SUPPORTED
    if (!calibrated) {
        ESP_LOGI(TAG, "calibration scheme version is %s", "Line Fitting");
        adc_cali_line_fitting_config_t cali_config = {
            .unit_id = unit,
            .atten = atten,
            .bitwidth = ADC_BITWIDTH_DEFAULT,
        };
        ret = adc_cali_create_scheme_line_fitting(&cali_config, &handle);
        if (ret == ESP_OK) {
            calibrated = true;
        }
    }
#endif

    *out_handle = handle;
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "Calibration Success");
    } else if (ret == ESP_ERR_NOT_SUPPORTED || !calibrated) {
        ESP_LOGW(TAG, "eFuse not burnt, skip software calibration");
    } else {
        ESP_LOGE(TAG, "Invalid arg or no memory");
    }

    return calibrated;
}

/**
 * @brief Initializes the ADC for voltage measurement.
 */
void adc_sensor_init(void)
{
    //-------------ADC1 Init---------------//
    adc_oneshot_unit_init_cfg_t init_config1 = {
        .unit_id = ADC_UNIT_1,
    };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_config1, &adc1_handle));

    //-------------ADC1 Config---------------//
    adc_oneshot_chan_cfg_t config = {
        .bitwidth = ADC_BITWIDTH_DEFAULT,
        .atten = ADC_ATTEN_DB_12,
    };
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc1_handle, VOLTAGE_DIVIDER_CHANNEL, &config));

    //-------------ADC1 Calibration Init---------------//
    do_calibration = adc_calibration_init(ADC_UNIT_1, VOLTAGE_DIVIDER_CHANNEL, ADC_ATTEN_DB_12, &adc1_cali_handle);
}


/**
 * @brief Reads the battery voltage from the ADC or returns a simulated value.
 *
 * FOR DEVELOPMENT: This function can return a simulated voltage to allow testing
 * without physical hardware. To use the real ADC, set USE_SIMULATED_VOLTAGE to 0.
 *
 * @return The calculated or simulated battery voltage as a float.
 */
float adc_sensor_read_voltage(void)
{
#if USE_SIMULATED_VOLTAGE
    // --- SIMULATED VOLTAGE (for testing) ---
    simulated_voltage -= 0.1f;
    if (simulated_voltage < 11.5f) {
        simulated_voltage = 15.0f; // Reset to a high voltage
    }
    ESP_LOGI(TAG, "Using simulated voltage: %.2f V", simulated_voltage);
    return simulated_voltage;
#else
    // --- REAL ADC READING ---
    uint32_t adc_reading = 0;
    int adc_raw;

    for (int i = 0; i < SAMPLE_COUNT; i++) {
        ESP_ERROR_CHECK(adc_oneshot_read(adc1_handle, VOLTAGE_DIVIDER_CHANNEL, &adc_raw));
        adc_reading += adc_raw;
    }
    adc_reading /= SAMPLE_COUNT;

    int voltage_mv = 0;
    if (do_calibration) {
        ESP_ERROR_CHECK(adc_cali_raw_to_voltage(adc1_cali_handle, adc_reading, &voltage_mv));
    } else {
        voltage_mv = (adc_reading * ADC_VREF) / 4095;
    }

    float battery_voltage = (voltage_mv * VOLTAGE_DIVIDER_RATIO) / 1000.0f;

    return battery_voltage;
#endif
}
