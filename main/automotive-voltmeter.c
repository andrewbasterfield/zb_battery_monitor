// Includes for Zigbee, FreeRTOS, ADC, NVS, and logging
#include "esp_zigbee_core.h"
#include "zcl/esp_zigbee_zcl_common.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include "nvs_flash.h"

#include "zigbee-protocol.h"

// --- Constants and Configuration ---

// Tag for logging
#define TAG "AUTOMOTIVE_VOLTMETER"

#define MANUFACTURER_NAME "\x09""ESPRESSIF"
#define MODEL_IDENTIFIER "\x07" CONFIG_IDF_TARGET

// --- Simulation Configuration ---
// Set to 1 to use simulated voltage for testing, 0 to use the real ADC.
#define USE_SIMULATED_VOLTAGE 1

// --- Hardware Configuration ---
#define VOLTAGE_DIVIDER_CHANNEL ADC_CHANNEL_0  // ADC channel connected to the voltage divider (GPIO0)
#define SAMPLE_COUNT 64                        // Number of ADC samples to average for each measurement
#define MEASUREMENT_INTERVAL_MS 5000           // Interval between voltage measurements in milliseconds

// --- Voltage Divider Configuration ---
// Adjust these values based on the resistors used in the voltage divider circuit.
// The goal is to scale the automotive voltage (e.g., 0-16V) down to the ESP32's ADC input range (e.g., 0-3.3V).
// Example: R1=47kΩ, R2=10kΩ -> Ratio = (R1+R2)/R2 = 57k/10k = 5.7
#define VOLTAGE_DIVIDER_RATIO 5.7f // The division ratio of the voltage divider circuit
#define ADC_VREF 3300              // ADC reference voltage in millivolts

// --- Zigbee Configuration ---
#define INSTALLCODE_POLICY_ENABLE false // Set to true to enable install code policy for joining, false to disable
#define ED_AGING_TIMEOUT ESP_ZB_ED_AGING_TIMEOUT_64MIN // Timeout for end device to be considered aged out by parent
#define ED_KEEP_ALIVE 3000              // Keep-alive interval for end device in milliseconds
#define HA_ESP_VOLTAGE_SENSOR_ENDPOINT 1 // Zigbee endpoint for this device
#define ESP_ZB_PRIMARY_CHANNEL_MASK ESP_ZB_TRANSCEIVER_ALL_CHANNELS_MASK // Scan all channels to find a network

// ZCL Attribute IDs for the Power Configuration Cluster
#define ESP_ZB_ZCL_ATTR_POWER_CONFIG_BATTERY_VOLTAGE_ID 0x0020
#define ESP_ZB_ZCL_ATTR_POWER_CONFIG_BATTERY_PERCENTAGE_REMAINING_ID 0x0021
#define ESP_ZB_ZCL_ATTR_POWER_CONFIG_BATTERY_ALARM_MASK_ID 0x0035
#define ESP_ZB_ZCL_ATTR_POWER_CONFIG_BATTERY_VOLTAGE_MIN_THRESHOLD_ID 0x0036

// --- Automotive Voltage Thresholds ---
// These thresholds define the different voltage states (in volts).
#define LOW_VOLTAGE_THRESHOLD 12.1f    // Below this, voltage is considered "low"
#define HIGH_VOLTAGE_THRESHOLD 14.8f   // Above this, voltage is considered "high" (e.g., alternator overcharging)
#define CRITICAL_LOW_VOLTAGE_THRESHOLD 11.8f // Below this, voltage is critically low

// --- Global Variables ---
static adc_oneshot_unit_handle_t adc1_handle;      // Handle for the ADC one-shot unit
static adc_cali_handle_t adc1_cali_handle = NULL;  // Handle for ADC calibration data
static bool do_calibration = false;                // Flag indicating if ADC calibration is available and should be used
static float current_voltage = 0.0f;               // Stores the most recently measured voltage
static uint8_t voltage_alarm_state = 0;            // Current alarm state: 0=OK, 1=LOW, 2=HIGH, 3=CRITICAL

static QueueHandle_t voltage_queue;                // FreeRTOS queue to pass voltage readings from measurement task to main loop
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
 *
 * This function configures the ADC one-shot unit and the specific channel
 * used for reading the voltage. It also initializes ADC calibration.
 */
static void adc_init(void)
{
    //-------------ADC1 Init---------------//
    // Configure the ADC unit.
    adc_oneshot_unit_init_cfg_t init_config1 = {
        .unit_id = ADC_UNIT_1,
    };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_config1, &adc1_handle));

    //-------------ADC1 Config---------------//
    // Configure the ADC channel, setting bitwidth and attenuation.
    adc_oneshot_chan_cfg_t config = {
        .bitwidth = ADC_BITWIDTH_DEFAULT,
        .atten = ADC_ATTEN_DB_12, // 12dB attenuation provides a measurement range of ~0-3.1V.
    };
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc1_handle, VOLTAGE_DIVIDER_CHANNEL, &config));

    //-------------ADC1 Calibration Init---------------//
    // Initialize calibration for the configured ADC unit and channel.
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
static float read_battery_voltage(void)
{
#if USE_SIMULATED_VOLTAGE
    // --- SIMULATED VOLTAGE (for testing) ---
    // This block cycles the voltage through high, normal, low, and critical states.
    simulated_voltage -= 0.5f;
    if (simulated_voltage < 11.5f) {
        simulated_voltage = 15.0f; // Reset to a high voltage
    }
    ESP_LOGI(TAG, "Using simulated voltage: %.2f V", simulated_voltage);
    return simulated_voltage;
#else
    // --- REAL ADC READING ---
    uint32_t adc_reading = 0;
    int adc_raw;

    // Take multiple samples and average them for better accuracy.
    for (int i = 0; i < SAMPLE_COUNT; i++) {
        ESP_ERROR_CHECK(adc_oneshot_read(adc1_handle, VOLTAGE_DIVIDER_CHANNEL, &adc_raw));
        adc_reading += adc_raw;
    }
    adc_reading /= SAMPLE_COUNT;

    // Convert the ADC reading to millivolts.
    int voltage_mv = 0;
    if (do_calibration) {
        // Use the calibration data if available for a more accurate conversion.
        ESP_ERROR_CHECK(adc_cali_raw_to_voltage(adc1_cali_handle, adc_reading, &voltage_mv));
    } else {
        // Fallback to a simple linear calculation if no calibration is available.
        voltage_mv = (adc_reading * ADC_VREF) / 4095; // For a 12-bit ADC.
    }

    // Apply the voltage divider ratio to get the actual battery voltage.
    float battery_voltage = (voltage_mv * VOLTAGE_DIVIDER_RATIO) / 1000.0f;

    return battery_voltage;
#endif
}

/**
 * @brief FreeRTOS task for periodic voltage measurement.
 *
 * This task continuously reads the battery voltage at a fixed interval
 * (`MEASUREMENT_INTERVAL_MS`) and sends the result to the `voltage_queue`.
 *
 * @param pvParameters Unused task parameters.
 */
static void voltage_measurement_task(void *pvParameters)
{
    while (1) {
        current_voltage = read_battery_voltage();
        // Send the measured voltage to the main loop via a queue.
        xQueueSend(voltage_queue, &current_voltage, 0);
        // Wait for the next measurement interval.
        vTaskDelay(pdMS_TO_TICKS(MEASUREMENT_INTERVAL_MS));
    }
}

// --- Zigbee Device Mode ---
// Set to 1 to configure the device as a Zigbee Router, 0 for an End Device.
// Routers can relay messages for other devices but consume more power.
// End Devices are low-power and sleep, but cannot relay messages.
#define ROUTER_MODE 0

/**
 * @brief Main application entry point.
 *
 * This function initializes all necessary components (NVS, ADC, Zigbee stack)
 * and contains the main application loop.
 */
void app_main(void)
{
    // --- Platform and NVS Initialization ---
    esp_zb_platform_config_t platform_config = {
        .radio_config = {
            .radio_mode = ZB_RADIO_MODE_NATIVE,
        },
        .host_config = {
            .host_connection_mode = ZB_HOST_CONNECTION_MODE_NONE,
        },
    };

    // Initialize Non-Volatile Storage (NVS) to store Zigbee network data.
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        // If NVS is corrupted or has an old version, erase and re-initialize it.
        ESP_LOGW(TAG, "NVS corrupted or new version found, erasing...");
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);
    ESP_LOGI(TAG, "NVS init result: %s", esp_err_to_name(ret));
    
    ESP_ERROR_CHECK(esp_zb_platform_config(&platform_config));

    // --- Hardware and Logging Initialization ---
    adc_init(); // Initialize the ADC.

    ESP_LOGI(TAG, "ESP32-H2 Automotive Voltmeter starting...");
    ESP_LOGI(TAG, "Voltage divider ratio: %.1f", VOLTAGE_DIVIDER_RATIO);
    ESP_LOGI(TAG, "Low voltage threshold: %.1fV", LOW_VOLTAGE_THRESHOLD);
    ESP_LOGI(TAG, "High voltage threshold: %.1fV", HIGH_VOLTAGE_THRESHOLD);
    ESP_LOGI(TAG, "Critical low threshold: %.1fV", CRITICAL_LOW_VOLTAGE_THRESHOLD);
    ESP_LOGI(TAG, "Device will attempt to join existing Zigbee network...");

    // --- Zigbee Stack Initialization ---
    // Set a custom IEEE address for the device (useful for debugging and static identification).
    uint8_t custom_ieee_addr[8] = {0xDE, 0xAD, 0xBE, 0xEF, 0x00, 0x00, 0x00, 0x01};
    esp_zb_set_long_address(custom_ieee_addr);

    // Print the device's IEEE address at startup for identification.
    esp_zb_ieee_addr_t ieee_addr;
    esp_zb_get_long_address(ieee_addr);
    ESP_LOGI(TAG, "Device IEEE Address: %02x:%02x:%02x:%02x:%02x:%02x:%02x:%02x",
        ieee_addr[7], ieee_addr[6], ieee_addr[5], ieee_addr[4],
        ieee_addr[3], ieee_addr[2], ieee_addr[1], ieee_addr[0]);
    
    // Configure the Zigbee device role (Router or End Device).
#if ROUTER_MODE
    ESP_LOGI(TAG, "Configuring as Zigbee Router");
    esp_zb_cfg_t network_config = {
        .esp_zb_role = ESP_ZB_DEVICE_TYPE_ROUTER,
        .install_code_policy = INSTALLCODE_POLICY_ENABLE,
        .nwk_cfg = {
            .zczr_cfg = {
                .max_children = 10,
            }
        },
    };
#else
    ESP_LOGI(TAG, "Configuring as End Device");
    esp_zb_cfg_t network_config = {
        .esp_zb_role = ESP_ZB_DEVICE_TYPE_ED,  // End Device - sleeps to save power.
        .install_code_policy = INSTALLCODE_POLICY_ENABLE,
        .nwk_cfg = {
            .zed_cfg = {
                .ed_timeout = ED_AGING_TIMEOUT,
                .keep_alive = ED_KEEP_ALIVE,
            },
        },
    };
#endif
    ESP_LOGI(TAG, "Initializing Zigbee stack...");
    esp_zb_init(&network_config);

    // Enable more detailed Zigbee logging for debugging purposes.
    esp_log_level_set("ESP_ZB", ESP_LOG_DEBUG);
    esp_log_level_set("ESP_ZB_ZCL", ESP_LOG_DEBUG);
    esp_log_level_set("ESP_ZB_ZDO", ESP_LOG_DEBUG);
    
#if ROUTER_MODE
    ESP_LOGI(TAG, "Zigbee stack initialized with role: Router");
#else
    ESP_LOGI(TAG, "Zigbee stack initialized with role: End Device");
#endif
    ESP_LOGI(TAG, "Install code policy: %s", INSTALLCODE_POLICY_ENABLE ? "enabled" : "disabled");
    ESP_LOGI(TAG, "Channel mask: 0x%08x", (unsigned int)ESP_ZB_PRIMARY_CHANNEL_MASK);
    ESP_LOGI(TAG, "ED timeout: %d, Keep alive: %d", ED_AGING_TIMEOUT, ED_KEEP_ALIVE);

    // --- Zigbee Cluster and Endpoint Configuration ---
    // Create a list of clusters that this device will support.
    esp_zb_cluster_list_t *clusters = esp_zb_zcl_cluster_list_create();

    // --- Basic Cluster (Server) ---
    // This cluster provides basic information about the device.
    esp_zb_attribute_list_t *basic_cluster_attributes = esp_zb_zcl_attr_list_create(ESP_ZB_ZCL_CLUSTER_ID_BASIC);
    
    
    // Add mandatory basic cluster attributes.
    uint8_t zcl_version = ESP_ZB_ZCL_BASIC_ZCL_VERSION_DEFAULT_VALUE;
    ESP_ERROR_CHECK(esp_zb_basic_cluster_add_attr(basic_cluster_attributes, ESP_ZB_ZCL_ATTR_BASIC_ZCL_VERSION_ID, &zcl_version));

#if ROUTER_MODE
    uint8_t power_source = 0x01; // Mains power
#else
    uint8_t power_source = 0x03; // Battery power
#endif
    ESP_ERROR_CHECK(esp_zb_basic_cluster_add_attr(basic_cluster_attributes, ESP_ZB_ZCL_ATTR_BASIC_POWER_SOURCE_ID, &power_source));
    
    // Add optional attributes like manufacturer and model.
    ESP_ERROR_CHECK(esp_zb_basic_cluster_add_attr(basic_cluster_attributes, ESP_ZB_ZCL_ATTR_BASIC_MANUFACTURER_NAME_ID, MANUFACTURER_NAME));
    ESP_ERROR_CHECK(esp_zb_basic_cluster_add_attr(basic_cluster_attributes, ESP_ZB_ZCL_ATTR_BASIC_MODEL_IDENTIFIER_ID, MODEL_IDENTIFIER));
    //char sw_version[] = "1.0.0";
    //ESP_ERROR_CHECK(esp_zb_basic_cluster_add_attr(basic_cluster_attributes, ESP_ZB_ZCL_ATTR_BASIC_SW_BUILD_ID, sw_version));

    ESP_ERROR_CHECK(esp_zb_cluster_list_add_basic_cluster(clusters, basic_cluster_attributes, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE));

    // --- Identify Cluster (Server) ---
    // This cluster allows the device to be identified (e.g., by blinking an LED).
    esp_zb_attribute_list_t *identify_cluster_attributes = esp_zb_zcl_attr_list_create(ESP_ZB_ZCL_CLUSTER_ID_IDENTIFY);
    uint16_t identify_time = 0;
    ESP_ERROR_CHECK(esp_zb_identify_cluster_add_attr(identify_cluster_attributes, ESP_ZB_ZCL_ATTR_IDENTIFY_IDENTIFY_TIME_ID, &identify_time));
    ESP_ERROR_CHECK(esp_zb_cluster_list_add_identify_cluster(clusters, identify_cluster_attributes, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE));

    // --- Power Configuration Cluster (Server) ---
    // This cluster is used to report battery information.
    esp_zb_attribute_list_t *power_config_cluster_attributes = esp_zb_zcl_attr_list_create(ESP_ZB_ZCL_CLUSTER_ID_POWER_CONFIG);

    // Add attributes for battery voltage, percentage, alarm mask, and threshold.
    uint8_t battery_voltage = 120; // Initial value (12.0V)
    ESP_ERROR_CHECK(esp_zb_power_config_cluster_add_attr(power_config_cluster_attributes, ESP_ZB_ZCL_ATTR_POWER_CONFIG_BATTERY_VOLTAGE_ID, &battery_voltage));

    uint8_t battery_percent = 100; // Initial value
    ESP_ERROR_CHECK(esp_zb_power_config_cluster_add_attr(power_config_cluster_attributes, ESP_ZB_ZCL_ATTR_POWER_CONFIG_BATTERY_PERCENTAGE_REMAINING_ID, &battery_percent));

    uint8_t alarm_mask = 0; // No alarms initially.
    ESP_ERROR_CHECK(esp_zb_power_config_cluster_add_attr(power_config_cluster_attributes, ESP_ZB_ZCL_ATTR_POWER_CONFIG_BATTERY_ALARM_MASK_ID, &alarm_mask));

    uint8_t battery_low_threshold = 10 * CRITICAL_LOW_VOLTAGE_THRESHOLD; // Set low threshold based on config.
    ESP_ERROR_CHECK(esp_zb_power_config_cluster_add_attr(power_config_cluster_attributes, ESP_ZB_ZCL_ATTR_POWER_CONFIG_BATTERY_VOLTAGE_MIN_THRESHOLD_ID, &battery_low_threshold));

    ESP_ERROR_CHECK(esp_zb_cluster_list_add_power_config_cluster(clusters, power_config_cluster_attributes, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE));

    
    // --- Create Endpoint ---
    // An endpoint is a logical interface on the device.
    esp_zb_ep_list_t *endpoints = esp_zb_ep_list_create();
    esp_zb_endpoint_config_t endpoint_config = {
        .endpoint = HA_ESP_VOLTAGE_SENSOR_ENDPOINT,
        .app_profile_id = ESP_ZB_AF_HA_PROFILE_ID, // Home Automation profile.
        .app_device_id = ESP_ZB_HA_SIMPLE_SENSOR_DEVICE_ID, // Standard sensor device ID.
        .app_device_version = 0
    };
    // Add the cluster list to the endpoint.
    ESP_ERROR_CHECK(esp_zb_ep_list_add_ep(endpoints, clusters, endpoint_config));

    // --- Register Device and Start Zigbee Stack ---
    ESP_ERROR_CHECK(esp_zb_device_register(endpoints));
    esp_zb_core_action_handler_register(zb_action_handler); // Register the callback dispatcher.
    ESP_ERROR_CHECK(esp_zb_set_primary_network_channel_set(ESP_ZB_PRIMARY_CHANNEL_MASK));


    ESP_LOGI(TAG, "Starting Zigbee stack...");
    ESP_ERROR_CHECK(esp_zb_start(false)); // Start the Zigbee stack. `false` means it won't auto-start commissioning.
    ESP_LOGI(TAG, "Zigbee stack started, waiting for network events...");


    // --- Application Task and Main Loop ---
    // Create a queue to pass voltage data between tasks.
    voltage_queue = xQueueCreate(4, sizeof(float));
    // Create the task that will periodically measure the voltage.
    xTaskCreate(voltage_measurement_task, "voltage_measurement", 4096, NULL, 2, NULL);

    // This is the main application loop.
    while (1) {

        float current_voltage;
        // Wait for a new voltage reading from the queue.
        if (xQueueReceive(voltage_queue, &current_voltage, pdMS_TO_TICKS(50))) {
            ESP_LOGI(TAG, "Battery Voltage: %.2f V", current_voltage);

            // Determine the current voltage state (OK, LOW, HIGH, CRITICAL).
            uint8_t new_state;

            if (current_voltage < CRITICAL_LOW_VOLTAGE_THRESHOLD) {
                new_state = 3; // Critical low
            } else if (current_voltage < LOW_VOLTAGE_THRESHOLD) {
                new_state = 1; // Low
            } else if (current_voltage > HIGH_VOLTAGE_THRESHOLD) {
                new_state = 2; // High
            } else {
                new_state = 0; // OK
            }

            // Prepare values for Zigbee reporting.
            // Voltage is reported in tenths of a volt.
            uint8_t battery_voltage_zb = (uint8_t)(current_voltage * 10.0f); 

            // Percentage is reported as 0-200 (0-100%).
            uint8_t battery_percent_zb = (current_voltage <= LOW_VOLTAGE_THRESHOLD) ? 0 :
                              (current_voltage >= HIGH_VOLTAGE_THRESHOLD) ? 200 :
                              (uint8_t)(((current_voltage - LOW_VOLTAGE_THRESHOLD) /
                              (HIGH_VOLTAGE_THRESHOLD - LOW_VOLTAGE_THRESHOLD)) * 200);
            ESP_LOGI(TAG, "Battery Percentage ZB: %d, Battery Voltage ZB: %d", battery_percent_zb, battery_voltage_zb);
            ESP_ERROR_CHECK(esp_zb_zcl_set_attribute_val(HA_ESP_VOLTAGE_SENSOR_ENDPOINT, ESP_ZB_ZCL_CLUSTER_ID_POWER_CONFIG, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE, ESP_ZB_ZCL_ATTR_POWER_CONFIG_BATTERY_VOLTAGE_ID, &battery_voltage_zb, false));
            ESP_ERROR_CHECK(esp_zb_zcl_set_attribute_val(HA_ESP_VOLTAGE_SENSOR_ENDPOINT, ESP_ZB_ZCL_CLUSTER_ID_POWER_CONFIG, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE, ESP_ZB_ZCL_ATTR_POWER_CONFIG_BATTERY_PERCENTAGE_REMAINING_ID, &battery_percent_zb, false));

            // Only send Zigbee reports if the device is connected to a network.
            if (zigbee_connected) {
                ESP_LOGI(TAG, "Reporting Battery Voltage: %d", battery_voltage_zb);
                ESP_ERROR_CHECK_WITHOUT_ABORT(esp_zb_zcl_manual_report(HA_ESP_VOLTAGE_SENSOR_ENDPOINT, ESP_ZB_ZCL_CLUSTER_ID_POWER_CONFIG, ESP_ZB_ZCL_ATTR_POWER_CONFIG_BATTERY_VOLTAGE_ID));
                ESP_LOGI(TAG, "Reported Battery Voltage: %d", battery_voltage_zb);
                //esp_zb_stack_main_loop_iteration();
                
                
                ESP_LOGI(TAG, "Reporting Battery Percentage: %d", battery_percent_zb);
                
                ESP_ERROR_CHECK_WITHOUT_ABORT(esp_zb_zcl_manual_report(HA_ESP_VOLTAGE_SENSOR_ENDPOINT, ESP_ZB_ZCL_CLUSTER_ID_POWER_CONFIG, ESP_ZB_ZCL_ATTR_POWER_CONFIG_BATTERY_PERCENTAGE_REMAINING_ID));
                ESP_LOGI(TAG, "Reported Battery Percentage: %d", battery_percent_zb);
                //esp_zb_stack_main_loop_iteration();
                
                // If the alarm state has changed, log it and update the Zigbee alarm attribute.
                /*if (new_state != voltage_alarm_state) {
                    voltage_alarm_state = new_state;
                    ESP_LOGI(TAG, "Voltage alarm state changed to: %s",
                         (new_state == 0) ? "OK" :
                         (new_state == 1) ? "LOW" :
                         (new_state == 2) ? "HIGH" : "CRITICAL");

                    // Update the BatteryAlarmMask attribute to signal a low voltage condition.
                    // Bit 0 corresponds to "Battery voltage too low".
                    uint8_t alarm_mask = 0;
                    if (new_state == 1 || new_state == 3) { // LOW or CRITICAL
                        alarm_mask = 1;
                    }
                    //ESP_ERROR_CHECK(esp_zb_zcl_set_attribute_val(HA_ESP_VOLTAGE_SENSOR_ENDPOINT, ESP_ZB_ZCL_CLUSTER_ID_POWER_CONFIG, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE, ESP_ZB_ZCL_ATTR_POWER_CONFIG_BATTERY_ALARM_MASK_ID, &alarm_mask, false));
                    ESP_LOGI(TAG, "Reporting Battery Alarm Mask: %d", alarm_mask);
                    ESP_ERROR_CHECK_WITHOUT_ABORT(esp_zb_zcl_manual_report(HA_ESP_VOLTAGE_SENSOR_ENDPOINT, ESP_ZB_ZCL_CLUSTER_ID_POWER_CONFIG,ESP_ZB_ZCL_ATTR_POWER_CONFIG_BATTERY_ALARM_MASK_ID, &alarm_mask));
                    ESP_LOGI(TAG, "Reported Battery Alarm Mask: %d", alarm_mask);
                    //esp_zb_stack_main_loop_iteration();
                }*/
            } else {
                ESP_LOGW(TAG, "Not joined to Zigbee network - skipping attribute update");
            }
        }
        // Allow the Zigbee stack to process its events. This is crucial.
        esp_zb_stack_main_loop_iteration();
    }
    
}
