// Includes for Zigbee, FreeRTOS, ADC, NVS, and logging
#include "adc-sensor.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "nvs_flash.h"

#include "zigbee-protocol.h"

// --- Constants and Configuration ---

// Tag for logging
#define TAG "ZB_BATTERY_MONITOR"

// --- Global Variables ---
static uint8_t voltage_alarm_state = 0;            // Current alarm state: 0=OK, 1=LOW, 2=HIGH, 3=CRITICAL

static QueueHandle_t voltage_queue;                // FreeRTOS queue to pass voltage readings from measurement task to main loop

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
    float current_voltage = 0.0f;
    while (1) {
        current_voltage = adc_sensor_read_voltage();
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
    adc_sensor_init(); // Initialize the ADC.

    ESP_LOGI(TAG, "ESP32-H2 Zigbee Battery Monitor starting...");

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
    esp_zb_attribute_list_t *power_config_cluster_attributes = esp_zb_power_config_cluster_create(NULL);

    // Add attributes for battery voltage, percentage, alarm mask, and threshold.
    // NOTE: Battery voltage is added but cannot be configured for automatic reporting due to SDK limitation.
    // The SDK defines BATTERY_VOLTAGE as READ_ONLY without the REPORTING flag, unlike BATTERY_PERCENTAGE_REMAINING
    // which includes READ_ONLY | REPORTING. See README.md for details.
    uint8_t battery_voltage_zb = 120; // Initial value (12.0V)
    ESP_ERROR_CHECK(esp_zb_power_config_cluster_add_attr(power_config_cluster_attributes, ESP_ZB_ZCL_ATTR_POWER_CONFIG_BATTERY_VOLTAGE_ID, &battery_voltage_zb));

    uint8_t battery_percent_zb = 100; // Initial value
    // Battery percentage supports automatic reporting and will be configured by the coordinator.
    ESP_ERROR_CHECK(esp_zb_power_config_cluster_add_attr(power_config_cluster_attributes, ESP_ZB_ZCL_ATTR_POWER_CONFIG_BATTERY_PERCENTAGE_REMAINING_ID, &battery_percent_zb));

    uint8_t alarm_mask_zb = 0; // No alarms initially.
    //ESP_ERROR_CHECK(esp_zb_power_config_cluster_add_attr(power_config_cluster_attributes, ESP_ZB_ZCL_ATTR_POWER_CONFIG_BATTERY_ALARM_MASK_ID, &alarm_mask_zb));

    //uint8_t battery_low_threshold = 10 * CRITICAL_LOW_VOLTAGE_THRESHOLD; // Set low threshold based on config.
    //ESP_ERROR_CHECK(esp_zb_power_config_cluster_add_attr(power_config_cluster_attributes, ESP_ZB_ZCL_ATTR_POWER_CONFIG_BATTERY_VOLTAGE_MIN_THRESHOLD_ID, &battery_low_threshold));

    ESP_ERROR_CHECK(esp_zb_cluster_list_add_power_config_cluster(clusters, power_config_cluster_attributes, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE));

    // --- Analog Input Cluster (Server) ---
    // This cluster provides an interface for reading the value of an analog measurement.
    esp_zb_analog_input_cluster_cfg_t analog_input_cfg = {
        .out_of_service = false,
        .present_value = 0.0f,
        .status_flags = 0
    };
    esp_zb_attribute_list_t *analog_input_attributes = esp_zb_analog_input_cluster_create(&analog_input_cfg);
    
    // Check if attributes are created successfully by the helper before adding manual ones.
    // The helper `esp_zb_analog_input_cluster_create` adds mandatory attributes:
    // PresentValue, StatusFlags, OutOfService.
    
    ESP_ERROR_CHECK(esp_zb_cluster_list_add_analog_input_cluster(clusters, analog_input_attributes, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE));

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

        float battery_voltage;
        // Wait for a new voltage reading from the queue.
        if (xQueueReceive(voltage_queue, &battery_voltage, pdMS_TO_TICKS(50))) {
            ESP_LOGI(TAG, "Battery Voltage: %.2f V", battery_voltage);

            // Determine the current voltage state (OK, LOW, HIGH, CRITICAL).
            uint8_t new_state;

            if (battery_voltage < CRITICAL_LOW_VOLTAGE_THRESHOLD) {
                new_state = 3; // Critical low
            } else if (battery_voltage < LOW_VOLTAGE_THRESHOLD) {
                new_state = 1; // Low
            } else if (battery_voltage > HIGH_VOLTAGE_THRESHOLD) {
                new_state = 2; // High
            } else {
                new_state = 0; // OK
            }

            // Prepare values for Zigbee reporting.
            // Voltage is reported in tenths of a volt.
            battery_voltage_zb = (uint8_t)(battery_voltage * 10.0f);

            // Percentage is reported as 0-200 (0-100%).
            battery_percent_zb = (battery_voltage <= LOW_VOLTAGE_THRESHOLD) ? 0 :
                              (battery_voltage >= HIGH_VOLTAGE_THRESHOLD) ? 200 :
                              (uint8_t)(((battery_voltage - LOW_VOLTAGE_THRESHOLD) /
                              (HIGH_VOLTAGE_THRESHOLD - LOW_VOLTAGE_THRESHOLD)) * 200);
            ESP_LOGI(TAG, "Battery Percentage ZB: %d, Battery Voltage ZB: %d", battery_percent_zb, battery_voltage_zb);
            
            // Update both attributes in the Zigbee cluster.
            // Voltage: Updated but cannot auto-report (SDK limitation - see README.md)
            ESP_ERROR_CHECK(esp_zb_zcl_set_attribute_val(HA_ESP_VOLTAGE_SENSOR_ENDPOINT, ESP_ZB_ZCL_CLUSTER_ID_POWER_CONFIG, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE, ESP_ZB_ZCL_ATTR_POWER_CONFIG_BATTERY_VOLTAGE_ID, &battery_voltage_zb, false));
            // Percentage: Updated and will auto-report when configured by coordinator
            ESP_ERROR_CHECK(esp_zb_zcl_set_attribute_val(HA_ESP_VOLTAGE_SENSOR_ENDPOINT, ESP_ZB_ZCL_CLUSTER_ID_POWER_CONFIG, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE, ESP_ZB_ZCL_ATTR_POWER_CONFIG_BATTERY_PERCENTAGE_REMAINING_ID, &battery_percent_zb, false));

            // Update Analog Input PresentValue with the float voltage
            ESP_ERROR_CHECK(esp_zb_zcl_set_attribute_val(HA_ESP_VOLTAGE_SENSOR_ENDPOINT, ESP_ZB_ZCL_CLUSTER_ID_ANALOG_INPUT, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE, ESP_ZB_ZCL_ATTR_ANALOG_INPUT_PRESENT_VALUE_ID, &battery_voltage, false));

            // If the alarm state has changed, log it and update the Zigbee alarm attribute.
            if (new_state != voltage_alarm_state) {
                voltage_alarm_state = new_state;
                ESP_LOGI(TAG, "Voltage alarm state changed to: %s",
                         (new_state == 0) ? "OK" :
                         (new_state == 1) ? "LOW" :
                         (new_state == 2) ? "HIGH" : "CRITICAL");

                // Update the BatteryAlarmMask attribute to signal a low voltage condition.
                // Bit 0 corresponds to "Battery voltage too low".
                alarm_mask_zb = 0;
                if (new_state == 1 || new_state == 3) { // LOW or CRITICAL
                    alarm_mask_zb = 1;
                }
                //ESP_ERROR_CHECK(esp_zb_zcl_set_attribute_val(HA_ESP_VOLTAGE_SENSOR_ENDPOINT, ESP_ZB_ZCL_CLUSTER_ID_POWER_CONFIG, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE, ESP_ZB_ZCL_ATTR_POWER_CONFIG_BATTERY_ALARM_MASK_ID, &alarm_mask_zb, false));
            }

            // Note: Voltage attribute is not reportable in the ESP Zigbee SDK (it's defined as READ_ONLY
            // without the REPORTING flag, unlike percentage which has READ_ONLY | REPORTING).
            // The voltage can still be read by the coordinator, but automatic reporting is not supported.
            // Percentage will report automatically via coordinator configuration.
        }
        // Allow the Zigbee stack to process its events. This is crucial.
        esp_zb_stack_main_loop_iteration();
    }
}
