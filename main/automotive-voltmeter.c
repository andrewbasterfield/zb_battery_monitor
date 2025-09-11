#include "esp_zigbee_core.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include "nvs_flash.h"
#include "esp_check.h"

#define TAG "AUTOMOTIVE_VOLTMETER"

// Hardware configuration
#define VOLTAGE_DIVIDER_CHANNEL ADC_CHANNEL_0  // GPIO0
#define SAMPLE_COUNT 64
#define MEASUREMENT_INTERVAL_MS 5000

// Voltage divider configuration (adjust based on your resistor values)
// For 12V automotive system, use voltage divider to scale down to ESP32 range
// Example: R1=47kΩ, R2=10kΩ gives ratio of 5.7:1 (max ~16.4V input for 3.3V ADC)
#define VOLTAGE_DIVIDER_RATIO 5.7f
#define ADC_VREF 3300  // mV

// Zigbee configuration
#define INSTALLCODE_POLICY_ENABLE false
#define ED_AGING_TIMEOUT ESP_ZB_ED_AGING_TIMEOUT_64MIN
#define ED_KEEP_ALIVE 3000
#define HA_ESP_VOLTAGE_SENSOR_ENDPOINT 1
//#define ESP_ZB_PRIMARY_CHANNEL_MASK ESP_ZB_TRANSCEIVER_ALL_CHANNELS_MASK
#define ESP_ZB_PRIMARY_CHANNEL_MASK (1 << 11)
#define ESP_ZB_ZCL_ATTR_POWER_CONFIG_BATTERY_VOLTAGE_ID 0x0020

// Automotive voltage thresholds (in volts)
#define LOW_VOLTAGE_THRESHOLD 11.5f
#define HIGH_VOLTAGE_THRESHOLD 14.8f
#define CRITICAL_LOW_THRESHOLD 10.5f

// Global variables
static adc_oneshot_unit_handle_t adc1_handle;
static adc_cali_handle_t adc1_cali_handle = NULL;
static bool do_calibration = false;
static float current_voltage = 0.0f;
static uint8_t voltage_alarm_state = 0; // 0=OK, 1=LOW, 2=HIGH, 3=CRITICAL

void esp_zb_app_signal_handler(esp_zb_app_signal_t *signal_struct)
{
    uint32_t *p_sg_p = signal_struct->p_app_signal;
    esp_err_t err_status = signal_struct->esp_err_status;
    esp_zb_app_signal_type_t sig_type = *p_sg_p;

    switch (sig_type) {
    case ESP_ZB_ZDO_SIGNAL_SKIP_STARTUP:
        ESP_LOGI(TAG, "Zigbee stack initialized");
        ESP_LOGI(TAG, "Primary channel mask: 0x%08x", (unsigned int)ESP_ZB_PRIMARY_CHANNEL_MASK);
        esp_zb_bdb_start_top_level_commissioning(ESP_ZB_BDB_MODE_INITIALIZATION);
        break;
    case ESP_ZB_BDB_SIGNAL_DEVICE_FIRST_START:
    case ESP_ZB_BDB_SIGNAL_DEVICE_REBOOT:
        if (err_status == ESP_OK) {
            ESP_LOGI(TAG, "Device started up in %s factory-reset mode",
                     esp_zb_bdb_is_factory_new() ? "" : "non");
            if (esp_zb_bdb_is_factory_new()) {
                ESP_LOGI(TAG, "Start network steering (joining network)");
                esp_zb_bdb_start_top_level_commissioning(ESP_ZB_BDB_MODE_NETWORK_STEERING);
            } else {
                ESP_LOGI(TAG, "Device rebooted, rejoining network");
                esp_zb_bdb_start_top_level_commissioning(ESP_ZB_BDB_MODE_NETWORK_STEERING);
            }
            ESP_LOGI(TAG, "PAN ID: 0x%04hx, Channel: %d", esp_zb_get_pan_id(), esp_zb_get_current_channel());
        } else {
            ESP_LOGE(TAG, "Failed to initialize Zigbee stack (status: %s, code: 0x%x)", esp_err_to_name(err_status), err_status);
        }
        break;
    case ESP_ZB_BDB_SIGNAL_STEERING:
        if (err_status == ESP_OK) {
            ESP_LOGI(TAG, "Successfully joined network");
            esp_zb_ieee_addr_t extended_pan_id;
            esp_zb_get_extended_pan_id(extended_pan_id);
            ESP_LOGI(TAG, "Joined network (Extended PAN ID: %02x:%02x:%02x:%02x:%02x:%02x:%02x:%02x, PAN ID: 0x%04hx, Channel:%d)",
                     extended_pan_id[7], extended_pan_id[6], extended_pan_id[5], extended_pan_id[4],
                     extended_pan_id[3], extended_pan_id[2], extended_pan_id[1], extended_pan_id[0],
                     esp_zb_get_pan_id(), esp_zb_get_current_channel());
        } else {
            ESP_LOGI(TAG, "Network steering failed (status: %s), retrying in 1 second...", esp_err_to_name(err_status));
            // Simple retry without deprecated scheduler - will be handled by task
        }
        break;
    case ESP_ZB_ZDO_SIGNAL_LEAVE:
        ESP_LOGI(TAG, "Leave network");
        break;
    default:
        ESP_LOGI(TAG, "ZDO signal: %s (0x%x), status: %s, code: 0x%x", esp_zb_zdo_signal_to_string(sig_type), sig_type,
                 esp_err_to_name(err_status), err_status);
        break;
    }
}

static esp_err_t zb_attribute_handler(const esp_zb_zcl_set_attr_value_message_t *message)
{
    esp_err_t ret = ESP_OK;
    ESP_RETURN_ON_FALSE(message, ESP_FAIL, TAG, "Empty message");
    ESP_RETURN_ON_FALSE(message->info.status == ESP_ZB_ZCL_STATUS_SUCCESS, ESP_ERR_INVALID_ARG, TAG, "Received message: error status(%d)",
                        message->info.status);

    ESP_LOGI(TAG, "Received ZCL attribute(0x%x) set to cluster(0x%x)",
             message->attribute.id, message->info.cluster);

    return ret;
}

static esp_err_t zb_read_attr_resp_handler(const esp_zb_zcl_cmd_read_attr_resp_message_t *message)
{
    ESP_RETURN_ON_FALSE(message, ESP_FAIL, TAG, "Empty message");
    ESP_RETURN_ON_FALSE(message->info.status == ESP_ZB_ZCL_STATUS_SUCCESS, ESP_ERR_INVALID_ARG, TAG, "Received message: error status(%d)",
                        message->info.status);

    ESP_LOGI(TAG, "Read attribute response: cluster(0x%x), attribute(0x%x), type(0x%x), value(%d)",
             message->info.cluster, message->variables->attribute.id, message->variables->attribute.data.type,
             message->variables->attribute.data.value ? *(uint8_t*)message->variables->attribute.data.value : 0);

    return ESP_OK;
}

static esp_err_t zb_action_handler(esp_zb_core_action_callback_id_t callback_id, const void *message)
{
    esp_err_t ret = ESP_OK;
    switch (callback_id) {
    case ESP_ZB_CORE_SET_ATTR_VALUE_CB_ID:
        ret = zb_attribute_handler((esp_zb_zcl_set_attr_value_message_t *)message);
        break;
    case ESP_ZB_CORE_CMD_READ_ATTR_RESP_CB_ID:
        ret = zb_read_attr_resp_handler((esp_zb_zcl_cmd_read_attr_resp_message_t *)message);
        break;
    default:
        ESP_LOGW(TAG, "Receive Zigbee action(0x%x) callback", callback_id);
        break;
    }
    return ret;
}

static bool adc_calibration_init(adc_unit_t unit, adc_channel_t channel, adc_atten_t atten, adc_cali_handle_t *out_handle)
{
    adc_cali_handle_t handle = NULL;
    esp_err_t ret = ESP_FAIL;
    bool calibrated = false;

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

static void adc_init(void)
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

static float read_battery_voltage(void)
{
    uint32_t adc_reading = 0;
    int adc_raw;

    // Multi-sample for accuracy
    for (int i = 0; i < SAMPLE_COUNT; i++) {
        ESP_ERROR_CHECK(adc_oneshot_read(adc1_handle, VOLTAGE_DIVIDER_CHANNEL, &adc_raw));
        adc_reading += adc_raw;
    }
    adc_reading /= SAMPLE_COUNT;

    // Convert ADC reading to voltage
    int voltage_mv = 0;
    if (do_calibration) {
        ESP_ERROR_CHECK(adc_cali_raw_to_voltage(adc1_cali_handle, adc_reading, &voltage_mv));
    } else {
        // Fallback calculation without calibration
        voltage_mv = (adc_reading * ADC_VREF) / 4095; // 12-bit ADC
    }

    // Apply voltage divider ratio to get actual battery voltage
    float battery_voltage = (voltage_mv * VOLTAGE_DIVIDER_RATIO) / 1000.0f;

    return battery_voltage;
}

static void update_voltage_alarm_state(float voltage)
{
    uint8_t new_state;

    if (voltage < CRITICAL_LOW_THRESHOLD) {
        new_state = 3; // Critical low
    } else if (voltage < LOW_VOLTAGE_THRESHOLD) {
        new_state = 1; // Low
    } else if (voltage > HIGH_VOLTAGE_THRESHOLD) {
        new_state = 2; // High
    } else {
        new_state = 0; // OK
    }

    if (new_state != voltage_alarm_state) {
        voltage_alarm_state = new_state;
        ESP_LOGI(TAG, "Voltage alarm state changed to: %s",
                 (new_state == 0) ? "OK" :
                 (new_state == 1) ? "LOW" :
                 (new_state == 2) ? "HIGH" : "CRITICAL");

        // Update occupancy sensor attribute to indicate alarm state
        esp_zb_zcl_set_attribute_val(HA_ESP_VOLTAGE_SENSOR_ENDPOINT, ESP_ZB_ZCL_CLUSTER_ID_OCCUPANCY_SENSING,
                                    ESP_ZB_ZCL_CLUSTER_SERVER_ROLE, ESP_ZB_ZCL_ATTR_OCCUPANCY_SENSING_OCCUPANCY_ID,
                                    &voltage_alarm_state, false);
    }
}

static void voltage_measurement_task(void *pvParameters)
{
    while (1) {
        current_voltage = read_battery_voltage();

        ESP_LOGI(TAG, "Battery Voltage: %.2f V", current_voltage);

        // Update alarm state
        update_voltage_alarm_state(current_voltage);

    // Update Zigbee attribute - using Power Configuration cluster (BatteryVoltage, tenths of a volt)
    uint8_t battery_voltage_zb = (uint8_t)(current_voltage * 10.0f); // Zigbee expects tenths of a volt
    esp_zb_zcl_set_attribute_val(HA_ESP_VOLTAGE_SENSOR_ENDPOINT, ESP_ZB_ZCL_CLUSTER_ID_POWER_CONFIG,
                    ESP_ZB_ZCL_CLUSTER_SERVER_ROLE, ESP_ZB_ZCL_ATTR_POWER_CONFIG_BATTERY_VOLTAGE_ID,
                    &battery_voltage_zb, false);

        vTaskDelay(pdMS_TO_TICKS(MEASUREMENT_INTERVAL_MS));
    }
}

static bool zigbee_join_retry = true;

static void zigbee_retry_task(void *pvParameters)
{
    while (zigbee_join_retry) {
        vTaskDelay(pdMS_TO_TICKS(10000)); // Wait 10 seconds between retries
        if (zigbee_join_retry) {
            ESP_LOGI(TAG, "Attempting to rejoin network...");
            esp_zb_bdb_start_top_level_commissioning(ESP_ZB_BDB_MODE_NETWORK_STEERING);
        }
    }
    vTaskDelete(NULL);
}

static void esp_zb_task(void *pvParameters)
{
    esp_zb_cfg_t zb_nwk_cfg = {
        .esp_zb_role = ESP_ZB_DEVICE_TYPE_ED,  // End Device - will join existing network
        .install_code_policy = INSTALLCODE_POLICY_ENABLE,
        .nwk_cfg = {
            .zed_cfg = {
                .ed_timeout = ED_AGING_TIMEOUT,
                .keep_alive = ED_KEEP_ALIVE,
            },
        },
    };
    esp_zb_init(&zb_nwk_cfg);

    // Enable more detailed Zigbee logging
    esp_log_level_set("ESP_ZB", ESP_LOG_DEBUG);
    esp_log_level_set("ESP_ZB_ZCL", ESP_LOG_DEBUG);
    esp_log_level_set("ESP_ZB_ZDO", ESP_LOG_DEBUG);
    
    ESP_LOGI(TAG, "Zigbee stack initialized with role: End Device");
    ESP_LOGI(TAG, "Install code policy: %s", INSTALLCODE_POLICY_ENABLE ? "enabled" : "disabled");
    ESP_LOGI(TAG, "Channel mask: 0x%08x", (unsigned int)ESP_ZB_PRIMARY_CHANNEL_MASK);

    // Create cluster list with standard Zigbee clusters
    esp_zb_cluster_list_t *esp_zb_cluster_list = esp_zb_zcl_cluster_list_create();

    // Basic cluster (mandatory for all devices)
    esp_zb_attribute_list_t *basic_cluster = esp_zb_zcl_attr_list_create(ESP_ZB_ZCL_CLUSTER_ID_BASIC);
    uint8_t zcl_version = ESP_ZB_ZCL_BASIC_ZCL_VERSION_DEFAULT_VALUE;
    uint8_t power_source = 0x01; // Mains (single phase)
    char manufacturer_name[] = "Espressif";
    char model_identifier[] = "ESP.VOLTMETER";
    char sw_version[] = "1.0.0";
    
    esp_zb_basic_cluster_add_attr(basic_cluster, ESP_ZB_ZCL_ATTR_BASIC_ZCL_VERSION_ID, &zcl_version);
    esp_zb_basic_cluster_add_attr(basic_cluster, ESP_ZB_ZCL_ATTR_BASIC_POWER_SOURCE_ID, &power_source);
    esp_zb_basic_cluster_add_attr(basic_cluster, ESP_ZB_ZCL_ATTR_BASIC_MANUFACTURER_NAME_ID, manufacturer_name);
    esp_zb_basic_cluster_add_attr(basic_cluster, ESP_ZB_ZCL_ATTR_BASIC_MODEL_IDENTIFIER_ID, model_identifier);
    esp_zb_basic_cluster_add_attr(basic_cluster, ESP_ZB_ZCL_ATTR_BASIC_SW_BUILD_ID, sw_version);
    esp_zb_cluster_list_add_basic_cluster(esp_zb_cluster_list, basic_cluster, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);

    // Identify cluster (mandatory for Home Automation)
    esp_zb_attribute_list_t *identify_cluster = esp_zb_zcl_attr_list_create(ESP_ZB_ZCL_CLUSTER_ID_IDENTIFY);
    uint16_t identify_time = 0;
    esp_zb_identify_cluster_add_attr(identify_cluster, ESP_ZB_ZCL_ATTR_IDENTIFY_IDENTIFY_TIME_ID, &identify_time);
    esp_zb_cluster_list_add_identify_cluster(esp_zb_cluster_list, identify_cluster, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);

    // Power Configuration cluster for battery voltage reporting
    esp_zb_attribute_list_t *power_config_cluster = esp_zb_zcl_attr_list_create(ESP_ZB_ZCL_CLUSTER_ID_POWER_CONFIG);
    uint8_t battery_voltage = 120; // 12.0V (in tenths of a volt, as per Zigbee spec)
    esp_zb_power_config_cluster_add_attr(power_config_cluster, ESP_ZB_ZCL_ATTR_POWER_CONFIG_BATTERY_VOLTAGE_ID, &battery_voltage);
    esp_zb_cluster_list_add_power_config_cluster(esp_zb_cluster_list, power_config_cluster, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);

    // Occupancy Sensing cluster for alarm status
    esp_zb_attribute_list_t *occupancy_cluster = esp_zb_zcl_attr_list_create(ESP_ZB_ZCL_CLUSTER_ID_OCCUPANCY_SENSING);
    uint8_t occupancy = 0;
    uint8_t occupancy_sensor_type = 0; // PIR
    esp_zb_occupancy_sensing_cluster_add_attr(occupancy_cluster, ESP_ZB_ZCL_ATTR_OCCUPANCY_SENSING_OCCUPANCY_ID, &occupancy);
    esp_zb_occupancy_sensing_cluster_add_attr(occupancy_cluster, ESP_ZB_ZCL_ATTR_OCCUPANCY_SENSING_OCCUPANCY_SENSOR_TYPE_ID, &occupancy_sensor_type);
    esp_zb_cluster_list_add_occupancy_sensing_cluster(esp_zb_cluster_list, occupancy_cluster, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);

    // Create endpoint list
    esp_zb_ep_list_t *esp_zb_ep_list = esp_zb_ep_list_create();
    esp_zb_endpoint_config_t endpoint_config = {
        .endpoint = HA_ESP_VOLTAGE_SENSOR_ENDPOINT,
        .app_profile_id = ESP_ZB_AF_HA_PROFILE_ID,
        .app_device_id = ESP_ZB_HA_SIMPLE_SENSOR_DEVICE_ID,
        .app_device_version = 0
    };
    esp_zb_ep_list_add_ep(esp_zb_ep_list, esp_zb_cluster_list, endpoint_config);

    esp_zb_device_register(esp_zb_ep_list);
    esp_zb_core_action_handler_register(zb_action_handler);
    esp_zb_set_primary_network_channel_set(ESP_ZB_PRIMARY_CHANNEL_MASK);

    ESP_ERROR_CHECK(esp_zb_start(false));

    // Create retry task for failed joins
    xTaskCreate(zigbee_retry_task, "zigbee_retry", 2048, NULL, 2, NULL);

    // Modern task loop instead of deprecated main loop iteration
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

void app_main(void)
{
    esp_zb_platform_config_t config = {
        .radio_config = {
            .radio_mode = ZB_RADIO_MODE_NATIVE,
        },
        .host_config = {
            .host_connection_mode = ZB_HOST_CONNECTION_MODE_NONE,
        },
    };

    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_LOGW(TAG, "NVS corrupted or new version found, erasing...");
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);
    ESP_LOGI(TAG, "NVS init result: %s", esp_err_to_name(ret));
    
    ESP_ERROR_CHECK(esp_zb_platform_config(&config));

    // Initialize ADC
    adc_init();

    ESP_LOGI(TAG, "ESP32-H2 Automotive Voltmeter starting...");
    ESP_LOGI(TAG, "Voltage divider ratio: %.1f", VOLTAGE_DIVIDER_RATIO);
    ESP_LOGI(TAG, "Low voltage threshold: %.1fV", LOW_VOLTAGE_THRESHOLD);
    ESP_LOGI(TAG, "High voltage threshold: %.1fV", HIGH_VOLTAGE_THRESHOLD);
    ESP_LOGI(TAG, "Critical low threshold: %.1fV", CRITICAL_LOW_THRESHOLD);
    ESP_LOGI(TAG, "Device will attempt to join existing Zigbee network...");

    // Set a static IEEE address for Zigbee (for testing)
    uint8_t custom_ieee_addr[8] = {0xDE, 0xAD, 0xBE, 0xEF, 0x00, 0x00, 0x00, 0x01};
    esp_zb_set_long_address(custom_ieee_addr);

    // Print IEEE address at startup
    esp_zb_ieee_addr_t ieee_addr;
    esp_zb_get_long_address(ieee_addr);
    ESP_LOGI(TAG, "Device IEEE Address: %02x:%02x:%02x:%02x:%02x:%02x:%02x:%02x",
        ieee_addr[7], ieee_addr[6], ieee_addr[5], ieee_addr[4],
        ieee_addr[3], ieee_addr[2], ieee_addr[1], ieee_addr[0]);
    
    // Create tasks
    xTaskCreate(esp_zb_task, "Zigbee_main", 8192, NULL, 5, NULL);
    xTaskCreate(voltage_measurement_task, "voltage_measurement", 4096, NULL, 3, NULL);
}