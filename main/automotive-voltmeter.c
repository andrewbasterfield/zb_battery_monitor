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
#define ESP_ZB_PRIMARY_CHANNEL_MASK ESP_ZB_TRANSCEIVER_ALL_CHANNELS_MASK

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

// Zigbee attribute IDs (custom cluster)
#define ESP_ZB_ZCL_CLUSTER_ID_VOLTAGE_MEASUREMENT 0xFC00
#define ESP_ZB_ZCL_ATTR_VOLTAGE_MEASUREMENT_VALUE 0x0000
#define ESP_ZB_ZCL_ATTR_VOLTAGE_ALARM_STATE 0x0001

void esp_zb_app_signal_handler(esp_zb_app_signal_t *signal_struct)
{
    uint32_t *p_sg_p = signal_struct->p_app_signal;
    esp_err_t err_status = signal_struct->esp_err_status;
    esp_zb_app_signal_type_t sig_type = *p_sg_p;

    switch (sig_type) {
    case ESP_ZB_ZDO_SIGNAL_SKIP_STARTUP:
        ESP_LOGI(TAG, "Zigbee stack initialized");
        esp_zb_bdb_start_top_level_commissioning(ESP_ZB_BDB_MODE_INITIALIZATION);
        break;
    case ESP_ZB_BDB_SIGNAL_DEVICE_FIRST_START:
    case ESP_ZB_BDB_SIGNAL_DEVICE_REBOOT:
        if (err_status == ESP_OK) {
            ESP_LOGI(TAG, "Device started up in %s factory-reset mode",
                     esp_zb_bdb_is_factory_new() ? "" : "non");
            if (esp_zb_bdb_is_factory_new()) {
                ESP_LOGI(TAG, "Start network formation");
                esp_zb_bdb_start_top_level_commissioning(ESP_ZB_BDB_MODE_NETWORK_FORMATION);
            } else {
                ESP_LOGI(TAG, "Device rebooted");
            }
        } else {
            ESP_LOGW(TAG, "Failed to initialize Zigbee stack (status: %s)", esp_err_to_name(err_status));
        }
        break;
    case ESP_ZB_BDB_SIGNAL_FORMATION:
        if (err_status == ESP_OK) {
            esp_zb_ieee_addr_t extended_pan_id;
            esp_zb_get_extended_pan_id(extended_pan_id);
            ESP_LOGI(TAG, "Formed network successfully (Extended PAN ID: %02x:%02x:%02x:%02x:%02x:%02x:%02x:%02x, PAN ID: 0x%04hx, Channel:%d)",
                     extended_pan_id[7], extended_pan_id[6], extended_pan_id[5], extended_pan_id[4],
                     extended_pan_id[3], extended_pan_id[2], extended_pan_id[1], extended_pan_id[0],
                     esp_zb_get_pan_id(), esp_zb_get_current_channel());
            esp_zb_bdb_start_top_level_commissioning(ESP_ZB_BDB_MODE_NETWORK_STEERING);
        } else {
            ESP_LOGI(TAG, "Restart network formation (status: %s)", esp_err_to_name(err_status));
            // Simple retry without using deprecated scheduler
            vTaskDelay(pdMS_TO_TICKS(1000));
            esp_zb_bdb_start_top_level_commissioning(ESP_ZB_BDB_MODE_NETWORK_FORMATION);
        }
        break;
    case ESP_ZB_BDB_SIGNAL_STEERING:
        if (err_status == ESP_OK) {
            ESP_LOGI(TAG, "Network steering started");
        }
        break;
    default:
        ESP_LOGI(TAG, "ZDO signal: %s (0x%x), status: %s", esp_zb_zdo_signal_to_string(sig_type), sig_type,
                 esp_err_to_name(err_status));
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

        // Update Zigbee attribute
        esp_zb_zcl_set_attribute_val(HA_ESP_VOLTAGE_SENSOR_ENDPOINT, ESP_ZB_ZCL_CLUSTER_ID_VOLTAGE_MEASUREMENT,
                                    ESP_ZB_ZCL_CLUSTER_SERVER_ROLE, ESP_ZB_ZCL_ATTR_VOLTAGE_ALARM_STATE,
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

        // Convert voltage to centivolt for Zigbee (standard practice)
        uint16_t voltage_centivolt = (uint16_t)(current_voltage * 100);

        // Update Zigbee attribute
        esp_zb_zcl_set_attribute_val(HA_ESP_VOLTAGE_SENSOR_ENDPOINT, ESP_ZB_ZCL_CLUSTER_ID_VOLTAGE_MEASUREMENT,
                                    ESP_ZB_ZCL_CLUSTER_SERVER_ROLE, ESP_ZB_ZCL_ATTR_VOLTAGE_MEASUREMENT_VALUE,
                                    &voltage_centivolt, false);

        vTaskDelay(pdMS_TO_TICKS(MEASUREMENT_INTERVAL_MS));
    }
}

static void esp_zb_task(void *pvParameters)
{
    esp_zb_cfg_t zb_nwk_cfg = {
        .esp_zb_role = ESP_ZB_DEVICE_TYPE_ED,
        .install_code_policy = INSTALLCODE_POLICY_ENABLE,
        .nwk_cfg = {
            .zed_cfg = {
                .ed_timeout = ED_AGING_TIMEOUT,
                .keep_alive = ED_KEEP_ALIVE,
            },
        },
    };
    esp_zb_init(&zb_nwk_cfg);

    // Create custom voltage measurement cluster
    esp_zb_cluster_list_t *esp_zb_cluster_list = esp_zb_zcl_cluster_list_create();

    esp_zb_attribute_list_t *voltage_cluster = esp_zb_zcl_attr_list_create(ESP_ZB_ZCL_CLUSTER_ID_VOLTAGE_MEASUREMENT);

    uint16_t voltage_value = 1200; // Initial value (12.00V)
    uint8_t alarm_state = 0;       // Initial state (OK)

    // Add custom attributes to custom cluster
    esp_zb_custom_cluster_add_custom_attr(voltage_cluster, ESP_ZB_ZCL_ATTR_VOLTAGE_MEASUREMENT_VALUE,
                                         ESP_ZB_ZCL_ATTR_TYPE_U16, ESP_ZB_ZCL_ATTR_ACCESS_READ_ONLY | ESP_ZB_ZCL_ATTR_ACCESS_REPORTING,
                                         &voltage_value);
    esp_zb_custom_cluster_add_custom_attr(voltage_cluster, ESP_ZB_ZCL_ATTR_VOLTAGE_ALARM_STATE,
                                         ESP_ZB_ZCL_ATTR_TYPE_U8, ESP_ZB_ZCL_ATTR_ACCESS_READ_ONLY | ESP_ZB_ZCL_ATTR_ACCESS_REPORTING,
                                         &alarm_state);

    // Add custom cluster to cluster list
    esp_zb_cluster_list_add_custom_cluster(esp_zb_cluster_list, voltage_cluster, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);

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

    ESP_ERROR_CHECK(nvs_flash_init());
    ESP_ERROR_CHECK(esp_zb_platform_config(&config));

    // Initialize ADC
    adc_init();

    ESP_LOGI(TAG, "ESP32-H2 Automotive Voltmeter starting...");
    ESP_LOGI(TAG, "Voltage divider ratio: %.1f", VOLTAGE_DIVIDER_RATIO);
    ESP_LOGI(TAG, "Low voltage threshold: %.1fV", LOW_VOLTAGE_THRESHOLD);
    ESP_LOGI(TAG, "High voltage threshold: %.1fV", HIGH_VOLTAGE_THRESHOLD);
    ESP_LOGI(TAG, "Critical low threshold: %.1fV", CRITICAL_LOW_THRESHOLD);

    // Create tasks
    xTaskCreate(esp_zb_task, "Zigbee_main", 4096, NULL, 5, NULL);
    xTaskCreate(voltage_measurement_task, "voltage_measurement", 2048, NULL, 3, NULL);
}