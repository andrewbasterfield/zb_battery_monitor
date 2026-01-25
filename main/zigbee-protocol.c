// Forward declaration to avoid circular dependency issue in ESP-IDF library
// This must come before any includes that might include esp_zigbee_zcl_core.h
typedef struct esp_zb_zcl_command_send_status_s esp_zb_zcl_command_send_status_message_t;
typedef void (*esp_zb_zcl_command_send_status_callback_t)(esp_zb_zcl_command_send_status_message_t message);

#include "esp_zigbee_core.h"
#include "zcl/esp_zigbee_zcl_common.h"
#include "esp_check.h"
#include "nvs_flash.h"
#include "nvs.h"

#include "zigbee-protocol.h"
#include "zcl/esp_zigbee_zcl_command.h"  // Include after other headers

bool zigbee_connected = false;

#define TAG "ZB_BATTERY_MONITOR_ZIGBEE"
#define COORDINATOR_ADDR 0x0000

static void retry_steering_cb(uint8_t param)
{
    ESP_LOGI(TAG, "Retrying network steering...");
    esp_zb_bdb_start_top_level_commissioning(ESP_ZB_BDB_MODE_NETWORK_STEERING);
}

static void bind_callback(esp_zb_zdp_status_t zdo_status, void *user_ctx) {
    esp_zb_zdo_bind_req_param_t *bind_req = (esp_zb_zdo_bind_req_param_t *)user_ctx;

    if (zdo_status == ESP_ZB_ZDP_STATUS_SUCCESS) {
        ESP_LOGI(TAG, "Successful bind from address(0x%x) on endpoint(%d)", bind_req->req_dst_addr, bind_req->dst_endp);
    }
    free(bind_req);
}

static void configure_binding() {
    esp_zb_zdo_bind_req_param_t *bind_req = (esp_zb_zdo_bind_req_param_t *)calloc(1, sizeof(esp_zb_zdo_bind_req_param_t));
    bind_req->req_dst_addr = esp_zb_get_short_address();
    bind_req->src_endp = HA_ESP_VOLTAGE_SENSOR_ENDPOINT;
    bind_req->dst_endp = HA_ESP_VOLTAGE_SENSOR_ENDPOINT;
    bind_req->cluster_id = ESP_ZB_ZCL_CLUSTER_ID_POWER_CONFIG;
    bind_req->dst_addr_mode = ESP_ZB_ZDO_BIND_DST_ADDR_MODE_64_BIT_EXTENDED;
    ESP_ERROR_CHECK(esp_zb_ieee_address_by_short(COORDINATOR_ADDR, bind_req->dst_address_u.addr_long));
    esp_zb_get_long_address(bind_req->src_address);
    esp_zb_zdo_device_bind_req(bind_req, bind_callback, bind_req);

    // Bind Analog Input Cluster
    esp_zb_zdo_bind_req_param_t *bind_req_analog = (esp_zb_zdo_bind_req_param_t *)calloc(1, sizeof(esp_zb_zdo_bind_req_param_t));
    bind_req_analog->req_dst_addr = esp_zb_get_short_address();
    bind_req_analog->src_endp = HA_ESP_VOLTAGE_SENSOR_ENDPOINT;
    bind_req_analog->dst_endp = HA_ESP_VOLTAGE_SENSOR_ENDPOINT;
    bind_req_analog->cluster_id = ESP_ZB_ZCL_CLUSTER_ID_ANALOG_INPUT;
    bind_req_analog->dst_addr_mode = ESP_ZB_ZDO_BIND_DST_ADDR_MODE_64_BIT_EXTENDED;
    ESP_ERROR_CHECK(esp_zb_ieee_address_by_short(COORDINATOR_ADDR, bind_req_analog->dst_address_u.addr_long));
    esp_zb_get_long_address(bind_req_analog->src_address);
    esp_zb_zdo_device_bind_req(bind_req_analog, bind_callback, bind_req_analog);
}

/**
 * @brief Configures automatic attribute reporting to the coordinator.
 */
void configure_reporting(void) {
    // --- Power Configuration Reporting ---
    esp_zb_zcl_config_report_cmd_t report_cmd = {
        .zcl_basic_cmd.dst_addr_u.addr_short = esp_zb_get_short_address(),
        .zcl_basic_cmd.dst_endpoint = HA_ESP_VOLTAGE_SENSOR_ENDPOINT,
        .zcl_basic_cmd.src_endpoint = HA_ESP_VOLTAGE_SENSOR_ENDPOINT,
        .address_mode = ESP_ZB_APS_ADDR_MODE_16_ENDP_PRESENT,
        .clusterID = ESP_ZB_ZCL_CLUSTER_ID_POWER_CONFIG,
    };

    static uint8_t reportable_change_percentage = 2; // 1% change (in 0.5% units)
    static uint32_t reportable_change_alarm = 1; // Report any alarm state change

    esp_zb_zcl_config_report_record_t records[] = {
        {
            .direction = ESP_ZB_ZCL_REPORT_DIRECTION_SEND,
            .attributeID = ESP_ZB_ZCL_ATTR_POWER_CONFIG_BATTERY_PERCENTAGE_REMAINING_ID,
            .attrType = ESP_ZB_ZCL_ATTR_TYPE_U8,
            .min_interval = 5,
            .max_interval = 60,
            .reportable_change = &reportable_change_percentage,
        },
        {
            .direction = ESP_ZB_ZCL_REPORT_DIRECTION_SEND,
            .attributeID = ESP_ZB_ZCL_ATTR_POWER_CONFIG_BATTERY_ALARM_STATE_ID,
            .attrType = ESP_ZB_ZCL_ATTR_TYPE_32BITMAP,
            .min_interval = 1,
            .max_interval = 300,
            .reportable_change = &reportable_change_alarm,
        },
    };

    report_cmd.record_number = sizeof(records) / sizeof(esp_zb_zcl_config_report_record_t);
    report_cmd.record_field = records;
    esp_zb_lock_acquire(portMAX_DELAY);
    esp_zb_zcl_config_report_cmd_req(&report_cmd);
    esp_zb_lock_release();

    // --- Analog Input Reporting ---
    esp_zb_zcl_config_report_cmd_t report_cmd_analog = {
        .zcl_basic_cmd.dst_addr_u.addr_short = esp_zb_get_short_address(),
        .zcl_basic_cmd.dst_endpoint = HA_ESP_VOLTAGE_SENSOR_ENDPOINT,
        .zcl_basic_cmd.src_endpoint = HA_ESP_VOLTAGE_SENSOR_ENDPOINT,
        .address_mode = ESP_ZB_APS_ADDR_MODE_16_ENDP_PRESENT,
        .clusterID = ESP_ZB_ZCL_CLUSTER_ID_ANALOG_INPUT,
    };

    static float reportable_change_analog = 0.1f; // 0.1V change

    esp_zb_zcl_config_report_record_t records_analog[] = {
        {
            .direction = ESP_ZB_ZCL_REPORT_DIRECTION_SEND,
            .attributeID = ESP_ZB_ZCL_ATTR_ANALOG_INPUT_PRESENT_VALUE_ID,
            .attrType = ESP_ZB_ZCL_ATTR_TYPE_SINGLE,
            .min_interval = 5,
            .max_interval = 60,
            .reportable_change = &reportable_change_analog,
        },
    };

    report_cmd_analog.record_number = sizeof(records_analog) / sizeof(esp_zb_zcl_config_report_record_t);
    report_cmd_analog.record_field = records_analog;
    esp_zb_lock_acquire(portMAX_DELAY);
    esp_zb_zcl_config_report_cmd_req(&report_cmd_analog);
    esp_zb_lock_release();

    ESP_LOGI(TAG, "Configured reporting for Power Config and Analog Input");
}

/**
 * @brief Manually flush a ZCL attribute report to the coordinator.
 *
 * This function sends a
 * report command to the coordinator (address 0x0000). This is used to proactively
 * send data without waiting for a poll or request.
 *
 * @param cluster_id The cluster ID of the attribute.
 * @param attr_id The attribute ID to report.
 */
esp_err_t esp_zb_zcl_manual_report(uint16_t cluster_id, uint16_t attr_id)
{
    // Construct and send a report command to the coordinator.
    esp_zb_zcl_report_attr_cmd_t report_attr_cmd = {
        .address_mode = ESP_ZB_APS_ADDR_MODE_16_ENDP_PRESENT,
        .zcl_basic_cmd.dst_addr_u.addr_short = COORDINATOR_ADDR,
        .zcl_basic_cmd.dst_endpoint = HA_ESP_VOLTAGE_SENSOR_ENDPOINT,
        .zcl_basic_cmd.src_endpoint = HA_ESP_VOLTAGE_SENSOR_ENDPOINT,
        .clusterID = cluster_id,
        .attributeID = attr_id,
        .direction = ESP_ZB_ZCL_CMD_DIRECTION_TO_CLI, // from server to client
    };

    esp_zb_lock_acquire(portMAX_DELAY);
    esp_err_t ret = esp_zb_zcl_report_attr_cmd_req(&report_attr_cmd);
    esp_zb_lock_release();

    return ret;
}

/**
 * @brief Handles signals from the Zigbee stack.
 *
 * This function processes events like stack initialization, network joining, leaving, etc.
 * It's the main entry point for Zigbee event handling.
 *
 * @param signal_struct A pointer to the structure containing signal information.
 */
void esp_zb_app_signal_handler(esp_zb_app_signal_t *signal_struct)
{
    uint32_t *p_sg_p = signal_struct->p_app_signal;
    esp_err_t err_status = signal_struct->esp_err_status;
    esp_zb_app_signal_type_t sig_type = *p_sg_p;

    switch (sig_type) {
    case ESP_ZB_ZDO_SIGNAL_SKIP_STARTUP:
        // This signal indicates the Zigbee stack is initialized and ready.
        ESP_LOGI(TAG, "Zigbee stack initialized");
        // Start the top-level commissioning process (e.g., network steering, forming).
        esp_zb_bdb_start_top_level_commissioning(ESP_ZB_BDB_MODE_INITIALIZATION);
        break;
    case ESP_ZB_BDB_SIGNAL_DEVICE_FIRST_START:
    case ESP_ZB_BDB_SIGNAL_DEVICE_REBOOT:
        // This signal indicates the device is starting up, either for the first time or after a reboot.
        if (err_status == ESP_OK) {
            ESP_LOGI(TAG, "Device started up in %s factory-reset mode",
                     esp_zb_bdb_is_factory_new() ? "" : "non");
            if (esp_zb_bdb_is_factory_new()) {
                // If the device is factory-new, it will try to join a network.
                ESP_LOGI(TAG, "Factory new device - starting network steering");
                esp_zb_bdb_start_top_level_commissioning(ESP_ZB_BDB_MODE_NETWORK_STEERING);
            } else {
                // If not factory-new, it will try to rejoin its previous network.
                ESP_LOGI(TAG, "Non-factory device - attempting to rejoin network");
                esp_zb_bdb_start_top_level_commissioning(ESP_ZB_BDB_MODE_NETWORK_STEERING);
            }
            ESP_LOGI(TAG, "PAN ID: 0x%04hx, Channel: %d", esp_zb_get_pan_id(), esp_zb_get_current_channel());
        } else {
            // Handle initialization failure.
            ESP_LOGE(TAG, "Failed to initialize Zigbee stack (status: %s, code: 0x%x)", esp_err_to_name(err_status), err_status);
            ESP_LOGE(TAG, "Stack initialization error - device may need reset");
        }
        break;
    case ESP_ZB_BDB_SIGNAL_STEERING:
        // This signal is received during the network steering (joining) process.
        if (err_status == ESP_OK) {
            // Successfully joined a network.
            ESP_LOGI(TAG, "Successfully joined network");
            zigbee_connected = true;
            // Let the coordinator configure reporting instead of doing it ourselves.
            // Device-side reporting configuration is disabled because:
            // 1. Standard practice: coordinator should configure reporting
            // 2. Battery voltage cannot be configured for reporting (SDK limitation)
            // 3. Battery percentage is successfully configured by the coordinator
            configure_reporting();  // See function documentation for details
            configure_binding();
            esp_zb_ieee_addr_t extended_pan_id;
            esp_zb_get_extended_pan_id(extended_pan_id);
            ESP_LOGI(TAG, "Joined network (Extended PAN ID: %02x:%02x:%02x:%02x:%02x:%02x:%02x:%02x, PAN ID: 0x%04hx, Channel:%d)",
                     extended_pan_id[7], extended_pan_id[6], extended_pan_id[5], extended_pan_id[4],
                     extended_pan_id[3], extended_pan_id[2], extended_pan_id[1], extended_pan_id[0],
                     esp_zb_get_pan_id(), esp_zb_get_current_channel());
        } else {
            // Failed to join, will be retried by the main loop logic.
            ESP_LOGI(TAG, "Network steering failed (status: %s), retrying in 1 second...", esp_err_to_name(err_status));
            // Simple retry without deprecated scheduler - will be handled by task
            esp_zb_scheduler_alarm(retry_steering_cb, 0, 1000);
        }
        break;
    case ESP_ZB_ZDO_SIGNAL_LEAVE:
        // This signal indicates the device has left the network.
        ESP_LOGI(TAG, "Leave network, retrying in 3 seconds...");
        zigbee_connected = false; // Enable retrying to join a network again.
        esp_zb_scheduler_alarm(retry_steering_cb, 0, 3000);
        break;
    default:
        // Catch-all for other unhandled signals.
        ESP_LOGI(TAG, "ZDO signal: %s (0x%x), status: %s, code: 0x%x", esp_zb_zdo_signal_to_string(sig_type), sig_type,
                 esp_err_to_name(err_status), err_status);
        break;
    }
}

/**
 * @brief Handles incoming ZCL attribute write commands.
 *
 * This function is called when a remote Zigbee device (like a coordinator)
 * writes to an attribute on this device.
 * 
 * It intercepts writes to the Power Configuration cluster attributes to implement
 * data persistence. When a threshold or alarm mask is updated remotely, this
 * handler saves the new value to Non-Volatile Storage (NVS) so it can be 
 * restored after a reboot.
 *
 * @param message A pointer to the message containing attribute information.
 * @return ESP_OK on success, or an error code on failure.
 */
static esp_err_t zb_attribute_handler(const esp_zb_zcl_set_attr_value_message_t *message)
{
    esp_err_t ret = ESP_OK;
    ESP_RETURN_ON_FALSE(message, ESP_FAIL, TAG, "Empty message");
    ESP_RETURN_ON_FALSE(message->info.status == ESP_ZB_ZCL_STATUS_SUCCESS, ESP_ERR_INVALID_ARG, TAG, "Received message: error status(%d)",
                        message->info.status);

    ESP_LOGI(TAG, "Received ZCL attribute(0x%x) set to cluster(0x%x)",
             message->attribute.id, message->info.cluster);

    // Persistence Logic: Intercept writes to Power Configuration Cluster
    if (message->info.cluster == ESP_ZB_ZCL_CLUSTER_ID_POWER_CONFIG) {
        if (message->attribute.id == ESP_ZB_ZCL_ATTR_POWER_CONFIG_BATTERY_ALARM_MASK_ID ||
            message->attribute.id == ESP_ZB_ZCL_ATTR_POWER_CONFIG_BATTERY_VOLTAGE_MIN_THRESHOLD_ID ||
            message->attribute.id == ESP_ZB_ZCL_ATTR_POWER_CONFIG_BATTERY_VOLTAGE_THRESHOLD1_ID) {

            nvs_handle_t my_handle;
            esp_err_t err = nvs_open("storage", NVS_READWRITE, &my_handle);
            if (err == ESP_OK) {
                // Extract value from the message (u8 type)
                uint8_t val = *(uint8_t*)message->attribute.data.value;
                const char* key = "";
                
                // Map Attribute ID to NVS Key
                if (message->attribute.id == ESP_ZB_ZCL_ATTR_POWER_CONFIG_BATTERY_ALARM_MASK_ID) key = "alarm_mask";
                else if (message->attribute.id == ESP_ZB_ZCL_ATTR_POWER_CONFIG_BATTERY_VOLTAGE_MIN_THRESHOLD_ID) key = "min_thresh";
                else if (message->attribute.id == ESP_ZB_ZCL_ATTR_POWER_CONFIG_BATTERY_VOLTAGE_THRESHOLD1_ID) key = "thresh1";

                // Save to NVS
                err = nvs_set_u8(my_handle, key, val);
                if (err == ESP_OK) {
                    err = nvs_commit(my_handle);
                    if (err == ESP_OK) {
                        ESP_LOGI(TAG, "Saved attribute 0x%x value %d to NVS key '%s'", message->attribute.id, val, key);
                    }
                }
                nvs_close(my_handle);
            }
            if (err != ESP_OK) {
                ESP_LOGE(TAG, "Failed to save attribute to NVS: %s", esp_err_to_name(err));
            }
        }
    }

    return ret;
}

/**
 * @brief Handles responses to ZCL attribute read commands.
 *
 * This function is called when this device receives a response to a read
 * attribute command it previously sent. It logs the content of the response.
 *
 * @param message A pointer to the message containing the read attribute response.
 * @return ESP_OK on success.
 */
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

/**
 * @brief Dispatches Zigbee action callbacks to the appropriate handlers.
 *
 * This function acts as a router for different Zigbee core actions, calling
 * the relevant handler based on the callback ID.
 *
 * @param callback_id The ID of the callback being invoked.
 * @param message A pointer to the message data associated with the callback.
 * @return ESP_OK on success, or an error code from the handler.
 */
esp_err_t zb_action_handler(esp_zb_core_action_callback_id_t callback_id, const void *message)
{
    esp_err_t ret = ESP_OK;
    switch (callback_id) {
    case ESP_ZB_CORE_SET_ATTR_VALUE_CB_ID:
        // Handle attribute write requests.
        ret = zb_attribute_handler((esp_zb_zcl_set_attr_value_message_t *)message);
        break;
    case ESP_ZB_CORE_CMD_READ_ATTR_RESP_CB_ID:
        // Handle attribute read responses.
        ret = zb_read_attr_resp_handler((esp_zb_zcl_cmd_read_attr_resp_message_t *)message);
        break;
    case ESP_ZB_CORE_CMD_DEFAULT_RESP_CB_ID:
        esp_zb_zcl_cmd_default_resp_message_t *cmd_default_resp_msg = (esp_zb_zcl_cmd_default_resp_message_t *)message;
        ESP_LOGI(TAG,
            "Default Response: cluster_id=0x%04X cmd_id=0x%02X cmd_dir=0x%02X cmd_is_common=0x%02X resp_to_cmd=0x%02X status=0x%02X",
            cmd_default_resp_msg->info.cluster,
            cmd_default_resp_msg->info.command.id,
            cmd_default_resp_msg->info.command.direction,
            cmd_default_resp_msg->info.command.is_common,
            cmd_default_resp_msg->resp_to_cmd,
            cmd_default_resp_msg->status_code
        );

        if (cmd_default_resp_msg->status_code == ESP_ZB_ZCL_STATUS_SUCCESS) {
            ESP_LOGI(TAG, "Command succeeded");
        } else {
            ESP_LOGW(TAG, "Command failed with status=0x%02X", cmd_default_resp_msg->status_code);
        }
        break;
    case ESP_ZB_CORE_CMD_REPORT_CONFIG_RESP_CB_ID:
        esp_zb_zcl_cmd_config_report_resp_message_t *cmd_config_report_resp_msg = (esp_zb_zcl_cmd_config_report_resp_message_t *)message;
        esp_zb_zcl_config_report_resp_variable_t *record = cmd_config_report_resp_msg->variables;
        ESP_LOGI(TAG, "Config Report Response: cluster_id=0x%04X, cmd_id=0x%02X, cmd_dir=0x%02X",
                cmd_config_report_resp_msg->info.cluster, cmd_config_report_resp_msg->info.command.id, cmd_config_report_resp_msg->info.command.direction);
        while (record) {
            ESP_LOGI(TAG, "  attribute_id=0x%04X, direction=0x%02X, status=0x%02X",
                        record->attribute_id,
                        record->direction,
                        record->status);
            record = record->next;
        }
        break;
    default:
        ESP_LOGW(TAG, "Receive Unhandled Zigbee action (0x%x) callback", callback_id);
        break;
    }
    return ret;
}
