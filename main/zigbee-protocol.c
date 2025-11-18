#include "esp_zigbee_core.h"
#include "zcl/esp_zigbee_zcl_common.h"
#include "esp_check.h"

#include "zigbee-protocol.h"

bool zigbee_connected = false;

#define HA_ESP_VOLTAGE_SENSOR_ENDPOINT 1 // Zigbee endpoint for this device
#define ESP_ZB_ZCL_ATTR_POWER_CONFIG_BATTERY_VOLTAGE_ID 0x0020
#define ESP_ZB_ZCL_ATTR_POWER_CONFIG_BATTERY_PERCENTAGE_REMAINING_ID 0x0021
#define ESP_ZB_PRIMARY_CHANNEL_MASK ESP_ZB_TRANSCEIVER_ALL_CHANNELS_MASK // Scan all channels to find a network
#define TAG "AUTOMOTIVE_VOLTMETER_ZIGBEE"


/**
* @brief Configures automatic attribute reporting to the coordinator.
 *
 * This function sets up the device to automatically report voltage and battery percentage
 * based on time intervals and value changes.
 */
void configure_reporting(void) {
    esp_zb_zcl_config_report_cmd_t report_cmd = {
        .zcl_basic_cmd.dst_addr_u.addr_short = 0x0000, // Report to coordinator
        .zcl_basic_cmd.dst_endpoint = 1,
        .zcl_basic_cmd.src_endpoint = HA_ESP_VOLTAGE_SENSOR_ENDPOINT,
        .address_mode = ESP_ZB_APS_ADDR_MODE_16_ENDP_PRESENT,
        .clusterID = ESP_ZB_ZCL_CLUSTER_ID_POWER_CONFIG,
    };

    uint8_t reportable_change_voltage = 1;    // 0.1V change
    uint8_t reportable_change_percentage = 2; // 1% change (in 0.5% units)

    esp_zb_zcl_config_report_record_t records[] = {
        {
            .direction = ESP_ZB_ZCL_CMD_DIRECTION_TO_CLI,
            .attributeID = ESP_ZB_ZCL_ATTR_POWER_CONFIG_BATTERY_VOLTAGE_ID,
            .attrType = ESP_ZB_ZCL_ATTR_TYPE_U8,
            .min_interval = 0,
            .max_interval = 60,
            .reportable_change = &reportable_change_voltage,
        },
        {
            .direction = ESP_ZB_ZCL_CMD_DIRECTION_TO_CLI,
            .attributeID = ESP_ZB_ZCL_ATTR_POWER_CONFIG_BATTERY_PERCENTAGE_REMAINING_ID,
            .attrType = ESP_ZB_ZCL_ATTR_TYPE_U8,
            .min_interval = 0,
            .max_interval = 60,
            .reportable_change = &reportable_change_percentage,
        },
        /*{
            .direction = ESP_ZB_ZCL_CMD_DIRECTION_TO_CLI,
            .attributeID = ESP_ZB_ZCL_ATTR_POWER_CONFIG_BATTERY_ALARM_MASK_ID,
            .attrType = ESP_ZB_ZCL_ATTR_TYPE_U8,
            .min_interval = 0,
            .max_interval = 60,
        },*/
    };

    report_cmd.record_number = sizeof(records) / sizeof(records[0]);
    report_cmd.record_field = records;
    uint8_t tx = esp_zb_zcl_config_report_cmd_req(&report_cmd);
    ESP_LOGI(TAG, "Configured reporting, tx: %d", tx);
}

/**
 * @brief Manually sends a ZCL attribute report to the coordinator.
 *
 * This function first updates the local attribute value and then sends a
 * report command to the coordinator (address 0x0000). This is used to proactively
 * send data without waiting for a poll or request.
 *
 * @param endpoint The source endpoint of the attribute.
 * @param cluster_id The cluster ID of the attribute.
 * @param attr_id The attribute ID to report.
 * @param value A pointer to the attribute's value.
 */
esp_err_t esp_zb_zcl_manual_report(uint8_t endpoint, uint16_t cluster_id, uint16_t attr_id)
{
    // Construct and send a report command to the coordinator.
    esp_zb_zcl_report_attr_cmd_t report_cmd = {
        .address_mode = ESP_ZB_APS_ADDR_MODE_16_ENDP_PRESENT,
        .zcl_basic_cmd.dst_addr_u.addr_short = 0x0000, // Address 0x0000 is the coordinator.
        .zcl_basic_cmd.dst_endpoint = 1, // Assuming coordinator endpoint is 1.
        .zcl_basic_cmd.src_endpoint = endpoint,
        .clusterID = cluster_id,
        .attributeID = attr_id,
        .direction = ESP_ZB_ZCL_CMD_DIRECTION_TO_CLI,
    };

    return esp_zb_zcl_report_attr_cmd_req(&report_cmd);
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
        ESP_LOGI(TAG, "Primary channel mask: 0x%08x", (unsigned int)ESP_ZB_PRIMARY_CHANNEL_MASK);
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
            configure_reporting();
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
        }
        break;
    case ESP_ZB_ZDO_SIGNAL_LEAVE:
        // This signal indicates the device has left the network.
        ESP_LOGI(TAG, "Leave network");
        zigbee_connected = false; // Enable retrying to join a network again.
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
 * writes to an attribute on this device. Currently, it only logs the event.
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

    // Future logic to handle attribute writes can be added here.
    // For example, changing a configuration parameter.

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
        esp_zb_zcl_cmd_default_resp_message_t *msg = (esp_zb_zcl_cmd_default_resp_message_t *)message;
        ESP_LOGI(TAG,
            "Default Response: cluster_id=0x%04X cmd_id=0x%02X cmd_dir=0x%02X cmd_is_common=0x%02X resp_to_cmd=0x%02X status=0x%02X",
            msg->info.cluster,
            msg->info.command.id,
            msg->info.command.direction,
            msg->info.command.is_common,
            msg->resp_to_cmd,
            msg->status_code
        );

        if (msg->status_code == ESP_ZB_ZCL_STATUS_SUCCESS) {
            ESP_LOGI(TAG, "Command succeeded");
        } else {
            ESP_LOGW(TAG, "Command failed with status=0x%02X", msg->status_code);
        }
        break;
    default:
        ESP_LOGW(TAG, "Receive Zigbee action(0x%x) callback", callback_id);
        break;
    }
    return ret;
}
