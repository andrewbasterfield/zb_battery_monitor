#pragma once

#include "esp_zigbee_core.h"

/**
 * @brief Configures automatic attribute reporting to the coordinator.
 *
 * This function sets up the device to automatically report voltage and battery percentage
 * based on time intervals and value changes.
 */
void configure_reporting(void);

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
esp_err_t zb_action_handler(esp_zb_core_action_callback_id_t callback_id, const void *message);

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
esp_err_t esp_zb_zcl_manual_report(uint8_t endpoint, uint16_t cluster_id, uint16_t attr_id);

extern bool zigbee_connected;              // Flag to indicate if connected to a Zigbee network
