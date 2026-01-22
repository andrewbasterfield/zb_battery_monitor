#pragma once

#include "esp_zigbee_core.h"
#include "zcl/esp_zigbee_zcl_power_config.h"
#include "zcl/esp_zigbee_zcl_common.h"

#define MANUFACTURER_NAME "\x09""ESPRESSIF"
#define MODEL_IDENTIFIER "\x07" CONFIG_IDF_TARGET

// --- Hardware Configuration ---
#define MEASUREMENT_INTERVAL_MS 5000           // Interval between voltage measurements in milliseconds

// --- Zigbee Configuration ---
#define INSTALLCODE_POLICY_ENABLE false // Set to true to enable install code policy for joining, false to disable
#define ED_AGING_TIMEOUT ESP_ZB_ED_AGING_TIMEOUT_64MIN // Timeout for end device to be considered aged out by parent
#define ED_KEEP_ALIVE 3000              // Keep-alive interval for end device in milliseconds
#define HA_ESP_VOLTAGE_SENSOR_ENDPOINT 1 // Zigbee endpoint for this device
#define ESP_ZB_PRIMARY_CHANNEL_MASK ESP_ZB_TRANSCEIVER_ALL_CHANNELS_MASK // Scan all channels to find a network

// These thresholds define the different voltage states (in volts).
#define LOW_VOLTAGE_THRESHOLD 12.1f    // Below this, voltage is considered "low"
#define HIGH_VOLTAGE_THRESHOLD 14.8f   // Above this, voltage is considered "high" (e.g., alternator overcharging)
#define CRITICAL_LOW_VOLTAGE_THRESHOLD 11.8f // Below this, voltage is critically low

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
 * @param cluster_id The cluster ID of the attribute.
 * @param attr_id The attribute ID to report.
 * @param value A pointer to the attribute's value.
 */
esp_err_t esp_zb_zcl_manual_report(uint16_t cluster_id, uint16_t attr_id);

extern bool zigbee_connected;              // Flag to indicate if connected to a Zigbee network
