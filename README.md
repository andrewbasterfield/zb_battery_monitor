# Zigbee Battery Monitor (WIP)

A Zigbee-enabled battery monitor based on the ESP32-H2. This device monitors 12V lead-acid battery voltage and reports it to a Zigbee Coordinator (such as Home Assistant via ZHA or Zigbee2MQTT).

## Features

-   **Voltage Monitoring:** Measures 12V lead-acid battery voltage via a voltage divider connected to the ADC.
-   **Zigbee Reporting:**
    -   **Battery Voltage (Precise):** Automatically reports precise voltage (float) via the **Analog Input** cluster (0x000C).
    -   **Battery Percentage:** Automatically reports estimated percentage based on AGM discharge curve (11.85V - 12.85V).
    -   **Battery Voltage (Legacy):** Reports via Power Configuration cluster (requires polling due to SDK limitations).
    -   **Battery Alarms:** Detects and reports Critical Low and Low voltage states with real-time notifications to coordinator.
    -   **Dynamic Configuration:** Alarm thresholds and masks are configurable via Zigbee and persist across reboots (saved to NVS).
-   **Device Roles:** Configurable as a **Router** (always on, relays messages) or **End Device** (sleeps, low power).
-   **Simulation Mode:** Includes a software simulation mode for testing without hardware.

## Hardware Requirements

-   **Microcontroller:** Espressif ESP32-H2 (supports Zigbee).
-   **Voltage Divider:** A resistive voltage divider to scale the 12V-15V battery voltage down to the ESP32's ADC range (0-3.3V).
    -   Default configuration assumes a ratio of ~5.7 (e.g., 47kΩ / 10kΩ).

## Software Requirements

-   **ESP-IDF:** Version 5.1 or newer (required for ESP32-H2 and Zigbee support).

## Project Structure

-   `main/zb_battery_monitor.c`: Main application logic. Handles Zigbee stack initialization, the main event loop, and attribute updates.
-   `main/adc-sensor.c` / `.h`: Handles ADC initialization, calibration, and reading. Includes a voltage simulation mode.
-   `main/zigbee-protocol.c` / `.h`: Manages Zigbee stack configuration, binding, and callbacks. Note: Reporting configuration is handled by the coordinator.
-   `main/idf_component.yml`: Project dependencies (`esp-zigbee-lib`, `esp-zboss-lib`).
-   `esp_voltmeter.mjs`: Custom zigbee2mqtt converter file. Copy this to your zigbee2mqtt `external_converters` directory.

## Installation & Build

1.  **Set up ESP-IDF:** Ensure you have the ESP-IDF environment set up.
    ```bash
    . $HOME/esp/esp-idf/export.sh
    ```

2.  **Set Target:**
    ```bash
    idf.py set-target esp32h2
    ```

3.  **Configuration (Optional):**
    You can adjust settings in the code macros (see [Configuration](#configuration) below).

4.  **Build:**
    ```bash
    idf.py build
    ```

5.  **Flash & Monitor:**
    ```bash
    idf.py flash monitor
    ```

## Configuration

Key settings can be modified in the source files:

**`main/zb_battery_monitor.c`:**
-   `ROUTER_MODE`: Set to `1` for Router (mains powered), `0` for End Device (battery powered).
-   `custom_ieee_addr`: Set a custom MAC address if needed.

**`main/adc-sensor.c`:**
-   `USE_SIMULATED_VOLTAGE`: Set to `1` to simulate voltage readings (for testing without a voltage divider), `0` to use real ADC.
-   `VOLTAGE_DIVIDER_CHANNEL`: The ADC channel used (default `ADC_CHANNEL_0`). *Note: Check your specific board's pinout for the corresponding GPIO.*
-   `VOLTAGE_DIVIDER_RATIO`: Adjust this float value to match your specific resistor values.

**`main/zigbee-protocol.h`:**
-   `CRITICAL_LOW_VOLTAGE_THRESHOLD`: Default 11.85V (configurable via Zigbee).
-   `LOW_VOLTAGE_THRESHOLD`: Default 12.2V (configurable via Zigbee).
-   `FULLY_CHARGED_VOLTAGE_THRESHOLD`: 12.85V - Reference point for 100% charge calculation.
-   `MEASUREMENT_INTERVAL_MS`: How often to read the sensor.

## Dynamic Configuration (Zigbee2MQTT)

The device supports changing alarm thresholds at runtime. These settings are saved to non-volatile storage (NVS) and persist after a reboot.

-   **`battery_voltage_min_threshold`**: Sets the critical low voltage alarm threshold (unit: 100mV). Example: `118` = 11.8V.
-   **`battery_voltage_threshold1`**: Sets the low voltage warning threshold (unit: 100mV). Example: `122` = 12.2V.
-   **`battery_alarm_mask`**: Configures which alarms are enabled (Bitmask).

To change these, go to the **Exposes** tab in Zigbee2MQTT (or use the Dev Console to write to the `genPowerCfg` cluster).

## How It Works

1.  **Initialization:** The app initializes NVS, loads stored thresholds (or defaults), sets up the ADC, and starts the Zigbee stack.
2.  **Zigbee Setup:** It registers a "Power Configuration" cluster with voltage, percentage, and configurable threshold attributes.
3.  **Measurement Loop:** A FreeRTOS task (`voltage_measurement_task`) wakes up periodically to read the voltage.
4.  **Data Processing:** The raw voltage is converted to a percentage and checked against the *current* alarm thresholds.
5.  **Persistence:** If you update a threshold via Zigbee, the device intercepts the write command, updates the active value, and saves it to NVS.
6.  **Reporting:**
    -   **Battery Percentage:** Automatically reports to the coordinator (min 5s, max 60s, change ≥1%).
    -   **Battery Alarm State:** Automatically reports when voltage crosses thresholds (min 1s, max 300s). Immediate manual report sent to coordinator for real-time notification.
    -   **Analog Voltage:** Continuously reports from Analog Input cluster (min 5s, max 60s, change ≥0.1V).
    -   **Battery Voltage (Legacy):** Updated locally but cannot auto-report due to SDK limitation.

## Zigbee2MQTT Configuration

This project includes a custom converter file (`esp_voltmeter.mjs`) for use with zigbee2mqtt. 

**Setup:**
1. Copy `esp_voltmeter.mjs` from the project root to your zigbee2mqtt `external_converters` directory.
   - Typical location: `/opt/zigbee2mqtt/data/external_converters/` or `/var/lib/iot-stack/zigbee2mqtt-data/external_converters/`
   - Check your zigbee2mqtt configuration for the exact path.
2. Restart zigbee2mqtt to load the new converter.

**Converter Configuration:**
- `battery` - Battery percentage (0-100%) with automatic reporting enabled.
- `battery_low` - Binary indicator of low battery condition (true when alarm state bits are set).
- `voltage_analog` - **Primary voltage reading**: Displays precise voltage from Analog Input cluster. Read-only sensor, updates automatically.
- `voltage` - Legacy voltage attribute in mV (readable but no auto-reporting due to SDK limitation).

## Known Issues

### Battery Voltage Reporting Limitation

**Issue:** The `BatteryVoltage` attribute cannot be configured for automatic reporting. The coordinator will receive `UNREPORTABLE_ATTRIBUTE` errors when attempting to configure reporting.

**Root Cause:** This is a limitation of the ESP Zigbee SDK. The SDK defines `BATTERY_VOLTAGE` as `READ_ONLY` without the `REPORTING` access flag, while `BATTERY_PERCENTAGE_REMAINING` is correctly defined as `READ_ONLY | REPORTING`. The SDK's `esp_zb_power_config_cluster_add_attr()` function doesn't allow overriding the attribute access flags.

**Impact:**
- ✅ Battery voltage is readable by the coordinator (can be polled/read on demand)
- ✅ Battery percentage reports automatically
- ❌ Battery voltage does not update automatically (requires manual read/poll)

**Workarounds:**
- **[Implemented]** Use the **Analog Input** cluster (`voltage_analog`) which supports automatic reporting.
- The legacy voltage attribute is updated in the device's local storage and can be read by the coordinator when requested.
- zigbee2mqtt can be configured to poll the legacy voltage attribute periodically if needed.

**Technical Details:**
- See `managed_components/espressif__esp-zboss-lib/include/zcl/zb_zcl_power_config.h` lines 474-480 (voltage) vs 529-536 (percentage)
- Voltage: `ZB_ZCL_ATTR_ACCESS_READ_ONLY`
- Percentage: `ZB_ZCL_ATTR_ACCESS_READ_ONLY | ZB_ZCL_ATTR_ACCESS_REPORTING`
