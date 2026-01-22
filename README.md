# Zigbee Battery Monitor (WIP)

A Zigbee-enabled battery monitor based on the ESP32-H2. This device monitors 12V lead-acid battery voltage and reports it to a Zigbee Coordinator (such as Home Assistant via ZHA or Zigbee2MQTT).

## Features

-   **Voltage Monitoring:** Measures 12V lead-acid battery voltage via a voltage divider connected to the ADC.
-   **Zigbee Reporting:**
    -   **Battery Voltage:** Reports voltage in 0.1V increments (readable, but see [Known Issues](#known-issues) regarding automatic reporting).
    -   **Battery Percentage:** Calculates an estimated percentage based on configurable thresholds (12.1V - 14.8V). Automatically reports to coordinator.
    -   **Alarms:** Detects and reports Low, High, and Critical voltage states.
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
-   `LOW_VOLTAGE_THRESHOLD`: Voltage considered "Low".
-   `HIGH_VOLTAGE_THRESHOLD`: Voltage considered "High" (alternator charging).
-   `CRITICAL_LOW_VOLTAGE_THRESHOLD`: Critical alarm level.
-   `MEASUREMENT_INTERVAL_MS`: How often to read the sensor.

## How It Works

1.  **Initialization:** The app initializes NVS, the ADC sensor, and the Zigbee stack.
2.  **Zigbee Setup:** It registers a "Power Configuration" cluster with battery voltage and percentage attributes.
3.  **Measurement Loop:** A FreeRTOS task (`voltage_measurement_task`) wakes up periodically to read the voltage (or simulate it).
4.  **Data Processing:** The raw voltage is converted to a percentage and checked against alarm thresholds.
5.  **Attribute Updates:** Both voltage and percentage attributes are updated in the Zigbee cluster whenever new measurements are taken.
6.  **Reporting:**
    -   **Battery Percentage:** Automatically reports to the coordinator when configured (handled by coordinator during device setup).
    -   **Battery Voltage:** Updated locally but cannot auto-report due to SDK limitation. Can be read/polled by coordinator on demand.

## Zigbee2MQTT Configuration

This project includes a custom converter file (`esp_voltmeter.mjs`) for use with zigbee2mqtt. 

**Setup:**
1. Copy `esp_voltmeter.mjs` from the project root to your zigbee2mqtt `external_converters` directory.
   - Typical location: `/opt/zigbee2mqtt/data/external_converters/` or `/var/lib/iot-stack/zigbee2mqtt-data/external_converters/`
   - Check your zigbee2mqtt configuration for the exact path.
2. Restart zigbee2mqtt to load the new converter.

**Converter Configuration:**
- `voltageReporting: false` - Disabled because the ESP Zigbee SDK doesn't support automatic reporting for the voltage attribute (see [Known Issues](#known-issues)).
- `percentageReporting: true` - Enabled and working correctly.
- The voltage attribute is still exposed and readable, but requires manual polling rather than automatic updates.

## Known Issues

### Battery Voltage Reporting Limitation

**Issue:** The `BatteryVoltage` attribute cannot be configured for automatic reporting. The coordinator will receive `UNREPORTABLE_ATTRIBUTE` errors when attempting to configure reporting.

**Root Cause:** This is a limitation of the ESP Zigbee SDK. The SDK defines `BATTERY_VOLTAGE` as `READ_ONLY` without the `REPORTING` access flag, while `BATTERY_PERCENTAGE_REMAINING` is correctly defined as `READ_ONLY | REPORTING`. The SDK's `esp_zb_power_config_cluster_add_attr()` function doesn't allow overriding the attribute access flags.

**Impact:**
- ✅ Battery voltage is readable by the coordinator (can be polled/read on demand)
- ✅ Battery percentage reports automatically
- ❌ Battery voltage does not update automatically (requires manual read/poll)

**Workarounds:**
- The voltage attribute is updated in the device's local storage and can be read by the coordinator when requested
- zigbee2mqtt can be configured to poll the voltage attribute periodically
- Consider using battery percentage for automatic monitoring, as it reports correctly

**Technical Details:**
- See `managed_components/espressif__esp-zboss-lib/include/zcl/zb_zcl_power_config.h` lines 474-480 (voltage) vs 529-536 (percentage)
- Voltage: `ZB_ZCL_ATTR_ACCESS_READ_ONLY`
- Percentage: `ZB_ZCL_ATTR_ACCESS_READ_ONLY | ZB_ZCL_ATTR_ACCESS_REPORTING`
