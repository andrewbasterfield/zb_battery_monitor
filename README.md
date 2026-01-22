# Zigbee Battery Monitor

A Zigbee-enabled battery monitor based on the ESP32-H2. This device monitors 12V lead-acid battery voltage and reports it to a Zigbee Coordinator (such as Home Assistant via ZHA or Zigbee2MQTT).

## Features

-   **Voltage Monitoring:** Measures 12V lead-acid battery voltage via a voltage divider connected to the ADC.
-   **Zigbee Reporting:**
    -   **Battery Voltage:** Reports voltage in 0.1V increments.
    -   **Battery Percentage:** Calculates an estimated percentage based on configurable thresholds (12.1V - 14.8V).
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

-   `main/zb_battery_monitor.c`: Main application logic. Handles Zigbee stack initialization, the main event loop, and reporting logic.
-   `main/adc-sensor.c` / `.h`: Handles ADC initialization, calibration, and reading. Includes a voltage simulation mode.
-   `main/zigbee-protocol.c` / `.h`: Manages Zigbee stack configuration, reporting (automatic and manual), and callbacks.
-   `main/idf_component.yml`: Project dependencies (`esp-zigbee-lib`, `esp-zboss-lib`).

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
2.  **Zigbee Setup:** It registers a "Power Configuration" cluster to report battery status.
3.  **Measurement Loop:** A FreeRTOS task (`voltage_measurement_task`) wakes up periodically to read the voltage (or simulate it).
4.  **Data Processing:** The raw voltage is converted to a percentage and checked against alarm thresholds.
5.  **Reporting:**
    -   If the value changes significantly or an alarm state changes, the attributes are updated.
    -   The Zigbee stack handles reporting these values to the coordinator.

## Current Status & Known Issues

*Reference: CONTEXT.md*

-   **Reporting Issue:** The device currently fails to configure automatic reporting for `BatteryVoltage` and `BatteryPercentageRemaining`. The Zigbee stack returns `UNSUPPORTED_ATTRIBUTE` or `FAILURE` when the device attempts to configure reporting on itself.
-   **Workaround:** Manual reporting was attempted but caused crashes. Currently, manual reporting is commented out in `main/zb_battery_monitor.c`.
-   **Next Steps:** The plan is to simplify the cluster configuration by removing non-essential attributes to isolate the reporting issue.
