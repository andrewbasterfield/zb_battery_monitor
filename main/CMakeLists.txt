idf_component_register(SRCS "zigbee-protocol.c" "zb_battery_monitor.c" "adc-sensor.c"
                    INCLUDE_DIRS "."
                    REQUIRES nvs_flash esp_adc)
