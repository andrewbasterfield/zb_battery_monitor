import * as m from 'zigbee-herdsman-converters/lib/modernExtend';

export default {
    zigbeeModel: ['esp32h2'],
    model: 'esp32h2',
    vendor: 'ESPRESSIF',
    description: 'Automotive Voltmeter Thingy',
    extend: [
        m.battery({
            voltage: true,                    // expose voltage in mV
            voltageReporting: false,          // disable reporting - SDK doesn't support it for voltage
            percentage: true,
            percentageReporting: true,        // percentage reporting works fine
            percentageReportingConfig: {min: 5, max: 60, change: 1},
            lowStatus: true,                  // enable battery_low from batteryAlarmState
            lowStatusReportingConfig: {min: 1, max: 300, change: 1},
        }),
        m.numeric({
            name: 'voltage_analog',
            cluster: 'genAnalogInput',
            attribute: 'presentValue',
            description: 'Voltage (Analog Input)',
            unit: 'V',
            precision: 2,
            access: 'STATE',  // Read-only sensor that auto-reports changes (not user-editable)
            reporting: {min: 5, max: 60, change: 0.1},
        }),
    ],
    meta: {},
};
