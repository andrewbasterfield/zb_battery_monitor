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
        }),
        m.numeric({
            name: 'voltage_analog',
            cluster: 'genAnalogInput',
            attribute: 'presentValue',
            description: 'Voltage (Analog Input)',
            unit: 'V',
            reporting: {min: 5, max: 60, change: 0.1},
        }),
    ],
    meta: {},
};
