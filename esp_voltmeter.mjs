import * as m from 'zigbee-herdsman-converters/lib/modernExtend';
import { presets as e, access as ea } from 'zigbee-herdsman-converters/lib/exposes';

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
        m.poll({
            key: 'legacy_battery_voltage',
            defaultIntervalSeconds: 60,
            poll: async (device) => {
                try {
                    await device.getEndpoint(1).read('genPowerCfg', ['batteryVoltage']);
                } catch (error) {
                    // Fail silently to avoid log spam
                }
            },
        }),
        m.numeric({
            name: 'battery_alarm_state',
            cluster: 'genPowerCfg',
            attribute: 'batteryAlarmState',
            description: 'Battery Alarm State',
            access: 'STATE_GET',
        }),
        m.numeric({
            name: 'battery_alarm_mask',
            cluster: 'genPowerCfg',
            attribute: 'batteryAlarmMask',
            description: 'Battery Alarm Mask',
            access: 'STATE_GET',
        }),
        {
            isModernExtend: true,
            exposes: [
                e.numeric('battery_voltage_min_threshold', ea.ALL).withUnit('100mV').withDescription('Battery Voltage Minimum Threshold'),
                e.numeric('battery_voltage_threshold1', ea.ALL).withUnit('100mV').withDescription('Battery Voltage Threshold 1'),
            ],
            fromZigbee: [{
                cluster: 'genPowerCfg',
                type: ['attributeReport', 'readResponse'],
                convert: (model, msg, publish, options, meta) => {
                    const payload = {};
                    if (msg.data['batteryVoltMinThres'] !== undefined) payload.battery_voltage_min_threshold = msg.data['batteryVoltMinThres'];
                    if (msg.data['batteryVoltThres1'] !== undefined) payload.battery_voltage_threshold1 = msg.data['batteryVoltThres1'];
                    return payload;
                },
            }],
            toZigbee: [{
                key: ['battery_voltage_min_threshold', 'battery_voltage_threshold1'],
                convertGet: async (entity, key, meta) => {
                    const lookup = {'battery_voltage_min_threshold': 'batteryVoltMinThres', 'battery_voltage_threshold1': 'batteryVoltThres1'};
                    await entity.read('genPowerCfg', [lookup[key]]);
                },
                convertSet: async (entity, key, value, meta) => {
                    const lookup = {'battery_voltage_min_threshold': 'batteryVoltMinThres', 'battery_voltage_threshold1': 'batteryVoltThres1'};
                    await entity.write('genPowerCfg', {[lookup[key]]: value});
                },
            }],
            configure: [
                m.setupConfigureForReading('genPowerCfg', ['batteryVoltMinThres', 'batteryVoltThres1', 'batteryAlarmMask']),
            ],
        },
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