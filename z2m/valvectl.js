const fz = require('zigbee-herdsman-converters/converters/fromZigbee');
const tz = require('zigbee-herdsman-converters/converters/toZigbee');
const exposes = require('zigbee-herdsman-converters/lib/exposes');
const e = exposes.presets, ea = exposes.access;

// Ground truth cross-checked against firmware/main/zigbee.h + zigbee.c (eng-t12, Task 21-22),
// NOT just the plan snippet — the plan's example only wired up 4 of the 14 custom attributes
// and got the 'mode' / 'valve_position' handling wrong relative to what the firmware sends.
// See task report for eng-t11 for the specific deltas.

// OTA (Task 26): `ota: true` below enables Z2M's OTA flow for this device via a *local*
// OTA index (no Espressif/Koenkk cloud index entry needed for this private manufacturer code).
// To publish a new build:
//   1. `cd firmware && idf.py build` (bumps whatever version is in the app).
//   2. Generate the Zigbee OTA image (`.ota`) from `firmware/build/valvecontroller.bin` using the
//      Zigbee OTA image tool (wraps the .bin with the ZCL OTA header: manufacturer code, image
//      type, file version).
//   3. Host the resulting `.ota` file somewhere Z2M's process can read (local path or URL).
//   4. Add an entry to Z2M's local OTA index file (referenced via `ota.zigbeeOTA` config or the
//      Z2M `ota_index.json` mechanism per Z2M docs), one JSON object per firmware version:
//        { "modelId": "ValveCtl-C6", "url": "<path-or-url-to-the-.ota-file>",
//          "fileVersion": <uint32, matches the version baked into the .ota image>,
//          "imageType": 0x0001, "manufacturerCode": 0x1234 }
//   5. Restart Z2M (or trigger a re-read of the index) so "Check for updates" picks it up.
// See Task 27 (MANUAL CHECKPOINT) for the on-device round-trip + rollback verification.

const MFR = 0x1234;                 // must match VALVECTL_MFR_CODE (firmware/main/zigbee.h)
const CLUSTER = 0xFC00;             // VALVECTL_CUSTOM_CLUSTER_ID
const opts = {manufacturerCode: MFR};

// ZCL attribute type codes used below.
const T_BOOL   = 0x10;
const T_U32    = 0x23;
const T_SINGLE = 0x39;

// attr id -> {key, type} exactly matching firmware/main/zigbee.h ATTR_* defines (0x0000-0x000D).
const CUSTOM_ATTRS = {
    0:  {key: 'heat_threshold',      type: T_SINGLE, rw: true},
    1:  {key: 'cool_threshold',      type: T_SINGLE, rw: true},
    2:  {key: 'travel_time_s',       type: T_U32,    rw: true},
    3:  {key: 'park_pos',            type: T_SINGLE, rw: true},
    4:  {key: 'direction_swap',      type: T_BOOL,   rw: true},
    5:  {key: 'kp',                  type: T_SINGLE, rw: true},
    6:  {key: 'ki',                  type: T_SINGLE, rw: true},
    7:  {key: 'gov_high',            type: T_SINGLE, rw: true},
    8:  {key: 'gov_low',             type: T_SINGLE, rw: true},
    9:  {key: 'alarm_dwell',         type: T_U32,    rw: true},
    10: {key: 'resync',              type: T_BOOL,   rw: true},   // self-clearing
    11: {key: 'alarm',               type: T_U32 /* 16bitmap */, rw: false},
    12: {key: 'fault_bitmap',        type: T_U32 /* 16bitmap */, rw: false},
    13: {key: 'travel_since_resync', type: T_SINGLE, rw: false},
};
const ATTR_ID_BY_KEY = Object.fromEntries(
    Object.entries(CUSTOM_ATTRS).map(([id, v]) => [v.key, Number(id)]));

const fzCustom = {
    cluster: CLUSTER, type: ['attributeReport', 'readResponse'],
    convert: (model, msg, publish, options, meta) => {
        const d = {}, a = msg.data;
        for (const [id, def] of Object.entries(CUSTOM_ATTRS)) {
            if (a[id] === undefined) continue;
            if (def.key === 'alarm') { d.alarm = (a[id] !== 0) ? 'ON' : 'OFF'; continue; }
            d[def.key] = a[id];
        }
        return d;
    },
};

const tzTunable = {
    key: Object.values(CUSTOM_ATTRS).filter((d) => d.rw).map((d) => d.key),
    convertSet: async (entity, key, value, meta) => {
        const id = ATTR_ID_BY_KEY[key];
        const type = CUSTOM_ATTRS[id].type;
        await entity.write(CLUSTER, {[id]: {value, type}}, opts);
        return {state: {[key]: value}};
    },
    convertGet: async (entity, key, meta) => {
        await entity.read(CLUSTER, [ATTR_ID_BY_KEY[key]], opts);
    },
};

// Firmware's ThermostatRunningMode (0x001E) values are ZCL-standard but not the same enum/name
// the generic fz.thermostat converter exposes: Off=0x00, Cool=0x03, Heat=0x04
// (see zigbee_push_status() in firmware/main/zigbee.c). Map to our own 'mode' property directly
// instead of relying on the generic converter to happen to produce it.
const fzRunningMode = {
    cluster: 'hvacThermostat', type: ['attributeReport', 'readResponse'],
    convert: (model, msg, publish, options, meta) => {
        if (msg.data.runningMode === undefined) return;
        const map = {0x00: 'idle', 0x03: 'cooling', 0x04: 'heating'};
        return {mode: map[msg.data.runningMode] ?? 'idle'};
    },
};

// Analog Output (valve position) IS writable while water_running is OFF (firmware attr_cb()
// honors the write only in that state and ignores it otherwise; a write supersedes park_pos
// until water_running goes ON or reboot). The plan's snippet exposed this read-only, which
// silently disagreed with the firmware/spec's documented behavior — fixed here.
const fzAnalogOutput = {
    cluster: 'genAnalogOutput', type: ['attributeReport', 'readResponse'],
    convert: (model, msg, publish, options, meta) => {
        if (msg.data.presentValue === undefined) return;
        return {valve_position: msg.data.presentValue};
    },
};
const tzAnalogOutput = {
    key: ['valve_position'],
    convertSet: async (entity, key, value, meta) => {
        await entity.write('genAnalogOutput', {presentValue: value});
        return {state: {valve_position: value}};
    },
    convertGet: async (entity, key, meta) => {
        await entity.read('genAnalogOutput', ['presentValue']);
    },
};

module.exports = [{
    zigbeeModel: ['ValveCtl-C6'],
    model: 'ValveCtl-C6',
    vendor: 'Kleist',
    description: 'Hydronic 3-way mixing valve controller',
    fromZigbee: [fz.on_off, fz.temperature, fz.thermostat, fzRunningMode, fzAnalogOutput, fzCustom],
    toZigbee: [tz.on_off, tz.thermostat_occupied_heating_setpoint,
               tz.thermostat_occupied_cooling_setpoint, tzAnalogOutput, tzTunable],
    exposes: [
        e.binary('water_running', ea.ALL, 'ON', 'OFF').withDescription('Regulation enable'),
        e.numeric('supply_temp', ea.STATE).withUnit('°C').withEndpoint('2'),
        e.numeric('return_temp', ea.STATE).withUnit('°C').withEndpoint('3'),
        e.numeric('source_temp', ea.STATE).withUnit('°C').withEndpoint('4'),
        e.numeric('hx_a_temp', ea.STATE).withUnit('°C').withEndpoint('5'),
        e.numeric('hx_b_temp', ea.STATE).withUnit('°C').withEndpoint('6'),
        e.numeric('valve_position', ea.ALL).withUnit('%').withValueMin(0).withValueMax(100)
            .withDescription('Writable only while water_running is OFF; supersedes park_pos until ON/reboot'),
        e.enum('mode', ea.STATE, ['idle', 'heating', 'cooling']),
        e.numeric('heat_threshold', ea.ALL).withUnit('°C').withValueMin(0).withValueMax(60),
        e.numeric('cool_threshold', ea.ALL).withUnit('°C').withValueMin(0).withValueMax(60),
        e.numeric('travel_time_s', ea.ALL).withUnit('s'),
        e.numeric('park_pos', ea.ALL).withUnit('%').withValueMin(0).withValueMax(100),
        e.binary('direction_swap', ea.ALL, true, false),
        e.numeric('kp', ea.ALL),
        e.numeric('ki', ea.ALL),
        e.numeric('gov_high', ea.ALL).withUnit('°C'),
        e.numeric('gov_low', ea.ALL).withUnit('°C'),
        e.numeric('alarm_dwell', ea.ALL).withUnit('ms'),
        e.binary('alarm', ea.STATE, 'ON', 'OFF'),
        e.numeric('fault_bitmap', ea.STATE),
        e.numeric('travel_since_resync', ea.STATE).withUnit('%'),
        e.binary('resync', ea.SET, true, false).withDescription('Trigger valve resync'),
    ],
    endpoint: (device) => ({'1': 1, '2': 2, '3': 3, '4': 4, '5': 5, '6': 6}),
    meta: {multiEndpoint: true},
    ota: true,
}];
