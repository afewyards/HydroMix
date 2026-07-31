import * as fz from 'zigbee-herdsman-converters/converters/fromZigbee';
import * as exposes from 'zigbee-herdsman-converters/lib/exposes';
import * as reporting from 'zigbee-herdsman-converters/lib/reporting';
import * as globalStore from 'zigbee-herdsman-converters/lib/store';
import {logger} from 'zigbee-herdsman-converters/lib/logger';

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
//        { "modelId": "HydroMix", "url": "<path-or-url-to-the-.ota-file>",
//          "fileVersion": <uint32, matches the version baked into the .ota image>,
//          "imageType": 0x0001, "manufacturerCode": 0x1234 }
//   5. Restart Z2M (or trigger a re-read of the index) so "Check for updates" picks it up.
// See Task 27 (MANUAL CHECKPOINT) for the on-device round-trip + rollback verification.

const CLUSTER = 0xFC00;             // VALVECTL_CUSTOM_CLUSTER_ID
// No manufacturerCode on 0xFC00 reads/writes. The firmware registers these as plain
// attributes via esp_zb_custom_cluster_add_custom_attr() (0xFC00 is already a private
// cluster id, so the attributes inside don't need their own code) — sending 0x1234 here
// matched no registered attribute and every read/write came back UNSUPPORTED_ATTRIBUTE.
// Keep in lockstep with build_custom_cluster() in firmware/main/zigbee.c.
const opts = {};

// ZCL attribute type codes used below.
const T_BOOL   = 0x10;
const T_U32    = 0x23;
const T_SINGLE = 0x39;

// attr id -> {key, type} exactly matching firmware/main/zigbee.h ATTR_* defines (0x0000-0x0012).
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
    14: {key: 'deadtime_s',          type: T_SINGLE, rw: true},
    15: {key: 'pi_deadband_k',       type: T_SINGLE, rw: true},
    16: {key: 'heat_setpoint',       type: T_SINGLE, rw: true},
    17: {key: 'cool_setpoint',       type: T_SINGLE, rw: true},
    18: {key: 'valve_deadband_pct',  type: T_SINGLE, rw: true},   // firmware 1.4.0+, see expose below
};
const ATTR_ID_BY_KEY = Object.fromEntries(
    Object.entries(CUSTOM_ATTRS).map(([id, v]) => [v.key, Number(id)]));

// fromZigbee matching is a strict === against the cluster name zigbee-herdsman reports.
// 0xFC00 isn't a known cluster, so herdsman names it by its decimal id as a STRING —
// matching on the number 0xFC00 silently matched nothing ("No converter available for
// ... cluster '64512'") and every attribute the device correctly returned was dropped on
// the floor. Reads/writes below still take the numeric id; only this matcher is a string.
const CLUSTER_FZ = String(CLUSTER);   // '64512'

const fzCustom = {
    cluster: CLUSTER_FZ, type: ['attributeReport', 'readResponse'],
    convert: (model, msg, publish, options, meta) => {
        const d = {}, a = msg.data;
        for (const [id, def] of Object.entries(CUSTOM_ATTRS)) {
            if (a[id] === undefined) continue;
            if (def.key === 'alarm') { d.alarm = (a[id] !== 0) ? 'ON' : 'OFF'; continue; }
            // Same float32 rounding as valve_position for the SINGLE-typed tunables.
            d[def.key] = (def.type === T_SINGLE) ? Math.round(a[id] * 100) / 100 : a[id];
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

// runningMode had no reporting configured on the firmware side, historically: hvacThermostat
// was never bound (see the comment above configure() below — live binding-table inspection
// showed EP1 bound for genOnOff + genAnalogOutput only, nothing for hvacThermostat), so
// RunningMode reports had nowhere to go and configure()'s one-time read was the only thing
// that ever populated 'mode'. That left the HA sensor 6 h stale on the live device while
// everything else updated every minute. Binding hvacThermostat in configure() is the actual
// root-cause fix; this poll is a belt-and-suspenders refresh on top of it — a plain ZCL read
// does NOT touch the firmware's link-activity tracking inside the ZBOSS stack, so it is NOT a
// device-side keepalive, only a periodic refresh of Z2M's own copy of 'mode'. Firmware 1.4.0
// additionally pushes runningMode on change once it arrives over a real binding, making this
// poll a fallback for missed pushes and for pre-1.4.0 devices that never push at all.
//
// Shaped like ZHC's built-in `poll()` modern-extend helper (src/lib/modernExtend.ts) rather
// than a hand-rolled setInterval: ZHC 25+ (we're on 26.76.0) passes onEvent a SINGLE
// discriminated-union event object, not the old `(type, data, device)` triple — the 'stop'
// event's `data` carries ONLY `ieeeAddr`, no `.device`, so teardown must key off that. ZHC
// also synthesizes a 'start' event on (re)pairing and maps device removal to 'stop', so
// pairing-after-boot and removal teardown come for free. State lives in ZHC's globalStore
// (keyed by ieeeAddr) rather than a module-level Map so it can't be orphaned by a module
// re-import.
const LOG_NS = 'zhc:hydromix';
const RUNNING_MODE_POLL_KEY = 'runningModePoll';
const RUNNING_MODE_POLL_MS = 5 * 60 * 1000;

// Analog Output (valve position) IS writable while water_running is OFF (firmware attr_cb()
// honors the write only in that state and ignores it otherwise; a write supersedes park_pos
// until water_running goes ON or reboot). The plan's snippet exposed this read-only, which
// silently disagreed with the firmware/spec's documented behavior — fixed here.
const fzAnalogOutput = {
    cluster: 'genAnalogOutput', type: ['attributeReport', 'readResponse'],
    convert: (model, msg, publish, options, meta) => {
        if (msg.data.presentValue === undefined) return;
        // float32 straight off the wire is e.g. 48.08318328857422 — 0.1 % is well past
        // anything the valve can actually resolve.
        return {valve_position: Math.round(msg.data.presentValue * 10) / 10};
    },
};
// water_running is the On/Off cluster on EP1. Same trap as the temps: fz.on_off/tz.on_off
// key on `state` (postfixed to `state_1` here by multiEndpoint), which never matches the
// `water_running` expose — it read Null and writes silently went nowhere. Hand-rolled both
// directions so the exposed property and the converter key agree, matching the
// fzAnalogOutput/tzAnalogOutput pattern above.
// The generic fz.temperature has no guard for the ZCL invalid sentinel and would
// publish -327.68 °C as a real reading. The firmware sends 0x8000 (-32768) for any
// probe whose fault is latched (temp_centi() in firmware/main/zigbee.c).
const fzTemperature = {
    cluster: 'msTemperatureMeasurement', type: ['attributeReport', 'readResponse'],
    convert: (model, msg, publish, options, meta) => {
        if (msg.data.measuredValue === undefined) return;
        // multiEndpoint postfix built explicitly rather than via postfixWithEndpointName(),
        // whose signature has changed across zigbee-herdsman-converters versions. The
        // endpoint map below names endpoints '2'..'6' identically to their IDs, so this
        // produces exactly the `temperature_<ep>` properties the exposes declare.
        const property = `temperature_${msg.endpoint.ID}`;
        if (msg.data.measuredValue === -32768) return {[property]: null};
        return {[property]: Math.round(msg.data.measuredValue) / 100};
    },
};
const fzWaterRunning = {
    cluster: 'genOnOff', type: ['attributeReport', 'readResponse'],
    convert: (model, msg, publish, options, meta) => {
        if (msg.data.onOff === undefined) return;
        return {water_running: msg.data.onOff ? 'ON' : 'OFF'};
    },
};
const tzWaterRunning = {
    key: ['water_running'],
    convertSet: async (entity, key, value, meta) => {
        const on = (value === 'ON' || value === true);
        await entity.command('genOnOff', on ? 'on' : 'off', {});
        return {state: {water_running: on ? 'ON' : 'OFF'}};
    },
    convertGet: async (entity, key, meta) => {
        await entity.read('genOnOff', ['onOff']);
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

export default [{
    zigbeeModel: ['HydroMix'],
    model: 'HydroMix',
    vendor: 'Knife',
    description: 'Hydronic 3-way mixing valve controller',
    fromZigbee: [fzWaterRunning, fzTemperature, fz.thermostat, fzRunningMode, fzAnalogOutput, fzCustom],
    // tz.thermostat_occupied_heating_setpoint / tz.thermostat_occupied_cooling_setpoint were
    // removed here: ZBOSS enforces heat<=cool-deadband on hvacThermostat, and this device's
    // independent seasonal targets (heat 35 / cool 18) always violate that, so those writes
    // were rejected INVALID_VALUE before firmware ever saw them (verified live). The regulation
    // targets are now heat_setpoint/cool_setpoint below, via tzTunable on the custom cluster.
    toZigbee: [tzWaterRunning, tzAnalogOutput, tzTunable],
    exposes: [
        e.binary('water_running', ea.ALL, 'ON', 'OFF').withDescription('Regulation enable'),
        // withProperty('temperature') is load-bearing: fz.temperature publishes under
        // `temperature`, postfixed to `temperature_<ep>` by multiEndpoint. Without it the
        // property defaults to the expose *name* (`supply_temp_2`), which nothing ever
        // writes — the entity then renders Null forever while the real value sits in
        // `temperature_2`. withProperty must come BEFORE withEndpoint (withEndpoint
        // appends the suffix to whatever property is set at that point).
        e.numeric('supply_temp', ea.STATE).withUnit('°C').withProperty('temperature').withEndpoint('2'),
        e.numeric('return_temp', ea.STATE).withUnit('°C').withProperty('temperature').withEndpoint('3'),
        e.numeric('source_temp', ea.STATE).withUnit('°C').withProperty('temperature').withEndpoint('4'),
        e.numeric('hx_a_temp', ea.STATE).withUnit('°C').withProperty('temperature').withEndpoint('5'),
        e.numeric('hx_b_temp', ea.STATE).withUnit('°C').withProperty('temperature').withEndpoint('6'),
        e.numeric('valve_position', ea.ALL).withUnit('%').withValueMin(0).withValueMax(100)
            .withDescription('Writable only while water_running is OFF; supersedes park_pos until ON/reboot'),
        e.enum('mode', ea.STATE, ['idle', 'heating', 'cooling']),
        // Regulation targets, on the custom cluster (attrs 0x0010/0x0011) — NOT the standard
        // hvacThermostat OccupiedHeating/CoolingSetpoint. ZBOSS enforces heat<=cool-deadband
        // on that cluster; this device keeps independent seasonal targets (heat 35 / cool 18)
        // that always violate it, so every ZCL write there was rejected INVALID_VALUE before
        // firmware ever saw it (verified live). No withEndpoint() here, matching kp/ki and
        // the other custom-tunable exposes below (they're plain attrs on EP1's custom cluster,
        // not postfixed like the generic fz.thermostat-published properties).
        e.numeric('heat_setpoint', ea.ALL).withUnit('°C')
            .withValueMin(17).withValueMax(35).withValueStep(0.5)
            .withDescription('Heating regulation target. Out-of-range writes are clamped by the device (17-35 °C) and echoed back.'),
        e.numeric('cool_setpoint', ea.ALL).withUnit('°C')
            .withValueMin(17).withValueMax(35).withValueStep(0.5)
            .withDescription('Cooling regulation target. Out-of-range writes are clamped by the device (17-35 °C) and echoed back.'),
        e.numeric('heat_threshold', ea.ALL).withUnit('°C').withValueMin(10).withValueMax(60),
        e.numeric('cool_threshold', ea.ALL).withUnit('°C').withValueMin(0).withValueMax(40),
        e.numeric('travel_time_s', ea.ALL).withUnit('s').withValueMin(30).withValueMax(600),
        e.numeric('park_pos', ea.ALL).withUnit('%').withValueMin(0).withValueMax(100),
        e.binary('direction_swap', ea.ALL, true, false),
        e.numeric('kp', ea.ALL).withValueMin(0.5).withValueMax(15),
        e.numeric('ki', ea.ALL).withValueMin(0).withValueMax(5)
            .withDescription('Integral gain, %/K per minute (1.1.0+; was per 10 s cycle). '
            + 'Out-of-range writes are clamped by the device (kp 0.5-15, ki 0-5) and echoed back.'),
        // release band is 35/17 (control_task.c) — bounds keep trip thresholds outside it
        // or the governor limit-cycles; must match config.c/config_map.c clamp_config/tunable_apply.
        e.numeric('gov_high', ea.ALL).withUnit('°C').withValueMin(36).withValueMax(60),
        e.numeric('gov_low', ea.ALL).withUnit('°C').withValueMin(0).withValueMax(16),
        e.numeric('alarm_dwell', ea.ALL).withUnit('ms').withValueMin(10000).withValueMax(3600000),
        e.numeric('deadtime_s', ea.ALL).withUnit('s').withValueMin(0).withValueMax(120)
            .withDescription('Transit hold: PI pauses this long after valve movement'),
        // Without withValueStep, HA defaults the number entity's step to 1.0 — over a
        // 0-1 range that only lets the UI produce 0 or 1, never anything in between.
        e.numeric('pi_deadband_k', ea.ALL).withUnit('K').withValueMin(0).withValueMax(1)
            .withValueStep(0.05)
            .withDescription('PI error deadband (gap form)'),
        // attr 0x0012, firmware 1.4.0+. On older firmware the configure() read of this attr
        // comes back UNSUPPORTED_ATTRIBUTE, which the isolated tryRead loop below already
        // tolerates per-attribute — the entity just stays null instead of blanking the rest.
        e.numeric('valve_deadband_pct', ea.ALL).withUnit('%').withValueMin(0.2).withValueMax(5)
            .withValueStep(0.1)
            .withDescription('Motor deadband: the valve only drives when |target − position| exceeds '
            + 'this percent of travel, and stops at the band edge. Smaller = tighter supply tracking '
            + 'but more actuator movement. Requires firmware 1.4.0+. Device clamps 0.2-5 %, and also '
            + 'enforces a travel-time-derived floor of 1.2×100/travel_time_s % (1.0 % at the default '
            + '120 s travel) — writes below that floor are clamped up and echoed back. Min stays 0.2 '
            + 'here since the floor is legal at longer travel times.'),
        e.binary('alarm', ea.STATE, 'ON', 'OFF'),
        e.numeric('fault_bitmap', ea.STATE),
        e.numeric('travel_since_resync', ea.STATE).withUnit('%'),
        e.binary('resync', ea.SET, true, false).withDescription('Trigger valve resync'),
    ],
    // The tunables are plain read/write attributes: the firmware only sets up *reporting*
    // for temps, valve position and the alarm/fault bitmaps, so nothing ever pushes the
    // rest and they stay null until something reads them. Seed them once at configure.
    // No reporting.bind() for the clusters that already work (genOnOff/genAnalogOutput/temp/
    // custom) — the firmware configures its own reporting locally
    // (esp_zb_zcl_update_reporting_info) and reports already arrive, so re-binding them would
    // only risk disturbing a working live device. hvacThermostat is the one exception: live
    // binding-table inspection showed EP1 bound for genOnOff + genAnalogOutput only (EP2-6
    // bound for temperature) — hvacThermostat was never bound at all, so RunningMode reports
    // (both the firmware's passive reporting engine and its 1.4.0+ explicit push) have nowhere
    // to go. That's the actual root cause of 'mode' going stale for hours. Bound additively
    // below, alongside the existing seed reads — this doesn't touch any cluster that already
    // works.
    configure: async (device, coordinatorEndpoint, definition) => {
        const ep1 = device.getEndpoint(1);
        // Same isolation as tryRead below: a failed bind (already bound, coordinator busy,
        // etc.) shouldn't block the seed reads that follow.
        try {
            await reporting.bind(ep1, coordinatorEndpoint, ['hvacThermostat']);
        } catch (e) {
            // logger.warning, not console.warn: console.warn bypasses Z2M's log-level
            // filtering, file rotation, and the bridge/logging MQTT topic entirely.
            logger.warning(`HydroMix: bind hvacThermostat failed: ${e.message}`, LOG_NS);
        }
        // Each read is isolated: an UNSUPPORTED_ATTRIBUTE on one cluster used to reject the
        // whole promise chain, so a single bad attribute left every later read unsent and
        // every remaining property null — hiding which one actually failed.
        const tryRead = async (cluster, attrs, options) => {
            try {
                await ep1.read(cluster, attrs, options);
            } catch (e) {
                logger.warning(`HydroMix: read ${cluster} [${attrs}] failed: ${e.message}`, LOG_NS);
            }
        };
        await tryRead('genOnOff', ['onOff']);
        await tryRead('genAnalogOutput', ['presentValue']);
        await tryRead('hvacThermostat', ['runningMode', 'localTemp']);
        // One attribute per Read Attributes request, deliberately. A batched read rejects
        // as a whole if any single attribute in it errors, so one bad id silently blanked
        // every other tunable in the same batch — single reads of the same ids work fine.
        for (const id of Object.keys(CUSTOM_ATTRS).map(Number)) {
            await tryRead(CLUSTER, [id], opts);
        }
    },
    endpoint: (device) => ({'1': 1, '2': 2, '3': 3, '4': 4, '5': 5, '6': 6}),
    meta: {multiEndpoint: true},
    ota: true,
    // See the runningMode comment above fzRunningMode for why this polls at all, and why it's
    // shaped like ZHC's poll() helper instead of a plain (type, data, device) setInterval.
    onEvent: async (event) => {
        if (event.type === 'stop') {
            // 'stop' data carries ONLY ieeeAddr — no .device — per ZHC 25+'s OnEvent contract.
            clearTimeout(globalStore.getValue(event.data.ieeeAddr, RUNNING_MODE_POLL_KEY));
            globalStore.clearValue(event.data.ieeeAddr, RUNNING_MODE_POLL_KEY);
            return;
        }
        const device = event.data.device;
        if (!device) return;
        if (event.data.options?.disabled) return;   // don't poll a disabled device
        // Start-if-absent rather than clear-and-restart: only the first event that carries a
        // device should kick this off, so a later event of some other type doesn't spawn a
        // second overlapping poll chain for the same device.
        if (globalStore.hasValue(device.ieeeAddr, RUNNING_MODE_POLL_KEY)) return;
        const scheduleNext = () => {
            const timer = setTimeout(async () => {
                // Resolved fresh every tick, not captured once at chain start: matches ZHC's
                // own poll() pattern, and means a device/endpoint that goes away mid-chain
                // just skips a tick instead of throwing the same swallowed TypeError forever.
                const ep = device.getEndpoint(1);
                if (ep) {
                    try {
                        await ep.read('hvacThermostat', ['runningMode']);
                    } catch (e) {
                        // debug, not warn: an offline device fails this every 5 min (288/day)
                        // — expected, not something worth spamming the log about.
                        logger.debug(`HydroMix: runningMode poll failed: ${e.message}`, LOG_NS);
                    }
                }
                // Keep the chain going only while we're still the active timer for this
                // device — a 'stop' event clears the stored value, which ends it here rather
                // than continuing to poll a device that's gone.
                if (globalStore.getValue(device.ieeeAddr, RUNNING_MODE_POLL_KEY) === timer) scheduleNext();
            }, RUNNING_MODE_POLL_MS);
            timer.unref?.();   // never let this be the reason the process stays alive
            globalStore.putValue(device.ieeeAddr, RUNNING_MODE_POLL_KEY, timer);
        };
        scheduleNext();
    },
}];
