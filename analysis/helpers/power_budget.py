#!/usr/bin/env python3.12
"""3V3 / 5V rail budget for ValveController.

Source of truth for loads: schematic analyzer JSON (resistor values, rail
membership) + datasheet figures for the active parts. Run from repo root.
"""

# --- supply chain -----------------------------------------------------------
IRM_V, IRM_I = 5.0, 0.600          # IRM-03-5: 5V 600mA 3W
D4_VF = 0.33                       # MBR130LSFT1G @ ~0.5A (datasheet 380mV @ 1A)
RAIL_5V = IRM_V - D4_VF
LDO_IMAX = 1.000                   # TLV75733P, 1A
LDO_VOUT = 3.3
LDO_DROPOUT_1A = 0.20

# --- 24V boost load (TPS61040) ---------------------------------------------
V24 = 24.0
R14 = 2200.0                       # Wilo iPWM2 pull-up
RFB = 249000.0 + 13300.0           # R10 + R11 feedback divider
I24 = V24 / R14 + V24 / RFB
BOOST_EFF = 0.80
I_boost_in = (V24 * I24) / BOOST_EFF / RAIL_5V

# --- 3V3 loads --------------------------------------------------------------
PROBE_ACTIVE = 0.0015              # DS18B20 max during Tconv
N_PROBES = 5
R_1WIRE_PU = 4700.0                # RS4..RS8
R_I2C_PU = 4700.0                  # R18, R19
R_AUX_PU = 100000.0                # R32, R33

i_probes = N_PROBES * PROBE_ACTIVE
i_1wire_pu = N_PROBES * (LDO_VOUT / R_1WIRE_PU)     # worst case: all buses low
i_i2c_pu = 2 * (LDO_VOUT / R_I2C_PU)
i_aux_pu = 2 * (LDO_VOUT / R_AUX_PU)
i_misc_pu = 5 * (LDO_VOUT / 10000.0)                # R2,R3,R7,R13,R16 worst case

PROBE_BLOCK = i_probes + i_1wire_pu

# ESP32-C6-WROOM-1 (datasheet)
ESP = {"sleep": 0.000_02, "cpu_active": 0.045, "boot_peak": 0.300,
       "zigbee_tx": 0.120, "wifi_tx": 0.335}

# IS31FL3730 + 8x8 matrix: multiplexed, 1 row lit at a time.
# Instantaneous = (lit LEDs in active row) x I_segment. I_segment is I2C
# programmable; full scale default is 40mA.
def display(i_seg, lit_per_row=8):
    return i_seg * lit_per_row

SCENARIOS = [
    ("Boot / reset peak", ESP["boot_peak"], display(0.0), 0.0),
    ("Idle, display off", ESP["cpu_active"], display(0.0), PROBE_BLOCK),
    ("Zigbee TX + display @ 5mA/seg", ESP["zigbee_tx"], display(0.005), PROBE_BLOCK),
    ("Zigbee TX + display @ 20mA/seg", ESP["zigbee_tx"], display(0.020), PROBE_BLOCK),
    ("Zigbee TX + display FULL 40mA/seg", ESP["zigbee_tx"], display(0.040), PROBE_BLOCK),
    ("WiFi TX + display FULL 40mA/seg", ESP["wifi_tx"], display(0.040), PROBE_BLOCK),
]

print(f"5V rail after D4 = {RAIL_5V:.2f} V   |   24V boost draws {I_boost_in*1000:.0f} mA from 5V")
print(f"Probe block (5x DS18B20 + 5x 4k7 pull-ups) = {PROBE_BLOCK*1000:.1f} mA "
      f"({i_probes*1000:.1f} mA silicon + {i_1wire_pu*1000:.1f} mA pull-ups)")
print(f"Other static pull-ups (I2C + AUX + straps) = {(i_i2c_pu+i_aux_pu+i_misc_pu)*1000:.1f} mA\n")

hdr = f"{'scenario':38s} {'3V3 mA':>8s} {'LDO %':>7s} {'5V mA':>8s} {'IRM %':>7s} {'LDO W':>7s}"
print(hdr); print("-" * len(hdr))
for name, esp, disp, probes in SCENARIOS:
    i33 = esp + disp + probes + i_i2c_pu + i_aux_pu + i_misc_pu
    i5 = i33 + I_boost_in                       # LDO passes output current 1:1
    ldo_w = (RAIL_5V - LDO_VOUT) * i33
    flag = "  <-- OVER" if i5 > IRM_I else ("  <-- OVER LDO" if i33 > LDO_IMAX else "")
    print(f"{name:38s} {i33*1000:8.0f} {i33/LDO_IMAX*100:6.0f}% {i5*1000:8.0f} "
          f"{i5/IRM_I*100:6.0f}% {ldo_w:7.2f}{flag}")

print(f"\nLDO thermal: WSON-6 2x2mm with thermal pad+vias, theta_JA ~ 55 C/W")
for w in (0.3, 0.5, 0.7):
    print(f"  P={w:.1f} W -> dT = {w*55:.0f} C  (Tj at 40 C ambient = {40+w*55:.0f} C, "
          f"at 60 C ambient = {60+w*55:.0f} C)")
