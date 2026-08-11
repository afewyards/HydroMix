#!/usr/bin/env python3
"""Numeric checks for the Wilo Para iPWM2 pump interface (U6/Q5/J14 blocks).

Datasheet anchors:
  TPS61040 SLVS413K: Eq.5 Vout=1.233*(1+R1/R2); VFB 1.208/1.233/1.258 V;
                     IFB max 1 uA; ILIM 350/400/450 mA; toff_min 250/400/550 ns;
                     ton_max 4/6/7.5 us; VIN 1.8-6 V; SW abs max 30 V; VOUT max 28 V.
  CSD17313Q2 SLPS260E: BVDSS 30 V; VGS(th) 0.9/1.3/1.8 V; RDS(on) 31 mO @VGS=3 V;
                     Coss 140/180 pF @15 V; Qoss 3.8 nC @13.5 V.
  ERJ-P08 (Panasonic AOA0000C331): 0.66 W @70 C, derate 70->155 C, RCWV limit.
  ESP32-C6 VIL_max = 0.25*VDD, VIH_min = 0.75*VDD.
"""

R10, R11 = 1e6, 53.6e3          # boost FB divider (top, bottom)
R12, R13 = 100.0, 10e3          # Q5 gate series, gate pull-up to 3V3
R14, R15 = 2.2e3, 100.0         # 24V pull-up, series to J14.1
R16, R17 = 5.6e3, 100.0         # FB pull-up to 3V3, series from J14.3
C12, C13 = 10e-9, 22e-12
L1, COUT_NOM = 10e-6, 4.4e-6
VREF_T, VREF_MIN, VREF_MAX = 1.233, 1.208, 1.258
IFB_MAX = 1e-6
VIN = 5.0
V3V3 = 3.3
PUMP_TH, PUMP_R = 7.0, 3.15e3   # pump iPWM input Thevenin model
PUMP_FB_R = 470.0               # pump feedback open-collector series R
ILIM = 0.400

out = []
def p(s): out.append(s)

# ---- a) 24 V rail envelope -------------------------------------------------
vnom = VREF_T * (1 + R10 / R11)
# worst-case high: VFB max, R1 +1%, R2 -1%, IFB max adds to R1 current
r1h, r2l = R10 * 1.01, R11 * 0.99
i_r2 = VREF_MAX / r2l
vmax = VREF_MAX + (i_r2 + IFB_MAX) * r1h
# worst-case low: VFB min, R1 -1%, R2 +1%, IFB = 0
r1l, r2h = R10 * 0.99, R11 * 1.01
vmin = VREF_MIN + (VREF_MIN / r2h) * r1l
p(f"[a] 24V rail: nom {vnom:.2f} V | worst-case {vmin:.2f} .. {vmax:.2f} V")
p(f"    IFB(1uA max) x R10(1M) alone contributes +{IFB_MAX*R10:.2f} V of the spread")
p(f"    Q5 BVDSS 30 V -> margin at nom {100*(30-vnom)/30:.0f}%, at wc-max {100*(30-vmax)/30:.0f}%")
p(f"    U6 SW abs max 30 V, SW sees Vout+Vf(D6~0.4) = {vmax+0.4:.2f} V at wc-max")
p(f"    U6 rec. max VOUT 28 V -> wc-max {vmax:.2f} V is {'WITHIN' if vmax<28 else 'OVER'}")

# ---- line levels -----------------------------------------------------------
# Q5 OFF: R14 from 24V into (R15 + pump 3.15k to 7 V)
rl = R15 + PUMP_R
vline_hi = (vnom / R14 + PUMP_TH / rl) / (1 / R14 + 1 / rl)
i_pump_hi = (vline_hi - PUMP_TH) / rl
p("")
p(f"[line] Q5 OFF: V(PUMP_PWM_LINE)={vline_hi:.2f} V, pump sinks {i_pump_hi*1e3:.2f} mA")
p(f"       V at J14.1 = {vline_hi - i_pump_hi*R15:.2f} V")
# Q5 ON: R14 fully across the rail
i_r14 = vnom / R14
p_r14 = vnom**2 / R14
p(f"       Q5 ON: I(R14)={i_r14*1e3:.2f} mA, drain current adds pump {PUMP_TH/rl*1e3:.2f} mA")
p(f"       Q5 Vds(on) = {(i_r14+PUMP_TH/rl)*0.031*1e3:.3f} mV (RDS(on) 31 mO @VGS=3 V)")

# ---- b) R14 dissipation ----------------------------------------------------
p("")
p(f"[b] R14 P = {p_r14*1e3:.1f} mW continuous when pump commanded STOP (0% duty)")
p(f"    ERJ-P08 1206 rated 0.66 W @70 C -> load factor {100*p_r14/0.66:.0f}%")
p(f"    derating starts at 70 C, so at 40 C cabinet ambient full 0.66 W is available")
p(f"    at wc-max rail {vmax:.2f} V: P = {vmax**2/R14*1e3:.1f} mW ({100*(vmax**2/R14)/0.66:.0f}% of rating)")
rcwv = (0.66 * R14) ** 0.5
p(f"    RCWV = sqrt(0.66*2200) = {rcwv:.1f} V > {vmax:.1f} V applied  -> OK")
p(f"    24V rail draw when stopped: {i_r14*1e3:.1f} mA -> {p_r14*1e3:.0f} mW out;")
p(f"      at ~80% boost eff that is {p_r14/0.8/VIN*1e3:.0f} mA on the 5 V rail, continuous")

# ---- c) boost operating point ---------------------------------------------
ton = L1 * ILIM / VIN
toff_demag = L1 * ILIM / (vnom - VIN)
q_cycle = 0.5 * ILIM * toff_demag
p("")
p(f"[c] ton = {ton*1e6:.2f} us (ton_max 6 us OK); toff_demag = {toff_demag*1e9:.0f} ns")
p(f"    toff_min spec 250-550 ns -> demag {toff_demag*1e9:.0f} ns is BELOW toff_min,")
p(f"    so off-time is clamped; fSmax = 1/(ton+400ns) = {1/(ton+400e-9)/1e3:.0f} kHz (< 1 MHz OK)")
p(f"    charge/pulse = {q_cycle*1e9:.1f} nC")
for iload, label in ((11.05e-3, "Q5 ON  (pump stopped)"), (3.20e-3, "Q5 OFF (pump max)")):
    fs = iload / q_cycle
    cout_eff = COUT_NOM * 0.80      # X7R 1206 50V at 24 V DC bias, ~80% retention
    dv = q_cycle / cout_eff
    zcff = 1 / (2 * 3.14159 * fs * C13)
    top = 1 / (1 / R10 + 1 / zcff)
    ratio = R11 / (R11 + top)
    cff_reco = 1 / (2 * 3.14159 * (fs / 20) * R10)
    p(f"    {label}: Iload={iload*1e3:5.2f} mA fS={fs/1e3:6.1f} kHz "
      f"Vripple={dv*1e3:5.1f} mV  FBripple={dv*ratio*1e3:4.1f} mV  Cff_reco(Eq6)={cff_reco*1e12:.0f} pF")
p(f"    TI guidance: ~50 mV p-p at FB for good line reg -> actual FB ripple is far below")
p(f"    C13=22 pF sits between the two Eq.6 results (12 pF @264 kHz, 41 pF @77 kHz)")

# ---- d) feedback low level -------------------------------------------------
p("")
p("[d] PUMP_FB low level at IO21 (R16 5.6k pull-up, R17 100, pump 470 + Vsat):")
for vsat, tag in ((0.20, "typical Vce(sat)"), (0.53, "worst case from Wilo UoL<=1V @1mA")):
    i = (V3V3 - vsat) / (R16 + R17 + PUMP_FB_R)
    vnode = V3V3 - i * R16
    vterm = vsat + i * PUMP_FB_R
    p(f"    Vsat={vsat:.2f} V -> I={i*1e3:.3f} mA, V(IO21)={vnode:.3f} V, V(J14.3)={vterm:.3f} V")
    p(f"      vs ESP32-C6 VIL_max=0.25*3.3={0.25*V3V3:.3f} V -> margin {(0.25*V3V3-vnode)*1e3:+.0f} mV"
      f" | Wilo budget: I {'OK' if i<=1e-3 else 'OVER'} (<=1 mA), UoL {'OK' if vterm<=1.0 else 'OVER'} (<=1 V)")
# high level
p(f"    high level = 3.3 V (pump OC off) -> VIH_min 0.75*3.3={0.75*V3V3:.3f} V, margin +{(V3V3-0.75*V3V3)*1e3:.0f} mV")

# ---- f) edge timing --------------------------------------------------------
p("")
coss_eff = 3.8e-9 / 13.5        # Qoss/V time-related effective Coss
for ccable, tag in ((300e-12, "3 m spec cable"), (500e-12, "5 m / shielded")):
    ctot = ccable + coss_eff + 30e-12
    reff = 1 / (1 / R14 + 1 / rl)
    tau = reff * ctot
    tr = 2.2 * tau
    p(f"[f] {tag}: Ctot={ctot*1e12:.0f} pF (cable {ccable*1e12:.0f} + Coss_eff {coss_eff*1e12:.0f} + stray 30)")
    p(f"    Reff = R14||(R15+Rpump) = {reff:.0f} O, tau={tau*1e9:.0f} ns, tr(10-90%)={tr*1e6:.2f} us")
    for f_pwm in (1000, 500, 200):
        lim = (1 / f_pwm) / 500
        p(f"      @{f_pwm} Hz: limit T/500 = {lim*1e6:.1f} us -> {'PASS' if tr < lim else 'FAIL'}"
          f" (margin {100*(lim-tr)/lim:+.0f}%)")
tau_f = R15 * 300e-12
p(f"    fall: Q5 on, cable discharges via R15 100 O -> tau={tau_f*1e9:.0f} ns, tf={2.2*tau_f*1e9:.0f} ns  PASS")

# ---- feedback edge timing (75 Hz) -----------------------------------------
p("")
tau_r = R16 * C12
rpar = 1 / (1 / R16 + 1 / (R17 + PUMP_FB_R))
tau_fb_f = rpar * C12
p(f"[fb-edges] 75 Hz, T=13.33 ms. rise tau=R16*C12={tau_r*1e6:.0f} us, "
  f"fall tau=(R16||(R17+470))*C12={tau_fb_f*1e6:.1f} us")
p(f"    R16/C12 corner = {1/(2*3.14159*R16*C12):.0f} Hz vs 75 Hz signal -> {1/(2*3.14159*R16*C12)/75:.0f}x headroom")
import math
t_rise_vih = -tau_r * math.log(1 - 0.75)
vfinal = V3V3 - (V3V3 - 0.20) / (R16 + R17 + PUMP_FB_R) * R16
t_fall_vil = -tau_fb_f * math.log((0.25 * V3V3 - vfinal) / (V3V3 - vfinal))
p(f"    t(rise->VIH)={t_rise_vih*1e6:.0f} us, t(fall->VIL)={t_fall_vil*1e6:.1f} us, "
  f"skew={((t_rise_vih-t_fall_vil)/13.33e-3)*100:.2f}% duty error")

# ---- gate drive / failsafe -------------------------------------------------
p("")
p("[failsafe] gate node PUMP_PWM_G:")
p(f"    GPIO hi-Z, no internal pull: Vg = 3.3 V (R13) -> Q5 ON -> line LOW -> 0% -> STOP")
vg_pd = V3V3 * 45e3 / (45e3 + R13)
p(f"    GPIO hi-Z + ESP internal 45k pull-DOWN worst case: Vg = {vg_pd:.2f} V "
  f"> VGS(th) max 1.8 V -> still ON -> STOP")
p(f"    GPIO driven LOW: Vg = 0 V through R12 100 O -> Q5 OFF -> line HIGH -> 100% -> MAX SPEED")
p(f"    R12 100 O + Ciss 340 pF max -> gate tau = {100*340e-12*1e9:.1f} ns (negligible vs edges)")
p(f"    R13 10k + Ciss 340 pF -> turn-ON tau = {10e3*340e-12*1e6:.2f} us when GPIO releases")

# ---- unpowered board -------------------------------------------------------
p("")
rtot = PUMP_R + R15
p("[e] board unpowered, pump powered:")
p("    3V3=0 -> Q5 gate 0 V -> Q5 OFF.  24V node floats: only load is R10+R11 = 1.05 MO")
vfloat = PUMP_TH * (R14 + R10 + R11) / (rtot + R14 + R10 + R11)
p(f"    C10/C11 charge from the pump's 7 V/3.15k through R15+R14 -> line settles ~{vfloat:.2f} V")
p(f"    = static HIGH at the pump input -> iPWM2 reads ~100% duty -> MAX SPEED (not STOP)")
p(f"    RC to get there: (R15+R14+3.15k)*4.4uF = {(rtot+R14)*4.4e-6*1e3:.0f} ms")

print("\n".join(out))
