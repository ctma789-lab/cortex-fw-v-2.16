# Dual-Cortex Obstacle Avoidance System — Operating Manual

**Comm Cortex v2.1.6 / Sensory Cortex v2.0.4**
Covers channel map, ceiling/floor capture, horizontal avoidance sensitivity, BLE calibration, and every runtime-tunable feature.

---

## 1. System Overview

Two ESP32-S3 boards sit between your FS-X6B receiver and the Naza M V2 flight controller:

- **Comm Cortex** — reads iBUS from the receiver, applies avoidance logic to the channels, outputs PPM to the Naza, and talks to the Sensory Cortex over UART.
- **Sensory Cortex** — reads the 5× TF-Luna, optical flow, IMU, compass, and barometer, fuses them into a `SafetyPacket` (34 bytes, sent ~180–200 Hz) and a live obstacle map (`MapPacket`, ~9 Hz), and sends both to the Comm Cortex.

A hardware watchdog (CD4053 mux + RC filter) bypasses both boards and routes raw PPM straight to the Naza if the Comm Cortex ever crashes or hangs — the avoidance system can only ever add authority on top of your sticks, never lock you out.

---

## 2. Channel Map (default)

| Ch | Function | Behavior |
|----|----------|----------|
| 1 | Roll | Avoidance-modified |
| 2 | Pitch | Avoidance-modified |
| 3 | Throttle | Floor/ceiling-modified |
| 4 | Yaw | Pass-through |
| 5 | Flight Mode | RTH/failsafe detect (PPM idx4; SBUS Naza would be idx6/ch7 — confirm `CH_FLIGHT_MODE` matches your TX) |
| 6 | IOC | Pass-through |
| 7 | Aux/Gimbal | Pass-through |
| 8 | CD4053 bypass | Hardware only — ESP ignores |
| 9 | Proximity VRB | Avoidance zone scale **and** floor-height set (see §4, §5) |
| 10 | Floodlight | GPIO output, on above 1400µs |
| 11 | VTX on/off | GPIO output, on above 1400µs |
| 12 | Nav lights | On above 1400µs |
| 13 | Landing mode | Above 1400µs = landing mode active |
| 14 | Ceiling switch | Above 1400µs = ceiling capture/enforce armed |

RTH/failsafe: Flight Mode channel below **1200µs** = RTH active.

---

## 3. Verified Logic — Audit Summary

I traced all three systems below through the actual code paths. Findings:

| Feature | Status |
|---|---|
| Ceiling capture | ✅ Correct — edge-triggered capture, level-flight gated |
| Floor set/capture | ✅ Correct, but note it's **knob-set**, not altitude-captured — see §4 |
| Horizontal avoidance sensitivity | ✅ Correct — `scale` propagates via `ProximityCmd` to the Sensory Cortex's block thresholds, not locally in the TTE ramp (intentional split, not a bug) |
| Optical-flow BLE calibration | ✅ **Fixed** — see §8.2 for the corrected workflow |
| Version string at boot | ⚠️ Cosmetic only, reported previously (line 1890 says v2.1.4, should read v2.1.6) |

---

## 4. Floor (Altitude Corridor — Lower Bound)

**This is a dialed-in floor, not a "capture current altitude" button** — worth understanding since it's easy to expect it to behave like the ceiling.

**How to set it:**
1. Enter **landing mode** (ch13 > 1400µs).
2. While in landing mode, dial **ch9 (Proximity VRB)**. The knob position is mapped 0.3–2.0 and multiplied by the floor base (0.5 m default), so the floor height ranges roughly **0.15 m to 1.0 m**. Turning ch9 fully low (≤ 0.35 scale) sets floor to **0 (disabled)**.
3. This value is continuously re-read and "remembered" the whole time you're in landing mode. It sticks at whatever it was when you leave landing mode.

**How it's enforced (only outside landing mode):**
- Must be armed: `afloor arm` over BLE (see §7).
- Only engages if: not landing, floor > 5cm, altitude reading valid, AHRS valid, bank angle under the configured max (`afloor bank`, default from config).
- As true height (altitude × cos(tilt), to correct for banked flight) drops within the engage band above the floor, a PD controller (`afloor kp`/`afloor kd`) adds throttle proportional to how fast you're descending into the floor, capped at `afloor cap`.
- A hard backstop also holds throttle at mid-stick if you punch through the floor entirely.

**Tuning commands:** `afloor arm`, `afloor disarm`, `afloor cap <0–0.6>`, `afloor band <0.2–6.0 m>`, `afloor kp <0–5>`, `afloor kd <0–5>`, `afloor bank <5–30 deg>`, `afloor status`.

---

## 5. Ceiling (Altitude Corridor — Upper Bound)

This one **is** a genuine "capture current altitude" mechanism.

**How to set it:**
1. Fly level (bank angle under 15°) at the altitude you want as your ceiling.
2. Flip **ch14 above 1400µs** — the rising edge of this switch is what captures the current altitude into the ceiling value. Must be level and altitude-valid at that instant, or the capture is skipped.
3. Leave ch14 high to keep the ceiling enforced.

**How it's enforced:**
- Only while ch14 is high, altitude reading valid, altitude sensible (0–8 m band), and you're actually climbing.
- If you climb above the captured ceiling, throttle is clamped to mid-stick. No knob/gain tuning exposed for ceiling — it's a hard clamp, not a PD-damped approach like the floor.
- Dropping ch14 low disarms it; flipping it high again re-captures at whatever altitude you're at on that new rising edge.

---

## 6. Obstacle Avoidance — Horizontal Sensitivity

### 6.1 How the scale knob (ch9) actually works

Ch9 does double duty (§4 explains the floor half). For avoidance, it's mapped 0.3–2.0 and sent to the **Sensory Cortex** every cycle inside the `ProximityCmd` packet. The Sensory Cortex uses it to scale its own static/dynamic block-distance thresholds — this is what actually makes obstacles "trip" sooner or later, not anything in the Comm Cortex's braking curve.

The Comm Cortex's own authority ramp (how hard it brakes/pushes as time-to-impact shrinks) uses **fixed** timing thresholds — `TTE_WARN_S = 1.0s`, `TTE_BRAKE_S = 0.5s` — deliberately not knob-scaled, since impact timing shouldn't need re-tuning by hand. Turning the knob up doesn't make the drone brake more aggressively in time, it makes it start reacting to obstacles at a greater physical distance.

### 6.2 Reaction stages, per side (front/left/right/back)

1. **Clear** — stick passes through untouched.
2. **Influence zone** (TTE between 0.5s and 1.0s) — stick authority is progressively reduced the closer the TTE gets to the brake threshold, further damped if closing velocity is high.
3. **Brake** — stick is zeroed to mid-point the instant the Sensory Cortex reports the zone blocked, or TTE < 0.5s.
4. **Active push-away** (optional, armed separately) — if `ahoriz arm` is set and you're not in landing mode, once braked the system actively pushes the opposing stick away from the obstacle, scaled by closing speed via its own PD controller.

Map-based secondary nudge: if the live obstacle map (not just the 5 point sensors) reports a repulsion force above a small deadband, and that axis isn't already being braked, the corresponding stick gets a proportional authority cut.

### 6.3 Tuning commands
`ahoriz arm`, `ahoriz disarm`, `ahoriz cap <0–0.6>`, `ahoriz kp <0–2>`, `ahoriz kd <0–2>`, `ahoriz vref <50–1000 cm/s>`, `ahoriz engage <0–200 cm/s>`, `ahoriz status`.

### 6.4 Master switches
`on` / `off` — avoidance overall. `horiz on/off` — horizontal only. `alt on/off` — floor+ceiling only. `map on/off` — map-based nudge only.

---

## 7. BLE Command Reference (Comm Cortex — "Avoidance" characteristic)

| Command | Effect |
|---|---|
| `on` / `off` | Master avoidance enable/disable |
| `horiz on` / `horiz off` | Horizontal avoidance enable/disable |
| `alt on` / `alt off` | Floor + ceiling enable/disable |
| `map on` / `map off` | Map-based nudge enable/disable |
| `afloor arm` / `disarm` | Active-descent floor braking |
| `afloor cap/band/kp/kd/bank <val>` | Floor PD tuning (see §4) |
| `afloor status` | Report current floor tuning + armed state |
| `ahoriz arm` / `disarm` | Active push-away braking |
| `ahoriz cap/kp/kd/vref/engage <val>` | Horizontal PD tuning (see §6) |
| `ahoriz status` | Report current horizontal tuning + armed state |

Other Comm Cortex characteristics: **Sensory** (`on`/`off` — whether the Sensory Cortex link is expected), **Failsafe** (write a PPM µs value 1000–2000 for the RTH failsafe position), **Timeouts** (`sensor:<ms> ibus:<ms>` — watchdog timeout tuning).

---

## 8. Calibration — Full Procedures

### 8.1 Compass (HMC5883L) hard-iron calibration — BLE, in-flight-ready

1. Power the drone fully (motors disarmed, ESCs live).
2. Write `go` to the **HMC Cal** BLE characteristic.
3. Rotate the drone slowly through all axes for 30 seconds — pitch, roll, and yaw through their full range, not just spinning flat.
4. The **Cal Status** characteristic notifies once per second with a countdown ("collecting - Ns left").
5. After 30s it automatically computes `offset = (max+min)/2` per axis, **saves to flash**, and notifies `done X=.. Y=.. Z=..`.

No further action needed — offsets persist across power cycles. Re-run any time you change the drone's frame, add magnetic hardware near the compass, or notice heading drift.

### 8.2 Optical-flow scale calibration — corrected workflow (v2.0.4 fix applied)

The original v2.0.4 code had a real bug: writing the distance reset the accumulator and only started counting *after* you'd already told it the flight was over — so it calibrated from near-zero motion instead of the actual flight. **This has been fixed** in the version below. The characteristic is now a proper two-stage start/finish, matching how the compass cal already worked:

1. Write `go` to the **OF Cal** BLE characteristic. This arms accumulation and zeroes the internal distance counter.
2. Fly a known, straight-line distance at a **steady altitude** (accuracy depends on altitude being roughly constant during the run — the scale factor is distance-per-pixel-per-metre-of-altitude).
3. Land or hold a steady hover, then write the distance you actually flew, in centimetres (accepted range 10–2000 cm), to the same characteristic.
4. The characteristic replies `calculating...`, then within about a second reports the computed scale, e.g. `scale 0.00123`, and **saves it to flash automatically**.

If it instead replies with an error:
- `error: write 'go' first, then fly, then write distance in cm` — you wrote a distance without first writing `go`.
- `error: enter distance flown in cm (10-2000)` — the value entered was outside the accepted range or not parseable as a number.
- `error: result out of range, try again` — the computed scale was implausible (e.g. altitude sensor invalid during the flight, or barely any distance flown); repeat the run.
- `error: fly further or check altitude` — accumulated flow distance or altitude was too small to get a reliable result; fly a longer straight-line distance and make sure the altitude sensor was reading validly throughout.

Re-run this if you change the optical flow sensor, its mounting height/angle, or the lens.

### 8.3 TF-Luna address assignment (one-time, per sensor)

Power **one TF-Luna at a time** on the shared I2C bus. Write the new address to register `0x22`, then save it with a write of `0x00` to register `0x25`. Repeat per sensor until all five have unique addresses (defaults used in this firmware: 0x10 Front, 0x11 Left, 0x12 Right, 0x13 Back, 0x14 Down).

### 8.4 Magnetic declination (one-time, per flying location)

Look up your local declination at https://www.ngdc.noaa.gov/geomag/calculators/magcalc.shtml, then update `CFG_MAG_DECLINATION_DEG` in the firmware and reflash. `COS_DECL`/`SIN_DECL` are derived automatically from that single value at boot — you only ever need to touch the one `CFG_MAG_DECLINATION_DEG` constant. Re-do this if you relocate to fly somewhere with meaningfully different declination.

### 8.5 Mahony AHRS gain tuning (firmware constant, rarely needed)

`MAHONY_KP` defaults to 2.0. If the attitude estimate oscillates, lower it; if it drifts, raise it. This is a compile-time constant, not BLE-tunable — requires a reflash to change.

### 8.6 Other BLE-tunable Sensory Cortex characteristics (setup, not calibration per se)
- **Point TTL** — map point lifetime in seconds, `5`–`600`.
- **Baro weight** — barometer fusion weight `0.0–1.0` (internally clamped `0.05–0.95` to always keep both sensors blended).
- **OF weight / cvel sub-commands** — `cvel alpha <0.05–1.0>`, `cvel gate <100–3000>`, `cvel inertial <0.0–1.0>`, `cvel status`; plain `0.0–1.0` sets legacy OF blend weight.
- **Map Clear** — write `clear` to wipe the live map, or `reset` to erase saved flash tunings/calibration (defaults resume next boot).

---

## 9. Fault Bits & LED States

**SafetyPacket fault byte:** bit0 front, bit1 left, bit2 right, bit3 AHRS/compass, bit4 altitude, bit5 optical flow, bit6 back, bit7 barometer.

**Comm Cortex status LED:** dim green solid = nominal · amber blink = influence zone · red solid = braking · red blink = RC lost · purple pulse = landing mode · slow green blink = Sensory Cortex link dead.

---

## 9. Changelog — This Session's Firmware Changes

**Comm Cortex: no changes.** `comm_cortex_v2_1_6.ino` provided alongside this doc is byte-identical to what you uploaded — confirmed with a diff, not just by eye. Still v2.1.6, still has the cosmetic stale version-string line noted in §10 if you want that touched later.

**Sensory Cortex: v2.0.4 → v2.0.5.** One functional fix, tagged `[v2.0.5 S-025]` in the source to match your existing changelog-comment convention (following on from the `v2.0.3 S-0xx` series). Exact diffs:

1. **Global state** (~line 220): replaced the single `g_ofCalPending` flag with two — `g_ofCalActive` (accumulating, from `go` to distance-entry) and `g_ofCalFinalize` (result pending computation). `g_ofCalDistCm`/`g_ofCalAccumX`/`g_ofCalAccumY` unchanged.
2. **`of_update()`** (~line 1196): accumulation condition changed from `if (g_ofCalPending)` to `if (g_ofCalActive)` — this is the actual behavior fix, since it now accumulates for the whole flight window instead of only after you'd already entered the distance.
3. **BLE `OFCAL` write handler** (~line 1420s): rewritten to accept `go` (arms + zeroes accumulator) as a first step, then a numeric distance (10–2000 cm) as a second step that stops accumulation and flags for finalization. Added explicit error strings for out-of-sequence writes.
4. **`ble_update_notify()`** (~line 1582): finalize condition changed from `if (g_ofCalPending)` to `if (g_ofCalFinalize)`; math and result-validity checks unchanged.
5. **BLE characteristic default value** (~line 1497): placeholder text updated from `"write distance in cm after flying"` to `"write 'go', fly a known distance, then write that distance in cm"`, so a BLE app shows the correct workflow before any command is sent.
6. **Version markers**: header comment (line 3) and boot serial print now read `v2.0.5` instead of `v2.0.4`.

To confirm you've flashed the fixed build: serial monitor should print `[BOOT] Sensory Cortex v2.0.5` at boot, and writing a bare numeric distance to the OF Cal characteristic *without* first writing `go` should return `error: write 'go' first, then fly, then write distance in cm` instead of silently starting a (broken) calibration.

## 10. Known Open Items

1. **Stale version string** — Comm Cortex line ~1890 prints "v2.1.4 running" instead of "v2.1.6"; cosmetic, doesn't affect operation. Not yet patched in the shared build.

Everything else — ceiling capture, floor engagement, horizontal avoidance authority curve, map-based nudge, all BLE command parsers, both `SafetyPacket`/`ProximityCmd` struct layouts, and the OF calibration accumulator (fixed, §8.2) — checked out logically consistent with the code as written.

**Firmware file provided alongside this doc:** `sensory_cortex_v2_0_5.ino` includes the OF-calibration fix described in §8.2/§9. Flash this version to pick up the corrected two-stage `go`/distance workflow — confirm via the v2.0.5 boot banner. `comm_cortex_v2_1_6.ino` is unchanged from what you uploaded (paired here for convenience — still has the cosmetic version-string issue above if you want it fixed too, just say so).
