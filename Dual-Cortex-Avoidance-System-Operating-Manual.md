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


| Feature | Status |
|---|---|
| Ceiling capture | ✅ Correct — edge-triggered capture, level-flight gated |
| Floor set/capture | ✅ Correct, but note it's **knob-set**, not altitude-captured — see §4 |
| Horizontal avoidance sensitivity | ✅ Correct — `scale` propagates via `ProximityCmd` to the Sensory Cortex's block thresholds, not locally in the TTE ramp (intentional split, not a bug) |
| Optical-flow BLE calibration | ✅ **Fixed** — see §8.2 for the corrected workflow |
| Version string at boot | ⚠️ Cosmetic only, SHOULD READ CURRENT VERSION

---

## 4. Floor (Altitude Corridor — Lower Bound)

**This is a dialed-in floor, not a "capture current altitude" button** — worth understanding since it's easy to expect it to behave like the ceiling.

**How to set it:**
1. Enter **landing mode** (ch13 > 1400µs).
2. While in landing mode, dial **ch9 (Proximity VRB)**. The knob position is mapped 0.3–2.0 and multiplied by the floor base (0.5 m default), so the floor height ranges roughly **0.15 m to 1.0 m**. Turning ch9 fully low (≤ 0.35 scale) sets floor to **0 (disabled)**.
3. This value is continuously re-read and "remembered" the whole time you're in landing mode. It sticks at whatever it was when you leave landing mode — and **as of v2.1.8, it's also saved to flash the instant landing mode ends**, so it survives a power cycle and comes back automatically next boot without re-dialing.

**How it's enforced (only outside landing mode):**
- Armed by default at boot (v2.1.7+) — no BLE tap needed every flight. Landing mode itself suppresses active assist, so "armed whenever not landing" is the default behavior. `afloor disarm` over BLE is still available as a manual override if you want it off entirely (e.g. bench testing).
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
| `afloor arm` / `disarm` | Active-descent floor braking (armed **by default at boot**, always — this one specific setting is NOT saved/restored, see §9) |
| `afloor cap/band/kp/kd/bank <val>` | Floor PD tuning (see §4) — saved to flash, restored at boot |
| `afloor status` | Report current floor tuning + armed state |
| `ahoriz arm` / `disarm` | Active push-away braking — saved to flash, restored at boot |
| `ahoriz cap/kp/kd/vref/engage <val>` | Horizontal PD tuning (see §6) — saved to flash, restored at boot |
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

**Comm Cortex: v2.1.6 → v2.1.8.** (v2.1.7's default-armed-floor change is folded into this — if you already flashed v2.1.7, this supersedes it, no separate step needed.)

**v2.1.7 change (included):** `g_activeFloorArmed` default changed from `false` to `true` — active floor climb-assist now works every flight without a BLE tap, since the existing `!landing` check already suppresses it during landing mode.

**v2.1.8 change (new, tagged `[v2.1.8 I-007]`):** all BLE-writable tunings and configs now persist to flash and reload automatically at boot. Specifically:

- **New `Preferences`-based storage** (mirrors the Sensory Cortex's existing pattern): `prefs_save()` writes every persisted key, `prefs_load()` reads them all back with the current compiled value as fallback if a key was never saved (so first boot after flashing behaves exactly as before).
- **`prefs_save()` is called after every valid BLE write** to: master avoidance toggles (`on`/`off`, `horiz`, `alt`, `map`), sensory-present toggle, all `afloor` tuning values (`cap`/`band`/`kp`/`kd`/`bank`), all `ahoriz` tuning values (`cap`/`kp`/`kd`/`vref`/`engage`) **and its arm/disarm state**, failsafe RTH value, and watchdog timeouts. These are naturally low-frequency (a person tapping a BLE app), so there's no flash-wear concern.
- **Floor height (`g_rememberedFloorM`) and horizontal scale (`g_rememberedScaleH`)** are handled differently, since they're derived continuously from the ch9 knob rather than from discrete BLE writes — saving on every loop would both wear the flash and risk a momentary stall in `task_logic()` from a real-time control loop. Instead they're saved at their natural, infrequent checkpoints: floor height is saved the instant landing mode ends (it's finished being dialed in at that point), and horizontal scale is saved the instant landing mode begins (it's about to freeze anyway). In practice that's at most a couple of flash writes per flight, not per loop.
- **One deliberate exception:** `g_activeFloorArmed` itself is **not** persisted. It always boots `true` (the v2.1.7 behavior) regardless of whether you disarmed it last session — otherwise a single `afloor disarm` would silently stick forever and defeat the point of defaulting it on. `afloor arm`/`disarm` still work live for the current session, they just don't survive reboot. If you'd actually rather it remembered a manual disarm across reboots too, say so and I'll change that one exception — it's a one-line change to move it into `prefs_save`/`prefs_load` alongside `ahoriz`'s armed state, which I did persist since it has no equivalent "must default on" requirement.
- **Trade-off worth knowing:** the two edge-triggered flash writes in `task_logic()` run on Core 1, in the same task that drives horizontal avoidance and the floor/ceiling corridor. An NVS write is typically single-digit milliseconds, occurring only at a landing-mode start/end transition (already a low-dynamics moment) — but it is a real, if brief, blocking call inside the control task. Flag if you'd rather this be offloaded to a queue and written from a lower-priority task instead; it's more code but removes even that small window.
- Version bumped: header comment and boot banner now read `v2.1.8`. On boot, USB serial now also prints the loaded config: `[OK] Loaded saved config: avoid=1 horiz=1 alt=1 map=1 floor=0.00m scaleH=1.00` (or whatever was last saved) — a quick way to confirm persistence is working.

**Sensory Cortex: no changes this round.** `sensory_cortex_v2_0_5.ino` from the previous fix is unchanged — still current, still has the OF-calibration fix below (kept here for reference).

<details>
<summary>Sensory Cortex v2.0.4 → v2.0.5 (previous round, unchanged since)</summary>

1. **Global state** (~line 220): replaced the single `g_ofCalPending` flag with two — `g_ofCalActive` (accumulating, from `go` to distance-entry) and `g_ofCalFinalize` (result pending computation). `g_ofCalDistCm`/`g_ofCalAccumX`/`g_ofCalAccumY` unchanged.
2. **`of_update()`** (~line 1196): accumulation condition changed from `if (g_ofCalPending)` to `if (g_ofCalActive)` — this is the actual behavior fix, since it now accumulates for the whole flight window instead of only after you'd already entered the distance.
3. **BLE `OFCAL` write handler** (~line 1420s): rewritten to accept `go` (arms + zeroes accumulator) as a first step, then a numeric distance (10–2000 cm) as a second step that stops accumulation and flags for finalization. Added explicit error strings for out-of-sequence writes.
4. **`ble_update_notify()`** (~line 1582): finalize condition changed from `if (g_ofCalPending)` to `if (g_ofCalFinalize)`; math and result-validity checks unchanged.
5. **BLE characteristic default value** (~line 1497): placeholder text updated from `"write distance in cm after flying"` to `"write 'go', fly a known distance, then write that distance in cm"`.
6. **Version markers**: header comment and boot serial print now read `v2.0.5`.

</details>

## 10. Known Open Items

1. **Stale version string** — Comm Cortex line ~1890 prints "v2.1.4 running" instead of "v2.1.6"; cosmetic, doesn't affect operation. Not yet patched in the shared build.

Everything else — ceiling capture, floor engagement, horizontal avoidance authority curve, map-based nudge, all BLE command parsers, both `SafetyPacket`/`ProximityCmd` struct layouts, and the OF calibration accumulator (fixed, §8.2) — checked out logically consistent with the code as written.

**Firmware files provided alongside this doc:** `sensory_cortex_v2_0_5.ino` includes the OF-calibration fix described in §8.2/§9. `comm_cortex_v2_1_8.ino` includes the default-armed floor fix and full BLE config/tuning persistence described in §9 — confirm via the `v2.1.8` boot banner and the `[OK] Loaded saved config...` line right after it. Both should be flashed to their respective boards as a matched pair.
