# CoRTX — Dual-Cortex Perception Coprocessor

A real-time perception coprocessor that adds spatial awareness to an existing
flight controller (e.g. DJI Naza-M V2) **without replacing it**. The flight
controller keeps command of stabilisation, attitude, and motor control; CoRTX
acts as its perception cortex, supplying bounded avoidance guidance the flight
controller may act on.

> **Safety note — read before wiring.** This document set is generated from the
> firmware's pin definitions. It is accurate to the code, but **no connection
> here has been physically verified**. Three subsystems are marked
> **VERIFY-BUILD** in the wiring docs — the CD4053 failsafe/bypass path, the
> 5 V↔3.3 V level shifting, and the power distribution rails. Do **not** solder
> those from this document. Build them only from your own bench-proven wiring,
> because they sit between the firmware and spinning propellers.

---

## Firmware versions

| Board | File | Version |
|---|---|---|
| Communication Cortex | `comm_cortex_v2_1_6.ino` | v2.1.6 |
| Sensory Cortex | `sensory_cortex_v2_0_4.ino` | v2.0.4 |

Both share the `SafetyPacket` / `ProximityCmd` protocol contract, guarded by
`SAFETY_PKT_VERSION`. **If you change a shared packet layout, bump
`SAFETY_PKT_VERSION` in BOTH firmwares** or they will (correctly) reject each
other.

---

## Architecture

Two ESP32-S3 processors with a clean responsibility split:

- **Sensory Cortex** — *"What does the world look like?"* Sensor acquisition,
  Mahony AHRS, dead reckoning, obstacle mapping (persistent voxel hash grid),
  optical-flow drift correction, altitude fusion. Emits a `SafetyPacket`. Makes
  **no** flight-control decisions.
- **Communication Cortex** — *"What should the aircraft do?"* iBUS RX, flight-mode
  logic, obstacle-avoidance mixing, RMT-based PPM generation, hardware watchdog,
  telemetry, failsafe. Ignorant of raw sensor detail; consumes only the
  `SafetyPacket`.

They communicate over a dedicated UART link:
- Sensory → Comm: `SafetyPacket` (obstacle state, confidence, faults)
- Comm → Sensory: `ProximityCmd`

If the Sensory Cortex fails, the aircraft keeps flying under its primary flight
controller; the Comm Cortex detects the loss and disables avoidance
(degrades to plain RC passthrough).

---

## Safety model (summary)

- The flight controller is **always** in command. CoRTX guidance is advisory and
  bounded; it mixes into pilot input, never replaces stabilisation.
- **Hardware bypass:** a CD4053 analog path can route raw receiver signal to the
  flight controller independent of firmware, FreeRTOS, UART, or BLE.
- **Hardware watchdog:** the Comm Cortex emits a 50 Hz pulse (GPIO5) through an
  RC filter to the CD4053 SELECT line. If the firmware stalls, the pulse stops,
  the filter discharges, and the CD4053 reverts to the raw path.
- **Task watchdog (TWDT):** both boards run the ESP32 hardware task watchdog
  (2 s, panic-reset) on their critical tasks.
- **Graceful degradation:** sensor faults reduce confidence before they change
  authority; missing packets reduce authority; recovery requires sustained good
  data (hysteresis) before authority is restored.

---

## Toolchain / build

- **Board package:** Arduino-ESP32 **3.x** (ESP-IDF 5.x). *Required* — the code
  uses the IDF-5 RMT driver (`driver/rmt_tx.h`) and the struct-based task
  watchdog API. It will **not** compile on Arduino-ESP32 2.x.
- **Board:** "ESP32S3 Dev Module" for both.
- **Arduino IDE settings:**

| Setting | Sensory Cortex | Communication Cortex |
|---|---|---|
| USB CDC On Boot | Enabled | Enabled |
| Flash Size / Partition | Huge APP | Huge APP |
| PSRAM | **OPI PSRAM** (required) | **Disabled** |

The Sensory Cortex allocates its obstacle map in PSRAM; if PSRAM is not enabled
it halts at boot with a `[FATAL] ps_malloc failed` message (by design — a reboot
would not fix a build-config error).

### Libraries
- `ICM42688` (IMU)
- `Bitcraze_PMW3901` (optical flow)
- `Adafruit_BMP280` (barometer)
- ESP32 BLE (bundled with the core)

---

## Bench testing — `DEBUG_SERIAL`

Each firmware has a single boolean at the top of its config block:

```cpp
static const bool DEBUG_SERIAL = false;   // ← set true for bench testing
```

Set `true`, recompile, and open a serial monitor at **115200 baud**. Set `false`
for flight (the block const-folds away — zero runtime cost). Output is throttled
to ~10 Hz so serial I/O cannot stall the loop or distort timing measurements.

- **Sensory** streams *perception*: distances, closing velocities, TTEs,
  attitude, position, altitude, optical flow, map occupancy, fault byte.
- **Comm** streams *decisions*: output channels (R/P/T/Y/FM), avoidance scale,
  `rcLost`/`sensDead`/`RTH`/`land` flags, active floor/brake engagement, and the
  blocked flags received from Sensory.

Read side by side (two monitors) you can watch a stimulus travel the whole chain:
wave a target → Sensory shows distance drop and map entry → Comm shows the
blocked flag, scale change, and channel bias — while confirming outputs stay
within 1000–2000 µs.

### Suggested bench sequence
1. **Powered bring-up** — confirm boot lines, packet flow, CD4053 in safe state.
2. **Static sensor sanity** — every sensor reads truthfully at rest.
3. **Hand-movement** — move by hand, watch obstacle → map → avoidance response.
   *(This is where to confirm the `vertVelCmS` sign convention for the active
   floor — descend by hand and check the throttle response direction.)*
4. **Fault injection** — pull the Sensory link / a LiDAR / the iBUS; confirm
   `sensDead`, latched sensor faults, and failsafe fire as expected.
5. **Timing / soak** — log loop time and latency; run 1–2 h for memory/map
   stability.

**Props off for all bench testing.**

---

## File map

```
docs/
  README.md                     ← this file
  WIRING_sensory_cortex.md      ← Sensory pinout + connections
  WIRING_comm_cortex.md         ← Comm pinout + connections
  schematic_sensory_cortex.svg  ← Sensory system block diagram
  schematic_comm_cortex.svg     ← Comm system block diagram
```
