# Sensory Cortex — Wiring & Pinout

**Firmware:** `sensory_cortex_v2_0_4.ino` · **MCU:** ESP32-S3 · **Bus speed:** I²C 400 kHz

> **VERIFY-BUILD sections** are marked below. Those are **not** to be soldered
> from this document — build them from your own bench-proven wiring. Everything
> else is taken directly from the firmware's pin definitions.

---

## 1. ESP32-S3 pin assignments (from firmware)

### I²C bus — all ranging/attitude/pressure sensors share one bus
| Signal | GPIO | Notes |
|---|---|---|
| I2C_SDA | 8 | 400 kHz; pull-ups required (see §4) |
| I2C_SCL | 9 | 400 kHz |

### SPI bus — optical flow (PMW3901 / PAA5100)
| Signal | GPIO | Notes |
|---|---|---|
| OF_SPI_SCK | 4 | SPI clock |
| OF_SPI_MISO | 5 | |
| OF_SPI_MOSI | 6 | |
| OF_SPI_CS | 15 | Chip select — deliberately a flash-safe GPIO |

*SPI mode 0 (CPOL=0, CPHA=0), ≤ 2 MHz.*

### UART — inter-cortex link (to Communication Cortex)
| Signal | Notes |
|---|---|
| SafetyPacket TX → Comm PIN_SENSE_RX (GPIO16) | Sensory transmits obstacle state |
| ProximityCmd RX ← Comm PIN_SENSE_TX (GPIO17) | Sensory receives commands |

> Confirm the exact Sensory-side UART GPIOs against your board's `Serial1`
> assignment before wiring; the pin numbers on the **Comm** side are fixed
> (16/17) — see the Comm wiring doc. Cross TX↔RX.

---

## 2. I²C device address map (from firmware)

| Device | Address | Enable flag | Role |
|---|---|---|---|
| TF-Luna FRONT | 0x10 | `LUNA_FRONT_ENABLED` | Forward range |
| TF-Luna LEFT | 0x11 | `LUNA_LEFT_ENABLED` | Left range |
| TF-Luna RIGHT | 0x12 | `LUNA_RIGHT_ENABLED` | Right range |
| TF-Luna BACK | 0x13 | `LUNA_BACK_ENABLED` | Rear range |
| TF-Luna DOWN | 0x14 | `LUNA_DOWN_ENABLED` | Altitude / floor |
| ICM-42688-P | 0x68 | (always) | 6-axis IMU |
| HMC5883L | 0x1E | (always) | Magnetometer |
| BMP280 | 0x76 | (always) | Barometer |

**TF-Luna address assignment:** all TF-Lunas ship at **0x10**. You must reassign
each one *before* fitting it to the shared bus. Power one sensor at a time, write
the new address to register `0x22`, then save with `0x00 → register 0x25` (per the
TF-Luna config procedure noted in the firmware header). Set the matching
`_ENABLED` flag to `0` for any sensor position you are not fitting.

---

## 3. Sensor mounting / orientation

| Sensor | Orientation |
|---|---|
| TF-Luna FRONT/LEFT/RIGHT/BACK | Horizontal, facing their named direction |
| TF-Luna DOWN | Facing straight down (altitude + active floor) |
| PMW3901 optical flow | Facing straight down, lens unobstructed |
| ICM-42688-P IMU | Axes aligned to airframe; note orientation for AHRS |
| HMC5883L mag | Away from power wiring / motors; declination set via `CFG_MAG_DECLINATION_DEG` |
| BMP280 baro | Shielded from prop wash and direct airflow |

The magnetic declination is now a single source of truth:
`CFG_MAG_DECLINATION_DEG` (default −0.8° for Long Eaton UK). Change that one value
for your location; the sin/cos rotation is derived from it at boot.

---

## 4. Power & pull-ups — **VERIFY-BUILD**

> The power tree and I²C pull-up values below are the *intent* from the project
> notes, not a verified schematic. Confirm against your own build.

- Sensors run from a regulated **3.3 V** rail.
- I²C needs pull-ups to 3.3 V on SDA and SCL (typically 2.2 k–4.7 kΩ; value
  depends on bus capacitance with five TF-Lunas plus three other devices — verify
  on the bench with a scope if you can).
- Decouple each sensor locally (100 nF at each device, bulk cap on the rail).
- Keep the analog/pressure sensor away from switching-regulator noise.

---

## 5. Boot-time serial output (115200 baud)

On boot the Sensory Cortex prints init results for every sensor, the declination
value, and the TWDT-armed line. Use these to confirm the bus is healthy before
enabling `DEBUG_SERIAL`:

```
[xx] TF-Luna Front(0x10) ...
[xx] TF-Luna Left(0x11)  ...
...
[OK] Declination: -0.80 deg (...)
[OK] TWDT armed: 2000ms panic-reset
[BOOT] Sensory Cortex v2.0.4
```

A missing sensor line, or a disabled/failed report, tells you exactly which
address is not answering.

---

## 6. Bench telemetry

Set `static const bool DEBUG_SERIAL = true;` at the top of the config block,
recompile, open serial at 115200. Streams distances, closing velocities, TTEs,
attitude, position, altitude, optical flow, map occupancy and fault byte at
~10 Hz. Set back to `false` for flight.
