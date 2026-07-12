/*
 * ================================================================
 *  Communication Cortex — v2.2.0-SBUS
 *  Dual Cortex Obstacle Avoidance System — SBUS receiver variant
 * ================================================================
 *
 *  Board: ESP32-S3 Super Mini
 *  Sits between an SBUS receiver (FrSky/Futaba/TBS/Radiomaster etc.)
 *  and the flight controller (Naza M V2 or any PPM/SBUS-input FC).
 *
 *  INPUT:  SBUS from receiver (16ch, 100000 baud 8E2 INVERTED, GPIO4)
 *          — inversion done in SOFTWARE by the ESP32-S3 UART peripheral.
 *            NO external inverter (74HCT04) and NO divider required:
 *            connect the receiver's SBUS pad (3.3V logic) directly
 *            to GPIO4. (5V-level SBUS receivers still need a divider.)
 *          SafetyPacket / MapPacket from Sensory Cortex (UART1)
 *
 *  OUTPUT (compile-time select — see OUTPUT_PROTOCOL below):
 *    OUT_PPM  : Modified 8ch PPM via RMT peripheral, GPIO6 (default)
 *    OUT_SBUS : Modified 16ch SBUS via UART2 TX, GPIO6,
 *               100000 8E2, software-inverted (standard SBUS polarity)
 *    Either way the signal feeds CD4053 IN-B, so the failsafe mux
 *    board is IDENTICAL for both variants. With OUT_SBUS, CD4053
 *    IN-A must carry the receiver's raw SBUS (like-for-like bypass)
 *    and the FC port must be an SBUS input (Naza M V2: X2 port).
 *          GPIO aux outputs (floodlight, VTX, nav LEDs)
 *          ProximityCmd to Sensory Cortex (UART1)
 *          50Hz watchdog pulse (GPIO5) → RC filter → CD4053 bypass
 *
 *  AVOIDANCE: identical to v2.1.8. All channel arithmetic stays in
 *  microseconds (1000–2000µs). SBUS raw values (172–1811) are mapped
 *  to µs at the decode boundary and back at the encode boundary:
 *      µs   = 880 + (raw × 5 + 4) / 8      (172→988, 992→1500, 1811→2012,
 *                                           then clamped to 1000–2000)
 *      raw  = ((µs − 880) × 8 + 2) / 5     (inverse, clamped 172–1811)
 *
 *  SBUS INPUT:
 *    25-byte frame: [0x0F] [22B: 16×11-bit ch LE] [flags] [end].
 *    end = 0x00 (SBUS1) or 0x04/0x14/0x24/0x34 (SBUS2 slots — accepted).
 *    flags bit3 = FAILSAFE  → frame rejected; RC-loss watchdog handles it.
 *    flags bit2 = FRAME-LOST → accepted (data is last-known-good).
 *    Frame rate 7ms (high-speed) or 14ms (analog) — both fine.
 *    Sync: 0x0F can occur inside the payload, so byte position alone is
 *    not enough. Frame start is found by inter-frame idle gap (>3ms)
 *    plus header AND end-byte validation on every frame.
 *    ch[0]–ch[13] used; SBUS ch15/16 and digital ch17/18 ignored.
 *
 *  TELEMETRY: the FlySky iBUS half-duplex telemetry responder from the
 *  v2.1.8 iBUS build does NOT apply to SBUS receivers and is removed.
 *  GPIO20/21 are free. (The g_tel* values are still computed for BLE /
 *  debug and for a future S.Port/CRSF telemetry add-on.)
 *
 *  AUX GPIO OUTPUTS (driven from channel values, task_logic):
 *    Floodlight (ch10, idx9)  → PIN_FLOOD (active HIGH to FET gate)
 *    VTX on/off  (ch11, idx10) → PIN_VTX   (active HIGH)
 *    Nav LEDs    (ch12, idx11) → 4 corner pins via inverter/transistor
 *
 *  BYPASS PATH (hardware, ESP-independent):
 *    ch8 output from receiver → decoder → ch8 PWM → Pololu RC
 *    switch → CD4053 Switch A SELECT. Completely upstream of this ESP.
 *    GPIO5 watchdog pulse (50Hz) → 47kΩ + 1µF RC filter → also feeds
 *    CD4053 Switch A SELECT. If ESP crashes, pulse stops, filter
 *    discharges (~40–100ms), CD4053 routes the RAW receiver signal
 *    (PPM or SBUS — must match what the FC port expects) to the FC.
 *
 *  CHANNEL MAP (default — see USER CONFIGURATION block below):
 *    Ch1  idx0  Roll          → FC, avoidance-modified
 *    Ch2  idx1  Pitch         → FC, avoidance-modified
 *    Ch3  idx2  Throttle      → FC, floor/ceiling-modified
 *    Ch4  idx3  Yaw           → FC, pass-through
 *    Ch5  idx4  Flight Mode   → RTH detect + failsafe (see note below)
 *    Ch6  idx5  IOC           → FC, pass-through
 *    Ch7  idx6  Aux / Gimbal  → FC, pass-through
 *    Ch8  idx7  CD4053 bypass → upstream hardware only, ESP passes thru
 *    Ch9  idx8  Proximity VRB → avoidance scale / floor set
 *    Ch10 idx9  Floodlight    → PIN_FLOOD GPIO output
 *    Ch11 idx10 VTX on/off    → PIN_VTX GPIO output
 *    Ch12 idx11 Nav lights    → PIN_NAV_FL/FR/RL/RR GPIO outputs
 *    Ch13 idx12 Landing mode  → logic flag
 *    Ch14 idx13 Ceiling switch→ logic flag
 *    (OUT_SBUS: idx0–13 are transmitted; SBUS ch15/16 sent at 1500µs.)
 *
 *  NOTE: Naza M V2 PPM  = Flight Mode on PPM ch5  → CH_FLIGHT_MODE 4.
 *        Naza M V2 SBUS = Flight Mode on SBUS ch7 → CH_FLIGHT_MODE 6.
 *        Other FCs: set CH_FLIGHT_MODE to whichever TX channel carries
 *        your GPS/Atti/RTH switch. See USER CONFIGURATION block.
 *
 *  INTER-CORTEX UART (unchanged):
 *    UART1 RX GPIO16 ← SafetyPacket / MapPacket from Sensory Cortex
 *    UART1 TX GPIO17 → ProximityCmd to Sensory Cortex
 *
 *  v2.1.8 (iBUS) → v2.2.0-SBUS changes:
 *    - iBUS 32-byte decoder replaced by SBUS 25-byte decoder
 *      (gap-synced, header+end validated, failsafe-flag aware)
 *    - Software UART inversion on RX (and TX in OUT_SBUS) — no new HW
 *    - Compile-time OUTPUT_PROTOCOL: OUT_PPM (RMT, as v2.1.8) or
 *      OUT_SBUS (UART2 TX GPIO6, 14ms frame, 16ch)
 *    - iBUS telemetry responder removed (FlySky-specific); GPIO20/21 free
 *    - Identifiers renamed protocol-neutral (RC_MIN/MID/MAX, g_wdgRcMs…)
 *      NVS keys: "wdgIbus" → "wdgRc"; BLE timeout token "ibus:" → "rc:"
 *    - All avoidance logic, BLE, NVS persistence, TWDT, watchdog pulse,
 *      SafetyPacket interface: UNCHANGED from v2.1.8
 *
 * ================================================================
 *
 *  HARDWARE vs the v2.1.8 iBUS build:
 *    IDENTICAL BOARD. No inverter ICs, no dividers (3.3V receivers).
 *    GPIO4 now carries SBUS-in instead of iBUS-in (same pad).
 *    GPIO6 carries PPM-out or SBUS-out (same pad, same CD4053 IN-B).
 *    GPIO20/21 (iBUS telemetry) unused in this variant.
 *    CD4053:  IN-A = RAW receiver output (PPM or SBUS, match the FC)
 *             IN-B = ESP output from GPIO6
 *             OUT  = FC RC input port
 *             (same bypass logic, 3.3V throughout — no level shift)

 *
 * ================================================================
 *
 *  Arduino IDE settings (unchanged):
 *    Board:     ESP32S3 Dev Module
 *    Flash:     8MB
 *    PSRAM:     Disabled
 *    Partition: Huge APP (3MB No OTA)
 *    USB CDC:   DISABLED for flight builds.
 *               ENABLED for bench builds — routes USBSerial debug to
 *               the USB monitor (see Serial routing block below) and
 *               pairs with DEBUG_SERIAL = true.
 *
 *  Libraries:
 *    FastLED    (WS2812 status LED)
 *    ArduinoBLE (BLE config server)
 *    ESP32 RMT  (built-in via esp32-hal-rmt.h)
 *
 * ================================================================
 */

#include <Arduino.h>
#include <FastLED.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/rmt_tx.h"
#include "esp_wifi.h"
#include "esp_task_wdt.h"   // v2.1.5: hardware Task Watchdog (TWDT)
#include <Preferences.h>    // [v2.1.8 I-007] persist BLE tunings/configs across reboot

// ── Serial routing: debug vs RC UART ──────────────────────────
// [v2.2.0 I-105] The debug port and the RC input UART are now routed
// by the USB CDC board setting, so BOTH settings compile:
//
//   USB CDC On Boot: ENABLED  (bench) — `Serial` is the USB CDC object,
//     `Serial0` is hardware UART0. Debug (USBSerial) → USB monitor,
//     SBUS RX (RC_SERIAL) → UART0. Set DEBUG_SERIAL true and watch
//     the decision stream over USB while feeding SBUS on GPIO4.
//
//   USB CDC On Boot: DISABLED (flight) — `Serial` IS hardware UART0
//     and `Serial0` does not exist. Both aliases map to UART0; debug
//     writes drain into the unattached TX and vanish (v2.1.8 behavior).
//
// v2.1.8 aliased USBSerial=Serial unconditionally, which made the
// CDC-enabled build FAIL to compile (the CDC class has no
// begin(baud, config, rx, tx, invert) overload for the RC init).
#if ARDUINO_USB_CDC_ON_BOOT
  #define USBSerial  Serial    // USB CDC — bench debug output
  #define RC_SERIAL  Serial0   // hardware UART0 — SBUS input
#else
  #define USBSerial  Serial    // UART0 (TX unattached — prints vanish)
  #define RC_SERIAL  Serial    // same UART0 — SBUS input
#endif

// ── Output protocol select (COMPILE-TIME) ─────────────────────
// OUT_PPM : 8ch PPM via RMT on GPIO6 (identical to v2.1.8 output)
// OUT_SBUS: 16ch SBUS via UART2 TX on GPIO6, 100000 8E2,
//           software-inverted, 14ms frame. CD4053 IN-A must then
//           carry the receiver's RAW SBUS and the FC port must be
//           an SBUS input (Naza M V2: X2 port).
#define OUT_PPM   0
#define OUT_SBUS  1
#define OUTPUT_PROTOCOL  OUT_PPM   // ← select before compiling

// ── GPIO map ──────────────────────────────────────────────────
#define PIN_RC_RX         4   // UART0 RX — SBUS in (software-inverted, 3.3V)
//                            // UART0 TX not used (SBUS input is RX-only)
#define PIN_RC_OUT        6   // Output to CD4053 IN-B:
                              //   OUT_PPM  → RMT PPM (idle-HIGH)
                              //   OUT_SBUS → UART2 TX (inverted, idle-LOW)
#define PIN_WD_PULSE      5   // 50Hz watchdog pulse → 47kΩ+1µF RC filter → CD4053 SELECT
#define PIN_FLOOD         1   // Floodlight FET gate (HIGH = on), ch10
#define PIN_VTX           2   // VTX power FET gate  (HIGH = on), ch11
#define PIN_LED          48   // Onboard WS2812 RGB status LED
// Nav LEDs — driven through 2N2222/HCT04 inverter stage (high-side, as before)
// NAV_LED_ON = LOW  when using inverting driver (HCT04 / 2N2222 open-collector pull-up)
// NAV_LED_ON = HIGH when driving LEDs directly from GPIO (no inverter)
#define PIN_NAV_FL        7   // Front-Left  — Green
#define PIN_NAV_FR        8   // Front-Right — Red
#define PIN_NAV_RL        9   // Rear-Left   — Green
#define PIN_NAV_RR       10   // Rear-Right  — Red
#define PIN_SENSE_RX     16   // UART1 RX — Sensory Cortex data in
#define PIN_SENSE_TX     17   // UART1 TX — Sensory Cortex command out
//  Free / future: GPIO3 (reserved Visual Cortex on Sensory side), GPIO48=LED above

// ── Nav LED polarity ──────────────────────────────────────────
// Set NAV_LED_ON = LOW  if using 2N2222 or HCT04 inverting driver
// Set NAV_LED_ON = HIGH if driving LEDs directly from GPIO
#define NAV_LED_ON   LOW
#define NAV_LED_OFF  HIGH

// ── Baud rates ────────────────────────────────────────────────
#define BAUD_SBUS      100000   // SBUS: 100000 8E2, INVERTED (done in software)
#define BAUD_SENSE     921600   // Sensory Cortex inter-cortex link

// ── SBUS constants ────────────────────────────────────────────
#define SBUS_FRAME_LEN     25   // 0x0F + 22 data + flags + end
#define SBUS_HEADER      0x0F
#define SBUS_END_1       0x00   // SBUS1 end byte
                                // SBUS2 slot bytes 0x04/0x14/0x24/0x34 also
                                // accepted: (end & 0xCF) == 0x04
#define SBUS_FLAG_FRAMELOST 0x04  // flags bit2 — informational, accepted
#define SBUS_FLAG_FAILSAFE  0x08  // flags bit3 — frame rejected
#define SBUS_RAW_MIN      172   // raw 11-bit range at ±100% travel
#define SBUS_RAW_MAX     1811
#define SBUS_GAP_US      3000   // inter-frame idle gap for RX sync (frames
                                // are 3ms of data every 7/14ms → real gap ≥4ms)
#define SBUS_OUT_PERIOD_MS 14   // OUT_SBUS frame period (analog-rate SBUS)

// ── Channel value constants (µs domain, protocol-neutral) ────
#define RC_NUM_CH        14   // channels used by this system
#define RC_MID         1500   // channel midpoint (µs)
#define RC_MIN         1000   // channel minimum (µs)
#define RC_MAX         2000   // channel maximum (µs)
#define RC_DEAD          30   // deadband around neutral (µs)

// ── PPM output constants ──────────────────────────────────────
// Standard RC PPM: idle HIGH, pulse = LOW sync_gap then HIGH for channel_us.
// 8 channels to Naza. Frame period 20ms (50Hz).
#define PPM_CHANNELS        8   // channels to Naza (ch1–7 + ch8 passthrough slot)
#define PPM_FRAME_US    20000   // total frame period µs
#define PPM_SYNC_US       300   // LOW pulse width between channels (µs)
#define PPM_RMT_TICK_HZ 1000000 // 1MHz RMT clock = 1µs resolution

// ── Naza PPM failsafe values (µs) ─────────────────────────────
// Default safe-state for every PPM channel on RC loss.
// The flight mode slot is overridden at runtime by ppm_failsafe()
// using CH_FLIGHT_MODE, so it is always consistent with the user
// config regardless of which index CH_FLIGHT_MODE points to.
static const uint16_t FAILSAFE_PPM[PPM_CHANNELS] = {
  1500,  // idx0 Roll     — neutral
  1500,  // idx1 Pitch    — neutral
  1500,  // idx2 Throttle — neutral (Naza holds position in GPS mode)
  1500,  // idx3 Yaw      — neutral
  1500,  // idx4          — overridden by ppm_failsafe() via CH_FLIGHT_MODE
  1500,  // idx5 IOC      — neutral
  1500,  // idx6 Aux      — neutral
  1000,  // idx7 Bypass   — low (safe state = raw receiver PPM)
};

// ── Avoidance zone distances (cm, at scale 1.0) ───────────────
#define ZONE_OUTER_CM    200.0f
#define ZONE_MID_CM      100.0f
#define ZONE_INNER_CM     60.0f
#define TTE_BRAKE_S        0.5f
#define TTE_WARN_S         1.0f
#define AUTHORITY_MIN      0.20f
#define SCALE_MIN          0.3f
#define SCALE_MAX          2.0f
#define MAP_NUDGE_MAX      0.30f
#define MAP_FORCE_DEADBAND 0.10f

// ── Watchdog timeouts ─────────────────────────────────────────
#define WDG_SENSOR_MS       50
#define WDG_RC_MS        100   // RC-loss watchdog (SBUS frame gap)
#define WD_PULSE_HZ         50

// ════════════════════════════════════════════════════════════════
//  USER CONFIGURATION — edit this block to match your wiring
// ════════════════════════════════════════════════════════════════

// ── Bench debug telemetry ────────────────────────────────────
// Set true for bench testing to stream the DECISION-side state over
// USBSerial (115200): output channels R/P/T/Y/FM, avoidance scale,
// the rcLost / sensorDead / RTH / landing flags, active floor/brake
// engagement, and the blocked flags received from the Sensory Cortex.
// Set false for flight (block compiles out — zero runtime cost).
// Prints every DBG_EVERY_N logic iterations so it can't flood the
// link or perturb loop timing.
static const bool DEBUG_SERIAL = false;   // ← flip to true on the bench
#define DBG_EVERY_N 15   // ~10 Hz at the 71–142 Hz SBUS logic rate

// ── Flight Mode channel ───────────────────────────────────────
// Which RC channel carries your GPS / Atti / RTH switch.
// This is the 0-based index (channel number minus 1).
//
// Naza M V2 PPM mode  (OUT_PPM):  Flight Mode = Naza PPM ch5  → idx 4
// Naza M V2 SBUS mode (OUT_SBUS): Flight Mode = Naza SBUS ch7 → idx 6
//
// Set to match whichever TX channel your 3-position switch is on.
// The failsafe frame automatically sends RTH on this same channel.
#define CH_FLIGHT_MODE    4    // OUT_PPM Naza: idx4=ch5. OUT_SBUS Naza: set 6 (ch7).

// ── Aux channel assignments (0-based channel index) ──────────
// Change these if you have remapped channels on your TX.
#define CH_PROX_VRB       8    // ch9  — proximity scale / floor set
#define CH_FLOOD          9    // ch10 — floodlight GPIO
#define CH_VTX           10    // ch11 — VTX on/off GPIO
#define CH_NAV_LIGHTS    11    // ch12 — nav LEDs
#define CH_LANDING       12    // ch13 — landing mode flag
#define CH_CEILING_SET   13    // ch14 — ceiling capture/enable

// ── Switch thresholds (PPM µs) ───────────────────────────────
// All switches: above threshold = ON/active.
// RTH_THRESHOLD: flight mode channel below this = RTH/failsafe active.
// Standard Naza RTH position = ~1000µs (full low).
#define RTH_THRESHOLD         1200   // CH_FLIGHT_MODE below this = RTH active (~1000µs)
#define LANDING_THRESHOLD     1400   // ch13 above = landing mode
#define NAV_LIGHTS_THRESHOLD  1400   // ch12 above = nav lights on
#define CEILING_SET_THRESHOLD 1400   // ch14 above = ceiling active
#define AUX_ON_THRESHOLD      1400   // ch10/ch11 above = GPIO HIGH

// ════════════════════════════════════════════════════════════════
//  END USER CONFIGURATION
// ════════════════════════════════════════════════════════════════

// ── Live telemetry values ─────────────────────────────────────
// [v2.2.0-SBUS] The FlySky iBUS telemetry responder is removed in this
// variant (FlySky-specific). These values are still computed by
// task_logic — used by DEBUG_SERIAL output and available for a future
// S.Port / CRSF telemetry add-on.
// All int16_t so a single aligned write is atomic on ESP32-S3 (32-bit bus)
static volatile int16_t g_telZone    = 0;    // 0=clear 1=influence 2=brake 3=fault
static volatile int16_t g_telObsCm   = 999;  // closest obstacle cm (999=clear)
static volatile int16_t g_telHead    = 0;    // threat bearing 0-359 deg (0=no threat)
static volatile int16_t g_telRocCms  = 0;    // vertical velocity cm/s signed
static volatile int16_t g_telFlorCm  = 0;    // floor height cm (0=floor off)
static volatile int16_t g_telAltCm   = 0;    // altitude above ground cm (TF-Luna Down)

// ── Floor config ──────────────────────────────────────────────
#define CFG_FLOOR_BASE_M    0.5f

// ── Active floor config ───────────────────────────────────────
#define CFG_AF_AUTH_CAP       0.25f
#define CFG_AF_ENGAGE_BAND_M  2.0f
#define CFG_AF_KP             0.8f
#define CFG_AF_KD             0.4f
#define CFG_AF_MAX_BANK_DEG   30.0f
#define CFG_AF_VVEL_TARGET_MAX 1.5f

// ── Active horizontal brake config ────────────────────────────
#define CFG_AH_AUTH_CAP    0.30f
#define CFG_AH_VEL_REF_CMS 200.0f
#define CFG_AH_KP          0.30f
#define CFG_AH_KD          0.10f
#define CFG_AH_ENGAGE_CMS  20.0f
#define CFG_AH_INFL_VEL_K  0.5f

// ── VRB freeze hysteresis ─────────────────────────────────────
#define VRB_MOVE_THRESHOLD   40    // PPM counts (~µs) to unfreeze after landing

// ── Sensory Cortex compile flag ───────────────────────────────
#define SENSORY_CORTEX_PRESENT  1  // 0 = standalone PPM pass-through test

// ── BLE UUIDs ─────────────────────────────────────────────────
#define BLE_CC_SERVICE_UUID   "4fafc202-1fb5-459e-8fcc-c5c9c331914b"
#define BLE_CC_STATUS_UUID    "4fafc202-1fb5-459e-8fcc-c5c9c3319111"
#define BLE_CC_ZONE_UUID      "4fafc202-1fb5-459e-8fcc-c5c9c3319112"
#define BLE_CC_WATCHDOG_UUID  "4fafc202-1fb5-459e-8fcc-c5c9c3319113"
#define BLE_CC_SENSORY_UUID   "4fafc202-1fb5-459e-8fcc-c5c9c3319114"
#define BLE_CC_AVOIDANCE_UUID "4fafc202-1fb5-459e-8fcc-c5c9c3319115"
#define BLE_CC_FAILSAFE_UUID  "4fafc202-1fb5-459e-8fcc-c5c9c3319116"
#define BLE_CC_TIMEOUTS_UUID  "4fafc202-1fb5-459e-8fcc-c5c9c3319117"

// ── Packet framing ────────────────────────────────────────────
#define PKT_START  0xAA  // SafetyPacket start byte
#define PKT_CMD    0xCC  // ProximityCmd start byte (different from PKT_START)
#define PKT_END    0x55  // both packet types

// ─────────────────────────────────────────────────────────────
//  Inter-cortex structs — must match the SafetyPacket layout in sensory_cortex
//  (guarded by SAFETY_PKT_VERSION below; bump it in BOTH on any layout change).
//  SAFETY_PKT_VERSION must be bumped in BOTH files on any struct change.
// ─────────────────────────────────────────────────────────────

// SafetyPacket version — must match SAFETY_PKT_VERSION in sensory_cortex.
// Bump both when struct layout changes. Parser rejects mismatched version.
#define SAFETY_PKT_VERSION  0x01

struct __attribute__((packed)) SafetyPacket {
  uint8_t  start;          // 0xAA
  uint8_t  version;        // SAFETY_PKT_VERSION — parser rejects on mismatch
  uint8_t  frontBlocked;
  uint8_t  leftBlocked;
  uint8_t  rightBlocked;
  uint8_t  backBlocked;
  uint8_t  faults;         // bit0=front bit1=left bit2=right bit3=AHRS
                           // bit4=alt bit5=OF bit6=back bit7=baro
  uint8_t  tteFront;       // tenths of second (255=clear)
  uint8_t  tteLeft;
  uint8_t  tteRight;
  uint8_t  tteBack;
  int16_t  velFront;       // cm/s negative=closing
  int16_t  velLeft;
  int16_t  velRight;
  int16_t  velBack;
  uint16_t altCm;          // altitude above ground (cm)
  int16_t  rollDeg;        // roll  ×10 deg
  int16_t  pitchDeg;       // pitch ×10 deg
  int16_t  vertVelCmS;     // vertical velocity cm/s, negative = descending
  int16_t  mapForceN;      // map repulsion FORWARD ×1000 (body frame)
  int16_t  mapForceE;      // map repulsion RIGHT   ×1000 (body frame)
  uint8_t  mapForceValid;  // 1 = dead-reckoning trusted
  uint8_t  checksum;       // XOR of bytes [version..mapForceValid]
  uint8_t  end;            // 0x55
};
static_assert(sizeof(SafetyPacket) == 34, "SafetyPacket size mismatch");

struct __attribute__((packed)) ProximityCmd {
  uint8_t  start;       // PKT_CMD = 0xCC
  float    scale;       // avoidance zone scale factor (0.3–2.0)
  uint8_t  landingMode; // 1 = landing mode active
  uint8_t  checksum;    // XOR of 4 scale bytes + landingMode
  uint8_t  end;         // PKT_END = 0x55
};
static_assert(sizeof(ProximityCmd) == 8, "ProximityCmd size mismatch");

// ─────────────────────────────────────────────────────────────
//  Inter-task data structures
// ─────────────────────────────────────────────────────────────

struct ChannelFrame {
  uint16_t ch[RC_NUM_CH];  // decoded RC channels 0–13, values 1000–2000 µs
  bool     valid;
};

// ─────────────────────────────────────────────────────────────
//  SBUS raw ↔ µs mapping
//  Standard scaling: µs = 0.625×raw + 880 (988–2012 at raw 172–1811),
//  clamped to the 1000–2000µs domain the avoidance logic uses.
//  Integer form avoids float in the RX/TX hot path.
// ─────────────────────────────────────────────────────────────

static inline uint16_t sbus_to_us(uint16_t raw) {
  uint32_t us = 880u + ((uint32_t)raw * 5u + 4u) / 8u;   // round-nearest
  if (us < RC_MIN) us = RC_MIN;
  if (us > RC_MAX) us = RC_MAX;
  return (uint16_t)us;
}

static inline uint16_t us_to_sbus(uint16_t us) {
  int32_t raw = (((int32_t)us - 880) * 8 + 2) / 5;       // round-nearest
  if (raw < SBUS_RAW_MIN) raw = SBUS_RAW_MIN;
  if (raw > SBUS_RAW_MAX) raw = SBUS_RAW_MAX;
  return (uint16_t)raw;
}

// ── Failsafe flight-mode value (µs) — used by BOTH output modes ──
static uint16_t g_failsafeRTH = 1000;   // failsafe value on CH_FLIGHT_MODE slot (RTH=1000); tunable via BLE

#if OUTPUT_PROTOCOL == OUT_PPM
// ─────────────────────────────────────────────────────────────
//  RMT PPM encoder                                  [OUT_PPM]
//  Uses ESP-IDF RMT TX driver (esp32-hal-rmt.h / driver/rmt_tx.h)
//  Generates 8-channel PPM frame autonomously in hardware.
//  task_logic writes to g_ppmShadow[]; ISR swaps into g_ppmActive[].
// ─────────────────────────────────────────────────────────────

static rmt_channel_handle_t g_rmtChan      = nullptr;
static rmt_encoder_handle_t g_rmtEncoder   = nullptr;

// Double buffer: task_logic writes shadow, ISR installs it.
// Memory ordering: __sync_synchronize() (full memory barrier) is used in
// ppm_update/ppm_failsafe AFTER channel writes and BEFORE setting
// g_ppmUpdated, and in the ISR AFTER reading g_ppmUpdated and BEFORE
// reading the shadow values. This prevents CPU store-buffer reordering
// on dual-core Xtensa LX7 where volatile alone is insufficient.
static volatile uint16_t g_ppmActive[PPM_CHANNELS];  // currently transmitting
static volatile uint16_t g_ppmShadow[PPM_CHANNELS];  // next frame from logic
static volatile bool     g_ppmUpdated = false;        // shadow has fresh data
static volatile bool     g_rmtStalled = false;        // set if ISR re-queue fails

// Pre-built RMT symbol array. Each channel = sync_gap LOW + channel_us HIGH.
// Final entry = frame gap HIGH. Total symbols = PPM_CHANNELS*2 + 1.
// rmt_symbol_word_t: { duration0, level0, duration1, level1 }
#define PPM_NUM_SYMBOLS  (PPM_CHANNELS * 2 + 1)
static rmt_symbol_word_t g_rmtSymbols[PPM_NUM_SYMBOLS];

static void IRAM_ATTR ppm_build_symbols(const volatile uint16_t *ch) {
  uint32_t used = 0;
  for (int i = 0; i < PPM_CHANNELS; i++) {
    uint16_t pw = ch[i];
    if (pw < RC_MIN) pw = RC_MIN;
    if (pw > RC_MAX) pw = RC_MAX;
    // Sync gap: LOW for PPM_SYNC_US
    g_rmtSymbols[i*2].duration0 = PPM_SYNC_US;
    g_rmtSymbols[i*2].level0    = 0;
    // Channel pulse: HIGH for (pw - PPM_SYNC_US)
    g_rmtSymbols[i*2].duration1 = pw - PPM_SYNC_US;
    g_rmtSymbols[i*2].level1    = 1;
    used += pw;
  }
  // Frame gap: HIGH for remainder of 20ms frame
  // Add one final sync LOW to close the last channel
  uint32_t gap = PPM_FRAME_US - used - PPM_SYNC_US;
  if (gap < 1000) gap = 1000;  // safety floor
  g_rmtSymbols[PPM_CHANNELS*2].duration0 = PPM_SYNC_US;
  g_rmtSymbols[PPM_CHANNELS*2].level0    = 0;
  g_rmtSymbols[PPM_CHANNELS*2].duration1 = gap;
  g_rmtSymbols[PPM_CHANNELS*2].level1    = 1;
}

// RMT transmit config — used for every rmt_transmit() call.
// eot_level=1 is critical: holds line HIGH after the last symbol completes,
// so the inter-frame idle is HIGH and the next ch1 sync LOW has a clean
// HIGH→LOW edge. With eot_level=0 (default) the RMT drives the line LOW
// immediately after the frame gap, merging with the ch1 sync pulse and
// presenting the Naza with an extended ambiguous LOW with no clean edge.
static const rmt_transmit_config_t g_rmtTxCfg = {
  .loop_count = 0,   // single shot — ISR re-queues each frame
  .flags = {
    .eot_level = 1,  // hold HIGH after frame — correct PPM idle
  }
};

// RMT tx_done callback — runs in ISR context on Core 0.
// Swaps in the shadow buffer if updated, rebuilds symbols, re-queues TX.
static bool IRAM_ATTR ppm_tx_done_cb(rmt_channel_handle_t chan,
                                      const rmt_tx_done_event_data_t *edata,
                                      void *user_ctx) {
  if (g_ppmUpdated) {
    // Memory barrier: ensure all shadow writes from Core 1 are visible
    // before we read g_ppmShadow[]. Pairs with barrier in ppm_update().
    __sync_synchronize();
    for (int i = 0; i < PPM_CHANNELS; i++) g_ppmActive[i] = g_ppmShadow[i];
    g_ppmUpdated = false;
  }
  ppm_build_symbols(g_ppmActive);
  // Re-queue transmission — non-blocking at queue_depth=2, eot_level=1 holds HIGH.
  // Check return value: a failure means PPM output stops. The hardware watchdog
  // (GPIO5 → CD4053) will engage within ~100ms, routing raw PPM to the Naza.
  // g_rmtStalled is checked by task_protocol and logged via USBSerial.
  esp_err_t err = rmt_transmit(chan, g_rmtEncoder, g_rmtSymbols,
                                sizeof(g_rmtSymbols), &g_rmtTxCfg);
  if (err != ESP_OK) g_rmtStalled = true;
  return false;  // no higher-priority task woken
}

// Initialise RMT channel and start continuous PPM output.
static void ppm_rmt_init() {
  // Initialise active buffer to failsafe values
  for (int i = 0; i < PPM_CHANNELS; i++) {
    g_ppmActive[i] = FAILSAFE_PPM[i];
    g_ppmShadow[i] = FAILSAFE_PPM[i];
  }

  // RMT TX channel config
  rmt_tx_channel_config_t tx_cfg = {};
  tx_cfg.gpio_num            = (gpio_num_t)PIN_RC_OUT;
  tx_cfg.clk_src             = RMT_CLK_SRC_DEFAULT;
  tx_cfg.resolution_hz       = PPM_RMT_TICK_HZ;  // 1MHz = 1µs per tick
  tx_cfg.mem_block_symbols   = 64;               // ≥ PPM_NUM_SYMBOLS (17)
  tx_cfg.trans_queue_depth   = 2;  // depth=2: ISR re-queue never blocks if prev frame still draining
  tx_cfg.flags.invert_out    = false;            // PPM idle HIGH, pulse LOW — correct
  tx_cfg.flags.with_dma      = false;
  ESP_ERROR_CHECK(rmt_new_tx_channel(&tx_cfg, &g_rmtChan));

  // Simple copy encoder — we manage the symbol array ourselves
  rmt_copy_encoder_config_t enc_cfg = {};
  ESP_ERROR_CHECK(rmt_new_copy_encoder(&enc_cfg, &g_rmtEncoder));

  // Register tx_done callback (ISR, Core 0)
  rmt_tx_event_callbacks_t cbs = {};
  cbs.on_trans_done = ppm_tx_done_cb;
  ESP_ERROR_CHECK(rmt_tx_register_event_callbacks(g_rmtChan, &cbs, nullptr));

  // Enable channel and start first frame
  ESP_ERROR_CHECK(rmt_enable(g_rmtChan));
  ppm_build_symbols(g_ppmActive);
  rmt_transmit(g_rmtChan, g_rmtEncoder, g_rmtSymbols,
               sizeof(g_rmtSymbols), &g_rmtTxCfg);
}

// Called by task_logic to push 8 new channel values.
// __sync_synchronize() (full memory barrier) ensures all channel writes
// are visible to Core 0 before g_ppmUpdated is set. Without this, the
// Xtensa LX7 store buffer could reorder writes across cores.
static inline void ppm_update(const uint16_t *ch8) {
  for (int i = 0; i < PPM_CHANNELS; i++) g_ppmShadow[i] = ch8[i];
  __sync_synchronize();   // drain store buffer before signalling ISR
  g_ppmUpdated = true;
}

// Push failsafe frame — applies g_failsafeRTH on CH_FLIGHT_MODE slot.
// Uses CH_FLIGHT_MODE as the index so it is always consistent with the
// user config block. All other slots use FAILSAFE_PPM defaults.
static inline void ppm_failsafe() {
  for (int i = 0; i < PPM_CHANNELS; i++) g_ppmShadow[i] = FAILSAFE_PPM[i];
  if (CH_FLIGHT_MODE < PPM_CHANNELS)
    g_ppmShadow[CH_FLIGHT_MODE] = g_failsafeRTH;  // RTH on correct flight mode slot
  __sync_synchronize();   // drain store buffer before signalling ISR
  g_ppmUpdated = true;
}

// Unified output API — task_logic / watchdog call these regardless of mode
static inline void rc_out_update(const uint16_t *ch)  { ppm_update(ch); }
static inline void rc_out_failsafe()                  { ppm_failsafe(); }

#else  // OUTPUT_PROTOCOL == OUT_SBUS
// ─────────────────────────────────────────────────────────────
//  SBUS output encoder                              [OUT_SBUS]
//  UART2 TX on GPIO6, 100000 8E2, software-inverted (idle LOW on
//  the wire — standard SBUS polarity). One 25-byte frame is sent
//  every SBUS_OUT_PERIOD_MS from task_protocol (Core 0).
//
//  25 bytes × 12 bits = 3ms of wire time per 14ms frame. The
//  ESP32-S3 UART TX FIFO is 128 bytes, so Serial2.write(25) never
//  blocks: the whole frame is queued in one call and drains in
//  hardware while the task continues.
//
//  Same double-buffer + memory-barrier discipline as the PPM path:
//  task_logic (Core 1) writes the shadow, task_protocol (Core 0)
//  latches it before each frame.
// ─────────────────────────────────────────────────────────────

static volatile uint16_t g_sbusShadow[RC_NUM_CH];   // µs, from task_logic
static volatile bool     g_sbusUpdated = false;
static uint16_t          g_sbusActive[RC_NUM_CH];   // µs, task_protocol-local

static inline void rc_out_update(const uint16_t *ch) {
  for (int i = 0; i < RC_NUM_CH; i++) g_sbusShadow[i] = ch[i];
  __sync_synchronize();   // drain store buffer before signalling Core 0
  g_sbusUpdated = true;
}

static inline void rc_out_failsafe() {
  // First 8 slots from the shared failsafe table, remainder neutral
  for (int i = 0; i < RC_NUM_CH; i++)
    g_sbusShadow[i] = (i < PPM_CHANNELS) ? FAILSAFE_PPM[i] : RC_MID;
  if (CH_FLIGHT_MODE < RC_NUM_CH)
    g_sbusShadow[CH_FLIGHT_MODE] = g_failsafeRTH;  // RTH on flight mode slot
  __sync_synchronize();
  g_sbusUpdated = true;
}

// Build one 25-byte SBUS frame from 14 µs channel values.
// SBUS ch15/16 transmitted at neutral; flags = 0 (we ARE the good link —
// the RC-loss condition is expressed through channel VALUES, exactly as
// the PPM variant does, so the FC's own failsafe is never triggered by us).
static void sbus_out_build(uint8_t *b, const uint16_t *chUs) {
  uint16_t raw[16];
  for (int i = 0; i < 16; i++)
    raw[i] = us_to_sbus(i < RC_NUM_CH ? chUs[i] : (uint16_t)RC_MID);

  b[0] = SBUS_HEADER;
  uint32_t bitbuf = 0;
  uint8_t  bits   = 0;
  uint8_t  idx    = 1;
  for (int i = 0; i < 16; i++) {
    bitbuf |= ((uint32_t)raw[i]) << bits;
    bits   += 11;
    while (bits >= 8) { b[idx++] = bitbuf & 0xFF; bitbuf >>= 8; bits -= 8; }
  }
  // 16×11 = 176 bits = exactly 22 bytes → idx is now 23, bits == 0
  b[23] = 0x00;        // flags: no framelost, no failsafe, no digital ch
  b[24] = SBUS_END_1;  // SBUS1 end byte
}

// Called from task_protocol every loop; sends at SBUS_OUT_PERIOD_MS.
static void sbus_out_tick() {
  static uint32_t lastTxMs = 0;
  static bool     seeded   = false;
  if (!seeded) {          // pre-seed with failsafe values before first RC frame
    for (int i = 0; i < RC_NUM_CH; i++)
      g_sbusActive[i] = (i < PPM_CHANNELS) ? FAILSAFE_PPM[i] : RC_MID;
    if (CH_FLIGHT_MODE < RC_NUM_CH) g_sbusActive[CH_FLIGHT_MODE] = g_failsafeRTH;
    seeded = true;
  }
  uint32_t now = millis();
  if (now - lastTxMs < SBUS_OUT_PERIOD_MS) return;
  lastTxMs = now;

  if (g_sbusUpdated) {
    __sync_synchronize();  // pair with barrier in rc_out_update/failsafe
    for (int i = 0; i < RC_NUM_CH; i++) g_sbusActive[i] = g_sbusShadow[i];
    g_sbusUpdated = false;
  }
  uint8_t frame[SBUS_FRAME_LEN];
  sbus_out_build(frame, g_sbusActive);
  Serial2.write(frame, SBUS_FRAME_LEN);   // ≤128B FIFO — non-blocking
}
#endif  // OUTPUT_PROTOCOL

// ─────────────────────────────────────────────────────────────
//  SBUS decoder
//  100000 8E2 inverted (inversion handled by UART peripheral).
//  Validates header, end byte (SBUS1 0x00 / SBUS2 slot bytes),
//  and the FAILSAFE flag. NO checksum exists in SBUS — the 8E2
//  parity bit plus header/end validation is all the protocol gives;
//  the inter-frame-gap sync in task_protocol prevents phase-lock
//  on payload bytes that happen to equal 0x0F.
//  Returns false on failsafe-flagged frames so the RC-loss watchdog
//  (g_wdgRcMs) engages exactly as it does when frames stop entirely.
// ─────────────────────────────────────────────────────────────

static bool sbus_decode(const uint8_t *buf, uint16_t *ch) {
  if (buf[0] != SBUS_HEADER) return false;
  const uint8_t end = buf[24];
  if (!(end == SBUS_END_1 || (end & 0xCF) == 0x04)) return false;
  if (buf[23] & SBUS_FLAG_FAILSAFE) return false;   // receiver lost TX
  // buf[23] & SBUS_FLAG_FRAMELOST: informational — data still last-known-good

  // Unpack 16 × 11-bit little-endian channels from bytes 1..22
  uint16_t raw[16];
  uint32_t bitbuf = 0;
  uint8_t  bits   = 0;
  uint8_t  idx    = 1;
  for (int i = 0; i < 16; i++) {
    while (bits < 11) { bitbuf |= ((uint32_t)buf[idx++]) << bits; bits += 8; }
    raw[i]  = bitbuf & 0x07FF;
    bitbuf >>= 11;
    bits    -= 11;
  }
  // First RC_NUM_CH channels used; SBUS ch15/16 + digital ch17/18 ignored
  for (int i = 0; i < RC_NUM_CH; i++) ch[i] = sbus_to_us(raw[i]);
  return true;
}

// ─────────────────────────────────────────────────────────────
//  SafetyPacket parser — ring-buffer state machine
//  Validates: end marker, version byte, XOR checksum.
//  Any field corruption causes silent discard and resync.
// ─────────────────────────────────────────────────────────────

static bool parse_safety_stream(HardwareSerial &ser, SafetyPacket &out) {
  static uint8_t buf[sizeof(SafetyPacket)];
  static uint8_t pos    = 0;
  static bool    synced = false;

  while (ser.available()) {
    uint8_t b = ser.read();
    if (!synced) {
      if (b == PKT_START) { buf[0] = b; pos = 1; synced = true; }
      continue;
    }
    buf[pos++] = b;
    if (pos == sizeof(SafetyPacket)) {
      synced = false; pos = 0;

      // End marker
      if (buf[sizeof(SafetyPacket)-1] != PKT_END) continue;

      // Version — reject packets from mismatched Sensory Cortex build
      if (buf[1] != SAFETY_PKT_VERSION) {
        USBSerial.printf("[WARN] SafetyPacket version mismatch: "
                         "got 0x%02X expected 0x%02X\n",
                         buf[1], SAFETY_PKT_VERSION);
        continue;
      }

      // XOR checksum over bytes [version..mapForceValid]
      // Matches sensory_cortex: i = 1 to offsetof(checksum)-1
      uint8_t cs = 0;
      for (size_t i = 1; i < offsetof(SafetyPacket, checksum); i++) cs ^= buf[i];
      if (cs != buf[offsetof(SafetyPacket, checksum)]) continue;

      memcpy(&out, buf, sizeof(SafetyPacket));
      return true;
    }
  }
  return false;
}

// ─────────────────────────────────────────────────────────────
//  ProximityCmd builder (unchanged)
// ─────────────────────────────────────────────────────────────

static ProximityCmd build_prox_cmd(float scale, bool landing) {
  ProximityCmd p;
  p.start       = PKT_CMD;
  p.scale       = scale;
  p.landingMode = landing ? 1 : 0;
  // XOR checksum over 4 scale bytes + landingMode — matches Sensory parser
  const uint8_t *pb = (const uint8_t *)&p;
  uint8_t cs = 0;
  for (size_t i = 1; i < offsetof(ProximityCmd, checksum); i++) cs ^= pb[i];
  p.checksum = cs;
  p.end       = PKT_END;
  return p;
}

// ─────────────────────────────────────────────────────────────
//  Avoidance helpers — all arithmetic now in PPM µs (1000–2000)
// ─────────────────────────────────────────────────────────────

static inline float fmap(float v, float in_lo, float in_hi,
                          float out_lo, float out_hi) {
  v = constrain(v, in_lo, in_hi);
  return out_lo + (v - in_lo) * (out_hi - out_lo) / (in_hi - in_lo);
}

// Returns true if stick is deflected past deadband in the closing direction.
static inline bool is_closing(uint16_t ch_val, bool positive_is_closing) {
  if (positive_is_closing) return ch_val > (RC_MID + RC_DEAD);
  else                      return ch_val < (RC_MID - RC_DEAD);
}

// Scale stick deflection from neutral by authority (0–1).
static inline uint16_t apply_authority(uint16_t ch_val, float authority) {
  float delta = (float)ch_val - RC_MID;
  return (uint16_t)(RC_MID + delta * authority);
}

// ─────────────────────────────────────────────────────────────
//  Runtime-tunable gains (BLE-writable)
// ─────────────────────────────────────────────────────────────

static uint16_t g_wdgSensorMs     = WDG_SENSOR_MS;
static uint16_t g_wdgRcMs       = WDG_RC_MS;
static bool     g_avoidEnabled      = true;
static bool     g_horizAvoidEnabled = true;
static bool     g_altCorridorEnabled = true;
static bool     g_mapAvoidEnabled   = true;
static float    g_floorBaseM        = CFG_FLOOR_BASE_M;
static float    g_rememberedFloorM  = 0.0f;
static bool     g_floorActive       = false;
// [v2.1.7 I-006] Default ARMED at boot — no BLE tap needed every flight.
// Landing mode already gates this off via the `!landing` term in activeOk
// below, so "armed whenever not landing" falls out naturally from just
// defaulting true here. `afloor disarm` over BLE still works as a manual
// override if you ever want it off entirely (e.g. bench testing).
static bool     g_activeFloorArmed  = true;
static bool     g_activeFloorActive = false;
static float    g_afAuthCap         = CFG_AF_AUTH_CAP;
static float    g_afEngageBandM     = CFG_AF_ENGAGE_BAND_M;
static float    g_afKp              = CFG_AF_KP;
static float    g_afKd              = CFG_AF_KD;
static float    g_afMaxBankDeg      = CFG_AF_MAX_BANK_DEG;
static float    g_afLastAddFrac     = 0.0f;
static bool     g_activeHorizArmed  = false;
static bool     g_activeHorizActive = false;
static float    g_ahAuthCap         = CFG_AH_AUTH_CAP;
static float    g_ahKp              = CFG_AH_KP;
static float    g_ahKd              = CFG_AH_KD;
static float    g_ahVelRefCms       = CFG_AH_VEL_REF_CMS;
static float    g_ahEngageCms       = CFG_AH_ENGAGE_CMS;
static float    g_ahPrevClose[4]    = {0,0,0,0};
static uint32_t g_ahPrevMs          = 0;
static float    g_ceilingM          = 0.0f;
static bool     g_ceilingActive     = false;
static bool     g_prevCeilingSet    = false;
static float    g_rememberedScaleH  = 1.0f;
static bool     g_prevLanding       = true;
static bool     g_hScaleFrozen      = false;
static uint16_t g_freezeVrbRef      = RC_MID;
static bool     g_sensoryPresent    = (SENSORY_CORTEX_PRESENT != 0);
static uint32_t g_wdPulseCount      = 0;
static uint32_t g_wdLastCheckMs     = 0;
static uint32_t g_wdLastCount       = 0;
static bool     g_wdOk              = false;

// ─────────────────────────────────────────────────────────────
//  Persistent storage (NVS flash via Preferences)             [v2.1.8 I-007]
// ─────────────────────────────────────────────────────────────
// All BLE-writable tunings/toggles, plus the two RC-knob "remembered"
// values (floor height, horizontal avoidance scale), survive a power cycle.
// BLE writes are inherently low-frequency (a person tapping an app), so
// saving on every valid BLE write is no flash-wear concern — mirrors the
// Sensory Cortex's existing prefs_save() pattern.
//
// The two RC-derived "remembered" values are NOT written on every loop —
// they change continuously while flying/landing, and flash-writing every
// loop would both wear the NVS partition and risk blocking task_logic's
// real-time path. Instead they're persisted only at the natural, low-
// frequency checkpoints where they stop changing: landing-mode entry
// (snapshots the flight scale before it freezes) and landing-mode exit
// (snapshots the floor height dialed in during that landing-mode session).
// See the edge-detection block in task_logic().
//
// NOT persisted, by design: g_activeFloorArmed. It defaults true at boot
// (v2.1.7 fix) so active floor assist works every flight without a BLE
// tap — if we persisted a manual `afloor disarm`, that default would stop
// meaning anything after the first disarm. `afloor arm`/`disarm` still
// work live, just don't survive reboot. g_activeHorizArmed IS persisted
// below, since it has no such default-on requirement.
static Preferences g_prefs;
#define PREFS_NAMESPACE "commcc"

static void prefs_save() {
  g_prefs.begin(PREFS_NAMESPACE, false);
  g_prefs.putBool ("avoidEn",    g_avoidEnabled);
  g_prefs.putBool ("horizEn",    g_horizAvoidEnabled);
  g_prefs.putBool ("altEn",      g_altCorridorEnabled);
  g_prefs.putBool ("mapEn",      g_mapAvoidEnabled);
  g_prefs.putBool ("sensPres",   g_sensoryPresent);
  g_prefs.putFloat("afCap",      g_afAuthCap);
  g_prefs.putFloat("afBand",     g_afEngageBandM);
  g_prefs.putFloat("afKp",       g_afKp);
  g_prefs.putFloat("afKd",       g_afKd);
  g_prefs.putFloat("afBank",     g_afMaxBankDeg);
  g_prefs.putBool ("ahArmed",    g_activeHorizArmed);
  g_prefs.putFloat("ahCap",      g_ahAuthCap);
  g_prefs.putFloat("ahKp",       g_ahKp);
  g_prefs.putFloat("ahKd",       g_ahKd);
  g_prefs.putFloat("ahVref",     g_ahVelRefCms);
  g_prefs.putFloat("ahEngage",   g_ahEngageCms);
  g_prefs.putUShort("failsafe",  g_failsafeRTH);
  g_prefs.putUShort("wdgSense",  g_wdgSensorMs);
  g_prefs.putUShort("wdgRc",     g_wdgRcMs);
  g_prefs.end();
}

// Separate from prefs_save() so the two RC-derived values can be persisted
// at their own edge-triggered checkpoints without re-writing every other
// key on the same call (keeps each NVS commit small and infrequent).
static void prefs_save_floor() {
  g_prefs.begin(PREFS_NAMESPACE, false);
  g_prefs.putFloat("floorM",     g_rememberedFloorM);
  g_prefs.end();
}
static void prefs_save_scaleH() {
  g_prefs.begin(PREFS_NAMESPACE, false);
  g_prefs.putFloat("scaleH",     g_rememberedScaleH);
  g_prefs.end();
}

static void prefs_load() {
  g_prefs.begin(PREFS_NAMESPACE, true);   // read-only
  g_avoidEnabled       = g_prefs.getBool ("avoidEn",  g_avoidEnabled);
  g_horizAvoidEnabled  = g_prefs.getBool ("horizEn",  g_horizAvoidEnabled);
  g_altCorridorEnabled = g_prefs.getBool ("altEn",    g_altCorridorEnabled);
  g_mapAvoidEnabled    = g_prefs.getBool ("mapEn",    g_mapAvoidEnabled);
  g_sensoryPresent     = g_prefs.getBool ("sensPres", g_sensoryPresent);
  g_afAuthCap          = g_prefs.getFloat("afCap",    g_afAuthCap);
  g_afEngageBandM      = g_prefs.getFloat("afBand",   g_afEngageBandM);
  g_afKp               = g_prefs.getFloat("afKp",     g_afKp);
  g_afKd               = g_prefs.getFloat("afKd",     g_afKd);
  g_afMaxBankDeg       = g_prefs.getFloat("afBank",   g_afMaxBankDeg);
  g_activeHorizArmed   = g_prefs.getBool ("ahArmed",  g_activeHorizArmed);
  g_ahAuthCap          = g_prefs.getFloat("ahCap",    g_ahAuthCap);
  g_ahKp               = g_prefs.getFloat("ahKp",     g_ahKp);
  g_ahKd               = g_prefs.getFloat("ahKd",     g_ahKd);
  g_ahVelRefCms        = g_prefs.getFloat("ahVref",   g_ahVelRefCms);
  g_ahEngageCms        = g_prefs.getFloat("ahEngage", g_ahEngageCms);
  g_failsafeRTH        = g_prefs.getUShort("failsafe", g_failsafeRTH);
  g_wdgSensorMs        = g_prefs.getUShort("wdgSense", g_wdgSensorMs);
  g_wdgRcMs          = g_prefs.getUShort("wdgRc",    g_wdgRcMs);
  g_rememberedFloorM   = g_prefs.getFloat("floorM",   g_rememberedFloorM);
  g_rememberedScaleH   = g_prefs.getFloat("scaleH",   g_rememberedScaleH);
  g_prefs.end();
}

// ── Shared state between tasks (written Core0, read Core1) ────
static volatile bool     rcLost      = false;
// [v2.1.6 I-004] Boot in the SAFE state: assume NO valid sensory data until a
// packet actually arrives (first valid packet clears this at RX). Previously
// this booted false, so there was a window at startup where the all-clear
// default packet was treated as real "clear airspace".
static volatile bool     sensorDead  = true;
static volatile uint32_t lastRcRxMs   = 0;
static volatile uint32_t lastSafetyPktMs = 0;
// [v2.1.6 I-005] Consecutive good packets seen while sensorDead — avoidance
// authority is only restored after SAFETY_RECOVER_PKTS in a row, so a flapping
// UART link can't snap avoidance back on/off. Touched only in task_protocol.
static uint16_t g_safetyRecoverCnt = 0;
#define SAFETY_RECOVER_PKTS 5   // ~35-100ms of stable link before restoring authority

// ─────────────────────────────────────────────────────────────
//  FreeRTOS queues
// ─────────────────────────────────────────────────────────────

static QueueHandle_t qRxChannels;   // Core0 → Core1: decoded RC channels (µs)
static QueueHandle_t qSafetyPkt;    // Core0 → Core1: latest SafetyPacket
static QueueHandle_t qProxCmd;      // Core1 → Core0: ProximityCmd to send

// ─────────────────────────────────────────────────────────────
//  WS2812 status LED
// ─────────────────────────────────────────────────────────────

#define NUM_LEDS 1
static CRGB leds[NUM_LEDS];

// ─────────────────────────────────────────────────────────────
//  Active-away push helper (horizontal brake, v1.9.1/v1.9.2)
//  Unchanged from v1.9.2 — just replaces SBUS_MID/MIN/MAX with PPM
// ─────────────────────────────────────────────────────────────

static inline float vel_damped_authority(float base_auth, float closingCmS) {
  float velFrac = closingCmS / g_ahVelRefCms;
  if (velFrac > 1.0f) velFrac = 1.0f;
  float reduced = base_auth * (1.0f - CFG_AH_INFL_VEL_K * velFrac);
  if (reduced < AUTHORITY_MIN) reduced = AUTHORITY_MIN;
  return reduced;
}

static inline uint16_t apply_active_away(uint16_t ch_val, float closingCmS,
                                         bool away_is_positive, int axis,
                                         float dt_s) {
  if (closingCmS <= g_ahEngageCms) {
    if (axis >= 0 && axis < 4) g_ahPrevClose[axis] = closingCmS;
    return ch_val;
  }
  float norm = closingCmS / g_ahVelRefCms;
  if (norm > 1.0f) norm = 1.0f;
  float pTerm = g_ahKp * norm;

  float dTerm = 0.0f;
  if (axis >= 0 && axis < 4 && dt_s > 0.0001f) {
    float dClose = (closingCmS - g_ahPrevClose[axis]) / dt_s;
    float dNorm  = dClose / g_ahVelRefCms;
    dTerm = g_ahKd * dNorm;
    if (dTerm < 0.0f) dTerm = 0.0f;
  }
  if (axis >= 0 && axis < 4) g_ahPrevClose[axis] = closingCmS;

  float push = pTerm + dTerm;
  if (push > g_ahAuthCap) push = g_ahAuthCap;
  if (push < 0.0f) push = 0.0f;

  uint16_t pushUs = (uint16_t)(push * (RC_MAX - RC_MIN));
  if (away_is_positive) {
    uint32_t target = (uint32_t)RC_MID + pushUs;
    if (target > RC_MAX) target = RC_MAX;
    return (uint16_t)target;
  } else {
    int32_t target = (int32_t)RC_MID - (int32_t)pushUs;
    if (target < RC_MIN) target = RC_MIN;
    return (uint16_t)target;
  }
}

// ─────────────────────────────────────────────────────────────
//  Horizontal avoidance (unchanged logic, PPM µs values)
// ─────────────────────────────────────────────────────────────

static void apply_horizontal_avoidance(uint16_t *ch, const SafetyPacket &sp,
                                        float scale, bool landing) {
  bool anyBrake     = false;
  bool anyInfluence = false;
  bool pitchLive    = false;
  bool rollLive     = false;
  g_activeHorizActive = false;

  uint32_t nowMs = millis();
  float ah_dt = (g_ahPrevMs == 0) ? 0.0f : (nowMs - g_ahPrevMs) * 0.001f;
  g_ahPrevMs = nowMs;
  if (ah_dt > 0.2f) ah_dt = 0.0f;

  // ── Front → ch1 (pitch), positive = forward = closing ──
  {
    bool  fault   = (sp.faults & 0x01);
    float tte_s   = sp.tteFront / 10.0f;
    bool  closing = is_closing(ch[1], true);
    if (!fault && closing) {
      if (sp.frontBlocked || tte_s < TTE_BRAKE_S) {
        ch[1] = RC_MID; anyBrake = true; pitchLive = true;
        if (g_activeHorizArmed && !landing && sp.frontBlocked && sp.velFront < 0) {
          ch[1] = apply_active_away(ch[1], (float)(-sp.velFront), false, 0, ah_dt);
          g_activeHorizActive = true;
        }
      } else if (tte_s < TTE_WARN_S) {
        float authority = fmap(tte_s, TTE_BRAKE_S, TTE_WARN_S, AUTHORITY_MIN, 1.0f);
        if (sp.velFront < 0) authority = vel_damped_authority(authority, (float)(-sp.velFront));
        ch[1] = apply_authority(ch[1], authority);
        anyInfluence = true; pitchLive = true;
      }
    }
  }

  // ── Left → ch0 (roll), negative = left = closing ──
  {
    bool  fault   = (sp.faults & 0x02);
    float tte_s   = sp.tteLeft / 10.0f;
    bool  closing = is_closing(ch[0], false);
    if (!fault && closing) {
      if (sp.leftBlocked || tte_s < TTE_BRAKE_S) {
        ch[0] = RC_MID; anyBrake = true; rollLive = true;
        if (g_activeHorizArmed && !landing && sp.leftBlocked && sp.velLeft < 0) {
          ch[0] = apply_active_away(ch[0], (float)(-sp.velLeft), true, 1, ah_dt);
          g_activeHorizActive = true;
        }
      } else if (tte_s < TTE_WARN_S) {
        float authority = fmap(tte_s, TTE_BRAKE_S, TTE_WARN_S, AUTHORITY_MIN, 1.0f);
        if (sp.velLeft < 0) authority = vel_damped_authority(authority, (float)(-sp.velLeft));
        ch[0] = apply_authority(ch[0], authority);
        anyInfluence = true; rollLive = true;
      }
    }
  }

  // ── Right → ch0 (roll), positive = right = closing ──
  {
    bool  fault   = (sp.faults & 0x04);
    float tte_s   = sp.tteRight / 10.0f;
    bool  closing = is_closing(ch[0], true);
    if (!fault && closing) {
      if (sp.rightBlocked || tte_s < TTE_BRAKE_S) {
        if (!rollLive) {
          ch[0] = RC_MID; anyBrake = true;
          // Active push: right obstacle → push LEFT (roll −, away_is_positive=false).
          if (g_activeHorizArmed && !landing && sp.rightBlocked && sp.velRight < 0) {
            ch[0] = apply_active_away(ch[0], (float)(-sp.velRight), false, 2, ah_dt);
            g_activeHorizActive = true;
          }
          rollLive = true;
        }
      } else if (tte_s < TTE_WARN_S && !rollLive) {
        float authority = fmap(tte_s, TTE_BRAKE_S, TTE_WARN_S, AUTHORITY_MIN, 1.0f);
        if (sp.velRight < 0) authority = vel_damped_authority(authority, (float)(-sp.velRight));
        ch[0] = apply_authority(ch[0], authority);
        anyInfluence = true;
      }
    }
  }

  // ── Back → ch1 (pitch), negative = backward = closing ──
  {
    bool  fault   = (sp.faults & 0x40);
    float tte_s   = sp.tteBack / 10.0f;
    bool  closing = is_closing(ch[1], false);
    if (!fault && closing) {
      if (sp.backBlocked || tte_s < TTE_BRAKE_S) {
        if (!pitchLive) {
          ch[1] = RC_MID; anyBrake = true;
          // Active push: back obstacle → push FORWARD (pitch +, away_is_positive=true).
          if (g_activeHorizArmed && !landing && sp.backBlocked && sp.velBack < 0) {
            ch[1] = apply_active_away(ch[1], (float)(-sp.velBack), true, 3, ah_dt);
            g_activeHorizActive = true;
          }
          pitchLive = true;
        }
      } else if (tte_s < TTE_WARN_S && !pitchLive) {
        float authority = fmap(tte_s, TTE_BRAKE_S, TTE_WARN_S, AUTHORITY_MIN, 1.0f);
        if (sp.velBack < 0) authority = vel_damped_authority(authority, (float)(-sp.velBack));
        ch[1] = apply_authority(ch[1], authority);
        anyInfluence = true;
      }
    }
  }

  // ── Map-based secondary nudge (v1.3) ──
  if (g_mapAvoidEnabled && sp.mapForceValid && !landing) {
    float fN = sp.mapForceN * 0.001f;
    float fE = sp.mapForceE * 0.001f;
    if (fabsf(fN) > MAP_FORCE_DEADBAND && !pitchLive) {
      float cut = fabsf(fN);
      if (cut > MAP_NUDGE_MAX) cut = MAP_NUDGE_MAX;
      ch[1] = apply_authority(ch[1], 1.0f - cut);
    }
    if (fabsf(fE) > MAP_FORCE_DEADBAND && !rollLive) {
      float cut = fabsf(fE);
      if (cut > MAP_NUDGE_MAX) cut = MAP_NUDGE_MAX;
      ch[0] = apply_authority(ch[0], 1.0f - cut);
    }
  }

  // ── Throttle: prevent climb in brake zone ──
  if (anyBrake && ch[2] > RC_MID) ch[2] = RC_MID;

  (void)anyInfluence;
}

// ─────────────────────────────────────────────────────────────
//  Altitude corridor — floor + ceiling (v1.7/v1.9, PPM µs)
// ─────────────────────────────────────────────────────────────

static void apply_altitude_corridor(uint16_t *ch, const SafetyPacket &sp,
                                    bool landing) {
  float altM = sp.altCm * 0.01f;

  // ── Floor ──
  {
    bool floorEnforced = false;
    if (landing) {
      float vrbScale = fmap((float)ch[CH_PROX_VRB],
                            RC_MIN, RC_MAX, SCALE_MIN, SCALE_MAX);
      g_rememberedFloorM = (vrbScale <= (SCALE_MIN + 0.05f))
                           ? 0.0f : vrbScale * g_floorBaseM;
    }
    float floor = g_rememberedFloorM;

    float rollD  = sp.rollDeg  * 0.1f;
    float pitchD = sp.pitchDeg * 0.1f;
    float tiltDeg = sqrtf(rollD*rollD + pitchD*pitchD);
    float vVel   = sp.vertVelCmS * 0.01f;

    bool ahrsOk    = !(sp.faults & 0x08);
    bool altOk     = !(sp.faults & 0x10);
    bool altBandOk = (altM > 0.0f && altM < 8.0f);
    float cosT = cosf(tiltDeg * 0.01745329f);
    if (cosT < 0.5f) cosT = 0.5f;
    float trueHeight = altM * cosT;

    g_activeFloorActive = false;

    bool activeOk = g_activeFloorArmed && !landing && floor > 0.05f
                    && altBandOk && altOk && ahrsOk
                    && (tiltDeg <= g_afMaxBankDeg);

    if (activeOk && trueHeight < (floor + g_afEngageBandM)) {
      float distAboveFloor = trueHeight - floor;
      float frac = distAboveFloor / g_afEngageBandM;
      frac = constrain(frac, 0.0f, 1.0f);
      float targetVVel = CFG_AF_VVEL_TARGET_MAX * frac;
      float measDescent = (vVel < 0.0f) ? -vVel : 0.0f;
      float rateErr = measDescent - targetVVel;
      if (rateErr > 0.0f) {
        float add = g_afKp * rateErr + g_afKd * measDescent;
        if (add > g_afAuthCap) add = g_afAuthCap;
        if (add < 0.0f) add = 0.0f;
        g_afLastAddFrac = add;
        uint16_t addUs = (uint16_t)(add * (RC_MAX - RC_MIN));
        uint32_t newThr = (uint32_t)ch[2] + addUs;
        if (newThr > RC_MAX) newThr = RC_MAX;
        if ((uint16_t)newThr > ch[2]) {
          ch[2] = (uint16_t)newThr;
          floorEnforced = true;
          g_activeFloorActive = true;
        }
      } else {
        g_afLastAddFrac = 0.0f;
      }
      // Passive backstop
      if (trueHeight < floor
          && ch[2] < (uint16_t)(RC_MID - RC_DEAD)
          && ch[2] < RC_MID) {
        ch[2] = RC_MID; floorEnforced = true;
      }
    } else {
      g_afLastAddFrac = 0.0f;
      bool descending = (ch[2] < (uint16_t)(RC_MID - RC_DEAD));
      if (floor > 0.05f && altM > 0.0f && altM < 8.0f
          && trueHeight < floor && descending) {
        ch[2] = RC_MID; floorEnforced = true;
      }
    }
    g_floorActive = floorEnforced;
  }

  // ── Ceiling ──
  {
    bool setSwitchHigh = (ch[CH_CEILING_SET] > CEILING_SET_THRESHOLD);
    bool ceilingEnforced = false;
    bool altValid  = !(sp.faults & 0x10);
    bool altBandOk = (altM > 0.0f && altM < 8.0f);
    float rollD  = sp.rollDeg  * 0.1f;
    float pitchD = sp.pitchDeg * 0.1f;
    bool level = (sqrtf(rollD*rollD + pitchD*pitchD) < 15.0f);
    bool climbing = (ch[2] > (uint16_t)(RC_MID + RC_DEAD));

    bool captureEdge = setSwitchHigh && !g_prevCeilingSet;
    if (captureEdge && altValid && level) g_ceilingM = altM;

    if (setSwitchHigh && g_ceilingM > 0.05f) {
      if (altValid && altBandOk && altM > g_ceilingM && climbing) {
        ch[2] = RC_MID;
        ceilingEnforced = true;
      }
    }
    g_ceilingActive  = ceilingEnforced;
    g_prevCeilingSet = setSwitchHigh;
  }
}

// ─────────────────────────────────────────────────────────────
//  Navigation LED state machine (unchanged logic)
// ─────────────────────────────────────────────────────────────

#define NAV_FL 0x01
#define NAV_FR 0x02
#define NAV_RL 0x04
#define NAV_RR 0x08

static void update_nav_leds(uint32_t now, bool navOn,
                             bool frontThreat, bool leftThreat,
                             bool rightThreat, bool backThreat,
                             bool anyBrake, bool sensorOk) {
  uint8_t corners = 0;

  if (!navOn) {
    digitalWrite(PIN_NAV_FL, NAV_LED_OFF);
    digitalWrite(PIN_NAV_FR, NAV_LED_OFF);
    digitalWrite(PIN_NAV_RL, NAV_LED_OFF);
    digitalWrite(PIN_NAV_RR, NAV_LED_OFF);
    return;
  }

  bool anyThreat = frontThreat || leftThreat || rightThreat || backThreat;

  if (anyBrake) {
    // Mode 4 — brake: threatened corners solid
    if (frontThreat) corners |= NAV_FL | NAV_FR;
    if (leftThreat)  corners |= NAV_FL | NAV_RL;
    if (rightThreat) corners |= NAV_FR | NAV_RR;
    if (backThreat)  corners |= NAV_RL | NAV_RR;
  } else if (anyThreat) {
    // Mode 3 — influence: threatened corners fast blink 80ms
    bool on = ((now / 80) % 2) == 0;
    if (on) {
      if (frontThreat) corners |= NAV_FL | NAV_FR;
      if (leftThreat)  corners |= NAV_FL | NAV_RL;
      if (rightThreat) corners |= NAV_FR | NAV_RR;
      if (backThreat)  corners |= NAV_RL | NAV_RR;
    }
  } else if (sensorOk) {
    // Mode 2 — all clear: double blink FR then FL, 1.8s cycle
    uint32_t t = now % 1800;
    if      (t <  80) corners = NAV_FR;
    else if (t < 160) corners = 0;
    else if (t < 240) corners = NAV_FR;
    else if (t < 320) corners = 0;
    else if (t < 400) corners = NAV_FL;
    else if (t < 480) corners = 0;
    else if (t < 560) corners = NAV_FL;
  } else {
    // Mode 1 — no sensor: single blink, 2s cycle
    uint32_t t = now % 2000;
    if      (t <  80) corners = NAV_FR;
    else if (t < 400) corners = 0;
    else if (t < 480) corners = NAV_FL;
  }

  digitalWrite(PIN_NAV_FL, (corners & NAV_FL) ? NAV_LED_ON : NAV_LED_OFF);
  digitalWrite(PIN_NAV_FR, (corners & NAV_FR) ? NAV_LED_ON : NAV_LED_OFF);
  digitalWrite(PIN_NAV_RL, (corners & NAV_RL) ? NAV_LED_ON : NAV_LED_OFF);
  digitalWrite(PIN_NAV_RR, (corners & NAV_RR) ? NAV_LED_ON : NAV_LED_OFF);
}

// ─────────────────────────────────────────────────────────────
//  WS2812 status LED update (unchanged)
// ─────────────────────────────────────────────────────────────

// Shared LED mode from task_logic
static volatile uint8_t g_ledMode = 0;
// 0=nominal 1=influence 2=brake 3=rclost 4=landing 5=sensorDead

static void update_led_from_mode(uint32_t now) {
  static uint32_t lastMs = 0;
  if (now - lastMs < 30) return;
  lastMs = now;

  CRGB c = CRGB::Black;
  switch (g_ledMode) {
    case 0: c = CRGB(0, 40, 0);  break;  // nominal: dim green solid
    case 1: c = ((now/100)%2) ? CRGB(180,80,0) : CRGB::Black; break;  // influence: amber blink
    case 2: c = CRGB(150, 0, 0); break;  // brake: red solid
    case 3: c = ((now/100)%2) ? CRGB(150,0,0) : CRGB::Black;  break;  // rc lost: red blink
    case 4: c = CRGB(60, 0, 90); // landing: purple pulse
            { uint8_t br = (uint8_t)(128 + 127 * sinf(now * 0.003f));
              c.r = (uint8_t)(60 * br / 255);
              c.b = (uint8_t)(90 * br / 255); } break;
    case 5: c = ((now/600)%2) ? CRGB(0,40,0) : CRGB::Black; break;  // sensor dead: slow green blink
  }
  leds[0] = c;
  FastLED.show();
}

// ─────────────────────────────────────────────────────────────
//  BLE server (CommCortex)
//  All commands unchanged from v1.9.2.
//  BLE interface unchanged from v2.1.8 (timeout token now "rc:").
// ─────────────────────────────────────────────────────────────

static BLECharacteristic *bleCC_status   = nullptr;
static BLECharacteristic *bleCC_zone     = nullptr;
static BLECharacteristic *bleCC_watchdog = nullptr;
static BLECharacteristic *bleCC_sensory  = nullptr;
static BLECharacteristic *bleCC_avoid    = nullptr;
static BLECharacteristic *bleCC_failsafe = nullptr;
static BLECharacteristic *bleCC_timeouts = nullptr;

class CCCallbacks : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic *c) override {
    String sval = c->getValue().c_str();
    String uuid = c->getUUID().toString();

    if (uuid == BLE_CC_SENSORY_UUID) {
      if (sval == "on")  { g_sensoryPresent = true;  c->setValue("sensory on");  prefs_save(); }
      if (sval == "off") { g_sensoryPresent = false; c->setValue("sensory off"); prefs_save(); }
    }
    else if (uuid == BLE_CC_AVOIDANCE_UUID) {
      // ── Active floor ──
      // NOTE: arm/disarm intentionally NOT saved — see prefs block comment
      // near g_prefs declaration. g_activeFloorArmed always boots true.
      if (sval == "afloor arm") {
        g_activeFloorArmed = true; c->setValue("afloor ARMED"); return;
      } else if (sval == "afloor disarm" || sval == "afloor off") {
        g_activeFloorArmed = false; c->setValue("afloor disarmed"); return;
      } else if (sval.startsWith("afloor cap ")) {
        float v = sval.substring(11).toFloat();
        if (v >= 0.0f && v <= 0.6f) { g_afAuthCap = v;
          char b[24]; snprintf(b,24,"afloor cap %.2f",v); c->setValue(b); prefs_save(); } return;
      } else if (sval.startsWith("afloor band ")) {
        float v = sval.substring(12).toFloat();
        if (v >= 0.2f && v <= 6.0f) { g_afEngageBandM = v;
          char b[24]; snprintf(b,24,"afloor band %.1f",v); c->setValue(b); prefs_save(); } return;
      } else if (sval.startsWith("afloor kp ")) {
        float v = sval.substring(10).toFloat();
        if (v >= 0.0f && v <= 5.0f) { g_afKp = v;
          char b[24]; snprintf(b,24,"afloor kp %.2f",v); c->setValue(b); prefs_save(); } return;
      } else if (sval.startsWith("afloor kd ")) {
        float v = sval.substring(10).toFloat();
        if (v >= 0.0f && v <= 5.0f) { g_afKd = v;
          char b[24]; snprintf(b,24,"afloor kd %.2f",v); c->setValue(b); prefs_save(); } return;
      } else if (sval.startsWith("afloor bank ")) {
        float v = sval.substring(12).toFloat();
        if (v >= 5.0f && v <= 30.0f) { g_afMaxBankDeg = v;
          char b[24]; snprintf(b,24,"afloor bank %.0f",v); c->setValue(b); prefs_save(); } return;
      } else if (sval == "afloor status" || sval == "afloor") {
        char b[80];
        snprintf(b,80,"afloor %s cap=%.2f band=%.1f kp=%.2f kd=%.2f bank=%.0f",
          g_activeFloorArmed?"ARMED":"disarmed",
          g_afAuthCap, g_afEngageBandM, g_afKp, g_afKd, g_afMaxBankDeg);
        c->setValue(b); return;
      } else if (sval.startsWith("afloor")) { return; }  // unknown — ignore

      // ── Active horizontal brake ──
      if (sval == "ahoriz arm") {
        g_activeHorizArmed = true; c->setValue("ahoriz ARMED"); prefs_save(); return;
      } else if (sval == "ahoriz disarm" || sval == "ahoriz off") {
        g_activeHorizArmed = false; c->setValue("ahoriz disarmed"); prefs_save(); return;
      } else if (sval.startsWith("ahoriz cap ")) {
        float v = sval.substring(11).toFloat();
        if (v >= 0.0f && v <= 0.6f) { g_ahAuthCap = v;
          char b[24]; snprintf(b,24,"ahoriz cap %.2f",v); c->setValue(b); prefs_save(); } return;
      } else if (sval.startsWith("ahoriz kp ")) {
        float v = sval.substring(10).toFloat();
        if (v >= 0.0f && v <= 2.0f) { g_ahKp = v;
          char b[24]; snprintf(b,24,"ahoriz kp %.2f",v); c->setValue(b); prefs_save(); } return;
      } else if (sval.startsWith("ahoriz kd ")) {
        float v = sval.substring(10).toFloat();
        if (v >= 0.0f && v <= 2.0f) { g_ahKd = v;
          char b[24]; snprintf(b,24,"ahoriz kd %.2f",v); c->setValue(b); prefs_save(); } return;
      } else if (sval.startsWith("ahoriz vref ")) {
        float v = sval.substring(12).toFloat();
        if (v >= 50.0f && v <= 1000.0f) { g_ahVelRefCms = v;
          char b[24]; snprintf(b,24,"ahoriz vref %.0f",v); c->setValue(b); prefs_save(); } return;
      } else if (sval.startsWith("ahoriz engage ")) {
        float v = sval.substring(14).toFloat();
        if (v >= 0.0f && v <= 200.0f) { g_ahEngageCms = v;
          char b[24]; snprintf(b,24,"ahoriz engage %.0f",v); c->setValue(b); prefs_save(); } return;
      } else if (sval == "ahoriz status" || sval == "ahoriz") {
        char b[80];
        snprintf(b,80,"ahoriz %s cap=%.2f kp=%.2f kd=%.2f vref=%.0f eng=%.0f",
          g_activeHorizArmed?"ARMED":"disarmed",
          g_ahAuthCap, g_ahKp, g_ahKd, g_ahVelRefCms, g_ahEngageCms);
        c->setValue(b); return;
      } else if (sval.startsWith("ahoriz")) { return; }

      // ── Master avoidance switches ──
      if      (sval == "on")         { g_avoidEnabled = true;       c->setValue("avoid on");  prefs_save(); }
      else if (sval == "off")        { g_avoidEnabled = false;      c->setValue("avoid off"); prefs_save(); }
      else if (sval == "horiz on")   { g_horizAvoidEnabled = true;  c->setValue("horiz on");  prefs_save(); }
      else if (sval == "horiz off")  { g_horizAvoidEnabled = false; c->setValue("horiz off"); prefs_save(); }
      else if (sval == "alt on")     { g_altCorridorEnabled = true; c->setValue("alt on");    prefs_save(); }
      else if (sval == "alt off")    { g_altCorridorEnabled = false;c->setValue("alt off");   prefs_save(); }
      else if (sval == "map on")     { g_mapAvoidEnabled = true;    c->setValue("map on");    prefs_save(); }
      else if (sval == "map off")    { g_mapAvoidEnabled = false;   c->setValue("map off");   prefs_save(); }
    }
    else if (uuid == BLE_CC_FAILSAFE_UUID) {
      int v = sval.toInt();
      if (v >= RC_MIN && v <= RC_MAX) {
        g_failsafeRTH = (uint16_t)v;
        char buf[8]; snprintf(buf, sizeof(buf), "%d", g_failsafeRTH);
        c->setValue(buf);
        prefs_save();
      }
    }
    else if (uuid == BLE_CC_TIMEOUTS_UUID) {
      // Format: "sensor:50 rc:100"
      int si = sval.indexOf("sensor:"); int ii = sval.indexOf("rc:");
      if (si >= 0) { int v = sval.substring(si+7).toInt(); if (v >= 20) g_wdgSensorMs = (uint16_t)v; }
      if (ii >= 0) { int v = sval.substring(ii+3).toInt(); if (v >= 50) g_wdgRcMs   = (uint16_t)v; }
      char buf[32]; snprintf(buf, sizeof(buf), "sensor:%d rc:%d",
                             g_wdgSensorMs, g_wdgRcMs);
      c->setValue(buf);
      prefs_save();
    }
  }
};

static void ble_cc_init() {
  BLEDevice::init("CommCortex");
  BLEServer *srv = BLEDevice::createServer();
  BLEService *svc = srv->createService(BLEUUID(BLE_CC_SERVICE_UUID), 28);

  auto mk = [&](const char *uuid, uint32_t props) -> BLECharacteristic* {
    auto *ch = svc->createCharacteristic(uuid, props);
    ch->addDescriptor(new BLE2902());
    ch->setCallbacks(new CCCallbacks());
    return ch;
  };
  const uint32_t RN  = BLECharacteristic::PROPERTY_READ  | BLECharacteristic::PROPERTY_NOTIFY;
  const uint32_t RW  = BLECharacteristic::PROPERTY_READ  | BLECharacteristic::PROPERTY_WRITE;
  const uint32_t RWN = BLECharacteristic::PROPERTY_READ  | BLECharacteristic::PROPERTY_WRITE
                     | BLECharacteristic::PROPERTY_NOTIFY;

  bleCC_status   = mk(BLE_CC_STATUS_UUID,    RN);
  bleCC_zone     = mk(BLE_CC_ZONE_UUID,      RN);
  bleCC_watchdog = mk(BLE_CC_WATCHDOG_UUID,  RN);
  bleCC_sensory  = mk(BLE_CC_SENSORY_UUID,   RWN);
  bleCC_avoid    = mk(BLE_CC_AVOIDANCE_UUID, RWN);
  bleCC_failsafe = mk(BLE_CC_FAILSAFE_UUID,  RW);
  bleCC_timeouts = mk(BLE_CC_TIMEOUTS_UUID,  RW);

  bleCC_status->setValue("OK");
  bleCC_zone->setValue("CLEAR");
  bleCC_watchdog->setValue("OK 50Hz");

  svc->start();
  BLEAdvertising *adv = BLEDevice::getAdvertising();
  adv->addServiceUUID(BLE_CC_SERVICE_UUID);
  adv->setScanResponse(true);
  BLEDevice::startAdvertising();
  USBSerial.println("[OK] BLE 'CommCortex' advertising");
}

static uint32_t blePrevMs = 0;

static void ble_update(uint32_t now, bool anyBrake, bool anyInfluence,
                       bool frontT, bool leftT, bool rightT, bool backT) {
  if (now - blePrevMs < 1000) return;
  blePrevMs = now;

  if (!bleCC_status) return;

  // Status
  if (rcLost)     bleCC_status->setValue("RC LOST");
  else if (sensorDead) bleCC_status->setValue("SENSOR DEAD");
  else            bleCC_status->setValue("OK");
  bleCC_status->notify();

  // Zone
  char zone[48] = "CLEAR";
  if (anyBrake) {
    snprintf(zone, sizeof(zone), "BRAKE%s%s%s%s",
      frontT?" FRONT":"", leftT?" LEFT":"",
      rightT?" RIGHT":"", backT?" BACK":"");
  } else if (anyInfluence) {
    snprintf(zone, sizeof(zone), "INFLUENCE%s%s%s%s",
      frontT?" FRONT":"", leftT?" LEFT":"",
      rightT?" RIGHT":"", backT?" BACK":"");
  }
  bleCC_zone->setValue(zone); bleCC_zone->notify();

  // Watchdog — compute rate once per second, display the captured value
  static uint32_t wdRate = 0;
  if (now - g_wdLastCheckMs >= 1000) {
    uint32_t cnt = g_wdPulseCount;
    wdRate = cnt - g_wdLastCount;
    g_wdLastCount   = cnt;
    g_wdLastCheckMs = now;
    g_wdOk = (wdRate >= 40);
  }
  char wd[24];
  snprintf(wd, sizeof(wd), "%s %uHz", g_wdOk ? "OK" : "STOPPED", (unsigned)wdRate);
  bleCC_watchdog->setValue(wd); bleCC_watchdog->notify();
}

// ─────────────────────────────────────────────────────────────
//  CORE 0 TASK — task_protocol
//  SBUS receive (+SBUS TX in OUT_SBUS), SafetyPacket RX/TX, watchdog pulse
//  Pinned to Core 0.
// ─────────────────────────────────────────────────────────────

static void task_protocol(void *) {
  uint8_t  sbusRxBuf[SBUS_FRAME_LEN];
  uint8_t  sbusRxPos    = 0;
  uint32_t sbusLastByteUs = 0;

  esp_task_wdt_add(NULL);        // v2.1.5: subscribe this task to the TWDT

  for (;;) {
    esp_task_wdt_reset();        // v2.1.5: pet — must be reached every iteration

    // ── SBUS RX ──────────────────────────────────────────────
    // 0x0F has no uniqueness inside the payload, so byte-position sync
    // alone can phase-lock onto a data byte. Sync strategy:
    //   1. Inter-frame idle gap: a frame is 3ms of wire time inside a
    //      7/14ms period, so ≥4ms of true idle separates frames. If
    //      >SBUS_GAP_US passes mid-frame, the partial frame is stale —
    //      discard and treat the next byte as a frame start. (Bytes of
    //      one frame usually arrive buffered together, so intra-frame
    //      read gaps are ~0; the 3ms threshold is safe at a 1ms loop.)
    //   2. Header + end-byte validation in sbus_decode() rejects any
    //      misaligned frame; the next real gap realigns within 1 frame.
    while (RC_SERIAL.available()) {
      uint32_t nowUs = micros();
      if (sbusRxPos && (uint32_t)(nowUs - sbusLastByteUs) > SBUS_GAP_US)
        sbusRxPos = 0;                       // stale partial frame — resync
      sbusLastByteUs = nowUs;

      uint8_t b = RC_SERIAL.read();
      if (sbusRxPos == 0 && b != SBUS_HEADER) continue;  // wait for header
      sbusRxBuf[sbusRxPos++] = b;

      if (sbusRxPos == SBUS_FRAME_LEN) {
        sbusRxPos = 0;

        ChannelFrame cf;
        cf.valid = sbus_decode(sbusRxBuf, cf.ch);
        if (cf.valid) {
          lastRcRxMs = millis();
          __sync_synchronize();  // barrier before flag write
          rcLost = false;
          xQueueOverwrite(qRxChannels, &cf);
        }
        // Invalid frame (bad end byte / failsafe flag): drop silently.
        // Failsafe-flagged frames intentionally do NOT refresh lastRcRxMs,
        // so the g_wdgRcMs watchdog engages just as if frames had stopped.
      }
    }

    // ── Sensory Cortex RX / TX ────────────────────────────────
    if (g_sensoryPresent) {
      SafetyPacket sp;
      if (parse_safety_stream(Serial1, sp)) {
        lastSafetyPktMs = millis();
        xQueueOverwrite(qSafetyPkt, &sp);   // always refresh data
        // [v2.1.6 I-005] Restore authority only after a run of good packets.
        if (sensorDead) {
          if (++g_safetyRecoverCnt >= SAFETY_RECOVER_PKTS) {
            __sync_synchronize();  // barrier before flag write
            sensorDead = false;
            g_safetyRecoverCnt = 0;
          }
        }
      }
      ProximityCmd pc;
      if (xQueueReceive(qProxCmd, &pc, 0) == pdTRUE) {
        Serial1.write((uint8_t*)&pc, sizeof(ProximityCmd));
      }
      if (millis() - lastSafetyPktMs > g_wdgSensorMs) {
        __sync_synchronize();  // barrier before flag write
        sensorDead = true;
        g_safetyRecoverCnt = 0;   // [v2.1.6 I-005] reset recovery progress on loss
      }
    }

    // ── 50Hz watchdog pulse ───────────────────────────────────
    // Hardware square wave — if this task stalls, pulse stops,
    // RC filter discharges, CD4053 routes raw PPM to Naza.
    {
      static uint32_t wdLastUs = 0;
      static bool     wdState  = false;
      uint32_t nowUs = micros();
      if (nowUs - wdLastUs >= 1000000u / (WD_PULSE_HZ * 2)) {
        wdLastUs = nowUs;
        wdState  = !wdState;
        digitalWrite(PIN_WD_PULSE, wdState ? HIGH : LOW);
        if (wdState) g_wdPulseCount++;
      }
    }

#if OUTPUT_PROTOCOL == OUT_SBUS
    // ── SBUS output — send one frame every SBUS_OUT_PERIOD_MS ─
    sbus_out_tick();
#endif

    // ── RC link watchdog ──────────────────────────────────────
    if (millis() - lastRcRxMs > g_wdgRcMs) {
      __sync_synchronize();  // barrier before flag write
      rcLost = true;
      // Push failsafe frame — RTH on CH_FLIGHT_MODE slot
      rc_out_failsafe();
      // Reset SBUS sync state so recovery starts clean on next valid frame.
      // Without this, a partial frame buffered at signal loss would cause
      // one spurious failed decode attempt before resync on resume.
      sbusRxPos = 0;
    }

#if OUTPUT_PROTOCOL == OUT_PPM
    // ── RMT stall detection ───────────────────────────────────
    // If ppm_tx_done_cb() fails to re-queue a frame, g_rmtStalled is set.
    // PPM output has stopped. The hardware watchdog (GPIO5 → CD4053) will
    // engage within ~100ms routing the raw receiver signal to the FC.
    // Log the event — this should never happen in normal operation.
    if (g_rmtStalled) {
      g_rmtStalled = false;
      USBSerial.println("[ERR] RMT re-queue failed — PPM output stalled. HW watchdog active.");
    }
#endif

    vTaskDelay(1);
  }
}

// ─────────────────────────────────────────────────────────────
//  CORE 1 TASK — task_logic
//  Avoidance, aux GPIO, LED, BLE, ProximityCmd
//  Pinned to Core 1.
// ─────────────────────────────────────────────────────────────

static void task_logic(void *) {
  ChannelFrame cf;
  SafetyPacket sp;
  memset(&sp, 0, sizeof(sp));
  sp.tteFront = 255; sp.tteLeft = 255; sp.tteRight = 255; sp.tteBack = 255;

  uint8_t proxCmdCounter = 0;
  float   scale          = 1.0f;
  bool    landingMode    = false;

  esp_task_wdt_add(NULL);        // v2.1.5: subscribe this task to the TWDT

  for (;;) {
    esp_task_wdt_reset();        // v2.1.5: pet at top — before the queue-wait
                                 // and before any 'continue' path below
    uint32_t now = millis();

    // Wait for fresh RC frame (up to 20ms)
    if (xQueueReceive(qRxChannels, &cf, pdMS_TO_TICKS(20)) != pdTRUE) {
      // No RC data — RC lost or startup. Force aux outputs safe.
      if (rcLost) {
        digitalWrite(PIN_FLOOD, LOW);
        digitalWrite(PIN_VTX,   LOW);
      }
      update_led_from_mode(now);
      continue;
    }

    // Collect latest SafetyPacket (non-blocking)
    SafetyPacket spNew;
    if (xQueueReceive(qSafetyPkt, &spNew, 0) == pdTRUE) sp = spNew;

    // Work on a local copy of channels
    uint16_t ch[RC_NUM_CH];
    memcpy(ch, cf.ch, sizeof(ch));

    // ── Landing mode ─────────────────────────────────────────
    landingMode = (ch[CH_LANDING] > LANDING_THRESHOLD);

    // ── Proximity VRB → horizontal scale + freeze logic ──────
    {
      uint16_t vrb = ch[CH_PROX_VRB];
      float vrbScale = fmap((float)vrb, RC_MIN, RC_MAX, SCALE_MIN, SCALE_MAX);

      // [v2.1.8 I-007] Rising edge: landing mode just STARTED. g_rememberedScaleH
      // still holds the last live flight-scale value (it stops updating the
      // moment landingMode goes true, below) — this is the one low-frequency
      // moment to persist it, instead of writing flash every loop in flight.
      if (!g_prevLanding && landingMode) {
        prefs_save_scaleH();
      }
      if (g_prevLanding && !landingMode) {
        g_hScaleFrozen = true;
        g_freezeVrbRef = vrb;
        // Falling edge: landing mode just ENDED. g_rememberedFloorM was last
        // written by apply_altitude_corridor() on the previous (still-landing)
        // iteration and is now final for this session — persist it here
        // rather than on every loop while landing mode dials it in.
        prefs_save_floor();
      }
      if (landingMode) {
        scale = g_rememberedScaleH;
      } else {
        if (g_hScaleFrozen) {
          if (abs((int)vrb - (int)g_freezeVrbRef) > VRB_MOVE_THRESHOLD)
            g_hScaleFrozen = false;
        }
        if (!g_hScaleFrozen) g_rememberedScaleH = vrbScale;
        scale = g_rememberedScaleH;
      }
      g_prevLanding = landingMode;
    }

    // ── Naza RTH / failsafe detection ────────────────────────
    bool nazaRTH = (ch[CH_FLIGHT_MODE] < RTH_THRESHOLD);

    // ── Apply avoidance ──────────────────────────────────────
    if (!sensorDead && !rcLost && g_avoidEnabled && !nazaRTH) {
      if (g_horizAvoidEnabled)  apply_horizontal_avoidance(ch, sp, scale, landingMode);
      if (g_altCorridorEnabled) apply_altitude_corridor(ch, sp, landingMode);
    } else if (nazaRTH) {
      g_floorActive       = false;
      g_ceilingActive     = false;
      g_activeFloorActive = false;
      g_activeHorizActive = false;
    }

    // ── Push modified channels to the RC output ──────────────
    // OUT_PPM : ch[0..7] → 8-channel PPM frame (identical to v2.1.8)
    // OUT_SBUS: ch[0..13] → SBUS channels 1–14 (15/16 sent neutral)
    // ch[7] (bypass) is passed through unmodified — the upstream hardware
    // uses it; the FC sees it and ignores it.
    if (!rcLost) {
      rc_out_update(ch);
    }
    // RC lost: rc_out_failsafe() already called by task_protocol watchdog

    // ── Aux GPIO outputs ──────────────────────────────────────
    // Ch10 (idx9)  = Floodlight: HIGH when channel above threshold
    // Ch11 (idx10) = VTX:        HIGH when channel above threshold
    // No hysteresis needed — channel values are stable from transmitter switches
    digitalWrite(PIN_FLOOD, (ch[CH_FLOOD] > AUX_ON_THRESHOLD) ? HIGH : LOW);
    digitalWrite(PIN_VTX,   (ch[CH_VTX]   > AUX_ON_THRESHOLD) ? HIGH : LOW);

    // ── Nav lights ────────────────────────────────────────────
    bool navLightsOn = (ch[CH_NAV_LIGHTS] > NAV_LIGHTS_THRESHOLD);
    bool frontT = sp.frontBlocked || (sp.tteFront != 255 && sp.tteFront < (uint8_t)(TTE_WARN_S * 10));
    bool leftT  = sp.leftBlocked  || (sp.tteLeft  != 255 && sp.tteLeft  < (uint8_t)(TTE_WARN_S * 10));
    bool rightT = sp.rightBlocked || (sp.tteRight != 255 && sp.tteRight < (uint8_t)(TTE_WARN_S * 10));
    bool backT  = sp.backBlocked  || (sp.tteBack  != 255 && sp.tteBack  < (uint8_t)(TTE_WARN_S * 10));
    bool anyBrake = sp.frontBlocked || sp.leftBlocked || sp.rightBlocked || sp.backBlocked;
    bool anyInfl  = !anyBrake && (frontT || leftT || rightT || backT);
    update_nav_leds(now, navLightsOn, frontT, leftT, rightT, backT,
                    anyBrake, !sensorDead);

    // ── WS2812 status LED mode ────────────────────────────────
    if (rcLost)       g_ledMode = 3;
    else if (sensorDead) g_ledMode = 5;
    else if (landingMode) g_ledMode = 4;
    else if (anyBrake)  g_ledMode = 2;
    else if (anyInfl)   g_ledMode = 1;
    else                g_ledMode = 0;
    update_led_from_mode(now);

    // ── ProximityCmd → Sensory Cortex (~7–14Hz at SBUS frame rate) ──
    if (++proxCmdCounter >= 10) {
      proxCmdCounter = 0;
      ProximityCmd pc = build_prox_cmd(scale, landingMode);
      xQueueOverwrite(qProxCmd, &pc);
    }

    // ── Telemetry values (DEBUG/BLE; future S.Port/CRSF add-on) ─
    // Compute each value from the current avoidance state.
    // All writes are int16_t — atomic on ESP32-S3 (single 32-bit bus cycle).
    {
      // Zone: 0=clear  1=influence  2=brake  3=system fault
      g_telZone = sensorDead ? 3 :
                  (anyBrake  ? 2 :
                  (anyInfl   ? 1 : 0));

      // Obs.: closest obstacle in cm. Use TTE fields from SafetyPacket.
      // TTE is in tenths of a second; closest = lowest TTE × (scale×200 cm/s approx).
      // Simpler and more direct: use the blocked flags + raw TTE to pick
      // the most urgent sensor and estimate distance from TTE × typical approach.
      // Most useful: report the minimum non-255 TTE scaled to cm, capped at 999.
      // Actually the SafetyPacket carries velFront etc in cm/s, and TTE in 0.1s.
      // Distance estimate = TTE(s) × |vel|(cm/s). But we don't have raw distance.
      // Use velFront/TTE product where valid, else 999 (clear).
      // If blocked: distance ≈ ZONE_INNER_CM (inside brake boundary).
      {
        int16_t closest = 999;
        struct { uint8_t blocked; uint8_t tte; int16_t vel; } axes[4] = {
          { sp.frontBlocked, sp.tteFront, sp.velFront },
          { sp.leftBlocked,  sp.tteLeft,  sp.velLeft  },
          { sp.rightBlocked, sp.tteRight, sp.velRight },
          { sp.backBlocked,  sp.tteBack,  sp.velBack  },
        };
        for (int i = 0; i < 4; i++) {
          if (axes[i].blocked) {
            closest = min(closest, (int16_t)(ZONE_INNER_CM * scale));
          } else if (axes[i].tte != 255 && axes[i].vel < 0) {
            // dist ≈ TTE(s) × closing_speed(cm/s)
            float dist = (axes[i].tte / 10.0f) * (float)(-axes[i].vel);
            int16_t d = (int16_t)constrain(dist, 0.0f, 999.0f);
            closest = min(closest, d);
          }
        }
        g_telObsCm = closest;
      }

      // Head: bearing to most urgent threat, 0-359 deg, 0 = no threat.
      // Front=0, Right=90, Back=180, Left=270. Pick the blocked/lowest-TTE axis.
      {
        int16_t bearing = 0;
        bool hasThreat = false;
        uint8_t minTTE = 255;
        // Priority: blocked first, then lowest TTE
        struct { uint8_t blocked; uint8_t tte; int16_t deg; } dirs[4] = {
          { sp.frontBlocked, sp.tteFront,   0 },
          { sp.rightBlocked, sp.tteRight,  90 },
          { sp.backBlocked,  sp.tteBack,  180 },
          { sp.leftBlocked,  sp.tteLeft,  270 },
        };
        for (int i = 0; i < 4; i++) {
          if (dirs[i].blocked) {
            if (!hasThreat || dirs[i].tte < minTTE) {
              bearing = dirs[i].deg; minTTE = dirs[i].tte; hasThreat = true;
            }
          }
        }
        if (!hasThreat) {
          for (int i = 0; i < 4; i++) {
            if (dirs[i].tte != 255 && dirs[i].tte < minTTE) {
              bearing = dirs[i].deg; minTTE = dirs[i].tte; hasThreat = true;
            }
          }
        }
        g_telHead = hasThreat ? bearing : 0;
      }

      // RoC: vertical velocity cm/s signed (positive = climb, negative = descent)
      g_telRocCms = sp.vertVelCmS;

      // Flor: floor height cm (0 = floor off)
      g_telFlorCm = (int16_t)constrain(g_rememberedFloorM * 100.0f, 0.0f, 32767.0f);

      // Alt.: altitude above ground cm directly from SafetyPacket (TF-Luna Down)
      // altCm is uint16_t; constrain to int16_t range (max ~327m, well above TF-Luna 8m range)
      g_telAltCm = (int16_t)constrain((float)sp.altCm, 0.0f, 32767.0f);
    }

    // ── Bench debug output ── [bench] gated by DEBUG_SERIAL at top of file.
    // Decision-side state, ~10 Hz. Compiled out when DEBUG_SERIAL is false.
    // Watch: channels stay in 1000-2000 (C-002 saturation); sensDead goes
    // 1→0 only after recovery (I-004/I-005); avoidance biases the right axis.
    static uint16_t dbgN = 0;
    if (DEBUG_SERIAL && (++dbgN >= DBG_EVERY_N)) {
      dbgN = 0;
      USBSerial.printf(
        "CH R:%4d P:%4d T:%4d Y:%4d FM:%4d | scale:%.2f | "
        "rcLost:%d sensDead:%d RTH:%d land:%d | aFloor:%d aHoriz:%d | "
        "blk F:%d L:%d R:%d B:%d\n",
        ch[0], ch[1], ch[2], ch[3], ch[CH_FLIGHT_MODE], scale,
        rcLost, sensorDead, nazaRTH, landingMode,
        g_activeFloorActive, g_activeHorizActive,
        sp.frontBlocked, sp.leftBlocked, sp.rightBlocked, sp.backBlocked);
    }

    // ── BLE telemetry update (1Hz) ────────────────────────────
    ble_update(now, anyBrake, anyInfl, frontT, leftT, rightT, backT);
  }
}

// ─────────────────────────────────────────────────────────────
//  Hardware Task Watchdog (TWDT)                        [v2.1.5]
// ─────────────────────────────────────────────────────────────
//  Arduino-ESP32 3.x / IDF 5.x auto-initialises the TWDT at boot.
//  We reconfigure it (2s, panic-reset) BEFORE launching the work
//  tasks. Each task then subscribes itself (esp_task_wdt_add) and
//  pets it (esp_task_wdt_reset) once per iteration. If any watched
//  task stalls >2s the chip panics and reboots; during the reboot
//  the GPIO5 pulse stops, the RC filter discharges, and CD4053
//  routes raw receiver PPM straight to the Naza.
//
//  NOTE: setup() runs in the Arduino loopTask, which here only
//  does vTaskDelay(1000) and never pets the dog — so we remove it
//  from the TWDT to avoid a guaranteed boot-loop.
#define TWDT_TIMEOUT_MS 2000
static void twdt_configure() {
  esp_task_wdt_config_t cfg = {
    .timeout_ms     = TWDT_TIMEOUT_MS,
    .idle_core_mask = 0,       // do NOT watch idle tasks
    .trigger_panic  = true,    // timeout → panic → reboot
  };
  esp_err_t e = esp_task_wdt_init(&cfg);
  if (e == ESP_ERR_INVALID_STATE) {
    esp_task_wdt_reconfigure(&cfg);   // already inited by Arduino core
  }
  esp_task_wdt_delete(NULL);          // unsubscribe loopTask (harmless if absent)
}

// ─────────────────────────────────────────────────────────────
//  setup()
// ─────────────────────────────────────────────────────────────

void setup() {
  // ── Nav LEDs ──────────────────────────────────────────────
  pinMode(PIN_NAV_FL, OUTPUT); digitalWrite(PIN_NAV_FL, NAV_LED_OFF);
  pinMode(PIN_NAV_FR, OUTPUT); digitalWrite(PIN_NAV_FR, NAV_LED_OFF);
  pinMode(PIN_NAV_RL, OUTPUT); digitalWrite(PIN_NAV_RL, NAV_LED_OFF);
  pinMode(PIN_NAV_RR, OUTPUT); digitalWrite(PIN_NAV_RR, NAV_LED_OFF);

  // ── Aux GPIO outputs (start safe/off) ─────────────────────
  pinMode(PIN_FLOOD, OUTPUT); digitalWrite(PIN_FLOOD, LOW);
  pinMode(PIN_VTX,   OUTPUT); digitalWrite(PIN_VTX,   LOW);

  // ── Watchdog pulse output ─────────────────────────────────
  pinMode(PIN_WD_PULSE, OUTPUT);
  digitalWrite(PIN_WD_PULSE, LOW);

  // ── Status LED — boot double-blue flash ───────────────────
  FastLED.addLeds<WS2812, PIN_LED, GRB>(leds, NUM_LEDS);
  FastLED.setBrightness(60);
  for (int i = 0; i < 2; i++) {
    leds[0] = CRGB(0, 0, 120); FastLED.show(); delay(120);
    leds[0] = CRGB::Black;     FastLED.show(); delay(80);
  }

  // ── Debug UART (USB-CDC) ──────────────────────────────────
  USBSerial.begin(115200);
  delay(200);
  USBSerial.println("[BOOT] Communication Cortex v2.2.0-SBUS");
#if OUTPUT_PROTOCOL == OUT_PPM
  USBSerial.println("[INFO] SBUS input (soft-inverted) / RMT PPM output");
#else
  USBSerial.println("[INFO] SBUS input (soft-inverted) / SBUS output (soft-inverted)");
#endif

  // ── Load saved tunings/configs from flash ──               [v2.1.8 I-007]
  // Overrides compiled defaults with whatever was last set over BLE (and
  // the last remembered floor height / horizontal scale). First boot after
  // flashing finds nothing and keeps the defaults.
  prefs_load();
  USBSerial.printf(
    "[OK] Loaded saved config: avoid=%d horiz=%d alt=%d map=%d floor=%.2fm scaleH=%.2f\n",
    g_avoidEnabled, g_horizAvoidEnabled, g_altCorridorEnabled, g_mapAvoidEnabled,
    (double)g_rememberedFloorM, (double)g_rememberedScaleH);

  // ── Disable WiFi ─────────────────────────────────────────
  esp_wifi_stop();
  esp_wifi_deinit();

  // ── SBUS input UART (UART0, Serial) ───────────────────────
  // 100000 baud, 8E2, INVERTED — the 5th argument of begin() enables the
  // ESP32 UART peripheral's line inversion (RX and TX; TX is unattached
  // here), so the receiver's SBUS pad (idle-LOW on the wire) connects
  // DIRECTLY to GPIO4. No 74HCT04, no divider (3.3V receivers).
  RC_SERIAL.begin(BAUD_SBUS, SERIAL_8E2, PIN_RC_RX, -1, true);  // RX only
  USBSerial.printf("[OK] SBUS UART 100000 8E2 inverted GPIO%d\n", PIN_RC_RX);

  // ── Sensory Cortex UART (UART1, Serial1) ─────────────────
  Serial1.begin(BAUD_SENSE, SERIAL_8N1, PIN_SENSE_RX, PIN_SENSE_TX);
  USBSerial.printf("[OK] Sensory UART 921600 GPIO%d/%d (%s)\n",
    PIN_SENSE_RX, PIN_SENSE_TX,
    g_sensoryPresent ? "expected" : "absent — avoidance disabled");

#if OUTPUT_PROTOCOL == OUT_PPM
  // ── RMT PPM output ────────────────────────────────────────
  ppm_rmt_init();
  USBSerial.printf("[OK] RMT PPM output GPIO%d 50Hz 8ch 1µs resolution\n",
    PIN_RC_OUT);
#else
  // ── SBUS output UART (UART2, Serial2) ─────────────────────
  // TX-only (rx = -1), 100000 8E2, inverted — standard SBUS polarity
  // straight into CD4053 IN-B. The first frame (failsafe values) is sent
  // by sbus_out_tick() as soon as task_protocol starts.
  Serial2.begin(BAUD_SBUS, SERIAL_8E2, -1, PIN_RC_OUT, true);
  USBSerial.printf("[OK] SBUS output UART 100000 8E2 inverted GPIO%d, %dms frame\n",
    PIN_RC_OUT, SBUS_OUT_PERIOD_MS);
#endif

  // ── FreeRTOS queues ───────────────────────────────────────
  qRxChannels = xQueueCreate(1, sizeof(ChannelFrame));
  qSafetyPkt  = xQueueCreate(1, sizeof(SafetyPacket));
  qProxCmd    = xQueueCreate(1, sizeof(ProximityCmd));
  if (!qRxChannels || !qSafetyPkt || !qProxCmd) {
    USBSerial.println("[FATAL] Queue allocation failed — heap exhausted");
    leds[0] = CRGB(150, 0, 0); FastLED.show();
    abort();  // hardware watchdog routes raw PPM to Naza
  }

  // ── Seed watchdog timestamps ──────────────────────────────
  lastRcRxMs    = millis();
  lastSafetyPktMs = millis();

  // ── Hardware Task Watchdog — configure BEFORE tasks start ─
  // Must precede task creation so tasks can subscribe cleanly.
  twdt_configure();
  USBSerial.printf("[OK] TWDT armed: %dms panic-reset\n", TWDT_TIMEOUT_MS);

  // ── Launch FreeRTOS tasks ─────────────────────────────────
  xTaskCreatePinnedToCore(task_protocol, "proto", 4096, NULL, 2, NULL, 0);
  xTaskCreatePinnedToCore(task_logic,    "logic", 6144, NULL, 1, NULL, 1);
  USBSerial.println("[OK] Tasks launched");
  delay(100);

  if (!g_sensoryPresent) {
    sensorDead = true;
    USBSerial.println("[INFO] Sensory Cortex absent — avoidance disabled");
  }

  // ── BLE ───────────────────────────────────────────────────
  ble_cc_init();
  USBSerial.println("[INFO] Communication Cortex v2.2.0-SBUS running");
  USBSerial.println("[INFO] SBUS in: 16ch (14 used) -> 1000-2000us");
#if OUTPUT_PROTOCOL == OUT_PPM
  USBSerial.println("[INFO] PPM out: 8ch RMT GPIO6");
#else
  USBSerial.println("[INFO] SBUS out: 16ch UART2 GPIO6 inverted");
#endif
  USBSerial.printf( "[INFO] Flood GPIO%d  VTX GPIO%d\n", PIN_FLOOD, PIN_VTX);
}

void loop() {
  vTaskDelay(pdMS_TO_TICKS(1000));
}

/*
 * ================================================================
 *  v2.2.0-SBUS GPIO SUMMARY
 * ================================================================
 *
 *  GPIO4   SBUS RX from receiver (100000 8E2, SOFTWARE-inverted, 3.3V direct)
 *  GPIO5   50Hz watchdog pulse → 47kΩ + 1µF → CD4053 Switch A SELECT
 *  GPIO6   OUT_PPM : RMT PPM output → CD4053 IN-B (idle-HIGH)
 *          OUT_SBUS: UART2 SBUS TX  → CD4053 IN-B (inverted, idle-LOW)
 *  GPIO1   Floodlight FET gate (ch10, active HIGH)
 *  GPIO2   VTX FET gate       (ch11, active HIGH)
 *  GPIO7   Nav LED Front-Left  (via 2N2222/HCT04 inverter, ch12)
 *  GPIO8   Nav LED Front-Right (via 2N2222/HCT04 inverter, ch12)
 *  GPIO9   Nav LED Rear-Left   (via 2N2222/HCT04 inverter, ch12)
 *  GPIO10  Nav LED Rear-Right  (via 2N2222/HCT04 inverter, ch12)
 *  GPIO16  Sensory Cortex UART RX
 *  GPIO17  Sensory Cortex UART TX
 *  GPIO20  unused in this variant (was iBUS telemetry RX)
 *  GPIO21  unused in this variant (was iBUS telemetry TX)
 *  GPIO48  WS2812 status LED
 *
 * ================================================================
 *  CD4053 WIRING (identical board to the v2.1.8 iBUS build)
 * ================================================================
 *
 *  Switch B (signal selector):
 *    IN-A  ← RAW receiver output (3.3V) — MUST match the FC port:
 *            OUT_PPM  build: receiver's PPM output
 *            OUT_SBUS build: receiver's SBUS output (inverted-idle-LOW,
 *                            passed through the mux untouched)
 *    IN-B  ← ESP output from GPIO6
 *    OUT   → FC RC input port (Naza M V2: PPM port for OUT_PPM,
 *            X2/SBUS port for OUT_SBUS)
 *    SELECT ← Switch A OUT
 *
 *  Switch A (watchdog gate):
 *    SELECT ← GPIO5 RC filter (47kΩ + 1µF) + Pololu RC switch (ch8 PWM)
 *    OUT   → Switch B SELECT
 *    INH   → GND, VCC → 3.3V, GND → GND
 *
 *  74HCT04: NOT REQUIRED for SBUS in this variant — inversion is done
 *  by the ESP32-S3 UART peripheral. Gates 3–6 may still be used for
 *  nav LEDs, or replaced with 2N2222 transistors.
 *
 *  5V-LEVEL RECEIVERS: most modern SBUS receivers output 3.3V logic.
 *  If yours outputs 5V, add the 4.7kΩ/10kΩ divider on GPIO4 —
 *  GPIO4 is NOT 5V-tolerant.
 *
 * ================================================================
 *  AUX OUTPUT DRIVE (ch10 Floodlight, ch11 VTX)
 * ================================================================
 *
 *  GPIO1/GPIO2 → 2N2222 base via 1kΩ resistor → collector = switch output
 *  Or: GPIO1/GPIO2 → logic-level N-MOSFET gate directly
 *  ESP32-S3 GPIO = 3.3V, 40mA max. Use transistor/FET for any load > 20mA.
 *
 * ================================================================
 *  BLE COMMANDS (CommCortex) — UNCHANGED except timeout token
 * ================================================================
 *
 *  Status/Zone/Watchdog: read-only, notify every 1s
 *  Sensory: on / off
 *  Avoidance: on / off / horiz on|off / alt on|off / map on|off
 *             afloor arm|disarm|cap|band|kp|kd|bank|status
 *             ahoriz arm|disarm|cap|kp|kd|vref|engage|status
 *  Failsafe RTH: µs value 1000–2000 (default 1000 = RTH low)
 *  Timeouts: "sensor:50 rc:100"   (was "ibus:" in v2.1.8)
 *
 * ================================================================
 *  SENSORY CORTEX COMPATIBILITY
 * ================================================================
 *
 *  Requires the paired sensory_cortex build with matching SAFETY_PKT_VERSION.
 *  SafetyPacket struct must be byte-identical in both files.
 *  SAFETY_PKT_VERSION = 0x01 — parser rejects on mismatch.
 *  ProximityCmd struct unchanged. UART1 GPIO16/17 unchanged.
 *
 * ================================================================
 */
