/*
 * ================================================================
 *  Communication Cortex — v2.1.6
 *  Dual Cortex Obstacle Avoidance System
 * ================================================================
 *
 *  Board: ESP32-S3 Super Mini
 *  Sits between FS-X6B receiver and Naza M V2 flight controller.
 *
 *  INPUT:  iBUS from FS-X6B (14 channels, 115200 8N1, GPIO4)
 *          SafetyPacket / MapPacket from Sensory Cortex (UART1)
 *
 *  OUTPUT: Modified PPM to Naza via CD4053 (RMT peripheral, GPIO6)
 *          GPIO aux outputs (floodlight, VTX, nav LEDs)
 *          ProximityCmd to Sensory Cortex (UART1)
 *          50Hz watchdog pulse (GPIO5) → RC filter → CD4053 bypass
 *
 *  AVOIDANCE: All logic unchanged from v1.9.2. Values are now in
 *  PPM microseconds (1000–2000µs) throughout. SBUS 172–1811 is gone.
 *
 *  PPM OUTPUT — RMT PERIPHERAL:
 *    The ESP32-S3 RMT hardware peripheral generates the PPM pulse
 *    train autonomously, completely independent of FreeRTOS scheduling
 *    and CPU load. Channel values are double-buffered; task_logic
 *    writes to the shadow buffer, the RMT tx_done ISR swaps it in.
 *    Jitter < 1µs. The Naza (and all RC hardware) tolerates ±10µs.
 *
 *    PPM frame: 8 channels × (sync_gap + channel_pulse) + frame_gap
 *    Idle HIGH, pulse = LOW for SYNC_US, then HIGH for channel_us.
 *    Polarity matches standard Naza PPM input expectation.
 *
 *  iBUS INPUT:
 *    Standard FlySky iBUS: 32-byte frame, 115200 8N1 idle-HIGH.
 *    No inversion needed. Frame: 0x20 0x40 [14ch × 2B LE] [CRC 2B].
 *    Values: 1000–2000µs. Frame rate ~7ms (~142Hz).
 *    ch[0]–ch[13] map to iBUS channels 1–14.
 *
 *  AUX GPIO OUTPUTS (driven from iBUS channel values, task_logic):
 *    Floodlight (ch10, idx9)  → PIN_FLOOD (active HIGH to FET gate)
 *    VTX on/off  (ch11, idx10) → PIN_VTX   (active HIGH)
 *    Nav LEDs    (ch12, idx11) → 4 corner pins via inverter/transistor
 *
 *  BYPASS PATH (hardware, ESP-independent):
 *    ch8 PPM from FS-X6B → PPM/PWM decoder → ch8 PWM → Pololu RC
 *    switch → CD4053 Switch A SELECT. Completely upstream of this ESP.
 *    GPIO5 watchdog pulse (50Hz) → 47kΩ + 1µF RC filter → also feeds
 *    CD4053 Switch A SELECT. If ESP crashes, pulse stops, filter
 *    discharges (~40–100ms), CD4053 routes raw PPM straight to Naza.
 *
 *  CHANNEL MAP (default — see USER CONFIGURATION block below):
 *    Ch1  idx0  Roll          → Naza PPM, avoidance-modified
 *    Ch2  idx1  Pitch         → Naza PPM, avoidance-modified
 *    Ch3  idx2  Throttle      → Naza PPM, floor/ceiling-modified
 *    Ch4  idx3  Yaw           → Naza PPM, pass-through
 *    Ch5  idx4  Flight Mode   → Naza PPM ch5 (U) — RTH detect + failsafe
 *    Ch6  idx5  IOC           → Naza PPM, pass-through
 *    Ch7  idx6  Aux / Gimbal  → Naza PPM, pass-through
 *    Ch8  idx7  CD4053 bypass → upstream hardware only, ESP ignores
 *    Ch9  idx8  Proximity VRB → avoidance scale / floor set
 *    Ch10 idx9  Floodlight    → PIN_FLOOD GPIO output
 *    Ch11 idx10 VTX on/off    → PIN_VTX GPIO output
 *    Ch12 idx11 Nav lights    → PIN_NAV_FL/FR/RL/RR GPIO outputs
 *    Ch13 idx12 Landing mode  → logic flag
 *    Ch14 idx13 Ceiling switch→ logic flag
 *
 *  NOTE: Naza M V2 PPM = Flight Mode on PPM ch5.
 *        Naza M V2 SBUS = Flight Mode on SBUS ch7.
 *        CH_FLIGHT_MODE must match whichever TX channel carries your
 *        GPS/Atti/RTH switch. See USER CONFIGURATION block.
 *
 *  INTER-CORTEX UART (unchanged from v1.9.2):
 *    UART1 RX GPIO16 ← SafetyPacket / MapPacket from Sensory Cortex
 *    UART1 TX GPIO17 → ProximityCmd to Sensory Cortex
 *
 *  v1.9.2 → v2.1.4 changes:
 *    - SBUS completely removed (decode, encode, failsafe, 8E2 UART)
 *    - iBUS 32-byte frame decoder replaces SBUS decoder
 *    - RMT PPM encoder replaces SBUS UART TX
 *    - All channel arithmetic in PPM µs (1000–2000) natively
 *    - ch10/ch11 now ESP GPIO outputs (were upstream PWM breakout)
 *    - SBUS-to-PWM breakout board no longer needed
 *    - 74HCT04 gates 1+2 no longer needed (PPM is 3.3V, non-inverted)
 *    - 4.7kΩ/10kΩ SBUS divider removed
 *    - All avoidance logic, BLE, watchdog, safety layers: unchanged
 *
 * ================================================================
 *
 *  v2.0 HARDWARE CHANGES vs v1.9.2:
 *    Removed: 74HCT04 gates 1+2 (SBUS inversion)
 *    Removed: 4.7kΩ/10kΩ voltage divider on GPIO20
 *    Removed: SBUS-to-PWM breakout board
 *    Removed: GPIO20 (SBUS RX), GPIO21 (SBUS TX)
 *    Added:   GPIO4  iBUS RX from FS-X6B (direct, 3.3V, no inversion)
 *    Added:   GPIO6  PPM OUT to CD4053 IN-B (3.3V idle-HIGH)
 *    Added:   GPIO1  Floodlight output (to FET gate via 2N2222/HCT04)
 *    Added:   GPIO2  VTX output (to FET gate via 2N2222/HCT04)
 *    Kept:    GPIO5  Watchdog pulse → RC filter → CD4053
 *    Kept:    GPIO7/8/9/10  Nav LEDs (via 2N2222/HCT04, same as before)
 *    Kept:    GPIO16/17  Sensory Cortex UART
 *    Kept:    GPIO48  WS2812 status LED
 *    CD4053:  IN-A = raw PPM from FS-X6B (3.3V)
 *             IN-B = ESP PPM from GPIO6 (3.3V)
 *             OUT  = Naza PPM port
 *             (same bypass logic, now 3.3V throughout — no level shift)
 *
 * ================================================================
 *
 *  Arduino IDE settings (unchanged):
 *    Board:     ESP32S3 Dev Module
 *    Flash:     8MB
 *    PSRAM:     Disabled
 *    Partition: Huge APP (3MB No OTA)
 *    USB CDC:   Enabled
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

// ── GPIO map ──────────────────────────────────────────────────
#define PIN_IBUS_RX       4   // UART0 RX — iBUS in from FS-X6B (3.3V, direct)
//                            // UART0 TX not used (iBUS is RX-only here)
#define PIN_PPM_OUT       6   // RMT PPM output to CD4053 IN-B (3.3V idle-HIGH)
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
#define BAUD_IBUS      115200   // iBUS: 115200 8N1, idle-HIGH, no inversion
#define BAUD_SENSE     921600   // Sensory Cortex inter-cortex link
#define BAUD_TELEM     115200   // iBUS telemetry: same baud, half-duplex

// ── iBUS telemetry GPIO ───────────────────────────────────────
// Half-duplex on GPIO20 (RX) / GPIO21 (TX), both wired to the
// FS-X6B iBUS sensor port. TX → 1kΩ series resistor → iBUS wire.
// RX taps iBUS wire directly. TX idles HIGH (ESP32-S3 UART idle
// state) which is correct iBUS idle — no contention when silent.
#define PIN_TELEM_RX   20   // Serial2 RX — listen for TX sensor polls
#define PIN_TELEM_TX   21   // Serial2 TX — send sensor replies (via 1kΩ)

// ── iBUS constants ────────────────────────────────────────────
#define IBUS_FRAME_LEN     32   // 2 header + 28 data (14ch×2B) + 2 CRC
#define IBUS_HEADER_0    0x20   // length byte
#define IBUS_HEADER_1    0x40   // command byte (channels)
#define IBUS_NUM_CH        14   // channels per standard FS-X6B iBUS frame
#define IBUS_MID         1500   // channel midpoint (µs)
#define IBUS_MIN         1000   // channel minimum (µs)
#define IBUS_MAX         2000   // channel maximum (µs)
#define IBUS_DEAD          30   // deadband around neutral (µs)

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
#define WDG_IBUS_MS        100   // renamed from WDG_SBUS_MS
#define WD_PULSE_HZ         50

// ════════════════════════════════════════════════════════════════
//  USER CONFIGURATION — edit this block to match your wiring
// ════════════════════════════════════════════════════════════════

// ── Flight Mode channel ───────────────────────────────────────
// Which iBUS channel carries your GPS / Atti / RTH switch.
// This is the 0-based index (channel number minus 1).
//
// Naza M V2 PPM mode:  Flight Mode = Naza PPM ch5 → iBUS idx 4
// Naza M V2 SBUS mode: Flight Mode = Naza SBUS ch7 → iBUS idx 6
//
// Set to match whichever TX channel your 3-position switch is on.
// The failsafe frame automatically sends RTH on this same channel.
#define CH_FLIGHT_MODE    4    // PPM Naza: idx4 = ch5. SBUS Naza was idx6 = ch7.

// ── Aux channel assignments (0-based iBUS index) ─────────────
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

// ── iBUS telemetry sensor definitions ────────────────────────
// The TX polls sensor addresses 0x00–0x0F. We respond on 6 slots.
// type_id is a DIRECT INDEX into the TX label table — not an arbitrary ID.
// i6 CoreTX label table after patches (indices are the type_ids to send):
//   [0x01]=Zone  [0x02]=Obs.  [0x06]=Flor  [0x08]=Head  [0x09]=RoC  [0x0F]=Alt.
// IntV[0x00] and ExtV[0x03] reserved for X6B real battery data — untouched.
//
// Slot  TypeID  Label    Data
//  0     0x01   Zone     avoidance zone 0-3
//  1     0x02   Obs.     closest obstacle cm (999=clear)
//  2     0x08   Head     threat bearing 0-359 deg (0=no threat)
//  3     0x09   RoC      vertical velocity cm/s signed
//  4     0x06   Flor     floor height cm (0=floor off)
//  5     0x0F   Alt.     altitude above ground cm (TF-Luna Down)
#define TELEM_NUM_SENSORS    6
#define TELEM_ADDR_BASE      0x00  // first sensor address we claim

// Sensor type IDs — DIRECT INDICES into the TX label table.
// TX firmware uses type_id as table[type_id] to find the display string.
// CoreTX label table (after patches):
//   [0x00]=IntV  [0x01]=Zone  [0x02]=Obs.  [0x03]=ExtV  [0x04]=Cell
//   [0x05]=Curr  [0x06]=Flor  [0x07]=RPM   [0x08]=Head  [0x09]=RoC
//   [0x0A]=CoG   [0x0B]=GPS   [0x0C]=AccX  [0x0D]=AccY  [0x0E]=AccZ
//   [0x0F]=Alt.  [0x10]=Pit.  [0x11]=Yaw   ...
static const uint8_t TELEM_TYPE[TELEM_NUM_SENSORS] = {
  0x01,  // Zone  table[0x01]
  0x02,  // Obs.  table[0x02]
  0x08,  // Head  table[0x08]  (was 0x09 — off by one)
  0x09,  // RoC   table[0x09]  (was 0x0A — off by one)
  0x06,  // Flor  table[0x06]  (was 0x07 — off by one)
  0x0F,  // Alt.  table[0x0F]  (was 0x16 — pointed at 'Mode')
};

// iBUS telemetry poll packet length
#define TELEM_POLL_LEN      4     // [0x04] [cmd_addr] [ckL] [ckH]

// Shared telemetry values — written by task_logic, read by task_protocol
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
  uint16_t ch[IBUS_NUM_CH];  // decoded iBUS channels 0–13, values 1000–2000
  bool     valid;
};

// ─────────────────────────────────────────────────────────────
//  RMT PPM encoder
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
    if (pw < IBUS_MIN) pw = IBUS_MIN;
    if (pw > IBUS_MAX) pw = IBUS_MAX;
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
  tx_cfg.gpio_num            = (gpio_num_t)PIN_PPM_OUT;
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

// ─────────────────────────────────────────────────────────────
//  iBUS decoder — ring-buffer state machine
//  115200 8N1, idle HIGH, no inversion required.
//  Validates header bytes and 16-bit CRC (0xFFFF - sum).
// ─────────────────────────────────────────────────────────────

static bool ibus_decode(const uint8_t *buf, uint16_t *ch) {
  // Validate header
  if (buf[0] != IBUS_HEADER_0 || buf[1] != IBUS_HEADER_1) return false;
  // Validate CRC: sum of bytes 0–29, CRC = 0xFFFF - sum stored LE at [30][31]
  uint16_t sum = 0;
  for (int i = 0; i < 30; i++) sum += buf[i];
  uint16_t crc = (uint16_t)buf[30] | ((uint16_t)buf[31] << 8);
  if (crc != (uint16_t)(0xFFFF - sum)) return false;
  // Unpack 14 channels LE uint16
  for (int i = 0; i < IBUS_NUM_CH; i++) {
    uint16_t v = (uint16_t)buf[2 + i*2] | ((uint16_t)buf[3 + i*2] << 8);
    ch[i] = constrain(v, (uint16_t)IBUS_MIN, (uint16_t)IBUS_MAX);
  }
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
  if (positive_is_closing) return ch_val > (IBUS_MID + IBUS_DEAD);
  else                      return ch_val < (IBUS_MID - IBUS_DEAD);
}

// Scale stick deflection from neutral by authority (0–1).
static inline uint16_t apply_authority(uint16_t ch_val, float authority) {
  float delta = (float)ch_val - IBUS_MID;
  return (uint16_t)(IBUS_MID + delta * authority);
}

// ─────────────────────────────────────────────────────────────
//  Runtime-tunable gains (BLE-writable)
// ─────────────────────────────────────────────────────────────

static uint16_t g_failsafeRTH     = 1000;   // failsafe PPM value on CH_FLIGHT_MODE slot (RTH = 1000)
static uint16_t g_wdgSensorMs     = WDG_SENSOR_MS;
static uint16_t g_wdgIbusMs       = WDG_IBUS_MS;
static bool     g_avoidEnabled      = true;
static bool     g_horizAvoidEnabled = true;
static bool     g_altCorridorEnabled = true;
static bool     g_mapAvoidEnabled   = true;
static float    g_floorBaseM        = CFG_FLOOR_BASE_M;
static float    g_rememberedFloorM  = 0.0f;
static bool     g_floorActive       = false;
static bool     g_activeFloorArmed  = false;
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
static uint16_t g_freezeVrbRef      = IBUS_MID;
static bool     g_sensoryPresent    = (SENSORY_CORTEX_PRESENT != 0);
static uint32_t g_wdPulseCount      = 0;
static uint32_t g_wdLastCheckMs     = 0;
static uint32_t g_wdLastCount       = 0;
static bool     g_wdOk              = false;

// ── Shared state between tasks (written Core0, read Core1) ────
static volatile bool     rcLost      = false;
// [v2.1.6 I-004] Boot in the SAFE state: assume NO valid sensory data until a
// packet actually arrives (first valid packet clears this at RX). Previously
// this booted false, so there was a window at startup where the all-clear
// default packet was treated as real "clear airspace".
static volatile bool     sensorDead  = true;
static volatile uint32_t lastIbusRxMs   = 0;
static volatile uint32_t lastSafetyPktMs = 0;
// [v2.1.6 I-005] Consecutive good packets seen while sensorDead — avoidance
// authority is only restored after SAFETY_RECOVER_PKTS in a row, so a flapping
// UART link can't snap avoidance back on/off. Touched only in task_protocol.
static uint16_t g_safetyRecoverCnt = 0;
#define SAFETY_RECOVER_PKTS 5   // ~35-100ms of stable link before restoring authority

// ─────────────────────────────────────────────────────────────
//  FreeRTOS queues
// ─────────────────────────────────────────────────────────────

static QueueHandle_t qRxChannels;   // Core0 → Core1: decoded iBUS channels
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

  uint16_t pushUs = (uint16_t)(push * (IBUS_MAX - IBUS_MIN));
  if (away_is_positive) {
    uint32_t target = (uint32_t)IBUS_MID + pushUs;
    if (target > IBUS_MAX) target = IBUS_MAX;
    return (uint16_t)target;
  } else {
    int32_t target = (int32_t)IBUS_MID - (int32_t)pushUs;
    if (target < IBUS_MIN) target = IBUS_MIN;
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
        ch[1] = IBUS_MID; anyBrake = true; pitchLive = true;
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
        ch[0] = IBUS_MID; anyBrake = true; rollLive = true;
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
          ch[0] = IBUS_MID; anyBrake = true;
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
          ch[1] = IBUS_MID; anyBrake = true;
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
  if (anyBrake && ch[2] > IBUS_MID) ch[2] = IBUS_MID;

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
                            IBUS_MIN, IBUS_MAX, SCALE_MIN, SCALE_MAX);
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
        uint16_t addUs = (uint16_t)(add * (IBUS_MAX - IBUS_MIN));
        uint32_t newThr = (uint32_t)ch[2] + addUs;
        if (newThr > IBUS_MAX) newThr = IBUS_MAX;
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
          && ch[2] < (uint16_t)(IBUS_MID - IBUS_DEAD)
          && ch[2] < IBUS_MID) {
        ch[2] = IBUS_MID; floorEnforced = true;
      }
    } else {
      g_afLastAddFrac = 0.0f;
      bool descending = (ch[2] < (uint16_t)(IBUS_MID - IBUS_DEAD));
      if (floor > 0.05f && altM > 0.0f && altM < 8.0f
          && trueHeight < floor && descending) {
        ch[2] = IBUS_MID; floorEnforced = true;
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
    bool climbing = (ch[2] > (uint16_t)(IBUS_MID + IBUS_DEAD));

    bool captureEdge = setSwitchHigh && !g_prevCeilingSet;
    if (captureEdge && altValid && level) g_ceilingM = altM;

    if (setSwitchHigh && g_ceilingM > 0.05f) {
      if (altValid && altBandOk && altM > g_ceilingM && climbing) {
        ch[2] = IBUS_MID;
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
//  "sbus" references in BLE text updated to "ibus" for clarity.
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
      if (sval == "on")  { g_sensoryPresent = true;  c->setValue("sensory on"); }
      if (sval == "off") { g_sensoryPresent = false; c->setValue("sensory off"); }
    }
    else if (uuid == BLE_CC_AVOIDANCE_UUID) {
      // ── Active floor ──
      if (sval == "afloor arm") {
        g_activeFloorArmed = true; c->setValue("afloor ARMED"); return;
      } else if (sval == "afloor disarm" || sval == "afloor off") {
        g_activeFloorArmed = false; c->setValue("afloor disarmed"); return;
      } else if (sval.startsWith("afloor cap ")) {
        float v = sval.substring(11).toFloat();
        if (v >= 0.0f && v <= 0.6f) { g_afAuthCap = v;
          char b[24]; snprintf(b,24,"afloor cap %.2f",v); c->setValue(b); } return;
      } else if (sval.startsWith("afloor band ")) {
        float v = sval.substring(12).toFloat();
        if (v >= 0.2f && v <= 6.0f) { g_afEngageBandM = v;
          char b[24]; snprintf(b,24,"afloor band %.1f",v); c->setValue(b); } return;
      } else if (sval.startsWith("afloor kp ")) {
        float v = sval.substring(10).toFloat();
        if (v >= 0.0f && v <= 5.0f) { g_afKp = v;
          char b[24]; snprintf(b,24,"afloor kp %.2f",v); c->setValue(b); } return;
      } else if (sval.startsWith("afloor kd ")) {
        float v = sval.substring(10).toFloat();
        if (v >= 0.0f && v <= 5.0f) { g_afKd = v;
          char b[24]; snprintf(b,24,"afloor kd %.2f",v); c->setValue(b); } return;
      } else if (sval.startsWith("afloor bank ")) {
        float v = sval.substring(12).toFloat();
        if (v >= 5.0f && v <= 30.0f) { g_afMaxBankDeg = v;
          char b[24]; snprintf(b,24,"afloor bank %.0f",v); c->setValue(b); } return;
      } else if (sval == "afloor status" || sval == "afloor") {
        char b[80];
        snprintf(b,80,"afloor %s cap=%.2f band=%.1f kp=%.2f kd=%.2f bank=%.0f",
          g_activeFloorArmed?"ARMED":"disarmed",
          g_afAuthCap, g_afEngageBandM, g_afKp, g_afKd, g_afMaxBankDeg);
        c->setValue(b); return;
      } else if (sval.startsWith("afloor")) { return; }  // unknown — ignore

      // ── Active horizontal brake ──
      if (sval == "ahoriz arm") {
        g_activeHorizArmed = true; c->setValue("ahoriz ARMED"); return;
      } else if (sval == "ahoriz disarm" || sval == "ahoriz off") {
        g_activeHorizArmed = false; c->setValue("ahoriz disarmed"); return;
      } else if (sval.startsWith("ahoriz cap ")) {
        float v = sval.substring(11).toFloat();
        if (v >= 0.0f && v <= 0.6f) { g_ahAuthCap = v;
          char b[24]; snprintf(b,24,"ahoriz cap %.2f",v); c->setValue(b); } return;
      } else if (sval.startsWith("ahoriz kp ")) {
        float v = sval.substring(10).toFloat();
        if (v >= 0.0f && v <= 2.0f) { g_ahKp = v;
          char b[24]; snprintf(b,24,"ahoriz kp %.2f",v); c->setValue(b); } return;
      } else if (sval.startsWith("ahoriz kd ")) {
        float v = sval.substring(10).toFloat();
        if (v >= 0.0f && v <= 2.0f) { g_ahKd = v;
          char b[24]; snprintf(b,24,"ahoriz kd %.2f",v); c->setValue(b); } return;
      } else if (sval.startsWith("ahoriz vref ")) {
        float v = sval.substring(12).toFloat();
        if (v >= 50.0f && v <= 1000.0f) { g_ahVelRefCms = v;
          char b[24]; snprintf(b,24,"ahoriz vref %.0f",v); c->setValue(b); } return;
      } else if (sval.startsWith("ahoriz engage ")) {
        float v = sval.substring(14).toFloat();
        if (v >= 0.0f && v <= 200.0f) { g_ahEngageCms = v;
          char b[24]; snprintf(b,24,"ahoriz engage %.0f",v); c->setValue(b); } return;
      } else if (sval == "ahoriz status" || sval == "ahoriz") {
        char b[80];
        snprintf(b,80,"ahoriz %s cap=%.2f kp=%.2f kd=%.2f vref=%.0f eng=%.0f",
          g_activeHorizArmed?"ARMED":"disarmed",
          g_ahAuthCap, g_ahKp, g_ahKd, g_ahVelRefCms, g_ahEngageCms);
        c->setValue(b); return;
      } else if (sval.startsWith("ahoriz")) { return; }

      // ── Master avoidance switches ──
      if      (sval == "on")         { g_avoidEnabled = true;       c->setValue("avoid on"); }
      else if (sval == "off")        { g_avoidEnabled = false;      c->setValue("avoid off"); }
      else if (sval == "horiz on")   { g_horizAvoidEnabled = true;  c->setValue("horiz on"); }
      else if (sval == "horiz off")  { g_horizAvoidEnabled = false; c->setValue("horiz off"); }
      else if (sval == "alt on")     { g_altCorridorEnabled = true; c->setValue("alt on"); }
      else if (sval == "alt off")    { g_altCorridorEnabled = false;c->setValue("alt off"); }
      else if (sval == "map on")     { g_mapAvoidEnabled = true;    c->setValue("map on"); }
      else if (sval == "map off")    { g_mapAvoidEnabled = false;   c->setValue("map off"); }
    }
    else if (uuid == BLE_CC_FAILSAFE_UUID) {
      int v = sval.toInt();
      if (v >= IBUS_MIN && v <= IBUS_MAX) {
        g_failsafeRTH = (uint16_t)v;
        char buf[8]; snprintf(buf, sizeof(buf), "%d", g_failsafeRTH);
        c->setValue(buf);
      }
    }
    else if (uuid == BLE_CC_TIMEOUTS_UUID) {
      // Format: "sensor:50 ibus:100"
      int si = sval.indexOf("sensor:"); int ii = sval.indexOf("ibus:");
      if (si >= 0) { int v = sval.substring(si+7).toInt(); if (v >= 20) g_wdgSensorMs = (uint16_t)v; }
      if (ii >= 0) { int v = sval.substring(ii+5).toInt(); if (v >= 50) g_wdgIbusMs   = (uint16_t)v; }
      char buf[32]; snprintf(buf, sizeof(buf), "sensor:%d ibus:%d",
                             g_wdgSensorMs, g_wdgIbusMs);
      c->setValue(buf);
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
//  iBUS telemetry — half-duplex sensor responder
//  Called from task_protocol (Core 0) every loop iteration.
//  Listens on Serial2 for poll packets from the TX, responds
//  in-place within microseconds (UART TX FIFO handles timing).
//
//  FlySky iBUS telemetry protocol (IBusBM-verified encoding):
//
//    Poll packet (4 bytes total):
//      [0x04] [cmd_addr] [ckL] [ckH]
//      cmd_addr = (addr & 0x0F) | cmd_class
//      cmd_class: 0x80=discover  0x90=type query  0xA0=value query
//      addr: 0x00–0x0F (sensor slot number, low nibble)
//      CRC: 0xFFFF − (byte[0] + byte[1]), stored LE
//
//    Discover reply (4 bytes):
//      [0x04] [0x80|addr] [ckL] [ckH]
//
//    Type reply (6 bytes):
//      [0x06] [0x90|addr] [type_id] [0x02] [ckL] [ckH]
//      type_id maps to 4-char label in TX sensor table
//      0x02 = int16 value size
//
//    Value reply (6 bytes):
//      [0x06] [0xA0|addr] [valL] [valH] [ckL] [ckH]
//      value is int16, little-endian, signed
//
//  We respond only to addresses TELEM_ADDR_BASE..ADDR_BASE+N-1.
//  All other addresses silently ignored.
// ─────────────────────────────────────────────────────────────

static inline uint16_t telem_ck(const uint8_t *buf, uint8_t len) {
  uint16_t sum = 0;
  for (uint8_t i = 0; i < len; i++) sum += buf[i];
  return 0xFFFF - sum;
}

static void telem_poll_handle() {
  while (Serial2.available() >= TELEM_POLL_LEN) {

    // Sync on length byte 0x04
    if (Serial2.peek() != 0x04) { Serial2.read(); continue; }

    uint8_t poll[TELEM_POLL_LEN];
    for (int i = 0; i < TELEM_POLL_LEN; i++) poll[i] = Serial2.read();

    // Verify checksum covers bytes 0–1
    uint16_t ck_rx   = (uint16_t)poll[2] | ((uint16_t)poll[3] << 8);
    uint16_t ck_calc  = telem_ck(poll, 2);
    if (ck_rx != ck_calc) continue;

    uint8_t addr      = poll[1] & 0x0F;   // sensor slot 0–15
    uint8_t cmd_class = poll[1] & 0xF0;   // 0x80 / 0x90 / 0xA0

    // Not our address?
    if (addr < TELEM_ADDR_BASE ||
        addr >= (uint8_t)(TELEM_ADDR_BASE + TELEM_NUM_SENSORS)) continue;
    uint8_t slot = addr - TELEM_ADDR_BASE;

    if (cmd_class == 0x80) {
      // Discover — confirm presence
      uint8_t r[4];
      r[0] = 0x04; r[1] = 0x80 | addr;
      uint16_t ck = telem_ck(r, 2);
      r[2] = ck & 0xFF; r[3] = ck >> 8;
      Serial2.write(r, 4);

    } else if (cmd_class == 0x90) {
      // Type query — report sensor type ID (2-byte int16 value)
      uint8_t r[6];
      r[0] = 0x06; r[1] = 0x90 | addr;
      r[2] = TELEM_TYPE[slot]; r[3] = 0x02;
      uint16_t ck = telem_ck(r, 4);
      r[4] = ck & 0xFF; r[5] = ck >> 8;
      Serial2.write(r, 6);

    } else if (cmd_class == 0xA0) {
      // Value query — send current telemetry value
      int16_t val;
      switch (slot) {
        case 0:  val = g_telZone;   break;
        case 1:  val = g_telObsCm;  break;
        case 2:  val = g_telHead;   break;
        case 3:  val = g_telRocCms; break;
        case 4:  val = g_telFlorCm; break;
        default: val = g_telAltCm;  break;  // slot 5
      }
      uint8_t r[6];
      r[0] = 0x06; r[1] = 0xA0 | addr;
      r[2] = (uint8_t)(val & 0xFF); r[3] = (uint8_t)((val >> 8) & 0xFF);
      uint16_t ck = telem_ck(r, 4);
      r[4] = ck & 0xFF; r[5] = ck >> 8;
      Serial2.write(r, 6);
    }
    // Unknown cmd_class: silently ignore
  }
}

// ─────────────────────────────────────────────────────────────
//  CORE 0 TASK — task_protocol
//  iBUS receive, SafetyPacket RX/TX, watchdog pulse
//  Pinned to Core 0.
// ─────────────────────────────────────────────────────────────

static void task_protocol(void *) {
  uint8_t  ibusRxBuf[IBUS_FRAME_LEN];
  uint8_t  ibusRxPos  = 0;
  bool     ibusSync   = false;

  esp_task_wdt_add(NULL);        // v2.1.5: subscribe this task to the TWDT

  for (;;) {
    esp_task_wdt_reset();        // v2.1.5: pet — must be reached every iteration
    uint32_t now = millis();

    // ── iBUS RX ──────────────────────────────────────────────
    // Frame sync: wait for 0x20 0x40 header pair.
    // iBUS frames arrive at ~142Hz (7ms). No inter-frame gap needed
    // (unlike SBUS) — header bytes provide sync.
    while (Serial.available()) {
      uint8_t b = Serial.read();

      if (!ibusSync) {
        if (ibusRxPos == 0 && b == IBUS_HEADER_0) {
          ibusRxBuf[0] = b; ibusRxPos = 1;
        } else if (ibusRxPos == 1 && b == IBUS_HEADER_1) {
          ibusRxBuf[1] = b; ibusRxPos = 2; ibusSync = true;
        } else {
          ibusRxPos = 0;  // reset on bad second byte
        }
        continue;
      }

      ibusRxBuf[ibusRxPos++] = b;

      if (ibusRxPos == IBUS_FRAME_LEN) {
        ibusSync  = false;
        ibusRxPos = 0;

        ChannelFrame cf;
        cf.valid = ibus_decode(ibusRxBuf, cf.ch);
        if (cf.valid) {
          lastIbusRxMs = millis();
          __sync_synchronize();  // barrier before flag write
          rcLost = false;
          xQueueOverwrite(qRxChannels, &cf);
        }
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

    // ── iBUS telemetry — respond to TX sensor polls ───────────
    telem_poll_handle();

    // ── iBUS link watchdog ────────────────────────────────────
    if (millis() - lastIbusRxMs > g_wdgIbusMs) {
      __sync_synchronize();  // barrier before flag write
      rcLost = true;
      // Push PPM failsafe frame — RTH on CH_FLIGHT_MODE slot, RMT transmits next cycle
      ppm_failsafe();
      // Reset iBUS sync state so recovery starts clean on next valid frame.
      // Without this, a partial frame buffered at signal loss would cause
      // one spurious CRC-fail decode attempt before resync on resume.
      ibusSync  = false;
      ibusRxPos = 0;
    }

    // ── RMT stall detection ───────────────────────────────────
    // If ppm_tx_done_cb() fails to re-queue a frame, g_rmtStalled is set.
    // PPM output has stopped. The hardware watchdog (GPIO5 → CD4053) will
    // engage within ~100ms routing raw receiver PPM to the Naza.
    // Log the event — this should never happen in normal operation.
    if (g_rmtStalled) {
      g_rmtStalled = false;
      USBSerial.println("[ERR] RMT re-queue failed — PPM output stalled. HW watchdog active.");
    }

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

    // Wait for fresh iBUS frame (up to 20ms)
    if (xQueueReceive(qRxChannels, &cf, pdMS_TO_TICKS(20)) != pdTRUE) {
      // No iBUS data — RC lost or startup. Force aux outputs safe.
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
    uint16_t ch[IBUS_NUM_CH];
    memcpy(ch, cf.ch, sizeof(ch));

    // ── Landing mode ─────────────────────────────────────────
    landingMode = (ch[CH_LANDING] > LANDING_THRESHOLD);

    // ── Proximity VRB → horizontal scale + freeze logic ──────
    {
      uint16_t vrb = ch[CH_PROX_VRB];
      float vrbScale = fmap((float)vrb, IBUS_MIN, IBUS_MAX, SCALE_MIN, SCALE_MAX);

      if (g_prevLanding && !landingMode) {
        g_hScaleFrozen = true;
        g_freezeVrbRef = vrb;
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

    // ── Push modified ch1–8 to RMT PPM output ────────────────
    // ch[0]–ch[7] = Roll, Pitch, Throttle, Yaw, ch[CH_FLIGHT_MODE]=FlightMode, IOC, Aux, Bypass
    // ch[7] (bypass) is passed through unmodified — the upstream hardware
    // uses it but Naza also sees it and ignores it on PPM (it's ch8).
    if (!rcLost) {
      ppm_update(ch);    // ch[0..7] → PPM channels 1–8
    }
    // RC lost: ppm_failsafe() already called by task_protocol watchdog

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

    // ── ProximityCmd → Sensory Cortex (~14Hz at 142Hz iBUS rate) ──
    if (++proxCmdCounter >= 10) {
      proxCmdCounter = 0;
      ProximityCmd pc = build_prox_cmd(scale, landingMode);
      xQueueOverwrite(qProxCmd, &pc);
    }

    // ── iBUS telemetry values (written for task_protocol ISR) ─
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
  USBSerial.println("[BOOT] Communication Cortex v2.1.6");
  USBSerial.println("[INFO] iBUS input / RMT PPM output");

  // ── Disable WiFi ─────────────────────────────────────────
  esp_wifi_stop();
  esp_wifi_deinit();

  // ── iBUS UART (UART0, Serial) ─────────────────────────────
  // 115200 8N1, idle HIGH, no inversion — direct from FS-X6B iBUS pad
  Serial.begin(BAUD_IBUS, SERIAL_8N1, PIN_IBUS_RX, -1);  // RX only
  USBSerial.printf("[OK] iBUS UART 115200 8N1 GPIO%d\n", PIN_IBUS_RX);

  // ── Sensory Cortex UART (UART1, Serial1) ─────────────────
  Serial1.begin(BAUD_SENSE, SERIAL_8N1, PIN_SENSE_RX, PIN_SENSE_TX);
  USBSerial.printf("[OK] Sensory UART 921600 GPIO%d/%d (%s)\n",
    PIN_SENSE_RX, PIN_SENSE_TX,
    g_sensoryPresent ? "expected" : "absent — avoidance disabled");

  // ── RMT PPM output ────────────────────────────────────────
  ppm_rmt_init();
  USBSerial.printf("[OK] RMT PPM output GPIO%d 50Hz 8ch 1µs resolution\n",
    PIN_PPM_OUT);

  // ── iBUS telemetry UART (UART2, Serial2) ─────────────────
  // Half-duplex: RX listens for TX polls, TX replies to them.
  // Both GPIO20 and GPIO21 connect to the FS-X6B iBUS sensor port.
  // GPIO21 TX → 1kΩ series resistor → iBUS wire (limits drive current,
  // prevents bus fight when TX and receiver both pull the line).
  // GPIO20 RX ← iBUS wire direct.
  Serial2.begin(BAUD_TELEM, SERIAL_8N1, PIN_TELEM_RX, PIN_TELEM_TX);
  USBSerial.printf("[OK] iBUS telemetry UART GPIO%d(RX) GPIO%d(TX) 115200\n",
    PIN_TELEM_RX, PIN_TELEM_TX);

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
  lastIbusRxMs    = millis();
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
  USBSerial.println("[INFO] Communication Cortex v2.1.4 running");
  USBSerial.println("[INFO] iBUS in: 14ch 1000-2000us");
  USBSerial.println("[INFO] PPM out: 8ch RMT GPIO6");
  USBSerial.printf( "[INFO] Flood GPIO%d  VTX GPIO%d\n", PIN_FLOOD, PIN_VTX);
}

void loop() {
  vTaskDelay(pdMS_TO_TICKS(1000));
}

/*
 * ================================================================
 *  v2.0 GPIO SUMMARY
 * ================================================================
 *
 *  GPIO4   iBUS RX from FS-X6B (115200 8N1, direct 3.3V, no inversion)
 *  GPIO5   50Hz watchdog pulse → 47kΩ + 1µF → CD4053 Switch A SELECT
 *  GPIO6   RMT PPM output → CD4053 IN-B (3.3V idle-HIGH)
 *  GPIO1   Floodlight FET gate (ch10, active HIGH)
 *  GPIO2   VTX FET gate       (ch11, active HIGH)
 *  GPIO7   Nav LED Front-Left  (via 2N2222/HCT04 inverter, ch12)
 *  GPIO8   Nav LED Front-Right (via 2N2222/HCT04 inverter, ch12)
 *  GPIO9   Nav LED Rear-Left   (via 2N2222/HCT04 inverter, ch12)
 *  GPIO10  Nav LED Rear-Right  (via 2N2222/HCT04 inverter, ch12)
 *  GPIO16  Sensory Cortex UART RX
 *  GPIO17  Sensory Cortex UART TX
 *  GPIO20  iBUS telemetry RX (Serial2 RX ← FS-X6B sensor port)
 *  GPIO21  iBUS telemetry TX (Serial2 TX → 1kΩ → FS-X6B sensor port)
 *  GPIO48  WS2812 status LED
 *
 *  PREVIOUSLY USED, NOW REASSIGNED:
 *  GPIO20  (was SBUS RX)  → now iBUS telemetry RX
 *  GPIO21  (was SBUS TX)  → now iBUS telemetry TX
 *
 * ================================================================
 *  CD4053 WIRING (v2.0 — all 3.3V, no level shift)
 * ================================================================
 *
 *  Switch B (PPM selector):
 *    IN-A  ← Raw PPM from FS-X6B PPM output (3.3V)
 *    IN-B  ← ESP PPM from GPIO6 (3.3V)
 *    OUT   → Naza M V2 PPM port
 *    SELECT ← Switch A OUT
 *
 *  Switch A (watchdog gate):
 *    SELECT ← GPIO5 RC filter (47kΩ + 1µF) + Pololu RC switch (ch8 PWM)
 *    OUT   → Switch B SELECT
 *    INH   → GND, VCC → 3.3V, GND → GND
 *
 *  74HCT04 (v2.0):
 *    Gates 1+2 (SBUS) — UNUSED, inputs tie to GND
 *    Gates 3–6 (nav LEDs) — retained as before
 *    If replacing with 2N2222 transistors for nav LEDs, 74HCT04 not needed at all
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
 *  BLE COMMANDS (CommCortex) — UNCHANGED from v1.9.2
 * ================================================================
 *
 *  Status/Zone/Watchdog: read-only, notify every 1s
 *  Sensory: on / off
 *  Avoidance: on / off / horiz on|off / alt on|off / map on|off
 *             afloor arm|disarm|cap|band|kp|kd|bank|status
 *             ahoriz arm|disarm|cap|kp|kd|vref|engage|status
 *  Failsafe RTH: PPM value 1000–2000 (default 1000 = RTH low)
 *  Timeouts: "sensor:50 ibus:100"
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
