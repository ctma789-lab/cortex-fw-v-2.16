/*
 * ================================================================
 *  DUAL CORTEX DRONE — SENSORY CORTEX FIRMWARE v2.0.4
 * ================================================================
 *  Target: ESP32-S3 DevKit N16R8 (16MB flash, 8MB PSRAM)
 *
 *  Perceives the world and streams obstacle data to the Comm Cortex.
 *
 *  Sensors (all on one I2C bus, SDA=8 SCL=9, plus one SPI device):
 *    - 5x TF-Luna LiDAR: front/left/right/back/down (0x10-0x14)
 *    - ICM-42688-P 6-axis IMU (0x68)
 *    - HMC5883L magnetometer (0x1E)
 *    - BMP280 barometer (0x76)
 *    - PMW3901 optical flow (SPI)
 *
 *  Processing:
 *    - Mahony AHRS (9-DOF) → attitude quaternion → NED rotation matrix.
 *    - Per-sensor approach velocity from LiDAR distance deltas, and
 *      time-to-impact (TTE) for the avoidance system.
 *    - Fused altitude: TF-Luna Down + BMP280 (complementary filter).
 *    - Optical flow velocity with rotational compensation from gyro.
 *    - Inertial dead-reckoning (gravity-removed) for the 3D map,
 *      corrected by optical flow and TF-Luna altitude ground truth.
 *    - PSRAM-backed 5000-point spatial obstacle map.
 *    - Map-based potential field: repulsion vector from mapped
 *      obstacles (1/d²), gated on dead-reckoning confidence, sent
 *      to the Comm Cortex as a secondary body-frame nudge.
 *    - Plain-text BLE interface ("SensoryCortex") with guided
 *      compass and optical-flow calibration.
 *
 *  SafetyPacket (34 bytes, version + XOR checksum) streamed to Comm Cortex over UART1.
 *
 *  Arduino IDE: ESP32S3 Dev Module, PSRAM = OPI PSRAM (REQUIRED),
 *  Flash 16MB, USB CDC On Boot Enabled, Huge APP partition.
 *  Libraries: ICM42688 (Finani), Adafruit BMP280, Bitcraze PMW3901,
 *  ESP32 BLE.
 * ================================================================
 */

#include <Wire.h>
#include <ICM42688.h>
#include <math.h>
#include <esp_wifi.h>     // WiFi disable at boot
#include <BLEDevice.h>    // BLE configuration interface
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include <Preferences.h>  // NVS flash storage for persistent tunings
#include "esp_task_wdt.h" // v2.0.2: hardware Task Watchdog (TWDT)
// Adafruit_BMP280.h and Bitcraze_PMW3901.h included in their driver sections

// ── Pins ─────────────────────────────────────────────────────
#define I2C_SDA        8
#define I2C_SCL        9
#define UART_TX       43    // Serial1 TX → Comm Cortex
#define UART_RX       44    // Serial1 RX ← Comm Cortex
#define COMM_CMD_RX    1    // Serial2 RX ← ProximityCmd from Comm Cortex
#define COMM_CMD_TX    2    // Serial2 TX  (reserved, not used yet)
// GPIO 3 reserved: Visual Cortex UART RX (future)

// ── Optical flow sensor (PMW3901 / PAA5100) — SPI ────────────
// Downward-facing, reads ground-relative velocity in X/Y (pixels/s)
// Corrects dead-reckoning drift in the position integrator.
#define OF_SPI_CS     15    // Chip select — safe GPIO, not flash-connected
#define OF_SPI_MOSI    6    // SPI MOSI
#define OF_SPI_MISO    5    // SPI MISO
#define OF_SPI_SCK     4    // SPI clock
// Set to -1 to disable optical flow (sensor not fitted):
#define OF_ENABLED      1   // 1=fitted, 0=not fitted

// ── Rearward TF-Luna — I2C ───────────────────────────────────
// Reassign from factory 0x10 to 0x13 using TF-Luna config tool.
#define LUNA_BACK   0x13
#define LUNA_BACK_ENABLED 1  // 1=fitted, 0=not fitted

// ── Downward TF-Luna (altitude) — I2C ────────────────────────
// Reassign from factory 0x10 to 0x14 using TF-Luna config tool.
#define LUNA_DOWN   0x14
#define LUNA_DOWN_ENABLED 1  // 1=fitted, 0=not fitted

// ================================================================
//  SENSOR CONFIGURATION TABLE
//  For each sensor: _ADDR = I2C address, _ENABLED = 1 fitted / 0 not
//  TF-Lunas ship at 0x10 — reassign each before fitting (see handbook)
// ================================================================

// TF-Luna Front — forward LiDAR, body +X axis
#define LUNA_FRONT  0x10
#define LUNA_FRONT_ENABLED 1
// TF-Luna Left — left LiDAR, body +Y axis
#define LUNA_LEFT   0x11
#define LUNA_LEFT_ENABLED  1
// TF-Luna Right — right LiDAR, body -Y axis
#define LUNA_RIGHT  0x12
#define LUNA_RIGHT_ENABLED 1
// (LUNA_BACK 0x13 and LUNA_DOWN 0x14 defined above with their enables)

// ICM-42688-P — 6-axis IMU, fixed address, always required
#define ICM_ADDR    0x68
// HMC5883L — magnetometer, fixed address
#define HMC_ADDR    0x1E
// BMP280 — barometer: 0x76 (SDO low) or 0x77 (SDO high)
#define BMP_ADDR    0x76
#define BMP_ENABLED 1

// ================================================================
//  USER CONFIGURATION — edit or tune via BLE without reflashing
//  BLE characteristic UUIDs are listed in the BLE section below.
//  Changes made via BLE are live immediately but NOT persisted
//  across power cycles unless you edit these defaults too.
// ================================================================

// ── Map point expiry ─────────────────────────────────────────
// How long a point stays in the map after the last time it was
// scanned. Without optical flow, 30s is honest (drift degrades
// accuracy beyond this). With optical flow, 180s is reasonable.
// Range: 5000 (5s) to 600000 (10 min). Configurable via BLE.
#define CFG_I2C_SPEED            400000UL  // 400kHz — safe for all sensors
#define CFG_MAG_DECLINATION_DEG     -0.8f  // update for your location
#define CFG_POINT_TTL_MS_DEFAULT   30000UL  // 30 seconds

// ── Safety thresholds (base — scaled by ch9 VRB) ─────────────
#define CFG_THRESH_BASE_STATIC_CM   60.0f   // hard brake boundary
#define CFG_THRESH_BASE_DYNAMIC_CM 200.0f   // fast-approach threshold

// ── Optical flow velocity weight ─────────────────────────────
// How much optical flow corrects the dead-reckoning velocity.
// 1.0 = fully trust optical flow, 0.0 = ignore it (accel only).
// 0.8 works well when OF sensor is calibrated and level.
#define CFG_OF_WEIGHT               0.8f

// ── Barometer fusion weight & mounting offset ────────────────
// CFG_BARO_WEIGHT: how much the baro contributes to fused altitude
//   (0.0 = TF-Luna only, 1.0 = baro only). 0.6 is a good default.
// CFG_BARO_OFFSET_M: altitude trim for baro mounting (metres).
#define CFG_BARO_WEIGHT             0.6f
#define CFG_BARO_OFFSET_M           0.0f

// ── Optical flow field-of-view scale (pixels/s → m/s) ────────
// Depends on sensor height above ground and lens focal length.
// For PMW3901 at ~50cm AGL: ~0.0015 is a reasonable starting point.
// Calibrate by flying a known distance and adjusting.
#define CFG_OF_SCALE             0.0015f

// ── CLOSING VELOCITY FUSION (v1.9.2) ──────────────────────────
// The raw per-sensor closing velocity is a finite difference on consecutive
// TF-Luna distances — noisy, and it only measures the LiDAR beam delta (it
// misses the drone's own motion toward a static obstacle when the distance
// reading is dominated by something else). v1.9.2 improves it with:
//   • OUTLIER REJECTION: reject implausible single-sample jumps (a glint or
//     dropout) before they reach the velocity.
//   • LOW-PASS FILTER: tunable α exponential smoothing.
//   • INERTIAL/OF PROJECTION: the drone's body velocity (from optical flow,
//     forward and right, m/s, projected onto each sensor axis, ADDED to the
//     LiDAR delta → true relative closing speed. Confidence-weighted by OF
//     health: full weight when OF is good, fading to ZERO (LiDAR-only) when
//     OF is stale/untrusted, so OF noise never contaminates the estimate.
// All tunable via BLE (cvel alpha / gate / inertial).
#define CFG_CV_ALPHA       0.4f    // LP filter weight on NEW sample, 0 to 1
#define CFG_CV_OUTLIER_CMS 600.0f  // reject per-sample delta implying > this cm/s
#define CFG_CV_INERTIAL_W  0.7f    // max weight of OF-projected term, 0 to 1

// ── Altitude sensor mounting offset ──────────────────────────
// If the downward TF-Luna is not exactly at the drone's CoM,
// add the offset here (metres, positive = sensor is below CoM).
#define CFG_ALT_OFFSET_M            0.0f

// ================================================================
//  END USER CONFIGURATION
// ================================================================

// ── Safety thresholds (base values — scaled by Comm Cortex ch9 VRB) ─────────
static constexpr float THRESH_BASE_STATIC_CM  = CFG_THRESH_BASE_STATIC_CM;
static constexpr float THRESH_BASE_DYNAMIC_CM = CFG_THRESH_BASE_DYNAMIC_CM;
static constexpr float TTE_WARN_S             =   1.0f;
static constexpr float TTE_CRIT_S             =   0.5f;

// ── Dynamic thresholds — updated by ProximityCmd from Comm Cortex ─────────
static float g_threshStaticCm  = THRESH_BASE_STATIC_CM;
static float g_threshDynamicCm = THRESH_BASE_DYNAMIC_CM;
static bool  g_landingMode      = false;
static float g_proxScale        = 1.0f;

// ── Map-based potential field (secondary avoidance layer) ─────────────────
// Queries the persistent 3D map within a VRB-scaled bubble around the drone
// and computes a repulsion vector from mapped obstacles, weighted by 1/d².
// This lets the drone avoid obstacles it mapped earlier even when no sensor
// currently points at them. STRICTLY SECONDARY to live-sensor avoidance:
//   - Output is capped to a low magnitude (influence-level only, never brake).
//   - Disabled automatically when dead-reckoning confidence is low (optical
//     flow dead or altitude invalid), since map accuracy depends on pose.
//   - The Comm Cortex treats it as a gentle nudge a live sensor can override.
// BLE-toggleable; defaults ON but heavily constrained.
#define CFG_MAP_BUBBLE_BASE_M    3.0f   // bubble radius at VRB scale 1.0×
#define CFG_MAP_FORCE_FALLOFF_M  0.5f   // 1/d² softening (avoids singularity)
#define CFG_MAP_FORCE_CAP        1.0f   // max |force| (normalised units)
static bool  g_mapAvoidEnabled  = true; // BLE-toggleable
static bool  g_mapForceValid    = false;// true when last force was trustworthy
static float g_mapForceN        = 0.0f; // world-frame repulsion, North
static float g_mapForceE        = 0.0f; // world-frame repulsion, East

// ── Runtime config — tunable via BLE without reflashing ──────
// These shadow the CFG_ compile-time defaults and are the live values.
static uint32_t g_pointTTLms   = CFG_POINT_TTL_MS_DEFAULT;
static float    g_ofWeight      = CFG_OF_WEIGHT;
static float    g_ofScale       = CFG_OF_SCALE;
static float    g_altOffsetM    = CFG_ALT_OFFSET_M;
static float    g_baroWeight    = CFG_BARO_WEIGHT;   // live baro fusion weight

// ── Barometer (BMP280) state ──────────────────────────────────
static float    g_baroRefPa     = 0.0f;   // ground reference pressure (Pa)
static float    g_baroRawM      = 0.0f;   // raw baro altitude (m)
static float    g_baroAltM      = 0.0f;   // filtered baro altitude (m)
static bool     g_baroOk        = false;  // true = baro reading valid
static constexpr float g_baroAlpha = 0.1f; // baro low-pass filter coefficient

// ── Optical flow state ────────────────────────────────────────
static float    of_vx = 0.0f;   // world-frame velocity X (m/s) from optical flow
static float    of_vy = 0.0f;   // world-frame velocity Y (m/s) from optical flow
// ── Closing velocity fusion state (v1.9.2) ──
static float    of_body_vx = 0.0f;  // BODY-frame forward velocity (m/s), OF
static float    of_body_vy = 0.0f;  // BODY-frame right   velocity (m/s), OF
static float    g_cvAlpha     = CFG_CV_ALPHA;       // BLE-tunable LP weight
static float    g_cvOutlierCms= CFG_CV_OUTLIER_CMS; // BLE-tunable outlier gate
static float    g_cvInertialW = CFG_CV_INERTIAL_W;  // BLE-tunable inertial weight
static bool     of_ok  = false;  // true when optical flow data is fresh
static uint32_t of_lastMs = 0;  // timestamp of last valid OF reading

// ── Altitude state ────────────────────────────────────────────
static float    g_altM    = 0.0f;   // altitude above ground (metres)
static float    g_vertVelMS = 0.0f; // vertical velocity (m/s), +ve = climbing.
                                    // Derived from corrected altitude (g_altM),
                                    // NOT from drifting accel-integrated vel.z.
static float    g_prevAltM  = -1.0f;// previous altitude for differentiation
static bool     g_altOk   = false;  // true when altitude sensor valid
static uint8_t  altFailCt = 0;      // consecutive altitude read failures

// ── ProximityCmd packet (from Comm Cortex, matches comm_cortex_firmware.ino) ──
struct __attribute__((packed)) ProximityCmd {
  uint8_t start;        // 0xCC
  float   scale;        // 0.3–2.0
  uint8_t landingMode;  // 0 or 1
  uint8_t checksum;     // XOR of 4 scale bytes + landingMode
  uint8_t end;          // 0x55
};
static_assert(sizeof(ProximityCmd) == 8, "ProximityCmd size");
#define PKT_PROX_CMD 0xCC
#define PKT_END      0x55
#define SAFETY_PKT_VERSION 0x01   // bump when SafetyPacket layout changes (v2.0.0)

// ── PSRAM configuration ───────────────────────────────────────
// Set USE_PSRAM 1 if board has PSRAM (N16R8, N8R8 etc).
// Set USE_PSRAM 0 for boards without PSRAM — falls back to
// static arrays in internal SRAM with smaller MAP_SIZE.
// Arduino IDE: Tools → PSRAM → "OPI PSRAM" must also be enabled.
#define USE_PSRAM  1

// ── Spatial map ───────────────────────────────────────────────
#if USE_PSRAM
  // PSRAM: 5000 points × 16 bytes = 80 KB in PSRAM
  // hashGrid stays in SRAM — it's the hot-path index
  static constexpr uint16_t MAP_SIZE   = 5000;
  static constexpr uint16_t HASH_SIZE  = 16384; // [v2.0.3] power of 2 > MAP_SIZE×2.
                                                // Was 8192: 5000/8192=0.61 load factor,
                                                // violating the <0.5 the probe logic assumes.
#else
  // No PSRAM: conservative sizes fitting in internal SRAM
  static constexpr uint16_t MAP_SIZE   = 500;
  static constexpr uint16_t HASH_SIZE  = 1024;
#endif
static constexpr float    VOXEL_M          = 0.20f;  // 20 cm grid step
// Point TTL is runtime-configurable via BLE — see g_pointTTLms
static constexpr uint8_t  MAP_TX_EVERY     = 20;     // transmit map every Nth loop

// ── Hash grid ─────────────────────────────────────────────────
// HASH_SIZE defined above (conditionally on USE_PSRAM).
// hashGrid kept in internal SRAM via DRAM_ATTR — it is the
// hot-path index accessed on every map_push(). Never move to PSRAM.
static constexpr uint16_t HASH_EMPTY = 0xFFFF;

// ── Mahony AHRS gains ─────────────────────────────────────────
static constexpr float MAHONY_KP = 2.0f;
static constexpr float MAHONY_KI = 0.005f;

// ── Position integration ──────────────────────────────────────
static constexpr float HP_TAU_S  = 60.0f;   // high-pass time constant (s)
static constexpr float ACCEL_DEAD = 0.05f;  // dead-band (m/s²)

// ── Magnetic declination ─────────────────────────────────────
// [v2.0.3 S-007] Single source of truth: CFG_MAG_DECLINATION_DEG (line ~119).
// COS_DECL/SIN_DECL are computed ONCE in setup() from that one value, so
// changing your location means editing CFG_MAG_DECLINATION_DEG and nothing
// else. Previously these were three independent hand-typed constants and the
// CFG define was dead — editing it did nothing.
static float MAG_DECL_RAD = CFG_MAG_DECLINATION_DEG * 0.017453293f; // deg→rad
static float COS_DECL = 1.0f;   // set in setup(): cosf(MAG_DECL_RAD)
static float SIN_DECL = 0.0f;   // set in setup(): sinf(MAG_DECL_RAD)

// ── HMC5883L hard-iron offsets ────────────────────────────────
// Calibrate per-drone. See procedure at bottom of file.
static float HMC_OFF_X = 0.0f;
static float HMC_OFF_Y = 0.0f;
static float HMC_OFF_Z = 0.0f;

// ─────────────────────────────────────────────────────────────
//  Persistent storage (NVS flash via Preferences)
// ─────────────────────────────────────────────────────────────
// Calibration results and tuning values are saved to flash so they
// survive a power cycle — calibrate/tune once, it sticks. Auto-saved
// whenever a value changes over BLE (all low-frequency, so no flash
// wear concern). A BLE "reset" on the map-clear characteristic restores
// compiled defaults. Safety toggles are deliberately NOT persisted —
// avoidance always boots ON regardless of last session (see setup()).
static Preferences g_prefs;
#define PREFS_NAMESPACE "sensory"

static void prefs_save() {
  // Single transaction — open, write all, close.
  if (!g_prefs.begin(PREFS_NAMESPACE, false)) return;  // false = read/write
  g_prefs.putFloat("hmcX",  HMC_OFF_X);
  g_prefs.putFloat("hmcY",  HMC_OFF_Y);
  g_prefs.putFloat("hmcZ",  HMC_OFF_Z);
  g_prefs.putFloat("ofScl", g_ofScale);
  g_prefs.putFloat("ofWt",  g_ofWeight);
  g_prefs.putFloat("cvAl", g_cvAlpha);
  g_prefs.putFloat("cvGt", g_cvOutlierCms);
  g_prefs.putFloat("cvIn", g_cvInertialW);
  g_prefs.putFloat("baroWt",g_baroWeight);
  g_prefs.putFloat("altOff", g_altOffsetM);   // [v2.0.3 S-002]
  g_prefs.putULong("ttl",   g_pointTTLms);
  g_prefs.end();
}

static void prefs_load() {
  // Load saved values, falling back to the current (compiled-default)
  // value of each global if the key is absent (first boot after flash).
  if (!g_prefs.begin(PREFS_NAMESPACE, true)) return;  // true = read-only
  HMC_OFF_X    = g_prefs.getFloat("hmcX",  HMC_OFF_X);
  HMC_OFF_Y    = g_prefs.getFloat("hmcY",  HMC_OFF_Y);
  HMC_OFF_Z    = g_prefs.getFloat("hmcZ",  HMC_OFF_Z);
  g_ofScale    = g_prefs.getFloat("ofScl", g_ofScale);
  g_ofWeight   = g_prefs.getFloat("ofWt",  g_ofWeight);
  g_cvAlpha    = g_prefs.getFloat("cvAl", g_cvAlpha);
  g_cvOutlierCms = g_prefs.getFloat("cvGt", g_cvOutlierCms);
  g_cvInertialW  = g_prefs.getFloat("cvIn", g_cvInertialW);
  g_baroWeight = g_prefs.getFloat("baroWt",g_baroWeight);
  g_altOffsetM = g_prefs.getFloat("altOff", g_altOffsetM);   // [v2.0.3 S-002]
  g_pointTTLms = g_prefs.getULong("ttl",   g_pointTTLms);
  g_prefs.end();
}

static void prefs_reset() {
  // Wipe saved values; globals keep their current (compiled-default) values
  // until next boot, where prefs_load() will find nothing and use defaults.
  if (!g_prefs.begin(PREFS_NAMESPACE, false)) return;
  g_prefs.clear();
  g_prefs.end();
}

// ─────────────────────────────────────────────────────────────
//  Structs
// ─────────────────────────────────────────────────────────────

struct Vec3 { float x, y, z; };

// Cached rotation matrix built from quaternion once per loop.
// R * v_body = v_world  (NED frame)
struct RotMat {
  float r00,r01,r02;
  float r10,r11,r12;
  float r20,r21,r22;
};

// Map point: world-fixed NED obstacle coordinate + age timestamp
struct __attribute__((packed)) MapPoint {
  float    north;     // metres
  float    east;
  float    down;
  uint32_t stampMs;   // millis() when recorded — used for expiry
};
static_assert(sizeof(MapPoint) == 16, "MapPoint size mismatch");

// Safety packet — sent every loop (~180 Hz)
// *** Must match the SafetyPacket layout in comm_cortex; guarded by SAFETY_PKT_VERSION.
// *** Bump SAFETY_PKT_VERSION in BOTH firmwares on any layout change. ***
struct __attribute__((packed)) SafetyPacket {
  uint8_t  start;          // 0xAA
  uint8_t  version;        // SAFETY_PKT_VERSION — reject on mismatch (v2.0.0)
  uint8_t  frontBlocked;   // 1 = inside threshold
  uint8_t  leftBlocked;
  uint8_t  rightBlocked;
  uint8_t  backBlocked;
  uint8_t  faults;         // bit0=front bit1=left bit2=right bit3=AHRS
                           // bit4=altFault bit5=ofFault bit6=back bit7=baro
  uint8_t  tteFront;       // time-to-impact (tenths sec, 255=clear)
  uint8_t  tteLeft;
  uint8_t  tteRight;
  uint8_t  tteBack;
  int16_t  velFront;       // cm/s negative=closing
  int16_t  velLeft;
  int16_t  velRight;
  int16_t  velBack;
  uint16_t altCm;
  int16_t  rollDeg;        // roll  ×10 (deg). Valid unless faults bit3 (AHRS) set
  int16_t  pitchDeg;       // pitch ×10 (deg). Valid unless faults bit3 (AHRS) set
  int16_t  vertVelCmS;     // vertical velocity cm/s, NEGATIVE = descending
  int16_t  mapForceN;      // map repulsion FORWARD ×1000 (body frame, nudge)
  int16_t  mapForceE;      // map repulsion RIGHT   ×1000 (body frame)
  uint8_t  mapForceValid;  // 1 = dead-reckoning trusted, force usable
  uint8_t  checksum;       // XOR of all bytes from version..mapForceValid (v2.0.0)
  uint8_t  end;            // 0x55
};
static_assert(sizeof(SafetyPacket) == 34, "SafetyPacket size mismatch");

// Map header — sent every MAP_TX_EVERY loops
struct __attribute__((packed)) MapHeader {
  uint8_t  start;     // 0xBB
  uint16_t count;     // number of MapPoints that follow
  uint16_t checksum;  // Fletcher-16 over all MapPoint bytes
  uint8_t  end;       // 0x55
};
static_assert(sizeof(MapHeader) == 6, "MapHeader size mismatch");

// ─────────────────────────────────────────────────────────────
//  Global state
// ─────────────────────────────────────────────────────────────

ICM42688 IMU(Wire, ICM_ADDR);

// Mahony quaternion (w,x,y,z)
static float q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f;
static float ibx = 0.0f, iby = 0.0f, ibz = 0.0f;

// Cached rotation matrix — rebuilt once per loop after AHRS update
static RotMat R;

// World-frame position and velocity (NED, metres)
static Vec3 pos     = {0,0,0};
static Vec3 vel     = {0,0,0};
static Vec3 aBias   = {0,0,0};  // accel bias (world frame)

// Spatial map — in PSRAM when USE_PSRAM=1, static SRAM otherwise.
// mapBuf is a pointer; initialised in setup() via ps_malloc() or new[].
// Access pattern: write ~5x/loop (map_push), full scan at 0.5Hz (expire).
// PSRAM latency (~160ns/access) is acceptable for this access pattern.
static MapPoint *mapBuf   = nullptr;   // allocated in setup()
static uint16_t  mapCount = 0;         // live point count — in SRAM

// Hash grid — DRAM_ATTR forces into internal SRAM regardless of compiler.
// This is the hot-path index: called on every map_push() O(1) lookup.
// At 5 sensors × 180Hz = 900 lookups/second — must be fast.
// Size: HASH_SIZE × 2 bytes = 8192×2 = 16KB (PSRAM variant)
//                            or 1024×2 = 2KB  (no-PSRAM variant)
// Both fit comfortably in internal SRAM alongside other state.
static DRAM_ATTR uint16_t hashGrid[HASH_SIZE];

// Sensor state — indices: 0=Front 1=Left 2=Right 3=Back 4=Down
// All in internal SRAM — read every loop, must be fast
static float    lastDist[5]   = {0,0,0,0,0};  // previous valid distance (cm)
static float    sensorVel[5]  = {0,0,0,0,0};  // approach velocity (cm/s)
static uint8_t  failCount[5]  = {0,0,0,0,0};  // consecutive read failures
// [v2.0.3 S-024] Fault hysteresis: a fault SETS after FAULT_SET_N consecutive
// bad reads, but only CLEARS after FAULT_CLEAR_N consecutive good reads. This
// stops a flapping sensor from toggling avoidance authority on/off. Previously
// a single good read cleared the fault instantly.
static uint8_t  goodStreak[5]    = {0,0,0,0,0};  // consecutive good reads
static bool     sensorFaulted[5] = {false,false,false,false,false};
#define FAULT_SET_N    10   // consecutive bad reads to raise a sensor fault
#define FAULT_CLEAR_N  15   // consecutive good reads to clear it (hysteresis)
static uint32_t magFailMs     = 0;          // how long mag has been absent (ms)

// Loop timing
static unsigned long lastLoopUs = 0;
static uint8_t  loopN        = 0;   // map TX counter
static uint8_t  expireN      = 0;   // expire counter
static uint8_t  hmcPhase        = 0;        // HMC poll phase (0..2)

// Fletcher-16 streaming state
static uint8_t  fcs1 = 0, fcs2 = 0;        // running checksum accumulators

// ─────────────────────────────────────────────────────────────
//  Rotation matrix helper
// ─────────────────────────────────────────────────────────────
// Call once after Mahony update. All subsequent rotations in this
// loop use R directly — no quaternion products at use-site.

static inline void build_rot_mat() {
  float q0q0=q0*q0, q1q1=q1*q1, q2q2=q2*q2, q3q3=q3*q3;
  float q0q1=q0*q1, q0q2=q0*q2, q0q3=q0*q3;
  float q1q2=q1*q2, q1q3=q1*q3, q2q3=q2*q3;
  R.r00 = q0q0+q1q1-q2q2-q3q3;  R.r01 = 2*(q1q2-q0q3);        R.r02 = 2*(q1q3+q0q2);
  R.r10 = 2*(q1q2+q0q3);        R.r11 = q0q0-q1q1+q2q2-q3q3;  R.r12 = 2*(q2q3-q0q1);
  R.r20 = 2*(q1q3-q0q2);        R.r21 = 2*(q2q3+q0q1);        R.r22 = q0q0-q1q1-q2q2+q3q3;
}

// Rotate body-frame vector → world-frame, cm → m, add drone position.
static inline MapPoint body_to_world(float bx, float by, float bz, uint32_t now) {
  float wx = R.r00*bx + R.r01*by + R.r02*bz;
  float wy = R.r10*bx + R.r11*by + R.r12*bz;
  float wz = R.r20*bx + R.r21*by + R.r22*bz;
  // Apply precomputed declination rotation to horizontal plane
  float north = wx*COS_DECL - wy*SIN_DECL;
  float east  = wx*SIN_DECL + wy*COS_DECL;
  MapPoint p;
  p.north   = north*0.01f + pos.x;
  p.east    = east *0.01f + pos.y;
  p.down    = wz   *0.01f + pos.z;
  p.stampMs = now;
  return p;
}

// ─────────────────────────────────────────────────────────────
//  3D hash grid  (O(1) voxel dedup)
// ─────────────────────────────────────────────────────────────
// Key: quantised (north, east, down) voxel coordinates as int16.
// Hash: FNV-1a mix of the three 16-bit values.

static inline void hash_init() {
  for (uint16_t i = 0; i < HASH_SIZE; i++) hashGrid[i] = HASH_EMPTY;
}

static inline uint16_t voxel_hash(int16_t vn, int16_t ve, int16_t vd) {
  uint32_t h = 2166136261u;
  h ^= (uint8_t)(vn);       h *= 16777619u;
  h ^= (uint8_t)(vn >> 8);  h *= 16777619u;
  h ^= (uint8_t)(ve);       h *= 16777619u;
  h ^= (uint8_t)(ve >> 8);  h *= 16777619u;
  h ^= (uint8_t)(vd);       h *= 16777619u;
  h ^= (uint8_t)(vd >> 8);  h *= 16777619u;
  return (uint16_t)(h & (HASH_SIZE - 1));
}

// Returns true if voxel already in hash. If not, inserts mapIdx.
// Returns -1 if voxel is new (and inserts mapIdx), or existing mapBuf index if already present
static int16_t hash_check_insert(int16_t vn, int16_t ve, int16_t vd, uint16_t mapIdx) {
  uint16_t slot = voxel_hash(vn, ve, vd);
  for (uint16_t probe = 0; probe < HASH_SIZE; probe++) {
    uint16_t s = (slot + probe) & (HASH_SIZE - 1);
    if (hashGrid[s] == HASH_EMPTY) {
      hashGrid[s] = mapIdx;
      return -1;  // not present — inserted
    }
    uint16_t idx = hashGrid[s];
    if (idx >= mapCount) {
      // Stale slot (point was compacted away) — treat as empty, reuse slot
      hashGrid[s] = mapIdx;
      return -1;  // reused stale slot
    }
    // Check if existing live slot matches this voxel
    int16_t en = (int16_t)roundf(mapBuf[idx].north / VOXEL_M);
    int16_t ee = (int16_t)roundf(mapBuf[idx].east  / VOXEL_M);
    int16_t ed = (int16_t)roundf(mapBuf[idx].down  / VOXEL_M);
    if (en == vn && ee == ve && ed == vd) return (int16_t)idx;  // already exists
  }
  return -1;  // table full (shouldn't happen at load factor <0.5)
}

// [v2.0.3 S-006] Pure lookup — never mutates the hash. Needed so map_push can
// dedup BEFORE resolving capacity, then insert the new point at its FINAL index
// once expiry/cluster/rebuild has settled. Skips stale slots (idx>=mapCount)
// and only terminates on a genuinely EMPTY slot, preserving probe-chain
// integrity (slots only go occupied→stale between rebuilds, never →EMPTY).
static int16_t hash_lookup(int16_t vn, int16_t ve, int16_t vd) {
  uint16_t slot = voxel_hash(vn, ve, vd);
  for (uint16_t probe = 0; probe < HASH_SIZE; probe++) {
    uint16_t s = (slot + probe) & (HASH_SIZE - 1);
    uint16_t idx = hashGrid[s];
    if (idx == HASH_EMPTY) return -1;   // end of chain — not present
    if (idx >= mapCount)   continue;    // stale slot — skip, keep probing
    int16_t en = (int16_t)roundf(mapBuf[idx].north / VOXEL_M);
    int16_t ee = (int16_t)roundf(mapBuf[idx].east  / VOXEL_M);
    int16_t ed = (int16_t)roundf(mapBuf[idx].down  / VOXEL_M);
    if (en == vn && ee == ve && ed == vd) return (int16_t)idx;
  }
  return -1;
}

// Full hash rebuild (called after expiry sweep or cluster pass)
static void hash_rebuild() {
  hash_init();
  for (uint16_t i = 0; i < mapCount; i++) {
    int16_t vn = (int16_t)roundf(mapBuf[i].north / VOXEL_M);
    int16_t ve = (int16_t)roundf(mapBuf[i].east  / VOXEL_M);
    int16_t vd = (int16_t)roundf(mapBuf[i].down  / VOXEL_M);
    hash_check_insert(vn, ve, vd, i);  // rebuild — always new slots
  }
}

// ─────────────────────────────────────────────────────────────
//  Map management
// ─────────────────────────────────────────────────────────────

// Expire points older than POINT_TTL_MS.
// Compact array in-place; rebuild hash.
static void map_expire(uint32_t nowMs) {
  uint16_t dst = 0;
  for (uint16_t i = 0; i < mapCount; i++) {
    if (nowMs - mapBuf[i].stampMs < g_pointTTLms) {
      if (dst != i) mapBuf[dst] = mapBuf[i];
      dst++;
    }
  }
  if (dst != mapCount) {
    mapCount = dst;
    hash_rebuild();
  }
}

// Proximity cluster: merge any two points within VOXEL_M*1.5 of each other.
// O(n²) but called only when buffer is full.
static void map_cluster() {
  static const float R2 = (VOXEL_M * 1.5f) * (VOXEL_M * 1.5f);
  // EXT_RAM_ATTR: place in PSRAM when available (5000 bytes at MAP_SIZE=5000)
  // cluster() runs rarely — PSRAM latency irrelevant here
  static EXT_RAM_ATTR bool merged[MAP_SIZE];  memset(merged, 0, MAP_SIZE);
  uint16_t dst = 0;
  for (uint16_t i = 0; i < mapCount; i++) {
    if (merged[i]) continue;
    for (uint16_t j = i + 1; j < mapCount; j++) {
      if (merged[j]) continue;
      float dn = mapBuf[i].north - mapBuf[j].north;
      float de = mapBuf[i].east  - mapBuf[j].east;
      float dd = mapBuf[i].down  - mapBuf[j].down;
      if (dn*dn + de*de + dd*dd <= R2) {
        mapBuf[i].north = (mapBuf[i].north + mapBuf[j].north) * 0.5f;
        mapBuf[i].east  = (mapBuf[i].east  + mapBuf[j].east)  * 0.5f;
        mapBuf[i].down  = (mapBuf[i].down  + mapBuf[j].down)  * 0.5f;
        // Keep newer timestamp
        if (mapBuf[j].stampMs > mapBuf[i].stampMs)
          mapBuf[i].stampMs = mapBuf[j].stampMs;
        merged[j] = true;
      }
    }
    mapBuf[dst++] = mapBuf[i];
  }
  mapCount = dst;
  hash_rebuild();
}

// Update existing point's timestamp if it's re-observed, or insert new point.
static void map_push(MapPoint p, uint32_t nowMs) {
  int16_t vn = (int16_t)roundf(p.north / VOXEL_M);
  int16_t ve = (int16_t)roundf(p.east  / VOXEL_M);
  int16_t vd = (int16_t)roundf(p.down  / VOXEL_M);

  // Snap to voxel grid
  p.north = vn * VOXEL_M;
  p.east  = ve * VOXEL_M;
  p.down  = vd * VOXEL_M;

  // [v2.0.3 S-006] O(1) dedup — LOOKUP ONLY (does not touch the hash).
  int16_t existIdx = hash_lookup(vn, ve, vd);
  if (existIdx >= 0) {
    // Point already in map — refresh its timestamp in O(1)
    mapBuf[existIdx].stampMs = nowMs;
    return;
  }

  if (mapCount >= MAP_SIZE) {
    // Buffer full — try expiry first, then cluster
    map_expire(nowMs);
    if (mapCount >= MAP_SIZE) {
      map_cluster();
      if (mapCount >= MAP_SIZE) {
        // All points are recent and spread out — drop oldest
        // Shift array left by 1 (oldest is index 0 after expiry)
        memmove(&mapBuf[0], &mapBuf[1], (MAP_SIZE-1) * sizeof(MapPoint));
        mapCount = MAP_SIZE - 1;
        hash_rebuild();
      }
    }
  }

  // [v2.0.3 S-006] Insert AFTER capacity resolution so the point is hashed at
  // its FINAL index. Previously the hash insert ran before expiry/cluster/
  // rebuild, so a point added on the map-full path was stored in mapBuf but
  // orphaned from the hash → transient duplicate voxels until the next rebuild.
  mapBuf[mapCount] = p;
  hash_check_insert(vn, ve, vd, mapCount);
  mapCount++;
}

// ─────────────────────────────────────────────────────────────
//  Map-based potential field (secondary avoidance)
// ─────────────────────────────────────────────────────────────
// Computes a horizontal repulsion vector from mapped obstacles within a
// VRB-scaled bubble around the drone's dead-reckoned position. Obstacles
// push the drone away, weighted by 1/d² so near points dominate. The
// vertical (down) axis is deliberately excluded — altitude is handled by
// the dedicated floor + live Down sensor, not by map repulsion.
//
// Result is written to g_mapForceN / g_mapForceE (normalised, capped) and
// g_mapForceValid. The caller gates validity on dead-reckoning confidence.
//
// Cost: O(mapCount) scan. Called at a reduced rate (not every loop) since
// the map and pose change slowly relative to the sensor loop.
static void compute_map_force(uint32_t nowMs) {
  // Gate on dead-reckoning confidence. The map is only meaningful if the
  // pose feeding it is trustworthy. Optical flow bounds horizontal drift;
  // a valid altitude bounds vertical. If either is bad, do not use the map.
  bool poseTrusted = of_ok && g_altOk;
  if (!g_mapAvoidEnabled || !poseTrusted || mapCount == 0) {
    g_mapForceN = 0.0f;
    g_mapForceE = 0.0f;
    g_mapForceValid = false;
    return;
  }

  // Bubble radius scales with the proximity VRB, same dial as live zones.
  float bubble   = CFG_MAP_BUBBLE_BASE_M * g_proxScale;
  float bubble2  = bubble * bubble;
  float soft     = CFG_MAP_FORCE_FALLOFF_M;
  float soft2    = soft * soft;

  float fN = 0.0f, fE = 0.0f;
  uint16_t contributing = 0;

  for (uint16_t i = 0; i < mapCount; i++) {
    // Vector FROM obstacle TO drone (the push direction), horizontal only.
    float dN = pos.x - mapBuf[i].north;
    float dE = pos.y - mapBuf[i].east;
    float d2 = dN*dN + dE*dE;

    if (d2 > bubble2 || d2 < 1e-6f) continue;  // outside bubble / coincident

    // 1/d² weighting with softening to avoid singularity at d→0.
    // magnitude = 1 / (d² + soft²); direction = unit(drone − obstacle)
    float d = sqrtf(d2);
    float w = 1.0f / (d2 + soft2);
    fN += (dN / d) * w;
    fE += (dE / d) * w;
    contributing++;
  }

  if (contributing == 0) {
    g_mapForceN = 0.0f;
    g_mapForceE = 0.0f;
    g_mapForceValid = false;
    return;
  }

  // Normalise to the strongest single-obstacle contribution so the cap is
  // meaningful regardless of how many points are in the bubble, then clamp.
  float mag = sqrtf(fN*fN + fE*fE);
  if (mag > 1e-6f) {
    float scale = CFG_MAP_FORCE_CAP / mag;
    if (scale < 1.0f) { fN *= scale; fE *= scale; }  // clamp, never amplify
  }

  g_mapForceN = fN;
  g_mapForceE = fE;
  g_mapForceValid = true;
}

// ─────────────────────────────────────────────────────────────
//  Raycasting map cleanup (drift-ghost removal)
// ─────────────────────────────────────────────────────────────
// When a sensor reports a clear path (or an obstacle at distance d), every
// mapped point lying ALONG that ray and NEARER than d must be empty — the
// sensor would have hit it otherwise. Such points are stale: either drifted
// "ghosts" from accumulated dead-reckoning error, or obstacles that have
// genuinely gone. Removing them keeps the map honest and counteracts drift.
//
// For each of the 4 horizontal sensors we have a world-space ray from the
// drone position along the sensor's bearing. A mapped point is "on the ray
// and in front of the hit" if:
//   - its projection onto the ray direction is in (CLEAR_MIN, hitDist), and
//   - its perpendicular distance from the ray is within RAYCAST_CORRIDOR.
//
// Points beyond the hit, behind the drone, or off to the side are untouched.
// Runs at reduced rate; O(mapCount) per call.
#define RAYCAST_CORRIDOR_M  0.25f   // half-width of the "cleared" tube
#define RAYCAST_MIN_M       0.30f   // don't clear right at the drone
#define LUNA_MAX_CM         800.0f  // TF-Luna max range

static void raycast_clear(const int *distCm, uint32_t nowMs) {
  // Only trust raycasting when the pose is good — same gate as the map force.
  if (!(of_ok && g_altOk) || mapCount == 0) return;

  // Build the 4 sensor ray directions in BODY frame, then rotate to world.
  // Body: front +X, back −X, left +Y, right −Y (matches map insertion).
  const float bodyDir[4][2] = {
    { 1.0f,  0.0f},  // front
    { 0.0f,  1.0f},  // left
    { 0.0f, -1.0f},  // right
    {-1.0f,  0.0f},  // back
  };

  float corridor2 = RAYCAST_CORRIDOR_M * RAYCAST_CORRIDOR_M;
  bool removedAny = false;

  for (int s = 0; s < 4; s++) {
    // hit distance in metres: a real reading clears up to it; a clear
    // reading (dist<=0) clears all the way to max range.
    float hitM = (distCm[s] > 0) ? (distCm[s] * 0.01f) : (LUNA_MAX_CM * 0.01f);
    if (hitM <= RAYCAST_MIN_M) continue;  // too close to clear anything

    // Ray direction in world frame (horizontal only).
    float dx = R.r00*bodyDir[s][0] + R.r01*bodyDir[s][1];
    float dy = R.r10*bodyDir[s][0] + R.r11*bodyDir[s][1];
    // Apply the same declination rotation used for map insertion.
    float rN = dx*COS_DECL - dy*SIN_DECL;
    float rE = dx*SIN_DECL + dy*COS_DECL;
    float rlen = sqrtf(rN*rN + rE*rE);
    if (rlen < 1e-6f) continue;
    rN /= rlen; rE /= rlen;   // unit ray direction

    // Scan the map; compact out points that fall inside this ray's corridor
    // and nearer than the hit (minus one corridor width, so we never delete
    // the obstacle we just detected).
    float clearTo = hitM - RAYCAST_CORRIDOR_M;
    if (clearTo <= RAYCAST_MIN_M) continue;

    uint16_t dst = 0;
    for (uint16_t i = 0; i < mapCount; i++) {
      float vN = mapBuf[i].north - pos.x;   // point relative to drone
      float vE = mapBuf[i].east  - pos.y;
      float along = vN*rN + vE*rE;          // projection onto ray
      bool remove = false;
      if (along > RAYCAST_MIN_M && along < clearTo) {
        // perpendicular distance² from the ray
        float perpN = vN - along*rN;
        float perpE = vE - along*rE;
        if (perpN*perpN + perpE*perpE <= corridor2) remove = true;
      }
      if (!remove) {
        if (dst != i) mapBuf[dst] = mapBuf[i];
        dst++;
      }
    }
    if (dst != mapCount) { mapCount = dst; removedAny = true; }
  }

  if (removedAny) hash_rebuild();
}

// (Forward/Right) so the Comm Cortex — which has no attitude data — can map
// it straight onto pitch/roll. Uses the cached rotation matrix's top-left
// 2×2 (yaw component dominates for a near-level drone). Returns via refs.
// Body forward = +X, body right = −Y (NED: +Y is East/right-ish, sign care).
static void map_force_to_body(float &fwd, float &right) {
  // World→body is the transpose of body→world (R is orthonormal).
  // bodyX (forward) = R.r00*N + R.r10*E ; bodyY (left+) = R.r01*N + R.r11*E
  float bx = R.r00*g_mapForceN + R.r10*g_mapForceE;   // forward (+)
  float by = R.r01*g_mapForceN + R.r11*g_mapForceE;   // left (+)
  fwd   = bx;
  right = -by;   // convert left+ to right+ for the Comm Cortex convention
}


// ─────────────────────────────────────────────────────────────
//  Fletcher-16 (streaming, no repacking)
// ─────────────────────────────────────────────────────────────

static void fcs_reset() { fcs1 = 0; fcs2 = 0; }

static void fcs_feed(const uint8_t *data, uint16_t len) {
  for (uint16_t i = 0; i < len; i++) {
    fcs1 += data[i];
    fcs2 += fcs1;
    // Reduce periodically to avoid overflow (Modulo deferred)
    // fcs1/fcs2 are uint8_t so they auto-wrap at 256 — but true
    // Fletcher-16 uses mod 255.  Apply mod lazily every 255 bytes.
    if ((i & 0xFF) == 0xFE) { fcs1 %= 255; fcs2 %= 255; }
  }
}

static uint16_t fcs_finish() {
  return ((uint16_t)(fcs2 % 255) << 8) | (fcs1 % 255);
}

// ─────────────────────────────────────────────────────────────
//  Mahony AHRS
// ─────────────────────────────────────────────────────────────

static void mahony_update(float gx, float gy, float gz,
                          float ax, float ay, float az,
                          float mx, float my, float mz,
                          float dt) {
  bool useMag = (mx!=0||my!=0||mz!=0);

  // Normalise accel
  float an = sqrtf(ax*ax+ay*ay+az*az);
  if (an < 1e-6f) return;
  ax/=an; ay/=an; az/=an;

  // Gravity estimate from quaternion
  float hvx =  q1*q3 - q0*q2;
  float hvy =  q0*q1 + q2*q3;
  float hvz =  q0*q0 - 0.5f + q3*q3;

  float hex, hey, hez;

  if (useMag) {
    float mn = sqrtf(mx*mx+my*my+mz*mz);
    if (mn > 1e-6f) {
      mx/=mn; my/=mn; mz/=mn;
      // Earth magnetic field reference in world frame
      float hx = 2*(mx*(0.5f-q2*q2-q3*q3)+my*(q1*q2-q0*q3)+mz*(q1*q3+q0*q2));
      float hy = 2*(mx*(q1*q2+q0*q3)+my*(0.5f-q1*q1-q3*q3)+mz*(q2*q3-q0*q1));
      float bx = sqrtf(hx*hx+hy*hy);
      float bz = 2*(mx*(q1*q3-q0*q2)+my*(q2*q3+q0*q1)+mz*(0.5f-q1*q1-q2*q2));
      // Estimated field direction
      float hwx = bx*(0.5f-q2*q2-q3*q3)+bz*(q1*q3-q0*q2);
      float hwy = bx*(q1*q2-q0*q3)      +bz*(q0*q1+q2*q3);
      float hwz = bx*(q1*q3+q0*q2)      +bz*(q0*q0-0.5f+q2*q2);
      // Cross-product errors
      hex = (ay*hvz-az*hvy)+(my*hwz-mz*hwy);
      hey = (az*hvx-ax*hvz)+(mz*hwx-mx*hwz);
      hez = (ax*hvy-ay*hvx)+(mx*hwy-my*hwx);
    } else { useMag=false; }
  }
  if (!useMag) {
    hex = ay*hvz - az*hvy;
    hey = az*hvx - ax*hvz;
    hez = ax*hvy - ay*hvx;
  }

  // Integral feedback
  ibx += MAHONY_KI*hex*dt;
  iby += MAHONY_KI*hey*dt;
  ibz += MAHONY_KI*hez*dt;
  gx  += MAHONY_KP*hex + ibx;
  gy  += MAHONY_KP*hey + iby;
  gz  += MAHONY_KP*hez + ibz;

  // Quaternion rate integration
  gx *= 0.5f*dt; gy *= 0.5f*dt; gz *= 0.5f*dt;
  float qa=q0, qb=q1, qc=q2;
  q0 += -qb*gx - qc*gy - q3*gz;
  q1 +=  qa*gx + qc*gz - q3*gy;
  q2 +=  qa*gy - qb*gz + q3*gx;
  q3 +=  qa*gz + qb*gy - qc*gx;  // FIXED: was qb*gx (wrong), now qb*gy

  // Renormalise
  float rn = 1.0f/sqrtf(q0*q0+q1*q1+q2*q2+q3*q3);
  q0*=rn; q1*=rn; q2*=rn; q3*=rn;
}

// ─────────────────────────────────────────────────────────────
//  Position integration
// ─────────────────────────────────────────────────────────────

static void integrate_position(float ax, float ay, float az, float dt) {
  // accX/Y/Z() return g — convert to m/s² for integration.
  const float G = 9.80665f;
  float axm = ax * G, aym = ay * G, azm = az * G;

  // World-frame acceleration via cached rotation matrix
  float awx = R.r00*axm + R.r01*aym + R.r02*azm;
  float awy = R.r10*axm + R.r11*aym + R.r12*azm;
  float awz = R.r20*axm + R.r21*aym + R.r22*azm;

  // Remove gravity. In world NED, gravity is +G on the Down axis.
  // The accelerometer measures specific force (reaction), so a level
  // hover reads +1g on body Z → +G on world Down. Subtract it directly.
  awz -= G;

  // Residual bias high-pass filter (removes slow sensor drift only —
  // gravity is already gone, so this no longer has to absorb 1g).
  float alpha = dt / (HP_TAU_S + dt);
  aBias.x += alpha*(awx - aBias.x);
  aBias.y += alpha*(awy - aBias.y);
  aBias.z += alpha*(awz - aBias.z);

  float ax2 = awx - aBias.x;
  float ay2 = awy - aBias.y;
  float az2 = awz - aBias.z;

  // Dead-band
  if (fabsf(ax2) < ACCEL_DEAD) ax2 = 0;
  if (fabsf(ay2) < ACCEL_DEAD) ay2 = 0;
  if (fabsf(az2) < ACCEL_DEAD) az2 = 0;

  vel.x += ax2*dt; vel.y += ay2*dt; vel.z += az2*dt;
  pos.x += vel.x*dt; pos.y += vel.y*dt; pos.z += vel.z*dt;
}

// ─────────────────────────────────────────────────────────────
//  HMC5883L driver
// ─────────────────────────────────────────────────────────────

static bool hmc_init() {
  Wire.beginTransmission(HMC_ADDR);
  Wire.write(0x00); Wire.write(0x78);  // 8-avg, 75 Hz, normal
  if (Wire.endTransmission()!=0) return false;
  Wire.beginTransmission(HMC_ADDR);
  Wire.write(0x01); Wire.write(0x20);  // gain ±1.3 Ga
  if (Wire.endTransmission()!=0) return false;
  Wire.beginTransmission(HMC_ADDR);
  Wire.write(0x02); Wire.write(0x00);  // continuous mode
  return Wire.endTransmission()==0;
}

// Forward declaration — defined with the BLE calibration state below.
static void hmc_cal_update(float rawX, float rawY, float rawZ);

static bool hmc_read(float &mx, float &my, float &mz) {
  // Check DRDY
  Wire.beginTransmission(HMC_ADDR);
  Wire.write(0x09);
  if (Wire.endTransmission(false)!=0) return false;
  Wire.requestFrom((uint8_t)HMC_ADDR,(uint8_t)1);
  if (!Wire.available()||(Wire.read()&0x01)==0) return false;
  // Burst read X,Z,Y (HMC register order)
  Wire.beginTransmission(HMC_ADDR);
  Wire.write(0x03);
  if (Wire.endTransmission(false)!=0) return false;
  Wire.requestFrom((uint8_t)HMC_ADDR,(uint8_t)6);
  if (Wire.available()<6) return false;
  int16_t rx=(int16_t)((Wire.read()<<8)|Wire.read());
  int16_t rz=(int16_t)((Wire.read()<<8)|Wire.read());
  int16_t ry=(int16_t)((Wire.read()<<8)|Wire.read());
  if (rx==-4096||ry==-4096||rz==-4096) return false;
  mx=(float)rx-HMC_OFF_X;
  my=(float)ry-HMC_OFF_Y;
  mz=(float)rz-HMC_OFF_Z;
  // Feed raw values to HMC calibration state machine if active
  hmc_cal_update((float)rx, (float)ry, (float)rz);
  return true;
}

// ─────────────────────────────────────────────────────────────
//  TF-Luna driver  (1 MHz I2C)
// ─────────────────────────────────────────────────────────────

static int luna_read(uint8_t addr) {
  Wire.beginTransmission(addr);
  Wire.write(0x00);
  if (Wire.endTransmission(false)!=0) return -1;
  Wire.requestFrom(addr,(uint8_t)2);
  if (Wire.available()<2) return -1;
  uint8_t lo=Wire.read(), hi=Wire.read();
  int d=(int)lo|((int)hi<<8);
  return (d>0&&d<=800)?d:-1;
}

// ─────────────────────────────────────────────────────────────
//  TTE helper — time-to-impact in tenths of seconds (0-254)
//  Returns 255 if no threat.
// ─────────────────────────────────────────────────────────────

static uint8_t compute_tte(float distCm, float velCms) {
  if (velCms >= 0.0f || distCm <= 0.0f) return 255;  // moving away or invalid
  float tte_s = distCm / (-velCms);  // velCms is negative when closing
  if (tte_s > 25.4f) return 255;
  uint8_t tenths = (uint8_t)(tte_s * 10.0f);
  return tenths;
}


// ── BMP280 barometric altimeter ──────────────────────────────
#if BMP_ENABLED
#include <Adafruit_BMP280.h>
static Adafruit_BMP280 bmp(&Wire);
static bool bmp_init() {
  if (!bmp.begin(BMP_ADDR)) return false;
  bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,
                  Adafruit_BMP280::SAMPLING_X2,
                  Adafruit_BMP280::SAMPLING_X16,
                  Adafruit_BMP280::FILTER_X16,
                  Adafruit_BMP280::STANDBY_MS_1);
  return true;
}
static void bmp_calibrate_ground() {
  float s=0; for(int i=0;i<20;i++){s+=bmp.readPressure();delay(10);}
  g_baroRefPa=s/20.0f;
  Serial.printf("[OK] BMP280 ref: %.2f Pa\n",g_baroRefPa);
}
static void bmp_update() {
  if(g_baroRefPa<1.0f) return;
  float p=bmp.readPressure();
  if(p<50000.0f||p>115000.0f){g_baroOk=false;return;}
  float raw=44330.0f*(1.0f-powf(p/g_baroRefPa,1.0f/5.255f))+CFG_BARO_OFFSET_M;
  g_baroRawM=raw;
  g_baroAltM=g_baroAltM*(1.0f-g_baroAlpha)+raw*g_baroAlpha;
  g_baroOk=true;
}
#else
static bool bmp_init()             { return false; }
static void bmp_calibrate_ground() {}
static void bmp_update()           {}
#endif

// ─────────────────────────────────────────────────────────────
//  Optical flow driver — PMW3901 / PAA5100 via SPI
//  Reads raw motion delta X/Y in pixel units.
//  Converts to world-frame velocity using g_ofScale and drone altitude.
//  Then blends with dead-reckoning velocity via g_ofWeight.
//
//  Hardware: PMW3901 (Bitcraze) or PAA5100 (Pimoroni).
//  Both use same SPI protocol. CPOL=0 CPHA=0, max 2 MHz.
//  Library: "Bitcraze PMW3901" by Bitcraze — install via Library Manager.
//  If library not installed, set OF_ENABLED 0 to compile without it.
// ─────────────────────────────────────────────────────────────

#if OF_ENABLED
#include <Bitcraze_PMW3901.h>
static Bitcraze_PMW3901 optFlow(OF_SPI_CS);

static bool of_init() {
  // PMW3901 uses Arduino SPI — must call SPI.begin() with custom pins
  SPI.begin(OF_SPI_SCK, OF_SPI_MISO, OF_SPI_MOSI, OF_SPI_CS);
  return optFlow.begin();
}

// Call once per loop (or every N loops — OF updates at ~100 Hz).
// Converts pixel delta to world-frame velocity and blends into vel.x/y.
static void of_update(float dt, float altM,
                       float gyro_x, float gyro_y) {
  // gyro_x = pitch rate rad/s (body X = forward axis)
  // gyro_y = roll rate  rad/s (body Y = right axis)
  // Passed directly from this loop iteration's IMU read — no extra latency.

  int16_t dx = 0, dy = 0;
  optFlow.readMotionCount(&dx, &dy);

  float effectiveAlt = (altM > 0.05f) ? altM : 0.3f;  // floor at 30cm

  // ── Rotational flow compensation ──────────────────────────────
  // Pitch or roll rotates the sensor over the ground, creating apparent
  // pixel movement that is NOT translational motion. Must be removed.
  //
  // rotational_pixels = angular_rate_rad_s × altitude_m / of_scale
  // (of_scale = metres per pixel per metre altitude — a lens constant)
  //
  // Guard against uninitialised g_ofScale:
  float rotCorrX = 0.0f;
  float rotCorrY = 0.0f;
  if (g_ofScale > 0.00001f) {
    rotCorrX = gyro_x * effectiveAlt / g_ofScale;
    rotCorrY = gyro_y * effectiveAlt / g_ofScale;
  }

  // Pure translational pixel deltas — rotational component removed
  float dx_comp = (float)dx - rotCorrX;
  float dy_comp = (float)dy - rotCorrY;

  // Body-frame velocity (m/s)
  float vx_of = dx_comp * effectiveAlt * g_ofScale / dt;
  float vy_of = dy_comp * effectiveAlt * g_ofScale / dt;

  // Expose body-frame components for the closing-velocity fusion (v1.9.2):
  // vx_of = forward (+ = moving toward FRONT sensor), vy_of = right
  // (+ = moving toward RIGHT sensor).
  of_body_vx = vx_of;
  of_body_vy = vy_of;

  // Rotate into world NED frame
  float wx = R.r00*vx_of + R.r01*vy_of;
  float wy = R.r10*vx_of + R.r11*vy_of;

  // Blend with dead-reckoning
  vel.x = vel.x*(1.0f - g_ofWeight) + wx*g_ofWeight;
  vel.y = vel.y*(1.0f - g_ofWeight) + wy*g_ofWeight;

  of_vx     = wx;
  of_vy     = wy;
  of_ok     = true;
  of_lastMs = millis();

  // Accumulate world-frame displacement for BLE flow calibration.
  // Feeds g_ofCalAccumX/Y — the variables the calibration handler reads
  // to compute distance flown vs the user-entered known distance.
  if (g_ofCalPending) {
    g_ofCalAccumX += wx * dt;
    g_ofCalAccumY += wy * dt;
  }
}
#else
static bool of_init() { return false; }
static void of_update(float dt, float altM, float gx, float gy) { (void)dt; (void)altM; (void)gx; (void)gy; }
#endif  // OF_ENABLED

// ─────────────────────────────────────────────────────────────
//  Downward TF-Luna altitude reader
//  Same luna_read() protocol, different I2C address (0x13).
//  Reads absolute altitude above ground in metres.
//  Corrects pos.z (NED down = positive) dead-reckoning.
// ─────────────────────────────────────────────────────────────

#if LUNA_DOWN_ENABLED
static void alt_update(float dt) {
  // Update the barometer first (no-op if BMP disabled or not initialised).
  bmp_update();

  int d = luna_read(LUNA_DOWN);  // returns cm
  if (d > 0 && d <= 800) {
    // TF-Luna valid (≤8m): use it ALONE as the altitude. It is the trusted
    // terrain-clearance ground-truth, and the Comm Cortex altitude floor
    // depends on it being uncontaminated near the deck. Baro (which drifts
    // with weather and prop wash) must not pull this around close to ground.
    g_altM  = (float)d * 0.01f + g_altOffsetM;
    pos.z   = -g_altM;
    g_altOk = true;
    altFailCt = 0;
    // Vertical velocity from CORRECTED altitude (not drifting vel.z).
    // +ve = climbing. Low-passed to suppress LiDAR quantisation jitter.
    if (g_prevAltM >= 0.0f && dt > 0.0001f) {
      float raw = (g_altM - g_prevAltM) / dt;        // m/s, +ve climbing
      const float a = 0.3f;                          // light low-pass
      g_vertVelMS = g_vertVelMS*(1.0f - a) + raw*a;
    }
    g_prevAltM = g_altM;
  } else {
    // TF-Luna out of range or failed. Fall back to baro alone if available
    // (lets the map/pose stay usable above 8m), otherwise declare alt failed.
    if (g_baroOk) {
      // Ease into the baro value rather than snapping — the TF-Luna and baro
      // can disagree by tens of cm at the hand-off (e.g. climbing past 8m),
      // and a hard jump would momentarily disturb pose/map. Once we've been
      // on baro for a few cycles this converges and tracks it directly.
      const float blend = 0.2f;
      g_altM  = g_altM*(1.0f - blend) + g_baroAltM*blend;
      pos.z   = -g_altM;
      g_altOk = true;
      altFailCt = 0;
      // Vertical velocity from blended altitude (baro-backed). Coarser than
      // LiDAR but keeps the estimate alive above 8m.
      if (g_prevAltM >= 0.0f && dt > 0.0001f) {
        float raw = (g_altM - g_prevAltM) / dt;
        const float a = 0.15f;                       // heavier LP — baro is noisy
        g_vertVelMS = g_vertVelMS*(1.0f - a) + raw*a;
      }
      g_prevAltM = g_altM;
    } else {
      if (altFailCt < 255) altFailCt++;
      if (altFailCt > 20) { g_altOk = false; g_prevAltM = -1.0f; g_vertVelMS = 0.0f; }
    }
  }
}
#else
static void alt_update(float dt) { (void)dt; }
#endif  // LUNA_DOWN_ENABLED


// BLE UUIDs
#define BLE_SERVICE_UUID       "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define BLE_CHAR_STATUS_UUID   "4fafc201-1fb5-459e-8fcc-c5c9c3319101"
#define BLE_CHAR_ALTITUDE_UUID "4fafc201-1fb5-459e-8fcc-c5c9c3319102"
#define BLE_CHAR_HEADING_UUID  "4fafc201-1fb5-459e-8fcc-c5c9c3319103"
#define BLE_CHAR_MAPINFO_UUID  "4fafc201-1fb5-459e-8fcc-c5c9c3319104"
#define BLE_CHAR_HMCCAL_UUID   "4fafc201-1fb5-459e-8fcc-c5c9c3319105"
#define BLE_CHAR_OFCAL_UUID    "4fafc201-1fb5-459e-8fcc-c5c9c3319106"
#define BLE_CHAR_CALSTAT_UUID  "4fafc201-1fb5-459e-8fcc-c5c9c3319107"
#define BLE_CHAR_TTL_UUID      "4fafc201-1fb5-459e-8fcc-c5c9c3319108"
#define BLE_CHAR_BAROWT_UUID   "4fafc201-1fb5-459e-8fcc-c5c9c3319109"
#define BLE_CHAR_OFWT_UUID     "4fafc201-1fb5-459e-8fcc-c5c9c331910A"
#define BLE_CHAR_MAPCLEAR_UUID "4fafc201-1fb5-459e-8fcc-c5c9c331910B"

// BLE characteristic pointers — declared at file scope
static BLECharacteristic *bleCharStatus   = nullptr;
static BLECharacteristic *bleCharAltitude = nullptr;
static BLECharacteristic *bleCharHeading  = nullptr;
static BLECharacteristic *bleCharMapInfo  = nullptr;
static BLECharacteristic *bleCharHmcCal   = nullptr;
static BLECharacteristic *bleCharOfCal    = nullptr;
static BLECharacteristic *bleCharTTL      = nullptr;
static BLECharacteristic *bleCharBaroWt   = nullptr;
static BLECharacteristic *bleCharOfWt     = nullptr;
static BLECharacteristic *bleCharMapClear = nullptr;
static BLECharacteristic *bleCharCalStatus= nullptr;

// ─────────────────────────────────────────────────────────────
//  BLE calibration state
// ─────────────────────────────────────────────────────────────
static volatile bool g_hmcCalActive = false;
static float g_hmcMinX=99999,g_hmcMaxX=-99999;
static float g_hmcMinY=99999,g_hmcMaxY=-99999;
static float g_hmcMinZ=99999,g_hmcMaxZ=-99999;
static uint32_t g_hmcCalStartMs = 0;
static constexpr uint32_t HMC_CAL_DURATION_MS = 30000;

// Feed RAW magnetometer values into the calibration min/max collector while
// calibration is active. Must use raw (pre-offset) values — the whole point
// of calibration is to FIND the hard-iron offsets, so collecting already-
// offset-corrected values would bias the result. Called from hmc_read().
static void hmc_cal_update(float rawX, float rawY, float rawZ) {
  if (!g_hmcCalActive) return;
  g_hmcMinX = min(g_hmcMinX, rawX); g_hmcMaxX = max(g_hmcMaxX, rawX);
  g_hmcMinY = min(g_hmcMinY, rawY); g_hmcMaxY = max(g_hmcMaxY, rawY);
  g_hmcMinZ = min(g_hmcMinZ, rawZ); g_hmcMaxZ = max(g_hmcMaxZ, rawZ);
}

static volatile bool g_ofCalPending = false;
static float g_ofCalDistCm=0,g_ofCalAccumX=0,g_ofCalAccumY=0;

static void ble_cal_notify(const char *msg) {
  if (bleCharCalStatus) {
    bleCharCalStatus->setValue(msg);
    bleCharCalStatus->notify();
  }
  Serial.printf("[CAL] %s\n", msg);
}

// ─────────────────────────────────────────────────────────────
//  BLE configuration server — plain text interface
//  Device: "SensoryCortex"
//
//  HOW TO USE:
//    Connect with nRF Connect or LightBlue app.
//    All values are plain readable text — no bytes, no data types.
//
//  LIVE READINGS (read / auto-notify every 1s):
//    Status    — "OK" or "FAULT: compass altitude"
//    Altitude  — "1.42m"
//    Heading   — "284 deg"
//    Map       — "247 / 5000 points"
//
//  CALIBRATION (write to trigger, read for live progress):
//    Compass   — write "go" → reads "collecting - 24s left"
//                → finally reads "done X=-127 Y=44 Z=8"
//    OF scale  — fly a known distance, write cm e.g. "150"
//                → reads "scale updated 0.001480"
//    Cal log   — notify-only, updates during calibration
//
//  TUNING (read/write):
//    Map TTL   — write "60" (seconds, 5–600)
//    Baro wt   — write "0.6" (0.0–1.0)
//    OF weight — write "0.8" (0.0–1.0)
//    Map clear — write "clear" to wipe map
// ─────────────────────────────────────────────────────────────

// Plain text write handler
class BLEWriteCallback : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic *c) override {
    String uuid = c->getUUID().toString();
    String sval = c->getValue();
    sval.trim();

    if (uuid == BLE_CHAR_TTL_UUID) {
      int s = sval.toInt();
      if (s >= 5 && s <= 600) {
        g_pointTTLms = (uint32_t)s * 1000UL;
        char b[8]; snprintf(b,8,"%d",s);
        c->setValue(b);
        prefs_save();
      } else c->setValue("error: use 5-600 seconds");
    }
    else if (uuid == BLE_CHAR_BAROWT_UUID) {
      float v = sval.toFloat();
      if (v >= 0.0f && v <= 1.0f) {
        v = fmaxf(0.05f, fminf(0.95f, v));  // [v2.0.3 S-012] keep both sensors in the fusion
        g_baroWeight = v;
        char b[8]; snprintf(b,8,"%.2f",v);
        c->setValue(b);
        prefs_save();
      } else c->setValue("error: use 0.0-1.0");
    }
    else if (uuid == BLE_CHAR_OFWT_UUID) {
      // Plain "0.0-1.0" sets OF weight (legacy). "cvel ..." sets the v1.9.2
      // closing-velocity fusion tunables on the same characteristic.
      if (sval.startsWith("cvel alpha ")) {
        float v = sval.substring(11).toFloat();
        if (v >= 0.05f && v <= 1.0f) { g_cvAlpha = v;
          char b[20]; snprintf(b,20,"cvel alpha %.2f",v); c->setValue(b); prefs_save();
        } else c->setValue("error: alpha 0.05-1.0");
      } else if (sval.startsWith("cvel gate ")) {
        float v = sval.substring(10).toFloat();
        if (v >= 100.0f && v <= 3000.0f) { g_cvOutlierCms = v;
          char b[24]; snprintf(b,24,"cvel gate %.0f",v); c->setValue(b); prefs_save();
        } else c->setValue("error: gate 100-3000");
      } else if (sval.startsWith("cvel inertial ")) {
        float v = sval.substring(14).toFloat();
        if (v >= 0.0f && v <= 1.0f) { g_cvInertialW = v;
          char b[24]; snprintf(b,24,"cvel inertial %.2f",v); c->setValue(b); prefs_save();
        } else c->setValue("error: inertial 0.0-1.0");
      } else if (sval == "cvel status" || sval == "cvel") {
        char b[40]; snprintf(b,40,"cvel a%.2f g%.0f i%.2f",
                             g_cvAlpha, g_cvOutlierCms, g_cvInertialW);
        c->setValue(b);
      } else if (sval.startsWith("cvel")) {
        c->setValue("error: bad cvel cmd");
      } else {
        float v = sval.toFloat();
        if (v >= 0.0f && v <= 1.0f) {
          g_ofWeight = v;
          char b[8]; snprintf(b,8,"%.2f",v);
          c->setValue(b);
          prefs_save();
        } else c->setValue("error: use 0.0-1.0");
      }
    }
    else if (uuid == BLE_CHAR_HMCCAL_UUID) {
      if (sval.equalsIgnoreCase("go")) {
        g_hmcMinX=g_hmcMinY=g_hmcMinZ= 99999.0f;
        g_hmcMaxX=g_hmcMaxY=g_hmcMaxZ=-99999.0f;
        g_hmcCalStartMs = millis();
        g_hmcCalActive  = true;
        c->setValue("collecting - rotate drone slowly through all axes");
        ble_cal_notify("Compass cal started - rotate drone 30s");
      }
    }
    else if (uuid == BLE_CHAR_OFCAL_UUID) {
      float d = sval.toFloat();
      if (d > 10.0f && d < 2000.0f) {
        g_ofCalDistCm  = d;
        g_ofCalAccumX  = g_ofCalAccumY = 0.0f;
        g_ofCalPending = true;
        c->setValue("calculating - hover still now");
        ble_cal_notify("OF scale calculating");
      } else c->setValue("error: enter distance flown in cm");
    }
    else if (uuid == BLE_CHAR_MAPCLEAR_UUID) {
      if (sval.equalsIgnoreCase("clear")) {
        mapCount = 0; hash_init();
        c->setValue("map cleared");
      } else if (sval.equalsIgnoreCase("reset")) {
        // Wipe saved tunings/calibration from flash. Compiled defaults take
        // effect on next boot. (Current session keeps running values.)
        prefs_reset();
        c->setValue("saved settings reset - defaults on reboot");
      }
    }
  }
};
static BLEWriteCallback bleWriteCb;

static void ble_init() {
  BLEDevice::init("SensoryCortex");
  BLEServer  *srv = BLEDevice::createServer();
  BLEService *svc = srv->createService(BLEUUID(BLE_SERVICE_UUID), 48);
  uint32_t RW  = BLECharacteristic::PROPERTY_READ|BLECharacteristic::PROPERTY_WRITE;
  uint32_t RN  = BLECharacteristic::PROPERTY_READ|BLECharacteristic::PROPERTY_NOTIFY;
  uint32_t RWN = RW|BLECharacteristic::PROPERTY_NOTIFY;
  auto mk=[&](const char*u,uint32_t p)->BLECharacteristic*{
    auto*c=svc->createCharacteristic(u,p);
    c->addDescriptor(new BLE2902()); return c; };

  bleCharStatus   = mk(BLE_CHAR_STATUS_UUID,   RN);
  bleCharAltitude = mk(BLE_CHAR_ALTITUDE_UUID, RN);
  bleCharHeading  = mk(BLE_CHAR_HEADING_UUID,  RN);
  bleCharMapInfo  = mk(BLE_CHAR_MAPINFO_UUID,  RN);
  bleCharHmcCal   = mk(BLE_CHAR_HMCCAL_UUID,   RWN);
  bleCharOfCal    = mk(BLE_CHAR_OFCAL_UUID,    RWN);
  bleCharCalStatus= mk(BLE_CHAR_CALSTAT_UUID,  RN);
  bleCharTTL      = mk(BLE_CHAR_TTL_UUID,      RW);
  bleCharBaroWt   = mk(BLE_CHAR_BAROWT_UUID,   RW);
  bleCharOfWt     = mk(BLE_CHAR_OFWT_UUID,     RW);
  bleCharMapClear = mk(BLE_CHAR_MAPCLEAR_UUID,  RW);

  bleCharHmcCal->setCallbacks(&bleWriteCb);
  bleCharOfCal->setCallbacks(&bleWriteCb);
  bleCharTTL->setCallbacks(&bleWriteCb);
  bleCharBaroWt->setCallbacks(&bleWriteCb);
  bleCharOfWt->setCallbacks(&bleWriteCb);
  bleCharMapClear->setCallbacks(&bleWriteCb);

  bleCharStatus->setValue("starting");
  bleCharAltitude->setValue("0.00m");
  bleCharHeading->setValue("0 deg");
  bleCharMapInfo->setValue("0 points");
  bleCharHmcCal->setValue("write go to start compass cal");
  bleCharOfCal->setValue("write distance in cm after flying");
  bleCharCalStatus->setValue("idle");
  char ttlb[8]; snprintf(ttlb,8,"%lu",g_pointTTLms/1000UL);
  bleCharTTL->setValue(ttlb);
  char bwb[8]; snprintf(bwb,8,"%.2f",g_baroWeight);
  bleCharBaroWt->setValue(bwb);
  char owb[8]; snprintf(owb,8,"%.2f",g_ofWeight);
  bleCharOfWt->setValue(owb);
  bleCharMapClear->setValue("write clear to reset map");

  svc->start();
  BLEAdvertising *adv = BLEDevice::getAdvertising();
  adv->addServiceUUID(BLE_SERVICE_UUID);
  adv->setScanResponse(true);
  adv->setMinPreferred(0x06);
  BLEDevice::startAdvertising();
  Serial.println("[OK] BLE 'SensoryCortex' advertising");
}

// Update BLE at ~1 Hz — plain text, human readable
static void ble_update_notify(uint8_t faultByte) {
  // Status
  if (bleCharStatus) {
    if (!faultByte) {
      bleCharStatus->setValue("OK");
    } else {
      String s = "FAULT:";
      if (faultByte&0x01) s += " front";
      if (faultByte&0x02) s += " left";
      if (faultByte&0x04) s += " right";
      if (faultByte&0x08) s += " compass";
      if (faultByte&0x10) s += " altitude";
      if (faultByte&0x20) s += " flow";
      if (faultByte&0x40) s += " back";
      if (faultByte&0x80) s += " baro";
      bleCharStatus->setValue(s.c_str());
    }
    bleCharStatus->notify();
  }
  // Altitude
  if (bleCharAltitude) {
    char b[16]; snprintf(b,16,"%.2fm",g_altM);
    bleCharAltitude->setValue(b);
    bleCharAltitude->notify();
  }
  // Heading
  if (bleCharHeading) {
    float y = atan2f(2.0f*(q1*q2+q0*q3),
                     q0*q0+q1*q1-q2*q2-q3*q3) * (180.0f/3.14159f);
    if (y < 0) y += 360.0f;
    char b[16]; snprintf(b,16,"%.0f deg",(double)y);
    bleCharHeading->setValue(b);
    bleCharHeading->notify();
  }
  // Map info
  if (bleCharMapInfo) {
    char b[32]; snprintf(b,32,"%d / %d points",mapCount,MAP_SIZE);
    bleCharMapInfo->setValue(b);
    bleCharMapInfo->notify();
  }
  // Compass calibration progress
  if (g_hmcCalActive && bleCharHmcCal) {
    uint32_t el = millis() - g_hmcCalStartMs;
    if (el < HMC_CAL_DURATION_MS) {
      uint32_t rem = (HMC_CAL_DURATION_MS - el) / 1000;
      char b[40]; snprintf(b,40,"collecting - %lus left", rem);
      bleCharHmcCal->setValue(b);
      bleCharHmcCal->notify();
      ble_cal_notify(b);
    } else {
      g_hmcCalActive = false;
      float ox=(g_hmcMaxX+g_hmcMinX)/2;
      float oy=(g_hmcMaxY+g_hmcMinY)/2;
      float oz=(g_hmcMaxZ+g_hmcMinZ)/2;
      HMC_OFF_X=ox; HMC_OFF_Y=oy; HMC_OFF_Z=oz;
      prefs_save();   // persist compass offsets — survives power cycle
      char b[64];
      snprintf(b,64,"done X=%.0f Y=%.0f Z=%.0f",(double)ox,(double)oy,(double)oz);
      bleCharHmcCal->setValue(b);
      bleCharHmcCal->notify();
      ble_cal_notify(b);
    }
  }
  // OF scale calibration result
  if (g_ofCalPending && bleCharOfCal) {
    g_ofCalPending = false;
    float pd = sqrtf(g_ofCalAccumX*g_ofCalAccumX + g_ofCalAccumY*g_ofCalAccumY);
    if (pd > 0.1f && g_ofCalDistCm > 0.0f && g_altM > 0.05f) {
      float ns = (g_ofCalDistCm * 0.01f) / (pd * g_altM);
      if (ns > 0.00001f && ns < 0.1f) {
        g_ofScale = ns;
        prefs_save();   // persist optical-flow scale — survives power cycle
        char b[40]; snprintf(b,40,"scale %.5f",ns);
        bleCharOfCal->setValue(b);
        bleCharOfCal->notify();
        ble_cal_notify(b);
      } else {
        bleCharOfCal->setValue("error: result out of range, try again");
      }
    } else {
      bleCharOfCal->setValue("error: fly further or check altitude");
    }
    g_ofCalAccumX = g_ofCalAccumY = 0.0f;
  }
}

// ─────────────────────────────────────────────────────────────
//  ProximityCmd parser — ring-buffer state machine on Serial2
//  Called once per main loop. Non-blocking.
//  Validates XOR checksum before applying any values.
// ─────────────────────────────────────────────────────────────

static void poll_prox_cmd() {
  static uint8_t buf[sizeof(ProximityCmd)];
  static uint8_t pos    = 0;
  static bool    synced = false;

  while (Serial2.available()) {
    uint8_t b = Serial2.read();

    if (!synced) {
      if (b == PKT_PROX_CMD) { buf[0] = b; pos = 1; synced = true; }
      continue;
    }

    buf[pos++] = b;

    if (pos == sizeof(ProximityCmd)) {
      synced = false; pos = 0;

      // Validate end marker
      if (buf[sizeof(ProximityCmd)-1] != PKT_END) continue;

      // Validate XOR checksum
      ProximityCmd *p = (ProximityCmd*)buf;
      uint8_t *sb = (uint8_t*)&p->scale;
      uint8_t  cs = sb[0] ^ sb[1] ^ sb[2] ^ sb[3] ^ p->landingMode;
      if (cs != p->checksum) continue;  // corrupted — discard

      // Sanity-check scale range
      if (p->scale < 0.1f || p->scale > 5.0f) continue;

      // Apply — update dynamic thresholds
      g_proxScale        = p->scale;
      g_threshStaticCm   = THRESH_BASE_STATIC_CM  * p->scale;
      g_threshDynamicCm  = THRESH_BASE_DYNAMIC_CM * p->scale;
      g_landingMode      = (p->landingMode != 0);
    }
  }
}

// ─────────────────────────────────────────────────────────────
//  Setup
// ─────────────────────────────────────────────────────────────

void setup() {
  Serial.begin(115200);
  delay(300);
  Serial.println("
[BOOT] Sensory Cortex v2.0.4");

  // ── Disable WiFi — BLE only ──────────────────────────────────
  // WiFi disabled permanently — BLE handles all wireless config.
  // Saves ~40KB RAM and eliminates RF noise near magnetometer.
  esp_wifi_stop();
  esp_wifi_deinit();
  Serial.println("[OK] WiFi disabled — BLE only");

  // ── Load saved tunings & calibration from flash ──
  // Overrides compiled defaults with whatever was last calibrated/tuned.
  // First boot after flashing finds nothing and keeps the defaults.
  prefs_load();
  Serial.printf("[OK] Loaded saved tunings: HMC(%.0f,%.0f,%.0f) ofScale=%.5f\n",
    (double)HMC_OFF_X,(double)HMC_OFF_Y,(double)HMC_OFF_Z,(double)g_ofScale);

  Serial1.begin(921600, SERIAL_8N1, UART_RX, UART_TX);

  // ── Comm Cortex command link (UART2) ──
  Serial2.begin(921600, SERIAL_8N1, COMM_CMD_RX, COMM_CMD_TX);
  Serial.println("[OK] Comm Cortex cmd link on GPIO 1/2");

  // I2C at 1 MHz — TF-Luna and ICM-42688-P both support it
  Wire.begin(I2C_SDA, I2C_SCL, CFG_I2C_SPEED);
  delay(80);

  // ── ICM-42688-P ──
  if (IMU.begin() < 0) {
    Serial.println("[FATAL] ICM-42688-P");
    while(true) delay(1000);
  }
  IMU.setGyroODR(ICM42688::odr1k);
  IMU.setGyroFS(ICM42688::dps2000);
  IMU.setAccelODR(ICM42688::odr1k);
  IMU.setAccelFS(ICM42688::gpm8);
  Serial.println("[OK] ICM-42688-P 1kHz ±2000dps ±8g");

  // ── HMC5883L ──
  if (!hmc_init()) {
    Serial.println("[FATAL] HMC5883L");
    while(true) delay(1000);
  }
  Serial.println("[OK] HMC5883L 75Hz");

  // ── TF-Luna sensors ──
  // Index: 0=Front 1=Left 2=Right 3=Back 4=Down
  const uint8_t la[]={LUNA_FRONT,LUNA_LEFT,LUNA_RIGHT,LUNA_BACK,LUNA_DOWN};
  const char*   ln[]={"Front(0x10)","Left(0x11)","Right(0x12)",
                       "Back(0x13)","Down(0x14)"};
  const bool    le[]={true,true,true,
                      (bool)LUNA_BACK_ENABLED,(bool)LUNA_DOWN_ENABLED};
  for (int i=0;i<5;i++) {
    if (!le[i]) { Serial.printf("[--  ] TF-Luna %s disabled\n",ln[i]); continue; }
    int d=luna_read(la[i]);
    Serial.printf("[%s] TF-Luna %s %s\n",
      d>=0?"OK  ":"WARN", ln[i], d>=0?"ready":"no response");
  }

  // ── Map buffer allocation ────────────────────────────────────
  // mapBuf goes to PSRAM when USE_PSRAM=1 (ps_malloc).
  // Falls back to heap (new[]) without PSRAM — smaller MAP_SIZE used.
  #if USE_PSRAM
  mapBuf = (MapPoint*)ps_malloc(MAP_SIZE * sizeof(MapPoint));
  if (!mapBuf) {
    Serial.println("[FATAL] ps_malloc failed — check PSRAM enabled in Tools menu");
    while(true) delay(1000);
  }
  Serial.printf("[OK] mapBuf: %u points × %u bytes = %u KB in PSRAM\n",
    MAP_SIZE, (unsigned)sizeof(MapPoint),
    (unsigned)(MAP_SIZE * sizeof(MapPoint) / 1024));
  #else
  mapBuf = new MapPoint[MAP_SIZE];
  if (!mapBuf) {
    Serial.println("[FATAL] map heap alloc failed");
    while(true) delay(1000);
  }
  Serial.printf("[OK] mapBuf: %u points in internal SRAM\n", MAP_SIZE);
  #endif
  memset(mapBuf, 0, MAP_SIZE * sizeof(MapPoint));

  // ── Hash grid init ──
  hash_init();

  // ── Optical flow sensor ──
  #if OF_ENABLED
  if (of_init()) {
    Serial.println("[OK] Optical flow (PMW3901/PAA5100)");
  } else {
    Serial.println("[WARN] Optical flow init failed — velocity correction disabled");
  }
  #else
  Serial.println("[INFO] Optical flow disabled (OF_ENABLED=0)");
  #endif

  // ── Barometer (BMP280) ──
  #if BMP_ENABLED
  if (bmp_init()) {
    Serial.println("[OK] BMP280 barometer");
    // Capture ground reference pressure now (drone must be still on the
    // ground at power-up). Without this g_baroRefPa stays 0 and baro
    // fusion does nothing.
    bmp_calibrate_ground();
  } else {
    Serial.println("[WARN] BMP280 init failed — baro fusion disabled");
  }
  #else
  Serial.println("[INFO] Barometer disabled (BMP_ENABLED=0)");
  #endif

  // ── Downward TF-Luna (altitude) ──
  #if LUNA_DOWN_ENABLED
  {
    int d = luna_read(LUNA_DOWN);
    Serial.printf("[%s] TF-Luna Down (0x%02X) %s\n",
      d>=0?"OK  ":"WARN", LUNA_DOWN, d>=0?"ready":"no response");
  }
  #else
  Serial.println("[INFO] Altitude sensor disabled (LUNA_DOWN_ENABLED=0)");
  #endif

  // ── BLE configuration server ──
  ble_init();

  // ── AHRS warm-up (3 s) ──
  Serial.println("[INFO] AHRS warm-up 3s...");
  unsigned long t0=millis();
  unsigned long prevUs=micros();
  while (millis()-t0 < 3000) {
    unsigned long now=micros();
    float dt=(float)(now-prevUs)*1e-6f;
    prevUs=now;
    if (dt<0.001f||dt>0.1f){delay(5);continue;}
    IMU.getAGT();
    float mx=0,my=0,mz=0;
    bool mok=hmc_read(mx,my,mz);
    // gyr in deg/s → rad/s for Mahony; acc in g (normalised internally)
    mahony_update(
      IMU.gyrX()*0.01745329f, IMU.gyrY()*0.01745329f, IMU.gyrZ()*0.01745329f,
      IMU.accX(), IMU.accY(), IMU.accZ(),
      mok?mx:0,mok?my:0,mok?mz:0, dt);
    delay(5);
  }
  build_rot_mat();

  // ── Accel bias starts at zero ──
  // Gravity is removed explicitly in integrate_position(), and the
  // high-pass filter there converges on residual sensor drift in flight.
  // No manual gravity-laden seed needed (that would fight the removal).
  aBias.x = aBias.y = aBias.z = 0.0f;
  Serial.printf("[INFO] MAP_SIZE=%d  HASH_SIZE=%d  sizeof(MapPoint)=%d\n",
    MAP_SIZE, HASH_SIZE, (int)sizeof(MapPoint));
  // [v2.0.3 S-007] Derive declination rotation from the single CFG value.
  MAG_DECL_RAD = CFG_MAG_DECLINATION_DEG * 0.017453293f;
  COS_DECL = cosf(MAG_DECL_RAD);
  SIN_DECL = sinf(MAG_DECL_RAD);
  Serial.printf("[OK] Declination: %.2f deg (cos=%.5f sin=%.5f)\n",
                CFG_MAG_DECLINATION_DEG, COS_DECL, SIN_DECL);

  Serial.println("[INFO] Running.");

  // ── Hardware Task Watchdog ──                          [v2.0.2]
  // Subscribe AFTER the 3s AHRS warm-up so the warm-up loop can't
  // trip it. All sensory work runs in loop() (this loopTask), so we
  // watch this task directly. 2s / panic-reset: a wedged I2C bus
  // (dead TF-Luna holding SDA) or a hung loop reboots the chip.
  {
    esp_task_wdt_config_t cfg = {
      .timeout_ms     = 2000,
      .idle_core_mask = 0,
      .trigger_panic  = true,
    };
    esp_err_t e = esp_task_wdt_init(&cfg);
    if (e == ESP_ERR_INVALID_STATE) esp_task_wdt_reconfigure(&cfg);
    esp_task_wdt_add(NULL);          // watch the main loop task
    Serial.println("[OK] TWDT armed: 2000ms panic-reset");
  }

  lastLoopUs=micros();
}

// ─────────────────────────────────────────────────────────────
//  Main loop
// ─────────────────────────────────────────────────────────────

void loop() {
  esp_task_wdt_reset();   // v2.0.2: pet at top — every iteration, before I2C

  // ── 0. Poll ProximityCmd from Comm Cortex (non-blocking) ──
  poll_prox_cmd();

  // ── Precise timing ──
  unsigned long nowUs = micros();
  float dt = (float)(nowUs - lastLoopUs) * 1e-6f;
  lastLoopUs = nowUs;
  if (dt < 0.0001f || dt > 0.05f) dt = 0.005f;
  uint32_t nowMs = millis();

  // ── 1. IMU ──
  // getAGT() reads accel + gyro + temp in one transaction.
  // Finani ICM42688: accX/Y/Z() return g, gyrX/Y/Z() return deg/s.
  // Mahony needs gyro in rad/s, so convert (×π/180 = ×0.01745329).
  // Accel stays in g — Mahony normalises it, so units cancel.
  IMU.getAGT();
  const float DEG2RAD = 0.01745329252f;
  float gx=IMU.gyrX()*DEG2RAD, gy=IMU.gyrY()*DEG2RAD, gz=IMU.gyrZ()*DEG2RAD;
  float ax=IMU.accX(), ay=IMU.accY(), az=IMU.accZ();

  // ── 2. Magnetometer (every 3rd loop to match 75 Hz output rate) ──
  float mx=0,my=0,mz=0;
  bool magOk=false;
  if (++hmcPhase >= 3) {
    hmcPhase=0;
    magOk=hmc_read(mx,my,mz);
  }
  if (magOk) {
    magFailMs = 0;
    // (Calibration min/max collection happens inside hmc_read() using RAW
    //  values — see hmc_cal_update. Nothing to do here.)
  } else { magFailMs += (uint32_t)(dt*1000); }

  // ── 3. AHRS update ──
  mahony_update(gx,gy,gz,ax,ay,az,
    magOk?mx:0, magOk?my:0, magOk?mz:0, dt);

  // ── 4. Cache rotation matrix (ONE build, used everywhere this loop) ──
  build_rot_mat();

  // ── 5. Position integration (uses cached R) ──
  integrate_position(ax,ay,az,dt);

  // ── 5a. Altitude update (downward TF-Luna) ──
  // Corrects pos.z before optical flow reads it for height scaling.
  // Called every loop — altitude sensor supports up to 250 Hz.
  alt_update(dt);

  // ── 5b. Optical flow update ──
  // Blends ground-truth velocity into vel.x/y, weighted by g_ofWeight.
  // Uses current altitude (g_altM) to scale pixel motion to metres/s.
  // Falls through silently if OF_ENABLED=0.
  // Pass gyro rates for rotational flow compensation.
  // gx=pitch rate, gy=roll rate (rad/s, body frame, from ICM-42688-P).
  // These are the same values read at step 1 this loop iteration.
  of_update(dt, g_altM, gx, gy);

  // ── 6. TF-Luna reads — all 5 sensors ────────────────────────
  // Index: 0=Front 1=Left 2=Right 3=Back 4=Down
  // Down sensor (4) used for altitude only — skip avoidance map push.
  const uint8_t addrs[5] = {
    LUNA_FRONT, LUNA_LEFT, LUNA_RIGHT, LUNA_BACK, LUNA_DOWN
  };
  int dist[5];
  for (int s=0; s<5; s++) {
    // Skip disabled sensors
    if (s==3 && !LUNA_BACK_ENABLED)  { dist[s]=-1; continue; }
    if (s==4 && !LUNA_DOWN_ENABLED)  { dist[s]=-1; continue; }
    dist[s] = luna_read(addrs[s]);
    if (dist[s] > 0) {
      if (lastDist[s] > 0 && dt > 0.0001f) {
        // ── Closing velocity fusion (v1.9.2) ──
        // 1) Raw LiDAR-beam closing velocity (cm/s). Negative = closing
        //    (distance shrinking). Note (dist - lastDist): shrinking → negative.
        float rawLidar = ((float)dist[s] - lastDist[s]) / dt;  // cm/s, +ve = opening

        // 2) Outlier rejection: a single implausible jump (glint/dropout)
        //    exceeding the gate is ignored — hold the previous filtered value.
        if (fabsf(rawLidar) > g_cvOutlierCms) {
          // reject: decay slightly toward 0 rather than trust the spike
          sensorVel[s] *= 0.8f;
        } else {
          // 3) Inertial/OF projection: project body velocity onto THIS
          //    sensor's axis. Body vx=forward(+front), vy=right(+right).
          //    Closing contribution from own motion = +velocity toward sensor.
          //    Convert m/s → cm/s (×100). Sign so that moving TOWARD the
          //    sensor makes the fused closing MORE negative (closing).
          float ownToward_cms = 0.0f;   // +ve = moving toward this sensor (cm/s)
          switch (s) {
            case 0: ownToward_cms =  of_body_vx * 100.0f; break; // FRONT: +fwd
            case 1: ownToward_cms = -of_body_vy * 100.0f; break; // LEFT:  +left=-right
            case 2: ownToward_cms =  of_body_vy * 100.0f; break; // RIGHT: +right
            case 3: ownToward_cms = -of_body_vx * 100.0f; break; // BACK:  +back=-fwd
            default: ownToward_cms = 0.0f; break;                // DOWN: n/a
          }
          // Confidence weight: full inertial weight only when OF is fresh.
          float infW = of_ok ? g_cvInertialW : 0.0f;
          // Fused closing velocity in cm/s, positive = opening:
          //   LiDAR delta gives relative closing already; the OF term refines
          //   it toward TRUE relative closing when the beam target moves too.
          //   own motion toward sensor is a CLOSING component.
          float fused = rawLidar - infW * ownToward_cms;

          // 4) Low-pass filter (exponential), tunable α.
          sensorVel[s] = sensorVel[s]*(1.0f - g_cvAlpha) + fused*g_cvAlpha;
        }
      }
      lastDist[s] = (float)dist[s];
      failCount[s] = 0;
      if (goodStreak[s] < 255) goodStreak[s]++;          // [v2.0.3 S-024]
      if (goodStreak[s] >= FAULT_CLEAR_N) sensorFaulted[s] = false;
    } else {
      if (failCount[s] < 255) failCount[s]++;
      goodStreak[s] = 0;                                  // [v2.0.3 S-024]
      if (failCount[s] >= FAULT_SET_N) sensorFaulted[s] = true;
      sensorVel[s] *= 0.9f;  // decay velocity on miss (retained)
    }
  }

  // ── 7. Map update (uses cached R) ── all 5 sensors ──────────
  // Body frame convention:
  //   Front → +X,  Back → −X,  Left → +Y,  Right → −Y,  Down → +Z
  // Down sensor (s==4) handled separately — altitude, not mapping.
  for (int s=0; s<4; s++) {   // 0-3 only: front/left/right/back
    if (dist[s] <= 0) continue;
    float bx=0, by=0, bz=0;
    float d = (float)dist[s];
    switch(s) {
      case 0: bx =  d; break;  // front → +X
      case 1: by =  d; break;  // left  → +Y
      case 2: by = -d; break;  // right → −Y
      case 3: bx = -d; break;  // back  → −X
    }
    MapPoint p = body_to_world(bx, by, bz, nowMs);
    map_push(p, nowMs);
  }

  // ── 8. Expire stale map points (every 100 loops ≈ 0.5 Hz) ──
  if (++expireN >= 100) { expireN=0; map_expire(nowMs); }

  // ── 8r. Raycast cleanup (every 15 loops ≈ 12 Hz) ──
  // Remove mapped points the sensors can now see through — corrects
  // dead-reckoning drift-ghosts and clears obstacles that have gone.
  // Reduced rate: O(mapCount) per sensor, off the hot path.
  static uint8_t raycastN = 0;
  if (++raycastN >= 15) { raycastN = 0; raycast_clear(dist, nowMs); }

  // ── 8b. Map-based potential field (every 10 loops ≈ 18 Hz) ──
  // Secondary avoidance: repulsion from mapped obstacles in a bubble.
  // Reduced rate — map and pose change slowly vs the sensor loop, and
  // this is an O(mapCount) scan we don't want on the hot path.
  static uint8_t mapForceN_ctr = 0;
  if (++mapForceN_ctr >= 10) { mapForceN_ctr = 0; compute_map_force(nowMs); }

  // ── 8a. BLE notify (every ~200 loops ≈ 1 Hz — low priority) ──
  static uint8_t bleNotifyN = 0;
  if (++bleNotifyN >= 200) {
    bleNotifyN = 0;
    uint8_t liveFaults = 0;
    if (sensorFaulted[0]) liveFaults|=0x01;   // [v2.0.3 S-024] latched
    if (sensorFaulted[1]) liveFaults|=0x02;
    if (sensorFaulted[2]) liveFaults|=0x04;
    if (magFailMs>=500)   liveFaults|=0x08;
    if (!g_altOk)         liveFaults|=0x10;
    if (!of_ok)           liveFaults|=0x20;
    if (sensorFaulted[3]) liveFaults|=0x40;  // back sensor
    ble_update_notify(liveFaults);
  }

  // ── 9. Safety packet (every loop — 14 bytes, always fast) ──
  uint8_t faults = 0;
  if (sensorFaulted[0]) faults |= 0x01;   // [v2.0.3 S-024] latched
  if (sensorFaulted[1]) faults |= 0x02;
  if (sensorFaulted[2]) faults |= 0x04;
  if (magFailMs   >= 500) faults |= 0x08;
  if (!g_altOk)           faults |= 0x10;  // bit4 = altitude fault
  if (!of_ok && OF_ENABLED) faults |= 0x20; // bit5 = optical flow fault
  if (sensorFaulted[3]) faults |= 0x40;  // bit6 = back sensor fault [v2.0.3 S-024]
  if (!g_baroOk && BMP_ENABLED) faults |= 0x80; // bit7 = barometer fault

  // Velocity-compensated block threshold
  // Use dynamic thresholds — updated by Comm Cortex ch9 VRB via 0xCC packet
  float thresh0 = (sensorVel[0] < -100.0f) ? g_threshDynamicCm : g_threshStaticCm;
  float thresh3 = (sensorVel[3] < -100.0f) ? g_threshDynamicCm : g_threshStaticCm;

  SafetyPacket sp;
  sp.start        = 0xAA;
  sp.version      = SAFETY_PKT_VERSION;
  sp.frontBlocked = (dist[0]>0 && dist[0]<thresh0)                        ? 1:0;
  sp.leftBlocked  = (dist[1]>0 && dist[1]<g_threshStaticCm)                  ? 1:0;
  sp.rightBlocked = (dist[2]>0 && dist[2]<g_threshStaticCm)                  ? 1:0;
  sp.backBlocked  = (LUNA_BACK_ENABLED && dist[3]>0 && dist[3]<thresh3)       ? 1:0;
  sp.faults       = faults;
  sp.tteFront     = compute_tte(dist[0], sensorVel[0]);
  sp.tteLeft      = compute_tte(dist[1], sensorVel[1]);
  sp.tteRight     = compute_tte(dist[2], sensorVel[2]);
  sp.tteBack      = compute_tte(dist[3], sensorVel[3]);
  // Clamp velocity to int16 range (-32768..32767 cm/s)
  sp.velFront     = (int16_t)constrain(sensorVel[0], -32000, 32000);
  sp.velLeft      = (int16_t)constrain(sensorVel[1], -32000, 32000);
  sp.velRight     = (int16_t)constrain(sensorVel[2], -32000, 32000);
  sp.velBack      = (int16_t)constrain(sensorVel[3], -32000, 32000);
  sp.altCm        = g_altOk ? (uint16_t)constrain(g_altM*100.0f, 0, 65535) : 0;
  // ── Attitude (from Mahony quaternion) + vertical velocity ──
  // roll/pitch ×10 deg for the Comm Cortex active floor (true-height
  // correction + bank gating). vertVelCmS: from g_vertVelMS, which is
  // derived from CORRECTED altitude (LiDAR/baro), NOT drifting accel
  // integration. g_vertVelMS is +ve = climbing; the packet convention is
  // negative = descending, so NEGATE.
  {
    float pitchD = degrees(asinf(-2.0f*(q1*q3 - q0*q2)));
    float rollD  = degrees(atan2f(2.0f*(q0*q1 + q2*q3),
                                  q0*q0 - q1*q1 - q2*q2 + q3*q3));
    sp.rollDeg    = (int16_t)constrain(rollD  * 10.0f, -32000, 32000);
    sp.pitchDeg   = (int16_t)constrain(pitchD * 10.0f, -32000, 32000);
    // g_vertVelMS +ve = climbing → negate for "negative = descending"
    sp.vertVelCmS = (int16_t)constrain(-g_vertVelMS * 100.0f, -32000, 32000);
  }
  // Map-based repulsion, rotated to BODY frame (forward/right) so the Comm
  // Cortex can map it onto pitch/roll without needing attitude. ×1000, int16.
  {
    float fFwd = 0.0f, fRight = 0.0f;
    if (g_mapForceValid) map_force_to_body(fFwd, fRight);
    sp.mapForceN = (int16_t)constrain(fFwd  * 1000.0f, -32000, 32000); // forward
    sp.mapForceE = (int16_t)constrain(fRight* 1000.0f, -32000, 32000); // right
  }
  sp.mapForceValid= g_mapForceValid ? 1 : 0;
  // XOR checksum over all payload bytes (version .. mapForceValid inclusive),
  // i.e. everything between start and checksum. Computed last (v1.9.3).
  {
    const uint8_t *pb = (const uint8_t*)&sp;
    uint8_t cs = 0;
    // offset 1 (version) .. offsetof(checksum)-1 (mapForceValid)
    for (size_t i = 1; i < offsetof(SafetyPacket, checksum); i++) cs ^= pb[i];
    sp.checksum = cs;
  }
  sp.end          = 0x55;

  Serial1.write((const uint8_t*)&sp, sizeof(sp));

  // ── 10. Map packet (every MAP_TX_EVERY loops ≈ 9 Hz) ──
  if (++loopN >= MAP_TX_EVERY) {
    loopN = 0;
    // Compute checksum over all live map points
    fcs_reset();
    for (uint16_t i=0; i<mapCount; i++)
      fcs_feed((const uint8_t*)&mapBuf[i], sizeof(MapPoint));

    MapHeader mh;
    mh.start    = 0xBB;
    mh.count    = mapCount;
    mh.checksum = fcs_finish();
    mh.end      = 0x55;

    Serial1.write((const uint8_t*)&mh, sizeof(mh));
    for (uint16_t i=0; i<mapCount; i++)
      Serial1.write((const uint8_t*)&mapBuf[i], sizeof(MapPoint));
  }

  // ── 11. Debug output ──
#ifdef DEBUG_SERIAL
  float pitch=degrees(asinf(-2*(q1*q3-q0*q2)));
  float roll =degrees(atan2f(2*(q0*q1+q2*q3), q0*q0-q1*q1-q2*q2+q3*q3));
  float yaw  =degrees(atan2f(2*(q1*q2+q0*q3), q0*q0+q1*q1-q2*q2-q3*q3));
  Serial.printf(
    "F:%3d L:%3d R:%3d B:%3d D:%3d cm | "
    "vel F:%.0f L:%.0f R:%.0f B:%.0f cm/s | "
    "TTE F:%.1f L:%.1f R:%.1f B:%.1f s | "
    "P:%.1f R:%.1f Y:%.1f | "
    "pos N:%.2f E:%.2f D:%.2f | alt:%.2fm | "
    "OF vx:%.3f vy:%.3f | map:%d/%d ttl:%lus | faults:0x%02X\n",
    dist[0],dist[1],dist[2],dist[3],dist[4],
    sensorVel[0],sensorVel[1],sensorVel[2],sensorVel[3],
    sp.tteFront/10.0f,sp.tteLeft/10.0f,sp.tteRight/10.0f,sp.tteBack/10.0f,
    pitch,roll,yaw,
    pos.x,pos.y,pos.z, g_altM,
    of_vx, of_vy,
    mapCount, MAP_SIZE, g_pointTTLms/1000UL, faults
  );
#endif

  // ── 12. Precise loop pacing ──
  // Target 5 ms per loop (200 Hz).  Account for actual elapsed time.
  unsigned long elapsed = micros() - lastLoopUs;
  if (elapsed < 4800) delayMicroseconds(4800 - elapsed);
  // No delay if we overran — just continue immediately
}

/*
 * ================================================================
 *  SUPER MINI RECEIVER NOTES
 * ================================================================
 *
 *  Two packet types on the UART stream:
 *
 *  SafetyPacket (0xAA start, 0x55 end, 34 bytes total, version + XOR checksum)
 *    Arrives every loop ~180-200 Hz.  Parse immediately on receipt.
 *    Fields:
 *      frontBlocked/leftBlocked/rightBlocked  — 1=brake NOW
 *      faults  bit0=front sensor dead  bit1=left dead  bit2=right dead  bit3=heading degraded
 *      tteFront/tteLeft/tteRight  — tenths of second to impact (255=no threat)
 *      velFront/velLeft/velRight  — cm/s, negative=approaching
 *
 *  MapPacket (0xBB start, 0x55 end, 6-byte header + N×16 bytes)
 *    Arrives ~9 Hz.  Not time-critical — process between safety checks.
 *    Each MapPoint: float north, east, down (metres NED), uint32 stampMs
 *    Verify checksum before use.
 *
 *  Recommended receiver logic:
 *    1. Read bytes into ring buffer
 *    2. On 0xAA: parse 14-byte SafetyPacket, act immediately
 *    3. On 0xBB: read MapHeader, then read count×16 bytes, verify checksum
 *    4. Use TTE < 1.0s (tteFront < 10) as soft warning
 *    5. Use blocked flags OR TTE < 0.5s (tteFront < 5) as hard brake
 *
 * ================================================================
 *  CALIBRATION PROCEDURES
 * ================================================================
 *
 *  HMC5883L Hard-Iron Calibration
 *  ────────────────────────────────
 *  1. Enable DEBUG_SERIAL, add: Serial.printf("MAG %d %d %d\n",(int)mx,(int)my,(int)mz);
 *  2. Power drone fully (motors disarmed, ESCs live).
 *  3. Slowly rotate through all axes for 30 s.
 *  4. Collect min/max per axis.
 *  5. HMC_OFF_X = (max_x+min_x)/2, same for Y, Z.
 *
 *  Magnetic Declination Update
 *  ────────────────────────────
 *  https://www.ngdc.noaa.gov/geomag/calculators/magcalc.shtml
 *  Update MAG_DECL_RAD and recompute COS_DECL/SIN_DECL:
 *    COS_DECL = cosf(MAG_DECL_RAD)
 *    SIN_DECL = sinf(MAG_DECL_RAD)
 *
 *  TF-Luna Address Assignment
 *  ───────────────────────────
 *  Power one at a time. Write new address to reg 0x22, save with 0x00→reg 0x25.
 *
 *  Mahony Tuning
 *  ──────────────
 *  MAHONY_KP=2.0 default.  Oscillates→lower.  Drifts→raise.
 *
 *  I2C at 1 MHz — Hardware Notes
 *  ───────────────────────────────
 *  TF-Luna: supports up to 1 MHz per datasheet.
 *  ICM-42688-P: I2C mode supports 400 kHz officially; in practice
 *  runs reliably at 1 MHz on short traces (<10 cm). If you see
 *  read failures, reduce to 800000UL or 400000UL.
 *  HMC5883L: rated 400 kHz but typically works at 1 MHz on short traces.
 *  Add 4.7 kΩ pull-ups if bus errors occur at 1 MHz.
 *
 * ================================================================
 */
