// =============================================================================
//  Bike Safety Belt — Arduino UNO firmware (speaks the web dashboard protocol)
// =============================================================================
//  Board   : Arduino UNO (ATmega328P, incl. CH340 clones)
//  Baud    : 115200, 8N1, lines end with '\n'
//  Library : none to install — only Wire and EEPROM, which ship with the
//            Arduino AVR core. The MPU-6050 is driven directly by register.
//
//  Wiring (do not change):
//    MPU-6050 / GY-521 (0x68): SDA=A4  SCL=A5  VCC=5V  GND=GND
//    Red LED   D12 -> 220R -> LED -> GND
//    Green LED D13 -> 220R -> LED -> GND
//    Buzzer    D8  (driven with tone())
//    Button    D2  -> button -> GND (INPUT_PULLUP, pressed = LOW)
//
//  Protocol (matches app/js/main.js, transports.js, emergency.js):
//    Device -> app : JSON lines {"t":"hello"|"cfg"|"tel"|"state"|"det"|"inc"|"sos"|"ack",...}
//                    and human-readable "[TAG] text" lines (shown in the app console)
//    App -> device : HELLO, PING, CANCEL, RESET, SOSACK <id>, LOG, CLEARLOG,
//                    PROFILE scooter|standard|sport, CAL LEVEL|GYRO|CLEAR,
//                    SIM <n> (answered "not available on UNO"), BTN, COMMS <s>, GPSLOSS <s>, STATUS, HELP
//    Telemetry and detection run all the time, connected or not.
//
//  This is a safety PROTOTYPE: it cannot guarantee that an accident is detected
//  or that an alert is delivered. There is no GPS and no GSM on this hardware;
//  an SOS can only reach contacts through the dashboard.
// =============================================================================
#include <Wire.h>
#include <EEPROM.h>
#include <math.h>
#include <avr/pgmspace.h>

// =============================================================================
// CONFIGURATION — every tunable value lives here
// =============================================================================
#define PIN_RED            12
#define PIN_GREEN          13
#define PIN_BUZZER         8
#define PIN_BUTTON         2

#define SERIAL_BAUD        115200UL
#define MPU_ADDR           0x68
#define FW_VERSION         "1.0.0-uno"

// Sensor mounting: which GY-521 axis points to the FRONT of the bike and which
// points UP when the bike is upright. AXIS_PX/NX/PY/NY/PZ/NZ. Any residual tilt
// is removed with the "Calibrate riding orientation" button (CAL LEVEL).
#define MOUNT_FWD          AXIS_PX
#define MOUNT_UP           AXIS_PZ

#define PROFILE_DEFAULT    1          // 0 scooter, 1 standard, 2 sport

#define SAMPLE_HZ          100        // sensor + detector rate
#define TEL_PERIOD_MS      200        // JSON telemetry (5 Hz), always on
#define IMU_FAIL_LIMIT     5          // consecutive bad reads -> IMU_FAULT
#define IMU_STUCK_FRAMES   25         // identical frames -> frozen sensor
#define IMU_RETRY_MS       2000

#define CAL_SAMPLES        300        // 3 s startup gyro calibration
#define CAL_ATTEMPTS       5
#define CAL_GYRO_STD_MAX   2.0f       // deg/s allowed while "still"
#define CAL_GYRO_BIAS_MAX  25.0f
#define CAL_G_TOL          0.15f
#define CAL_LEVEL_SAMPLES  200
#define CAL_LEVEL_MAX_DEG  45.0f

#define FUSION_ALPHA       0.98f
#define FUSION_ACC_LPF     0.25f
#define FUSION_TRUST_G     0.10f
#define FUSION_TRUST_DPS   30.0f

// Detector (same algorithm and defaults as the main firmware)
#define DET_TRIG_ACC_G     1.8f
#define DET_TRIG_GYRO_DPS  90.0f
#define DET_TRIG_TILT_DPS  60.0f
#define DET_TRIG_K         2          // K of the last M samples anomalous
#define DET_TRIG_M         4
#define DET_QUIET_DPS      30.0f
#define DET_QUIET_G        0.5f
#define DET_QUIET_MS       500
#define DET_ACTIVE_MAX_MS  4000
#define DET_REST_DPS       10.0f
#define DET_REST_G         0.12f
#define DET_REST_MS        1000
#define DET_POST_MAX_MS    5000
#define DET_W_LEAN         0.15f
#define DET_W_RATE         0.20f
#define DET_W_IMPACT       0.15f
#define DET_W_CHANGE       0.20f
#define DET_W_POST         0.30f
#define DET_SUS_N_MS       100.0f
#define DET_SUS_C_MS       400.0f
#define DET_VERT_FRAC      0.80f
#define DET_VERT_WEIGHT    0.50f
#define DET_MAJOR          0.50f
#define DET_ACTIVE_LVL     0.50f
#define DET_MIN_IND        3
#define DET_POSSIBLE       0.30f
#define DET_CRASH          0.65f
#define DET_PARKED_MS      30000UL
#define DET_STILL_DPS      1.5f
#define DET_STILL_G        0.02f
#define DET_PARKED_FACTOR  0.60f
#define DET_REARM_MS       3000
#define DET_HYST_DEG       5.0f
// "Too much tilt": static tilt beyond the profile's abnormal lean for this long
// raises POSSIBLE_CRASH (alarm). Tilt alone never starts the SOS countdown.
#define TILT_ALARM_MS      1500

#define COUNTDOWN_S        10
#define CANCEL_SHOW_MS     3000UL
#define SOS_ALARM_MS       60000UL    // SOS alarm, then quiet RECOVERY
#define SOS_RETRY_MS       3000UL     // re-send SOS until the app sends SOSACK
#define BTN_DEBOUNCE_MS    30
#define BTN_LOCKOUT_MS     1500
#define BUZZER_HZ          2700

// =============================================================================
typedef const __FlashStringHelper* FS;
#define PF(s) (reinterpret_cast<FS>(s))

enum { AXIS_PX = 0, AXIS_NX, AXIS_PY, AXIS_NY, AXIS_PZ, AXIS_NZ };
static const float D2R = 0.01745329f, R2D = 57.2957795f;

struct V3 { float x, y, z; };

// ---- System states (names must match the dashboard) ------------------------
enum State : uint8_t { ST_BOOT, ST_CALIBRATING, ST_SAFE, ST_MONITORING, ST_POSSIBLE, ST_CRASH,
                       ST_CANCELLED, ST_SOS, ST_RECOVERY, ST_TEST, ST_IMU_FAULT };
const char sn0[] PROGMEM = "BOOT";            const char sn1[] PROGMEM = "CALIBRATING";
const char sn2[] PROGMEM = "SAFE";            const char sn3[] PROGMEM = "MONITORING";
const char sn4[] PROGMEM = "POSSIBLE_CRASH";  const char sn5[] PROGMEM = "CRASH_DETECTED";
const char sn6[] PROGMEM = "USER_CANCELLED";  const char sn7[] PROGMEM = "SOS_SENT";
const char sn8[] PROGMEM = "RECOVERY";        const char sn9[] PROGMEM = "MANUAL_TEST";
const char sn10[] PROGMEM = "IMU_FAULT";
const char* const STATE_NAMES[] PROGMEM = {sn0, sn1, sn2, sn3, sn4, sn5, sn6, sn7, sn8, sn9, sn10};

// ---- Incidents ----------------------------------------------------------------
enum { INC_AUTO = 0, INC_TEST = 1, INC_SIM = 2 };
enum { OUT_OPEN = 0, OUT_DISMISSED, OUT_CANCELLED, OUT_SOS, OUT_RESOLVED, OUT_TEST_CLEARED };
enum { F_SUSPECTED = 1, F_CONFIRMED = 2, F_CANCELLED = 4, F_SOS = 8, F_SOS_ACK = 16, F_GPS = 32,
       F_RESUMED = 64, F_IMU_LOST = 128 };
struct Inc {                     // 20 bytes, stored in an EEPROM ring
  uint32_t id, up;
  uint16_t boot;
  uint8_t type, out, flags, conf;
  int16_t acc, gyr, lean;        // centi-g, deg/s, deci-deg
};

// ---- Vehicle profiles (PROGMEM) ----------------------------------------------
struct Profile { float leanN, leanC, gyrN, gyrC, accN, accC, tcN, tcC, restN, restC; };
const Profile PROFILES[3] PROGMEM = {
  {35, 60, 100, 250, 2.0f, 4.0f, 25, 60, 20, 55},   // scooter
  {45, 65, 120, 280, 2.2f, 4.5f, 30, 65, 20, 60},   // standard
  {55, 72, 150, 320, 2.5f, 5.0f, 35, 70, 20, 60},   // sport
};
const char pn0[] PROGMEM = "scooter"; const char pn1[] PROGMEM = "standard"; const char pn2[] PROGMEM = "sport";
const char* const PROFILE_NAMES[] PROGMEM = {pn0, pn1, pn2};

// ---- EEPROM layout --------------------------------------------------------------
struct Settings {
  uint16_t magic;
  uint8_t ver, profile;
  uint16_t boot, dev;
  uint8_t flags;                 // 1 = level calibrated, 2 = gyro bias saved
  uint8_t activeState;           // 0 none, 1 countdown, 2 SOS (restart recovery)
  uint32_t activeId;
  V3 up, gbias;
  uint8_t ringHead, ringCount, crc;
};
// Detector report (also sent to the app as a "det" message)
struct Report {
  uint8_t trig, act, verdict;
  bool settled, vert, parked, post;
  float pkA, pkG, pkT, pkR, dT, sus, base, rest, conf;
  float s[5];
};


// -----------------------------------------------------------------------------
// Small helpers. NOTE: all types are declared above this point because the
// Arduino IDE inserts auto-generated function prototypes before the first
// function definition.
// -----------------------------------------------------------------------------
#define NOINLINE __attribute__((noinline))      // keeps float code small on the UNO
static NOINLINE V3 v3(float x, float y, float z) { V3 v = {x, y, z}; return v; }
static NOINLINE float vlen(const V3& v) { return sqrt(v.x * v.x + v.y * v.y + v.z * v.z); }
static NOINLINE V3 vsub(const V3& a, const V3& b) { return v3(a.x - b.x, a.y - b.y, a.z - b.z); }
static NOINLINE float clampf(float v, float lo, float hi) { return v < lo ? lo : (v > hi ? hi : v); }
static NOINLINE float rampf(float v, float lo, float hi) { return clampf((v - lo) / (hi - lo), 0, 1); }
static NOINLINE float wrapPi(float a) { while (a > PI) a -= TWO_PI; while (a < -PI) a += TWO_PI; return a; }
// Unsigned decimal parser (avoids pulling strtoul/atol into flash)
static uint32_t parseU(const char* p) { uint32_t v = 0; while (*p >= '0' && *p <= '9') v = v * 10 + (uint32_t)(*p++ - '0'); return v; }
static inline bool since(uint32_t now, uint32_t t0, uint32_t ms) { return (uint32_t)(now - t0) >= ms; }
static inline uint8_t pct(float v) { return (uint8_t)(clampf(v, 0, 1) * 100.0f + 0.5f); }
static FS stateName(uint8_t s) { return PF(pgm_read_ptr(&STATE_NAMES[s])); }
static FS profileName(uint8_t i) { return PF(pgm_read_ptr(&PROFILE_NAMES[i])); }

static const uint16_t EE_MAGIC = 0xB5B2;
static const uint16_t EE_RING = 64;
static const uint8_t RING_N = 8;

// =============================================================================
// Non-blocking serial output. Everything printed is queued and drained as the
// UART has room, so printing never stalls the 100 Hz sensor loop.
// =============================================================================
class TxQueue : public Print {
 public:
  bool muted = false;                         // "COMMS" communication-failure test
  size_t write(uint8_t c) override {
    if (muted) return 1;
    const uint8_t n = (uint8_t)(head_ + 1);
    while (n == tail_) drainOne();            // queue full (rare): wait for the UART
    buf_[head_] = c;
    head_ = n;
    return 1;
  }
  using Print::write;
  void pump() {
    while (tail_ != head_ && Serial.availableForWrite() > 0) Serial.write(buf_[tail_++]);
  }
  uint8_t freeSpace() const { return (uint8_t)(255 - (uint8_t)(head_ - tail_)); }
 private:
  void drainOne() {
    while (Serial.availableForWrite() <= 0) {}
    Serial.write(buf_[tail_++]);
  }
  uint8_t buf_[256];
  uint8_t head_ = 0, tail_ = 0;
};
static TxQueue tx;

// JSON helpers (keys/values from flash; numbers never printed as nan/inf)
static void jBegin(FS type) { tx.print(F("{\"t\":\"")); tx.print(type); tx.print('"'); }
static void jKey(FS k) { tx.print(F(",\"")); tx.print(k); tx.print(F("\":")); }
static void jI(FS k, long v) { jKey(k); tx.print(v); }
static void jF(FS k, float v, uint8_t d) {
  jKey(k);
  if (isnan(v) || isinf(v)) tx.print(F("null"));
  else tx.print(v, d);
}
static void jS(FS k, FS v) { jKey(k); tx.print('"'); tx.print(v); tx.print('"'); }
static void jEnd() { tx.print(F("}\n")); }
static void logTag(FS tag) { tx.print('['); tx.print(tag); tx.print(F("] ")); }

// =============================================================================
// Global state
// =============================================================================
static Settings cfg;
static Profile prof;
static char devName[9] = "BSB-0000";

// Sensor
static bool imuReady = false, imuHealthy = false;
static uint8_t failCount = 0, sameCount = 0, lastErr = 0;
static uint16_t lastHash = 0;
static uint32_t lastImuRetry = 0;
static int8_t mapIdx[3], mapSign[3];          // vehicle axis <- sensor axis
static V3 gBias = {0, 0, 0}, aOff = {0, 0, 0};
static float levelRoll = 0, levelPitch = 0;   // riding-orientation offsets (deg)
static bool levelSet = false;

// Calibration
static bool calibrating = false, levelCal = false;
static uint8_t calQuality = 0;                // 0 running, 1 ok, 2 degraded, 3 uncalibrated
static uint8_t calAttempts = 0;
static uint16_t calN = 0;
static V3 calMean, calM2, calAcc;

// Fusion
static float fPhi = 0, fTheta = 0;
static V3 accLp = {0, 0, 1};
static bool fInit = false;
static uint8_t attRef = 0;
static float roll = 0, pitch = 0, tilt = 0, rollRate = 0, pitchRate = 0;

// Latest sample / telemetry peaks
static float aMag = 1, wMag = 0, peakA = 0, peakG = 0;

// Detector
enum { PH_IDLE = 0, PH_ACTIVE = 1, PH_SETTLE = 2 };
enum { TRIG_ACC = 1, TRIG_GYRO = 2, TRIG_TILTRATE = 4, TRIG_SAT = 8, TRIG_STATIC_TILT = 16 };
enum { DEV_CAND = 1, DEV_POSS = 2, DEV_VERDICT = 4 };
enum { V_NONE = 0, V_CRASH = 1, V_DISMISSED = 2 };
static Report rep;
static uint8_t phase = PH_IDLE;
static bool armed = true, possibleRaised = false, staticAbn = false;
static float tiltHist[10];
static uint8_t histHead = 0, histCount = 0;
static uint32_t lastHistMs = 0;
static float tiltChg = 0, liveConf = 0;
static uint8_t anomShift = 0, pendTrig = 0;
static float preA = 0, preG = 0;
static bool preVert = false;
static float lpG = 0, lpA = 0;
static bool stillNow = false, longStill = false;
static uint32_t stillStart = 0, longStillMs = 0, phaseMs = 0, calmSince = 0, restSince = 0, uprightSince = 0;
static bool calm = false, resting = false, uprightTiming = false;
static bool tiltTiming = false, tiltAlarm = false;
static uint32_t tiltSince = 0;

// Controller
static uint8_t state = ST_BOOT;
static uint32_t stateSince = 0, countdownStart = 0, lockoutUntil = 0;
static bool incOpen = false;
static Inc inc;
static uint16_t seq = 0;
static bool sosPending = false;
static uint32_t sosLastTx = 0;
static uint16_t sosTx = 0;

// The on-device test simulator of the full firmware does not fit in the UNO's
// 32 KB flash: SIM commands are answered with an explanation instead.
static const uint8_t simId = 0;

// I/O
static bool btnStable = false, btnLastRaw = false;
static uint32_t btnChange = 0;
static bool buzzerOn = false;
static char rxLine[40];
static uint8_t rxLen = 0;
static bool rxOverflow = false;
static uint32_t nextSampleUs = 0, lastSampleUs = 0, lastTel = 0, mutedUntil = 0;
static bool greeted = false;

// Forward declarations
static void setState(uint8_t to, FS why);
static void sendState(uint8_t from, uint8_t to, FS why);
static void sendIncident(const Inc& r);

// =============================================================================
// EEPROM persistence
// =============================================================================
static uint8_t crc8(const uint8_t* d, uint16_t n) {
  uint8_t c = 0x5A;
  while (n--) {
    c ^= *d++;
    for (uint8_t i = 0; i < 8; ++i) c = (c & 0x80) ? (uint8_t)((c << 1) ^ 0x31) : (uint8_t)(c << 1);
  }
  return c;
}
static void saveSettings() {
  cfg.crc = crc8(reinterpret_cast<const uint8_t*>(&cfg), sizeof(cfg) - 1);
  EEPROM.put(0, cfg);                                   // put() only rewrites changed bytes
}
static void loadSettings(uint16_t seed) {
  EEPROM.get(0, cfg);
  const bool ok = cfg.magic == EE_MAGIC && cfg.ver == 1 && cfg.profile < 3 && cfg.ringHead < RING_N &&
                  cfg.ringCount <= RING_N && cfg.crc == crc8(reinterpret_cast<const uint8_t*>(&cfg), sizeof(cfg) - 1);
  if (!ok) {
    memset(&cfg, 0, sizeof(cfg));
    cfg.magic = EE_MAGIC;
    cfg.ver = 1;
    cfg.profile = PROFILE_DEFAULT;
    cfg.dev = seed ? seed : 0x5A5A;
    cfg.up = v3(0, 0, 1);
  }
}
static void setActive(uint8_t st, uint32_t id) {
  if (cfg.activeState == st && cfg.activeId == id) return;
  cfg.activeState = st;
  cfg.activeId = id;
  saveSettings();
}
static uint16_t slotAddr(uint8_t slot) { return EE_RING + slot * (sizeof(Inc) + 1); }
static uint8_t lastSlot = 0xFF;
static uint32_t lastSlotId = 0;
static void storeIncident(const Inc& r) {
  if (r.type == INC_SIM) return;                        // simulations are not persisted
  uint8_t slot;
  if (lastSlot != 0xFF && lastSlotId == r.id) slot = lastSlot;
  else {
    slot = cfg.ringHead;
    cfg.ringHead = (uint8_t)((cfg.ringHead + 1) % RING_N);
    if (cfg.ringCount < RING_N) cfg.ringCount++;
    saveSettings();
    lastSlot = slot;
    lastSlotId = r.id;
  }
  EEPROM.put(slotAddr(slot), r);
  EEPROM.update(slotAddr(slot) + sizeof(Inc), crc8(reinterpret_cast<const uint8_t*>(&r), sizeof(Inc)));
}
static bool readIncident(uint8_t newest, Inc& r) {
  if (newest >= cfg.ringCount) return false;
  const uint8_t slot = (uint8_t)((cfg.ringHead + RING_N - 1 - newest) % RING_N);
  EEPROM.get(slotAddr(slot), r);
  return EEPROM.read(slotAddr(slot) + sizeof(Inc)) == crc8(reinterpret_cast<const uint8_t*>(&r), sizeof(Inc));
}

// =============================================================================
// MPU-6050 (register level)
// =============================================================================
static bool mpuWrite(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(reg);
  Wire.write(val);
  return Wire.endTransmission() == 0;
}
static bool mpuRead(uint8_t reg, uint8_t* buf, uint8_t n) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom((uint8_t)MPU_ADDR, n) != n) { while (Wire.available()) Wire.read(); return false; }
  for (uint8_t i = 0; i < n; ++i) buf[i] = (uint8_t)Wire.read();
  return true;
}
static uint8_t whoAmI = 0;
static bool mpuBegin() {
  if (!mpuRead(0x75, &whoAmI, 1) || whoAmI == 0x00 || whoAmI == 0xFF) return false;
  if (!mpuWrite(0x6B, 0x01)) return false;        // wake, PLL clock
  if (!mpuWrite(0x1A, 3)) return false;           // DLPF 44 Hz (anti-aliasing at 100 Hz)
  if (!mpuWrite(0x19, 9)) return false;           // 1 kHz / (1+9) = 100 Hz
  if (!mpuWrite(0x1B, 0x18)) return false;        // gyro +/-2000 deg/s -> 16.4 LSB/(deg/s)
  if (!mpuWrite(0x1C, 0x18)) return false;        // accel +/-16 g      -> 2048 LSB/g
  uint8_t v;
  if (!mpuRead(0x6B, &v, 1) || (v & 0x40)) return false;
  if (!mpuRead(0x1C, &v, 1) || (v & 0x18) != 0x18) return false;
  sameCount = 0;
  return true;
}
// Mount mapping: vehicle X = forward, Y = left, Z = up
static void setupMount() {
  int8_t f[3] = {0, 0, 0}, u[3] = {0, 0, 0};
  f[MOUNT_FWD / 2] = (MOUNT_FWD & 1) ? -1 : 1;
  u[MOUNT_UP / 2] = (MOUNT_UP & 1) ? -1 : 1;
  const int8_t l[3] = {(int8_t)(u[1] * f[2] - u[2] * f[1]), (int8_t)(u[2] * f[0] - u[0] * f[2]),
                       (int8_t)(u[0] * f[1] - u[1] * f[0])};
  const int8_t* rows[3] = {f, l, u};
  for (uint8_t r = 0; r < 3; ++r)
    for (uint8_t c = 0; c < 3; ++c)
      if (rows[r][c]) { mapIdx[r] = (int8_t)c; mapSign[r] = rows[r][c]; }
}
static_assert(MOUNT_FWD / 2 != MOUNT_UP / 2, "MOUNT_FWD and MOUNT_UP must be different axes");

// Reads one sample (vehicle frame, uncorrected). Returns 0 = OK, else error code.
enum { E_OK = 0, E_BUS = 1, E_INVALID = 2, E_STUCK = 3 };
static uint8_t mpuSample(V3& a, V3& g, bool& sat) {
  uint8_t b[14];
  if (!mpuRead(0x3B, b, 14)) return E_BUS;
  bool zero = true, ff = true;
  uint16_t h = 0;
  for (uint8_t i = 0; i < 14; ++i) {
    if (b[i]) zero = false;
    if (b[i] != 0xFF) ff = false;
    if (i < 6 || i >= 8) h = (uint16_t)((h << 3) ^ (h >> 13) ^ b[i]);
  }
  if (zero || ff) return E_INVALID;                // sensor reset/asleep or bus stuck
  if (h == lastHash) { if (++sameCount >= IMU_STUCK_FRAMES) return E_STUCK; }
  else sameCount = 0;
  lastHash = h;
  int16_t ra[3], rg[3];
  for (uint8_t i = 0; i < 3; ++i) {
    ra[i] = (int16_t)((b[2 * i] << 8) | b[2 * i + 1]);
    rg[i] = (int16_t)((b[8 + 2 * i] << 8) | b[9 + 2 * i]);
  }
  sat = false;
  float av[3], gv[3];
  for (uint8_t r = 0; r < 3; ++r) {
    const int16_t va = ra[mapIdx[r]];
    if (va == 32767 || va == -32768) sat = true;
    av[r] = mapSign[r] * va / 2048.0f;
    gv[r] = mapSign[r] * rg[mapIdx[r]] / 16.4f;
  }
  a = v3(av[0], av[1], av[2]);
  g = v3(gv[0], gv[1], gv[2]);
  return E_OK;
}

// =============================================================================
// Calibration
// =============================================================================
// Riding orientation: the gravity direction measured while the bike stands upright
// defines 0 deg roll / 0 deg pitch, so the sensor never has to be mounted level.
static void setLevel(const V3& up, bool on) {
  levelSet = on;
  levelRoll = on ? atan2(up.y, up.z) * R2D : 0;
  levelPitch = on ? atan2(up.x, sqrt(up.y * up.y + up.z * up.z)) * R2D : 0;
}
static void calStart() { calN = 0; calMean = calM2 = calAcc = v3(0, 0, 0); }
// calMean = sum of gyro, calM2 = sum of gyro^2, calAcc = sum of accel.
// Returns 0 running, 1 done OK, 2 moved / invalid.
static uint8_t calAdd(const V3& a, const V3& g, uint16_t target, bool level) {
  float* sg = &calMean.x; float* s2 = &calM2.x; float* sa = &calAcc.x;
  const float* ga = &g.x; const float* aa = &a.x;
  for (uint8_t i = 0; i < 3; ++i) { sg[i] += ga[i]; s2[i] += ga[i] * ga[i]; sa[i] += aa[i]; }
  ++calN;
  if (vlen(g) > (level ? 10.0f : 50.0f)) return 2;            // obvious movement
  if (calN < target) return 0;
  const float gm = vlen(calAcc) / calN;
  if (fabs(gm - 1) > (level ? 3 : 1) * CAL_G_TOL) return 2;
  float sd = 0;
  for (uint8_t i = 0; i < 3; ++i) { sg[i] /= calN; sd = fmax(sd, s2[i] / calN - sg[i] * sg[i]); }
  if (level) return 1;
  return (sqrt(sd) <= CAL_GYRO_STD_MAX && vlen(calMean) <= CAL_GYRO_BIAS_MAX) ? 1 : 2;
}
static void startCalibration() {
  calibrating = true;
  calAttempts = 0;
  calQuality = 0;
  calStart();
  logTag(F("CALIBRATION")); tx.println(F("Keep still (3 s)"));
}

// =============================================================================
// Sensor fusion: complementary filter
//   angle = alpha * (angle + rate * dt) + (1 - alpha) * accelerometer_angle
// The accelerometer is trusted only when |a| ~ 1 g and rotation is low; during
// impacts, braking and hard cornering the angle comes from the gyro alone.
// Pitch uses q*cos(roll) - r*sin(roll) so yaw while leaned does not leak into it.
// =============================================================================
static V3 gravityBody() {
  const float st = sin(fTheta), ct = cos(fTheta);
  return v3(-st, sin(fPhi) * ct, cos(fPhi) * ct);
}
static void fusionUpdate(const V3& f, const V3& w, float dt) {
  float* lp = &accLp.x;
  const float* ff = &f.x;
  if (!fInit) {
    accLp = f;
    fPhi = fTheta = 0;
    rollRate = pitchRate = 0;
  } else {
    for (uint8_t i = 0; i < 3; ++i) lp[i] += FUSION_ACC_LPF * (ff[i] - lp[i]);
    dt = clampf(dt, 0, 0.05f);
    rollRate = w.x;
    pitchRate = sin(fPhi) * w.z - cos(fPhi) * w.y;          // nose-up positive
    fPhi += rollRate * D2R * dt;
    fTheta -= pitchRate * D2R * dt;
  }
  const float acc = vlen(accLp);
  attRef = 3;
  if (!fInit || (fabs(acc - 1) <= FUSION_TRUST_G && vlen(w) <= FUSION_TRUST_DPS)) {
    const float k = fInit ? 1 - FUSION_ALPHA : 1;
    fPhi += k * wrapPi(atan2(accLp.y, accLp.z) - fPhi);
    fTheta += k * (atan2(-accLp.x, sqrt(accLp.y * accLp.y + accLp.z * accLp.z)) - fTheta);
    attRef = 1;
  }
  fInit = true;
  if (fTheta > HALF_PI) { fTheta = PI - fTheta; fPhi += PI; }
  else if (fTheta < -HALF_PI) { fTheta = -PI - fTheta; fPhi += PI; }
  fPhi = wrapPi(fPhi);
  roll = fPhi * R2D - levelRoll;
  pitch = -fTheta * R2D - levelPitch;
  tilt = acos(clampf(cos(roll * D2R) * cos(pitch * D2R), -1, 1)) * R2D;
}

// =============================================================================
// Crash detector: 5 indicators scored against the vehicle profile, temporal
// logic (IDLE -> ACTIVE -> SETTLE -> verdict). No single indicator can produce
// a crash verdict on its own. Mirrors firmware/BikeSafetyBelt/src/detection.
// =============================================================================
static void detReset() {
  phase = PH_IDLE;
  memset(&rep, 0, sizeof(rep));
  anomShift = pendTrig = 0;
  preA = preG = 0;
  preVert = false;
  possibleRaised = false;
  tiltTiming = tiltAlarm = false;
}
static void detDisarm() { detReset(); armed = false; uprightTiming = false; }
static void detBegin() { detReset(); armed = true; histCount = histHead = 0; liveConf = 0; stillNow = longStill = false; }

static bool vertical(const V3& a) {
  const V3 d = vsub(a, gravityBody());
  const float n = vlen(d);
  return n >= 0.3f && fabs(d.z) / n >= DET_VERT_FRAC;
}
static void detScores(bool final) {
  float* s = rep.s;
  s[0] = rampf(rep.pkT, prof.leanN, prof.leanC);
  s[1] = rampf(rep.pkG, prof.gyrN, prof.gyrC);
  s[2] = rampf(rep.pkA, prof.accN, prof.accC);
  if (rep.vert && rep.pkT < prof.leanN) s[2] *= DET_VERT_WEIGHT;
  s[3] = fmax(rampf(rep.dT, prof.tcN, prof.tcC), rampf(rep.sus, DET_SUS_N_MS, DET_SUS_C_MS));
  rep.post = fmax(s[1], fmax(s[2], s[3])) >= DET_MAJOR || (rep.trig & TRIG_STATIC_TILT);
  s[4] = 0;
  if (rep.post && (final || phase == PH_SETTLE || (rep.trig & TRIG_STATIC_TILT))) {
    s[4] = rampf(final ? rep.rest : tilt, prof.restN, prof.restC);
    if (!(final && rep.settled)) s[4] *= 0.5f;
  }
  float c = DET_W_LEAN * s[0] + DET_W_RATE * s[1] + DET_W_IMPACT * s[2] + DET_W_CHANGE * s[3] + DET_W_POST * s[4];
  if (rep.parked) c *= DET_PARKED_FACTOR;
  rep.conf = clampf(c, 0, 1);
  rep.act = 0;
  for (uint8_t i = 0; i < 5; ++i) if (s[i] >= DET_ACTIVE_LVL) rep.act++;
}
static void detFinish(uint32_t t) {
  detScores(true);
  const bool crash = !(rep.trig & TRIG_STATIC_TILT) && rep.conf >= DET_CRASH && rep.act >= DET_MIN_IND;
  rep.verdict = crash ? V_CRASH : V_DISMISSED;
  phase = PH_IDLE;
  anomShift = pendTrig = 0;
  liveConf = rep.conf;
  tiltAlarm = false;
  if (crash) { armed = false; uprightTiming = false; }
  (void)t;
}
static void detStart(uint32_t t, uint8_t trig) {
  const bool keepPossible = tiltAlarm;                   // static-tilt alarm already raised
  memset(&rep, 0, sizeof(rep));
  rep.trig = trig;
  rep.base = histCount ? tiltHist[histCount < 10 ? 0 : histHead] : 0;
  rep.parked = longStill && !since(t, longStillMs, 2000);
  rep.pkA = preA; rep.pkG = preG; rep.vert = preVert;
  phase = PH_ACTIVE;
  phaseMs = t;
  calm = resting = false;
  possibleRaised = keepPossible;
  tiltAlarm = false;
}
static void detTrack(const V3& a, bool sat, float dt) {
  if (aMag > rep.pkA) { rep.pkA = aMag; rep.vert = vertical(a); }
  if (wMag > rep.pkG) rep.pkG = wMag;
  if (tilt > rep.pkT) { rep.pkT = tilt; rep.pkR = roll; }
  if (tiltChg > rep.dT) rep.dT = tiltChg;
  if (wMag >= prof.gyrN) rep.sus += dt * 1000.0f;
  if (sat) rep.trig |= TRIG_SAT;
}
static uint8_t detUpdate(const V3& a, bool sat, float dt, uint32_t t) {
  uint8_t ev = 0;
  const float tiltRate = sqrt(rollRate * rollRate + pitchRate * pitchRate);
  // Stillness (parked guard)
  lpG += 0.05f * (wMag - lpG);
  lpA += 0.05f * (fabs(aMag - 1) - lpA);
  const bool still = lpG <= DET_STILL_DPS && lpA <= DET_STILL_G;
  if (still && !stillNow) { stillNow = true; stillStart = t; }
  if (!still) stillNow = false;
  if (stillNow && since(t, stillStart, DET_PARKED_MS)) { longStill = true; longStillMs = t; }
  // Tilt history, 10 Hz x 1 s
  if (histCount == 0 || since(t, lastHistMs, 100)) {
    tiltHist[histHead] = tilt;
    histHead = (uint8_t)((histHead + 1) % 10);
    if (histCount < 10) histCount++;
    lastHistMs = t;
  }
  tiltChg = 0;
  for (uint8_t k = 0; k < 5 && k < histCount; ++k)
    tiltChg = fmax(tiltChg, fabs(tilt - tiltHist[(histHead + 9 - k) % 10]));
  if (tilt >= prof.leanN) staticAbn = true;
  else if (tilt < prof.leanN - DET_HYST_DEG) staticAbn = false;
  // Re-arm after an incident once upright for 3 s
  if (!armed) {
    if (tilt < prof.leanN - DET_HYST_DEG && wMag < DET_QUIET_DPS) {
      if (!uprightTiming) { uprightTiming = true; uprightSince = t; }
      else if (since(t, uprightSince, DET_REARM_MS)) armed = true;
    } else uprightTiming = false;
  }
  // Anomalous-sample trigger (K of last M)
  uint8_t bits = 0;
  if (aMag >= DET_TRIG_ACC_G) bits |= TRIG_ACC;
  if (wMag >= DET_TRIG_GYRO_DPS) bits |= TRIG_GYRO;
  if (tiltRate >= DET_TRIG_TILT_DPS) bits |= TRIG_TILTRATE;
  if (sat) bits |= TRIG_SAT;
  const uint8_t mask = (1 << DET_TRIG_M) - 1;
  anomShift = (uint8_t)((anomShift << 1) | (bits ? 1 : 0));
  if (!(anomShift & mask)) { pendTrig = 0; preA = preG = 0; preVert = false; }
  else {
    pendTrig |= bits;
    if (aMag > preA) { preA = aMag; preVert = vertical(a); }
    preG = fmax(preG, wMag);
  }
  uint8_t cnt = 0;
  for (uint8_t m = anomShift & mask; m; m >>= 1) cnt += m & 1;
  const bool triggered = cnt >= DET_TRIG_K;

  if (phase == PH_IDLE) {
    if (armed && triggered) {
      detStart(t, pendTrig);
      ev |= DEV_CAND;
      detTrack(a, sat, dt);
    } else if (armed) {
      // "Too much tilt": abnormal static tilt held for TILT_ALARM_MS -> POSSIBLE_CRASH
      if (tilt >= prof.leanC) {
        if (!tiltTiming) { tiltTiming = true; tiltSince = t; }
        else if (!tiltAlarm && since(t, tiltSince, TILT_ALARM_MS)) {
          memset(&rep, 0, sizeof(rep));
          rep.trig = TRIG_STATIC_TILT;
          rep.pkT = rep.rest = tilt; rep.pkR = roll;
          rep.parked = longStill && !since(t, longStillMs, 2000);
          rep.settled = true;
          tiltAlarm = true;
          possibleRaised = true;
          ev |= DEV_POSS;
        }
        if (tiltAlarm) { rep.pkT = fmax(rep.pkT, tilt); rep.rest = tilt; detScores(false); liveConf = rep.conf; }
      } else if (tilt < prof.leanC - DET_HYST_DEG) {
        tiltTiming = false;
        if (tiltAlarm) {                        // back upright: tilt alarm cleared
          rep.rest = tilt;
          detFinish(t);
          ev |= DEV_VERDICT;
          return ev;
        }
      }
    }
    if (!tiltAlarm) liveConf *= 0.98f;            // last event's confidence fades out
    return ev;
  }

  if (phase == PH_ACTIVE) {
    detTrack(a, sat, dt);
    const bool c = wMag < DET_QUIET_DPS && fabs(aMag - 1) < DET_QUIET_G;
    if (!c) calm = false;
    else if (!calm) { calm = true; calmSince = t; }
    if ((calm && since(t, calmSince, DET_QUIET_MS)) || since(t, phaseMs, DET_ACTIVE_MAX_MS)) {
      detScores(false);
      if (!rep.post) { rep.rest = tilt; detFinish(t); return ev | DEV_VERDICT; }
      phase = PH_SETTLE;
      phaseMs = t;
      resting = false;
    }
  } else {   // PH_SETTLE
    if (triggered && bits) {
      phase = PH_ACTIVE; phaseMs = t; calm = false;
      detTrack(a, sat, dt);
    } else {
      const bool r = lpG < DET_REST_DPS && lpA < DET_REST_G;
      if (!r) resting = false;
      else if (!resting) { resting = true; restSince = t; }
      if ((resting && since(t, restSince, DET_REST_MS)) || since(t, phaseMs, DET_POST_MAX_MS)) {
        rep.settled = resting;
        rep.rest = tilt;
        detFinish(t);
        return ev | DEV_VERDICT;
      }
    }
  }
  detScores(false);
  liveConf = rep.conf;
  if (!possibleRaised && rep.conf >= DET_POSSIBLE) { possibleRaised = true; ev |= DEV_POSS; }
  return ev;
}

// =============================================================================
// Messages to the app
// =============================================================================
static FS typeName(uint8_t t) { return t == INC_TEST ? F("TEST") : t == INC_SIM ? F("SIM") : F("AUTO"); }
static FS outName(uint8_t o) {
  switch (o) {
    case OUT_DISMISSED: return F("DISMISSED");
    case OUT_CANCELLED: return F("CANCELLED");
    case OUT_SOS: return F("SOS");
    case OUT_RESOLVED: return F("RESOLVED");
    case OUT_TEST_CLEARED: return F("TEST_CLEARED");
    default: return F("OPEN");
  }
}
static uint8_t countdownLeft(uint32_t now) {
  if (state != ST_CRASH) return 0;
  const uint32_t e = now - countdownStart, d = COUNTDOWN_S * 1000UL;
  return e >= d ? 0 : (uint8_t)((d - e + 999) / 1000);
}
static void sendHello() {
  jBegin(F("hello"));
  jS(F("fw"), F(FW_VERSION)); jI(F("proto"), 1);
  jKey(F("dev")); tx.print('"'); tx.print(devName); tx.print('"');
  jS(F("board"), F("UNO")); jI(F("boot"), cfg.boot);
  jI(F("gpsHw"), 0); jI(F("batHw"), 0); jI(F("cd"), COUNTDOWN_S); jI(F("ms"), millis());
  jEnd();
}
static void sendCfg() {
  jBegin(F("cfg"));
  jS(F("profile"), profileName(cfg.profile));
  jF(F("leanN"), prof.leanN, 0); jF(F("leanC"), prof.leanC, 0); jF(F("gyrN"), prof.gyrN, 0); jF(F("gyrC"), prof.gyrC, 0);
  jF(F("accN"), prof.accN, 1); jF(F("accC"), prof.accC, 1); jF(F("tcN"), prof.tcN, 0); jF(F("tcC"), prof.tcC, 0);
  jF(F("restN"), prof.restN, 0); jF(F("restC"), prof.restC, 0);
  jI(F("possible"), pct(DET_POSSIBLE)); jI(F("crash"), pct(DET_CRASH)); jI(F("minInd"), DET_MIN_IND);
  
  jI(F("level"), levelSet); jI(F("cal"), calQuality);
  jEnd();
}
static void sendState(uint8_t from, uint8_t to, FS why) {
  jBegin(F("state"));
  jS(F("st"), stateName(to)); jS(F("prev"), stateName(from)); jS(F("why"), why);
  jI(F("inc"), incOpen ? inc.id : 0); jS(F("type"), typeName(inc.type)); jI(F("cd"), countdownLeft(millis()));
  jI(F("ms"), millis());
  jEnd();
}
static void sendTel(uint32_t now) {
  const bool v = imuHealthy && fInit && !calibrating;
  jBegin(F("tel"));
  jI(F("ms"), now); jS(F("st"), stateName(state));
  jF(F("r"), v ? roll : NAN, 1); jF(F("p"), v ? pitch : NAN, 1); jF(F("tl"), v ? tilt : NAN, 1);
  jF(F("a"), v ? aMag : NAN, 2); jF(F("ap"), v ? peakA : NAN, 2); jF(F("g"), v ? wMag : NAN, 0); jF(F("gp"), v ? peakG : NAN, 0);
  jI(F("c"), pct(liveConf)); jI(F("ph"), phase); jI(F("imu"), imuHealthy); jI(F("cal"), calQuality);
  jI(F("lvl"), levelSet); jI(F("sim"), simId); jI(F("cd"), countdownLeft(now)); jI(F("ab"), staticAbn);
  jI(F("arm"), armed); jI(F("ref"), v ? attRef : 0); jI(F("bat"), -1); jI(F("gps"), 0);
  jEnd();
  peakA = peakG = 0;
}
static void sendDet(FS kind) {
  jBegin(F("det"));
  jS(F("ev"), kind); jI(F("inc"), (incOpen || (inc.flags & F_SUSPECTED)) ? inc.id : 0); jI(F("sim"), simId != 0);
  jI(F("conf"), pct(rep.conf));
  jKey(F("s")); tx.print('[');
  for (uint8_t i = 0; i < 5; ++i) { if (i) tx.print(','); tx.print(pct(rep.s[i])); }
  tx.print(']');
  jI(F("act"), rep.act); jF(F("pkA"), rep.pkA, 2); jF(F("pkG"), rep.pkG, 0); jF(F("pkT"), rep.pkT, 1);
  jF(F("dT"), rep.dT, 1); jF(F("sus"), rep.sus, 0); 
  jF(F("rest"), rep.rest, 1); jI(F("set"), rep.settled); jI(F("vert"), rep.vert); jI(F("park"), rep.parked);
  jI(F("post"), rep.post);
  jS(F("verdict"), rep.verdict == V_CRASH ? F("CRASH") : rep.verdict == V_DISMISSED ? F("DISMISSED") : F(""));
  jEnd();
}
static void sendIncident(const Inc& r) {
  jBegin(F("inc"));
  jI(F("id"), r.id); jI(F("boot"), r.boot); jI(F("up"), r.up); jI(F("cur"), r.boot == cfg.boot);
  jS(F("type"), typeName(r.type)); jS(F("out"), outName(r.out)); jI(F("flags"), r.flags); jI(F("conf"), r.conf);
  jF(F("acc"), r.acc / 100.0f, 2); jI(F("gyr"), r.gyr); jF(F("lean"), r.lean / 10.0f, 1); jI(F("gpsOk"), 0);
  jEnd();
}
static void sendSos(uint32_t now) {
  jBegin(F("sos"));
  jI(F("id"), inc.id); jKey(F("dev")); tx.print('"'); tx.print(devName); tx.print('"');
  jI(F("sim"), inc.type == INC_SIM); jI(F("conf"), inc.conf); jI(F("boot"), inc.boot); jI(F("up"), inc.up);
  jF(F("acc"), inc.acc / 100.0f, 2); jI(F("gyr"), inc.gyr); jF(F("lean"), inc.lean / 10.0f, 1);
  jI(F("tx"), sosTx); jI(F("ms"), now); jI(F("resumed"), (inc.flags & F_RESUMED) != 0); jI(F("gps"), 0);
  jEnd();
}
static void ack(const char* cmd, bool ok) {
  jBegin(F("ack"));
  jKey(F("cmd")); tx.print('"');
  for (uint8_t i = 0; cmd[i] && cmd[i] != ' ' && i < 12; ++i)
    if (isalnum(cmd[i])) tx.print(cmd[i]);
  tx.print('"');
  jI(F("ok"), ok);
  jEnd();
}

// =============================================================================
// State machine
// =============================================================================
static uint8_t idleState() { return !imuHealthy ? ST_IMU_FAULT : (calibrating && !simId) ? ST_CALIBRATING : ST_SAFE; }
static bool alarmState(uint8_t s) { return s == ST_POSSIBLE || s == ST_CRASH || s == ST_SOS || s == ST_TEST; }

static void setState(uint8_t to, FS why) {
  if (to == state) return;
  const uint8_t from = state;
  state = to;
  stateSince = millis();
  logTag(F("STATE")); tx.print(stateName(to)); tx.print(F(" (")); tx.print(why); tx.println(')');
  if (to == ST_CRASH) {
    countdownStart = stateSince;
    if (inc.type == INC_AUTO) setActive(1, inc.id);
    logTag(F("EMERGENCY")); tx.println(F("POSSIBLE ACCIDENT - press button to cancel"));
  } else if (to == ST_SOS) {
    if (inc.type == INC_AUTO) setActive(2, inc.id);
  } else if (to == ST_CANCELLED || ((from == ST_SOS || from == ST_RECOVERY) && to != ST_RECOVERY)) {
    setActive(0, 0);
  }
  sendState(from, to, why);
}
static void publishInc() { storeIncident(inc); sendIncident(inc); }
static void openInc(uint8_t type) {
  memset(&inc, 0, sizeof(inc));
  seq = (uint16_t)(seq >= 999 ? 1 : seq + 1);
  inc.id = (uint32_t)cfg.boot * 1000UL + seq;
  inc.up = millis();
  inc.boot = cfg.boot;
  inc.type = type;
  incOpen = true;
}
static void closeInc(uint8_t out) { inc.out = out; publishInc(); incOpen = false; }
static void fillInc() {
  const uint8_t c = pct(rep.conf);
  if (c > inc.conf || rep.verdict) inc.conf = c;
  const int16_t a = (int16_t)clampf(rep.pkA * 100, 0, 32767), g = (int16_t)clampf(rep.pkG, 0, 32767),
                l = (int16_t)clampf(rep.pkT * 10, 0, 32767);
  if (a > inc.acc) inc.acc = a;
  if (g > inc.gyr) inc.gyr = g;
  if (l > inc.lean) inc.lean = l;
}
static void enterCrash(FS why) {
  inc.flags |= F_SUSPECTED | F_CONFIRMED;
  publishInc();
  setState(ST_CRASH, why);
}
static void onDetector(uint8_t ev) {
  const uint8_t type = simId ? INC_SIM : INC_AUTO;
  const bool normal = state == ST_SAFE || state == ST_MONITORING;
  if ((ev & DEV_CAND) && state == ST_SAFE) setState(ST_MONITORING, F("crash detector"));
  if ((ev & DEV_POSS) && (normal || state == ST_TEST)) {
    if (state == ST_TEST) closeInc(OUT_TEST_CLEARED);
    openInc(type);
    inc.flags |= F_SUSPECTED;
    fillInc();
    publishInc();
    setState(ST_POSSIBLE, (rep.trig & TRIG_STATIC_TILT) ? F("too much tilt") : F("crash detector"));
  }
  if (ev & DEV_VERDICT) {
    if (rep.verdict == V_CRASH && (normal || state == ST_POSSIBLE || state == ST_TEST)) {
      if (state == ST_TEST) closeInc(OUT_TEST_CLEARED);
      if (!incOpen || state == ST_TEST) openInc(type);
      fillInc();
      enterCrash(F("crash detector"));
    } else if (rep.verdict == V_DISMISSED) {
      if (state == ST_POSSIBLE) { fillInc(); closeInc(OUT_DISMISSED); setState(idleState(), F("motion evaluated as normal")); }
      else if (state == ST_MONITORING) setState(idleState(), F("motion evaluated as normal"));
    }
  }
}
// "I'M OKAY": button press during an alarm, or CANCEL from the app.
static bool cancelAlarm(FS src) {
  const uint32_t now = millis();
  if (state == ST_POSSIBLE || state == ST_CRASH || state == ST_SOS || state == ST_RECOVERY) {
    const bool afterSos = state == ST_SOS || state == ST_RECOVERY;
    inc.flags |= F_CANCELLED;
    if (afterSos) { inc.out = OUT_RESOLVED; publishInc(); incOpen = false; sosPending = false; }
    else closeInc(OUT_CANCELLED);
    lockoutUntil = now + BTN_LOCKOUT_MS;
    detDisarm();                                   // re-arms once the bike is upright again
    setState(ST_CANCELLED, src);
    logTag(F("EMERGENCY"));
    tx.println(afterSos ? F("Rider OK after SOS")
                        : F("Cancelled by rider"));
    return true;
  }
  if (state == ST_TEST) {
    closeInc(OUT_TEST_CLEARED);
    lockoutUntil = now + BTN_LOCKOUT_MS / 3;
    setState(idleState(), src);
    return true;
  }
  return false;
}
static bool resetAfterSos() {
  if (state != ST_SOS && state != ST_RECOVERY) return false;
  inc.out = OUT_RESOLVED;
  publishInc();
  incOpen = false;
  sosPending = false;
  detDisarm();
  setState(idleState(), F("explicit reset"));
  return true;
}
static void buttonPress(FS src) {
  if ((int32_t)(millis() - lockoutUntil) < 0) return;
  if (alarmState(state) || state == ST_RECOVERY) { cancelAlarm(src); return; }
  if (state == ST_SAFE || state == ST_MONITORING || state == ST_CALIBRATING || state == ST_IMU_FAULT) {
    openInc(INC_TEST);
    publishInc();
    setState(ST_TEST, src);
    logTag(F("TEST")); tx.println(F("TEST / MANUAL TRIGGER"));
  }
}
static void setImuHealthy(bool ok) {
  if (ok == imuHealthy) return;
  imuHealthy = ok;
  if (!ok) {
    if (state == ST_POSSIBLE) { inc.flags |= F_IMU_LOST; enterCrash(F("IMU lost during possible crash")); }
    else if (state == ST_SAFE || state == ST_MONITORING || state == ST_CALIBRATING || state == ST_BOOT)
      setState(ST_IMU_FAULT, F("IMU not responding"));
    else if (incOpen) { inc.flags |= F_IMU_LOST; publishInc(); }
  } else if (state == ST_IMU_FAULT) {
    setState(idleState(), F("IMU reconnected"));
  }
}

// =============================================================================
// Sensor processing (100 Hz)
// =============================================================================
static void imuFault(bool simulated) {  // simulated is always false on the UNO
  fInit = false;
  detReset();
  logTag(F("IMU")); tx.print(F("DISCONNECTED ("));
  tx.print(lastErr == E_BUS ? F("I2C error") : lastErr == E_INVALID ? F("invalid data") : lastErr == E_STUCK ? F("frozen data") : F("not found"));
  tx.println(simulated ? F(", sim)") : F(")"));
  if (!simulated) imuReady = false;
  setImuHealthy(false);
}
static void processSample(uint32_t now, float dt) {
  V3 a, g;
  bool sat = false, ok;
  if (imuReady) {
    lastErr = mpuSample(a, g, sat);
    ok = lastErr == E_OK;
  } else return;

  if (!ok) {
    if (failCount < 255) failCount++;
    if (failCount >= IMU_FAIL_LIMIT && imuHealthy) imuFault(false);
    return;
  }
  failCount = 0;
  if (!imuHealthy) {
    fInit = false;
    detReset();
    logTag(F("IMU")); tx.println(F("Connected"));
    if (calibrating) calStart();
    setImuHealthy(true);
  }

  {
    if (calibrating) {
      const uint8_t r = calAdd(a, g, CAL_SAMPLES, false);
      if (r == 0) return;
      if (r == 1) {
        gBias = calMean;
        const float n = vlen(calAcc) / calN;
        aOff = v3(calAcc.x / calN * (1 - 1 / n), calAcc.y / calN * (1 - 1 / n), calAcc.z / calN * (1 - 1 / n));
        calQuality = 1;
        if (!(cfg.flags & 2) || vlen(vsub(gBias, cfg.gbias)) > 0.3f) { cfg.gbias = gBias; cfg.flags |= 2; saveSettings(); }
        logTag(F("CALIBRATION")); tx.print(F("Done, gyro bias (")); tx.print(gBias.x, 2); tx.print(F(", "));
        tx.print(gBias.y, 2); tx.print(F(", ")); tx.print(gBias.z, 2); tx.print(F(") deg/s, gravity ")); tx.print(n, 3); tx.println(F(" g"));
      } else if (++calAttempts < CAL_ATTEMPTS) {
        logTag(F("CALIBRATION")); tx.print(F("Moved - retry (")); tx.print(calAttempts + 1); tx.println(F("/5)"));
        calStart();
        return;
      } else if (cfg.flags & 2) {
        gBias = cfg.gbias; calQuality = 2;
        logTag(F("CALIBRATION")); tx.println(F("Using saved calibration"));
      } else {
        calQuality = 3;
        logTag(F("CALIBRATION")); tx.println(F("FAILED - uncalibrated"));
      }
      calibrating = false;
      fInit = false;
      detReset();
      if (state == ST_CALIBRATING) setState(idleState(), F("calibration complete"));
      sendCfg();
      return;
    }
    a = vsub(a, aOff);
    g = vsub(g, gBias);
    if (levelCal) {
      const uint8_t r = calAdd(a, g, CAL_LEVEL_SAMPLES, true);
      if (r == 0) return;
      levelCal = false;
      const float n = vlen(calAcc);
      const V3 up = v3(calAcc.x / n, calAcc.y / n, calAcc.z / n);
      const float tiltDeg = acos(clampf(up.z, -1, 1)) * R2D;
      logTag(F("CALIBRATION"));
      if (r == 2) tx.println(F("Aborted - sensor moved"));
      else if (tiltDeg > CAL_LEVEL_MAX_DEG) { tx.print(F("Rejected - sensor is ")); tx.print(tiltDeg, 0); tx.println(F(" deg off: check MOUNT_*")); }
      else {
        cfg.up = up; cfg.flags |= 1; saveSettings(); setLevel(up, true); fInit = false; detReset();
        tx.print(F("Level saved, tilt ")); tx.print(tiltDeg, 1); tx.println(F(" deg"));
        sendCfg();
      }
      return;
    }
  }

  fusionUpdate(a, g, dt);
  aMag = vlen(a);
  wMag = vlen(g);
  if (aMag > peakA) peakA = aMag;
  if (wMag > peakG) peakG = wMag;
  const uint8_t ev = detUpdate(a, sat, dt, now);
  if (!ev) return;
  if (ev & DEV_CAND) { logTag(F("DETECTOR")); tx.println(F("Abnormal motion")); }
  if (ev & DEV_POSS) {
    logTag(F("DETECTOR"));
    if (rep.trig & TRIG_STATIC_TILT) { tx.print(F("Too much tilt: ")); tx.print(tilt, 0); tx.println(F(" deg")); }
    else { tx.print(F("Possible crash - confidence ")); tx.print(pct(rep.conf)); tx.println('%'); }
  }
  if (ev & DEV_VERDICT) {
    logTag(F("DETECTOR"));
    tx.print(rep.verdict == V_CRASH ? F("CRASH LIKELY") : F("Not a crash")); tx.print(F(" - confidence "));
    tx.print(pct(rep.conf)); tx.print(F("%, ")); tx.print(rep.act); tx.print(F("/5 indicators | lean "));
    tx.print(rep.pkT, 0); tx.print(F(" deg, rotation ")); tx.print(rep.pkG, 0); tx.print(F(" deg/s, impact "));
    tx.print(rep.pkA, 1); tx.println(F(" g"));
  }
  onDetector(ev);
  sendDet((ev & DEV_VERDICT) ? F("verdict") : (ev & DEV_POSS) ? F("possible") : F("candidate"));
}

// =============================================================================
// Commands
// =============================================================================
static bool cmdIs(const char*& p, FS word) {
  const char* w = reinterpret_cast<const char*>(word);
  uint8_t i = 0;
  for (;; ++i) {
    const char wc = pgm_read_byte(w + i);
    const char c = toupper(p[i]);
    if (!wc) {
      if (c == 0 || c == ' ') { p += i; while (*p == ' ') ++p; return true; }
      return false;
    }
    if (c != wc) return false;
  }
}
static bool idleForCommands() { return state == ST_SAFE || state == ST_MONITORING || state == ST_IMU_FAULT; }
static void restartDetection() {         // keeps "armed": re-arming still needs the bike upright
  fInit = false;
  detReset();
  histCount = histHead = 0;
  if (state == ST_MONITORING) setState(idleState(), F("motion evaluated as normal"));
  else if (state == ST_POSSIBLE) { closeInc(OUT_DISMISSED); setState(idleState(), F("motion evaluated as normal")); }
}
static void printHelp() {
  tx.println(F("[HELP] CANCEL RESET STATUS LOG CLEARLOG BTN PROFILE <name>"));
  tx.println(F("[HELP] CAL LEVEL|GYRO|CLEAR COMMS <s> GPSLOSS <s> SOSACK <id>"));
}
static void printStatus() {
  logTag(F("STATUS")); tx.print(stateName(state)); tx.print(F(" | IMU ")); tx.print(imuHealthy ? F("OK") : F("DISCONNECTED"));
  tx.print(F(" | roll ")); tx.print(roll, 1); tx.print(F(" pitch ")); tx.print(pitch, 1); tx.print(F(" | ")); tx.print(aMag, 2);
  tx.print(F(" g ")); tx.print(wMag, 0); tx.print(F(" deg/s | conf ")); tx.print(pct(liveConf)); tx.print(F("% | ")); tx.print(profileName(cfg.profile));
  tx.print(F(" | cal ")); tx.print(calQuality); tx.print(F(" | level ")); tx.print(levelSet); tx.print(F(" | incidents ")); tx.println(cfg.ringCount);
}
static void handleCommand(const char* line, uint32_t now) {
  const char* p = line;
  bool ok = true;
  if (cmdIs(p, F("PING"))) {
    // The app's HELLO is usually swallowed by the bootloader (the CH340 resets the
    // UNO when the port opens). Answer the first PING as if it were the HELLO.
    if (!greeted) { greeted = true; sendHello(); sendCfg(); sendState(state, state, F("boot")); if (sosPending) sendSos(now); }
    return;
  }
  if (cmdIs(p, F("HELLO"))) {
    greeted = true;
    logTag(F("LINK")); tx.println(F("App connected"));
    sendHello(); sendCfg(); sendState(state, state, F("boot"));
    if (sosPending) sendSos(now);
    return;
  }
  if (cmdIs(p, F("CANCEL"))) ok = cancelAlarm(F("app"));
  else if (cmdIs(p, F("RESET"))) ok = resetAfterSos();
  else if (cmdIs(p, F("SOSACK"))) {
    const uint32_t id = parseU(p);
    ok = sosPending && id == inc.id;
    if (ok) {
      sosPending = false;
      inc.flags |= F_SOS_ACK;
      publishInc();
      logTag(F("SOS")); tx.print(F("App got SOS #")); tx.println(id);
    }
  } else if (cmdIs(p, F("BTN"))) {
    logTag(F("BUTTON")); tx.println(F("Pressed (app)"));
    buttonPress(F("button"));
  } else if (cmdIs(p, F("PROFILE"))) {
    uint8_t id = 3;
    for (uint8_t i = 0; i < 3; ++i) if (strcasecmp_P(p, reinterpret_cast<const char*>(profileName(i))) == 0) id = i;
    ok = id < 3 && idleForCommands() && !simId;
    if (ok) {
      cfg.profile = id; saveSettings();
      memcpy_P(&prof, &PROFILES[id], sizeof(prof));
      restartDetection();
      logTag(F("CONFIG")); tx.print(F("Profile ")); tx.println(profileName(id));
      sendCfg();
    }
  } else if (cmdIs(p, F("CAL"))) {
    ok = state == ST_SAFE && !simId && !calibrating && !levelCal && imuHealthy;
    if (ok && cmdIs(p, F("LEVEL"))) {
      calStart(); levelCal = true;
      logTag(F("CALIBRATION")); tx.println(F("Hold upright and still"));
    } else if (ok && cmdIs(p, F("GYRO"))) {
      startCalibration();
      setState(ST_CALIBRATING, F("calibration requested"));
    } else if (ok && cmdIs(p, F("CLEAR"))) {
      setLevel(cfg.up, false); cfg.flags &= (uint8_t)~1; saveSettings(); restartDetection();
      logTag(F("CALIBRATION")); tx.println(F("Level cleared"));
      sendCfg();
    } else {
      ok = false;
      logTag(F("CALIBRATION")); tx.println(F("Not now"));
    }
  } else if (cmdIs(p, F("SIM"))) {
    ok = false;
    logTag(F("TEST")); tx.println(F("SIM not available on UNO (flash full). Test: flip the sensor fast onto its side"));
  } else if (cmdIs(p, F("COMMS"))) {
    long s = constrain((long)parseU(p), 1L, 60L);
    logTag(F("TEST")); tx.print(F("Output paused ")); tx.print(s); tx.println(F(" s"));
    ack(line, true);
    mutedUntil = now + (uint32_t)s * 1000UL;
    tx.muted = true;
    return;
  } else if (cmdIs(p, F("GPSLOSS"))) {
    logTag(F("GPS")); tx.println(F("No GPS module"));
  } else if (cmdIs(p, F("LOG"))) {
    logTag(F("LOG")); tx.print(cfg.ringCount); tx.println(F(" incidents"));
    Inc r;
    for (uint8_t i = 0; i < cfg.ringCount; ++i) if (readIncident(i, r)) sendIncident(r);
  } else if (cmdIs(p, F("CLEARLOG"))) {
    cfg.ringCount = cfg.ringHead = 0; lastSlot = 0xFF; saveSettings();
    logTag(F("LOG")); tx.println(F("Incident log cleared"));
  } else if (cmdIs(p, F("STATUS"))) printStatus();
  else if (cmdIs(p, F("HELP"))) printHelp();
  else {
    ok = false;
    logTag(F("LINK")); tx.print(F("Unknown: ")); tx.print(line); tx.println(F(""));
  }
  ack(line, ok);
}
static void serviceSerial(uint32_t now) {
  while (Serial.available() > 0) {
    const char c = (char)Serial.read();
    if (c == '\n' || c == '\r') {
      if (rxOverflow) { rxOverflow = false; rxLen = 0; continue; }
      if (!rxLen) continue;
      rxLine[rxLen] = '\0';
      rxLen = 0;
      handleCommand(rxLine, now);
    } else if (rxLen < sizeof(rxLine) - 1) rxLine[rxLen++] = c;
    else rxOverflow = true;
  }
}

// =============================================================================
// LEDs, buzzer, button
// =============================================================================
// Per state: blink period (ms, 0 = steady), which LED, and whether the buzzer
// beeps in sync with the red LED.
//   normal states -> green blinks | alarms -> red blinks + beeps | IMU fault -> red solid
static const uint8_t OUT_RED = 1, OUT_GREEN = 2, OUT_BEEP = 4, OUT_FLASH = 8;
const uint8_t OUT_MODE[] PROGMEM = {
  OUT_GREEN, OUT_GREEN, OUT_GREEN, OUT_GREEN,   // BOOT, CALIBRATING, SAFE, MONITORING
  OUT_RED | OUT_BEEP, OUT_RED | OUT_BEEP,       // POSSIBLE_CRASH, CRASH_DETECTED
  OUT_GREEN, OUT_RED | OUT_BEEP,                // USER_CANCELLED (buzzer silenced), SOS_SENT
  OUT_RED | OUT_FLASH, OUT_RED | OUT_BEEP,      // RECOVERY (quiet flash), MANUAL_TEST
  OUT_RED};                                     // IMU_FAULT: red solid
const uint16_t OUT_PERIOD[] PROGMEM = {250, 250, 1000, 1000, 500, 250, 1000, 200, 2000, 500, 0};
static void serviceOutputs(uint32_t now) {
  const uint8_t m = pgm_read_byte(&OUT_MODE[state]);
  const uint16_t per = pgm_read_word(&OUT_PERIOD[state]);
  const uint16_t ph = per ? (uint16_t)((now - stateSince) % per) : 0;
  const bool on = per == 0 || ph < ((m & OUT_FLASH) ? 100 : per / 2);
  const bool beep = (m & OUT_BEEP) && on;
  digitalWrite(PIN_RED, ((m & OUT_RED) && on) ? HIGH : LOW);
  digitalWrite(PIN_GREEN, ((m & OUT_GREEN) && on) ? HIGH : LOW);
  if (beep != buzzerOn) {
    buzzerOn = beep;
    if (beep) tone(PIN_BUZZER, BUZZER_HZ);
    else noTone(PIN_BUZZER);
  }
}
static void serviceButton(uint32_t now) {
  const bool raw = digitalRead(PIN_BUTTON) == LOW;
  if (raw != btnLastRaw) { btnLastRaw = raw; btnChange = now; }
  if (raw != btnStable && since(now, btnChange, BTN_DEBOUNCE_MS)) {
    btnStable = raw;
    if (btnStable) {                                  // press edge only; holding never repeats
      logTag(F("BUTTON")); tx.println(F("Pressed"));
      buttonPress(F("button"));
    }
  }
}
static void serviceTimers(uint32_t now) {
  if (state == ST_CRASH) {
    if (!countdownLeft(now)) {                                      // countdown expired -> SOS (once per incident)
      inc.flags |= F_SOS;
      inc.out = OUT_SOS;
      publishInc();
      sosPending = true;
      sosTx = 0;
      sosLastTx = now - SOS_RETRY_MS;
      logTag(F("SOS")); tx.print(F("EMERGENCY ALERT #")); tx.println(inc.id);
      tx.println(F("[SOS] LOCATION UNAVAILABLE (no GPS). Sent via app"));
      if (inc.type == INC_SIM) tx.println(F("[SOS] SIMULATION"));
      setState(ST_SOS, F("countdown expired"));
    }
  } else if (state == ST_CANCELLED && since(now, stateSince, CANCEL_SHOW_MS)) {
    setState(idleState(), F("timeout"));
  } else if (state == ST_SOS && since(now, stateSince, SOS_ALARM_MS)) {
    setState(ST_RECOVERY, F("timeout"));
    logTag(F("RECOVERY")); tx.println(F("Press button or RESET"));
  }
  if (sosPending && since(now, sosLastTx, SOS_RETRY_MS)) { sosLastTx = now; sosTx++; sendSos(now); }
  if (!imuReady && since(now, lastImuRetry, IMU_RETRY_MS)) {
    lastImuRetry = now;
    if (mpuBegin()) { imuReady = true; failCount = 0; logTag(F("IMU")); tx.print(F("Detected (WHO_AM_I 0x")); tx.print(whoAmI, HEX); tx.println(')'); }
  }
  if (tx.muted && (int32_t)(now - mutedUntil) >= 0) tx.muted = false;
}

// =============================================================================
// Setup / loop
// =============================================================================
void setup() {
  pinMode(PIN_RED, OUTPUT);
  pinMode(PIN_GREEN, OUTPUT);
  pinMode(PIN_BUZZER, OUTPUT);
  pinMode(PIN_BUTTON, INPUT_PULLUP);
  Serial.begin(SERIAL_BAUD);
  Wire.begin();
  Wire.setClock(400000UL);
#if defined(WIRE_HAS_TIMEOUT)
  Wire.setWireTimeout(3000, true);                    // a hung I2C bus must never freeze the loop
#endif

  uint16_t seed = (uint16_t)micros();
  for (uint8_t i = 0; i < 16; ++i) seed = (uint16_t)((seed << 1) ^ (analogRead(A0) & 1) ^ (analogRead(A1) & 1));
  loadSettings(seed);
  cfg.boot++;
  saveSettings();
  static const char hex[] PROGMEM = "0123456789ABCDEF";
  for (uint8_t i = 0; i < 4; ++i) devName[4 + i] = pgm_read_byte(&hex[(cfg.dev >> (12 - 4 * i)) & 0xF]);
  memcpy_P(&prof, &PROFILES[cfg.profile], sizeof(prof));
  setupMount();
  setLevel(cfg.up, cfg.flags & 1);
  btnStable = btnLastRaw = digitalRead(PIN_BUTTON) == LOW;   // a button held at power-up is ignored

  logTag(F("BOOT")); tx.print(F("Bike Safety Belt " FW_VERSION " UNO ")); tx.print(devName);
  tx.print(F(" boot #")); tx.println(cfg.boot);
  logTag(F("CONFIG")); tx.print(F("Profile ")); tx.print(profileName(cfg.profile));
  tx.println(F(" | no GPS/GSM/BLE"));

  imuReady = mpuBegin();
  imuHealthy = imuReady;
  lastImuRetry = millis();
  logTag(F("IMU"));
  if (imuReady) { tx.print(F("Connected (WHO_AM_I 0x")); tx.print(whoAmI, HEX); tx.println(')'); }
  else tx.println(F("NOT FOUND - check wiring"));

  detBegin();
  stateSince = millis();
  startCalibration();
  state = imuReady ? ST_CALIBRATING : ST_IMU_FAULT;

  // Power was lost (or the USB port reset the UNO) during an incident? Resume it.
  if (cfg.activeState) {
    Inc r;
    bool found = false;
    for (uint8_t i = 0; i < cfg.ringCount && !found; ++i) if (readIncident(i, r) && r.id == cfg.activeId) found = true;
    if (found) {
      inc = r;
      inc.flags |= F_RESUMED;
      incOpen = true;
      logTag(F("BOOT")); tx.print(F("Resumed incident #")); tx.println(inc.id);
      if (cfg.activeState == 2) {
        sosPending = !(inc.flags & F_SOS_ACK);
        sosLastTx = millis() - SOS_RETRY_MS;
        state = ST_RECOVERY;
      } else {
        state = ST_BOOT;
        enterCrash(F("restored after restart"));
      }
    } else setActive(0, 0);
  }

  // Announce ourselves: the CH340 resets the UNO when the dashboard opens the
  // port, so the app's HELLO is often lost in the bootloader.
  sendHello();
  sendCfg();
  sendState(state, state, F("boot"));
  if (state == ST_IMU_FAULT) { lastErr = 4; logTag(F("IMU")); tx.println(F("SENSOR ERROR - red LED solid")); }
  nextSampleUs = lastSampleUs = micros();
}

void loop() {
  const uint32_t us = micros();
  if ((int32_t)(us - nextSampleUs) >= 0) {
    float dt = (us - lastSampleUs) * 1e-6f;
    lastSampleUs = us;
    nextSampleUs += 1000000UL / SAMPLE_HZ;
    if ((int32_t)(us - nextSampleUs) > 50000L) nextSampleUs = us;     // fell far behind: resync
    if (dt > 0.05f) { fInit = false; dt = 0.01f; }                     // long stall: re-init attitude
    processSample(millis(), dt);
  }
  const uint32_t now = millis();
  serviceSerial(now);
  serviceButton(now);
  serviceTimers(now);
  serviceOutputs(now);
  if (since(now, lastTel, TEL_PERIOD_MS) && tx.freeSpace() > 235) { lastTel = now; sendTel(now); }
  tx.pump();
}
