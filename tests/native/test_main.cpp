// Native end-to-end test-suite for the Bike Safety Belt firmware.
//
// Runs the REAL firmware modules (calibration, fusion, detector, state machine,
// protocol, storage) on a PC with a virtual clock and fake hardware:
//   * FakeImu       — raw MPU-6050 counts with gyro bias + noise; can replay any
//                     simulator scenario through the full calibration pipeline,
//                     or simulate a disconnected sensor
//   * FakeNv        — EEPROM image that survives a simulated reboot
//   * FakeTransport — captures every output line, injects app commands
//
// Build & run:  make -C tests/native        (or: tests/native/run_tests.sh)
#include <cstdio>
#include <cstring>
#include <deque>
#include <map>
#include <memory>
#include <set>
#include <string>
#include <vector>

#include "native_platform.h"
#include "src/app/bike_safety_app.h"
#include "src/gps/gps_nmea.h"
#include "src/hardware/button.h"

static int g_fail = 0, g_pass = 0;
#define CHECK(cond, ...)                                    \
  do {                                                      \
    if (cond) { ++g_pass; }                                 \
    else {                                                  \
      ++g_fail;                                             \
      printf("  FAIL %s:%d: %s — ", __FILE__, __LINE__, #cond); \
      printf(__VA_ARGS__);                                  \
      printf("\n");                                         \
    }                                                       \
  } while (0)

// ---------------------------------------------------------------------------
class FakeImu : public RawImuSensor {
 public:
  bool disconnected = false;
  uint8_t playback = 0;      // simulator scenario replayed through the raw pipeline
  ImuSimulator sim;
  uint32_t rng = 99;
  Vec3 bias = {0.6f, -0.9f, 0.35f};

  bool begin() override { return !disconnected; }
  uint8_t whoAmI() const override { return 0x68; }
  float accelLsbPerG() const override { return 2048.0f; }
  float gyroLsbPerDps() const override { return 16.4f; }
  void play(uint8_t id) { playback = id; sim.start(id, millis(), 777); }
  float noise() { rng = rng * 1664525u + 1013904223u; return ((rng >> 8) / 16777216.0f - 0.5f) * 0.01f; }

  ImuStatus read(RawImu& o) override {
    if (disconnected) return ImuStatus::BUS_ERROR;
    Vec3 a = v3(noise(), noise(), 1.0f + noise()), g = v3(noise() * 10, noise() * 10, noise() * 10);
    if (playback) {
      if (sim.finished(millis())) { playback = 0; }
      else {
        ImuSample s;
        if (!sim.next(millis(), s)) return ImuStatus::BUS_ERROR;
        a = s.acc; g = s.gyro;
      }
    }
    g = vadd(g, bias);
    auto c = [](float v) { return (int16_t)(v > 32767 ? 32767 : v < -32768 ? -32768 : v); };
    o.ax = c(a.x * 2048); o.ay = c(a.y * 2048); o.az = c(a.z * 2048);
    o.gx = c(g.x * 16.4f); o.gy = c(g.y * 16.4f); o.gz = c(g.z * 16.4f);
    return ImuStatus::OK;
  }
};

class FakeNv : public NvBackend {
 public:
  uint8_t mem[512];
  FakeNv() { memset(mem, 0xFF, sizeof(mem)); }
  uint16_t size() const override { return sizeof(mem); }
  uint8_t read(uint16_t a) override { return mem[a]; }
  bool ready() override { return true; }
  void write(uint16_t a, uint8_t v) override { mem[a] = v; }
};

class FakeTransport : public Transport {
 public:
  std::vector<std::pair<uint32_t, std::string>> lines;
  std::deque<std::string> inbox;
  std::string cur, rx;
  void begin() override {}
  const char* pollLine() override {
    if (inbox.empty()) return nullptr;
    rx = inbox.front();
    inbox.pop_front();
    return rx.c_str();
  }
  void write(const char* d, uint8_t n) override { cur.append(d, n); }
  void endLine() override { lines.push_back({millis(), cur}); cur.clear(); }
  bool carriesLogs() const override { return true; }
  bool linkUp() const override { return true; }
  FStr name() const override { return "USB"; }
  size_t count(const std::string& needle, size_t from = 0) const {
    size_t n = 0;
    for (size_t i = from; i < lines.size(); ++i) if (lines[i].second.find(needle) != std::string::npos) ++n;
    return n;
  }
};

struct Rig {
  FakeImu imu;
  FakeNv nv;
  FakeTransport tx;
  std::unique_ptr<BikeSafetyApp> app;
  std::set<SystemState> seen;
  bool keepAlive = true;
  uint32_t lastPing = 0;

  void boot() {
    app.reset(new BikeSafetyApp());
    app->begin(&imu, &nv, &tx, nullptr, nullptr, 0x1A2B);
  }
  void cmd(const std::string& c) { tx.inbox.push_back(c); }
  void run(uint32_t ms) {
    for (uint32_t i = 0; i < ms; ++i) {
      g_us += 1000;
      if (keepAlive && millis() - lastPing >= 1000) { lastPing = millis(); cmd("PING"); }
      app->loop();
      seen.insert(app->state());
    }
  }
  // Run until state reached (or timeout). Returns true if reached.
  bool runUntil(SystemState s, uint32_t maxMs) {
    for (uint32_t i = 0; i < maxMs; i += 10) { run(10); if (app->state() == s) return true; }
    return false;
  }
};

static const char* sname(SystemState s) { return SafetyController::stateName(s); }

// ---------------------------------------------------------------------------
static void testUnits() {
  printf("[unit] button debouncer, NMEA parser, level rotation, countdown\n");
  {
    ButtonDebouncer b;
    b.begin(30, 3000, 15000, false, 0);
    uint8_t ev = 0;
    // bouncing contact: must give exactly one PRESS
    int presses = 0;
    for (uint32_t t = 1; t < 60; ++t) { ev = b.update((t % 3) != 0 || t > 20, t); if (ev & BTN_PRESS) ++presses; }
    for (uint32_t t = 60; t < 2000; ++t) { ev = b.update(true, t); if (ev & BTN_PRESS) ++presses; }
    CHECK(presses == 1, "presses=%d (holding must not repeat)", presses);
    int longs = 0;
    for (uint32_t t = 2000; t < 6000; ++t) { ev = b.update(true, t); if (ev & BTN_LONG) ++longs; }
    CHECK(longs == 1, "long presses=%d", longs);
    ButtonDebouncer held;
    held.begin(30, 3000, 15000, true, 0);   // held at power-up
    int p2 = 0;
    for (uint32_t t = 1; t < 500; ++t) if (held.update(true, t) & BTN_PRESS) ++p2;
    CHECK(p2 == 0, "button held at boot must be ignored");
  }
  {
    NmeaParser g;
    const char* s1 = "$GPRMC,123519,A,4807.038,N,01131.000,E,022.4,084.4,230394,003.1,W*6A\r\n";
    const char* s2 = "$GPGGA,123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,*47\r\n";
    for (const char* p = s1; *p; ++p) g.feed(*p, 1000);
    for (const char* p = s2; *p; ++p) g.feed(*p, 1000);
    CHECK(g.hasValidFix(1500), "valid fix expected");
    CHECK(g.fix().latE7 == 481173000, "lat %d", (int)g.fix().latE7);
    CHECK(g.fix().lonE7 == 115166666, "lon %d", (int)g.fix().lonE7);
    CHECK(!g.hasValidFix(1000 + GPS_MAX_FIX_AGE_MS + 1), "stale fix must be invalid");
    NmeaParser bad;
    const char* s3 = "$GPRMC,123519,A,4807.038,N,01131.000,E,022.4,084.4,230394,003.1,W*00\r\n";
    for (const char* p = s3; *p; ++p) bad.feed(*p, 1000);
    CHECK(!bad.hasValidFix(1000) && bad.checksumErrors() == 1, "bad checksum must be rejected");
    NmeaParser nofix;
    const char* s4 = "$GPRMC,123519,V,,,,,,,230394,,*3C\r\n";
    for (const char* p = s4; *p; ++p) nofix.feed(*p, 1000);
    CHECK(!nofix.hasValidFix(1000), "status V must not give a fix");
  }
  {
    ImuPipeline p;
    p.begin(nullptr, AXIS_POS_X, AXIS_POS_Z);
    const Vec3 up = vscale(v3(0.3f, -0.2f, 0.93f), 1.0f / vlen(v3(0.3f, -0.2f, 0.93f)));
    p.setLevel(up);
    Vec3 a = up, g = v3(0, 0, 0);
    p.correct(a, g);
    CHECK(fabsf(a.x) < 1e-4f && fabsf(a.y) < 1e-4f && fabsf(a.z - 1) < 1e-4f, "level rotation maps up->z (%f %f %f)", a.x, a.y, a.z);
    MountTransform m;
    CHECK(m.configure(AXIS_NEG_Y, AXIS_POS_Z), "mount config");
    const Vec3 v = m.apply(v3(0, -1, 0));   // sensor -Y points forward
    CHECK(fabsf(v.x - 1) < 1e-6f, "mount mapping forward");
    CHECK(!m.configure(AXIS_POS_X, AXIS_NEG_X), "parallel axes must be rejected");
  }
  {
    Countdown c;
    c.start(1000, 10);
    CHECK(c.remaining(1000) == 10 && c.remaining(1001) == 10 && c.remaining(1999) == 10 && c.remaining(2000) == 9,
          "countdown rounding");
    CHECK(!c.expired(10999) && c.expired(11000), "countdown expiry");
  }
}

static void testBootAndProtocol() {
  printf("[e2e] boot, calibration, app session, telemetry\n");
  Rig r;
  r.keepAlive = false;   // a plain serial monitor: no app heartbeat
  r.boot();
  CHECK(r.app->state() == SystemState::CALIBRATING, "state %s", sname(r.app->state()));
  r.run(4000);
  CHECK(r.app->state() == SystemState::SAFE, "after calibration: %s", sname(r.app->state()));
  CHECK(r.app->calQuality() == CAL_OK, "calibration quality %d", r.app->calQuality());
  CHECK(r.tx.count("[CALIBRATION] Complete") == 1, "calibration log");
  CHECK(r.tx.count("{\"t\":\"tel\"") == 0, "no JSON before HELLO (clean serial monitor)");
  r.cmd("HELLO");
  r.keepAlive = true;
  r.run(2000);
  CHECK(r.tx.count("{\"t\":\"hello\"") == 1, "hello");
  CHECK(r.tx.count("{\"t\":\"cfg\"") >= 1, "cfg");
  const size_t tel = r.tx.count("{\"t\":\"tel\"");
  CHECK(tel >= 15 && tel <= 21, "telemetry ~10 Hz, got %zu in 2 s", tel);
  CHECK(fabsf(r.app->attitude().rollDeg) < 2.0f, "upright roll %f", r.app->attitude().rollDeg);
  // Gyro bias removed by calibration
  CHECK(r.app->detector().liveConfidence() < 0.05f, "idle confidence");
  // Heartbeat loss
  r.keepAlive = false;
  r.run(5000);
  CHECK(r.tx.count("App heartbeat lost") == 1, "heartbeat loss detected");
  const size_t before = r.tx.count("{\"t\":\"tel\"");
  r.run(1000);
  CHECK(r.tx.count("{\"t\":\"tel\"") == before, "telemetry stops when the app is gone");
}

struct Expect { uint8_t sim; bool possibleAllowed; bool crash; };

static void testScenarios() {
  printf("[e2e] simulated scenarios through full firmware (3 profiles)\n");
  const Expect cases[] = {
    {SIM_STATIONARY, false, false}, {SIM_NORMAL_LEAN, false, false}, {SIM_CORNERING, false, false},
    {SIM_BRAKING, false, false}, {SIM_SPEED_BREAKER, false, false}, {SIM_SUDDEN_ROTATION, false, false},
    {SIM_POTHOLE, false, false}, {SIM_TIPOVER_PARKED, true, false},
    {SIM_CRASH, true, true}, {SIM_HIGHSIDE, true, true},
  };
  const char* profiles[] = {"scooter", "standard", "sport"};
  for (const char* prof : profiles) {
    for (const Expect& e : cases) {
      Rig r;
      r.boot();
      r.run(4000);
      r.cmd(std::string("PROFILE ") + prof);
      r.run(100);
      r.seen.clear();
      r.cmd("SIM " + std::to_string(e.sim));
      r.run(ImuSimulator::durationMs(e.sim) + 200);
      const bool sawPossible = r.seen.count(SystemState::POSSIBLE_CRASH) > 0;
      const bool sawCrash = r.seen.count(SystemState::CRASH_DETECTED) > 0;
      CHECK(sawCrash == e.crash, "%s sim %d: crash=%d expected %d", prof, e.sim, sawCrash, e.crash);
      if (!e.possibleAllowed) CHECK(!sawPossible, "%s sim %d: false POSSIBLE_CRASH", prof, e.sim);
      if (e.crash) {
        CHECK(r.tx.count("Evaluation: CRASH LIKELY") == 1, "%s sim %d explanation logged", prof, e.sim);
      }
    }
  }
}

static void testCountdownCancelAndExpiry() {
  printf("[e2e] countdown cancel (button + app), expiry -> SOS once, ack, recovery, reset\n");
  {
    Rig r; r.boot(); r.run(4000); r.cmd("HELLO"); r.cmd("SIM 7");
    CHECK(r.runUntil(SystemState::CRASH_DETECTED, 12000), "crash detected");
    CHECK(!r.app->leds().greenOn(), "green off in countdown");
    r.run(3000);
    r.cmd("CANCEL");
    r.run(50);
    CHECK(r.app->state() == SystemState::USER_CANCELLED, "cancelled: %s", sname(r.app->state()));
    CHECK(r.tx.count("{\"t\":\"sos\"") == 0, "no SOS after cancel");
    r.run(3500);
    CHECK(r.app->state() == SystemState::SAFE, "back to SAFE: %s", sname(r.app->state()));
  }
  {
    Rig r; r.boot(); r.run(4000); r.cmd("SIM 16");
    CHECK(r.runUntil(SystemState::CRASH_DETECTED, 12000), "crash detected (highside)");
    g_pinIn[PIN_BUTTON] = 0; r.run(100); g_pinIn[PIN_BUTTON] = 1; r.run(100);
    CHECK(r.app->state() == SystemState::USER_CANCELLED, "button cancel: %s", sname(r.app->state()));
    g_pinIn[PIN_BUTTON] = 0; r.run(100); g_pinIn[PIN_BUTTON] = 1; r.run(100);   // double press within lockout
    CHECK(r.app->state() != SystemState::MANUAL_TEST, "double press after cancel ignored");
  }
  {
    Rig r; r.boot(); r.run(4000); r.cmd("HELLO"); r.cmd("SIM 7");
    CHECK(r.runUntil(SystemState::CRASH_DETECTED, 12000), "crash detected");
    CHECK(r.runUntil(SystemState::SOS_SENT, 11000), "SOS after countdown");
    r.run(10000);
    const size_t sos = r.tx.count("{\"t\":\"sos\"");
    CHECK(sos >= 3 && sos <= 5, "SOS retransmitted until acknowledged (%zu)", sos);
    CHECK(r.tx.count("\"sim\":1") > 0, "simulated SOS flagged sim");
    const uint32_t id = r.app->controller().incident().id;
    r.cmd("SOSACK " + std::to_string(id));
    r.run(100);
    const size_t after = r.tx.count("{\"t\":\"sos\"");
    r.run(10000);
    CHECK(r.tx.count("{\"t\":\"sos\"") == after, "no retransmission after ack");
    CHECK(!r.app->sos().pending(), "SOS no longer pending");
    CHECK(r.runUntil(SystemState::RECOVERY, 60000), "RECOVERY after alarm period");
    CHECK(r.tx.count("Countdown expired") == 1, "SOS issued exactly once");
    g_pinIn[PIN_BUTTON] = 0; r.run(200); g_pinIn[PIN_BUTTON] = 1; r.run(100);
    CHECK(r.app->state() == SystemState::RECOVERY, "short press does not reset");
    g_pinIn[PIN_BUTTON] = 0; r.run(3200); g_pinIn[PIN_BUTTON] = 1; r.run(100);
    CHECK(r.app->state() == SystemState::SAFE, "long press reset: %s", sname(r.app->state()));
  }
}

static void testManualButton() {
  printf("[e2e] manual test button\n");
  Rig r; r.boot(); r.run(4000);
  CHECK(r.app->leds().greenOn() && !r.app->leds().redOn(), "SAFE LEDs");
  g_pinIn[PIN_BUTTON] = 0; r.run(100);
  CHECK(r.app->state() == SystemState::MANUAL_TEST, "first press: %s", sname(r.app->state()));
  CHECK(!r.app->leds().greenOn() && r.app->leds().redOn() && r.app->buzzer().on(), "test outputs");
  r.run(4000);   // held: no repeat
  CHECK(r.app->state() == SystemState::MANUAL_TEST, "holding does not toggle");
  g_pinIn[PIN_BUTTON] = 1; r.run(100);
  g_pinIn[PIN_BUTTON] = 0; r.run(100); g_pinIn[PIN_BUTTON] = 1; r.run(100);
  CHECK(r.app->state() == SystemState::SAFE, "second press: %s", sname(r.app->state()));
  CHECK(r.app->leds().greenOn() && !r.app->leds().redOn() && !r.app->buzzer().on(), "SAFE outputs restored");
  CHECK(r.tx.count("{\"t\":\"sos\"") == 0 && r.tx.count("Countdown") == 0, "manual test never sends SOS");
}

static void testImuFailures() {
  printf("[e2e] sensor disconnect (simulated + real), IMU lost during possible crash\n");
  {
    Rig r; r.boot(); r.run(4000); r.cmd("SIM 9");
    CHECK(r.runUntil(SystemState::IMU_FAULT, 4000), "simulated disconnect -> IMU_FAULT");
    CHECK(r.runUntil(SystemState::SAFE, 8000), "recovers to SAFE");
  }
  {
    Rig r; r.boot(); r.run(4000);
    r.imu.disconnected = true;
    CHECK(r.runUntil(SystemState::IMU_FAULT, 500), "real disconnect -> IMU_FAULT");
    CHECK(!r.app->leds().greenOn() || !r.app->leds().redOn(), "error pattern alternates");
    r.run(3000);
    CHECK(r.app->state() == SystemState::IMU_FAULT, "stays faulted, never SAFE");
    r.imu.disconnected = false;
    CHECK(r.runUntil(SystemState::SAFE, 5000), "reconnect -> SAFE");
  }
  {
    Rig r; r.boot(); r.run(4000);
    r.imu.play(SIM_CRASH);
    CHECK(r.runUntil(SystemState::POSSIBLE_CRASH, 6000), "possible crash (real pipeline)");
    r.imu.disconnected = true;
    CHECK(r.runUntil(SystemState::CRASH_DETECTED, 500), "IMU lost during possible crash escalates");
  }
  {
    Rig r; r.imu.disconnected = true; r.boot(); r.run(1000);
    CHECK(r.app->state() == SystemState::IMU_FAULT, "boot without IMU: %s", sname(r.app->state()));
  }
}

static void testRestartRecovery() {
  printf("[e2e] controller restart during countdown / after SOS\n");
  Rig r; r.boot(); r.run(4000);
  r.imu.play(SIM_CRASH);   // real (non-simulated) incident through the raw pipeline
  CHECK(r.runUntil(SystemState::CRASH_DETECTED, 12000), "real crash detected");
  r.run(3000);
  const uint32_t id = r.app->controller().incident().id;
  r.imu.playback = 0;
  r.boot();   // power glitch
  CHECK(r.app->state() == SystemState::CRASH_DETECTED, "countdown resumed after restart: %s", sname(r.app->state()));
  CHECK(r.app->controller().incident().id == id, "same incident id");
  CHECK(r.runUntil(SystemState::SOS_SENT, 11000), "SOS after resumed countdown");
  r.run(500);
  r.boot();   // restart after SOS
  CHECK(r.app->state() == SystemState::RECOVERY, "restart after SOS -> RECOVERY: %s", sname(r.app->state()));
  CHECK(r.app->sos().pending(), "unacknowledged SOS re-queued for delivery");
}

static void testComms() {
  printf("[e2e] communication failure test, unknown commands\n");
  Rig r; r.boot(); r.run(4000); r.cmd("HELLO"); r.run(500);
  r.cmd("COMMS 5"); r.run(100);
  const size_t t0 = r.tx.count("{\"t\":\"tel\"");
  r.run(4500);
  CHECK(r.tx.count("{\"t\":\"tel\"") == t0, "telemetry paused during COMMS test");
  r.run(1500);
  CHECK(r.tx.count("{\"t\":\"tel\"") > t0, "telemetry resumes");
  r.cmd("FOO"); r.run(10);
  CHECK(r.tx.count("Unknown command") == 1, "unknown command reported");
  r.cmd("GPSLOSS 5"); r.run(10);
  CHECK(r.tx.count("No GPS module installed") == 1, "no fake GPS");
}

// Generates the dashboard replay sample from a scripted session.
static void writeDemo(const char* path) {
  Rig r; r.boot(); r.run(4000); r.cmd("HELLO"); r.run(3000);
  r.cmd("SIM 3"); r.run(ImuSimulator::durationMs(3) + 500);
  r.cmd("SIM 5"); r.run(ImuSimulator::durationMs(5) + 500);
  r.cmd("SIM 7");
  r.runUntil(SystemState::SOS_SENT, 30000);
  r.run(4000);
  r.cmd("SOSACK " + std::to_string(r.app->controller().incident().id));
  r.run(3000);
  r.cmd("RESET"); r.run(3000);
  r.cmd("BTN"); r.run(3000); r.cmd("BTN"); r.run(3000);
  FILE* f = fopen(path, "w");
  if (!f) { printf("cannot write %s\n", path); return; }
  for (auto& l : r.tx.lines) fprintf(f, "%u\t%s\n", l.first, l.second.c_str());
  fclose(f);
  printf("[demo] wrote %zu lines to %s\n", r.tx.lines.size(), path);
}

int main(int argc, char** argv) {
  testUnits();
  testBootAndProtocol();
  testScenarios();
  testCountdownCancelAndExpiry();
  testManualButton();
  testImuFailures();
  testRestartRecovery();
  testComms();
  if (argc > 1) writeDemo(argv[1]);
  printf("\n%d checks passed, %d failed\n", g_pass, g_fail);
  return g_fail ? 1 : 0;
}
