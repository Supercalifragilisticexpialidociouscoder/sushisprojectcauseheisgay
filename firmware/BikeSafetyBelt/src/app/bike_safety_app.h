// Application: wires sensor -> calibration -> fusion -> detector -> state
// machine -> LEDs / buzzer / app link. Runs as a cooperative, non-blocking
// scheduler: loop() never delays; every task checks its own timer.
#pragma once

#include "../../config.h"
#include "../core/types.h"
#include "../core/state_machine.h"
#include "../config/vehicle_profiles.h"
#include "../sensor/mpu6050.h"
#include "../sensor/calibration.h"
#include "../sensor/sensor_fusion.h"
#include "../sensor/imu_simulator.h"
#include "../detection/crash_detector.h"
#include "../hardware/leds.h"
#include "../hardware/buzzer.h"
#include "../hardware/button.h"
#include "../communication/link.h"
#include "../communication/line_writer.h"
#include "../emergency/sos.h"
#include "../gps/gps_nmea.h"
#include "../storage/persistent_store.h"

// Byte source for the optional GPS module (returns -1 when no data).
class CharSource {
 public:
  virtual int read() = 0;
};

enum CalQuality : uint8_t { CAL_RUNNING = 0, CAL_OK = 1, CAL_DEGRADED = 2, CAL_UNCALIBRATED = 3 };

class BikeSafetyApp : public ControllerListener {
 public:
  BikeSafetyApp() : w_(link_) {}
  // `deviceName()` is filled in begin() before any transport starts.
  const char* deviceName() const { return deviceName_; }

  void begin(RawImuSensor* imu, NvBackend* nv, Transport* t0, Transport* t1, CharSource* gps, uint16_t seed);
  void loop();

  // ControllerListener
  void onStateChange(SystemState from, SystemState to, StateReason why) override;
  void onIncidentUpdate(const IncidentRecord& rec) override;
  void onSosIssued(const IncidentRecord& rec) override;
  bool currentLocation(int32_t& latE7, int32_t& lonE7) override;

  // Introspection (used by the native test-suite)
  SystemState state() const { return controller_.state(); }
  const SafetyController& controller() const { return controller_; }
  const CrashDetector& detector() const { return detector_; }
  const Attitude& attitude() const { return fusion_.attitude(); }
  const SosManager& sos() const { return sos_; }
  const LedController& leds() const { return leds_; }
  const BuzzerController& buzzer() const { return buzzer_; }
  bool calibrating() const { return calibrating_; }
  bool imuHealthy() const { return imuHealthy_; }
  uint8_t simScenario() const { return sim_.scenario(); }
  uint8_t calQuality() const { return calQuality_; }

 private:
  // Scheduler tasks
  void processSample(uint32_t now, float dt);
  void serviceLink(uint32_t now);
  void serviceButton(uint32_t now);
  void serviceOutputs(uint32_t now);
  void serviceTelemetry(uint32_t now);
  void serviceSos(uint32_t now);
  void serviceImu(uint32_t now);
  void serviceSim(uint32_t now);
  void serviceGps(uint32_t now);
  void serviceStatusLog(uint32_t now);
  void serviceCountdownLog(uint32_t now);

  // Sensor / calibration
  void beginStartupCalibration(uint32_t now);
  void handleStartupCalibration(const Vec3& acc, const Vec3& gyro, uint32_t now);
  void finishCalibration(uint32_t now);
  void handleLevelCalibration(const Vec3& acc, const Vec3& gyro, uint32_t now);
  void markImuFault(uint32_t now, bool simulated);
  void markImuRecovered(uint32_t now);
  void resetDetection(uint32_t now);
  void handleDetectorEvents(uint8_t ev, uint32_t now);
  void logVerdict(const DetectionReport& rep);
  void logIndicator(FStr label, float value, uint8_t dec, FStr unit, float score);

  // Commands
  void handleCommand(const char* line, uint8_t src, uint32_t now);
  void ack(const char* cmd, bool ok);
  bool commandsAllowed() const;
  void printHelp();

  // Messages
  void sendHello(uint32_t now);
  void sendConfig();
  void sendState(SystemState from, SystemState to, StateReason why);
  void sendTelemetry(uint8_t idx, uint32_t now);
  void sendDetection(FStr kind, const DetectionReport& rep);
  void sendIncident(const IncidentRecord& rec);
  void sendSos(uint32_t now);
  void writeLocationFields(uint32_t now);
  void logStatus(uint32_t now);
  void logLocation(uint32_t now);
  bool gpsValid(uint32_t now) const;

  // Modules
  ImuPipeline pipeline_;
  SensorFusion fusion_;
  CrashDetector detector_;
  SafetyController controller_;
  ImuSimulator sim_;
  StartupCalibrator startupCal_;
  LevelCalibrator levelCal_;
  LedController leds_;
  BuzzerController buzzer_;
  Button button_;
  Link link_;
  LineWriter w_;
  PersistentStore store_;
  SosManager sos_;
#if GPS_ENABLED
  NmeaParser gpsParser_;
  bool gpsWasValid_ = false;
#endif
  CharSource* gps_ = nullptr;
  VehicleProfile profile_;
  ControllerConfig ctrlCfg_;

  char deviceName_[10] = "BSB-0000";
  uint16_t bootNo_ = 0;

  // Sampling
  uint32_t nextSampleUs_ = 0, lastSampleUs_ = 0;
  bool imuReady_ = false;
  bool imuHealthy_ = true;
  uint8_t failCount_ = 0;
  ImuStatus lastImuStatus_ = ImuStatus::OK;
  uint32_t lastImuRetryMs_ = 0;

  // Calibration
  bool calibrating_ = false;
  bool levelCalActive_ = false;
  uint8_t calAttempts_ = 0;
  uint8_t calQuality_ = CAL_RUNNING;

  // Telemetry
  float lastAcc_ = 1.0f, lastGyro_ = 0.0f;
  float peakAcc_[Link::kMax] = {0, 0};
  float peakGyro_[Link::kMax] = {0, 0};
  uint32_t lastTelMs_[Link::kMax] = {0, 0};
  int16_t batteryMv_ = -1;
  uint32_t lastBatteryMs_ = 0;

  // Logging
  uint32_t lastStatusMs_ = 0;
  uint32_t lastCandLogMs_ = 0;
  bool candLogged_ = false;
  uint8_t lastCountdownLogged_ = 0xFF;
};
