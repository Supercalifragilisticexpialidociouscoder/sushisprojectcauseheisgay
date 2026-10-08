// =============================================================================
//  Bike Safety Belt — CENTRAL CONFIGURATION
// =============================================================================
//  Every pin, threshold, timing and tuning value used by the firmware lives in
//  this file. Nothing else in the code base should contain "magic numbers"
//  that change system behaviour.
//
//  Sections
//    1. Pins (existing prototype wiring)
//    2. Optional hardware (BLE / GPS / battery sense)
//    3. MPU-6050 sensor setup
//    4. Sensor mounting orientation
//    5. Calibration
//    6. Sensor fusion (complementary filter)
//    7. Vehicle profiles (lean / impact / rotation thresholds)
//    8. Crash detector (scoring, timing, false-positive guards)
//    9. Emergency countdown / SOS / recovery
//   10. Button
//   11. Buzzer
//   12. Communication / telemetry / logging
//   13. IMU health monitoring
// =============================================================================
#pragma once

// -----------------------------------------------------------------------------
// 1. PINS — existing prototype wiring. Do not change unless the wiring changes.
// -----------------------------------------------------------------------------
#define PIN_RED_LED    8    // D8  -> 220R -> red LED   -> GND
#define PIN_GREEN_LED  9    // D9  -> 220R -> green LED -> GND
#define PIN_BUZZER     10   // D10 -> buzzer (+), buzzer (-) -> GND
#define PIN_BUTTON     3    // D3  -> push button -> GND  (INPUT_PULLUP)

// I2C pins: -1 = board default (Uno/Nano: A4=SDA, A5=SCL; ESP32: 21/22).
#define PIN_I2C_SDA    -1
#define PIN_I2C_SCL    -1

#define LED_ACTIVE_HIGH     1   // 1: digitalWrite(HIGH) turns the LED on
#define BUTTON_ACTIVE_LOW   1   // 1: pressed = LOW (button to GND + INPUT_PULLUP)

// -----------------------------------------------------------------------------
// 2. OPTIONAL HARDWARE — set to 1 ONLY if the part is physically connected.
//    The firmware never reports data from hardware that is disabled here.
// -----------------------------------------------------------------------------
#if defined(ARDUINO_ARCH_ESP32)
  #define BLE_ENABLED  1    // ESP32 has a BLE radio built in (Nordic UART service)
#else
  #define BLE_ENABLED  0    // Uno / Nano have no radio
#endif
#define BLE_DEVICE_NAME_PREFIX "BSB-"

#define GPS_ENABLED          0     // 1 = NMEA GPS module (e.g. NEO-6M) connected
#define GPS_BAUD             9600
#define GPS_RX_PIN           4     // MCU RX  <- GPS TX  (SoftwareSerial on AVR)
#define GPS_TX_PIN           5     // MCU TX  -> GPS RX  (optional)
#define GPS_MAX_FIX_AGE_MS   5000UL
#define GPS_MIN_SATELLITES   4
#define GPS_MAX_HDOP         5.0f
#define GPS_UERE_M           5.0f  // accuracy estimate = HDOP * UERE (estimate only)

#define BATTERY_ADC_PIN      -1    // -1 = no battery sensing hardware
#define BATTERY_DIVIDER      2.0f  // Vbat = Vadc * divider
#define BATTERY_ADC_REF_MV   5000.0f
#define BATTERY_ADC_MAX      1023.0f

// -----------------------------------------------------------------------------
// 3. MPU-6050 SENSOR SETUP
// -----------------------------------------------------------------------------
#define MPU_I2C_ADDRESS      0x68  // AD0 = GND -> 0x68, AD0 = VCC -> 0x69
#define MPU_I2C_CLOCK_HZ     400000UL
#define MPU_I2C_TIMEOUT_US   3000UL

// Accelerometer full-scale range: 2, 4, 8 or 16 (g).
// 16 g is used so crash impacts do not saturate the sensor.
#define MPU_ACCEL_RANGE_G    16
// Gyroscope full-scale range: 250, 500, 1000 or 2000 (deg/s).
#define MPU_GYRO_RANGE_DPS   2000
// Digital low-pass filter (register CONFIG.DLPF_CFG). 3 = 44 Hz accel / 42 Hz gyro.
// Must stay below SAMPLE_RATE_HZ / 2 to avoid aliasing road/engine vibration.
#define MPU_DLPF_CFG         3

#define SAMPLE_RATE_HZ       100   // detector / fusion update rate

// -----------------------------------------------------------------------------
// 4. SENSOR MOUNTING ORIENTATION
//    Which axis printed on the MPU-6050 board points to the FRONT of the bike,
//    and which points UP (towards the sky) when the bike is upright.
//    Options: AXIS_POS_X, AXIS_NEG_X, AXIS_POS_Y, AXIS_NEG_Y, AXIS_POS_Z, AXIS_NEG_Z
//    Small residual tilt is removed with the "CAL LEVEL" calibration.
// -----------------------------------------------------------------------------
#define MOUNT_FORWARD_AXIS   AXIS_POS_X
#define MOUNT_UP_AXIS        AXIS_POS_Z

// -----------------------------------------------------------------------------
// 5. CALIBRATION
// -----------------------------------------------------------------------------
#define CAL_SAMPLES              300     // startup samples (3 s at 100 Hz)
#define CAL_MAX_ATTEMPTS         5       // retries when motion is detected
#define CAL_MAX_GYRO_STD_DPS     2.0f    // gyro noise allowed while "stationary"
#define CAL_MAX_GYRO_BIAS_DPS    25.0f   // larger mean rate = sensor is rotating
#define CAL_GRAVITY_TOLERANCE_G  0.15f   // |a| must be 1 g +/- this while stationary
#define CAL_LEVEL_SAMPLES        200     // "CAL LEVEL" riding-orientation capture
#define CAL_LEVEL_MAX_TILT_DEG   45.0f   // bigger = mount axes configured wrong
#define CAL_GYRO_SAVE_DELTA_DPS  0.3f    // only rewrite EEPROM if bias changed this much

// -----------------------------------------------------------------------------
// 6. SENSOR FUSION — adaptive complementary filter
//    angle = alpha * (angle + rate * dt) + (1 - alpha) * reference_angle
// -----------------------------------------------------------------------------
#define FUSION_ALPHA              0.98f  // accel reference (tau ~ 0.5 s at 100 Hz)
#define FUSION_ALPHA_TURN         0.99f  // turn-kinematics reference while cornering
#define FUSION_ACC_LPF_ALPHA      0.25f  // accel low-pass used for the reference
#define FUSION_ACC_TRUST_BAND_G   0.10f  // accel trusted only if | |a| - 1 g | < band
#define FUSION_GYRO_TRUST_MAX_DPS 30.0f  // ... and total rotation below this
#define FUSION_TURN_RATE_MIN_DPS  3.0f   // yaw rate that counts as "cornering"
#define FUSION_STEADY_ROLL_DPS    15.0f  // roll rate limit for turn-kinematic lean
#define FUSION_TURN_MAX_LEAN_DEG  70.0f  // turn-kinematic lean only valid below this

// -----------------------------------------------------------------------------
// 7. VEHICLE PROFILES
//    lean*      : tilt from upright (deg)  — normal max cornering / clearly abnormal
//    gyro*      : total angular velocity (deg/s)
//    impact*    : resultant acceleration (g)
//    tiltChg*   : orientation change within 0.5 s (deg)
//    rest*      : tilt after the bike comes to rest (deg) — side stand ~10-15 deg
//    Values between "normal" and "crash" are scored linearly 0..100 %.
// -----------------------------------------------------------------------------
#define PROFILE_DEFAULT  PROFILE_STANDARD   // PROFILE_SCOOTER / PROFILE_STANDARD / PROFILE_SPORT

//                         leanN leanC gyroN gyroC impN  impC tiltChgN tiltChgC restN restC
#define PROFILE_SCOOTER_VALUES  35.0f, 60.0f, 100.0f, 250.0f, 2.0f, 4.0f, 25.0f, 60.0f, 20.0f, 55.0f
#define PROFILE_STANDARD_VALUES 45.0f, 65.0f, 120.0f, 280.0f, 2.2f, 4.5f, 30.0f, 65.0f, 20.0f, 60.0f
#define PROFILE_SPORT_VALUES    55.0f, 72.0f, 150.0f, 320.0f, 2.5f, 5.0f, 35.0f, 70.0f, 20.0f, 60.0f

// -----------------------------------------------------------------------------
// 8. CRASH DETECTOR
// -----------------------------------------------------------------------------
// Candidate trigger: a sample is "anomalous" if any of these is exceeded.
// A candidate starts only when DET_TRIGGER_MIN_SAMPLES of the last
// DET_TRIGGER_WINDOW samples are anomalous (single noisy samples are ignored).
#define DET_TRIGGER_IMPACT_G        1.8f
#define DET_TRIGGER_GYRO_DPS        90.0f
#define DET_TRIGGER_TILT_RATE_DPS   60.0f
#define DET_TRIGGER_MIN_SAMPLES     2
#define DET_TRIGGER_WINDOW          4      // max 8

// Event timing
#define DET_QUIET_GYRO_DPS          30.0f  // "violent motion has stopped" ...
#define DET_QUIET_ACC_G             0.5f   // ... | |a| - 1 g | below this
#define DET_QUIET_MS                500    // for this long -> active phase ends
#define DET_ACTIVE_MAX_MS           4000   // active phase hard limit
#define DET_REST_GYRO_DPS           10.0f  // "at rest": low-passed rotation below
#define DET_REST_ACC_TOL_G          0.12f  // "at rest": | |a| - 1 g | below
#define DET_REST_CONFIRM_MS         1000   // must stay at rest this long
#define DET_POST_MAX_MS             5000   // give up waiting for rest after this
#define DET_UNSETTLED_POST_FACTOR   0.5f   // post-impact score weight if never at rest

// Indicator weights (must sum to 1.0)
#define DET_W_LEAN      0.15f   // 1: abnormal lean
#define DET_W_RATE      0.20f   // 2: angular velocity
#define DET_W_IMPACT    0.15f   // 3: acceleration / impact
#define DET_W_CHANGE    0.20f   // 4: sudden change in motion / orientation
#define DET_W_POST      0.30f   // 5: post-impact orientation

#define DET_SUSTAINED_RATE_NORMAL_MS 100.0f // time above gyroN: normal flick ...
#define DET_SUSTAINED_RATE_CRASH_MS  400.0f // ... vs tumbling
#define DET_VERTICAL_IMPACT_FRACTION 0.80f  // impact mostly along bike's vertical axis
#define DET_VERTICAL_IMPACT_WEIGHT   0.50f  // = road bump pattern -> impact score x 0.5
#define DET_MAJOR_MOTION_LEVEL       0.50f  // post-impact only counts after major motion
#define DET_INDICATOR_ACTIVE_LEVEL   0.50f  // indicator "active" at >= 50 %
#define DET_MIN_INDICATORS           3      // crash needs >= 3 active indicators
#define DET_POSSIBLE_THRESHOLD       0.30f  // -> POSSIBLE_CRASH
#define DET_CRASH_THRESHOLD          0.65f  // -> CRASH_DETECTED (countdown)

// Parked-vehicle guard: if the bike was completely still (engine off, nobody
// riding) for DET_PARKED_MIN_MS before the event, confidence is scaled down so a
// parked bike being knocked over raises POSSIBLE_CRASH but never sends an SOS.
#define DET_PARKED_MIN_MS           30000UL
#define DET_STILL_GYRO_DPS          1.5f
#define DET_STILL_ACC_G             0.02f
#define DET_PARKED_FACTOR           0.60f

// After an incident the detector re-arms only when the bike has been upright again.
#define DET_REARM_UPRIGHT_MS        3000
#define DET_LEAN_HYSTERESIS_DEG     5.0f

// -----------------------------------------------------------------------------
// 9. EMERGENCY COUNTDOWN / SOS / RECOVERY
// -----------------------------------------------------------------------------
#define COUNTDOWN_SECONDS          10
#define CANCEL_ACK_DISPLAY_MS      3000UL   // USER_CANCELLED shown before SAFE
#define SOS_ALARM_MS               60000UL  // loud SOS pattern, then RECOVERY
#define SOS_RETRY_MS               3000UL   // re-send SOS to app until acknowledged
#define SOS_UNDELIVERED_LOG_MS     30000UL  // "no app connected" reminder interval

// -----------------------------------------------------------------------------
// 10. BUTTON (TEST / MANUAL TRIGGER, cancel, reset)
// -----------------------------------------------------------------------------
#define BUTTON_DEBOUNCE_MS     30
#define BUTTON_LONG_PRESS_MS   3000   // hold to RESET after an SOS
#define BUTTON_STUCK_MS        15000  // held longer = treat as stuck, ignore
#define BUTTON_LOCKOUT_MS      2000   // ignore presses right after a cancel

// -----------------------------------------------------------------------------
// 11. BUZZER
// -----------------------------------------------------------------------------
#define BUZZER_ENABLED         1
#define BUZZER_PASSIVE         0      // 0 = active buzzer (on/off), 1 = passive (tone)
#define BUZZER_ACTIVE_HIGH     1
#define BUZZER_FREQ_HZ         2700   // passive buzzer only
#define BUZZER_TEST_CONTINUOUS 1      // manual test: continuous tone (prototype behaviour)
#define BUZZER_STARTUP_CHIRP   1      // short self-test chirp at boot

// -----------------------------------------------------------------------------
// 12. COMMUNICATION / TELEMETRY / LOGGING
// -----------------------------------------------------------------------------
#define SERIAL_BAUD              115200UL
#define TELEMETRY_PERIOD_MS      100UL    // USB serial JSON telemetry (app connected)
#define BLE_TELEMETRY_PERIOD_MS  200UL
#define APP_HEARTBEAT_TIMEOUT_MS 4000UL   // app must PING at least this often
#define STATUS_LOG_PERIOD_MS     5000UL   // human-readable status line (0 = off)
#define FIRMWARE_VERSION         "1.0.0"
#define PROTOCOL_VERSION         1

// -----------------------------------------------------------------------------
// 13. IMU HEALTH MONITORING
// -----------------------------------------------------------------------------
#define IMU_FAIL_LIMIT          5        // consecutive bad reads -> IMU_FAULT
#define IMU_STUCK_SAMPLES       25       // identical raw frames -> sensor frozen
#define IMU_RETRY_MS            2000UL   // re-initialisation attempt interval
#define IMU_MAX_GAP_MS          50UL     // sample gap that resets fusion integration
