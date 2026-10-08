# Bike Safety Belt

Two-wheeler rider safety system. An **MPU-6050** IMU detects a *possible* accident, the rider gets a **10-second countdown** to cancel, and if nobody cancels an **SOS** goes to the companion web app, which notifies configured emergency contacts.

> **Safety notice.** This is a prototype. It **cannot guarantee** that an accident is detected, and it **cannot guarantee** that an emergency alert is delivered. A detected event is not a confirmed accident. It never contacts emergency services by itself.

```
MPU-6050 ──I²C──► Microcontroller (firmware) ──USB serial / BLE──► Web dashboard ──► ntfy / webhook / SMS (manual)
                    │ calibration → fusion → crash detector → state machine
                    └ LEDs · buzzer · test button
```

## What's in the repo

| Path | Contents |
|---|---|
| `firmware/BikeSafetyBelt/` | Arduino sketch (open `BikeSafetyBelt.ino`). **All settings are in `config.h`.** |
| `firmware/BikeSafetyBelt/src/sensor/` | MPU-6050 driver, calibration, sensor fusion, test-scenario simulator |
| `firmware/BikeSafetyBelt/src/detection/` | Multi-indicator crash detector with temporal logic |
| `firmware/BikeSafetyBelt/src/core/` | State machine, shared types |
| `firmware/BikeSafetyBelt/src/hardware/` | LEDs, buzzer, debounced button |
| `firmware/BikeSafetyBelt/src/communication/` | USB-serial and BLE transports, line protocol |
| `firmware/BikeSafetyBelt/src/emergency/` | Countdown, SOS delivery and retry |
| `firmware/BikeSafetyBelt/src/gps/`, `storage/` | Optional GPS (NMEA), EEPROM persistence |
| `app/` | Safety dashboard (plain HTML/JS, no build step) |
| `tests/native/` | PC test suite that runs the real firmware code with fake hardware |
| `docs/` | Setup, wiring, algorithm, protocol, troubleshooting |

## Quick start

1. **Board**: use an **Arduino Nano ESP32** (recommended: same `D3/D8/D9/D10` pin labels as the prototype, BLE built in), any ESP32 board, or an Arduino Mega 2560. A classic Uno/Nano (ATmega328P, 32 KB) **is too small** (the firmware needs ~64 KB) and the build stops with a clear message. See [docs/SETUP.md](docs/SETUP.md).
2. Wire it per [docs/WIRING.md](docs/WIRING.md) (prototype pins kept: red LED D8, green LED D9, buzzer D10, button D3 to GND).
3. Set `MOUNT_FORWARD_AXIS` / `MOUNT_UP_AXIS` and `PROFILE_DEFAULT` in `config.h`, then upload.
4. Open the Serial Monitor at **115200**. You will see `[IMU] Connected`, `[CALIBRATION] Complete`, `[STATE] SAFE`.
5. Serve the dashboard: `cd app && python3 -m http.server 8000`, then open `http://localhost:8000` in Chrome or Edge and click **Connect USB** (or **Connect Bluetooth** for ESP32). Click **Demo replay** to see the UI without hardware.
6. With the bike upright on its centre stand: **Settings → Calibrate riding orientation**.
7. Settings: add emergency contacts and an automatic alert channel (ntfy topic or webhook).

## How crashes are detected (short version)

A lean angle on its own **never** triggers an alarm, and neither does an acceleration spike on its own. Five indicators are scored from 0 to 100 % against the selected vehicle profile (scooter / standard / sport):

1. **Abnormal lean**: peak tilt from upright
2. **Angular velocity**: peak rotation rate
3. **Impact**: peak resultant acceleration `A = √(Ax²+Ay²+Az²)`, down-weighted when purely vertical (the road-bump pattern)
4. **Sudden change**: orientation change within 0.5 s, or sustained fast rotation (tumbling)
5. **Post-impact orientation**: resting tilt once the bike stops moving. It only counts after a *major* motion event.

The confidence is the weighted sum. **CRASH_DETECTED** needs confidence ≥ 65 % **and** at least 3 indicators ≥ 50 %. A candidate event starts only when 2 of the last 4 samples are anomalous, so single noisy readings are ignored. A bike that was parked and still before falling over gets reduced confidence (POSSIBLE_CRASH, no SOS). Details: [docs/ALGORITHM.md](docs/ALGORITHM.md).

Lean angle uses an **adaptive complementary filter**. While cornering, the accelerometer cannot see lean, because gravity plus centripetal force point along the bike's axis. So the filter then uses turn kinematics (`lean = atan(ωy/ωz)`) instead, and its Euler-rate integration keeps yaw out of roll and pitch. In simulated 40° turns the estimate stays within 1.5° of the true lean.

## States

`CALIBRATING → SAFE ⇄ MONITORING → POSSIBLE_CRASH → CRASH_DETECTED (10 s countdown) → SOS_SENT → RECOVERY → (explicit reset) → SAFE`, plus `USER_CANCELLED`, `MANUAL_TEST` and `IMU_FAULT`.

| State | Green | Red | Buzzer |
|---|---|---|---|
| SAFE / MONITORING | on | off | off |
| POSSIBLE_CRASH | off | blink 2 Hz | two short beeps / 1.5 s |
| CRASH_DETECTED | off | blink 5 Hz | beep each second, rapid in last 3 s |
| SOS_SENT | off | Morse SOS | Morse SOS |
| RECOVERY | off | flash every 2 s | chirp every 20 s |
| MANUAL_TEST | off | on | on (continuous) |
| IMU_FAULT (error) | alternates with red | alternates | long beep, then chirp every 30 s |

**Button (D3), labelled TEST / MANUAL TRIGGER:** the first press starts the test alarm (never an SOS) and the second press clears it. During a possible crash or countdown, a press means *"I'm OK"* and cancels. After an SOS, **hold for 3 s** to reset.

## Hardware honesty

- **No GPS module** is fitted by default (`GPS_ENABLED 0`), so the system reports **LOCATION UNAVAILABLE**, never made-up coordinates. Optionally, the app can use the phone's own location, labelled as such.
- **No GSM/SMS module**: the device cannot send SMS. The SOS travels over the app link. The app sends it automatically through ntfy.sh or your webhook, or offers a prefilled SMS that someone has to tap to send.
- **No battery sensing**: battery is shown as *not available*.

## Tests

```bash
make -C tests/native        # 128 checks: unit tests + every scenario × 3 profiles end-to-end
```

The suite also regenerates `app/samples/demo-session.log` (used by **Demo replay**).

More: [Setup](docs/SETUP.md) · [Wiring](docs/WIRING.md) · [Algorithm](docs/ALGORITHM.md) · [Protocol](docs/PROTOCOL.md) · [Troubleshooting](docs/TROUBLESHOOTING.md)
