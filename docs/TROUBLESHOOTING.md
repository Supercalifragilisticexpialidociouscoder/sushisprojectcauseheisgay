# Troubleshooting

| Symptom | Likely cause | Fix |
|---|---|---|
| Build error *"Uno / Nano / Leonardo have only 32 KB flash"* | ATmega328P board selected | Use an Arduino Nano ESP32, an ESP32 board or a Mega 2560 |
| Build error *"Classic ESP32: GPIO 6-11 belong to the flash chip"* | Generic ESP32 with prototype pins 8/9/10 | Set free pins in `config.h` (e.g. 25/26/27/33) |
| `[IMU] NOT FOUND` / `IMU DISCONNECTED` | Wiring, wrong I²C pins, no power, wrong address | Check VCC/GND/SDA/SCL. If AD0 is tied high, set `MPU_I2C_ADDRESS 0x69`. Keep I²C wires short; for long runs drop `MPU_I2C_CLOCK_HZ` to 100000 |
| `WHO_AM_I 0x70/0x72/0x98 (compatible clone)` | Clone chip | Usually fine. Check that `[CALIBRATION] Complete` reports gravity ≈ 1.000 g |
| `[IMU] DISCONNECTED (frozen data)` | Sensor stuck or browned out | Check the power supply; the firmware re-initialises automatically every 2 s |
| `Movement detected - restarting` keeps repeating | Bike moving or engine vibration during boot | Keep the bike still. After 5 attempts it falls back to the last saved calibration (*degraded*); later send `CAL GYRO` while still |
| Roll/pitch show 90° when the bike is upright | Mount axes wrong | Fix `MOUNT_FORWARD_AXIS` / `MOUNT_UP_AXIS`, then **Calibrate riding orientation** |
| Lean has the wrong sign (left shows R) | Forward axis reversed | Swap the `MOUNT_FORWARD_AXIS` sign (e.g. `AXIS_POS_X` → `AXIS_NEG_X`) |
| *Level calibration rejected — sensor is NN° off* | Mount axes misconfigured (> 45° off) | Fix the axis settings first |
| False POSSIBLE_CRASH on rough roads | Profile too strict | Switch to the matching profile, or raise `impactNormal`/`gyroNormal` in `config.h` section 7 |
| No alarm in a deliberate test tip-over of a **parked** bike | Parked-vehicle guard (still > 30 s before the fall) | By design: POSSIBLE_CRASH only. Use `SIM 7` to test the countdown |
| No new detection after a cancelled alarm | Detector re-arms only after the bike has been upright for 3 s | Lift the bike upright |
| Serial Monitor shows only `[..]` lines, no JSON | Normal: JSON is sent only while the app is connected (HELLO/PING) | — |
| Dashboard: *Web Serial is not supported* | Firefox, Safari or a mobile browser | Use Chrome or Edge on a computer, or BLE in Chrome on Android |
| Dashboard: **DEVICE OFFLINE** while connected | No data for 3 s: cable, BLE range, device reset, or the `COMMS` test | Reconnect; the device resumes automatically on the next PING |
| Connect Bluetooth finds nothing | Not an ESP32 build, or already connected elsewhere | Only ESP32 builds advertise `BSB-XXXX`; close other BLE apps |
| SOS screen: *NO AUTOMATIC DELIVERY CONFIGURED* | No ntfy topic or webhook set | Settings → Automatic alert delivery → Send test alert |
| Webhook *FAILED (Failed to fetch)* | CORS or HTTPS problem on your server | The endpoint must allow CORS `POST` from the dashboard origin |
| **LOCATION UNAVAILABLE** | No GPS module (default) or no fix | Expected without GPS. Enable *Use this phone's location*, or fit a GPS module and set `GPS_ENABLED 1` |
| After a power cut the device starts in CRASH_DETECTED or RECOVERY | Restart during an active incident (by design) | Cancel (countdown) or hold the button 3 s (recovery) |
| Buzzer silent | Passive buzzer used as active | Set `BUZZER_PASSIVE 1` |
| Buzzer sounds weak or clicks | Active buzzer driven with tone | Set `BUZZER_PASSIVE 0` |
