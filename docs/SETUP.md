# Setup

## 1. Firmware

1. Install the Arduino IDE 2.x.
2. Add board support:
   - **Arduino Nano ESP32**: *Boards Manager → "Arduino ESP32 Boards"*.
   - Other ESP32 boards: *Boards Manager → "esp32 by Espressif"*.
   - Mega 2560: *Arduino AVR Boards* (built in).
3. Open `firmware/BikeSafetyBelt/BikeSafetyBelt.ino`. No extra libraries are needed: the MPU-6050 driver, NMEA parser and BLE code are included (BLE uses the ESP32 core's built-in library).
4. Edit `config.h`:
   - **Section 1, pins.** The prototype defaults are kept. On a generic ESP32 DevKit, change them (see WIRING.md).
   - **Section 4, mounting**: `MOUNT_FORWARD_AXIS`, `MOUNT_UP_AXIS`.
   - **Section 7, vehicle**: `PROFILE_DEFAULT` = `PROFILE_SCOOTER`, `PROFILE_STANDARD` or `PROFILE_SPORT`. You can also change it later from the app.
   - Leave `GPS_ENABLED 0` unless a GPS module is actually connected.
5. Select the board and port, then **Upload**.
6. Open the Serial Monitor at **115200 baud**. Keep the bike still for about 3 s while it calibrates:

```
[BOOT] Bike Safety Belt firmware 1.0.0 (ESP32)
[IMU] Connected (WHO_AM_I 0x68)
[CALIBRATION] Keep the bike still - measuring sensor offsets...
[CALIBRATION] Complete - gyro bias (0.52, -1.10, 0.33) deg/s, gravity 1.000 g
[STATE] SAFE (calibration complete)
[IMU] Roll: 0.4° | Pitch: -1.2° | Acceleration: 1.00g | Angular velocity: 0°/s
[DETECTOR] Crash confidence: 0% | State: SAFE
```

Type `HELP` in the Serial Monitor (line ending: *Newline*) for the commands.

## 2. Dashboard

The dashboard is the static files in `app/`, with no build step. Web Serial and Web Bluetooth only work on secure pages (`https://` or `http://localhost`), which GitHub Pages provides.

### GitHub Pages (recommended)

1. Repo **Settings → Pages → Build and deployment → Source: GitHub Actions** (one time).
2. Merge to `main`, or run the *Deploy dashboard to GitHub Pages* workflow manually (Actions tab). The workflow runs the firmware test-suite, regenerates the demo data, and publishes `app/`.
3. Open <https://supercalifragilisticexpialidociouscoder.github.io/sushisprojectcauseheisgay/>.

On GitHub Pages the dashboard:
- **works offline** after the first visit (service worker), so it still opens with no mobile data. Sending alerts still needs internet.
- **can be installed** as an app: Chrome menu → *Install app* / *Add to Home screen*.
- **keeps the screen on** while a device is connected (Wake Lock), because the SOS path runs through this page.
- shows a warning banner and disables the button when the browser cannot use USB or Bluetooth.

| Device / browser | USB (Connect USB) | Bluetooth (Connect Bluetooth, ESP32 only) |
|---|---|---|
| Chrome / Edge on Windows, macOS, Linux, ChromeOS | yes | yes |
| Chrome on Android | no | yes |
| Firefox, Safari, any browser on iPhone/iPad | no | no (Demo replay only) |

The device advertises over Bluetooth as `BSB-XXXX`.

### Local

```bash
cd app
python3 -m http.server 8000
# open http://localhost:8000 in Chrome or Edge
```

**Demo replay** plays `app/samples/demo-session.log` (a recorded session that includes a simulated crash), so you can explore the UI without hardware.

## 3. First-time configuration (Settings tab)

1. **Rider** name and notes for responders.
2. **Emergency contacts**: name and phone.
3. **Automatic alert delivery**. Set at least one:
   - **ntfy.sh topic**: pick a hard-to-guess topic name; each contact installs the *ntfy* app and subscribes to it. Alerts are sent with urgent priority.
   - **Webhook URL**: receives a JSON `POST` (incident, location, contacts, message). Use it with your own server, Home Assistant, or a Twilio Function that sends SMS.
   - Use **Send test alert** to check delivery.
   - With no channel configured, the SOS screen says **NO AUTOMATIC DELIVERY CONFIGURED** and offers *Send SMS to contacts* (opens the SMS app with the message prefilled; someone has to tap Send).
4. Optional: **Use this phone's location** when the device has no GPS fix. It is labelled "Phone location", with the accuracy the phone reports.
5. **Vehicle profile**.
6. **Calibrate riding orientation**: bike upright on the centre stand, nobody touching it.

Keep the dashboard open, with the screen on, while riding: the SOS path depends on it.

## 4. Testing without crashing (Test mode tab)

Every test runs on the device. The simulator replaces the sensor input and the real fusion, detector and state machine process it. Simulated SOS alerts are never sent to contacts.

| # | Test | Command | Expected |
|---|---|---|---|
| 1 | Stationary | `SIM 1` | SAFE |
| 2 | Normal lean | `SIM 2` | SAFE |
| 3 | Cornering | `SIM 3` | SAFE |
| 4 | Sudden braking | `SIM 4` | SAFE |
| 5 | Speed breaker | `SIM 5` | MONITORING briefly, then cleared |
| 6 | Sudden rotation | `SIM 6` | MONITORING briefly, then cleared |
| 7 | Crash-like motion | `SIM 7` | POSSIBLE_CRASH → CRASH_DETECTED |
| 8 | Manual button | press D3, or `BTN` | MANUAL_TEST, second press → SAFE |
| 9 | Sensor disconnect | `SIM 9` (or unplug SDA) | IMU_FAULT → SAFE |
| 10 | GPS unavailable | `GPSLOSS 30` | LOCATION UNAVAILABLE |
| 11 | Communication failure | `COMMS 8` | app shows DEVICE OFFLINE, then recovers |
| 12 | Countdown cancel | `SIM 7`, then I'M OKAY | USER_CANCELLED → SAFE |
| 13 | Countdown expiry | `SIM 7`, then wait | SOS_SENT → RECOVERY |
| + | Pothole / parked tip-over / high-side | `SIM 14` / `15` / `16` | cleared / POSSIBLE only / crash |

## 5. Native test suite (PC)

```bash
make -C tests/native
```

This compiles the firmware modules with `g++` and runs 128 checks: unit tests, every scenario for all three profiles, cancel, expiry, single SOS with retry until acknowledged, IMU loss, restart during a countdown or after an SOS, and the communication-failure test.
