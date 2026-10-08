# Detection algorithm

## Sensor pipeline (100 Hz, non-blocking)

1. **Raw read**: 14-byte burst from `0x3B`. A read is rejected on an I²C error or short read (`BUS_ERROR`), an all-0x00 or all-0xFF frame (`INVALID_DATA`, typically the sensor reset into sleep), or 25 identical frames in a row (`STUCK`; a live sensor always has LSB noise). Five consecutive failures put the system in `IMU_FAULT`; it re-initialises every 2 s.
2. **Scaling**: ±16 g → 2048 LSB/g, ±2000 °/s → 16.4 LSB/(°/s), from `MPU_ACCEL_RANGE_G` / `MPU_GYRO_RANGE_DPS`. The ±16 g range avoids clipping on impacts; a full-scale reading sets a *saturated* flag. The DLPF is 44 Hz, below the 50 Hz Nyquist limit of 100 Hz sampling, so engine and road vibration do not alias.
3. **Mount transform**: sensor axes are mapped to the vehicle frame (X forward, Y left, Z up).
4. **Startup calibration** (every boot, about 3 s): gyro bias = mean, accepted only if gyro σ ≤ 2 °/s, |mean| ≤ 25 °/s and |a| = 1 ± 0.15 g. Accelerometer offset = the component along gravity (the only part observable in a single pose), so a stationary sensor reads exactly 1 g. Up to 5 retries, then fall back to the last saved bias (*degraded*).
5. **Riding-orientation calibration** (`CAL LEVEL`, stored in EEPROM): the measured gravity direction **u** is rotated onto +Z with Rodrigues' formula, which removes any mounting tilt. The sensor never has to be mounted level.

## Sensor fusion: adaptive complementary filter

```
angle = α·(angle + euler_rate·dt) + (1 − α)·reference
```

- Gyro integration uses **ZYX Euler kinematics** (`φ̇ = p + (q sinφ + r cosφ) tanθ`, `θ̇ = q cosφ − r sinφ`), so yaw rate in a turn does not leak into roll or pitch.
- The reference is chosen sample by sample:
  - **Accelerometer** (α = 0.98) when not cornering, `||a| − 1 g| < 0.1 g` and rotation < 30 °/s.
  - **Turn kinematics** (α = 0.99) when cornering steadily. In a coordinated turn the specific force points along the bike's own Z axis, so the accelerometer reads about 0° of lean at a real 45°. The yaw rate, however, projects onto the leaned body as `q = ψ̇ sinφ`, `r = ψ̇ cosφ`, so **lean = atan(q / r)**, with no speed sensor needed.
  - **Gyro only** for impacts, braking and crash dynamics.
- **Tilt** = `acos(cosφ·cosθ)`: the angle of the bike's up axis from vertical.

In simulated cornering (40–45°, chicane) the estimated roll stays within **1.5°** of the true lean.

## Crash detector

### Temporal logic

```
IDLE ──(2 of last 4 samples anomalous)──► ACTIVE ──(calm 0.5 s, or 4 s max)──► SETTLE ──(at rest 1 s, or 5 s max)──► verdict
```

A sample counts as anomalous if |a| ≥ 1.8 g, |ω| ≥ 90 °/s, lean rate ≥ 60 °/s, or the sensor saturated. During ACTIVE the detector tracks peaks. An event with no *major motion* (bump, pothole, swerve) is cleared at the end of ACTIVE, so it is resolved in about 0.5 s. A new violent motion during SETTLE (a secondary impact) returns to ACTIVE.

### Indicators

Each indicator is scored linearly from 0 at the "normal" threshold to 1 at the "crash" threshold of the vehicle profile:

| # | Indicator | Measure | Weight |
|---|---|---|---|
| 1 | Abnormal lean | peak tilt | 0.15 |
| 2 | Angular velocity | peak \|ω\| | 0.20 |
| 3 | Impact | peak resultant \|a\|; ×0.5 if ≥ 80 % along the bike's vertical axis while upright (road bump) | 0.15 |
| 4 | Sudden change | max(tilt change in 0.5 s, time spent above the normal rotation rate) | 0.20 |
| 5 | Post-impact orientation | resting tilt; counted only after major motion (indicator 2, 3 or 4 ≥ 50 %); ×0.5 if the bike never came to rest | 0.30 |

`confidence = Σ wᵢ·sᵢ`, multiplied by **0.6 if the vehicle was parked**, meaning completely still (engine off) for ≥ 30 s before the event.

- **POSSIBLE_CRASH** when the live confidence reaches 30 %.
- **CRASH_DETECTED** when the final confidence is ≥ 65 % **and** at least 3 indicators are ≥ 50 %.

After a crash or a cancel the detector **re-arms only once the bike has been upright for 3 s**. This prevents repeated alarms while the bike lies on the ground.

### Profiles (`config.h` section 7)

| | lean N/C | gyro N/C °/s | impact N/C g | Δtilt N/C ° | rest N/C ° |
|---|---|---|---|---|---|
| Scooter | 35/60 | 100/250 | 2.0/4.0 | 25/60 | 20/55 |
| Standard | 45/65 | 120/280 | 2.2/4.5 | 30/65 | 20/60 |
| Sport | 55/72 | 150/320 | 2.5/5.0 | 35/70 | 20/60 |

### Verified behaviour (native suite, all 3 profiles)

| Scenario | Result |
|---|---|
| Stationary, side stand / walking, cornering 40–45°, braking 1 g, acceleration | SAFE |
| Speed breaker 3 g, pothole 4.4 g (vertical), emergency swerve 170 °/s | MONITORING → cleared, no alarm |
| Parked bike knocked over | POSSIBLE_CRASH only (no countdown) |
| Low-side crash, high-side crash | CRASH_DETECTED → countdown |

**These thresholds come from physics-based simulation, not from real crash data.** Tune them on the real bike: ride normally with the dashboard's console open and look at the `[DETECTOR]` lines and peak values.

## Fail-safe rules

- IMU lost: `IMU_FAULT`, never shown as SAFE. If it is lost **during** POSSIBLE_CRASH, the system escalates to CRASH_DETECTED (an impact can tear the wiring), and the rider can still cancel.
- Restart during a countdown: the countdown restarts. Restart after an SOS: the device enters RECOVERY and re-queues an unacknowledged SOS. Both use EEPROM, written one byte per loop so writes never block.
- One SOS per incident id, re-sent every 3 s until the app acknowledges it. The app de-duplicates by id.
- RECOVERY requires an explicit reset (hold the button 3 s, or RESET in the app).
- A long stall (> 50 ms) resets the fusion integration instead of integrating a large `dt`.
- An I²C bus timeout is enabled, so a hung bus cannot freeze the loop.
