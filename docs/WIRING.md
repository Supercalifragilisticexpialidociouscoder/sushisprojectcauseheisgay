# Wiring and pins

Prototype pins are unchanged. They are set at the top of `firmware/BikeSafetyBelt/config.h`.

| Part | Board pin | Notes |
|---|---|---|
| Red LED | **D8** | D8 → 220 Ω → LED anode, cathode → GND |
| Green LED | **D9** | D9 → 220 Ω → LED anode, cathode → GND |
| Buzzer | **D10** | Active buzzer + → D10, − → GND (`BUZZER_PASSIVE 1` for a passive one) |
| Push button | **D3** | One side → D3, other side → **GND**. Internal pull-up (`INPUT_PULLUP`), pressed = LOW |
| MPU-6050 VCC | 3.3 V (or 5 V if the breakout has a regulator, e.g. GY-521) | |
| MPU-6050 GND | GND | |
| MPU-6050 SDA | Nano ESP32: **A4** · Mega: **20** · ESP32 DevKit: **21** | |
| MPU-6050 SCL | Nano ESP32: **A5** · Mega: **21** · ESP32 DevKit: **22** | |
| MPU-6050 AD0 | GND → address 0x68 (VCC → 0x69, then set `MPU_I2C_ADDRESS`) | |

```
            ┌──────────────────────┐
  GND ──┬───┤ GND          D8  ├──220Ω──►|── GND   (red)
        │   │              D9  ├──220Ω──►|── GND   (green)
        │   │              D10 ├──── buzzer(+)  buzzer(−) ── GND
        └─[button]──────── D3  │
            │              SDA ├──── MPU-6050 SDA
            │              SCL ├──── MPU-6050 SCL
            │              3V3 ├──── MPU-6050 VCC
            └──────────────────┘
```

## Board notes

- **Arduino Nano ESP32** (recommended). Same Nano footprint and `D3/D8/D9/D10` labels as the prototype. In the IDE, keep *Tools → Pin Numbering → By Arduino pin (default)*. It is a 3.3 V board, so power the MPU-6050 from 3V3.
- **Generic ESP32 DevKit.** GPIO 6–11 are connected to the flash chip, so pins 8, 9 and 10 **cannot** be used. The build stops with an error until you set free pins in `config.h`, for example `PIN_RED_LED 25`, `PIN_GREEN_LED 26`, `PIN_BUZZER 27`, `PIN_BUTTON 33`.
- **Arduino Mega 2560.** The pins work as-is (5 V board). No Bluetooth, so connect the app over USB.
- **Arduino Uno / Nano (ATmega328P).** Not supported: 32 KB of flash is too small (the firmware is ~64 KB).

## Mounting

- Mount the MPU-6050 **rigidly** to the frame (not to the handlebars, and not on foam). Vibration isolation hides impacts.
- Set `MOUNT_FORWARD_AXIS` (the axis printed on the board that points to the front of the bike) and `MOUNT_UP_AXIS` (the axis that points to the sky). The sensor does **not** need to be mounted level: run **Calibrate riding orientation** once with the bike upright on its centre stand.

## Optional GPS (not fitted by default)

NEO-6M or similar: GPS TX → `GPS_RX_PIN`, GPS RX → `GPS_TX_PIN`, then set `GPS_ENABLED 1`. On ESP32 this uses `Serial2`; on AVR it uses SoftwareSerial.

## Optional battery sense

Connect a voltage divider to an ADC pin, then set `BATTERY_ADC_PIN`, `BATTERY_DIVIDER` and `BATTERY_ADC_REF_MV`.
