# Device ↔ app protocol

The same protocol runs over USB serial (115200 8N1) and BLE (Nordic UART Service): newline-terminated text lines.

- Lines starting with `[` are human-readable logs (USB only).
- Lines starting with `{` are JSON messages. They are sent **only while an app session is active**: the app sends `HELLO`, then `PING` at least every 4 s. A plain serial monitor therefore sees clean logs.

## App → device

| Command | Effect |
|---|---|
| `HELLO` | start a session; the device replies `hello`, `cfg`, `state` (and any pending `sos`) |
| `PING` | heartbeat (resumes the session after a link loss) |
| `CANCEL` | rider is OK: cancels POSSIBLE_CRASH or the countdown; clears MANUAL_TEST |
| `RESET` | explicit recovery after an SOS |
| `SOSACK <id>` | app received SOS `<id>`; the device stops re-sending |
| `BTN` | emulates a press of the test button |
| `PROFILE scooter\|standard\|sport` | change the vehicle profile (stored) |
| `CAL LEVEL` / `CAL GYRO` / `CAL CLEAR` | calibration (state SAFE only) |
| `SIM <n>` / `SIM STOP` | run a test scenario (1–7, 9, 14–16) |
| `COMMS <s>` | communication-failure test: pause JSON for s seconds |
| `GPSLOSS <s>` | GPS-unavailable test |
| `LOG` / `CLEARLOG` | dump / clear the incidents stored on the device |
| `STATUS`, `HELP` | human-readable info |

## Device → app (`"t"` = type)

```jsonc
{"t":"hello","fw":"1.0.0","proto":1,"dev":"BSB-1A2B","board":"ESP32","boot":12,"gpsHw":0,"bleHw":1,"batHw":0,"cd":10,"ms":5321}
{"t":"cfg","profile":"standard","leanN":45,"leanC":65,"gyrN":120,"gyrC":280,"accN":2.2,"accC":4.5,"tcN":30,"tcC":65,"restN":20,"restC":60,"possible":30,"crash":65,"minInd":3,"accRange":16,"gyrRange":2000,"rate":100,"alpha":0.980,"mount":"+X","up":"+Z","level":1,"cal":1}
{"t":"tel","ms":..,"st":"SAFE","r":12.4,"p":-1.2,"tl":12.5,"a":1.02,"ap":1.10,"g":4,"gp":10,"c":3,"ph":0,"imu":1,"cal":1,"lvl":1,"sim":0,"cd":0,"ab":0,"arm":1,"ref":1,"bat":-1,"gps":0}
{"t":"state","st":"POSSIBLE_CRASH","prev":"MONITORING","why":"crash detector","inc":12001,"type":"AUTO","cd":0,"ms":..}
{"t":"det","ev":"verdict","inc":12001,"sim":0,"conf":90,"s":[100,82,100,67,100],"act":5,"pkA":7.20,"pkG":251,"pkT":88.0,"pkR":-88.0,"dT":53.6,"sus":250,"base":35.3,"rest":84.0,"set":1,"vert":0,"park":0,"post":1,"trig":4,"verdict":"CRASH"}
{"t":"inc","id":12001,"boot":12,"up":916000,"cur":1,"type":"AUTO","out":"SOS","flags":11,"conf":90,"acc":7.20,"gyr":251,"lean":87.9,"gpsOk":0}
{"t":"sos","id":12001,"dev":"BSB-1A2B","sim":0,"conf":90,"boot":12,"up":916000,"acc":7.20,"gyr":251,"lean":87.9,"tx":1,"ms":..,"resumed":0,"gps":0}
{"t":"ack","cmd":"CANCEL","ok":1}
```

- `tel.r/p/tl/a/g` are `null` when the IMU is not delivering valid data.
- `gps`: 0 = no GPS module, 1 = module present but no valid fix, 2 = valid fix (adds `lat`, `lon`, `hacc` = estimated accuracy in m, `sat`, and `utc` in `sos`).
- Incident `flags`: 0x01 suspected, 0x02 crash detected, 0x04 cancelled, 0x08 SOS sent, 0x10 SOS acknowledged by app, 0x20 GPS valid, 0x40 resumed after restart, 0x80 IMU lost during incident.
- Incident ids are `boot × 1000 + sequence`, so they are unique across restarts.
