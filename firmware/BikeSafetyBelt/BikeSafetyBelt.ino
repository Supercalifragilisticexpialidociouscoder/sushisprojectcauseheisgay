// =============================================================================
//  Bike Safety Belt — rider safety firmware (MPU-6050 crash detection)
// =============================================================================
//  ALL pins, thresholds and settings live in config.h (pins at the very top).
//  The code is split into modules under src/:
//    sensor/        MPU-6050 driver, calibration, sensor fusion, test simulator
//    detection/     multi-indicator crash detector (temporal logic)
//    core/          state machine, shared types
//    hardware/      LEDs, buzzer, button
//    communication/ USB serial + BLE transports, line protocol
//    emergency/     countdown + SOS delivery
//    gps/           optional NMEA GPS parser
//    storage/       EEPROM persistence (calibration, incidents, restart recovery)
//    app/           glue: non-blocking scheduler
//
//  IMPORTANT: this is a safety PROTOTYPE. It cannot guarantee that an accident
//  is detected, nor that an emergency alert is delivered.
// =============================================================================
#include "config.h"
#include <Wire.h>

#include "src/app/bike_safety_app.h"
#include "src/sensor/mpu6050.h"
#include "src/communication/serial_transport.h"
#include "src/communication/ble_transport.h"
#include "src/storage/persistent_store.h"

#if defined(__AVR_ATmega328P__) || defined(__AVR_ATmega168__) || defined(__AVR_ATmega32U4__)
#error "Uno / Nano / Leonardo have only 32 KB flash; this firmware needs ~64 KB. Use an Arduino Nano ESP32 (same D3/D8/D9/D10 pins, BLE built in), an ESP32 board, or an Arduino Mega 2560. See docs/SETUP.md."
#endif

#if defined(CONFIG_IDF_TARGET_ESP32)
// Classic ESP32: GPIO 6..11 are wired to the SPI flash; using them crashes the board.
#define BSB_FLASH_PIN(p) ((p) >= 6 && (p) <= 11)
static_assert(!BSB_FLASH_PIN(PIN_RED_LED) && !BSB_FLASH_PIN(PIN_GREEN_LED) && !BSB_FLASH_PIN(PIN_BUZZER) &&
                  !BSB_FLASH_PIN(PIN_BUTTON),
              "Classic ESP32: GPIO 6-11 belong to the flash chip. Set PIN_* in config.h (e.g. 25, 26, 27, 33).");
#endif

static BikeSafetyApp app;
static Mpu6050 imu(MPU_I2C_ADDRESS);
static SerialTransport usb;
static EepromBackend eeprom;
#if BSB_HAS_BLE
static BleTransport ble(app.deviceName());   // name is filled in by app.begin()
#endif

#if GPS_ENABLED
  #if defined(ARDUINO_ARCH_ESP32)
    static HardwareSerial& gpsSerial = Serial2;
  #else
    #include <SoftwareSerial.h>
    static SoftwareSerial gpsSerial(GPS_RX_PIN, GPS_TX_PIN);
  #endif
class GpsStream : public CharSource {
 public:
  int read() override { return gpsSerial.available() > 0 ? gpsSerial.read() : -1; }
};
static GpsStream gpsStream;
#endif

static uint16_t entropySeed() {
#if defined(ARDUINO_ARCH_ESP32)
  const uint64_t mac = ESP.getEfuseMac();
  return (uint16_t)(mac ^ (mac >> 16) ^ (mac >> 32));
#else
  uint16_t s = (uint16_t)micros();
  for (uint8_t i = 0; i < 16; ++i) s = (uint16_t)((s << 1) ^ (analogRead(A0) & 1) ^ (analogRead(A1) & 1));
  return s ? s : 0xBEEF;
#endif
}

void setup() {
#if defined(ARDUINO_ARCH_ESP32)
  if (PIN_I2C_SDA >= 0 && PIN_I2C_SCL >= 0) Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL);
  else Wire.begin();
  Wire.setTimeOut((uint16_t)(MPU_I2C_TIMEOUT_US / 1000UL + 1));
#else
  Wire.begin();
  #if defined(WIRE_HAS_TIMEOUT)
  Wire.setWireTimeout(MPU_I2C_TIMEOUT_US, true);   // a hung I2C bus must never freeze the loop
  #endif
#endif
  Wire.setClock(MPU_I2C_CLOCK_HZ);

  eeprom.begin();

#if GPS_ENABLED
  #if defined(ARDUINO_ARCH_ESP32)
  gpsSerial.begin(GPS_BAUD, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);
  #else
  gpsSerial.begin(GPS_BAUD);
  #endif
  CharSource* gps = &gpsStream;
#else
  CharSource* gps = nullptr;
#endif

#if BSB_HAS_BLE
  Transport* second = &ble;
#else
  Transport* second = nullptr;
#endif
  app.begin(&imu, &eeprom, &usb, second, gps, entropySeed());
}

void loop() { app.loop(); }
