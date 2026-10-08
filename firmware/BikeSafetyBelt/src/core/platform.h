// Platform abstraction: lets the same core code build for Arduino (AVR / ESP32)
// and natively on a PC for the automated test suite (tests/native).
#pragma once

#include <stdint.h>
#include <stddef.h>
#include <math.h>

#ifdef ARDUINO
  #include <Arduino.h>
#else
  // Minimal Arduino API, implemented by tests/native/native_platform.cpp
  uint32_t millis();
  uint32_t micros();
  void pinMode(uint8_t pin, uint8_t mode);
  void digitalWrite(uint8_t pin, uint8_t val);
  int digitalRead(uint8_t pin);
  int analogRead(uint8_t pin);
  void tone(uint8_t pin, unsigned int freq);
  void noTone(uint8_t pin);
  #ifndef HIGH
    #define HIGH 1
    #define LOW 0
    #define INPUT 0
    #define OUTPUT 1
    #define INPUT_PULLUP 2
  #endif
#endif

// --- Strings kept in flash on AVR (2 KB RAM), plain const char* elsewhere ----
#if defined(ARDUINO_ARCH_AVR)
  #include <avr/pgmspace.h>
  typedef const __FlashStringHelper* FStr;
  #define FS(s) F(s)
  inline char fsRead(FStr s, size_t i) { return (char)pgm_read_byte(reinterpret_cast<const char*>(s) + i); }
  #define BSB_PROGMEM PROGMEM
  inline uint16_t bsbReadU16(const uint16_t* p) { return pgm_read_word(p); }
  #define BSB_BOARD_NAME "AVR"
#else
  typedef const char* FStr;
  #define FS(s) (s)
  inline char fsRead(FStr s, size_t i) { return s[i]; }
  #define BSB_PROGMEM
  inline uint16_t bsbReadU16(const uint16_t* p) { return *p; }
  #if defined(ARDUINO_ARCH_ESP32)
    #define BSB_BOARD_NAME "ESP32"
  #elif defined(ARDUINO)
    #define BSB_BOARD_NAME "ARDUINO"
  #else
    #define BSB_BOARD_NAME "NATIVE"
  #endif
#endif

// Wrap-safe "has `interval` elapsed since `since`" for millis()/micros().
inline bool elapsed(uint32_t now, uint32_t since, uint32_t interval) {
  return (uint32_t)(now - since) >= interval;
}
