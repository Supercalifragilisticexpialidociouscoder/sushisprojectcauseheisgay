// Minimal Arduino API for the native (PC) build: virtual clock + recorded pins.
#include "native_platform.h"

uint64_t g_us = 0;
int g_pin[64] = {0};
int g_pinIn[64];
static bool g_init = false;

static void initPins() {
  if (g_init) return;
  for (int i = 0; i < 64; ++i) g_pinIn[i] = 1;   // pulled up = not pressed
  g_init = true;
}

uint32_t millis() { return (uint32_t)(g_us / 1000); }
uint32_t micros() { return (uint32_t)g_us; }
void pinMode(uint8_t, uint8_t) { initPins(); }
void digitalWrite(uint8_t pin, uint8_t val) { if (pin < 64) g_pin[pin] = val; }
int digitalRead(uint8_t pin) { initPins(); return pin < 64 ? g_pinIn[pin] : 1; }
int analogRead(uint8_t) { return 512; }
void tone(uint8_t pin, unsigned int) { if (pin < 64) g_pin[pin] = 1; }
void noTone(uint8_t pin) { if (pin < 64) g_pin[pin] = 0; }
