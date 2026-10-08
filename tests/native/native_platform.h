#pragma once
#include <stdint.h>
#include "src/core/platform.h"

extern uint64_t g_us;     // virtual time in microseconds
extern int g_pin[64];     // last written output level per pin
extern int g_pinIn[64];   // input level per pin (1 = HIGH)
