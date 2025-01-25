#ifdef ARDUINO_ARCH_ESP32
#include "lmicrand.h"

#include <esp_system.h>
#include <esp_random.h>

// return next random from esp system random generator
uint8_t LmicRand::uint8() { return esp_random() & 0xFF; }

//! Get random number .
uint16_t LmicRand::uint16() { return esp_random() & 0xFFFF; }

#endif
