#ifndef hal_print_debug_h
#define hal_print_debug_h

#include "hal.h"
#include "stdio.h"
#include <cinttypes>

#ifdef ARDUINO

#include "WString.h"
#include <Arduino.h>

#ifdef ARDUINO_ARCH_AVR
template <typename... T>
void PRINT_DEBUG(int X, const __FlashStringHelper *str, T const... div) {
  if (debugLevel >= X) {
    printf_P(PSTR("%" PRIu32 " "), hal_ticks().tick());
    PGM_P p = reinterpret_cast<PGM_P>(str);
    printf_P(p, div...);
    // printf_P(PSTR("\n"));
    printf("\n");
  }
}
#else

namespace {
template <typename... T>
void serialPrintf(const __FlashStringHelper *ifsh, T const... div) {
  // Cast the special class use by AVR to standart char *
  LMIC_PRINTF_TO.printf(reinterpret_cast<char const *>(ifsh), div...);
}

} // namespace

template <typename... T>
void PRINT_DEBUG(int X, const __FlashStringHelper *str, T const... div) {
  if (debugLevel >= X) {
    serialPrintf(F("%" PRIu32 " "), hal_ticks().tick());
    serialPrintf(str, div...);
    serialPrintf(F("\n"));
  }
}

#endif

#else
#define F(string_literal) string_literal

__attribute__((format(printf, 2, 3)))
void PRINT_DEBUG(int X, const char *str, ...);

#endif

constexpr bool IS_DEBUG_ENABLE(int x) { return debugLevel >= x; }

void hal_printf_init();

#endif