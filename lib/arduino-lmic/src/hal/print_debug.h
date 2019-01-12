#ifndef _print_debug_h_
#define _print_debug_


#include "hal.h"
#include "WString.h"
#include "stdio.h"


template <typename... T>
void PRINT_DEBUG(int X, const __FlashStringHelper *str, T... div) {
  if (debugLevel >= X) {
    printf_P(PSTR("%lu "), hal_ticks().tick());
    PGM_P p = reinterpret_cast<PGM_P>(str);
    printf_P(p, div...);
    //printf_P(PSTR("\n"));
    printf("\n");
  }
};


void hal_printf_init();

#endif