/*******************************************************************************
 * Copyright (c) 2015 Matthijs Kooijman
 * All rights reserved. This program and the accompanying materials
 * are made available under the terms of the Eclipse Public License v1.0
 * which accompanies this distribution, and is available at
 * http://www.eclipse.org/legal/epl-v10.html
 *
 * This the HAL to run LMIC on top of the Arduino environment.
 *******************************************************************************/

#include "../boardconfig.h"
#if LMIC_HAL == LMIC_ARDUINO
#include "hal.h"
#include "print_debug.h"
#include <Arduino.h>
#include <cstdio>

// -----------------------------------------------------------------------------
// TIME

namespace {
OsDeltaTime time_in_sleep{0};
uint8_t overflow{0};
} // namespace

void hal_add_time_in_sleep(OsDeltaTime nb_tick) {
  time_in_sleep += nb_tick;
  hal_ticks();
}

OsTime hal_ticks() {
  // hal_ticks is use in interrupt and is not reentrant
  DisableIRQsGard gard;

  // Because micros() is scaled down in this function, micros() will
  // overflow before the tick timer should, causing the tick timer to
  // miss a significant part of its values if not corrected. To fix
  // this, the "overflow" serves as an overflow area for the micros()
  // counter. It consists of three parts:
  //  - The US_PER_OSTICK upper bits are effectively an extension for
  //    the micros() counter and are added to the result of this
  //    function.
  //  - The next bit overlaps with the most significant bit of
  //    micros(). This is used to detect micros() overflows.
  //  - The remaining bits are always zero.
  //
  // By comparing the overlapping bit with the corresponding bit in
  // the micros() return value, overflows can be detected and the
  // upper bits are incremented. This is done using some clever
  // bitwise operations, to remove the need for comparisons and a
  // jumps, which should result in efficient code. By avoiding shifts
  // other than by multiples of 8 as much as possible, this is also
  // efficient on AVR (which only has 1-bit shifts).

  // Scaled down timestamp. The top US_PER_OSTICK_EXPONENT bits are 0,
  // the others will be the lower bits of our return value.
  uint32_t scaled = micros() >> US_PER_OSTICK_EXPONENT;
  // Most significant byte of scaled
  uint8_t msb = scaled >> 24;
  // Mask pointing to the overlapping bit in msb and overflow.
  const uint8_t mask = (1 << (7 - US_PER_OSTICK_EXPONENT));
  // Update overflow. If the overlapping bit is different
  // between overflow and msb, it is added to the stored value,
  // so the overlapping bit becomes equal again and, if it changed
  // from 1 to 0, the upper bits are incremented.
  overflow += (msb ^ overflow) & mask;

  // Return the scaled value with the upper bits of stored added. The
  // overlapping bit will be equal and the lower bits will be 0, so
  // bitwise or is a no-op for them.
  return OsTime((scaled | ((uint32_t)overflow << 24)) + time_in_sleep.tick());

  // 0 leads to correct, but overly complex code (it could just return
  // micros() unmodified), 8 leaves no room for the overlapping bit.
  static_assert(US_PER_OSTICK_EXPONENT > 0 && US_PER_OSTICK_EXPONENT < 8,
                "Invalid US_PER_OSTICK_EXPONENT value");
}

void hal_waitUntil(OsTime time) {
  OsDeltaTime delta = time - hal_ticks();
  hal_wait(delta);
}

void hal_wait(OsDeltaTime delta) {
  // From delayMicroseconds docs: Currently, the largest value that
  // will produce an accurate delay is 16383.
  while (delta > OsDeltaTime::from_us(16000)) {
    delay(16);
    delta -= OsDeltaTime::from_us(16000);
  }
  
  if (delta > OsDeltaTime(0))
    delayMicroseconds(delta.to_us());
}

#ifdef __AVR__
DisableIRQsGard::DisableIRQsGard() : sreg_save(SREG) { cli(); }
DisableIRQsGard::~DisableIRQsGard() { SREG = sreg_save; }
#else
DisableIRQsGard::DisableIRQsGard() {
  noInterrupts();
  ++intNumber;
}
DisableIRQsGard::~DisableIRQsGard() {
  if (--intNumber == 0) {
    interrupts();
    interrupts();
  }
}
uint8_t DisableIRQsGard::intNumber = 0;
#endif

// -----------------------------------------------------------------------------

void hal_init() {
  if constexpr(debugLevel > 0) {
    // printf support
    hal_printf_init();
  }
}

void hal_failed(const char *file, uint16_t line) {
  (void)file;
  (void)line;
#if defined(LMIC_FAILURE_TO)
  LMIC_FAILURE_TO.println("FAILURE ");
  LMIC_FAILURE_TO.print(file);
  LMIC_FAILURE_TO.print(':');
  LMIC_FAILURE_TO.println(line);
  LMIC_FAILURE_TO.flush();
#endif
  DisableIRQsGard irqguard;

  while (1)
    ;
}
#endif
