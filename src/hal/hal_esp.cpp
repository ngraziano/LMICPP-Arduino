/*******************************************************************************
 * Copyright (c) 2015 Matthijs Kooijman
 * All rights reserved. This program and the accompanying materials
 * are made available under the terms of the Eclipse Public License v1.0
 * which accompanies this distribution, and is available at
 * http://www.eclipse.org/legal/epl-v10.html
 *
 * This the HAL to run LMIC on top of the Arduino environment.
 *******************************************************************************/

#ifdef ARDUINO_ARCH_ESP32
#include "hal.h"
#include <stdio.h>
#include "print_debug.h"
#include <sys/time.h>

// -----------------------------------------------------------------------------
// TIME

OsTime hal_ticks() {
  timeval val;
  gettimeofday(&val, nullptr);
  return OsTime((val.tv_sec * OSTICKS_PER_SEC) + (val.tv_usec >> US_PER_OSTICK_EXPONENT));
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

DisableIRQsGard::DisableIRQsGard()  { noInterrupts(); }
DisableIRQsGard::~DisableIRQsGard() { interrupts(); }

void hal_init() {
  // printf support
  hal_printf_init();
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