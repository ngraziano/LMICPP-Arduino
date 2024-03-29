/*******************************************************************************
 * All rights reserved. This program and the accompanying materials
 * are made available under the terms of the Eclipse Public License v1.0
 * which accompanies this distribution, and is available at
 * http://www.eclipse.org/legal/epl-v10.html
 *
 * This the HAL to run LMIC on other environment than arduino.
 *******************************************************************************/
#include "../boardconfig.h"
#if LMIC_HAL == LMIC_GENERIC

#include "hal.h"
#include "print_debug.h"
#include <cstdio>
#include <sys/time.h>
#include <unistd.h>

namespace {
OsDeltaTime time_in_sleep{0};
} // namespace

void hal_add_time_in_sleep(OsDeltaTime nb_tick) {
  time_in_sleep += nb_tick;
  hal_ticks();
}

// -----------------------------------------------------------------------------
// TIME

OsTime hal_ticks() {
  timeval val;
  gettimeofday(&val, nullptr);
  return OsTime((val.tv_sec * OSTICKS_PER_SEC) +
                (val.tv_usec >> US_PER_OSTICK_EXPONENT)) +
         time_in_sleep;
}

void hal_waitUntil(OsTime time) {
  OsDeltaTime delta = time - hal_ticks();
  hal_wait(delta);
}

void hal_wait(OsDeltaTime delta) {
  auto us = delta.to_us();
  if (us > 0) {
    usleep(us);
  }
}

DisableIRQsGard::DisableIRQsGard() {}
DisableIRQsGard::~DisableIRQsGard() {}

void hal_init() {
  // nothing to do
}

void hal_failed(const char *file, uint16_t line) {
  (void)file;
  (void)line;

  /*
#if defined(LMIC_FAILURE_TO)
  LMIC_FAILURE_TO.println("FAILURE ");
  LMIC_FAILURE_TO.print(file);
  LMIC_FAILURE_TO.print(':');
  LMIC_FAILURE_TO.println(line);
  LMIC_FAILURE_TO.flush();
#endif
*/

  while (1)
    ;
}
#endif