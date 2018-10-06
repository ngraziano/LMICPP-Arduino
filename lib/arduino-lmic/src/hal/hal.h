/*******************************************************************************
 * Copyright (c) 2015 Matthijs Kooijman
 * All rights reserved. This program and the accompanying materials
 * are made available under the terms of the Eclipse Public License v1.0
 * which accompanies this distribution, and is available at
 * http://www.eclipse.org/legal/epl-v10.html
 *
 * This the HAL to run LMIC on top of the Arduino environment.
 *******************************************************************************/
#ifndef _hal_hal_h_
#define _hal_hal_h_

#include <stdbool.h>
#include <stdint.h>

#include "../lmic/osticks.h"

class Lmic;
class Radio;

static const uint8_t NUM_DIO = 2;

struct lmic_pinmap {
  uint8_t nss;
  uint8_t rxtx;
  uint8_t rst;
  uint8_t dio[NUM_DIO];
};

// Use this for any unused pins.
const uint8_t LMIC_UNUSED_PIN = 0xff;

// Declared here, to be defined an initialized by the application
extern const lmic_pinmap lmic_pins;

/*
 * initialize hardware (IO, SPI, TIMER, IRQ).
 */
void hal_init(void);

/*
 * intialize random generator
 */
void hal_init_random(Radio &radio);

/*
 * random number (8bit)
 */
uint8_t hal_rand1();

/*
 * random number (16bit)
 */
uint16_t hal_rand2();

/*
 * drive radio NSS pin (0=low, 1=high).
 */
void hal_pin_nss(uint8_t val);

/*
 * drive radio RX/TX pins (0=rx, 1=tx).
 */
void hal_pin_rxtx(uint8_t val);

/*
 * control radio RST pin (0=low, 1=high, 2=floating)
 */
void hal_pin_rst(uint8_t val);

/*
 * perform 8-bit SPI transaction with radio.
 *   - write given byte 'outval'
 *   - read byte and return value
 */
uint8_t hal_spi(uint8_t outval);

/*
 * disable all CPU interrupts.
 *   - might be invoked nested
 *   - will be followed by matching call to hal_enableIRQs()
 */
void hal_disableIRQs(void);

/*
 * enable CPU interrupts.
 */
void hal_enableIRQs(void);

/*
 * check "interrupt" pin
 */
void hal_io_check(Lmic &lmic);

/*
 * return system time.
 */
OsTime hal_ticks();

void hal_add_time_in_sleep(OsDeltaTime const &nb_tick);

bool hal_is_sleep_allow();

void hal_allow_sleep();

void hal_forbid_sleep();

/*
 * busy-wait until specified timestamp is reached.
 */
void hal_waitUntil(OsTime const &time);

/*
 * wait this interval.
 */ 
void hal_wait(OsDeltaTime time);

/*
 * check and rewind timer for target time.
 *   - return 1 if target time is close
 *   - otherwise rewind timer for target time or full period and return 0
 */
bool hal_checkTimer(OsTime const &targettime);

/*
 * perform fatal failure action.
 *   - called by assertions
 *   - action could be HALT or reboot
 */
void hal_failed(const char *file, uint16_t line);


void hal_store_trigger();

#endif // _hal_hal_h_
