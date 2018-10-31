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





/*
 * initialize hardware (IO, SPI, TIMER, IRQ).
 */
void hal_init(void);


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
 * return system time.
 */
OsTime hal_ticks();

void hal_add_time_in_sleep(OsDeltaTime nb_tick);

bool hal_is_sleep_allow();

void hal_allow_sleep();

void hal_forbid_sleep();

/*
 * busy-wait until specified timestamp is reached.
 */
void hal_waitUntil(OsTime time);

/*
 * wait this interval.
 */ 
void hal_wait(OsDeltaTime time);

/*
 * check and rewind timer for target time.
 *   - return 1 if target time is close
 *   - otherwise rewind timer for target time or full period and return 0
 */
bool hal_checkTimer(OsTime targettime);

/*
 * perform fatal failure action.
 *   - called by assertions
 *   - action could be HALT or reboot
 */
void hal_failed(const char *file, uint16_t line);


void hal_store_trigger();

#endif // _hal_hal_h_
