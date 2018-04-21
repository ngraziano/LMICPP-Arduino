/*******************************************************************************
 * Copyright (c) 2015 Matthijs Kooijman
 * All rights reserved. This program and the accompanying materials
 * are made available under the terms of the Eclipse Public License v1.0
 * which accompanies this distribution, and is available at
 * http://www.eclipse.org/legal/epl-v10.html
 *
 * This the HAL to run LMIC on top of the Arduino environment.
 *******************************************************************************/

#include <Arduino.h>
#include <SPI.h>
#include "../lmic.h"
#include "hal.h"
#include <stdio.h>

// -----------------------------------------------------------------------------
// I/O

ostime_t last_int_trigger = 0;

void hal_store_trigger() {
    last_int_trigger = os_getTime();
}

static void hal_io_init () {
    // NSS and DIO0 are required, DIO1 is required for LoRa
    ASSERT(lmic_pins.nss != LMIC_UNUSED_PIN);
    ASSERT(lmic_pins.dio[0] != LMIC_UNUSED_PIN);
    ASSERT(lmic_pins.dio[1] != LMIC_UNUSED_PIN);

    pinMode(lmic_pins.nss, OUTPUT);
    if (lmic_pins.rxtx != LMIC_UNUSED_PIN)
        pinMode(lmic_pins.rxtx, OUTPUT);
    if (lmic_pins.rst != LMIC_UNUSED_PIN)
        pinMode(lmic_pins.rst, OUTPUT);

    pinMode(lmic_pins.dio[0], INPUT);
    if (lmic_pins.dio[1] != LMIC_UNUSED_PIN)
        pinMode(lmic_pins.dio[1], INPUT);

    attachInterrupt(digitalPinToInterrupt(lmic_pins.dio[0]), hal_store_trigger, RISING);
    attachInterrupt(digitalPinToInterrupt(lmic_pins.dio[1]), hal_store_trigger, RISING);
}



// val == 1  => tx 1
void hal_pin_rxtx (uint8_t val) {
    if (lmic_pins.rxtx != LMIC_UNUSED_PIN)
        digitalWrite(lmic_pins.rxtx, val);
}

// set radio RST pin to given value (or keep floating!)
void hal_pin_rst (uint8_t val) {
    if (lmic_pins.rst == LMIC_UNUSED_PIN)
        return;

    if(val == 0 || val == 1) { // drive pin
        pinMode(lmic_pins.rst, OUTPUT);
        digitalWrite(lmic_pins.rst, val);
    } else { // keep pin floating
        pinMode(lmic_pins.rst, INPUT);
    }
}

static bool dio_states[NUM_DIO] = {0};

void hal_io_check() {
    uint8_t i;
    for (i = 0; i < NUM_DIO; ++i) {
        if (lmic_pins.dio[i] == LMIC_UNUSED_PIN)
            continue;

        if (dio_states[i] != digitalRead(lmic_pins.dio[i])) {
            dio_states[i] = !dio_states[i];
            if (dio_states[i])
                radio_irq_handler(i, last_int_trigger);
        }
        last_int_trigger = 0;
    }
}

// -----------------------------------------------------------------------------
// SPI

static const SPISettings settings(10E6, MSBFIRST, SPI_MODE0);

static void hal_spi_init () {
    SPI.begin();
}

void hal_pin_nss (uint8_t val) {
    if (!val)
        SPI.beginTransaction(settings);
    else
        SPI.endTransaction();

    //Serial.println(val?">>":"<<");
    digitalWrite(lmic_pins.nss, val);
}

// perform SPI transaction with radio
uint8_t hal_spi (uint8_t out) {
    uint8_t res = SPI.transfer(out);
/*
    Serial.print(">");
    Serial.print(out, HEX);
    Serial.print("<");
    Serial.println(res, HEX);
    */
    return res;
}

// -----------------------------------------------------------------------------
// TIME

bool is_sleep_allow = false;

bool hal_is_sleep_allow() {
    return is_sleep_allow;
}

void hal_allow_sleep() {
    is_sleep_allow = true;
}

void hal_forbid_sleep() {
    is_sleep_allow = false;
}

uint32_t time_in_sleep = 0;

void hal_add_time_in_sleep(uint32_t nb_ms)
{
    time_in_sleep += ms2osticks(nb_ms);
    os_getTime();
}

uint32_t hal_ticks () {
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
    static uint8_t overflow = 0;

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
    return (scaled | ((uint32_t)overflow << 24)) + time_in_sleep;

    // 0 leads to correct, but overly complex code (it could just return
    // micros() unmodified), 8 leaves no room for the overlapping bit.
    static_assert(US_PER_OSTICK_EXPONENT > 0 && US_PER_OSTICK_EXPONENT < 8, "Invalid US_PER_OSTICK_EXPONENT value");
}

// Returns the number of ticks until time. Negative values indicate that
// time has already passed.
int32_t delta_time(uint32_t time) {
    return (int32_t)(time - hal_ticks());
}

void hal_waitUntil (uint32_t time) {
    int32_t delta = delta_time(time);
    // From delayMicroseconds docs: Currently, the largest value that
    // will produce an accurate delay is 16383.
    while (delta > (16000 / US_PER_OSTICK)) {
        delay(16);
        delta -= (16000 / US_PER_OSTICK);
    }
    if (delta > 0)
        delayMicroseconds(delta * US_PER_OSTICK);
}

// check and rewind for target time
bool hal_checkTimer (uint32_t time) {
    
    auto delta = delta_time(time);
    if(delta <= 0)
        return true;

    // HACK for a bug (I will track it down.)
    if(delta >= sec2osticks(60*60)) {
        PRINT_DEBUG_2("WARN delta is too big, execute now ref : %lu, delta : %li", time, delta);
        return true;
    }
    return false;
}

static uint8_t irqlevel = 0;

void hal_disableIRQs () {
    noInterrupts();
    irqlevel++;
}

void hal_enableIRQs () {
    if(--irqlevel == 0) {
        interrupts();
    }
}

// -----------------------------------------------------------------------------

#if defined(LMIC_PRINTF_TO)
static int uart_putchar (char c, FILE *)
{
    LMIC_PRINTF_TO.write(c) ;
    return 0 ;
}

void hal_printf_init() {
    // create a FILE structure to reference our UART output function
    static FILE uartout;
    memset(&uartout, 0, sizeof(uartout));

    // fill in the UART file descriptor with pointer to writer.
    fdev_setup_stream (&uartout, uart_putchar, NULL, _FDEV_SETUP_WRITE);

    // The uart is the standard output device STDOUT.
    stdout = &uartout ;
}
#endif // defined(LMIC_PRINTF_TO)

void hal_init () {
    // configure radio I/O and interrupt handler
    hal_io_init();
    // configure radio SPI
    hal_spi_init();
    // configure timer 
#if defined(LMIC_PRINTF_TO)
    // printf support
    hal_printf_init();
#endif
}

void hal_failed (const char *file, uint16_t line) {
#if defined(LMIC_FAILURE_TO)
    LMIC_FAILURE_TO.println("FAILURE ");
    LMIC_FAILURE_TO.print(file);
    LMIC_FAILURE_TO.print(':');
    LMIC_FAILURE_TO.println(line);
    LMIC_FAILURE_TO.flush();
#endif
    hal_disableIRQs();
    while(1);
}
