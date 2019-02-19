#include "sleepandwatchdog.h"

#include "Arduino.h"
#include <avr/sleep.h>
#include <avr/wdt.h>
#include <avr/power.h>
#include <avr/interrupt.h>

namespace
{
volatile bool wdtEnable = false;
}

void powerDown(Sleep period)
{
    bool back = wdtEnable;
    wdtEnable = false;
    ADCSRA &= ~(1 << ADEN);

    if (period != Sleep::FOREVER)
    {
        wdt_enable(static_cast<uint8_t>(period));
        WDTCSR |= (1 << WDIE);
    }
    set_sleep_mode(SLEEP_MODE_PWR_DOWN);
    cli();
    sleep_enable();
    sleep_bod_disable();
    sei();
    sleep_cpu();
    sleep_disable();
    sei();
    ADCSRA |= (1 << ADEN);

    if (back)
        configure_wdt();
}

void configure_wdt()
{
    wdtEnable = true;
    wdt_enable(WDTO_8S);
    WDTCSR |= (1 << WDIE);
}

void rst_wdt()
{
    wdt_reset();
}

ISR(WDT_vect)
{
    if (!wdtEnable)
    {
        // WDIE & WDIF is cleared in hardware upon entering this ISR
        wdt_disable();
    }
    else
    {
        // enable watchdog without interupt to reboot
        wdt_enable(static_cast<uint8_t>(Sleep::P8S));
        // reboot
        while (1)
            ;
    }
}