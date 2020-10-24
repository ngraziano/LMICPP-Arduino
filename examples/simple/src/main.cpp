
#include <Arduino.h>
#include <SPI.h>

#include <lmic.h>
#include <hal/hal_io.h>
#include <hal/print_debug.h>
#include <keyhandler.h>
#include <algorithm>

#define DEVICE_SIMPLE
#include "lorakeys.h"

// Schedule TX every this many seconds (might become longer due to duty
// cycle limitations).
constexpr OsDeltaTime TX_INTERVAL = OsDeltaTime::from_sec(135);

constexpr unsigned int BAUDRATE = 19200;

// Pin mapping
constexpr lmic_pinmap lmic_pins = {
    .nss = 10,
    .prepare_antenna_tx = nullptr,
    .rst = 14,
    .dio = {9, 8},
};

RadioSx1276 radio{lmic_pins};
LmicEu868 LMIC{radio};

OsTime nextSend;

void do_send()
{
    // battery
    uint8_t val = ((uint32_t)analogRead(A1)) * 255 / 683;

    // Prepare upstream data transmission at the next possible time.
    LMIC.setTxData2(2, &val, 1, false);
    PRINT_DEBUG(1, F("Packet queued"));
    nextSend = hal_ticks() + TX_INTERVAL;
}

void setup()
{
    if (debugLevel > 0)
    {
        Serial.begin(BAUDRATE);
    }

    SPI.begin();
    // LMIC init
    os_init();
    LMIC.init();
    // Reset the MAC state. Session and pending data transfers will be discarded.
    LMIC.reset();

    SetupLmicKey<appEui, devEui, appKey>::setup(LMIC);
    // set clock error to allow good connection.
    LMIC.setClockError(MAX_CLOCK_ERROR * 3 / 100);
    // reduce power
    // LMIC.setAntennaPowerAdjustment(-14);

    // Start job (sending automatically starts OTAA too)
    nextSend = os_getTime();
}

void loop()
{
    OsDeltaTime freeTimeBeforeNextCall = LMIC.run();
    if (freeTimeBeforeNextCall < OsDeltaTime::from_ms(100))
    {
        // not enought time, do nothing else.
        return;
    }

    if (LMIC.getOpMode().test(OpState::TXRXPEND))
    {
        // OpState::TXRXPEND, do not try to send new message
        // so wait (with 100ms of marging) and return
        auto waittime = freeTimeBeforeNextCall.to_ms() - 100;
        PRINT_DEBUG(1, F("Delay TXRXPEND %dms"), waittime);
        delay(waittime);
        return;
    }

    // we have more than 100 ms to do some work.
    // the test must be adapted from the time spend in other task
    auto timebeforesend = nextSend - hal_ticks();
    PRINT_DEBUG(1, F("Time before send %ds"), timebeforesend.to_s());

    if (timebeforesend < OsDeltaTime(0))
    {
        // time to send now.
        do_send();
    }
    else
    {
        // sleep if we have nothing to do.
        auto sleeps = std::min(timebeforesend, freeTimeBeforeNextCall).to_s();
        PRINT_DEBUG(1, F("Sleep %ds"), sleeps);
        delay(sleeps * 1000);
    }
}