
#include <Arduino.h>
#include <SPI.h>

#include <lmic.h>
#include <hal/hal_io.h>
#include <hal/print_debug.h>
#include <keyhandler.h>
#include <algorithm>

#define DEVICE_SIMPLE_SX1262
#include "lorakeys.h"

// Schedule TX every this many seconds (might become longer due to duty
// cycle limitations).
constexpr OsDeltaTime TX_INTERVAL = OsDeltaTime::from_sec(135);
constexpr unsigned int BAUDRATE = 9600;

// Pin mapping
constexpr lmic_pinmap lmic_pins = {
    .nss = 10,
    .prepare_antenna_tx = nullptr,
    .rst = 14,
    .dio = {/* busy */ 9, /* DIO1 */ 8},
};
// Radio class for SX1262
RadioSx1262 radio{lmic_pins, ImageCalibrationBand::band_863_870};
// Create an LMIC object with the right band
LmicEu868 LMIC{radio};
OsTime nextSend;


void do_send()
{
    // some value
    uint8_t val = ((uint32_t)analogRead(A1)) * 255 / 683;

    // Prepare upstream data transmission at the next possible time.
    LMIC.setTxData2(2, &val, 1, false);
    PRINT_DEBUG(1, F("Packet queued"));
    nextSend = hal_ticks() + TX_INTERVAL;
}

// lmic_pins.dio[0]  = 9 => PCINT1
// lmic_pins.dio[1]  = 8 => PCINT0
// PCI2 PCINT[23:16]
// PCI1 PCINT[14:8]
// PCI0 PCINT[7:0]

/* INTERUPT ON CHANGE OPTIONAL
ISR(PCINT0_vect)
{
    // one of pins D8 to D13 has changed
    // store time, will be check in LMIC.run()
    LMIC.store_trigger();
}

void pciSetup(byte pin)
{
    *digitalPinToPCMSK(pin) |= bit(digitalPinToPCMSKbit(pin)); // enable pin
    PCIFR |= bit(digitalPinToPCICRbit(pin));                   // clear any outstanding interrupt
    PCICR |= bit(digitalPinToPCICRbit(pin));                   // enable interrupt for the group
}

INTERUPT ON CHANGE OPTIONAL
*/
void setup()
{
    // To handle VCC <= 2.4v
    // clock start at 8MHz / 8 => 1 MHz
    // set clock to 8MHz / 4 => 2MHz
    // maybe 4Mhz could also work
    // clock_prescale_set(clock_div_4);
    if (debugLevel > 0)
    {
        Serial.begin(BAUDRATE);
    }

    if (debugLevel > 0)
    {
        Serial.begin(BAUDRATE);
    }

    // INTERUPT ON CHANGE OPTIONAL
    // pciSetup(lmic_pins.dio[1]);

    SPI.begin();
    // LMIC init
    os_init();
    LMIC.init();
    // Reset the MAC state. Session and pending data transfers will be discarded.
    LMIC.reset();

    SetupLmicKey<appEui, devEui, appKey>::setup(LMIC);
    // set clock error to allow good connection.
    LMIC.setClockError(MAX_CLOCK_ERROR * 1 / 100);

    // Start job (sending automatically starts OTAA too)
   nextSend = os_getTime();
   PRINT_DEBUG(1, F("END SETUP"));

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
        auto waittime = freeTimeBeforeNextCall.to_ms()-100;
        PRINT_DEBUG(1, F("Delay TXRXPEND %dms"), waittime);
        delay(waittime);
        return;
    }

    // we have more than 100 ms to do some work.
    // the test must be adapted from the time spend in other task
    auto timebeforesend = nextSend - hal_ticks();
    PRINT_DEBUG(1, F("Time before send %ds"), timebeforesend.to_s());

    if (timebeforesend < OsDeltaTime(0) )
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