
#include <Arduino.h>
#include <SPI.h>

#include <lmic.h>
#include <hal/hal_io.h>
#include <hal/print_debug.h>
#include <keyhandler.h>

#define DEVICE_SIMPLE_SX1262
#include "lorakeys.h"

void do_send();
void reset_and_do_send();
void powersave(OsDeltaTime maxTime);

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
OsScheduler OSS;
// Radio class for SX1262
RadioSx1262 radio{lmic_pins, ImageCalibrationBand::band_863_870};
LmicEu868 LMIC{radio, OSS};

OsJob sendjob{OSS};

void onEvent(EventType ev)
{
    switch (ev)
    {
    case EventType::JOINING:
        PRINT_DEBUG(2, F("EV_JOINING"));
        //        LMIC.setDrJoin(0);
        break;
    case EventType::JOINED:
        PRINT_DEBUG(2, F("EV_JOINED"));
        // disable ADR because it will be mobile.
        LMIC.setLinkCheckMode(false);
        break;
    case EventType::JOIN_FAILED:
        PRINT_DEBUG(2, F("EV_JOIN_FAILED"));
        break;
    case EventType::TXCOMPLETE:
        PRINT_DEBUG(2, F("EV_TXCOMPLETE (includes waiting for RX windows)"));
        if (LMIC.getTxRxFlags().test(TxRxStatus::ACK))
        {
            PRINT_DEBUG(1, F("Received ack"));
        }
        if (LMIC.getDataLen())
        {
            PRINT_DEBUG(1, F("Received %d bytes of payload"), LMIC.getDataLen());
            auto data = LMIC.getData();
            if (data)
            {
                uint8_t port = LMIC.getPort();
            }
        }
        // we have transmit
        // Schedule next transmission
        sendjob.setTimedCallback(os_getTime() + TX_INTERVAL, do_send);

        break;
    case EventType::RESET:
        PRINT_DEBUG(2, F("EV_RESET"));
        break;
    case EventType::LINK_DEAD:
        PRINT_DEBUG(2, F("EV_LINK_DEAD"));
        break;
    case EventType::LINK_ALIVE:
        PRINT_DEBUG(2, F("EV_LINK_ALIVE"));
        break;
    default:
        PRINT_DEBUG(2, F("Unknown event"));
        break;
    }
}

void do_send()
{
    // Check if there is not a current TX/RX job running
    if (LMIC.getOpMode().test(OpState::TXRXPEND))
    {
        PRINT_DEBUG(1, F("OpState::TXRXPEND, not sending"));
        // should not happen so reschedule anymway
        sendjob.setTimedCallback(os_getTime() + TX_INTERVAL, do_send);
    }
    else
    {
        // some value
        uint8_t val = ((uint32_t)analogRead(A1)) * 255 / 683;

        // Prepare upstream data transmission at the next possible time.
        LMIC.setTxData2(2, &val, 1, false);
        PRINT_DEBUG(1, F("Packet queued"));
    }
    // Next TX is scheduled after TX_COMPLETE event.
}

// lmic_pins.dio[0]  = 9 => PCINT1
// lmic_pins.dio[1]  = 8 => PCINT0
// PCI2 PCINT[23:16]
// PCI1 PCINT[14:8]
// PCI0 PCINT[7:0]

ISR(PCINT0_vect)
{
    // one of pins D8 to D13 has changed
    // store time, will be check in OSS.runloopOnce()
    LMIC.store_trigger();
}

void pciSetup(byte pin)
{
    *digitalPinToPCMSK(pin) |= bit(digitalPinToPCMSKbit(pin)); // enable pin
    PCIFR |= bit(digitalPinToPCICRbit(pin));                   // clear any outstanding interrupt
    PCICR |= bit(digitalPinToPCICRbit(pin));                   // enable interrupt for the group
}

void setup()
{
    if (debugLevel > 0)
    {
        Serial.begin(BAUDRATE);
    }

    pciSetup(lmic_pins.dio[1]);

    SPI.begin();
    // LMIC init
    os_init();
    LMIC.init();
    // Reset the MAC state. Session and pending data transfers will be discarded.
    LMIC.reset();

    LMIC.setEventCallBack(onEvent);
    SetupLmicKey<appEui, devEui, appKey>::setup(LMIC);
    // set clock error to allow good connection.
    LMIC.setClockError(MAX_CLOCK_ERROR * 1 / 100);

    // Start job (sending automatically starts OTAA too)
    do_send();
}

void loop()
{
    OsDeltaTime to_wait = OSS.runloopOnce();
    if (to_wait > OsDeltaTime(0))
    {
        // sleep if we have nothing to do.
    }
}