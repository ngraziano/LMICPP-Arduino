
#include <Arduino.h>

#include <lmic.h>
#include <hal/hal_io.h>

#include <sleepandwatchdog.h>

#define DEVICE_BALISE1
#include "lorakeys.h"

void do_send();
void reset_and_do_send();
void powersave(OsDeltaTime maxTime);

// This EUI must be in little-endian format, so least-significant-byte
// first. When copying an EUI from ttnctl output, this means to reverse
// the bytes. For TTN issued EUIs the last bytes should be 0xD5, 0xB3,
// 0x70.
// defined in lorakeys.h
void getArtEui(uint8_t *buf) { memcpy_P(buf, APPEUI, 8); }

// This should also be in little endian format, see above.
// defined in lorakeys.h
// static const uint8_t PROGMEM DEVEUI[8]={ 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
void getDevEui(uint8_t *buf) { memcpy_P(buf, DEVEUI, 8); }

// This key should be in big endian format (or, since it is not really a
// number but a block of memory, endianness does not really apply). In
// practice, a key taken from ttnctl can be copied as-is.
// The key shown here is the semtech default key.
// defined in lorakeys.h

uint8_t data[2] = {};

OsJob sendjob;

// Schedule TX every this many seconds (might become longer due to duty
// cycle limitations).
const OsDeltaTime TX_INTERVAL = OsDeltaTime::from_sec(135);


const unsigned int BAUDRATE = 19200;

// Pin mapping
constexpr lmic_pinmap lmic_pins = {
    .nss = 10,
    .rxtx = LMIC_UNUSED_PIN,
    .rst = 14,
    .dio = {9, 8},
};
LmicEu868 LMIC(lmic_pins);

void onEvent(EventType ev)
{
    rst_wdt();
    switch (ev)
    {
    case EventType::JOINING:
        PRINT_DEBUG_2("EV_JOINING");
        //        LMIC.setDrJoin(0);
        break;
    case EventType::JOINED:
        PRINT_DEBUG_2("EV_JOINED");
        // disable ADR because it will be mobile.
        LMIC.setLinkCheckMode(false);
        break;
    case EventType::JOIN_FAILED:
        PRINT_DEBUG_2("EV_JOIN_FAILED");
        break;
    case EventType::REJOIN_FAILED:
        PRINT_DEBUG_2("EV_REJOIN_FAILED");
        break;
    case EventType::TXCOMPLETE:
        PRINT_DEBUG_2("EV_TXCOMPLETE (includes waiting for RX windows)");
        if (LMIC.getTxRxFlags().test(TxRxStatus::ACK)) {
            PRINT_DEBUG_2("Received ack");
        }
        if (LMIC.getDataLen())
        {
            PRINT_DEBUG_2("Received %d  bytes of payload", LMIC.getDataLen());
            auto data = LMIC.getData();
            if (data)
            {
                uint8_t port = LMIC.getPort();
                if (port == 9)
                {
                }
            }
        }
        // we have transmit
            // Schedule next transmission
            sendjob.setTimedCallback(os_getTime() + TX_INTERVAL, do_send);

        break;
    case EventType::RESET:
        PRINT_DEBUG_2("EV_RESET");
        break;
    case EventType::LINK_DEAD:
        PRINT_DEBUG_2("EV_LINK_DEAD");
        break;
    case EventType::LINK_ALIVE:
        PRINT_DEBUG_2("EV_LINK_ALIVE");
        break;
    default:
        PRINT_DEBUG_2("Unknown event");
        break;
    }
}



void do_send()
{
    // Check if there is not a current TX/RX job running
    if (LMIC.getOpMode().test(OpState::TXRXPEND))
    {
        PRINT_DEBUG_1("OpState::TXRXPEND, not sending");
        // should not happen so reschedule anymway
        sendjob.setTimedCallback(os_getTime() + TX_INTERVAL, do_send);
    }
    else
    {
        // battery
        uint8_t val = ((uint32_t)analogRead(A1)) * 255 / 683;

        // Prepare upstream data transmission at the next possible time.
        LMIC.setTxData2(2, &val, 1, false);
        PRINT_DEBUG_1("Packet queued");
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

void testDuration(int32_t ms)
{
    const auto delta = OsDeltaTime::from_ms(ms);
    PRINT_DEBUG_1("Test sleep time for %i ms.", ms);
    const OsTime start = os_getTime();
    PRINT_DEBUG_1("Start Test sleep time.");
    powersave(delta);
    const OsTime end = os_getTime();
    PRINT_DEBUG_1("End Test sleep time.");
    PRINT_DEBUG_1("Test Time should be : %d ms", (end - start).to_ms());
}

void setup()
{
#if LMIC_DEBUG_LEVEL > 0
    Serial.begin(BAUDRATE);
    Serial.println(F("Starting"));
#endif
    pciSetup(lmic_pins.dio[0]);
    pciSetup(lmic_pins.dio[1]);

    // LMIC init
    os_init();
    LMIC.init();
    // Reset the MAC state. Session and pending data transfers will be discarded.
    LMIC.reset();

    uint8_t buf[16];
    memcpy_P(buf, APPKEY, 16);
    LMIC.setDevKey(buf);
    LMIC.setEventCallBack(onEvent);
    LMIC.setDevEuiCallback(getDevEui);
    LMIC.setArtEuiCallback(getArtEui);
    // set clock error to allow good connection.
    LMIC.setClockError(MAX_CLOCK_ERROR * 15 / 100);
    LMIC.setAntennaPowerAdjustment(-14);

    // Only work with special boot loader.
    // configure_wdt();


   // test duration and in case of reboot loop  prevent flood
   testDuration(1000);
   testDuration(8000);
   testDuration(30000);
    

    // Start job (sending automatically starts OTAA too)
    do_send();
}

const int64_t sleepAdj = 1080;

void powersave(OsDeltaTime maxTime)
{
    OsDeltaTime duration_selected;
    Sleep period_selected;
    // these value are base on test
    if (maxTime > OsDeltaTime::from_ms(8700))
    {
        duration_selected = OsDeltaTime::from_ms(8000 * sleepAdj /1000);
        period_selected = Sleep::P8S;
    }
    else if (maxTime > OsDeltaTime::from_ms(4600))
    {
        duration_selected = OsDeltaTime::from_ms(4000 * sleepAdj /1000);
        period_selected = Sleep::P4S;
    }
    else if (maxTime > OsDeltaTime::from_ms(2600))
    {
        duration_selected = OsDeltaTime::from_ms(2000 * sleepAdj /1000);
        period_selected = Sleep::P2S;
    }
    else if (maxTime > OsDeltaTime::from_ms(1500))
    {
        duration_selected = OsDeltaTime::from_ms(1000 * sleepAdj /1000);
        period_selected = Sleep::P1S;
    }
    else if (maxTime > OsDeltaTime::from_ms(800))
    {
        duration_selected = OsDeltaTime::from_ms(500  * sleepAdj /1000);
        period_selected = Sleep::P500MS;
    }
    else if (maxTime > OsDeltaTime::from_ms(500))
    {
        duration_selected = OsDeltaTime::from_ms(250  * sleepAdj /1000);
        period_selected = Sleep::P250MS;
    }
    else
    {
        return;
    }

#if LMIC_DEBUG_LEVEL > 0
    PRINT_DEBUG_1("Sleep (ostick) :%lix%i", duration_selected.to_ms(), maxTime / duration_selected);
    Serial.flush();
#endif

    for (uint16_t nbsleep = maxTime / duration_selected; nbsleep > 0; nbsleep--)
    {
        powerDown(period_selected);
        hal_add_time_in_sleep(duration_selected);
    }

    PRINT_DEBUG_1("Wakeup");
}

void loop()
{
    rst_wdt();
    OsDeltaTime to_wait = OSS.runloopOnce();
    if (to_wait > OsDeltaTime(0))
    {
        // Go to sleep if we have nothing to do.
        powersave(to_wait);
    }
    else
    {
        // Instead of using proper interrupts (which are a bit tricky
        // and/or not available on all pins on AVR), just poll the pin
        // values. Here makes sure we check at least once every
        // loop.
        //
        // As an additional bonus, this prevents the can of worms that
        // we would otherwise get for running SPI transfers inside ISRs
        LMIC.io_check();
    }
}