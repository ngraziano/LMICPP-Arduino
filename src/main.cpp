
#include <Arduino.h>

#include <lmic.h>


#include <hal/hal.h>
#include <SPI.h>
#include <LowPower.h>
#include <SparkFun_APDS9960.h>

#define DEVICE_POSTDETECT
#include "lorakeys.h"

void do_send();
void reset_and_do_send();

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

uint8_t data[12] = {};

OsJob sendjob;

SparkFun_APDS9960 apds = SparkFun_APDS9960();

// Schedule TX every this many seconds (might become longer due to duty
// cycle limitations).
const OsDeltaTime TX_INTERVAL = OsDeltaTime::from_sec(60 * 60);
// keep ON for one minute
const OsDeltaTime TX_ONLENGTH = OsDeltaTime::from_sec(20);

const unsigned int BAUDRATE = 19200;


LmicEu868 LMIC;
// Pin mapping
const lmic_pinmap lmic_pins = {
    .nss = 10,
    .rxtx = LMIC_UNUSED_PIN,
    .rst = 14,
    .dio = {9, 8},
};

const uint8_t NUMBERTIME_TO_SEND = 3;
uint8_t apds_tosend = 0;
bool apds_new = false;

void initProximitySensor(uint8_t threshold)
{
    if (apds.init())
    {
        Serial.println(F("APDS-9960 initialization complete"));
    }
    else
    {
        Serial.println(F("Something went wrong during APDS-9960 init!"));
    }

    // Set proximity interrupt thresholds
    apds.setProximityIntLowThreshold(0);
    apds.setProximityIntHighThreshold(threshold);
    apds.setProximityGain(DEFAULT_PGAIN);
    apds.setLEDDrive(LED_DRIVE_50MA);
    apds.setProximityIntEnable(1);
    apds.enablePower();
    apds.setMode(PROXIMITY, 1);
}

void disableProximitySensor()
{
    apds.setMode(PROXIMITY, 0);
    apds.disablePower();
}

void apdsInterrupt()
{
    // If we are still sending detect event
    // do not try do detect another one.
    if (apds_tosend > 0)
        return;
    if (!digitalRead(3))
    {
        apds_tosend = NUMBERTIME_TO_SEND;
        apds_new = true;
    }
}

void onEvent(EventType ev)
{
    switch (ev)
    {
    case EventType::JOINING:
        PRINT_DEBUG_2("EV_JOINING");
        break;
    case EventType::JOINED:
        PRINT_DEBUG_2("EV_JOINED");
        break;
    case EventType::JOIN_FAILED:
        PRINT_DEBUG_2("EV_JOIN_FAILED");
        break;
    case EventType::REJOIN_FAILED:
        PRINT_DEBUG_2("EV_REJOIN_FAILED");
        break;
    case EventType::TXCOMPLETE:
        PRINT_DEBUG_2("EV_TXCOMPLETE (includes waiting for RX windows)");
        if (LMIC.txrxFlags & TXRX_ACK)
            PRINT_DEBUG_2("Received ack");
        if (LMIC.dataLen)
        {
            PRINT_DEBUG_2("Received %d  bytes of payload", LMIC.dataLen);
            if (LMIC.dataBeg > 0)
            {
                uint8_t port = LMIC.frame[LMIC.dataBeg - 1];
                if (port == 9)
                {
                    disableProximitySensor();
                    delay(500);
                    initProximitySensor(LMIC.frame[LMIC.dataBeg]);
                }
            }
        }
        // we have transmit
        if (apds_tosend)
        {

            // schedule back to off
            sendjob.setTimedCallback(os_getTime() + TX_ONLENGTH, reset_and_do_send);
        }
        else
        {
            // Schedule next transmission
            sendjob.setTimedCallback(os_getTime() + TX_INTERVAL, do_send);
        }

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

void reset_and_do_send()
{
    // we have sent one
    if (apds_tosend > 0)
        apds_tosend--;
    if (apds_tosend == 0)
        apds.clearProximityInt();
    do_send();
}

void do_send()
{
    // Check if there is not a current TX/RX job running
    if (LMIC.getOpMode() & OP_TXRXPEND)
    {
        PRINT_DEBUG_1("OP_TXRXPEND, not sending");
        // should not happen so reschedule anymway
        sendjob.setTimedCallback(os_getTime() + TX_INTERVAL, do_send);
    }
    else
    {
        const uint8_t pinCmd = 6;
        pinMode(pinCmd, OUTPUT);
        digitalWrite(pinCmd, 1);
        delay(100);
        // battery
        data[0] = 1;
        data[1] = 2;
        uint16_t val = analogRead(A1) * (6.6 / 1024 * 100);
        data[2] = val >> 8;
        data[3] = val;

        data[4] = 2;
        data[5] = 2;
        uint8_t prox;
        apds.readProximity(prox);
        val = prox * 100;
        data[6] = val >> 8;
        data[7] = val;

        data[8] = 3;
        data[9] = 0;
        data[10] = apds_tosend > 0 ? 1 : 0;

        // Prepare upstream data transmission at the next possible time.
        LMIC.setTxData2(1, (uint8_t *)data, 11, false);
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
    hal_store_trigger();
}

void pciSetup(byte pin)
{
    *digitalPinToPCMSK(pin) |= bit(digitalPinToPCMSKbit(pin)); // enable pin
    PCIFR |= bit(digitalPinToPCICRbit(pin));                   // clear any outstanding interrupt
    PCICR |= bit(digitalPinToPCICRbit(pin));                   // enable interrupt for the group
}

void setup()
{
#if LMIC_DEBUG_LEVEL > 0
    Serial.begin(BAUDRATE);
    Serial.println(F("Starting"));
#endif
    pinMode(3, INPUT);
    attachInterrupt(digitalPinToInterrupt(3), apdsInterrupt, FALLING);
    initProximitySensor(40);

    pciSetup(lmic_pins.dio[0]);
    pciSetup(lmic_pins.dio[1]);

    // LMIC init
    os_init();
    LMIC.init();
    // Reset the MAC state. Session and pending data transfers will be discarded.
    LMIC.reset();

    uint8_t buf[16];
    memcpy_P(buf, APPKEY, 16);
    LMIC.aes.setDevKey(buf);
    LMIC.setEventCallBack(onEvent);
    LMIC.setDevEuiCallback(getDevEui);
    LMIC.setArtEuiCallback(getArtEui);

    // set clock error to allow good connection.
    LMIC.setClockError(MAX_CLOCK_ERROR * 3 / 100);

    // Start job (sending automatically starts OTAA too)
    do_send();
}

void powersave(OsDeltaTime const &maxTime)
{
    OsDeltaTime duration_selected;
    period_t period_selected;
    // these value are base on test
    if (maxTime > OsDeltaTime::from_ms(8700))
    {
        duration_selected = OsDeltaTime::from_ms(8050);
        period_selected = SLEEP_8S;
    }
    else if (maxTime > OsDeltaTime::from_ms(4600))
    {
        duration_selected = OsDeltaTime::from_ms(4050);
        period_selected = SLEEP_4S;
    }
    else if (maxTime > OsDeltaTime::from_ms(2600))
    {
        duration_selected = OsDeltaTime::from_ms(2000);
        period_selected = SLEEP_2S;
    }
    else if (maxTime > OsDeltaTime::from_ms(1500))
    {
        duration_selected = OsDeltaTime::from_ms(1000);
        period_selected = SLEEP_1S;
    }
    else if (maxTime > OsDeltaTime::from_ms(800))
    {
        duration_selected = OsDeltaTime::from_ms(500);
        period_selected = SLEEP_500MS;
    }
    else if (maxTime > OsDeltaTime::from_ms(500))
    {
        duration_selected = OsDeltaTime::from_ms(250);
        period_selected = SLEEP_250MS;
    }
    else
    {
        return;
    }

#if LMIC_DEBUG_LEVEL > 0
    PRINT_DEBUG_1("Sleep (ostick) :%lix%i", duration_selected.to_ms(), maxTime / duration_selected);
    Serial.flush();
#endif

    for (uint16_t nbsleep = maxTime / duration_selected; nbsleep > 0 && !apds_new; nbsleep--)
    {
        LowPower.powerDown(period_selected, ADC_OFF, BOD_OFF);
        hal_add_time_in_sleep(duration_selected);

        // Check if we are wakeup by external pin.
        apdsInterrupt();
    }

    PRINT_DEBUG_1("Wakeup");
}

void loop()
{
    OsDeltaTime to_wait = OSS.runloopOnce();
    if (to_wait > 0 && hal_is_sleep_allow())
    {
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
        hal_io_check(LMIC);
    }
    // was wakeup by interrupt, send new state.
    if (apds_new)
    {
        apds_new = false;
        do_send();
    }
}