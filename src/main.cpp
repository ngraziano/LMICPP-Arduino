
#include <Arduino.h>

#include <lmic.h>
#include <hal/hal.h>
#include <SPI.h>
#include <LowPower.h>

void do_send(OsJob* j);

// This EUI must be in little-endian format, so least-significant-byte
// first. When copying an EUI from ttnctl output, this means to reverse
// the bytes. For TTN issued EUIs the last bytes should be 0xD5, 0xB3,
// 0x70.
void os_getArtEui (uint8_t* buf) { memcpy_P(buf, APPEUI, 8);}

// This should also be in little endian format, see above.
static const uint8_t PROGMEM DEVEUI[8]={ 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
void os_getDevEui (uint8_t* buf) { memcpy_P(buf, DEVEUI, 8);}

// This key should be in big endian format (or, since it is not really a
// number but a block of memory, endianness does not really apply). In
// practice, a key taken from ttnctl can be copied as-is.
// The key shown here is the semtech default key.

void os_getDevKey (uint8_t* buf) {  memcpy_P(buf, APPKEY, 16);}

static uint8_t mydata[] = "Hello, world!";

OsJob sendjob;

// Schedule TX every this many seconds (might become longer due to duty
// cycle limitations).
const unsigned TX_INTERVAL = 60;

const unsigned int BAUDRATE = 19200;

// Pin mapping
const lmic_pinmap lmic_pins = {
    .nss = 14,
    .rxtx = LMIC_UNUSED_PIN,
    .rst = 5,
    .dio = {2, 3},
};

void onEvent (ev_t ev) {
    Serial.print(os_getTime());
    Serial.print(": ");
    switch(ev) {
        case EV_SCAN_TIMEOUT:
            Serial.println(F("EV_SCAN_TIMEOUT"));
            break;
        case EV_BEACON_FOUND:
            Serial.println(F("EV_BEACON_FOUND"));
            break;
        case EV_BEACON_MISSED:
            Serial.println(F("EV_BEACON_MISSED"));
            break;
        case EV_BEACON_TRACKED:
            Serial.println(F("EV_BEACON_TRACKED"));
            break;
        case EV_JOINING:
            Serial.println(F("EV_JOINING"));
            break;
        case EV_JOINED:
            Serial.println(F("EV_JOINED"));

            // Disable link check validation (automatically enabled
            // during join, but not supported by TTN at this time).
            LMIC_setLinkCheckMode(0);
            break;
        case EV_RFU1:
            Serial.println(F("EV_RFU1"));
            break;
        case EV_JOIN_FAILED:
            Serial.println(F("EV_JOIN_FAILED"));
            break;
        case EV_REJOIN_FAILED:
            Serial.println(F("EV_REJOIN_FAILED"));
            break;
            break;
        case EV_TXCOMPLETE:
            Serial.println(F("EV_TXCOMPLETE (includes waiting for RX windows)"));
            if (LMIC.txrxFlags & TXRX_ACK)
              Serial.println(F("Received ack"));
            if (LMIC.dataLen) {
              Serial.println(F("Received "));
              Serial.println(LMIC.dataLen);
              Serial.println(F(" bytes of payload"));
            }
            // Schedule next transmission
            sendjob.setTimedCallback(os_getTime()+sec2osticks(TX_INTERVAL), do_send);
            break;
        case EV_LOST_TSYNC:
            Serial.println(F("EV_LOST_TSYNC"));
            break;
        case EV_RESET:
            Serial.println(F("EV_RESET"));
            break;
        case EV_RXCOMPLETE:
            // data received in ping slot
            Serial.println(F("EV_RXCOMPLETE"));
            break;
        case EV_LINK_DEAD:
            Serial.println(F("EV_LINK_DEAD"));
            break;
        case EV_LINK_ALIVE:
            Serial.println(F("EV_LINK_ALIVE"));
            break;
         default:
            Serial.println(F("Unknown event"));
            break;
    }
}

void do_send(OsJob* j){
    // Check if there is not a current TX/RX job running
    if (LMIC.opmode & OP_TXRXPEND) {
        Serial.println(F("OP_TXRXPEND, not sending"));
    } else {
        // Prepare upstream data transmission at the next possible time.
        LMIC_setTxData2(1, mydata, sizeof(mydata)-1, 0);
        Serial.println(F("Packet queued"));
    }
    // Next TX is scheduled after TX_COMPLETE event.
}

void setup() {
    Serial.begin(BAUDRATE);
    Serial.println(F("Starting"));


    // LMIC init
    os_init();
    
   
    // Reset the MAC state. Session and pending data transfers will be discarded.
    LMIC_reset();

    // for(int i = 1; i <= 8; i++) LMIC_disableChannel(i);
    // LMIC_setupChannel(0, 868100000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);

    // TTN uses SF9 for its RX2 window.
    // LMIC.dn2Dr = DR_SF9;
    // Set data rate and transmit power for uplink (note: txpow seems to be ignored by the library)
    // LMIC_setDrTxpow(DR_SF9,14);


    
    // Start job (sending automatically starts OTAA too)
    do_send(&sendjob);
}


void powersave(int32_t maxTime) {
    // fastpath
    if(maxTime == 0)
        return;

    int32_t duration_selected = 0;
    period_t period_selected;

    if(maxTime > 8500) {
        duration_selected = 8500;
        period_selected = SLEEP_8S;
    } else if (maxTime > 4500) {
        duration_selected = 4200;
        period_selected = SLEEP_4S;
    } else if (maxTime > 2500) {
        duration_selected = 2100;
        period_selected = SLEEP_2S;
    } else if (maxTime > 1500) {
        duration_selected = 1000;
        period_selected = SLEEP_1S;
    } else if (maxTime > 1000) {
        duration_selected = 510;
        period_selected = SLEEP_500MS;
    } else {
        return;
    }
    #if LMIC_DEBUG_LEVEL > 1
        Serial.print(os_getTime());
        Serial.print(": Sleep :");
        Serial.println(duration_selected);
    #endif
    Serial.end();
    LowPower.powerDown(period_selected, ADC_OFF, BOD_OFF);
    hal_add_time_in_sleep(duration_selected);
    Serial.begin(BAUDRATE);
    delay(100);
    #if LMIC_DEBUG_LEVEL > 1            
        Serial.print(os_getTime());
        Serial.println(": wakeup");
    #endif
}


void loop() {
    int32_t to_wait = OSS.runloopOnce();
    if(hal_is_sleep_allow()) {
        powersave(to_wait);
    } 
}