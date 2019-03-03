#include <Arduino.h>

#include <hal/hal_io.h>
#include <hal/print_debug.h>
#include <keyhandler.h>
#include <lmic.h>

#define DEVICE_TESTESP32
#include "lorakeys.h"

void do_send();

// Schedule TX every this many seconds (might become longer due to duty
// cycle limitations).
constexpr OsDeltaTime TX_INTERVAL = OsDeltaTime::from_sec(135);

constexpr unsigned int BAUDRATE = 115200;
// Pin mapping
constexpr lmic_pinmap lmic_pins = {
    .nss = 18,
    .rxtx = LMIC_UNUSED_PIN,
    .rst = 14,
    .dio = {26, 33},
};
OsScheduler OSS;
LmicEu868 LMIC{lmic_pins, OSS};

OsJob sendjob{OSS};

void onEvent(EventType ev) {
  switch (ev) {
  case EventType::JOINING:
    PRINT_DEBUG(2, F("EV_JOINING"));
    //        LMIC.setDrJoin(0);
    break;
  case EventType::JOINED:
    PRINT_DEBUG(2, F("EV_JOINED"));
    // disable ADR because if will be mobile.
    // LMIC.setLinkCheckMode(false);
    break;
  case EventType::JOIN_FAILED:
    PRINT_DEBUG(2, F("EV_JOIN_FAILED"));
    break;
  case EventType::REJOIN_FAILED:
    PRINT_DEBUG(2, F("EV_REJOIN_FAILED"));
    break;
  case EventType::TXCOMPLETE:
    PRINT_DEBUG(2, F("EV_TXCOMPLETE (includes waiting for RX windows)"));
    if (LMIC.getTxRxFlags().test(TxRxStatus::ACK)) {
      PRINT_DEBUG(1, F("Received ack"));
    }
    if (LMIC.getDataLen()) {
      PRINT_DEBUG(1, F("Received %d bytes of payload"), LMIC.getDataLen());
      auto data = LMIC.getData();
      if (data) {
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

void do_send() {
  // Check if there is not a current TX/RX job running
  if (LMIC.getOpMode().test(OpState::TXRXPEND)) {
    PRINT_DEBUG(1, F("OpState::TXRXPEND, not sending"));
    // should not happen so reschedule anyway
    sendjob.setTimedCallback(os_getTime() + TX_INTERVAL, do_send);
  } else {
    // Some analog value
    // val = analogRead(A1) >> 4;
    uint8_t val = temperatureRead(); 
    // Prepare upstream data transmission at the next possible time.
    LMIC.setTxData2(2, &val, 1, false);
    PRINT_DEBUG(1, F("Packet queued"));
  }
  // Next TX is scheduled after TX_COMPLETE event.
}

void setup() {

  if (debugLevel > 0) {
    Serial.begin(BAUDRATE);
  }

  // LMIC init
  os_init();
  LMIC.init();
  // Reset the MAC state. Session and pending data transfers will be discarded.
  LMIC.reset();
  LMIC.setEventCallBack(onEvent);
  SetupLmicKey<appEui, devEui, appKey>::setup(LMIC);

  // set clock error to allow good connection.
  LMIC.setClockError(MAX_CLOCK_ERROR * 3 / 100);
  LMIC.setAntennaPowerAdjustment(-14);

  // Start job (sending automatically starts OTAA too)
  sendjob.setCallbackRunnable(do_send);
}

void loop() {
  OsDeltaTime to_wait = OSS.runloopOnce();
  if (to_wait > OsDeltaTime(0)) {
    // if we have nothing to do just wait a little.
    delay(to_wait.to_ms() / 2);
  }
}