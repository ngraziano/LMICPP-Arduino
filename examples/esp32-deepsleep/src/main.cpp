#include <Arduino.h>
#include <SPI.h>

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
    .prepare_antenna_tx = nullptr,
    .rst = 14,
    .dio = {26, 33},
};
RadioSx1276 radio{lmic_pins};
LmicEu868 LMIC{radio};

OsTime nextSend;

// buffer to save current lmic state (size may be reduce)
RTC_DATA_ATTR uint8_t saveState[301];

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
  case EventType::TXCOMPLETE: {
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
    // save before going to deep sleep.
    auto store = StoringBuffer{saveState};
    LMIC.saveState(store);
    saveState[300] = 51;
    PRINT_DEBUG(1, F("State save len = %i"), store.length());
    ESP.deepSleep(TX_INTERVAL.to_us());
    break;
  }
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
  // Some analog value
  // val = analogRead(A1) >> 4;
  uint8_t val = temperatureRead();
  // Prepare upstream data transmission at the next possible time.
  LMIC.setTxData2(2, &val, 1, false);
  PRINT_DEBUG(1, F("Packet queued"));
  nextSend = os_getTime() + TX_INTERVAL;
}

void setup() {

  if (debugLevel > 0) {
    Serial.begin(BAUDRATE);
  }

  SPI.begin();
  // LMIC init
  os_init();
  LMIC.init();
  // Reset the MAC state. Session and pending data transfers will be discarded.
  LMIC.reset();
  LMIC.setEventCallBack(onEvent);
  SetupLmicKey<appEui, devEui, appKey>::setup(LMIC);

  // set clock error to allow good connection.
  LMIC.setClockError(MAX_CLOCK_ERROR * 3 / 100);
  // LMIC.setAntennaPowerAdjustment(-14);

  if (saveState[300] == 51) {
    auto retrieve = RetrieveBuffer{saveState};
    LMIC.loadState(retrieve);
    // PRINT_DEBUG(1, F("State load len = %i"), lbuf);
    saveState[300] = 0;
  }
  // Start job (sending automatically starts OTAA too)
  nextSend = os_getTime();
}

void loop() {

  OsDeltaTime freeTimeBeforeNextCall = LMIC.run();

  if (freeTimeBeforeNextCall > OsDeltaTime::from_ms(10)) {
    // we have more than 10 ms to do some work.
    // the test must be adapted from the time spend in other task
    if (nextSend < os_getTime()) {
      if (LMIC.getOpMode().test(OpState::TXRXPEND)) {
        PRINT_DEBUG(1, F("OpState::TXRXPEND, not sending"));
      } else {
        do_send();
      }
    } else {
      OsDeltaTime freeTimeBeforeSend = nextSend - os_getTime();
      OsDeltaTime to_wait =
          std::min(freeTimeBeforeNextCall, freeTimeBeforeSend);
      delay(to_wait.to_ms() / 2);
    }
  }
}