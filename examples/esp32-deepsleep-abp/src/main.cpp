
#include <Arduino.h>
#include <SPI.h>

#include <hal/hal_io.h>
#include <hal/print_debug.h>
#include <keyhandler.h>
#include <lmic.h>

void do_send();

// Schedule TX every this many seconds (might become longer due to duty
// cycle limitations).
constexpr OsDeltaTime TX_INTERVAL = OsDeltaTime::from_sec(135);

constexpr unsigned int BAUDRATE = 19200;
constexpr uint8_t button_pin = 3;
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
const size_t SAVE_BUFFER_SIZE = 200;
RTC_DATA_ATTR uint8_t saveState[SAVE_BUFFER_SIZE];

void onEvent(EventType ev) {
  switch (ev) {
  case EventType::JOINING:
    PRINT_DEBUG(2, F("EV_JOINING"));
    break;
  case EventType::JOINED:
    PRINT_DEBUG(2, F("EV_JOINED"));
    break;
  case EventType::JOIN_FAILED:
    PRINT_DEBUG(2, F("EV_JOIN_FAILED"));
    break;
  case EventType::TXCOMPLETE: {
    PRINT_DEBUG(1, F("EV_TXCOMPLETE (includes waiting for RX windows)"));
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
    saveState[SAVE_BUFFER_SIZE - 1] = 51;
    PRINT_DEBUG(1, F("State save len = %i"), store.length());
    ESP.deepSleep(TX_INTERVAL.to_us());

    break;
  }
  case EventType::RESET:
    PRINT_DEBUG(1, F("EV_RESET"));
    break;
  case EventType::LINK_DEAD:
    PRINT_DEBUG(1, F("EV_LINK_DEAD"));
    break;
  case EventType::LINK_ALIVE:
    PRINT_DEBUG(1, F("EV_LINK_ALIVE"));
    break;
  default:
    PRINT_DEBUG(1, F("Unknown event"));
    break;
  }
}

void do_send() {
  uint8_t val = 51;
  // Prepare upstream data transmission at the next possible time.
  LMIC.setTxData2(3, &val, 1, false);
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

  // ABP Set session information
  // Change to your device info
  const uint32_t TTN_NET_ID = 0x000013;
  const uint32_t DEV_ADRESS = 0x26010000;
  const uint8_t NET_KEY[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
                             0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
  const uint8_t APP_KEY[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
                             0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
  AesKey appkey;
  std::copy(APP_KEY, APP_KEY + 16, appkey.begin());
  AesKey netkey;
  std::copy(NET_KEY, NET_KEY + 16, netkey.begin());
  LMIC.setSession(TTN_NET_ID, DEV_ADRESS, netkey, appkey);

  // SF9 rx2
  LMIC.setRx2Parameter(869525000, 3);

  // Channel 0,1,2 : default channel for EU868
  // LMIC.setupChannel(0, 868100000, dr_range_map(0, 5));
  // LMIC.setupChannel(1, 868300000, dr_range_map(0, 5));
  // LMIC.setupChannel(2, 868500000, dr_range_map(0, 5));
  LMIC.setupChannel(3, 867100000, dr_range_map(0, 5));
  LMIC.setupChannel(4, 867300000, dr_range_map(0, 5));
  LMIC.setupChannel(5, 867500000, dr_range_map(0, 5));
  LMIC.setupChannel(6, 867700000, dr_range_map(0, 5));
  LMIC.setupChannel(7, 867900000, dr_range_map(0, 5));

  // Tx Datarate for EU868  0 => SF12 ... 5 => SF7
  LMIC.setDrTx(5);

  LMIC.setEventCallBack(onEvent);
  // set clock error to allow good connection.
  LMIC.setClockError(MAX_CLOCK_ERROR * 1 / 100);
  // reduce power
  // LMIC.setAntennaPowerAdjustment(-14);

  // Restore saved state
  if (saveState[SAVE_BUFFER_SIZE - 1] == 51) {
    auto retrieve = RetrieveBuffer{saveState};
    LMIC.loadState(retrieve);
    saveState[SAVE_BUFFER_SIZE - 1] = 0;
  }

  // Start job to send data
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