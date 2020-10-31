
#include <Arduino.h>
#include <SPI.h>

#include <hal/hal_io.h>
#include <hal/print_debug.h>
#include <keyhandler.h>
#include <lmic.h>

#include "STM32LowPower.h"
#include "lorakeys.h"

// Schedule TX every this many seconds (might become longer due to duty
// cycle limitations).
constexpr uint32_t TX_INTERVAL = 140;

constexpr unsigned int BAUDRATE = 9600;

// Pin mapping
const lmic_pinmap lmic_pins = {
    .nss = RADIO_NSS,
    .prepare_antenna_tx =
        [](bool isTx) {
          if (isTx) {
            digitalWrite(RADIO_RF_CTX_PA, HIGH);
            digitalWrite(RADIO_RF_CBT_HF, LOW);
            digitalWrite(RADIO_RF_CRX_RX, LOW);
          } else {
            digitalWrite(RADIO_RF_CTX_PA, LOW);
            digitalWrite(RADIO_RF_CBT_HF, LOW);
            digitalWrite(RADIO_RF_CRX_RX, HIGH);
          }
        }, // = LMIC_UNUSED_PIN, //
    .rst = RADIO_RESET,
    .dio = {RADIO_DIO_0, RADIO_DIO_1},
};

RadioSx1276 radio{lmic_pins};
LmicEu868 LMIC{radio};

uint32_t nextSendEpoch;

STM32RTC &rtc = STM32RTC::getInstance();

void onEvent(EventType ev) {
  switch (ev) {
  case EventType::JOINING:
    PRINT_DEBUG(2, F("EV_JOINING"));
    //        LMIC.setDrJoin(0);
    break;
  case EventType::JOINED:
    PRINT_DEBUG(2, F("EV_JOINED"));
    // disable ADR because it will be mobile.
    // LMIC.setLinkCheckMode(false);
    break;
  case EventType::JOIN_FAILED:
    PRINT_DEBUG(2, F("EV_JOIN_FAILED"));
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
  // battery
  // uint8_t val = ((uint32_t)analogRead(A1)) * 255 / 683;
  uint32_t bat_value = 100;
  PRINT_DEBUG(1, F("Batterie value %i"), bat_value);
  uint8_t val = bat_value * 255 / 3000;

  // Prepare upstream data transmission at the next possible time.
  LMIC.setTxData2(3, &val, 1, false);
  PRINT_DEBUG(1, F("Packet queued"));
  nextSendEpoch = rtc.getEpoch() + TX_INTERVAL;
}

void setup() {
  LowPower.begin();
  rtc.begin(true);

  if (debugLevel > 0) {
    Serial.begin(BAUDRATE);
  }

  // Enable RF switch Pin control
  pinMode(RADIO_RF_CTX_PA, OUTPUT);
  pinMode(RADIO_RF_CBT_HF, OUTPUT);
  pinMode(RADIO_RF_CRX_RX, OUTPUT);

  // enable XTAL for RF
  pinMode(RADIO_XTAL_EN, OUTPUT);
  digitalWrite(RADIO_XTAL_EN, 1);

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
  // reduce power
  // LMIC.setAntennaPowerAdjustment(-14);

  // first send
  nextSendEpoch = rtc.getEpoch();
}

void goToSleep(uint32_t nb_sec_to_sleep) {
  if (nb_sec_to_sleep < 20) {
    return;
  }
  PRINT_DEBUG(1, F("Sleep %ds"), nb_sec_to_sleep);
  auto const start_sleep_time = rtc.getEpoch();
  LowPower.deepSleep(nb_sec_to_sleep * 1000);
  auto const end_sleep_time = rtc.getEpoch();
  hal_add_time_in_sleep(
      OsDeltaTime::from_sec(end_sleep_time - start_sleep_time));
}

void goToSleep(OsDeltaTime time_to_sleep) { goToSleep(time_to_sleep.to_s()); }

void loop() {

  OsDeltaTime to_wait = LMIC.run();
  if (to_wait < OsDeltaTime::from_ms(100)) {
    // do not try to do something if we have less than 100ms
    return;
  }

  if (LMIC.getOpMode().test(OpState::TXRXPEND)) {
    // a value already waiting to be sent
    // just sleep
    goToSleep(to_wait);
    return;
  }

  int32_t timebeforesend = nextSendEpoch - rtc.getEpoch();
  if (timebeforesend < 0) {
    do_send();
  } else {
    goToSleep(std::min(timebeforesend, to_wait.to_s()));
  }
}