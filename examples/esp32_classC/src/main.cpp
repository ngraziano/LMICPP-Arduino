
#include <Arduino.h>
#include <SPI.h>

#include <hal/hal_io.h>
#include <hal/print_debug.h>
#include <keyhandler.h>
#include <lmic.h>

#include "SSD1306Wire.h"

#include "lorakeys.h"

SSD1306Wire display(0x3c, SDA_OLED, SCL_OLED, GEOMETRY_128_64);

void do_send();

// Schedule TX every this many seconds (might become longer due to duty
// cycle limitations).
constexpr OsDeltaTime TX_INTERVAL = OsDeltaTime::from_sec(135);

constexpr unsigned int BAUDRATE = 19200;
constexpr uint8_t button_pin = 3;
// Pin mapping
constexpr lmic_pinmap lmic_pins = {
    .nss = SS,
    .prepare_antenna_tx = nullptr,
    .rst = RST_LoRa,
    .dio = {DIO0, DIO1},
};

RadioSx1276 radio{lmic_pins};
LmicEu868 LMIC{radio};

// Time of next send of data.
OsTime nextSend;

void showState(const String &value) {
  display.setColor(BLACK);
  display.fillRect(0, 0, 128, 40);
  display.setColor(WHITE);

  display.drawString(0, 0, value);
  char buffer[128];
  display.drawStringf(0, 20, buffer, "rssi %i", radio.get_last_packet_rssi());
  display.display();
}

void showText(const char *value) {
  display.setColor(BLACK);
  display.fillRect(0, 40, 128, 20);
  display.setColor(WHITE);
  display.drawString(0, 40, value);
  display.display();
}

void onEvent(EventType ev) {
  switch (ev) {
  case EventType::JOINING:
    PRINT_DEBUG(2, F("EV_JOINING"));
    showState("Joining");

    // We can change here the setting for joining
    // LMIC.setDrJoin(0);
    break;
  case EventType::JOINED:
    PRINT_DEBUG(2, F("EV_JOINED"));
    showState("Joined");

    // disable ADR because if will be mobile.
    // LMIC.setLinkCheckMode(false);
    break;
  case EventType::JOIN_FAILED:
    PRINT_DEBUG(2, F("EV_JOIN_FAILED"));
    showState("Join failed");

    break;
  case EventType::TXCOMPLETE: {
    PRINT_DEBUG(1, F("EV_TXCOMPLETE (includes waiting for RX windows)"));
    showState("Tx complete");

    if (LMIC.getTxRxFlags().test(TxRxStatus::ACK)) {
      PRINT_DEBUG(1, F("Received ack"));
    }
    if (LMIC.getDataLen()) {
      PRINT_DEBUG(1, F("Received %d bytes of payload"), LMIC.getDataLen());
      auto data = LMIC.getData();
      if (data) {
        uint8_t port = LMIC.getPort();
        if (port == 10) {
          showText(reinterpret_cast<const char *>(data));
        }
      }
    }

    break;
  }
  case EventType::RESET:
    PRINT_DEBUG(1, F("EV_RESET"));
    showState("Reset");

    break;
  case EventType::LINK_DEAD:
    PRINT_DEBUG(1, F("EV_LINK_DEAD"));
    showState("Link dead");

    break;
  case EventType::LINK_ALIVE:
    PRINT_DEBUG(1, F("EV_LINK_ALIVE"));
    showState("Link alive");

    break;
  case EventType::RXC:
    PRINT_DEBUG(1, F("EV_RXC receive a class C downlink message"));
    showState("Receive class C");

    if (LMIC.getDataLen()) {
      PRINT_DEBUG(1, F("Received %d bytes of payload"), LMIC.getDataLen());
      auto data = LMIC.getData();
      if (data) {
        uint8_t port = LMIC.getPort();
        if (port == 10) {
          showText(reinterpret_cast<const char *>(data));
        }
      }
    }
    break;
  default:
    PRINT_DEBUG(1, F("Unknown event"));
    break;
  }
}

void do_send() {

  // The data to transmit, (here a fake constant value)
  uint8_t val = 51;

  // Prepare upstream data transmission at the next possible time.
  LMIC.setTxData2(3, &val, 1, false);
  PRINT_DEBUG(1, F("Packet queued"));

  // Set time time of the next send.
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

  // Display setup
  PRINT_DEBUG(1, F("Init display"));
  pinMode(Vext, OUTPUT);
  digitalWrite(Vext, LOW);

  pinMode(RST_OLED, OUTPUT);
  digitalWrite(RST_OLED, LOW);
  delay(500);
  digitalWrite(RST_OLED, HIGH);

  display.init();
  display.flipScreenVertically();
  display.setFont(ArialMT_Plain_16);
  display.setContrast(255);
  showState("Setup in 10s");
  PRINT_DEBUG(1, F("Init display done"));

  delay(10000);

  showState("Setup");

  // Reset the MAC state. Session and pending data transfers will be discarded.
  LMIC.reset();
  SetupLmicKey<appEui, devEui, appKey>::setup(LMIC);

  LMIC.setEventCallBack(onEvent);
  // set clock error to allow good connection.
  LMIC.setClockError(MAX_CLOCK_ERROR * 1 / 100);

  // reduce power (if neeeded)
  // LMIC.setAntennaPowerAdjustment(-2);

  // listen between tx
  LMIC.activateClassC();

  // Set next send time to now (sending automatically starts OTAA too)
  nextSend = os_getTime();

  showState("Setup end");
}

void loop() {
  // Let's LMIC do it's job
  OsDeltaTime freeTimeBeforeNextCall = LMIC.run();

  if (freeTimeBeforeNextCall > OsDeltaTime::from_ms(50)) {
    // we have more than 50 ms so we do some work.
    // the test must be adapted from the time spend in other task

    // Check if we want to send data.
    if (nextSend < os_getTime()) {
      if (LMIC.getOpMode().test(OpState::TXRXPEND)) {
        // some data is already scheduled to be send
        // we don't replace it.
        PRINT_DEBUG(1, F("OpState::TXRXPEND, not sending"));
      } else {
        // enque some data to be send
        do_send();
      }
    } else {
      // We do not wait here because we want LMIC to check if we receive some
      // class C message
    }
  }
}