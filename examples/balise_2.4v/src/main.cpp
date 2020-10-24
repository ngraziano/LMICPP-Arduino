#include <Arduino.h>
#include <SPI.h>
#include <avr/power.h>

#include <hal/hal_io.h>
#include <hal/print_debug.h>
#include <keyhandler.h>
#include <lmic.h>

#include <algorithm>
#include <sleepandwatchdog.h>

#define DEVICE_TEMP1
#include "lorakeys.h"
#include "powersave.h"

void do_send();
void reset_and_do_send();

// Schedule TX every this many seconds (might become longer due to duty
// cycle limitations).
constexpr OsDeltaTime TX_INTERVAL = OsDeltaTime::from_sec(135);

constexpr unsigned int BAUDRATE = 9600;
constexpr uint8_t button_pin = 3;
// Pin mapping
constexpr lmic_pinmap lmic_pins = {
    .nss = 10,
    .prepare_antenna_tx = nullptr,
    .rst = 14,
    .dio = {9, 8},
};

RadioSx1276 radio{lmic_pins};
LmicEu868 LMIC{radio};

OsTime nextSend;

bool new_click = false;
bool send_now = false;

void onEvent(EventType ev) {
  rst_wdt();
  switch (ev) {
  case EventType::JOINING:
    PRINT_DEBUG(2, F("EV_JOINING"));
    //        LMIC.setDrJoin(0);
    break;
  case EventType::JOINED:
    PRINT_DEBUG(2, F("EV_JOINED"));
    // disable ADR because it will be mobile.
    LMIC.setLinkCheckMode(false);
    // LMIC.setDrTx(0);
    LMIC.setDutyRate(12);
    break;
  case EventType::JOIN_FAILED:
    PRINT_DEBUG(2, F("EV_JOIN_FAILED"));
    break;
  case EventType::TXCOMPLETE:
    PRINT_DEBUG(2, F("EV_TXCOMPLETE (includes waiting for RX windows)"));
    send_now = false;
    if (LMIC.getTxRxFlags().test(TxRxStatus::ACK)) {
      PRINT_DEBUG(1, F("Received ack"));
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

uint16_t read_vcc() {
  // -Selects AVcc external reference
  // REFS1 REFS0          --> 0 1, AVcc internal ref.
  // -Selects channel 14, bandgap voltage, to measure
  // MUX3 MUX2 MUX1 MUX0  --> 1110 1.1V (VBG)
  ADMUX = (0 << REFS1) | (1 << REFS0) | (0 << ADLAR) | (1 << MUX3) |
          (1 << MUX2) | (1 << MUX1) | (0 << MUX0);
  // Let Vref settle
  delay(1);
  // start the conversion
  ADCSRA |= (1 << ADSC);
  while (bit_is_set(ADCSRA, ADSC))
    ;
  return (1100UL * 1023 / ADC);
}

void do_send() {

  // battery
  uint32_t bat_value = read_vcc();
  PRINT_DEBUG(1, F("Batterie value %i"), bat_value);
  uint8_t val = bat_value * 255 / 3000;

  if (LMIC.getTxRxFlags().test(TxRxStatus::NEED_BATTERY_LEVEL)) {
    LMIC.setBatteryLevel(val);
  }

  // Prepare upstream data transmission at the next possible time.
  LMIC.setTxData2(3, &val, 1, false);
  PRINT_DEBUG(1, F("Packet queued"));
  nextSend = os_getTime() + TX_INTERVAL;

}

// lmic_pins.dio[0]  = 9 => PCINT1
// lmic_pins.dio[1]  = 8 => PCINT0
// PCI2 PCINT[23:16]
// PCI1 PCINT[14:8]
// PCI0 PCINT[7:0]

ISR(PCINT0_vect) {
  // one of pins D8 to D13 has changed
  // store time, will be check in OSS.runloopOnce()
  LMIC.store_trigger();
}

void pciSetup(byte pin) {
  *digitalPinToPCMSK(pin) |= bit(digitalPinToPCMSKbit(pin)); // enable pin
  PCIFR |= bit(digitalPinToPCICRbit(pin)); // clear any outstanding interrupt
  PCICR |= bit(digitalPinToPCICRbit(pin)); // enable interrupt for the group
}

void testDuration(int32_t ms) {
  const auto delta = OsDeltaTime::from_ms(ms);
  PRINT_DEBUG(1, F("Test sleep time for %i ms."), ms);
  const OsTime start = os_getTime();
  PRINT_DEBUG(1, F("Start Test sleep time."));
  powersave(delta, []() { return false; });
  const OsTime end = os_getTime();
  PRINT_DEBUG(1, F("End Test sleep time."));
  PRINT_DEBUG(1, F("Test Time should be : %d ms"), (end - start).to_ms());
}

void buttonInterupt() {
  // Do nothing if send is already scheduled.
  if (send_now) {
    return;
  }
  if (digitalRead(button_pin) == 0) {
    new_click = true;
  }
}

void setup() {
  // To handle VCC <= 2.4v
  // clock start at 8MHz / 8 => 1 MHz
  // set clock to 8MHz / 4 => 2MHz
  // maybe 4Mhz could also work
  clock_prescale_set(clock_div_4);

  if (debugLevel > 0) {
    Serial.begin(BAUDRATE);
  }

  pciSetup(lmic_pins.dio[0]);
  pciSetup(lmic_pins.dio[1]);

  pinMode(button_pin, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(button_pin), &buttonInterupt, FALLING);

  SPI.begin();
  // LMIC init
  os_init();
  LMIC.init();
  // Reset the MAC state. Session and pending data transfers will be discarded.
  LMIC.reset();
  LMIC.setEventCallBack(onEvent);
  SetupLmicKey<appEui, devEui, appKey>::setup(LMIC);

  // set clock error to allow good connection.
  LMIC.setClockError(MAX_CLOCK_ERROR * 2 / 100);
  // LMIC.setAntennaPowerAdjustment(-14);

  // Only work with special boot loader.
  configure_wdt();

  // test duration and in case of reboot loop  prevent flood
  // testDuration(1000);
  // testDuration(8000);
  testDuration(30000);

  // Start job (sending automatically starts OTAA too)
  nextSend = os_getTime();
}

void loop() {
  rst_wdt();
  OsDeltaTime freeTimeBeforeNextCall = LMIC.run();

  if (freeTimeBeforeNextCall > OsDeltaTime::from_ms(10)) {
    // we have more than 10 ms to do some work.
    // the test must be adapted from the time spend in other task
    if (nextSend < os_getTime() || new_click) {
      if (LMIC.getOpMode().test(OpState::TXRXPEND)) {
        PRINT_DEBUG(1, F("OpState::TXRXPEND, not sending"));
      } else {
        send_now = true;
        new_click = false;
        do_send();
      }
    } else {
      OsDeltaTime freeTimeBeforeSend = nextSend - os_getTime();
      OsDeltaTime to_wait =
          std::min(freeTimeBeforeNextCall, freeTimeBeforeSend);
      // Go to sleep if we have nothing to do.
      powersave(to_wait, []() {
        buttonInterupt();
        return new_click;
      });
    }
  }

}