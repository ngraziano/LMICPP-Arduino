#include "hal_io.h"
#include "../lmic/lmic.h"
#include "hal.h"
#include <Arduino.h>
#include <SPI.h>

static const SPISettings settings(10E6, MSBFIRST, SPI_MODE0);

HalIo::HalIo(lmic_pinmap const &pins) : lmic_pins(pins) {}

void HalIo::beginspi() const {
  SPI.beginTransaction(settings);
  digitalWrite(lmic_pins.nss, 0);
}

void HalIo::endspi() const {
  digitalWrite(lmic_pins.nss, 1);
  SPI.endTransaction();
}


// perform SPI transaction with radio
uint8_t HalIo::spi(uint8_t const out) const {
  uint8_t res = SPI.transfer(out);
  /*
      Serial.print(">");
      Serial.print(out, HEX);
      Serial.print("<");
      Serial.println(res, HEX);
      */
  return res;
}

static void hal_spi_init() { SPI.begin(); }

void HalIo::pin_rxtx(uint8_t val) const {
  // val == 1  => tx 1
  if (lmic_pins.rxtx != LMIC_UNUSED_PIN)
    digitalWrite(lmic_pins.rxtx, val);
}

// set radio RST pin to given value (or keep floating!)
void HalIo::pin_rst(uint8_t val) const {
  if (lmic_pins.rst == LMIC_UNUSED_PIN)
    return;

  if (val == 0 || val == 1) { // drive pin
    pinMode(lmic_pins.rst, OUTPUT);
    digitalWrite(lmic_pins.rst, val);
  } else { // keep pin floating
    pinMode(lmic_pins.rst, INPUT);
  }
}

uint8_t HalIo::io_check() {
  for (uint8_t i = 0; i < NUM_DIO; ++i) {
    if (lmic_pins.dio[i] == LMIC_UNUSED_PIN)
      continue;
    uint8_t newVal = digitalRead(lmic_pins.dio[i]);
    if (dio_states[i] != newVal) {
      dio_states[i] = newVal;
      if (dio_states[i])  {
        return i;
      }
    }
  }
  return NUM_DIO;
}

void HalIo::init() {
  // NSS and DIO0 are required, DIO1 is required for LoRa
  ASSERT(lmic_pins.nss != LMIC_UNUSED_PIN);
  ASSERT(lmic_pins.dio[0] != LMIC_UNUSED_PIN);
  ASSERT(lmic_pins.dio[1] != LMIC_UNUSED_PIN);

  pinMode(lmic_pins.nss, OUTPUT);
  if (lmic_pins.rxtx != LMIC_UNUSED_PIN)
    pinMode(lmic_pins.rxtx, OUTPUT);
  if (lmic_pins.rst != LMIC_UNUSED_PIN)
    pinMode(lmic_pins.rst, OUTPUT);

  pinMode(lmic_pins.dio[0], INPUT);
  if (lmic_pins.dio[1] != LMIC_UNUSED_PIN)
    pinMode(lmic_pins.dio[1], INPUT);

  // configure radio SPI
  hal_spi_init();
}

