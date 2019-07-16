#include "hal_io.h"
#include "../lmic/lmic.h"
#include "hal.h"
#include <Arduino.h>
#include <SPI.h>

static const SPISettings settings(10000000, MSBFIRST, SPI_MODE0);

HalIo::HalIo(lmic_pinmap const &pins) : lmic_pins(pins) {}

void HalIo::write_reg(uint8_t const addr, uint8_t const data) const {
  beginspi();
  spi(addr | 0x80);
  spi(data);
  endspi();
}

uint8_t HalIo::read_reg(uint8_t const addr) const {
  beginspi();
  spi(addr & 0x7F);
  uint8_t const val = spi(0x00);
  endspi();
  return val;
}

void HalIo::write_buffer(uint8_t const addr, uint8_t const *const buf,
                         uint8_t const len) const {
  beginspi();
  spi(addr | 0x80);
  for (uint8_t i = 0; i < len; i++) {
    spi(buf[i]);
  }
  endspi();
}

void HalIo::read_buffer(uint8_t const addr, uint8_t *const buf,
                        uint8_t const len) const {
  beginspi();
  spi(addr & 0x7F);
  for (uint8_t i = 0; i < len; i++) {
    buf[i] = spi(0x00);
  }
  endspi();
}

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

bool HalIo::io_check() const {
  for (uint8_t i = 0; i < NUM_DIO; ++i) {
    uint8_t newVal = digitalRead(lmic_pins.dio[i]);
    if (newVal) {
      return true;
    }
  }
  return false;
}

bool HalIo::io_check0() const {
  return digitalRead(lmic_pins.dio[0]) ? true : false ;
}

bool HalIo::io_check1() const {
  return digitalRead(lmic_pins.dio[1]) ? true : false ;
}

void HalIo::init() const {
  // NSS, DIO0 , DIO1 are required for LoRa
  ASSERT(lmic_pins.nss != LMIC_UNUSED_PIN);
  ASSERT(lmic_pins.dio[0] != LMIC_UNUSED_PIN);
  ASSERT(lmic_pins.dio[1] != LMIC_UNUSED_PIN);

  pinMode(lmic_pins.nss, OUTPUT);
  if (lmic_pins.rxtx != LMIC_UNUSED_PIN)
    pinMode(lmic_pins.rxtx, OUTPUT);
  if (lmic_pins.rst != LMIC_UNUSED_PIN)
    pinMode(lmic_pins.rst, OUTPUT);

  pinMode(lmic_pins.dio[0], INPUT);
  pinMode(lmic_pins.dio[1], INPUT);

  // configure radio SPI
}
