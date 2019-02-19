#ifndef _hal_hal_io_h_
#define _hal_hal_io_h_
#pragma once

#include "../lmic/osticks.h"
#include <stdint.h>

constexpr uint8_t NUM_DIO = 2;

struct lmic_pinmap {
  uint8_t nss;
  uint8_t rxtx;
  uint8_t rst;
  uint8_t dio[NUM_DIO];
};

// Use this for any unused pins.
const uint8_t LMIC_UNUSED_PIN = 0xff;

class HalIo final {
public:
  explicit HalIo(lmic_pinmap const &pins);

  /**
   * drive radio NSS pin for start transfer.
   */
  void beginspi() const;
  /**
   * drive radio NSS pin to end transfer.
   */
  void endspi() const;
  /**
   * perform 8-bit SPI transaction with radio.
   *   - write given byte 'outval'
   *   - read byte and return value
   */
  uint8_t spi(uint8_t outval) const;

  /**
   * drive radio RX/TX pins (0=rx, 1=tx).
   */
  void pin_rxtx(uint8_t val) const;
  /**
   * control radio RST pin (0=low, 1=high, 2=floating)
   */
  void pin_rst(uint8_t val) const;

  /**
   * check "interrupt" pin and return if one set to 1.
   */
  uint8_t io_check() const;

  // configure radio I/O and interrupt handler and SPI
  void init() const;

private:
  const lmic_pinmap &lmic_pins;
};

#endif