#ifndef _hal_hal_io_h_
#define _hal_hal_io_h_
#pragma once

#include "../lmic/osticks.h"
#include <stdint.h>

constexpr uint8_t NUM_DIO = 2;

using prepare_antenna_type = void (bool);

struct lmic_pinmap {
  uint8_t nss;
  prepare_antenna_type* prepare_antenna_tx;
  uint8_t rst;
  uint8_t dio[NUM_DIO];
};

// Use this for any unused pins.
const uint8_t LMIC_UNUSED_PIN = 0xff;

class HalIo final {
public:
  explicit HalIo(lmic_pinmap const &pins);

  void write_reg(uint8_t addr, uint8_t data) const;
  uint8_t read_reg(uint8_t addr) const;
  void write_buffer(uint8_t addr, uint8_t const *buf, uint8_t len) const;
  void read_buffer(uint8_t addr, uint8_t *buf, uint8_t len) const;

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
   * drive radio RX/TX pins (false=rx, true=tx).
   */
  void pin_switch_antenna_tx(bool isTx) const;

  /**
   * control radio RST pin (0=low, 1=high, 2=floating)
   */
  void pin_rst(uint8_t val) const;

  /**
   * check "interrupt" pin and return if one set to 1.
   */
  bool io_check() const;

  /**
   * Check pin DI0 or busy and return true if set to 1.
   */
  bool io_check0() const;

  /**
   * Check pin DI1 and return true if set to 1.
   */
  bool io_check1() const;

  // configure radio I/O and interrupt handler and SPI
  void init() const;

private:
  const lmic_pinmap &lmic_pins;
};

#endif