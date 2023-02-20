#include "../boardconfig.h"
#if LMIC_HAL_IO == LMIC_GENERIC

#include "../lmic/lmic.h"
#include "hal.h"
#include "hal_io.h"

#include <algorithm>
#include <hal/print_debug.h>

HalIo::HalIo(lmic_pinmap const &pins) : lmic_pins(pins) {}

void HalIo::yield() const {}

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
  std::generate_n(buf, len, [this]() { return spi(0x00); });
  endspi();
}

void HalIo::beginspi() const {}

void HalIo::endspi() const {}

// perform SPI transaction with radio
uint8_t HalIo::spi(uint8_t const) const { return 0; }

void HalIo::pin_switch_antenna_tx(bool isTx) const {
  if (lmic_pins.prepare_antenna_tx)
    lmic_pins.prepare_antenna_tx(isTx);
}

// set radio RST pin to given value (or keep floating!)
void HalIo::pin_rst(uint8_t val) const {}

bool HalIo::io_check() const { return true; }

bool HalIo::io_check0() const { return true; }

bool HalIo::io_check1() const { return true; }

void HalIo::init() const {}

#endif