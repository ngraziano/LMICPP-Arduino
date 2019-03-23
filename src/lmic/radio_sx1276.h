/*******************************************************************************
 * Copyright (c) 2014-2015 IBM Corporation.
 * All rights reserved. This program and the accompanying materials
 * are made available under the terms of the Eclipse Public License v1.0
 * which accompanies this distribution, and is available at
 * http://www.eclipse.org/legal/epl-v10.html
 *
 * Contributors:
 *    IBM Zurich Research Lab - initial API, implementation and documentation
 *    Nicolas Graziano - cpp style.
 *******************************************************************************/

#ifndef _radio_sx1276_h_
#define _radio_sx1276_h_

#include "../hal/hal_io.h"
#include "lorabase.h"
#include "osticks.h"
#include "radio.h"
#include <stdint.h>

struct RegSet {
  const uint8_t reg;
  const uint8_t val;

  constexpr uint16_t raw() const { return reg << 8 | val; };
  constexpr RegSet(uint8_t reg, uint8_t val) : reg(reg), val(val){};
  constexpr RegSet(uint16_t raw) : reg(raw >> 8), val(raw & 0xFF){};
};


class RadioSx1276 final : public Radio {

public:
  explicit RadioSx1276(lmic_pinmap const &pins);
  void init(void) override;
  void rst() const override;
  void tx(uint32_t freq, rps_t rps, int8_t txpow, uint8_t const *framePtr,
          uint8_t frameLength) override;
  void rx(uint32_t freq, rps_t rps, uint8_t rxsyms, OsTime rxtime) override;

  void init_random(uint8_t randbuf[16]) override;
  uint8_t handle_end_rx(uint8_t *framePtr) override;
  void handle_end_tx() const override;

  bool io_check() const override;
  uint8_t rssi() const override;

private:
  HalIo hal;

  void opmode(uint8_t mode) const;
  void opmodeLora() const;
  void configLoraModem(rps_t rps);
  void configChannel(uint32_t freq) const;
  void configPower(int8_t pw) const;
  void rxrssi() const;
  void clear_irq() const;
  void write_list_of_reg(uint16_t const * listcmd, uint8_t nb_cmd) const;

};

#endif