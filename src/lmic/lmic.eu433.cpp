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

//! \file
#include "../hal/print_debug.h"

#include "bufferpack.h"
#include "lmic.eu433.h"
#include "lmic_table.h"
#include <algorithm>

// Default frequency plan for EU 433
enum {
  EU433_F1 = 433175000, // SF7-12
  EU433_F2 = 433375000, // SF7-12
  EU433_F3 = 433575000, // SF7-12
  EU433_R2 = 434665000,
};

namespace EU433 {
constexpr uint32_t FREQ_MIN = 433050000;
constexpr uint32_t FREQ_MAX = 434665000;

extern CONST_TABLE2(uint8_t, _DR2RPS_CRC)[] = {
    rps_DR0, rps_DR1, rps_DR2, rps_DR3, rps_DR4, rps_DR5, rps_DR6};


} // namespace EU433



void Eu433RegionalChannelParams::initDefaultChannels() {
  DynamicRegionalChannelParams::initDefaultChannels();
  setupChannel(0, EU433_F1, 0);
  setupChannel(1, EU433_F2, 0);
  setupChannel(2, EU433_F3, 0);
}

bool Eu433RegionalChannelParams::setupChannel(uint8_t const chidx,
                                              uint32_t const newfreq,
                                              uint16_t const drmap) {
  if (chidx >= channels.LIMIT_CHANNELS)
    return false;

  if (chidx < 3 && drmap != 0) {
    // channel 0, 1 and 2 are fixed
    // drmap == 0 is only used internally to reset the channel
    return false;
  }

  if (newfreq == 0) {
    channels.disable(chidx);
    return true;
  }

  if (newfreq < EU433::FREQ_MIN || newfreq > EU433::FREQ_MAX) {
    return false;
  }

  channels.configure(chidx, newfreq,
                     drmap == 0 ? dr_range_map(Dr::SF12, Dr::SF7) : drmap);
  return true;
}

Eu433RegionalChannelParams::Eu433RegionalChannelParams(LmicRand &arand)
    : DynamicRegionalChannelParams(arand, bands) {}

LmicEu433::LmicEu433(Radio &aradio)
    : Lmic(aradio, aes, rand, channelParams), rand(aes), channelParams(rand) {}
