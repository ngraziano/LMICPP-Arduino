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
#include "lmic.eu868.h"
#include "lmic_table.h"
#include <algorithm>

// Default frequency plan for EU 868MHz ISM band
// Bands:
//  g1 :   1%  14dBm
//  g2 : 0.1%  14dBm
//  g3 :  10%  27dBm
//                 freq             band     datarates
enum {
  EU868_F1 = 868100000, // g1   SF7-12
  EU868_F2 = 868300000, // g1   SF7-12 FSK SF7/250
  EU868_F3 = 868500000, // g1   SF7-12
  EU868_F4 = 868850000, // g2   SF7-12
  EU868_F5 = 869050000, // g2   SF7-12
  EU868_F6 = 869525000, // g3   SF7-12
};

namespace {
constexpr uint32_t EU868_FREQ_MIN = 863000000;
constexpr uint32_t EU868_FREQ_MAX = 870000000;
constexpr uint32_t FREQ_DNW2 = EU868_F6;
constexpr Eu868RegionalChannelParams::Dr DR_DNW2 =
    Eu868RegionalChannelParams::Dr::SF12;

constexpr OsDeltaTime DNW2_SAFETY_ZONE = OsDeltaTime::from_ms(3000);
constexpr rps_t rps_DNW2 =
    rps_t{SF12, BandWidth::BW125, CodingRate::CR_4_5, true};

constexpr uint8_t rps_DR0 = rps_t{SF12, BandWidth::BW125, CodingRate::CR_4_5};
constexpr uint8_t rps_DR1 = rps_t{SF11, BandWidth::BW125, CodingRate::CR_4_5};
constexpr uint8_t rps_DR2 = rps_t{SF10, BandWidth::BW125, CodingRate::CR_4_5};
constexpr uint8_t rps_DR3 = rps_t{SF9, BandWidth::BW125, CodingRate::CR_4_5};
constexpr uint8_t rps_DR4 = rps_t{SF8, BandWidth::BW125, CodingRate::CR_4_5};
constexpr uint8_t rps_DR5 = rps_t{SF7, BandWidth::BW125, CodingRate::CR_4_5};
constexpr uint8_t rps_DR6 = rps_t{SF7, BandWidth::BW250, CodingRate::CR_4_5};

CONST_TABLE(uint8_t, _DR2RPS_CRC)
[] = {rps_DR0, rps_DR1, rps_DR2, rps_DR3, rps_DR4, rps_DR5, rps_DR6};

constexpr int8_t MaxEIRPValue = 16;

} // namespace

int8_t Eu868RegionalChannelParams::pow2dBm(uint8_t const powerIndex) const {
  if (powerIndex >= 8) {
    return InvalidPower;
  }

  return MaxEIRP - 2 * powerIndex;
}

bool Eu868RegionalChannelParams::validRx1DrOffset(
    uint8_t const drOffset) const {
  return drOffset < 6;
}

void Eu868RegionalChannelParams::initDefaultChannels() {
  DynamicRegionalChannelParams::initDefaultChannels();
  setupChannel(0, EU868_F1, 0);
  setupChannel(1, EU868_F2, 0);
  setupChannel(2, EU868_F3, 0);
}

bool Eu868RegionalChannelParams::setupChannel(uint8_t const chidx,
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

  if (newfreq < EU868_FREQ_MIN || newfreq > EU868_FREQ_MAX) {
    return false;
  }

  channels.configure(chidx, newfreq,
                     drmap == 0 ? dr_range_map(Dr::SF12, Dr::SF7) : drmap);
  return true;
}

void Eu868RegionalChannelParams::resetRX2Parameter() {
  rx2Parameter = {FREQ_DNW2, rps_DNW2, 0};
}

Eu868RegionalChannelParams::Eu868RegionalChannelParams(LmicRand &arand)
    : DynamicRegionalChannelParams(arand, MaxEIRPValue, 5, 0,
                                   RESOLVE_TABLE(_DR2RPS_CRC), 7, bandeu) {}

LmicEu868::LmicEu868(Radio &aradio)
    : Lmic(aradio, aes, rand, channelParams), rand(aes), channelParams(rand) {}