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

namespace {
constexpr uint32_t EU433_FREQ_MIN = 433050000;
constexpr uint32_t EU433_FREQ_MAX = 434665000;
constexpr uint32_t FREQ_DNW2 = EU433_R2;
constexpr LmicEu433::Dr DR_DNW2 = LmicEu433::Dr::SF12;

constexpr uint8_t rps_DR0 = rps_t{SF12, BandWidth::BW125, CodingRate::CR_4_5};
constexpr uint8_t rps_DR1 = rps_t{SF11, BandWidth::BW125, CodingRate::CR_4_5};
constexpr uint8_t rps_DR2 = rps_t{SF10, BandWidth::BW125, CodingRate::CR_4_5};
constexpr uint8_t rps_DR3 = rps_t{SF9, BandWidth::BW125, CodingRate::CR_4_5};
constexpr uint8_t rps_DR4 = rps_t{SF8, BandWidth::BW125, CodingRate::CR_4_5};
constexpr uint8_t rps_DR5 = rps_t{SF7, BandWidth::BW125, CodingRate::CR_4_5};
constexpr uint8_t rps_DR6 = rps_t{SF7, BandWidth::BW250, CodingRate::CR_4_5};

CONST_TABLE(uint8_t, _DR2RPS_CRC)
[] = {ILLEGAL_RPS, rps_DR0, rps_DR1, rps_DR2,    rps_DR3,
      rps_DR4,     rps_DR5, rps_DR6, ILLEGAL_RPS};

// normaly 12.5
constexpr int8_t MaxEIRPValue = 12;

// Table below defines the size of one symbol as
//   symtime = 256us * 2^T(sf,bw)
// 256us is called one symunit.
//                 SF:
//      BW:      |__7___8___9__10__11__12
//      125kHz   |  2   3   4   5   6   7
//      250kHz   |  1   2   3   4   5   6
//      500kHz   |  0   1   2   3   4   5
//
// Times for half symbol per DR
// Per DR table to minimize rounding errors
CONST_TABLE(int32_t, DR2HSYM)
[] = {
    OsDeltaTime::from_us_round(128 << 7).tick(), // DR_SF12
    OsDeltaTime::from_us_round(128 << 6).tick(), // DR_SF11
    OsDeltaTime::from_us_round(128 << 5).tick(), // DR_SF10
    OsDeltaTime::from_us_round(128 << 4).tick(), // DR_SF9
    OsDeltaTime::from_us_round(128 << 3).tick(), // DR_SF8
    OsDeltaTime::from_us_round(128 << 2).tick(), // DR_SF7
    OsDeltaTime::from_us_round(128 << 1).tick(), // DR_SF7B
    OsDeltaTime::from_us_round(80).tick() // FSK -- not used (time for 1/2 byte)
};

} // namespace

uint8_t LmicEu433::getRawRps(dr_t const dr) const {
  return TABLE_GET_U1(_DR2RPS_CRC, dr + 1);
}

int8_t LmicEu433::pow2dBm(uint8_t const powerIndex) const {
  if (powerIndex >= 6) {
    return InvalidPower;
  }

  return MaxEIRP - 2 * powerIndex;
}

OsDeltaTime LmicEu433::dr2hsym(dr_t const dr) const {
  return OsDeltaTime(TABLE_GET_S4(DR2HSYM, dr));
}

bool LmicEu433::validRx1DrOffset(uint8_t const drOffset) const {
  return drOffset < 6;
}

void LmicEu433::initDefaultChannels() {
  LmicDynamicChannel::initDefaultChannels();
  setupChannel(0, EU433_F1, 0);
  setupChannel(1, EU433_F2, 0);
  setupChannel(2, EU433_F3, 0);
}

bool LmicEu433::setupChannel(uint8_t const chidx, uint32_t const newfreq,
                             uint16_t const drmap) {
  if (chidx >= channels.LIMIT_CHANNELS)
    return false;

  channels.configure(chidx, newfreq,
                     drmap == 0 ? dr_range_map(Dr::SF12, Dr::SF7) : drmap);
  return true;
}

uint32_t LmicEu433::convFreq(const uint8_t *ptr) const {
  uint32_t newfreq = rlsbf3(ptr) * 100;
  if (newfreq < EU433_FREQ_MIN || newfreq > EU433_FREQ_MAX)
    newfreq = 0;
  return newfreq;
}

FrequencyAndRate LmicEu433::defaultRX2Parameter() const {
  return {FREQ_DNW2, static_cast<dr_t>(DR_DNW2)};
}

LmicEu433::LmicEu433(Radio &aradio)
    : LmicDynamicChannel(aradio, MaxEIRPValue, 5, 0, bands) {}