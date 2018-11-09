/*******************************************************************************
 * Copyright (c) 2014-2015 IBM Corporation.
 * All rights reserved. This program and the accompanying materials
 * are made available under the terms of the Eclipse Public License v1.0
 * which accompanies this distribution, and is available at
 * http://www.eclipse.org/legal/epl-v10.html
 *
 * Contributors:
 *    IBM Zurich Research Lab - initial API, implementation and documentation
 *******************************************************************************/

//! \file
#include "lmic.us915.h"
#include "bufferpack.h"
#include <algorithm>

enum _dr_us915_t {
  DR_SF10 = 0,
  DR_SF9,
  DR_SF8,
  DR_SF7,
  DR_SF8C,
  DR_NONE,
  // Devices behind a router:
  DR_SF12CR = 8,
  DR_SF11CR,
  DR_SF10CR,
  DR_SF9CR,
  DR_SF8CR,
  DR_SF7CR
};
enum { DR_DFLTMIN = DR_SF8C };

// Default frequency plan for US 915MHz
enum {
  US915_125kHz_UPFBASE = 902300000,
  US915_125kHz_UPFSTEP = 200000,
  US915_500kHz_UPFBASE = 903000000,
  US915_500kHz_UPFSTEP = 1600000,
  US915_500kHz_DNFBASE = 923300000,
  US915_500kHz_DNFSTEP = 600000
};
enum { US915_FREQ_MIN = 902000000, US915_FREQ_MAX = 928000000 };

enum {
  CHNL_PING = 0
}; // used only for default init of state (follows beacon - rotating)
enum {
  FREQ_PING = US915_500kHz_DNFBASE + CHNL_PING * US915_500kHz_DNFSTEP
};                            // default ping freq
enum { DR_PING = DR_SF10CR }; // default ping DR
enum { CHNL_DNW2 = 0 };
enum { FREQ_DNW2 = US915_500kHz_DNFBASE + CHNL_DNW2 * US915_500kHz_DNFSTEP };
enum { DR_DNW2 = DR_SF12CR };
enum {
  CHNL_BCN = 0
}; // used only for default init of state (rotating beacon scheme)
enum { DR_BCN = DR_SF10CR };

#define DNW2_SAFETY_ZONE OsDeltaTime::from_ms(750)

#define maxFrameLen(dr)                                                        \
  ((dr) <= DR_SF11CR ? TABLE_GET_U1(maxFrameLens, (dr)) : 0xFF)
CONST_TABLE(uint8_t, maxFrameLens)
[] = {24, 66, 142, 255, 255, 255, 255, 255, 66, 142};

namespace {
  constexpr uint8_t rps_DR0 = rps_t {SF10, BandWidth::BW125, CodingRate::CR_4_5, false, 0 }.rawValue();
  constexpr uint8_t rps_DR1 = rps_t {SF9, BandWidth::BW125, CodingRate::CR_4_5, false, 0 }.rawValue();
  constexpr uint8_t rps_DR2 = rps_t {SF8, BandWidth::BW125, CodingRate::CR_4_5, false, 0 }.rawValue();
  constexpr uint8_t rps_DR3 = rps_t {SF7, BandWidth::BW125, CodingRate::CR_4_5, false, 0 }.rawValue();
  constexpr uint8_t rps_DR4 = rps_t {SF8, BandWidth::BW500, CodingRate::CR_4_5, false, 0 }.rawValue();

  constexpr uint8_t rps_DR8 = rps_t {SF12, BandWidth::BW500, CodingRate::CR_4_5, false, 0 }.rawValue();
  constexpr uint8_t rps_DR9 = rps_t {SF11, BandWidth::BW500, CodingRate::CR_4_5, false, 0 }.rawValue();
  constexpr uint8_t rps_DR10 = rps_t {SF10, BandWidth::BW500, CodingRate::CR_4_5, false, 0 }.rawValue();
  constexpr uint8_t rps_DR11 = rps_t {SF9, BandWidth::BW500, CodingRate::CR_4_5, false, 0 }.rawValue();
  constexpr uint8_t rps_DR12 = rps_t {SF8, BandWidth::BW500, CodingRate::CR_4_5, false, 0 }.rawValue();
  constexpr uint8_t rps_DR13 = rps_t {SF7, BandWidth::BW500, CodingRate::CR_4_5, false, 0 }.rawValue();

}

CONST_TABLE(uint8_t, _DR2RPS_CRC)
[] = {ILLEGAL_RPS,
      rps_DR0,
      rps_DR1,
      rps_DR2,
      rps_DR3,
      rps_DR4,
      ILLEGAL_RPS,
      ILLEGAL_RPS,
      ILLEGAL_RPS,
      rps_DR8,
      rps_DR9,
      rps_DR10,
      rps_DR11,
      rps_DR12,
      rps_DR13,
      ILLEGAL_RPS};

uint8_t LmicUs915::getRawRps(dr_t dr) const {
  return TABLE_GET_U1(_DR2RPS_CRC, dr + 1);
}

int8_t LmicUs915::pow2dBm(uint8_t mcmd_ladr_p1) const {
  return ((int8_t)(30 - (((mcmd_ladr_p1)&MCMD_LADR_POW_MASK) << 1)));
}

OsDeltaTime LmicUs915::getDwn2SafetyZone() const { return DNW2_SAFETY_ZONE; }

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
static CONST_TABLE(int32_t, DR2HSYM)[] = {
    OsDeltaTime::from_us_round(128 << 5).tick(), // DR_SF10   DR_SF12CR
    OsDeltaTime::from_us_round(128 << 4).tick(), // DR_SF9    DR_SF11CR
    OsDeltaTime::from_us_round(128 << 3).tick(), // DR_SF8    DR_SF10CR
    OsDeltaTime::from_us_round(128 << 2).tick(), // DR_SF7    DR_SF9CR
    OsDeltaTime::from_us_round(128 << 1).tick(), // DR_SF8C   DR_SF8CR
    OsDeltaTime::from_us_round(128 << 0).tick()  // ------    DR_SF7CR
};

// map DR_SFnCR -> 0-6
OsDeltaTime LmicUs915::dr2hsym(dr_t dr) const {
  return OsDeltaTime(TABLE_GET_S4(DR2HSYM, (dr)&7));
}

bool LmicUs915::validRx1DrOffset(uint8_t drOffset) const {
  return drOffset < 4;
}

// ================================================================================
//
// BEG: US915 related stuff
//

void LmicUs915::initDefaultChannels(bool join) {
  // only init in first phase.
  if (!join)
    return;

  for (uint8_t i = 0; i < 4; i++)
    channelMap[i] = 0xFFFF;
  channelMap[4] = 0x00FF;
}

uint32_t LmicUs915::convFreq(const uint8_t *ptr) const {
  uint32_t freq = (rlsbf4(ptr - 1) >> 8) * 100;
  if (freq < US915_FREQ_MIN || freq > US915_FREQ_MAX)
    freq = 0;
  return freq;
}

void LmicUs915::handleCFList(const uint8_t *ptr) {
  // just ignore cflist
}

bool LmicUs915::setupChannel(uint8_t chidx, uint32_t freq, uint16_t drmap,
                             int8_t band) {
  if (chidx < 72 || chidx >= 72 + MAX_XCHANNELS)
    return false; // channels 0..71 are hardwired
  chidx -= 72;
  xchFreq[chidx] = freq;
  xchDrMap[chidx] = drmap == 0 ? DR_RANGE_MAP(DR_SF10, DR_SF8C) : drmap;
  channelMap[chidx >> 4] |= (1 << (chidx & 0xF));
  return true;
}

void LmicUs915::disableChannel(uint8_t channel) {
  if (channel < 72 + MAX_XCHANNELS)
    channelMap[channel >> 4] &= ~(1 << (channel & 0xF));
}

void LmicUs915::enableChannel(uint8_t channel) {
  if (channel < 72 + MAX_XCHANNELS)
    channelMap[channel >> 4] |= (1 << (channel & 0xF));
}

void LmicUs915::enableSubBand(uint8_t band) {
  ASSERT(band < 8);
  uint8_t start = band * 8;
  uint8_t end = start + 8;
  for (int channel = start; channel < end; ++channel)
    enableChannel(channel);
}
void LmicUs915::disableSubBand(uint8_t band) {
  ASSERT(band < 8);
  uint8_t start = band * 8;
  uint8_t end = start + 8;
  for (int channel = start; channel < end; ++channel)
    disableChannel(channel);
}
void LmicUs915::selectSubBand(uint8_t band) {
  ASSERT(band < 8);
  for (int b = 0; b < 8; ++b) {
    if (band == b)
      enableSubBand(b);
    else
      disableSubBand(b);
  }
}

uint8_t LmicUs915::mapChannels(uint8_t chMaskCntl, uint16_t chMask) {
  if (chMaskCntl == MCMD_LADR_CHP_125ON || chMaskCntl == MCMD_LADR_CHP_125OFF) {
    uint16_t en125 = chMaskCntl == MCMD_LADR_CHP_125ON ? 0xFFFF : 0x0000;
    for (uint8_t u = 0; u < 4; u++)
      channelMap[u] = en125;
    channelMap[64 / 16] = chMask;
  } else {
    if (chMaskCntl >= (72 + MAX_XCHANNELS + 15) / 16)
      return 0;
    channelMap[chMaskCntl] = chMask;
  }
  return 1;
}

void LmicUs915::updateTx(OsTime txbeg, OsDeltaTime airtime) {
  uint8_t chnl = txChnl;
  if (chnl < 64) {
    freq = US915_125kHz_UPFBASE + chnl * US915_125kHz_UPFSTEP;
    txpow = 30;
    return;
  }
  txpow = 26;
  if (chnl < 64 + 8) {
    freq = US915_500kHz_UPFBASE + (chnl - 64) * US915_500kHz_UPFSTEP;
  } else {
    ASSERT(chnl < 64 + 8 + MAX_XCHANNELS);
    freq = xchFreq[chnl - 72];
  }

  // Update global duty cycle stats
  if (globalDutyRate != 0) {
    globalDutyAvail = txbeg + OsDeltaTime(airtime.tick() << globalDutyRate);
  }
}

// US does not have duty cycling - return now as earliest TX time
OsTime LmicUs915::nextTx(OsTime now) {
  if (chRnd == 0)
    chRnd = rand.uint8() & 0x3F;
  if (datarate >= DR_SF8C) { // 500kHz
    uint8_t map = channelMap[64 / 16] & 0xFF;
    for (uint8_t i = 0; i < 8; i++) {
      if ((map & (1 << (++chRnd & 7))) != 0) {
        txChnl = 64 + (chRnd & 7);
        return now;
      }
    }
  } else { // 125kHz
    for (uint8_t i = 0; i < 64; i++) {
      uint8_t chnl = ++chRnd & 0x3F;
      if ((channelMap[(chnl >> 4)] & (1 << (chnl & 0xF))) != 0) {
        txChnl = chnl;
        return now;
      }
    }
  }
  // No feasible channel  found! Keep old one.
  return now;
}

void LmicUs915::setRx1Params() {
  // TODO handle offset
  freq = US915_500kHz_DNFBASE + (txChnl & 0x7) * US915_500kHz_DNFSTEP;
  if (/* TX datarate */ dndr < DR_SF8C)
    dndr += DR_SF10CR - DR_SF10;
  else if (dndr == DR_SF8C)
    dndr = DR_SF7CR;
}

#if !defined(DISABLE_JOIN)
void LmicUs915::initJoinLoop() {
  chRnd = 0;
  txChnl = 0;
  adrTxPow = 20;
  ASSERT(!(opmode & OpState::NEXTCHNL));
  txend = os_getTime();
  setDrJoin(DR_SF7);
}

bool LmicUs915::nextJoinState() {
  // Try the following:
  //   SF7/8/9/10  on a random channel 0..63
  //   SF8C        on a random channel 64..71
  //
  bool failed = false;
  if (datarate != DR_SF8C) {
    txChnl = 64 + (txChnl & 7);
    datarate = DR_SF8C;
  } else {
    txChnl = rand.uint8() & 0x3F;
    int8_t dr = DR_SF7 - ++txCnt;
    if (dr < DR_SF10) {
      dr = DR_SF10;
      failed = true; // All DR exhausted - signal failed
    }
    datarate = dr;
  }

  txend = os_getTime() + (isTESTMODE()
                              // Avoid collision with JOIN ACCEPT being sent by
                              // GW (but we missed it - GW is still busy)
                              ? DNW2_SAFETY_ZONE
                              // Otherwise: randomize join (street lamp case):
                              // SF10:16, SF9=8,..SF8C:1secs
                              : OsDeltaTime::rnd_delay(rand, 16 >> datarate));
  // 1 - triggers EV_JOIN_FAILED event
  return !failed;
}
#endif // !DISABLE_JOIN

dr_t LmicUs915::defaultRX2Dr() const { return DR_DNW2; }
uint32_t LmicUs915::defaultRX2Freq() const { return FREQ_DNW2; }

LmicUs915::LmicUs915(lmic_pinmap const& pins) : Lmic(pins) {};
