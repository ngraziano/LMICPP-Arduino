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
#include "lmic.eu868.h"
#include "bufferpack.h"
#include <algorithm>

enum class Eu868Dr : dr_t {
  SF12 = 0,
  SF11,
  SF10,
  SF9,
  SF8,
  SF7,
  SF7B,
  FSK,
  NONE
};
const Eu868Dr DR_DFLTMIN = Eu868Dr::SF7;

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
  EU868_J4 = 864100000, // g2   SF7-12  used during join
  EU868_J5 = 864300000, // g2   SF7-12   ditto
  EU868_J6 = 864500000, // g2   SF7-12   ditto
};
enum { EU868_FREQ_MIN = 863000000, EU868_FREQ_MAX = 870000000 };

enum { CHNL_PING = 5 };
enum { FREQ_PING = EU868_F6 }; // default ping freq
enum { CHNL_DNW2 = 5 };
enum { FREQ_DNW2 = EU868_F6 };
const Eu868Dr DR_DNW2 = Eu868Dr::SF12;

namespace {
  constexpr OsDeltaTime  DNW2_SAFETY_ZONE = OsDeltaTime::from_ms(3000);
}


#define maxFrameLen(dr)                                                        \
  ((dr) <= Eu868Dr::SF9 ? TABLE_GET_U1(maxFrameLens, (dr)) : 0xFF)
CONST_TABLE(uint8_t, maxFrameLens)[] = {64, 64, 64, 123};

namespace {
  constexpr uint8_t rps_DR0 = rps_t {SF12, BandWidth::BW125, CodingRate::CR_4_5, false,0 }.rawValue();
  constexpr uint8_t rps_DR1 = rps_t {SF11, BandWidth::BW125, CodingRate::CR_4_5, false,0 }.rawValue();
  constexpr uint8_t rps_DR2 = rps_t {SF10, BandWidth::BW125, CodingRate::CR_4_5, false,0 }.rawValue();
  constexpr uint8_t rps_DR3 = rps_t {SF9, BandWidth::BW125, CodingRate::CR_4_5, false,0 }.rawValue();
  constexpr uint8_t rps_DR4 = rps_t {SF8, BandWidth::BW125, CodingRate::CR_4_5, false,0 }.rawValue();
  constexpr uint8_t rps_DR5 = rps_t {SF7, BandWidth::BW125, CodingRate::CR_4_5, false,0 }.rawValue();
  constexpr uint8_t rps_DR6 = rps_t {SF7, BandWidth::BW250, CodingRate::CR_4_5, false,0 }.rawValue();
  constexpr uint8_t rps_DR7 = rps_t {FSK, BandWidth::BW125, CodingRate::CR_4_5, false,0 }.rawValue();
}

CONST_TABLE(uint8_t, _DR2RPS_CRC)
[] = {ILLEGAL_RPS,
      rps_DR0,
      rps_DR1,
      rps_DR2,
      rps_DR3,
      rps_DR4,
      rps_DR5,
      rps_DR6,
      rps_DR7,
      ILLEGAL_RPS};

static CONST_TABLE(int8_t, TXPOWLEVELS)[] = {16, 14, 12, 10, 8, 6, 4, 2,
                                             0,  0,  0,  0,  0, 0, 0, 0};

uint8_t LmicEu868::getRawRps(dr_t dr) const {
  return TABLE_GET_U1(_DR2RPS_CRC, dr + 1);
}

int8_t LmicEu868::pow2dBm(uint8_t mcmd_ladr_p1) const {
  return TABLE_GET_S1(TXPOWLEVELS, (mcmd_ladr_p1 & MCMD_LADR_POW_MASK) >>
                                       MCMD_LADR_POW_SHIFT);
}

OsDeltaTime LmicEu868::getDwn2SafetyZone() const { return DNW2_SAFETY_ZONE; }

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
CONST_TABLE(int32_t, DR2HSYM)[] = {
    OsDeltaTime::from_us_round(128 << 7).tick(), // DR_SF12
    OsDeltaTime::from_us_round(128 << 6).tick(), // DR_SF11
    OsDeltaTime::from_us_round(128 << 5).tick(), // DR_SF10
    OsDeltaTime::from_us_round(128 << 4).tick(), // DR_SF9
    OsDeltaTime::from_us_round(128 << 3).tick(), // DR_SF8
    OsDeltaTime::from_us_round(128 << 2).tick(), // DR_SF7
    OsDeltaTime::from_us_round(128 << 1).tick(), // DR_SF7B
    OsDeltaTime::from_us_round(80).tick()        // FSK -- not used (time for 1/2 byte)
};

OsDeltaTime LmicEu868::dr2hsym(dr_t dr) const {
  return OsDeltaTime(TABLE_GET_S4(DR2HSYM, (dr)));
}

bool LmicEu868::validRx1DrOffset(uint8_t drOffset) const {
  return drOffset < 6;
}

// ================================================================================
//
// BEG: EU868 related stuff
//
enum { NUM_DEFAULT_CHANNELS = 3 };
static CONST_TABLE(uint32_t, iniChannelFreq)[6] = {
    // Join frequencies and duty cycle limit (0.1%)
    EU868_F1 | BAND_MILLI,
    EU868_F2 | BAND_MILLI,
    EU868_F3 | BAND_MILLI,
    // Default operational frequencies
    EU868_F1 | BAND_CENTI,
    EU868_F2 | BAND_CENTI,
    EU868_F3 | BAND_CENTI,
};

void LmicEu868::initDefaultChannels(bool join) {
  PRINT_DEBUG_2("Init Default Channel join?=%d", join);
  ChannelDetail empty = {};
  std::fill(channels + 3, channels + MAX_CHANNELS, empty);

  channelMap = 0x07;
  uint8_t su = join ? 0 : 3;
  for (uint8_t fu = 0; fu < 3; fu++, su++) {
    channels[fu].freq = TABLE_GET_U4(iniChannelFreq, su);
    channels[fu].drMap = dr_range_map(Eu868Dr::SF12, Eu868Dr::SF7);
  }

  bands[BAND_MILLI].txcap = 1000; // 0.1%
  bands[BAND_CENTI].txcap = 100; // 1%
  bands[BAND_DECI].txcap = 10; // 10%
  bands[BAND_MILLI].txpow = 16;
  bands[BAND_CENTI].txpow = 16;
  bands[BAND_DECI].txpow = 16;
  bands[BAND_MILLI].lastchnl = rand.uint8() % MAX_CHANNELS;
  bands[BAND_CENTI].lastchnl = rand.uint8() % MAX_CHANNELS;
  bands[BAND_DECI].lastchnl = rand.uint8() % MAX_CHANNELS;
  auto now = os_getTime();
  bands[BAND_MILLI].avail = now;
  bands[BAND_CENTI].avail = now;
  bands[BAND_DECI].avail = now;
}

bool LmicEu868::setupBand(uint8_t bandidx, int8_t txpow, uint16_t txcap) {
  if (bandidx > BAND_AUX)
    return false;
  band_t *b = &bands[bandidx];
  b->txpow = txpow;
  b->txcap = txcap;
  b->avail = os_getTime();
  b->lastchnl = rand.uint8() % MAX_CHANNELS;
  return true;
}

bool LmicEu868::setupChannel(uint8_t chidx, uint32_t newfreq, uint16_t drmap,
                             int8_t band) {
  if (chidx >= MAX_CHANNELS)
    return false;
  if (band == -1) {
    if (newfreq >= 869400000 && newfreq <= 869650000)
      band = BAND_DECI; // 10%
    else if ((newfreq >= 868000000 && newfreq <= 868600000) ||
             (newfreq >= 869700000 && newfreq <= 870000000))
      band = BAND_CENTI; // 1%
    else
      band = BAND_MILLI; // 0.1%
  } else {
    if (band > BAND_AUX)
      return 0;
  }
  channels[chidx].freq = (newfreq & ~3) | band;
  channels[chidx].drMap =
      drmap == 0 ? dr_range_map(Eu868Dr::SF12, Eu868Dr::SF7) : drmap;
  channelMap |= 1 << chidx; // enabled right away
  return true;
}

void LmicEu868::disableChannel(uint8_t channel) {
  channels[channel].freq = 0;
  channels[channel].drMap = 0;
  channelMap &= ~(1 << channel);
}

uint32_t LmicEu868::convFreq(const uint8_t *ptr) const {
  uint32_t newfreq = rlsbf3(ptr) * 100;
  if (newfreq < EU868_FREQ_MIN || newfreq > EU868_FREQ_MAX)
    newfreq = 0;
  return newfreq;
}

void LmicEu868::handleCFList(const uint8_t *ptr) {

  for (uint8_t chidx = 3; chidx < 8; chidx++, ptr += 3) {
    uint32_t newfreq = convFreq(ptr);
    if (newfreq) {
      setupChannel(chidx, newfreq, 0, -1);
#if LMIC_DEBUG_LEVEL > 1
      lmic_printf("%lu: Setup channel, idx=%d, freq=%lu\n", os_getTime().tick(), chidx,
                  newfreq);
#endif
    }
  }
}

uint8_t LmicEu868::mapChannels(uint8_t chMaskCntl, uint16_t chMask) {
  // LoRaWAN™ 1.0.2 Regional Parameters §2.1.5
  // FIXME ChMaskCntl=6 => All channels ON
  // Bad page, disable all channel, enable non-existent
  if (chMaskCntl != 0 || chMask == 0 || (chMask & ~channelMap) != 0)
    return 0; // illegal input
  for (uint8_t chnl = 0; chnl < MAX_CHANNELS; chnl++) {
    if ((chMask & (1 << chnl)) != 0 && channels[chnl].freq == 0)
      chMask &= ~(1 << chnl); // ignore - channel is not defined
  }
  channelMap = chMask;
  return 1;
}

void LmicEu868::updateTx(OsTime txbeg, OsDeltaTime airtime) {

  // Update global/band specific duty cycle stats

  // Update channel/global duty cycle stats
  band_t *band = &bands[getBand(txChnl)];
  freq = getFreq(txChnl);
  txpow = band->txpow;
  // limit power to value ask in adr
  txpow = txpow > adrTxPow ? adrTxPow : txpow;
  // TODO check time calculation
  band->avail = txbeg + band->txcap * airtime;
  if (globalDutyRate != 0)
    globalDutyAvail = txbeg + OsDeltaTime(airtime.tick() << globalDutyRate);
#if LMIC_DEBUG_LEVEL > 1
  lmic_printf("%lu: Updating info for TX at %lu, airtime will be %lu. Setting "
              "available time for band %d to %lu\n",
              os_getTime().tick(), txbeg, airtime, freq, band->avail);
  if (globalDutyRate != 0)
    lmic_printf("%lu: Updating global duty avail to %lu\n", os_getTime().tick(),
                globalDutyAvail);
#endif
}

uint32_t LmicEu868::getFreq(uint8_t channel) const {
  return channels[channel].freq & ~(uint32_t)3;
}

uint8_t LmicEu868::getBand(uint8_t channel) const {
  return channels[channel].freq & 0x3;
}

OsTime LmicEu868::nextTx(OsTime now) {
  uint8_t bmap = 0xF;
#if LMIC_DEBUG_LEVEL > 1
  for (uint8_t bi = 0; bi < 4; bi++) {
    PRINT_DEBUG_2("Band %d, available at %lu and last channel %d", bi,
                  bands[bi].avail, bands[bi].lastchnl);
  }
#endif
  do {
    OsTime mintime = now + /*8h*/ OsDeltaTime::from_sec(28800);
    uint8_t band = 0xFF;
    for (uint8_t bi = 0; bi < 4; bi++) {
      if ((bmap & (1 << bi)) && mintime > bands[bi].avail) {
#if LMIC_DEBUG_LEVEL > 1
        lmic_printf("%lu: Considering band %d, which is available at %lu\n",
                    os_getTime().tick(), bi, bands[bi].avail);
#endif
        band = bi;
        mintime = bands[band].avail;
      }
    }
    if (band == 0xFF) {
      // Try to handle a strange bug wich appen afert 7 hours
      PRINT_DEBUG_2("Error No band available.");
      OsTime resetTime = now + OsDeltaTime::from_sec(15 * 60);
      for (uint8_t bi = 0; bi < MAX_BANDS; bi++) {
        PRINT_DEBUG_2("Band %i Reseting avail from %lu to %lu, lastchnl: %i.",
                      bi, bands[bi].avail, resetTime, bands[bi].lastchnl);
        bands[bi].avail = resetTime;
      }
      // force band 0.
      band = 0;
      mintime = resetTime;
    }

    // Find next channel in given band
    uint8_t chnl = bands[band].lastchnl;
    for (uint8_t ci = 0; ci < MAX_CHANNELS; ci++) {
      chnl++;
      if (chnl >= MAX_CHANNELS)
        chnl -= MAX_CHANNELS;
      // channel enabled
      if ((channelMap & (1 << chnl)) != 0) {
        PRINT_DEBUG_2(
            "Considering channel %d for band %d, set band = %d, drMap = %x",
            chnl, band, getBand(chnl), channels[chnl].drMap);
        if ((channels[chnl].drMap & (1 << (datarate & 0xF))) != 0 &&
            band == getBand(chnl)) { // in selected band
          txChnl = bands[band].lastchnl = chnl;
          return mintime;
        }
      }
    }

    PRINT_DEBUG_2("No channel found in band %d", band);

    bmap &= ~(1 << band);
    if (bmap == 0) {
      // No feasible channel  found!
      return mintime;
    }
  } while (true);
}

void LmicEu868::setRx1Params() {
  /*freq remain unchanged*/
  lowerDR(dndr, rx1DrOffset);
}

#if !defined(DISABLE_JOIN)
void LmicEu868::initJoinLoop() {
  txChnl = rand.uint8() % 3;
  adrTxPow = 14;
  setDrJoin(static_cast<dr_t>(Eu868Dr::SF7));
  initDefaultChannels(true);
  ASSERT(!(opmode & OpState::NEXTCHNL));
  txend = bands[BAND_MILLI].avail + OsDeltaTime::rnd_delay(rand, 8);
  PRINT_DEBUG_1("Init Join loop : avail=%lu txend=%lu", bands[BAND_MILLI].avail,
                txend);
}

bool LmicEu868::nextJoinState() {
  bool failed = false;

  // Try 869.x and then 864.x with same DR
  // If both fail try next lower datarate
  if (++txChnl == 3)
    txChnl = 0;
  if ((++txCnt & 1) == 0) {
    // Lower DR every 2nd try (having tried 868.x and 864.x with the same DR)
    if (datarate == static_cast<dr_t>(Eu868Dr::SF12))
      failed = true; // we have tried all DR - signal EV_JOIN_FAILED
    else
      datarate = decDR(datarate);
  }

  // Move txend to randomize synchronized concurrent joins.
  // Duty cycle is based on txend.
  OsTime time = os_getTime();
  if (time - bands[BAND_MILLI].avail < OsDeltaTime (0))
    time = bands[BAND_MILLI].avail;
  txend = time + (isTESTMODE()
                      // Avoid collision with JOIN ACCEPT @ SF12 being sent by
                      // GW (but we missed it)
                      ? DNW2_SAFETY_ZONE
                      // Otherwise: randomize join (street lamp case):
                      // SF12:255, SF11:127, .., SF7:8secs
                      : DNW2_SAFETY_ZONE +
                            OsDeltaTime::rnd_delay(rand, 255 >> datarate));
  PRINT_DEBUG_1(" Next available : %li , Choosen %li", time.tick(),
                txend.tick());
#if LMIC_DEBUG_LEVEL > 1
  if (failed)
    lmic_printf("%lu: Join failed\n", os_getTime().tick());
  else
    lmic_printf("%lu: Scheduling next join at %lu\n", os_getTime().tick(), txend);
#endif
  // 1 - triggers EV_JOIN_FAILED event
  return !failed;
}
#endif // !DISABLE_JOIN

dr_t LmicEu868::defaultRX2Dr() const { return static_cast<dr_t>(DR_DNW2); }
uint32_t LmicEu868::defaultRX2Freq() const { return FREQ_DNW2; }

LmicEu868::LmicEu868(lmic_pinmap const& pins) : Lmic(pins) {};