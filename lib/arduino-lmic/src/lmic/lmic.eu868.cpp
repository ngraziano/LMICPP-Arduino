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
  EU868_J4 = 864100000, // g2   SF7-12  used during join
  EU868_J5 = 864300000, // g2   SF7-12   ditto
  EU868_J6 = 864500000, // g2   SF7-12   ditto
};
enum { EU868_FREQ_MIN = 863000000, EU868_FREQ_MAX = 870000000 };

enum { FREQ_PING = EU868_F6 }; // default ping freq
enum { CHNL_DNW2 = 5 };
enum { FREQ_DNW2 = EU868_F6 };

namespace {
constexpr LmicEu868::Dr DR_DNW2 = LmicEu868::Dr::SF12;

constexpr OsDeltaTime DNW2_SAFETY_ZONE = OsDeltaTime::from_ms(3000);

constexpr uint8_t rps_DR0 =
    rps_t{SF12, BandWidth::BW125, CodingRate::CR_4_5, false, 0}.rawValue();
constexpr uint8_t rps_DR1 =
    rps_t{SF11, BandWidth::BW125, CodingRate::CR_4_5, false, 0}.rawValue();
constexpr uint8_t rps_DR2 =
    rps_t{SF10, BandWidth::BW125, CodingRate::CR_4_5, false, 0}.rawValue();
constexpr uint8_t rps_DR3 =
    rps_t{SF9, BandWidth::BW125, CodingRate::CR_4_5, false, 0}.rawValue();
constexpr uint8_t rps_DR4 =
    rps_t{SF8, BandWidth::BW125, CodingRate::CR_4_5, false, 0}.rawValue();
constexpr uint8_t rps_DR5 =
    rps_t{SF7, BandWidth::BW125, CodingRate::CR_4_5, false, 0}.rawValue();
constexpr uint8_t rps_DR6 =
    rps_t{SF7, BandWidth::BW250, CodingRate::CR_4_5, false, 0}.rawValue();
constexpr uint8_t rps_DR7 =
    rps_t{FSK, BandWidth::BW125, CodingRate::CR_4_5, false, 0}.rawValue();

CONST_TABLE(uint8_t, _DR2RPS_CRC)
[] = {ILLEGAL_RPS, rps_DR0, rps_DR1, rps_DR2, rps_DR3,
      rps_DR4,     rps_DR5, rps_DR6, rps_DR7, ILLEGAL_RPS};

constexpr int8_t MaxEIRP = 16;

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

const uint32_t MIN_BAND1_CENTI = 868000000;
const uint32_t MAX_BAND1_CENTI = 868600000;
const uint32_t MIN_BAND_DECI = 869400000;
const uint32_t MAX_BAND_DECI = 869650000;
const uint32_t MIN_BAND2_CENTI = 869700000;
const uint32_t MAX_BAND2_CENTI = 870000000;

} // namespace

void BandsEu868::init(LmicRand &rand, uint8_t maxchannels) {
  // bands[BAND_MILLI].txcap = 1000; // 0.1%
  // bands[BAND_CENTI].txcap = 100;  // 1%
  // bands[BAND_DECI].txcap = 10;    // 10%
  for (int i = 0; i < MAX_BAND; i++) {
    lastchnl[i] = rand.uint8() % maxchannels;
  }
  auto now = os_getTime();
  for (int i = 0; i < MAX_BAND; i++) {
    avail[i] = now;
  }
}

void BandsEu868::updateBandAvailability(uint8_t band, OsTime lastusage,
                                        OsDeltaTime duration) {
  uint16_t cap;
  if (band == BAND_MILLI) {
    cap = 1000;
  } else if (band == BAND_CENTI) {
    cap = 100;
  } else {
    // DECI
    cap = 10;
  }
  avail[band] = lastusage + cap * duration;

  PRINT_DEBUG(2, F("Setting  available time for band %d to %" PRIu32 ""), band,
              avail[band].tick());
}

void BandsEu868::print_state() {

  for (uint8_t bi = 0; bi < MAX_BAND; bi++) {
    PRINT_DEBUG(2, F("Band %d, available at %" PRIu32 " and last channel %d"),
                bi, avail[bi].tick(), lastchnl[bi]);
  }
}
uint8_t LmicEu868::getRawRps(dr_t dr) const {
  return TABLE_GET_U1(_DR2RPS_CRC, dr + 1);
}

int8_t LmicEu868::pow2dBm(uint8_t powerIndex) const {
  if (powerIndex < 8) {
    return MaxEIRP - 2 * powerIndex;
  }
  // TODO handle bad value
  return 0;
}

OsDeltaTime LmicEu868::getDwn2SafetyZone() const { return DNW2_SAFETY_ZONE; }

OsDeltaTime LmicEu868::dr2hsym(dr_t dr) const {
  return OsDeltaTime(TABLE_GET_S4(DR2HSYM, (dr)));
}

bool LmicEu868::validRx1DrOffset(uint8_t drOffset) const {
  return drOffset < 6;
}

void LmicEu868::initDefaultChannels(bool join) {
  PRINT_DEBUG(2, F("Init Default Channel join?=%d"), join);

  channels.disableAll();
  uint8_t su = join ? 0 : 3;
  for (uint8_t fu = 0; fu < 3; fu++, su++) {
    channels.configure(fu, ChannelDetail{TABLE_GET_U4(iniChannelFreq, su),
                                         dr_range_map(Dr::SF12, Dr::SF7)});
  }

  bands.init(rand, MAX_CHANNELS);
}

bool LmicEu868::setupChannel(uint8_t chidx, uint32_t newfreq, uint16_t drmap) {
  if (chidx >= MAX_CHANNELS)
    return false;

  int8_t band;
  if (newfreq >= MIN_BAND_DECI && newfreq <= MAX_BAND_DECI)
    band = BAND_DECI; // 10%
  else if ((newfreq >= MIN_BAND1_CENTI && newfreq <= MAX_BAND1_CENTI) ||
           (newfreq >= MIN_BAND2_CENTI && newfreq <= MAX_BAND2_CENTI))
    band = BAND_CENTI; // 1%
  else
    band = BAND_MILLI; // 0.1%

  channels.configure(
      chidx,
      ChannelDetail{newfreq, band,
                    drmap == 0 ? dr_range_map(Dr::SF12, Dr::SF7) : drmap});
  return true;
}

void LmicEu868::disableChannel(uint8_t channel) { channels.disable(channel); }

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
      setupChannel(chidx, newfreq, 0);

      PRINT_DEBUG(2, F("Setup channel, idx=%d, freq=%" PRIu32 ""), chidx,
                  newfreq);
    }
  }
}

bool LmicEu868::mapChannels(uint8_t chMaskCntl, uint16_t chMask) {
  // LoRaWAN™ 1.0.2 Regional Parameters §2.1.5
  // ChMaskCntl=6 => All channels ON
  if (chMaskCntl == 6) {
    channels.enableAll();
    return true;
  }

  // Bad page, disable all channel
  if (chMaskCntl != 0 || chMask == 0)
    return false; // illegal input

  for (uint8_t chnl = 0; chnl < MAX_CHANNELS; chnl++) {
    if ((chMask & (1 << chnl)) != 0)
      channels.enable(chnl);
  }

  return true;
}

void LmicEu868::updateTx(OsTime txbeg, OsDeltaTime airtime) {

  freq = getFreq(txChnl);

  // limit power to value ask in adr (at init MaxEIRP)
  txpow = adrTxPow;

  // Update band specific duty cycle stats
  bands.updateBandAvailability(getBand(txChnl), txbeg, airtime);

  PRINT_DEBUG(
      2, F("Updating info for TX at %" PRIu32 ", airtime will be %" PRIu32 "."),
      txbeg, airtime);
}

uint32_t LmicEu868::getFreq(uint8_t channel) const {
  return channels[channel].getFrequency();
}

uint8_t LmicEu868::getBand(uint8_t channel) const {
  return channels[channel].getBand();
}

OsTime LmicEu868::nextTx(OsTime const now) {
  uint8_t bmap = 0x07;

  bands.print_state();

  do {
    OsTime mintime = now + /*8h*/ OsDeltaTime::from_sec(28800);
    uint8_t band = 0xFF;

    for (uint8_t bi = 0; bi < bands.MAX_BAND; bi++) {
      if ((bmap & (1 << bi)) && mintime > bands.getAvailability(bi)) {
        PRINT_DEBUG(2,
                    F("Considering band %d, which is available at %" PRIu32 ""),
                    bi, bands.getAvailability(bi).tick());
        band = bi;
        mintime = bands.getAvailability(bi);
      }
    }

    if (band == 0xFF) {
      // Try to handle a strange bug wich appen afert from time to time
      // Reset all bands value;
      PRINT_DEBUG(2, F("Error No band available."));
      bands.init(rand, MAX_CHANNELS);
      // force band 0.
      band = 0;
      mintime = os_getTime();
    }

    // Find next channel in given band
    uint8_t chnl = bands.getLastChannel(band);
    for (uint8_t ci = 0; ci < MAX_CHANNELS; ci++) {
      chnl++;
      if (chnl >= MAX_CHANNELS)
        chnl -= MAX_CHANNELS;
      // channel enabled
      if (channels.is_enable(chnl)) {
        PRINT_DEBUG(
            2,
            F("Considering channel %d for band %d, set band = %d, drMap = %x"),
            chnl, band, getBand(chnl), channels[chnl].getDrMap());
        if ((channels[chnl].getDrMap() & (1 << (datarate & 0xF))) != 0 &&
            band == getBand(chnl)) {
          // in selected band
          bands.setLastChannel(band, chnl);
          txChnl = chnl;
          return mintime;
        }
      }
    }

    PRINT_DEBUG(2, F("No channel found in band %d"), band);

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

void LmicEu868::initJoinLoop() {
  txChnl = rand.uint8() % 3;
  adrTxPow = MaxEIRP;
  setDrJoin(static_cast<dr_t>(Dr::SF7));
  initDefaultChannels(true);
  txend = bands.getAvailability(BAND_MILLI) + OsDeltaTime::rnd_delay(rand, 8);
  PRINT_DEBUG(1, F("Init Join loop : avail=%" PRIu32 " txend=%" PRIu32 ""),
              bands.getAvailability(BAND_MILLI).tick(), txend.tick());
}

bool LmicEu868::nextJoinState() {
  bool failed = false;

  // Try 869.x and then 864.x with same DR
  // If both fail try next lower datarate
  if (++txChnl == 3)
    txChnl = 0;
  if ((++txCnt & 1) == 0) {
    // Lower DR every 2nd try (having tried 868.x and 864.x with the same DR)
    if (datarate == static_cast<dr_t>(Dr::SF12))
      failed = true; // we have tried all DR - signal EV_JOIN_FAILED
    else
      datarate = decDR(datarate);
  }

  // Move txend to randomize synchronized concurrent joins.
  // Duty cycle is based on txend.
  OsTime time = os_getTime();
  if (time < bands.getAvailability(BAND_MILLI))
    time = bands.getAvailability(BAND_MILLI);

  // Avoid collision with JOIN ACCEPT @ SF12 being sent by
  // GW (but we missed it) randomize join (street lamp case):
  // SF12:255, SF11:127, .., SF7:8secs
  txend =
      time + DNW2_SAFETY_ZONE + OsDeltaTime::rnd_delay(rand, 255 >> datarate);

  PRINT_DEBUG(1, F("Next available : %" PRIu32 " , Choosen %" PRIu32 ""),
              time.tick(), txend.tick());

  if (failed)
    PRINT_DEBUG(2, F("Join failed"));
  else
    PRINT_DEBUG(2, F("Scheduling next join at %" PRIu32 ""), txend);

  // 1 - triggers EV_JOIN_FAILED event
  return !failed;
}

dr_t LmicEu868::defaultRX2Dr() const { return static_cast<dr_t>(DR_DNW2); }
uint32_t LmicEu868::defaultRX2Freq() const { return FREQ_DNW2; }

#if defined(ENABLE_SAVE_RESTORE)
size_t LmicEu868::saveState(uint8_t *buffer) {
  uint8_t *orig = buffer;
  buffer += Lmic::saveState(buffer);
  // todo save BAND

  // todo save

  PRINT_DEBUG(1, F("Size save %i"), buffer - orig);
  return buffer - orig;
}
#endif

LmicEu868::LmicEu868(lmic_pinmap const &pins, OsScheduler &scheduler)
    : Lmic(pins, scheduler) {}