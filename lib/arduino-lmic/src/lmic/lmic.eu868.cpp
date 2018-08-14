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
#include "lmic.h"
#include "../aes/aes.h"
#include "bufferpack.h"
#include "radio.h"
#include <algorithm>


#if defined(CFG_eu868) // ========================================

#define DNW2_SAFETY_ZONE OsDeltaTime::from_ms(3000)

#define maxFrameLen(dr)                                                        \
  ((dr) <= DR_SF9 ? TABLE_GET_U1(maxFrameLens, (dr)) : 0xFF)
CONST_TABLE(uint8_t, maxFrameLens)[] = {64, 64, 64, 123};

CONST_TABLE(uint8_t, _DR2RPS_CRC)
[] = {ILLEGAL_RPS,
      (uint8_t)MAKERPS(SF12, BW125, CR_4_5, 0, 0),
      (uint8_t)MAKERPS(SF11, BW125, CR_4_5, 0, 0),
      (uint8_t)MAKERPS(SF10, BW125, CR_4_5, 0, 0),
      (uint8_t)MAKERPS(SF9, BW125, CR_4_5, 0, 0),
      (uint8_t)MAKERPS(SF8, BW125, CR_4_5, 0, 0),
      (uint8_t)MAKERPS(SF7, BW125, CR_4_5, 0, 0),
      (uint8_t)MAKERPS(SF7, BW250, CR_4_5, 0, 0),
      (uint8_t)MAKERPS(FSK, BW125, CR_4_5, 0, 0),
      ILLEGAL_RPS};

static CONST_TABLE(int8_t, TXPOWLEVELS)[] = {20, 14, 11, 8, 5, 2, 0, 0,
                                             0,  0,  0,  0, 0, 0, 0, 0};

int8_t Lmic::pow2dBm(uint8_t mcmd_ladr_p1) {
  return TABLE_GET_S1(TXPOWLEVELS,
                (mcmd_ladr_p1 & MCMD_LADR_POW_MASK) >> MCMD_LADR_POW_SHIFT);
} 

OsDeltaTime Lmic::getDwn2SafetyZone() {
  return DNW2_SAFETY_ZONE;
}

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
    us2osticksRound(128 << 7), // DR_SF12
    us2osticksRound(128 << 6), // DR_SF11
    us2osticksRound(128 << 5), // DR_SF10
    us2osticksRound(128 << 4), // DR_SF9
    us2osticksRound(128 << 3), // DR_SF8
    us2osticksRound(128 << 2), // DR_SF7
    us2osticksRound(128 << 1), // DR_SF7B
    us2osticksRound(80)        // FSK -- not used (time for 1/2 byte)
};

OsDeltaTime Lmic::dr2hsym(dr_t dr) {
 return OsDeltaTime(TABLE_GET_S4(DR2HSYM, (dr)));
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

void Lmic::initDefaultChannels(bool join) {
  PRINT_DEBUG_2("Init Default Channel join?=%d", join);
  ChannelDetail empty = {};
  std::fill(channels + 3, channels + MAX_CHANNELS, empty);

  channelMap = 0x07;
  uint8_t su = join ? 0 : 3;
  for (uint8_t fu = 0; fu < 3; fu++, su++) {
    channels[fu].freq = TABLE_GET_U4(iniChannelFreq, su);
    channels[fu].drMap = DR_RANGE_MAP(DR_SF12, DR_SF7);
  }

  bands[BAND_MILLI].txcap = 1000; // 0.1%
  bands[BAND_MILLI].txpow = 14;
  bands[BAND_MILLI].lastchnl = hal_rand1() % MAX_CHANNELS;
  bands[BAND_CENTI].txcap = 100; // 1%
  bands[BAND_CENTI].txpow = 14;
  bands[BAND_CENTI].lastchnl = hal_rand1() % MAX_CHANNELS;
  bands[BAND_DECI].txcap = 10; // 10%
  bands[BAND_DECI].txpow = 27;
  bands[BAND_DECI].lastchnl = hal_rand1() % MAX_CHANNELS;
  auto now = os_getTime();
  bands[BAND_MILLI].avail = now;
  bands[BAND_CENTI].avail = now;
  bands[BAND_DECI].avail = now;
}

bool Lmic::setupBand(uint8_t bandidx, int8_t txpow, uint16_t txcap) {
  if (bandidx > BAND_AUX)
    return false;
  band_t *b = &bands[bandidx];
  b->txpow = txpow;
  b->txcap = txcap;
  b->avail = os_getTime();
  b->lastchnl = hal_rand1() % MAX_CHANNELS;
  return true;
}

bool Lmic::setupChannel(uint8_t chidx, uint32_t newfreq, uint16_t drmap,
                        int8_t band) {
  if (chidx >= MAX_CHANNELS)
    return false;
  if (band == -1) {
    if (newfreq >= 869400000 && newfreq <= 869650000)
      band = BAND_DECI; // 10% 27dBm
    else if ((newfreq >= 868000000 && newfreq <= 868600000) ||
             (newfreq >= 869700000 && newfreq <= 870000000))
      band = BAND_CENTI; // 1% 14dBm
    else
      band = BAND_MILLI; // 0.1% 14dBm
  } else {
    if (band > BAND_AUX)
      return 0;
  }
  channels[chidx].freq = (newfreq & ~3) | band;
  channels[chidx].drMap = drmap == 0 ? DR_RANGE_MAP(DR_SF12, DR_SF7) : drmap;
  channelMap |= 1 << chidx; // enabled right away
  return true;
}

void Lmic::disableChannel(uint8_t channel) {
  channels[channel].freq = 0;
  channels[channel].drMap = 0;
  channelMap &= ~(1 << channel);
}

uint32_t Lmic::convFreq(const uint8_t *ptr) {
  uint32_t newfreq = (rlsbf4(ptr - 1) >> 8) * 100;
  if (newfreq < EU868_FREQ_MIN || newfreq > EU868_FREQ_MAX)
    newfreq = 0;
  return newfreq;
}

uint8_t Lmic::mapChannels(uint8_t chMaskCntl, uint16_t chMask) {
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

void Lmic::updateTx(OsTime const &txbeg) {

  // Update global/band specific duty cycle stats
  OsDeltaTime airtime = calcAirTime(rps, dataLen);
  // Update channel/global duty cycle stats
  band_t *band = &bands[getBand(txChnl)];
  freq = getFreq(txChnl);
  txpow = band->txpow;
  // TODO check time calculation
  band->avail = txbeg + band->txcap * airtime;
  if (globalDutyRate != 0)
    globalDutyAvail = txbeg + OsDeltaTime(airtime.tick() << globalDutyRate);
#if LMIC_DEBUG_LEVEL > 1
  lmic_printf("%lu: Updating info for TX at %lu, airtime will be %lu. Setting "
              "available time for band %d to %lu\n",
              os_getTime(), txbeg, airtime, freq, band->avail);
  if (globalDutyRate != 0)
    lmic_printf("%lu: Updating global duty avail to %lu\n", os_getTime(),
                globalDutyAvail);
#endif
}

uint32_t Lmic::getFreq(uint8_t channel) {
  return channels[channel].freq & ~(uint32_t)3;
}

uint8_t Lmic::getBand(uint8_t channel) { return channels[channel].freq & 0x3; }

OsTime Lmic::nextTx(OsTime const &now) {
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
      if ((bmap & (1 << bi)) && mintime - bands[bi].avail > 0) {
#if LMIC_DEBUG_LEVEL > 1
        lmic_printf("%lu: Considering band %d, which is available at %lu\n",
                    os_getTime(), bi, bands[bi].avail);
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

void Lmic::setRx1Params() { /*freq/rps remain unchanged*/
}

#if !defined(DISABLE_JOIN)
void Lmic::initJoinLoop() {
  txChnl = hal_rand1() % 3;
  adrTxPow = 14;
  setDrJoin(DR_SF7);
  initDefaultChannels(true);
  ASSERT((opmode & OP_NEXTCHNL) == 0);
  txend = bands[BAND_MILLI].avail + OsDeltaTime::rnd_delay(8);
  PRINT_DEBUG_1("Init Join loop : avail=%lu txend=%lu", bands[BAND_MILLI].avail,
                txend);
}

bool Lmic::nextJoinState() {
  bool failed = 0;

  // Try 869.x and then 864.x with same DR
  // If both fail try next lower datarate
  if (++txChnl == 3)
    txChnl = 0;
  if ((++txCnt & 1) == 0) {
    // Lower DR every 2nd try (having tried 868.x and 864.x with the same DR)
    if (datarate == DR_SF12)
      failed = true; // we have tried all DR - signal EV_JOIN_FAILED
    else
      setDrJoin(decDR(datarate));
  }
  // Clear NEXTCHNL because join state engine controls channel hopping
  opmode &= ~OP_NEXTCHNL;
  // Move txend to randomize synchronized concurrent joins.
  // Duty cycle is based on txend.
  OsTime time = os_getTime();
  if (time - bands[BAND_MILLI].avail < 0)
    time = bands[BAND_MILLI].avail;
  txend =
      time + (isTESTMODE()
                  // Avoid collision with JOIN ACCEPT @ SF12 being sent by
                  // GW (but we missed it)
                  ? DNW2_SAFETY_ZONE
                  // Otherwise: randomize join (street lamp case):
                  // SF12:255, SF11:127, .., SF7:8secs
                  : DNW2_SAFETY_ZONE + OsDeltaTime::rnd_delay(255 >> datarate));
#if LMIC_DEBUG_LEVEL > 1
  if (failed)
    lmic_printf("%lu: Join failed\n", os_getTime());
  else
    lmic_printf("%lu: Scheduling next join at %lu\n", os_getTime(), txend);
#endif
  // 1 - triggers EV_JOIN_FAILED event
  return !failed;
}
#endif // !DISABLE_JOIN



#endif