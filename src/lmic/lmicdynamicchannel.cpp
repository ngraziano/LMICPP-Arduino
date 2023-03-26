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
#include "lmic_table.h"
#include "lmicdynamicchannel.h"
#include <algorithm>

namespace {

constexpr OsDeltaTime DNW2_SAFETY_ZONE = OsDeltaTime::from_ms(3000);

} // namespace

// decrease data rate
dr_t DynamicRegionalChannelParams::decDR(dr_t const dr) const {
  return dr == 0 ? 0 : dr - 1;
}

// in range
bool DynamicRegionalChannelParams::validDR(dr_t const dr) const {
  return dr < MaxDr;
}

rps_t DynamicRegionalChannelParams::getRps(dr_t const dr) const {
  return rps_t(table_get_u1(dr_table, dr));
}

rps_t DynamicRegionalChannelParams::getRpsDw(dr_t const dr) const {
  auto val = rps_t(table_get_u1(dr_table, dr));
  val.nocrc = true;
  return val;
}

// decrease data rate by n steps
dr_t DynamicRegionalChannelParams::lowerDR(dr_t dr, uint8_t n) const {
  return dr < n ? 0 : dr - n;
}

OsDeltaTime DynamicRegionalChannelParams::getDwn2SafetyZone() const {
  return DNW2_SAFETY_ZONE;
}

void DynamicRegionalChannelParams::initDefaultChannels() {
  PRINT_DEBUG(2, F("Init Default Channel"));

  channels.disableAll();
  channels.init();
}

void DynamicRegionalChannelParams::disableChannel(uint8_t const channel) {
  channels.disable(channel);
}

void DynamicRegionalChannelParams::handleCFList(const uint8_t *ptr) {
  // Check CFList type
  if (ptr[15] != 0) {
    PRINT_DEBUG(2, F("Wrong cflist type %d"), ptr[15]);
    return;
  }
  for (uint8_t chidx = 3; chidx < 8; chidx++, ptr += 3) {
    uint32_t newfreq = read_frequency(ptr);
    if (newfreq != 0) {
      setupChannel(chidx, newfreq, 0);

      PRINT_DEBUG(2, F("Setup channel, idx=%d, freq=%" PRIu32 ""), chidx,
                  newfreq);
    }
  }
}

bool DynamicRegionalChannelParams::validMapChannels(uint8_t const chMaskCntl,
                                                    uint16_t const chMask) {
  // Bad page
  if (chMaskCntl != 0 && chMaskCntl != 6)
    return false;

  //  disable all channel
  if (chMaskCntl == 0 && chMask == 0)
    return false;

  return true;
}

void DynamicRegionalChannelParams::mapChannels(uint8_t const chMaskCntl,
                                               uint16_t const chMask) {
  // LoRaWAN™ 1.0.2 Regional Parameters §2.1.5
  // ChMaskCntl=6 => All channels ON
  if (chMaskCntl == 6) {
    channels.enableAll();
    return;
  }

  for (uint8_t chnl = 0; chnl < ChannelList::LIMIT_CHANNELS; chnl++) {
    if ((chMask & (1u << chnl)) != 0) {
      channels.enable(chnl);
    } else {
      channels.disable(chnl);
    }
  }
}

uint32_t DynamicRegionalChannelParams::getTxFrequency() const {
  return channels.getFrequency(txChnl);
}

int8_t DynamicRegionalChannelParams::getTxPower() const {
  // limit power to value ask in adr (at init MaxEIRP)
  return adrTxPow;
}

void DynamicRegionalChannelParams::updateTxTimes(OsDeltaTime const airtime) {
  channels.updateAvailabitility(txChnl, os_getTime(), airtime);

  PRINT_DEBUG(
      2, F("Updating info for TX channel %d, airtime will be %" PRIu32 "."),
      txChnl, airtime.tick());
}

OsTime DynamicRegionalChannelParams::nextTx(OsTime const now) {

  bool channelFound = false;
  OsTime nextTransmitTime;
  // next channel or other (random)
  uint8_t nextChannel = txChnl + 1 + (rand.uint8() % 2);

  for (uint8_t channelIndex = 0; channelIndex < ChannelList::LIMIT_CHANNELS;
       channelIndex++) {
    if (nextChannel >= channels.LIMIT_CHANNELS) {
      nextChannel = 0;
    }

    if (channels.is_enable_at_dr(nextChannel, datarate)) {
      auto availability = channels.getAvailability(nextChannel);

      PRINT_DEBUG(2, F("Considering channel %d"), nextChannel);

      if (!channelFound || availability < nextTransmitTime) {
        txChnl = nextChannel;
        nextTransmitTime = availability;
        channelFound = true;
      }
      if (availability < now) {
        // no need to search better
        txChnl = nextChannel;
        return availability;
      }
    }
    nextChannel++;
  }

  if (channelFound) {
    return nextTransmitTime;
  }

  // Fail to find a channel continue on current one.
  // UGLY FAILBACK
  PRINT_DEBUG(1, F("Error Fail to find a channel."));
  return now;
}

uint32_t DynamicRegionalChannelParams::getRx1Frequency() const {
  return channels.getFrequencyRX(txChnl);
}

dr_t DynamicRegionalChannelParams::getRx1Dr() const {
  return lowerDR(datarate, rx1DrOffset);
}

TransmitionParameters DynamicRegionalChannelParams::getTxParameter() const {
  return {getTxFrequency(), getRps(datarate), getTxPower()};
}

TransmitionParameters DynamicRegionalChannelParams::getRx1Parameter() const {
  return {getRx1Frequency(), getRpsDw(getRx1Dr()), 0};
}

TransmitionParameters DynamicRegionalChannelParams::getRx2Parameter() const {
  return rx2Parameter;
}

void DynamicRegionalChannelParams::setRx2Parameter(uint32_t const freq,
                                                   dr_t const dr) {
  rx2Parameter = {freq, getRpsDw(dr), 0};
}

void DynamicRegionalChannelParams::setRx2DataRate(dr_t const dr) {
  rx2Parameter.rps = getRpsDw(dr);
}

OsTime DynamicRegionalChannelParams::initJoinLoop() {
  txChnl = rand.uint8() % 3;
  adrTxPow = MaxEIRP;
  setDrJoin(MaxJoinDR);
  auto startTime =
      channels.getAvailability(0) + OsDeltaTime::rnd_delay(rand, 8);
  PRINT_DEBUG(1, F("Init Join loop : avail=%" PRIu32 " startTime=%" PRIu32 ""),
              channels.getAvailability(0).tick(), startTime.tick());
  return startTime;
}

TimeAndStatus DynamicRegionalChannelParams::nextJoinState() {
  bool failed = false;

  // Try the tree default channels with same DR
  // If both fail try next lower datarate
  if (++txChnl == 3)
    txChnl = 0;
  if ((++joinCount & 1) == 0) {
    // Lower DR every 2nd try
    if (datarate == MinJoinDR) {
      // we have tried all DR - signal EV_JOIN_FAILED
      failed = true;
      // and retry from highest datarate.
      datarate = MaxJoinDR;
    } else {
      datarate = decDR(datarate);
    }
  }

  // Set minimal next join time
  auto time = os_getTime();
  auto availability = channels.getAvailability(txChnl);
  if (time < availability)
    time = availability;

  if (failed)
    PRINT_DEBUG(2, F("Join failed"));
  else
    PRINT_DEBUG(2, F("Scheduling next join at %" PRIu32 ""), time.tick());

  // 1 - triggers EV_JOIN_FAILED event
  return {time, !failed};
}

void DynamicRegionalChannelParams::setRx1DrOffset(uint8_t drOffset) {
  rx1DrOffset = drOffset;
}

void DynamicRegionalChannelParams::setRegionalDutyCycleVerification(
    bool enabled) {
  channels.setCheckDutyCycle(enabled);
}

bool DynamicRegionalChannelParams::setAdrToMaxIfNotAlreadySet() {
  if (adrTxPow != MaxEIRP) {
    adrTxPow = MaxEIRP;
    return true;
  }
  return false;
}

#if defined(ENABLE_SAVE_RESTORE)
void DynamicRegionalChannelParams::saveStateWithoutTimeData(
    StoringAbtract &store) const {

  channels.saveStateWithoutTimeData(store);
  store.write(txChnl);
  store.write(adrTxPow);
  store.write(datarate);
  store.write(rx1DrOffset);
  store.write(rx2Parameter);
}

void DynamicRegionalChannelParams::saveState(StoringAbtract &store) const {
  channels.saveState(store);
  store.write(txChnl);
  store.write(adrTxPow);
  store.write(datarate);
  store.write(rx1DrOffset);
  store.write(rx2Parameter);
}

void DynamicRegionalChannelParams::loadStateWithoutTimeData(
    RetrieveAbtract &store) {

  channels.loadStateWithoutTimeData(store);
  store.read(txChnl);
  store.read(adrTxPow);
  store.read(datarate);
  store.read(rx1DrOffset);
  store.read(rx2Parameter);
}

void DynamicRegionalChannelParams::loadState(RetrieveAbtract &store) {

  channels.loadState(store);
  store.read(txChnl);
  store.read(adrTxPow);
  store.read(datarate);
  store.read(rx1DrOffset);
  store.read(rx2Parameter);
}
#endif

DynamicRegionalChannelParams::DynamicRegionalChannelParams(
    LmicRand &arand, uint8_t aMaxEIRP, dr_t aMaxJoinDr, dr_t aMinJoinDr,
    const uint8_t *drtable, dr_t aMaxDr, Bands &aBands)
    : rand{arand}, MaxEIRP(aMaxEIRP), MaxJoinDR(aMaxJoinDr),
      MinJoinDR(aMinJoinDr), dr_table(drtable),
      MaxDr(aMaxDr), channels{aBands} {}
