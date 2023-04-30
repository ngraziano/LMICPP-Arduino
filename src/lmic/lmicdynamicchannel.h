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

#ifndef lmic_lmicdynamicchannel_h
#define lmic_lmicdynamicchannel_h

#include "bands.h"
#include "bufferpack.h"
#include "channelList.h"
#include "lmic.h"

#include "../hal/print_debug.h"
#include "lmic_table.h"
#include <algorithm>

namespace DYNAMIC_CHANNEL {
constexpr OsDeltaTime DNW2_SAFETY_ZONE = OsDeltaTime::from_ms(3000);

template <typename BandsType, int8_t MaxEIRP, dr_t MaxJoinDR, dr_t MinJoinDR,
          const uint8_t *dr_table, dr_t MaxDr, uint32_t default_Freq_RX2,
          uint8_t default_rps_RX2, uint8_t maxPowerIndex,
          uint8_t limitRX1DrOffset, uint8_t nbFixedChannels,
          const uint32_t *defaultChannelFreq, uint16_t defaultChannelDrMap,
          uint32_t minFrequency, uint32_t maxFrequency>
class DynamicRegionalChannelParams : public RegionalChannelParams {

  using ChannelListType = ChannelList<BandsType>;

public:
  bool setupChannel(uint8_t const chidx, uint32_t const newfreq,
                    uint16_t const drmap) {
    if (chidx >= channels.LIMIT_CHANNELS)
      return false;

    if (chidx < nbFixedChannels) {
      // channel 0, 1 and 2 are fixed
      return false;
    }

    if (newfreq == 0) {
      channels.disable(chidx);
      return true;
    }

    if (newfreq < minFrequency || newfreq > maxFrequency) {
      return false;
    }

    channels.configure(chidx, newfreq, drmap);
    return true;
  }

  TransmitionParameters getTxParameter() const final {
    return {channels.getFrequency(txChnl), getRps(datarate), adrTxPow};
  };
  TransmitionParameters getRx1Parameter() const final {
    return {channels.getFrequencyRX(txChnl), getRpsDw(getRx1Dr()), 0};
  };
  TransmitionParameters getRx2Parameter() const final { return rx2Parameter; };

  int8_t pow2dBm(uint8_t const powerIndex) const final {
    if (powerIndex > maxPowerIndex) {
      return InvalidPower;
    }

    return MaxEIRP - 2 * powerIndex;
  }

  OsDeltaTime getDwn2SafetyZone() const final { return DNW2_SAFETY_ZONE; };
  bool validRx1DrOffset(uint8_t const drOffset) const final {
    return drOffset < limitRX1DrOffset;
  }

  void initDefaultChannels() override {
    PRINT_DEBUG(2, F("Init Default Channel"));
    rx2Parameter = {default_Freq_RX2, rps_t(default_rps_RX2), 0};
    setRx1DrOffset(0);

    channels = ChannelListType();

    for (uint8_t chnl = 0; chnl < nbFixedChannels; chnl++) {
      channels.configure(chnl, table_get_u4(defaultChannelFreq, chnl),
                         defaultChannelDrMap);
    }
  };

  void disableChannel(uint8_t channel) final { channels.disable(channel); };
  void handleCFList(const uint8_t *ptr) final {
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
  };

  bool validMapChannels(uint8_t const chMaskCntl, uint16_t const chMask) final {
    // Bad page
    if (chMaskCntl != 0 && chMaskCntl != 6)
      return false;

    //  disable all channel
    if (chMaskCntl == 0 && chMask == 0)
      return false;

    return true;
  };

  void mapChannels(uint8_t const chMaskCntl, uint16_t const chMask) final {
    // LoRaWAN™ 1.0.2 Regional Parameters §2.1.5
    // ChMaskCntl=6 => All channels ON
    if (chMaskCntl == 6) {
      channels.enableAll();
      return;
    }

    for (uint8_t chnl = 0; chnl < ChannelListType::LIMIT_CHANNELS; chnl++) {
      if ((chMask & (1u << chnl)) != 0) {
        channels.enable(chnl);
      } else {
        channels.disable(chnl);
      }
    }
  };
  void updateTxTimes(OsDeltaTime airtime) final {
    channels.updateAvailabitility(txChnl, os_getTime(), airtime);

    PRINT_DEBUG(
        2, F("Updating info for TX channel %d, airtime will be %" PRIu32 "."),
        txChnl, airtime.tick());
  };

  OsTime nextTx(OsTime now) final {

    bool channelFound = false;
    OsTime nextTransmitTime;
    // next channel or other (random)
    uint8_t nextChannel = txChnl + 1 + (rand.uint8() % 2);

    for (uint8_t channelIndex = 0;
         channelIndex < ChannelListType::LIMIT_CHANNELS; channelIndex++) {
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
  };

  OsTime initJoinLoop() final {
    txChnl = rand.uint8() % 3;
    adrTxPow = MaxEIRP;
    setDrJoin(MaxJoinDR);
    auto startTime =
        channels.getAvailability(0) + OsDeltaTime::rnd_delay(rand, 8);
    PRINT_DEBUG(1,
                F("Init Join loop : avail=%" PRIu32 " startTime=%" PRIu32 ""),
                channels.getAvailability(0).tick(), startTime.tick());
    return startTime;
  };
  TimeAndStatus nextJoinState() final {
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
  };

  void setRx2Parameter(uint32_t const rx2frequency,
                       dr_t const rx2datarate) final {
    rx2Parameter = {rx2frequency, getRpsDw(rx2datarate), 0};
  };
  void setRx2DataRate(dr_t const rx2datarate) final {
    rx2Parameter.rps = getRpsDw(rx2datarate);
  };
  void setRx1DrOffset(uint8_t drOffset) final { rx1DrOffset = drOffset; };

  void setDrJoin(dr_t dr) { datarate = dr; }
  virtual void setDrTx(uint8_t dr) final { datarate = dr; }
  virtual void setAdrTxPow(int8_t newPower) final { adrTxPow = newPower; }
  virtual bool setAdrToMaxIfNotAlreadySet() final {
    if (adrTxPow != MaxEIRP) {
      adrTxPow = MaxEIRP;
      return true;
    }
    return false;
  };

  // decrease data rate
  dr_t decDR(dr_t const dr) const { return dr == 0 ? 0 : dr - 1; };
  // in range
  bool validDR(dr_t const dr) const final { return dr < MaxDr; };
  // decrease data rate by n steps
  dr_t lowerDR(dr_t const dr, uint8_t const n) const {
    return dr < n ? 0 : dr - n;
  };
  rps_t getRps(dr_t const dr) const {
    return rps_t(table_get_u1(dr_table, dr));
  };
  rps_t getRpsDw(dr_t const dr) const {
    auto val = rps_t(table_get_u1(dr_table, dr));
    val.nocrc = true;
    return val;
  };

  void reduceDr(uint8_t diff) final { setDrTx(lowerDR(datarate, diff)); }

#if defined(ENABLE_SAVE_RESTORE)
private:
  void saveStateCommun(StoringAbtract &store) const {
    store.write(txChnl);
    store.write(adrTxPow);
    store.write(datarate);
    store.write(rx1DrOffset);
    store.write(rx2Parameter);
  };
  void loadStateCommun(RetrieveAbtract &store) {
    store.read(txChnl);
    store.read(adrTxPow);
    store.read(datarate);
    store.read(rx1DrOffset);
    store.read(rx2Parameter);
  };

public:
  virtual void saveState(StoringAbtract &store) const final {
    channels.saveState(store);
    saveStateCommun(store);
  };
  virtual void saveStateWithoutTimeData(StoringAbtract &store) const final {

    channels.saveStateWithoutTimeData(store);
    saveStateCommun(store);
  };
  virtual void loadState(RetrieveAbtract &store) final {

    channels.loadState(store);
    loadStateCommun(store);
  };
  virtual void loadStateWithoutTimeData(RetrieveAbtract &store) final {

    channels.loadStateWithoutTimeData(store);
    loadStateCommun(store);
  };
#endif

  DynamicRegionalChannelParams(LmicRand &arand) : rand{arand} {};

protected:
  void setRegionalDutyCycleVerification(bool enabled) final {
    channels.setCheckDutyCycle(enabled);
  };

  LmicRand &rand;
  ChannelListType channels;

  // channel for next TX
  uint8_t txChnl = 0;
  // ADR adjusted TX power, limit power to this value.
  // dBm
  int8_t adrTxPow = 0;
  // current data rate
  dr_t datarate = 0;
  // 1 RX window DR offset
  uint8_t rx1DrOffset = 0;
  // Number of join requests sent
  uint8_t joinCount = 0;
  TransmitionParameters rx2Parameter;

private:
  dr_t getRx1Dr() const { return lowerDR(datarate, rx1DrOffset); };
};

} // namespace DYNAMIC_CHANNEL
#endif