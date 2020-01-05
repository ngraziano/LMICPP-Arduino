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

#ifndef _lmic_eu868_h_
#define _lmic_eu868_h_

#include "band.eu868.h"
#include "bufferpack.h"
#include "lmic.h"

struct ChannelDetail {
private:
  // three low bit of freq is used to store band.
  uint32_t frequency{};
  uint16_t drMap{};

public:
  constexpr uint32_t getFrequency() const { return frequency; };
  constexpr uint16_t getDrMap() const { return drMap; };
  constexpr bool isConfigured() const { return drMap != 0; }
  constexpr bool isDrActive(dr_t datarate) const {
    return (drMap & (1 << datarate)) != 0;
  };
  constexpr ChannelDetail() = default;
  constexpr ChannelDetail(uint32_t aFrequency, uint16_t adrMap)
      : frequency(aFrequency), drMap(adrMap){};

#if defined(ENABLE_SAVE_RESTORE)
  void saveState(StoringAbtract &store) const {
    store.write(frequency);
    store.write(drMap);
  };

  void loadState(RetrieveAbtract &store) {
    store.read(frequency);
    store.read(drMap);
  };
#endif
};

// Channel map is store in one 16bit
enum { LIMIT_CHANNELS = 16 };

template <uint8_t size, class BandType> class ChannelList {
private:
  ChannelDetail channels[size] = {};
  uint16_t channelMap;
  BandType bands;

  static_assert(size <= LIMIT_CHANNELS, "Number of channel too large.");

public:
  void init() { bands.init(); }
  void disableAll() { channelMap = 0; }
  void disable(uint8_t channel) { channelMap &= ~(1 << channel); }
  void enable(uint8_t channel) {
    // ignore - channel is not defined
    if (channels[channel].isConfigured()) {
      channelMap |= (1 << channel);
    }
  }
  void enableAll() {
    for (uint8_t channel = 0; channel < size; channel++) {
      enable(channel);
    }
  }
  bool is_enable(uint8_t channel) const { return channelMap & (1 << channel); }

  bool is_enable_at_dr(uint8_t channel, dr_t datarate) const {
    return (channelMap & (1 << channel)) &&
           channels[channel].isDrActive(datarate);
  }

  void configure(uint8_t channel, uint32_t const newfreq,
                 uint16_t const drmap) {
    channels[channel] = ChannelDetail{newfreq, drmap};
    channelMap |= 1 << channel;
  }

  uint8_t getBand(uint8_t const channel) const {
    return bands.getBandForFrequency(getFrequency(channel));
  }

  void updateAvailabitility(uint8_t const channel, OsTime const txbeg,
                            OsDeltaTime const airtime) {
    // Update band specific duty cycle stats
    bands.updateBandAvailability(getBand(channel), txbeg, airtime);
  }

  OsTime getAvailability(uint8_t const channel) {
    auto band = getBand(channel);
    return bands.getAvailability(band);
  };

  constexpr uint32_t getFrequency(uint8_t const channel) {
    return channels[channel].getFrequency();
  };

#if defined(ENABLE_SAVE_RESTORE)
  void saveState(StoringAbtract &store) const {
    bands.saveState(store);
    saveState(store);
  };

  void saveStateWithoutTimeData(StoringAbtract &store) const {
    for (uint8_t channel = 0; channel < size; channel++) {
      channels[channel].saveState(store);
    }
    store.write(channelMap);
  };

  void loadState(RetrieveAbtract &store) {
    bands.loadState(store);
    loadStateWithoutTimeData(store);
  };

  void loadStateWithoutTimeData(RetrieveAbtract &store) {
    bands.loadState(store);
    for (uint8_t channel = 0; channel < size; channel++) {
      channels[channel].loadState(store);
    }
    store.read(channelMap);
  };
#endif
};

class LmicEu868 final : public Lmic {
public:
  // Max supported channels
  static const uint8_t MAX_CHANNELS = 16;

  enum class Dr : dr_t { SF12 = 0, SF11, SF10, SF9, SF8, SF7, SF7B, FSK, NONE };

  explicit LmicEu868(Radio &radio, OsScheduler &scheduler);

#if defined(ENABLE_SAVE_RESTORE)
  virtual void saveState(StoringAbtract &store) const final;
  virtual void saveStateWithoutTimeData(StoringAbtract &store) const final;
  virtual void loadState(RetrieveAbtract &store) final;
  virtual void loadStateWithoutTimeData(RetrieveAbtract &store) final;
#endif

protected:
  uint8_t getRawRps(dr_t dr) const final;
  int8_t pow2dBm(uint8_t powerIndex) const final;
  OsDeltaTime getDwn2SafetyZone() const final;
  OsDeltaTime dr2hsym(dr_t dr) const final;
  uint32_t convFreq(const uint8_t *ptr) const final;
  bool validRx1DrOffset(uint8_t drOffset) const final;

  void initDefaultChannels() final;

  bool setupChannel(uint8_t channel, uint32_t newfreq, uint16_t drmap) final;

  void disableChannel(uint8_t channel) final;
  void handleCFList(const uint8_t *ptr) final;

  bool validMapChannels(uint8_t chpage, uint16_t chmap) final;
  void mapChannels(uint8_t chpage, uint16_t chmap) final;
  int8_t updateTx(OsTime txbeg, OsDeltaTime airtime) final;
  OsTime nextTx(OsTime now) final;
  void setRx1Params() final;
  void initJoinLoop() final;
  bool nextJoinState() final;
  dr_t defaultRX2Dr() const final;
  uint32_t defaultRX2Freq() const final;

private:
  ChannelList<MAX_CHANNELS, BandsEu868> channels;
  // channel for next TX
  uint8_t txChnl = 0;
};

#endif