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

#include "lmic.h"

struct ChannelDetail {
private:
  // three low bit of freq is used to store band.
  uint32_t raw{};
  uint16_t drMap{};

public:
  constexpr uint32_t getFrequency() const { return raw & ~(uint32_t)3; };
  constexpr uint32_t getBand() const { return raw & 0x3; };
  constexpr uint16_t getDrMap() const { return drMap; };
  constexpr bool isDrActive(dr_t datarate) const {
    return (drMap & (1 << datarate)) != 0;
  };
  ChannelDetail() = default;
  constexpr ChannelDetail(uint32_t raw, uint16_t drMap)
      : raw(raw), drMap(drMap){};
  constexpr ChannelDetail(uint32_t frequency, int8_t band, uint16_t drMap)
      : raw((frequency & ~(uint32_t)3) | band), drMap(drMap){};
  
#if defined(ENABLE_SAVE_RESTORE)
  size_t saveState(uint8_t *buffer) const {
    // TODO raw
    // TODO drMap
    return 4+2;
  };
#endif
};

// Channel map is store in one 16bit
enum { LIMIT_CHANNELS = 16 };

template <uint8_t size> class ChannelList {
private:
  ChannelDetail channels[size] = {};
  uint16_t channelMap;
  static_assert(size <= LIMIT_CHANNELS, "Number of channel too large.");

public:
  void disableAll() { channelMap = 0; }
  void disable(uint8_t channel) { channelMap &= ~(1 << channel); }
  void enable(uint8_t channel) {
    // ignore - channel is not defined
    if (channels[channel].getFrequency() != 0) {
      channelMap |= (1 << channel);
    }
  }
  void enableAll() {
    for (uint8_t channel = 0; channel < size; channel++) {
      enable(channel);
    }
  }
  bool is_enable(uint8_t channel) const { return channelMap & (1 << channel); }
  void configure(uint8_t channel, ChannelDetail detail) {
    channels[channel] = detail;
    channelMap |= 1 << channel;
  }

  ChannelDetail const &operator[](int channel) const {
    return channels[channel];
  }

#if defined(ENABLE_SAVE_RESTORE)
  size_t saveState(uint8_t *buffer) const {
    // TODO channels
    // TODO channelMap

    return size * 6+ 2;
  };
#endif
};

enum { BAND_MILLI = 0, BAND_CENTI = 1, BAND_DECI = 2 };

class BandsEu868 {
public:
  void init(LmicRand &rand, uint8_t maxchannels);
  void updateBandAvailability(uint8_t band, OsTime lastusage,
                              OsDeltaTime duration);
  void print_state() const;
  int8_t getNextAvailableBand(OsTime const max_delay, uint8_t bmap) const;
  OsTime getAvailability(uint8_t band) { return avail[band]; };
  uint8_t getLastChannel(uint8_t band) { return lastchnl[band]; };
  void setLastChannel(uint8_t band, uint8_t lastChannel) {
    lastchnl[band] = lastChannel;
  };

  static constexpr uint8_t MAX_BAND = 3;
  static constexpr uint8_t FULL_MAP = (1 << MAX_BAND) - 1;

#if defined(ENABLE_SAVE_RESTORE)
  size_t saveState(uint8_t *buffer) const;
#endif

private:
  OsTime avail[MAX_BAND];
  uint8_t lastchnl[MAX_BAND];
};

struct band_t {
  uint16_t txcap;   // duty cycle limitation: 1/txcap
  uint8_t lastchnl; // last used channel
  OsTime avail;     // channel is blocked until this time
};

class LmicEu868 final : public Lmic {
public:
  // Max supported channels
  static const uint8_t MAX_CHANNELS = 16;

  enum class Dr : dr_t { SF12 = 0, SF11, SF10, SF9, SF8, SF7, SF7B, FSK, NONE };

  explicit LmicEu868(lmic_pinmap const &pins, OsScheduler &scheduler);

#if defined(ENABLE_SAVE_RESTORE)
  virtual size_t saveState(uint8_t *buffer) const override;
#endif

protected:
  uint8_t getRawRps(dr_t dr) const override;
  int8_t pow2dBm(uint8_t powerIndex) const override;
  OsDeltaTime getDwn2SafetyZone() const override;
  OsDeltaTime dr2hsym(dr_t dr) const override;
  uint32_t convFreq(const uint8_t *ptr) const override;
  bool validRx1DrOffset(uint8_t drOffset) const override;

  void initDefaultChannels() override;

  bool setupChannel(uint8_t channel, uint32_t newfreq, uint16_t drmap) override;

  void disableChannel(uint8_t channel) override;
  void handleCFList(const uint8_t *ptr) override;

  bool mapChannels(uint8_t chpage, uint16_t chmap) override;
  void updateTx(OsTime txbeg, OsDeltaTime airtime) override;
  OsTime nextTx(OsTime now) override;
  void setRx1Params() override;
  void initJoinLoop() override;
  bool nextJoinState() override;
  dr_t defaultRX2Dr() const override;
  uint32_t defaultRX2Freq() const override;

private:
  BandsEu868 bands;
  ChannelList<MAX_CHANNELS> channels;
  // channel for next TX
  uint8_t txChnl = 0;

  uint32_t getFreq(uint8_t channel) const;
  uint8_t getBand(uint8_t channel) const;
};

#endif