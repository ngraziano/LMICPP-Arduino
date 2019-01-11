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
  ChannelDetail() = default;
  constexpr ChannelDetail(uint32_t raw, uint16_t drMap)
      : raw(raw), drMap(drMap){};
  constexpr ChannelDetail(uint32_t frequency, int8_t band, uint16_t drMap)
      : raw((frequency & ~(uint32_t)3) | band), drMap(drMap){};
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
};

//! \internal
struct band_t {
  uint16_t txcap;   // duty cycle limitation: 1/txcap
  uint8_t lastchnl; // last used channel
  OsTime avail;     // channel is blocked until this time
};

enum { BAND_MILLI = 0, BAND_CENTI = 1, BAND_DECI = 2, BAND_AUX = 3 };

class LmicEu868 final : public Lmic {
public:
  // Max supported channels
  static const uint8_t MAX_CHANNELS = 16;
  static const uint8_t MAX_BANDS = 4;
  enum class Dr : dr_t { SF12 = 0, SF11, SF10, SF9, SF8, SF7, SF7B, FSK, NONE };

  explicit LmicEu868(lmic_pinmap const &pins, OsScheduler &scheduler);

protected:
  uint8_t getRawRps(dr_t dr) const override;
  int8_t pow2dBm(uint8_t powerIndex) const override;
  OsDeltaTime getDwn2SafetyZone() const override;
  OsDeltaTime dr2hsym(dr_t dr) const override;
  uint32_t convFreq(const uint8_t *ptr) const override;
  bool validRx1DrOffset(uint8_t drOffset) const override;

  void initDefaultChannels(bool join) override;

  bool setupChannel(uint8_t channel, uint32_t newfreq, uint16_t drmap) override;

  void disableChannel(uint8_t channel) override;
  void handleCFList(const uint8_t *ptr) override;

  bool mapChannels(uint8_t chpage, uint16_t chmap) override;
  void updateTx(OsTime txbeg, OsDeltaTime airtime) override;
  OsTime nextTx(OsTime now) override;
  void setRx1Params() override;
#if !defined(DISABLE_JOIN)
  void initJoinLoop() override;
  bool nextJoinState() override;
#endif
  dr_t defaultRX2Dr() const override;
  uint32_t defaultRX2Freq() const override;

private:
  band_t bands[MAX_BANDS]{};
  ChannelList<MAX_CHANNELS> channels;

  uint32_t getFreq(uint8_t channel) const;
  uint8_t getBand(uint8_t channel) const;
  bool setupBand(uint8_t bandidx, uint16_t txcap);
};

#endif