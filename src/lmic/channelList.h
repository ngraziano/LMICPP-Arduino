#ifndef channel_list_h
#define channel_list_h

#include "bands.h"
#include "bufferpack.h"
#include "lorabase.h"
#include <array>
#include <stdint.h>

struct ChannelDetail {
private:
  // three low bit of freq is used to store band.
  uint32_t frequency{};
  uint32_t frequencyRX{};
  uint16_t drMap{};

public:
  constexpr uint32_t getFrequency() const { return frequency; };
  constexpr uint32_t getFrequencyRX() const { return frequencyRX; };
  constexpr uint16_t getDrMap() const { return drMap; };
  constexpr bool isConfigured() const { return drMap != 0; }
  constexpr bool isDrActive(dr_t datarate) const {
    return (drMap & (1 << datarate)) != 0;
  };
  constexpr ChannelDetail() = default;
  constexpr ChannelDetail(uint32_t aFrequency, uint16_t adrMap)
      : frequency(aFrequency), frequencyRX(aFrequency), drMap(adrMap){};

#if defined(ENABLE_SAVE_RESTORE)
  void saveState(StoringAbtract &store) const {
    store.write(frequency);
    store.write(frequencyRX);
    store.write(drMap);
  };

  void loadState(RetrieveAbtract &store) {
    store.read(frequency);
    store.read(frequencyRX);
    store.read(drMap);
  };
#endif
};

template <typename BandsType,  uint16_t defaultChannelDrMap,uint32_t... defaultChannelFreq> class ChannelList {
public:
  // Channel map store a maximum of 16 channel
  // (in rp_2-1.0.1 all dynamic channel region have a minumum of 16 )
  constexpr static const uint8_t LIMIT_CHANNELS = 16;
  constexpr static const uint8_t NB_FIXED_CHANNELS = sizeof...(defaultChannelFreq);
  constexpr static const uint16_t DEFAULT_CHANNEL_DR_MAP = defaultChannelDrMap;

private:
  std::array<ChannelDetail, LIMIT_CHANNELS> channels = {};
  uint16_t channelMap = 0;
  BandsType bands;
  bool checkDutyCycle = true;

  uint8_t getBand(uint8_t const channel) const {
    return bands.getBandForFrequency(getFrequency(channel));
  }

public:
  constexpr ChannelList() {
    uint8_t chnl = 0;
    for (const auto frequency : {defaultChannelFreq...}) {
      configure(chnl++, frequency, defaultChannelDrMap);
    }
  }

  void disable(uint8_t channel) { channelMap &= ~(1 << channel); }
  void enable(uint8_t channel) {
    // ignore - channel is not defined
    if (channels[channel].isConfigured()) {
      channelMap |= (1 << channel);
    }
  }
  void enableAll() {
    for (uint8_t channel = 0; channel < channels.max_size(); channel++) {
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

  void updateAvailabitility(uint8_t const channel, OsTime const txbeg,
                            OsDeltaTime const airtime) {
    // Update band specific duty cycle stats
    bands.updateBandAvailability(getBand(channel), txbeg, airtime);
  }

  OsTime getAvailability(uint8_t const channel) const {
    if (!checkDutyCycle) {
      return hal_ticks();
    }
    auto band = getBand(channel);
    return bands.getAvailability(band);
  };

  uint32_t getFrequency(uint8_t const channel) const {
    return channels[channel].getFrequency();
  };

  uint32_t getFrequencyRX(uint8_t const channel) const {
    return channels[channel].getFrequencyRX();
  };

  void setCheckDutyCycle(bool check) { checkDutyCycle = check; }

#if defined(ENABLE_SAVE_RESTORE)
  void saveState(StoringAbtract &store) const {
    bands.saveState(store);
    saveStateWithoutTimeData(store);
  };

  void saveStateWithoutTimeData(StoringAbtract &store) const {
    for (auto &&channel : channels) {
      channel.saveState(store);
    }

    store.write(channelMap);
  };

  void loadState(RetrieveAbtract &store) {
    bands.loadState(store);
    loadStateWithoutTimeData(store);
  };

  void loadStateWithoutTimeData(RetrieveAbtract &store) {
    for (auto &&channel : channels) {
      channel.loadState(store);
    }
    store.read(channelMap);
  };
#endif
};

#endif