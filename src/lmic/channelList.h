#ifndef channel_list_h
#define channel_list_h

#include "bufferpack.h"
#include "lorabase.h"
#include <stdint.h>

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

  uint8_t getBand(uint8_t const channel) const {
    return bands.getBandForFrequency(getFrequency(channel));
  }


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
    saveStateWithoutTimeData(store);
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
    for (uint8_t channel = 0; channel < size; channel++) {
      channels[channel].loadState(store);
    }
    store.read(channelMap);
  };
#endif
};

#endif