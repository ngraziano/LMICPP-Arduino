#ifndef _lmic_eu868_h_
#define _lmic_eu868_h_

#include "lmic.h"

#undef min
#undef max

#include <valarray>
struct ChannelDetail {
  // three low bit of freq is used to store band.
  uint32_t freq;
  uint16_t drMap;
};

enum { MAX_CHANNELS = 16 }; //!< Max supported channels
enum { MAX_BANDS = 4 };

enum { LIMIT_CHANNELS = (1 << 4) }; // EU868 will never have more channels
//! \internal
struct band_t {
  uint16_t txcap;   // duty cycle limitation: 1/txcap
  int8_t txpow;     // maximum TX power
  uint8_t lastchnl; // last used channel
  OsTime avail;     // channel is blocked until this time
};

enum { BAND_MILLI = 0, BAND_CENTI = 1, BAND_DECI = 2, BAND_AUX = 3 };

class LmicEu868 final : public Lmic {
  public:
  LmicEu868(lmic_pinmap const &pins);
protected:
  uint8_t getRawRps(dr_t dr) const override;
  int8_t pow2dBm(uint8_t mcmd_ladr_p1) const override;
  OsDeltaTime getDwn2SafetyZone() const override;
  OsDeltaTime dr2hsym(dr_t dr) const override;
  uint32_t convFreq(const uint8_t *ptr) const override;
  bool validRx1DrOffset(uint8_t drOffset) const override;

  void initDefaultChannels(bool join) override;

  bool setupChannel(uint8_t channel, uint32_t newfreq, uint16_t drmap,
                    int8_t band) override;

  void disableChannel(uint8_t channel) override;
  void handleCFList(const uint8_t *ptr) override;

  uint8_t mapChannels(uint8_t chpage, uint16_t chmap) override;
  void updateTx(OsTime txbeg, OsDeltaTime airtime);
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
  ChannelDetail channels[MAX_CHANNELS] = {};
  uint16_t channelMap = 0;

  uint32_t getFreq(uint8_t channel) const;
  uint8_t getBand(uint8_t channel) const;
  bool setupBand(uint8_t bandidx, int8_t txpow, uint16_t txcap);
};

#endif