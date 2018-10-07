#ifndef _lmic_eu868_h_
#define _lmic_eu868_h_

#include "regionlmic.h"

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

class LmicEu868 final : public RegionLmic {
protected:
  uint8_t getRawRps(dr_t dr) const override;
public:
  LmicEu868(LmicRand &rand);
  static int8_t pow2dBm(uint8_t mcmd_ladr_p1);
  static OsDeltaTime getDwn2SafetyZone();
  static OsDeltaTime dr2hsym(dr_t dr);
  static uint32_t convFreq(const uint8_t *ptr);
  static bool validRx1DrOffset(uint8_t drOffset);

  void initDefaultChannels(bool join);
  bool setupChannel(uint8_t channel, uint32_t newfreq, uint16_t drmap,
                    int8_t band);
  void disableChannel(uint8_t channel);
  void handleCFList(const uint8_t *ptr);

  uint8_t mapChannels(uint8_t chpage, uint16_t chmap);
  void updateTx(OsTime const &txbeg, uint8_t globalDutyRate,
                OsDeltaTime const &airtime, uint8_t txChnl, int8_t adrTxPow,
                uint32_t &freq, int8_t &txpow, OsTime &globalDutyAvail);
  OsTime nextTx(OsTime const &now, dr_t datarate, uint8_t &txChnl);
  void setRx1Params(uint8_t txChnl, uint8_t rx1DrOffset, dr_t &dndr,
                    uint32_t &freq);
#if !defined(DISABLE_JOIN)
  void initJoinLoop(uint8_t &txChnl, int8_t &adrTxPow, dr_t &newDr,
                    OsTime &txend);
  bool nextJoinState(uint8_t &txChnl, uint8_t &txCnt, dr_t &datarate,
                     OsTime &txend);
#endif
  uint8_t defaultRX2Dr() const override;
  uint32_t defaultRX2Freq() const override;


private:
  LmicRand &rand;
  band_t bands[MAX_BANDS]{};
  ChannelDetail channels[MAX_CHANNELS] = {};
  uint16_t channelMap = 0;

  uint32_t getFreq(uint8_t channel) const;
  uint8_t getBand(uint8_t channel) const;
  bool setupBand(uint8_t bandidx, int8_t txpow, uint16_t txcap);
};

#endif