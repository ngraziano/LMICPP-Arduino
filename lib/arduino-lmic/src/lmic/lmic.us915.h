#ifndef _lmic_us915_h_
#define _lmic_us915_h_

#include "regionlmic.h"

enum {
  MAX_XCHANNELS = 2
}; // extra channels in RAM, channels 0-71 are immutable


class LmicUs915 final : public RegionLmic {
protected:
  uint8_t getRawRps(dr_t dr) const override;
public:
  LmicUs915(LmicRand &rand);
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
  uint32_t xchFreq[MAX_XCHANNELS]; // extra channel frequencies (if device is
                                   // behind a repeater)
  uint16_t
      xchDrMap[MAX_XCHANNELS]; // extra channel datarate ranges  ---XXX: ditto
  uint16_t channelMap[(72 + MAX_XCHANNELS + 15) / 16]; // enabled bits
  uint16_t chRnd;

  void enableChannel(uint8_t channel);
  void enableSubBand(uint8_t band);
  void disableSubBand(uint8_t band);
  void selectSubBand(uint8_t band);
};

#endif