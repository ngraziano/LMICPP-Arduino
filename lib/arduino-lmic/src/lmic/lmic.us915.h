#ifndef _lmic_us915_h_
#define _lmic_us915_h_

#include "lmic.h"

enum {
  MAX_XCHANNELS = 2
}; // extra channels in RAM, channels 0-71 are immutable

class LmicUs915 final : public Lmic {
public:
  LmicUs915(lmic_pinmap const& pins);

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