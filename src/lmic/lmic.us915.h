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

#ifndef lmic_us915_h
#define lmic_us915_h

#include "lmic.h"

class Us915RegionalChannelParams : public RegionalChannelParams {
public:
  enum Dr : dr_t {
    SF10 = 0,
    SF9,
    SF8,
    SF7,
    SF8C,
    // Only for Downlink
    SF12CR = 8,
    SF11CR,
    SF10CR,
    SF9CR,
    SF8CR,
    SF7CR
  };

  // increase data rate
  dr_t incDR(dr_t dr) const;
  // decrease data rate
  dr_t decDR(dr_t dr) const;
  // in range
  bool validDR(dr_t dr) const final;
  // decrease data rate by n steps
  dr_t lowerDR(dr_t dr, uint8_t n) const;

  virtual void reduceDr(uint8_t diff) final {
    setDrTx(lowerDR(datarate, diff));
  }

  bool setupChannel(uint8_t channel, uint32_t newfreq, uint16_t drmap) final;
  void selectSubBand(uint8_t band);
  void setRegionalDutyCycleVerification(bool) final{};
  void setDrJoin(dr_t dr) { datarate = dr; }
  virtual void setDrTx(uint8_t dr) final { datarate = dr; }
  virtual void setAdrTxPow(int8_t newPower) final { adrTxPow = newPower; }
  virtual bool setAdrToMaxIfNotAlreadySet() final;

  uint32_t getTxFrequency() const;
  int8_t getTxPower() const;
  FrequencyAndRate getTxParameter() const final;
  FrequencyAndRate getRx1Parameter() const final;
  uint8_t getRawRps(dr_t dr) const final;

  int8_t pow2dBm(uint8_t powerIndex) const final;
  OsDeltaTime getDwn2SafetyZone() const final;
  bool validRx1DrOffset(uint8_t drOffset) const final;

  void initDefaultChannels() final;

  void disableChannel(uint8_t channel) final;
  void handleCFList(const uint8_t *ptr) final;

  bool validMapChannels(uint8_t chpage, uint16_t chmap) final;
  void mapChannels(uint8_t chpage, uint16_t chmap) final;
  void updateTxTimes(OsDeltaTime airtime) final;
  OsTime nextTx(OsTime now) final;
  OsTime initJoinLoop() final;
  TimeAndStatus nextJoinState() final;
  FrequencyAndRate defaultRX2Parameter() const final;
  void setRx1DrOffset(uint8_t drOffset) final;

#if defined(ENABLE_SAVE_RESTORE)
  virtual void saveState(StoringAbtract &store) const final;
  virtual void saveStateWithoutTimeData(StoringAbtract &store) const final;
  virtual void loadState(RetrieveAbtract &store) final;
  virtual void loadStateWithoutTimeData(RetrieveAbtract &store) final;
#endif

  Us915RegionalChannelParams(LmicRand &arand);

private:
  // enabled bits
  std::array<uint16_t, (72 + 15) / 16> channelMap;
  uint16_t chRnd = 0;
  // channel for next TX
  uint8_t txChnl = 0;
  // ADR adjusted TX power, limit power to this value.
  // dBm
  int8_t adrTxPow = 0;
  dr_t datarate = 0; // current data rate
  // 1 RX window DR offset
  uint8_t rx1DrOffset = 0;

  LmicRand &rand;

  void enableChannel(uint8_t channel);
  void enableSubBand(uint8_t band);
  void disableSubBand(uint8_t band);
  uint32_t getRx1Frequency() const;
  dr_t getRx1Dr() const;
};

class LmicUs915 final : public Lmic {
public:
  explicit LmicUs915(Radio &radio);

private:
  Aes aes;
  LmicRand rand;
  
  Us915RegionalChannelParams channelParams;
};

#endif
