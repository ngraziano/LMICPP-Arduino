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

#ifndef lmic_lmicdynamicchannel_h
#define lmic_lmicdynamicchannel_h

#include "bands.h"
#include "bufferpack.h"
#include "channelList.h"
#include "lmic.h"

class DynamicRegionalChannelParams : public RegionalChannelParams {
public:
  bool setupChannel(uint8_t channel, uint32_t newfreq,
                    uint16_t drmap) override = 0;
  uint32_t getTxFrequency() const;
  int8_t getTxPower() const;
  TransmitionParameters getTxParameter() const final;
  TransmitionParameters getRx1Parameter() const final;
  TransmitionParameters getRx2Parameter() const final;
  virtual uint8_t getRawRps(dr_t dr) const = 0;

  int8_t pow2dBm(uint8_t powerIndex) const override = 0;
  OsDeltaTime getDwn2SafetyZone() const final;
  bool validRx1DrOffset(uint8_t drOffset) const override = 0;

  void initDefaultChannels() override;

  void disableChannel(uint8_t channel) final;
  void handleCFList(const uint8_t *ptr) final;

  bool validMapChannels(uint8_t chpage, uint16_t chmap) final;
  void mapChannels(uint8_t chpage, uint16_t chmap) final;
  void updateTxTimes(OsDeltaTime airtime) final;
  OsTime nextTx(OsTime now) final;
  OsTime initJoinLoop() final;
  TimeAndStatus nextJoinState() final;

  void setRx2Parameter(uint32_t rx2frequency, dr_t rx2datarate) final;
  void setRx2DataRate(dr_t rx2datarate) final;
  void setRx1DrOffset(uint8_t drOffset) final;

  void setDrJoin(dr_t dr) { datarate = dr; }
  virtual void setDrTx(uint8_t dr) final { datarate = dr; }
  virtual void setAdrTxPow(int8_t newPower) final { adrTxPow = newPower; }
  virtual bool setAdrToMaxIfNotAlreadySet() final;

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

#if defined(ENABLE_SAVE_RESTORE)
  virtual void saveState(StoringAbtract &store) const final;
  virtual void saveStateWithoutTimeData(StoringAbtract &store) const final;
  virtual void loadState(RetrieveAbtract &store) final;
  virtual void loadStateWithoutTimeData(RetrieveAbtract &store) final;
#endif

  DynamicRegionalChannelParams(LmicRand &arand, uint8_t aMaxEIRP,
                               dr_t aMaxJoinDr, dr_t aMinJoinDr, Bands &aBands);

protected:
  void setRegionalDutyCycleVerification(bool enabled) final;
  LmicRand &rand;
  const int8_t MaxEIRP;
  const dr_t MaxJoinDR;
  const dr_t MinJoinDR;
  ChannelList channels;

  // channel for next TX
  uint8_t txChnl = 0;
  // ADR adjusted TX power, limit power to this value.
  // dBm
  int8_t adrTxPow = 0;
  // current data rate
  dr_t datarate = 0;
  // 1 RX window DR offset
  uint8_t rx1DrOffset = 0;
  // Number of join requests sent
  uint8_t joinCount = 0;
  TransmitionParameters rx2Parameter;

private:
  uint32_t getRx1Frequency() const;
  dr_t getRx1Dr() const;
};

#endif