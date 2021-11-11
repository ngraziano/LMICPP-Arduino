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

class LmicDynamicChannel : public Lmic {
public:

#if defined(ENABLE_SAVE_RESTORE)
  virtual void saveState(StoringAbtract &store) const final;
  virtual void saveStateWithoutTimeData(StoringAbtract &store) const final;
  virtual void loadState(RetrieveAbtract &store) final;
  virtual void loadStateWithoutTimeData(RetrieveAbtract &store) final;
#endif

  bool setupChannel(uint8_t channel, uint32_t newfreq, uint16_t drmap) override = 0;

protected:
  explicit LmicDynamicChannel(Radio &radio,
                              uint8_t aMaxEIRP, dr_t aMaxJoinDr,
                              dr_t aMinJoinDr, Bands &aBands);


  uint32_t getTxFrequency() const final;
  int8_t getTxPower() const final;
  FrequencyAndRate getRx1Parameter() const final;

  uint8_t getRawRps(dr_t dr) const override =0;
  int8_t pow2dBm(uint8_t powerIndex) const override =0;
  OsDeltaTime getDwn2SafetyZone() const final;
  OsDeltaTime dr2hsym(dr_t dr) const override =0;
  bool validRx1DrOffset(uint8_t drOffset) const override =0;

  void initDefaultChannels() override;

  void disableChannel(uint8_t channel) final;
  void handleCFList(const uint8_t *ptr) final;

  bool validMapChannels(uint8_t chpage, uint16_t chmap) final;
  void mapChannels(uint8_t chpage, uint16_t chmap) final;
  void updateTxTimes(OsDeltaTime airtime) final;
  OsTime nextTx(OsTime now) final;
  void initJoinLoop() final;
  bool nextJoinState() final;
  FrequencyAndRate defaultRX2Parameter() const override = 0;

  void setRegionalDutyCycleVerification(bool enabled) final;


  const int8_t MaxEIRP;
  const dr_t MaxJoinDR;
  const dr_t MinJoinDR;
  ChannelList channels;
  // channel for next TX
  uint8_t txChnl = 0;

private:

  uint32_t getRx1Frequency() const;
  dr_t getRx1Dr() const;
};

#endif