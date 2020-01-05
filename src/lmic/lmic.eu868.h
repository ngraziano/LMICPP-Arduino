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

#include "band.eu868.h"
#include "bufferpack.h"
#include "channelList.h"
#include "lmic.h"

class LmicEu868 final : public Lmic {
public:
  // Max supported channels
  static const uint8_t MAX_CHANNELS = 16;

  enum class Dr : dr_t { SF12 = 0, SF11, SF10, SF9, SF8, SF7, SF7B, FSK, NONE };

  explicit LmicEu868(Radio &radio, OsScheduler &scheduler);

#if defined(ENABLE_SAVE_RESTORE)
  virtual void saveState(StoringAbtract &store) const final;
  virtual void saveStateWithoutTimeData(StoringAbtract &store) const final;
  virtual void loadState(RetrieveAbtract &store) final;
  virtual void loadStateWithoutTimeData(RetrieveAbtract &store) final;
#endif

protected:
  uint32_t getTxFrequency() const final;
  uint32_t getRx1Frequency() const final;
  uint8_t getRawRps(dr_t dr) const final;
  int8_t pow2dBm(uint8_t powerIndex) const final;
  OsDeltaTime getDwn2SafetyZone() const final;
  OsDeltaTime dr2hsym(dr_t dr) const final;
  uint32_t convFreq(const uint8_t *ptr) const final;
  bool validRx1DrOffset(uint8_t drOffset) const final;

  void initDefaultChannels() final;

  bool setupChannel(uint8_t channel, uint32_t newfreq, uint16_t drmap) final;

  void disableChannel(uint8_t channel) final;
  void handleCFList(const uint8_t *ptr) final;

  bool validMapChannels(uint8_t chpage, uint16_t chmap) final;
  void mapChannels(uint8_t chpage, uint16_t chmap) final;
  int8_t updateTx(OsTime txbeg, OsDeltaTime airtime) final;
  OsTime nextTx(OsTime now) final;
  void setRx1Params() final;
  void initJoinLoop() final;
  bool nextJoinState() final;
  dr_t defaultRX2Dr() const final;
  uint32_t defaultRX2Freq() const final;

private:
  ChannelList<MAX_CHANNELS, BandsEu868> channels;
  // channel for next TX
  uint8_t txChnl = 0;
};

#endif