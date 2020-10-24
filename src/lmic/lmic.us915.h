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

#ifndef _lmic_us915_h_
#define _lmic_us915_h_

#include "lmic.h"

class LmicUs915 final : public Lmic {
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

  explicit LmicUs915(Radio &radio);

  bool setupChannel(uint8_t channel, uint32_t newfreq, uint16_t drmap) final;
  void selectSubBand(uint8_t band);

protected:
  uint32_t getTxFrequency() const final;
  int8_t getTxPower() const final;
  FrequencyAndRate getRx1Parameter() const final;
  uint8_t getRawRps(dr_t dr) const final;

  int8_t pow2dBm(uint8_t powerIndex) const final;
  OsDeltaTime getDwn2SafetyZone() const final;
  OsDeltaTime dr2hsym(dr_t dr) const final;
  uint32_t convFreq(const uint8_t *ptr) const final;
  bool validRx1DrOffset(uint8_t drOffset) const final;

  void initDefaultChannels() final;

  void disableChannel(uint8_t channel) final;
  void handleCFList(const uint8_t *ptr) final;

  bool validMapChannels(uint8_t chpage, uint16_t chmap) final;
  void mapChannels(uint8_t chpage, uint16_t chmap) final;
  void updateTxTimes(OsDeltaTime airtime) final;
  OsTime nextTx(OsTime now) final;
  void initJoinLoop() final;
  bool nextJoinState() final;
  FrequencyAndRate defaultRX2Parameter() const final;

private:
  uint16_t channelMap[(72 + 15) / 16]; // enabled bits
  uint16_t chRnd;
  // channel for next TX
  uint8_t txChnl = 0;

  void enableChannel(uint8_t channel);
  void enableSubBand(uint8_t band);
  void disableSubBand(uint8_t band);
  uint32_t getRx1Frequency() const;
  dr_t getRx1Dr() const;
};

#endif
