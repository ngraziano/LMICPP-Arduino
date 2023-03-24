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

#ifndef _lmic_eu433_h_
#define _lmic_eu433_h_

#include "bufferpack.h"
#include "channelList.h"
#include "lmic.h"
#include "lmicdynamicchannel.h"

class Eu433RegionalChannelParams : public DynamicRegionalChannelParams {
public:
  enum class Dr : dr_t { SF12 = 0, SF11, SF10, SF9, SF8, SF7, SF7B, FSK, NONE };

  bool setupChannel(uint8_t channel, uint32_t newfreq, uint16_t drmap) final;
  Eu433RegionalChannelParams(LmicRand &arand);

  uint8_t getRawRps(dr_t dr) const final;
  int8_t pow2dBm(uint8_t powerIndex) const final;
  bool validRx1DrOffset(uint8_t drOffset) const final;

  void initDefaultChannels() final;
  FrequencyAndRate defaultRX2Parameter() const final;

private:
  BandSingle bands{100};
};

class LmicEu433 final : public Lmic {
public:
  explicit LmicEu433(Radio &radio);

private:
  Aes aes;
  LmicRand rand;
  Eu433RegionalChannelParams channelParams;
};

#endif