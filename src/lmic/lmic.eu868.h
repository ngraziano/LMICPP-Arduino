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
#include "lmicdynamicchannel.h"

namespace EU868 {

constexpr int8_t MaxEIRPValue = 16;

constexpr uint32_t FREQ_DNW2 = 869525000;
constexpr rps_t rps_DNW2 =
    rps_t{SF12, BandWidth::BW125, CodingRate::CR_4_5, true};

} // namespace EU868

class Eu868RegionalChannelParams final
    : public DYNAMIC_CHANNEL::DynamicRegionalChannelParams<
          EU868::MaxEIRPValue, 5, 0, 7, EU868::FREQ_DNW2, EU868::rps_DNW2> {
public:
  enum class Dr : dr_t { SF12 = 0, SF11, SF10, SF9, SF8, SF7, SF7B, FSK, NONE };
  bool setupChannel(uint8_t channel, uint32_t newfreq, uint16_t drmap) final;
  int8_t pow2dBm(uint8_t powerIndex) const final;
  bool validRx1DrOffset(uint8_t drOffset) const final;
  void initDefaultChannels() final;
  Eu868RegionalChannelParams(LmicRand &arand);

private:
  BandsEu868 bandeu;
};

class LmicEu868 final : public Lmic {
public:
  explicit LmicEu868(Radio &radio);

private:
  Aes aes;
  LmicRand rand;
  Eu868RegionalChannelParams channelParams;
};

#endif