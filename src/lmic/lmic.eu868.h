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
constexpr uint8_t MaxPowerIndex = 7;

constexpr uint32_t FREQ_DNW2 = 869525000;
constexpr rps_t rps_DNW2 =
    rps_t{SF12, BandWidth::BW125, CodingRate::CR_4_5, true};

constexpr uint8_t rps_DR0 = rps_t{SF12, BandWidth::BW125, CodingRate::CR_4_5};
constexpr uint8_t rps_DR1 = rps_t{SF11, BandWidth::BW125, CodingRate::CR_4_5};
constexpr uint8_t rps_DR2 = rps_t{SF10, BandWidth::BW125, CodingRate::CR_4_5};
constexpr uint8_t rps_DR3 = rps_t{SF9, BandWidth::BW125, CodingRate::CR_4_5};
constexpr uint8_t rps_DR4 = rps_t{SF8, BandWidth::BW125, CodingRate::CR_4_5};
constexpr uint8_t rps_DR5 = rps_t{SF7, BandWidth::BW125, CodingRate::CR_4_5};
constexpr uint8_t rps_DR6 = rps_t{SF7, BandWidth::BW250, CodingRate::CR_4_5};

extern CONST_TABLE2(uint8_t, _DR2RPS_CRC)[];

constexpr uint8_t limitRX1DrOffset = 6;

constexpr uint32_t FREQ_MIN = 863000000;
constexpr uint32_t FREQ_MAX = 870000000;

enum class Dr : dr_t { SF12 = 0, SF11, SF10, SF9, SF8, SF7, SF7B, FSK, NONE };

constexpr uint32_t F1 = 868100000; // g1   SF7-12
constexpr uint32_t F2 = 868300000; // g1   SF7-12 FSK SF7/250
constexpr uint32_t F3 = 868500000; // g1   SF7-12

} // namespace EU868

using EU868Channels =
    ChannelList<BandsEu868, dr_range_map(EU868::Dr::SF12, EU868::Dr::SF7),
                EU868::F1, EU868::F2, EU868::F3>;

using Eu868RegionalChannelParams =
    DYNAMIC_CHANNEL::DynamicRegionalChannelParams<
        EU868Channels, EU868::MaxEIRPValue, 5, 0,
        EU868::RESOLVE_TABLE(_DR2RPS_CRC), 7, EU868::FREQ_DNW2, EU868::rps_DNW2,
        EU868::MaxPowerIndex, EU868::limitRX1DrOffset, EU868::FREQ_MIN,
        EU868::FREQ_MAX>;

class LmicEu868 final : public Lmic {
public:
  explicit LmicEu868(Radio &radio);

private:
  Aes aes;
  LmicRand rand;
  Eu868RegionalChannelParams channelParams;
};

#endif