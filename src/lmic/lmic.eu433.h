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

namespace EU433 {

// normaly 12.5
constexpr int8_t MaxEIRPValue = 12;
constexpr uint8_t MaxPowerIndex = 5;

constexpr uint32_t FREQ_DNW2 = 434665000;

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

using Bands = BandSingle<100>;

constexpr uint32_t FREQ_MIN = 433050000;
constexpr uint32_t FREQ_MAX = 434665000;

enum class Dr : dr_t { SF12 = 0, SF11, SF10, SF9, SF8, SF7, SF7B, FSK, NONE };

// Default frequency plan for EU 433
constexpr uint32_t F1 = 433175000; // SF7-12
constexpr uint32_t F2 = 433375000; // SF7-12
constexpr uint32_t F3 = 433575000; // SF7-12

} // namespace EU433

using EU433Channels =
    ChannelList<EU433::Bands, dr_range_map(EU433::Dr::SF12, EU433::Dr::SF7),
                EU433::F1, EU433::F2, EU433::F3>;

using Eu433RegionalChannelParams =
    DYNAMIC_CHANNEL::DynamicRegionalChannelParams<
        EU433Channels, EU433::MaxEIRPValue, 5, 0,
        EU433::RESOLVE_TABLE(_DR2RPS_CRC), 7, EU433::FREQ_DNW2, EU433::rps_DNW2,
        EU433::MaxPowerIndex, EU433::limitRX1DrOffset, EU433::FREQ_MIN,
        EU433::FREQ_MAX>;

class LmicEu433 final : public Lmic {
public:
  explicit LmicEu433(Radio &radio);

private:
  Aes aes;
  LmicRand rand;
  Eu433RegionalChannelParams channelParams;
};

#endif