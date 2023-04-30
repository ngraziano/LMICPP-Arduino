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

//! \file
#include "../hal/print_debug.h"

#include "bufferpack.h"
#include "lmic.eu868.h"
#include "lmic_table.h"
#include <algorithm>

// Default frequency plan for EU 868MHz ISM band
// Bands:
//  g1 :   1%  14dBm
//  g2 : 0.1%  14dBm
//  g3 :  10%  27dBm
//                 freq             band     datarates
enum {
  EU868_F1 = 868100000, // g1   SF7-12
  EU868_F2 = 868300000, // g1   SF7-12 FSK SF7/250
  EU868_F3 = 868500000, // g1   SF7-12
  EU868_F4 = 868850000, // g2   SF7-12
  EU868_F5 = 869050000, // g2   SF7-12
  EU868_F6 = 869525000, // g3   SF7-12
};

namespace EU868 {

constexpr Dr DR_DNW2 = Dr::SF12;

CONST_TABLE2(uint8_t, _DR2RPS_CRC)
[] = {rps_DR0, rps_DR1, rps_DR2, rps_DR3, rps_DR4, rps_DR5, rps_DR6};

CONST_TABLE2(uint32_t, _defaultChannels)[] = {EU868_F1, EU868_F2, EU868_F3};

} // namespace EU868


LmicEu868::LmicEu868(Radio &aradio)
    : Lmic(aradio, aes, rand, channelParams), rand(aes), channelParams(rand) {}