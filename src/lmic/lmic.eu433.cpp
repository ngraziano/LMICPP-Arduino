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
#include "lmic.eu433.h"
#include "lmic_table.h"
#include <algorithm>

// Default frequency plan for EU 433
enum {
  EU433_F1 = 433175000, // SF7-12
  EU433_F2 = 433375000, // SF7-12
  EU433_F3 = 433575000, // SF7-12
  EU433_R2 = 434665000,
};

namespace EU433 {


extern CONST_TABLE2(uint8_t, _DR2RPS_CRC)[] = {
    rps_DR0, rps_DR1, rps_DR2, rps_DR3, rps_DR4, rps_DR5, rps_DR6};

CONST_TABLE2(uint32_t, _defaultChannels)[] = {EU433_F1, EU433_F2, EU433_F3};

} // namespace EU433


LmicEu433::LmicEu433(Radio &aradio)
    : Lmic(aradio, aes, rand, channelParams), rand(aes), channelParams(rand) {}
