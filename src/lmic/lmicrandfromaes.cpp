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

#ifndef ARDUINO_ARCH_ESP32

#include "lmicrand.h"
#include "radio.h"
#include "../aes/lmic_aes.h"

LmicRand::LmicRand(Aes &anaes) : aes(anaes) {}

void LmicRand::init(Radio &radio) {
  radio.init_random(randbuf);
  // set initial index to encrypt at next
  index = 16;
}

// return next random byte derived from seed buffer
uint8_t LmicRand::uint8() {
  if (index >= randbuf.size()) {
    // encrypt seed with any key
    aes.encrypt(randbuf.begin(), randbuf.size()); 
    index = 0;
  }
  return randbuf[index++];
}

//! Get random number (default impl for uint16_t).
uint16_t LmicRand::uint16() { return ((uint16_t)((uint8() << 8U) | uint8())); }

#endif