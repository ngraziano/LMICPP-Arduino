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


#include "lmicrand.h"
#include "radio.h"
#include "../aes/lmic_aes.h"

LmicRandFromAes::LmicRandFromAes(Aes &aes) : aes(aes) {}

void LmicRandFromAes::init(Radio &radio) {
  radio.init_random(randbuf);
  // set initial index
  randbuf[0] = 16;
}

// return next random byte derived from seed buffer
// (buf[0] holds index of next byte to be returned)
uint8_t LmicRandFromAes::uint8() {
  uint8_t i = randbuf[0];

  if (i == 16) {
    aes.encrypt(randbuf, 16); // encrypt seed with any key
    i = 0;
  }
  uint8_t v = randbuf[i++];
  randbuf[0] = i;
  return v;
}

//! Get random number (default impl for uint16_t).
uint16_t LmicRandFromAes::uint16() { return ((uint16_t)((uint8() << 8) | uint8())); }
