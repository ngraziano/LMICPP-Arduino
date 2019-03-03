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

#ifndef _lmicrand_h_
#define _lmicrand_h_

#include <stdint.h>

class Radio;
class Aes;

class LmicRandFromAes {
public:
  explicit LmicRandFromAes(Aes &aes);
  void init(Radio &radio);
  uint8_t uint8();
  uint16_t uint16();

private:
  Aes &aes;
  // (initialized by init() with radio RSSI, used by rand1())
  uint8_t randbuf[16];
};

#ifdef ARDUINO_ARCH_ESP32

class LmicRandEsp {
public:
  explicit LmicRandFromAes(Aes &){};
  void init(Radio &){};
  uint8_t uint8();
  uint16_t uint16();
};
using LmicRand = LmicRandEsp;

#else
using LmicRand = LmicRandFromAes;
#endif

#endif