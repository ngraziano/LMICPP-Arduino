
#include "lmicrand.h"
#include "radio.h"

LmicRand::LmicRand(Aes &aes) : aes(aes) {}

void LmicRand::init(Radio &radio) { radio.init_random(randbuf); }

// return next random byte derived from seed buffer
// (buf[0] holds index of next byte to be returned)
uint8_t LmicRand::uint8() {
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
uint16_t LmicRand::uint16() { return ((uint16_t)((uint8() << 8) | uint8())); }
