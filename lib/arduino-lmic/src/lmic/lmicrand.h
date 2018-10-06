
#ifndef _lmicrand_h_
#define _lmicrand_h_

#include <stdint.h>
#include "../aes/aes.h"
class Radio;

class LmicRand {
public:
  LmicRand(Aes &aes);
  void init(Radio &radio);
  uint8_t uint8();
  uint16_t uint16();

private:
  Aes &aes;
  // (initialized by init() with radio RSSI, used by rand1())
  uint8_t randbuf[16];
};

#endif