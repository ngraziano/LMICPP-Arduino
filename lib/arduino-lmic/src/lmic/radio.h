#ifndef _radio_h_
#define _radio_h_

#include "lorabase.h"
#include "osticks.h"
#include <stdint.h>

class Radio {

public:
  void init(void);
  void rst();
  void tx(uint32_t freq, rps_t rps, int8_t txpow);
  void rx(uint32_t freq, rps_t rps, uint8_t rxsyms, OsTime const &rxtime);
  void rxon(uint32_t freq, rps_t rps, uint8_t rxsyms, OsTime const &rxtime);

  void irq_handler(OsJobBase &nextJob, uint8_t dio, OsTime const &trigger);
  void init_random(uint8_t randbuf[16]);

  uint8_t rssi();

  Radio(uint8_t *frame, uint8_t &frameLength, OsTime &txend, OsTime &rxTime);

private:
  uint8_t *framePtr = nullptr;
  uint8_t &frameLength;

  OsTime &txEnd;
  OsTime &rxTime;

  rps_t currentRps;
};

#endif