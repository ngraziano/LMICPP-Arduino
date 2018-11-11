#ifndef _radio_h_
#define _radio_h_

#include "../hal/hal_io.h"
#include "lorabase.h"
#include "osticks.h"
#include <stdint.h>

class Radio {

public:
  void init(void);
  void rst() const;
  void tx(uint32_t freq, rps_t rps, int8_t txpow, uint8_t *framePtr,
          uint8_t frameLength) const;
  void rx(uint32_t freq, rps_t rps, uint8_t rxsyms, OsTime rxtime) const;
  void rxon(uint32_t freq, rps_t rps, uint8_t rxsyms, OsTime rxtime) const;

  void irq_handler(uint8_t dio, uint8_t *framePtr, uint8_t &frameLength,
                   OsTime &txEnd, OsTime &rxTime, rps_t currentRps) const;
  void init_random(uint8_t randbuf[16]);
  bool io_check(uint8_t *framePtr, uint8_t &frameLength, OsTime &txEnd,
                OsTime &rxTime, rps_t currentRps);
  void store_trigger();
  uint8_t rssi() const;

  Radio(lmic_pinmap const &pins);

private:
  OsTime last_int_trigger;

  HalIo hal;
  
  OsTime int_trigger_time() const;
  void writeReg(uint8_t addr, uint8_t data) const;
  uint8_t readReg(uint8_t addr) const;
  void writeBuf(uint8_t addr, uint8_t *buf, uint8_t len) const;
  void readBuf(uint8_t addr, uint8_t *buf, uint8_t len) const;
  void opmode(uint8_t mode) const;
  void opmodeLora() const;
  void configLoraModem(rps_t rps) const;
  void configChannel(uint32_t freq) const;
  void configPower(int8_t pw) const;
  void txlora(uint32_t freq, rps_t rps, int8_t txpow, uint8_t *frame,
              uint8_t dataLen) const;
  void starttx(uint32_t freq, rps_t rps, int8_t txpow, uint8_t *frame,
               uint8_t dataLen) const;
  void rxlora(uint8_t rxmode, uint32_t freq, rps_t rps, uint8_t rxsyms,
              OsTime rxtime) const;
  void startrx(uint8_t rxmode, uint32_t freq, rps_t rps, uint8_t rxsyms,
               OsTime rxtime) const;
};

#endif