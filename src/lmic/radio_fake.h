/*******************************************************************************

 *******************************************************************************/

#ifndef radio_fake_h
#define radio_fake_h

#include "lorabase.h"
#include "osticks.h"
#include "radio.h"
#include <array>
#include <stdint.h>

class RadioFake final : public Radio {
private:
  std::array<uint8_t, 64> simulateReceive;
  uint8_t simulateReceiveSize = 0;
  OsTime rxTime;

  OsTime endOfOperation;

public:
  explicit RadioFake(lmic_pinmap const &pins);

  void simulateRx(OsTime timeofreceive, uint8_t const *buffer, uint8_t size);

  void init() final;
  void rst() const final;
  void tx(uint32_t freq, rps_t rps, int8_t txpow, uint8_t const *framePtr,
          uint8_t frameLength) final;
  void rx(uint32_t freq, rps_t rps, uint8_t rxsyms, OsTime rxtime) final;

  void init_random(std::array<uint8_t, 16> &randbuf) final;
  uint8_t handle_end_rx(FrameBuffer &frame) final;
  void handle_end_tx() const final;
  bool io_check() const final;

  uint8_t rssi() const final;
};

#endif