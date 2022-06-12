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
public:
  struct Packet {
    // frequency is zero if not a valid packet
    uint32_t freq;
    rps_t rps;
    std::array<uint8_t, 64> data;
    uint8_t length;
    // end of send time, or start of receive time
    OsTime time;
    bool is_valid() const { return freq != 0; }
  };

private:
  Packet simulateReceive;
  bool isReceived = false;

  OsTime endOfOperation;
  Packet lastSend;

public:
  explicit RadioFake();

  void simulateRx(Packet const &packet);
  Packet popLastSend();

  void init() final;
  void rst() const final;
  void tx(uint32_t freq, rps_t rps, int8_t txpow, uint8_t const *framePtr,
          uint8_t frameLength) final;
  void rx(uint32_t freq, rps_t rps, uint8_t rxsyms, OsTime rxtime) final;
  void rx(uint32_t freq, rps_t rps) final;

  void init_random(std::array<uint8_t, 16> &randbuf) final;
  uint8_t handle_end_rx(FrameBuffer &frame, bool goSleep) final;
  void handle_end_tx() const final;
  bool io_check() const final;

  uint8_t rssi() const final;
};

#endif