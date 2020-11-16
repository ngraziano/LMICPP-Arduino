/*******************************************************************************

 *******************************************************************************/

#include "radio_fake.h"
#include "../hal/print_debug.h"

#include "../aes/lmic_aes.h"
#include "bufferpack.h"
#include "lmic.h"
#include "lmic_table.h"

#include <algorithm>

void RadioFake::init() { PRINT_DEBUG(1, F("Radio Init")); }

// get random seed from wideband noise rssi
void RadioFake::init_random(uint8_t randbuf[16]) {
  PRINT_DEBUG(1, F("Init random"));
  PRINT_DEBUG(1, F("Fake init with constant values"));

  uint8_t i = 0;
  std::generate_n(randbuf, 16, [&i]() { return i++; });
}

uint8_t RadioFake::rssi() const { return 0; }

// called by hal ext IRQ handler
// (radio goes to stanby mode after tx/rx operations)
uint8_t RadioFake::handle_end_rx(FrameBuffer &frame) {
  PRINT_DEBUG(1, F("Handle end rx"));
  uint8_t length =
      std::min(simulateReceiveSize, static_cast<uint8_t>(frame.max_size()));
  // max frame size 64
  std::copy(simulateReceive, simulateReceive + length, begin(frame));
  simulateReceiveSize = 0;
  return length;
}

void RadioFake::handle_end_tx() const { PRINT_DEBUG(1, F("Handle end tx")); }

void RadioFake::rst() const { PRINT_DEBUG(1, F("Radio reset")); }

constexpr uint8_t crForLog(rps_t const rps) {
  return (5 - static_cast<uint8_t>(CodingRate::CR_4_5) +
          static_cast<uint8_t>(rps.getCr()));
}

CONST_TABLE(uint16_t, BW_ENUM_TO_VAL)[] = {125, 250, 500, 0};

uint16_t bwForLog(rps_t const rps) {
  auto index = static_cast<uint8_t>(rps.getBw());
  return TABLE_GET_U2(BW_ENUM_TO_VAL, index);
}

void RadioFake::tx(uint32_t const freq, rps_t const rps, int8_t const txpow,
                   uint8_t const *const framePtr, uint8_t const frameLength) {
  endOfOperation = hal_ticks() + Lmic::calcAirTime(rps, frameLength);
  char buffer[64 * 2];
  char *pos = buffer;
  std::for_each(framePtr, framePtr + frameLength, [&pos](uint8_t const elem) {
    // *pos= elem;
    sprintf(pos, "%02X", elem);
    pos += 2;
  });
  PRINT_DEBUG(1,
              F("TXDATA: { \"ts\":%" PRIu32 ", \"f\":%" PRIu32
                ", \"len\":%d, \"SF\":%d, \"BW\":%d, \"CR\":\"4/%d\", "
                "\"pow\":%d, \"data\":\"%s\" }"),
              endOfOperation, freq, frameLength, rps.sf + 6, bwForLog(rps),
              crForLog(rps), txpow, buffer);
}

void RadioFake::rx(uint32_t const freq, rps_t const rps, uint8_t const rxsyms,
                   OsTime const rxtime) {
  // now instruct the radio to receive
  // busy wait until exact rx time
  if (rxtime < os_getTime()) {
    PRINT_DEBUG(1, F("RX LATE :  %" PRIu32 " WANTED, late %" PRIi32 " ms"),
                rxtime, (os_getTime() - rxtime).to_ms());
  }
  hal_waitUntil(rxtime);
  if (simulateReceiveSize > 0) {
    endOfOperation = rxTime + Lmic::calcAirTime(rps, simulateReceiveSize);

  } else {
    endOfOperation = hal_ticks() + Lmic::calcAirTime(rps, rxsyms);
  }

  PRINT_DEBUG(1,
              F("RXMODE, freq=%" PRIu32 ", rxsymb=%d, SF=%d, BW=%d, CR=4/%d"),
              freq, rxsyms, rps.sf + 6, bwForLog(rps), crForLog(rps));
}

/**
 * Check the IO pin.
 * Return true if the radio has finish it's operation
 */
bool RadioFake::io_check() const { return (endOfOperation < hal_ticks()); }

void RadioFake::simulateRx(OsTime const timeOfReceive,
                           uint8_t const *const buffer, uint8_t const size) {
  rxTime = timeOfReceive;
  simulateReceiveSize = std::min(size, (uint8_t)64);
  std::copy(buffer, buffer + simulateReceiveSize, simulateReceive);
}

RadioFake::RadioFake(lmic_pinmap const &pins) : Radio(pins) {}
