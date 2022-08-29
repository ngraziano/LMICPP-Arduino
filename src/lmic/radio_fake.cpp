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
void RadioFake::init_random(std::array<uint8_t, 16> &randbuf) {
  PRINT_DEBUG(1, F("Init random"));
  PRINT_DEBUG(1, F("Fake init with constant values"));

  uint8_t i = 0;
  std::generate(begin(randbuf), end(randbuf), [&i]() { return i++; });
}

uint8_t RadioFake::rssi() const { return 0; }

// called by hal ext IRQ handler
// (radio goes to stanby mode after tx/rx operations)
uint8_t RadioFake::handle_end_rx(FrameBuffer &frame, bool) {
  if (!isReceived) {
    PRINT_DEBUG(1, F("Handle end rx without message"));
    return 0;
  }
  PRINT_DEBUG(1, F("Handle end rx"));
  uint8_t length =
      std::min(simulateReceive.length, static_cast<uint8_t>(frame.max_size()));
  // max frame size 64
  std::copy(begin(simulateReceive.data), begin(simulateReceive.data) + length,
            begin(frame));
  simulateReceive.length = 0;
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
  lastSend.freq = freq;
  lastSend.rps = rps;
  lastSend.length = frameLength;
  lastSend.time = endOfOperation;

  std::copy(framePtr, framePtr + frameLength, begin(lastSend.data));
}

OsDeltaTime timeByChar(rps_t rps) {
  // Tsymb = 2^SF / BW
  // SF = 6 + rps.sf
  // BW =  125000 * 2^rps.bwRaw

  // return OsDeltaTime::from_sec(
  //    (1 << (6 + rps.sf)) / ( 125000 * (1 << rps.bwRaw)));

  return OsDeltaTime::from_us(256 * (1 << (1 + rps.sf - rps.bwRaw)));
}


OsTime minTimeToReceive(rps_t const rps, OsTime const now) {
  // The radio can can miss 3 syncrhonisation symbols
  return now - 3 * timeByChar(rps);
}

void RadioFake::rx(uint32_t const freq, rps_t const rps, uint8_t const rxsyms,
                   OsTime const rxtime) {
  // now instruct the radio to receive
  // busy wait until exact rx time
  auto now = os_getTime();
  if (rxtime < now) {
    PRINT_DEBUG(1, F("RX LATE :  %" PRIu32 " WANTED, late %" PRIi32 " ms"),
                rxtime, (os_getTime() - rxtime).to_ms());
  }
  hal_waitUntil(rxtime);
  auto windows_end = hal_ticks() + Lmic::calcAirTime(rps, rxsyms);
  // simulate timing is good ?

  if (simulateReceive.length > 0 &&
      simulateReceive.time >= minTimeToReceive(rps, now) &&
      simulateReceive.time < windows_end) {
    endOfOperation =
        simulateReceive.time + Lmic::calcAirTime(rps, simulateReceive.length);
    isReceived = true;
  } else {
    if (simulateReceive.length > 0) {
      PRINT_DEBUG(1,
                  F("RX WANTED AT :  %" PRIu32 ", current windows %" PRIu32
                    " to %" PRIu32 " "),
                  simulateReceive.time, rxtime, windows_end);
    }
    endOfOperation = windows_end;
    isReceived = false;
  }

  PRINT_DEBUG(1,
              F("RXMODE, freq=%" PRIu32 ", rxsymb=%d, SF=%d, BW=%d, CR=4/%d"),
              freq, rxsyms, rps.sf + 6, bwForLog(rps), crForLog(rps));
}

void RadioFake::rx(uint32_t const freq, rps_t const rps) {
  // now instruct the radio to receive
  if (simulateReceive.length > 0 && simulateReceive.time > os_getTime()) {
    endOfOperation =
        simulateReceive.time + Lmic::calcAirTime(rps, simulateReceive.length);
    isReceived = true;
  }

  PRINT_DEBUG(1, F("RXMODE, freq=%" PRIu32 ", SF=%d, BW=%d, CR=4/%d"), freq,
              rps.sf + 6, bwForLog(rps), crForLog(rps));
}

/**
 * Check the IO pin.
 * Return true if the radio has finish it's operation
 */
bool RadioFake::io_check() const { return (endOfOperation < hal_ticks()); }

void RadioFake::simulateRx(Packet const &packet) { simulateReceive = packet; }

RadioFake::Packet RadioFake::popLastSend() {
  auto packet = lastSend;
  lastSend = {};
  return packet;
}

RadioFake::RadioFake() {}
