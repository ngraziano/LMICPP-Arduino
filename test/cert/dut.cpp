
#include "dut.h"

#include "certificationprotocol.h"
#include "hal/print_debug.h"
#include "keyhandler.h"
namespace {

RadioFake radio;
LmicEu868 LMIC(radio);
VersionDetails version = {0x01, 0x00, 0x00, 0x00};
VersionDetails lorawanVersion = {0x01, 0x00, 0x03, 0x00};
VersionDetails lorawanRpVersion = {0x01, 0x00, 0x00, 0x00};
CertificationProtocol cert(LMIC, version, lorawanVersion, lorawanRpVersion);

OsTime nextSend;

// Application in string format.
constexpr char const appEui[] = "0000000000000001";
// Device EUI in string format.
constexpr char const devEui[] = "0000000000000002";
// Application key in string format.
constexpr char const appKey[] = "00000000000000010000000000000002";

void on_event(EventType evt) { cert.handle(evt); };

bool should_send_data() {
  return !LMIC.getOpMode().test(OpState::TXRXPEND) && nextSend < os_getTime();
}

OsDeltaTime interval_to_send() {
  return cert.getPeriodicity() == OsDeltaTime(0) ? OsDeltaTime::from_sec(5)
                                                 : cert.getPeriodicity();
}

} // namespace

namespace dut {
void reset() {
  os_init();
  LMIC.init();
  LMIC.reset();
  LMIC.setEventCallBack(on_event);
  LMIC.setClockError(5);
  SetupLmicKey<appEui, devEui, appKey>::setup(LMIC);
  nextSend = os_getTime();
}

OsDeltaTime loop() {
  if (should_send_data()) {
    uint8_t val = 1;
    nextSend = os_getTime() + interval_to_send();
    // send fake data
    LMIC.setTxData2(1, &val, 1, false);
  }
  auto freeTimeBeforeNextCall = LMIC.run();
  auto timeToNextPacket = nextSend - os_getTime();

  if (timeToNextPacket < freeTimeBeforeNextCall &&
      timeToNextPacket > OsDeltaTime(0)) {
    return timeToNextPacket;
  }
  return freeTimeBeforeNextCall;
}

RadioFake::Packet wait_for_data(OsDeltaTime timeout) {
  return wait_for_data(os_getTime() + timeout);
}

RadioFake::Packet wait_for_data(OsTime timeout) {
  RadioFake::Packet packet;
  do {
    auto toWait = loop();
    packet = radio.popLastSend();
    if (!packet.is_valid() && toWait != OsInfiniteDeltaTime) {
      auto wait = OsDeltaTime(toWait.tick() / 2 + 1);
      if (toWait.to_ms() > 100) {
        wait = toWait + OsDeltaTime::from_ms(-90);
        PRINT_DEBUG(1, F("Skip %d ms (speedup clock)"), wait.to_ms());
      }
      hal_add_time_in_sleep(wait);
    }

  } while (!packet.is_valid() && os_getTime() < timeout);
  return packet;
}

void send_data(RadioFake::Packet const &packet) { radio.simulateRx(packet); }

} // namespace dut