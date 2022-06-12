#ifndef test_cert_dut_h
#define test_cert_dut_h

#include "LMIC.h"
#include "lmic/radio_fake.h"

namespace dut {

void reset();
OsDeltaTime loop();
RadioFake::Packet wait_for_data(OsTime timeout);
RadioFake::Packet wait_for_data(OsDeltaTime timeout);

void send_data(RadioFake::Packet const &packet);

} // namespace dut
#endif