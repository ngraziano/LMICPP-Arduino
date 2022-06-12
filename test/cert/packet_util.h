#ifndef cert_packet_test_h
#define cert_packet_test_h


#include "lmic/radio_fake.h"

void printpacket(RadioFake::Packet const &packet);
bool is_join_request(RadioFake::Packet const &packet);
bool is_data(RadioFake::Packet const &packet);
uint16_t get_dev_nonce(RadioFake::Packet const &packet);

RadioFake::Packet make_join_response(RadioFake::Packet packet);

#endif