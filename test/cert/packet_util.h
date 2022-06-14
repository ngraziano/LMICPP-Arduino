#ifndef cert_packet_test_h
#define cert_packet_test_h


#include "lmic/radio_fake.h"
#include "aes/lmic_aes.h"

#include <vector>


struct TestServerState {
    Aes aes;
    uint8_t joinNonce = 0;
    uint32_t fCntUp = 0;
    uint32_t fCntDown = 0;
};

void printpacket(RadioFake::Packet const &packet);
bool is_join_request(RadioFake::Packet const &packet);
bool is_data(RadioFake::Packet const &packet);
bool is_confirmed_uplink(RadioFake::Packet const &packet);
uint16_t get_dev_nonce(RadioFake::Packet const &packet);

RadioFake::Packet make_join_response(RadioFake::Packet packet, TestServerState &state);
RadioFake::Packet make_data_response(uint8_t port, std::vector<uint8_t> const & data,bool acknowledged, TestServerState &state);
void read_join_key(uint16_t devNonce, TestServerState &state);
bool check_is_next_packet(RadioFake::Packet const &packet, TestServerState &state);

#endif