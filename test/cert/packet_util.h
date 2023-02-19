#ifndef cert_packet_test_h
#define cert_packet_test_h

#include "aes/lmic_aes.h"
#include "lmic/radio_fake.h"

#include <vector>

struct TestServerState {
  Aes aes;
  uint8_t joinNonce = 0;
  uint32_t fCntUp = 0;
  uint32_t fCntDown = 0;
};

constexpr OsDeltaTime JOIN_ACCEPT_DELAY1 = OsDeltaTime::from_sec(5);

constexpr OsDeltaTime JOIN_ACCEPT_DELAY2 = OsDeltaTime::from_sec(6);

// constexpr OsDeltaTime RECEIVE_DELAY1;
constexpr OsDeltaTime RECEIVE_DELAY2 = OsDeltaTime::from_sec(2);

void printpacket(RadioFake::Packet const &packet);
bool is_join_request(RadioFake::Packet const &packet);
bool is_data(RadioFake::Packet const &packet);
bool is_confirmed_uplink(RadioFake::Packet const &packet);
bool is_adr(RadioFake::Packet const &packet);
bool is_ack_set(RadioFake::Packet const &packet);
uint16_t get_dev_nonce(RadioFake::Packet const &packet);
uint8_t get_port(RadioFake::Packet const &packet);

RadioFake::Packet make_join_response(TestServerState &state);
RadioFake::Packet make_data_response(uint8_t port,
                                     std::vector<uint8_t> const &data,
                                     bool acknowledged, TestServerState &state,
                                     bool confirmed = false,
                                     std::vector<uint8_t> const &fOpts = {});
RadioFake::Packet make_empty_response(bool acknowledged,
                                      TestServerState &state);
void read_join_key(uint16_t devNonce, TestServerState &state);
bool check_is_next_packet(RadioFake::Packet const &packet,
                          TestServerState &state);
std::vector<uint8_t> get_mac_command_values(RadioFake::Packet const &packet,
                                            TestServerState const &state);

std::vector<uint8_t> get_payload(RadioFake::Packet const &packet,
                                 TestServerState const &state);

#endif