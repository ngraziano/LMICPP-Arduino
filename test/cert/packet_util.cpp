
#include "packet_util.h"
#include "aes/lmic_aes.h"
#include "aesdecrypt.h"

constexpr std::array<uint8_t, 16> APPK_KEY = {
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x02};

constexpr std::array<uint8_t, 8> JOIN_EUI = {0x01, 0x00, 0x00, 0x00,
                                             0x00, 0x00, 0x00, 0x00};

constexpr std::array<uint8_t, 8> DEV_EUI = {0x02, 0x00, 0x00, 0x00,
                                            0x00, 0x00, 0x00, 0x00};

constexpr std::array<uint8_t, 4> DEVADDR = {0x01, 0x01, 0x01, 0x01};

constexpr std::array<uint8_t, 3> NETID = {0x01, 0x01, 0x01};

uint8_t joinNonce = 0;

Aes get_Aes() {
  Aes aes;
  aes.setDevKey(APPK_KEY);
  aes.setNetworkSessionKey(APPK_KEY);
  aes.setApplicationSessionKey(APPK_KEY);
  return aes;
}

void printpacket(RadioFake::Packet const &packet) {
  printf("PACKET: ");
  for (uint8_t i = 0; i < packet.length; i++) {
    printf("%02x ", packet.data[i]);
  }
  printf("\n");
}

// Join packet :
// MHDR  JoinEUI  DevEUI  DevNonce MIC
// 00 0100000000000000 0200000000000000 CB7A BEBC1902
bool is_join_request(RadioFake::Packet const &packet) {
  return packet.data[0] == 0x00 && packet.length == 1 + 8 + 8 + 2 + 4 &&
         std::equal(packet.data.begin() + 1, packet.data.begin() + 9,
                    JOIN_EUI.begin()) &&
         std::equal(packet.data.begin() + 9, packet.data.begin() + 17,
                    DEV_EUI.begin()) &&
         get_Aes().verifyMic0(packet.data.data(), packet.length);
}

uint16_t get_dev_nonce(RadioFake::Packet const &packet) {
  return packet.data[17] | (packet.data[18] << 8);
}

bool is_data(RadioFake::Packet const &packet) {
  return packet.data[0] == 0x40;
}

// Join accept packet :
// MHDR {JoinNonce NetID DevAddr DLSettings RxDelay [CFList] MIC}
// {} is aes128_decrypt with the app key
RadioFake::Packet make_join_response(RadioFake::Packet const packet) {
  RadioFake::Packet response;
  response.length = 17;
  response.data[0] = 0b00100000;
  joinNonce++;
  response.data[1] = joinNonce >> 16;
  response.data[2] = joinNonce >> 8;
  response.data[3] = joinNonce;
  std::copy(NETID.begin(), NETID.end(), response.data.begin() + 4);
  std::copy(DEVADDR.begin(), DEVADDR.end(), response.data.begin() + 7);
  // Dlsettings
  response.data[11] = 0b00000000;
  // RxDelay
  response.data[12] = 0b00000001;

  get_Aes().appendMic0(response.data.begin(), response.length);
  aes128_decrypt(APPK_KEY, response.data.begin() + 1, 16);
  return response;
}