
#include "packet_util.h"

#include "aesdecrypt.h"
#include <algorithm>

constexpr std::array<uint8_t, 16> APPK_KEY = {
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x02};

constexpr std::array<uint8_t, 8> JOIN_EUI = {0x01, 0x00, 0x00, 0x00,
                                             0x00, 0x00, 0x00, 0x00};

constexpr std::array<uint8_t, 8> DEV_EUI = {0x02, 0x00, 0x00, 0x00,
                                            0x00, 0x00, 0x00, 0x00};

constexpr uint32_t DEVADDR = 0x01010102;

constexpr std::array<uint8_t, 3> NETID = {0x01, 0x01, 0x01};

Aes get_Aes() {
  Aes aes;
  aes.setDevKey(APPK_KEY);
  aes.setNetworkSessionKey(APPK_KEY);
  aes.setApplicationSessionKey(APPK_KEY);
  return aes;
}

uint32_t getFCntUpPacket(RadioFake::Packet const &packet,
                         TestServerState &state) {
  uint32_t fCntUpPacket = rlsbf2(packet.data.cbegin() + 6);
  fCntUpPacket += state.fCntUp & 0xFFFF0000;
  if (fCntUpPacket < state.fCntUp)
    fCntUpPacket += 0x10000;
  return fCntUpPacket;
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
  return packet.data[0] == 0x40 || packet.data[0] == 0x80;
}

bool is_confirmed_uplink(RadioFake::Packet const &packet) {
  return packet.data[0] == 0x80;
}

bool is_adr(RadioFake::Packet const &packet) {
  return packet.data[1 + 4] & 0x80;
}

bool is_ack_set(RadioFake::Packet const &packet) {
  return packet.data[1 + 4] & 0x20;
}

// Join accept packet :
// MHDR {JoinNonce NetID DevAddr DLSettings RxDelay [CFList] MIC}
// {} is aes128_decrypt with the app key
RadioFake::Packet make_join_response(TestServerState &state) {
  // reinit the state
  state.aes = get_Aes();
  state.fCntUp = 0xFFFFFFFF;
  state.fCntDown = 0xFFFFFFFF;

  RadioFake::Packet response;
  response.length = 17;
  response.data[0] = 0b00100000;
  state.joinNonce++;
  response.data[1] = state.joinNonce;
  response.data[2] = state.joinNonce >> 8;
  response.data[3] = state.joinNonce >> 16;
  std::copy(NETID.begin(), NETID.end(), response.data.begin() + 4);
  response.data[7] = DEVADDR & 0xFF;
  response.data[8] = (DEVADDR >> 8) & 0xFF;
  response.data[9] = (DEVADDR >> 16) & 0xFF;
  response.data[10] = (DEVADDR >> 24) & 0xFF;

  // Dlsettings
  response.data[11] = 0b00000000;
  // RxDelay
  response.data[12] = 0b00000001;

  state.aes.appendMic0(response.data.begin(), response.length);
  aes128_decrypt(APPK_KEY, response.data.begin() + 1, 16);
  return response;
}

void read_join_key(uint16_t devNonce, TestServerState &state) {
  std::array<uint8_t, 6> data;
  data[0] = state.joinNonce;
  data[1] = state.joinNonce >> 8;
  data[2] = state.joinNonce >> 16;
  std::copy(NETID.begin(), NETID.end(), data.begin() + 3);
  state.aes.sessKeys(devNonce, data.begin());
}

bool check_is_next_packet(RadioFake::Packet const &packet,
                          TestServerState &state) {
  const uint32_t addr = rlsbf4(packet.data.cbegin() + 1);

  if (addr != DEVADDR)
    return false;

  state.fCntUp++;
  // recover 32bit of fcntUp
  uint32_t fCntUpPacket = getFCntUpPacket(packet, state);

  if (fCntUpPacket < state.fCntUp) {
    return false;
  }

  state.fCntUp = fCntUpPacket;
  return state.aes.verifyMic(DEVADDR, fCntUpPacket, PktDir::UP,
                             packet.data.data(), packet.length);
}

RadioFake::Packet make_data_response(uint8_t port,
                                     std::vector<uint8_t> const &data,
                                     bool acknowledged, TestServerState &state,
                                     bool confirmed,
                                     std::vector<uint8_t> const &fOpts) {
  state.fCntDown++;
  RadioFake::Packet response;
  response.data[0] = confirmed ? 0b10100000 : 0b01100000;
  wlsbf4(response.data.begin() + 1, DEVADDR);
  // FCtrl
  constexpr size_t maxOptsLen = 15;
  uint8_t fOptsLen = std::min(fOpts.size(), maxOptsLen) & 0xFF;
  response.data[5] = fOptsLen | (acknowledged ? 0b00100000 : 0);
  // FCnt
  wlsbf2(response.data.begin() + 6, state.fCntDown);
  // FOpts
  std::copy(fOpts.begin(), fOpts.begin() + fOptsLen, response.data.begin() + 8);
  // Port
  response.data[8 + fOptsLen] = port;
  // Data
  std::copy(data.begin(), data.end(), 9 + fOptsLen + response.data.begin());

  response.length = 8 + 1 + fOptsLen + data.size() + 4;
  state.aes.framePayloadEncryption(port, DEVADDR, state.fCntDown, PktDir::DOWN,
                                   response.data.begin() + 8 + 1 + fOptsLen,
                                   data.size());

  state.aes.appendMic(DEVADDR, state.fCntDown, PktDir::DOWN,
                      response.data.begin(), response.length);
  printpacket(response);
  return response;
}

RadioFake::Packet make_empty_response(bool acknowledged,
                                      TestServerState &state) {
  state.fCntDown++;
  RadioFake::Packet response;
  response.data[0] = 0b01100000;
  wlsbf4(response.data.begin() + 1, DEVADDR);
  // FCtrl
  response.data[5] = 0b00000000 | (acknowledged ? 0b00100000 : 0);
  wlsbf2(response.data.begin() + 6, state.fCntDown);
  response.length = 8 + 4;
  state.aes.appendMic(DEVADDR, state.fCntDown, PktDir::DOWN,
                      response.data.begin(), response.length);
  printpacket(response);
  return response;
}

uint8_t get_opts_len(RadioFake::Packet const &packet) {
  return packet.data[5] & 0x0F;
}

std::vector<uint8_t> get_mac_command_values(RadioFake::Packet const &packet,
                                            TestServerState const &state) {

  // read length from FHDR
  uint8_t fOptsLen = get_opts_len(packet);
  if (fOptsLen > 0) {
    std::vector<uint8_t> values;
    for (uint8_t i = 8; i < packet.length && i < 8 + fOptsLen; i++) {
      values.push_back(packet.data[i]);
    }
    return values;
  } else {
    if (get_port(packet) == 0) {
      return get_payload(packet, state);
    } else {
      return std::vector<uint8_t>();
    }
  }
}

std::vector<uint8_t> get_payload(RadioFake::Packet const &packet,
                                 TestServerState const &state) {
  // read length from FHDR
  uint8_t fOptsLen = get_opts_len(packet);

  if (packet.length - 4 <= 8 + fOptsLen + 1)
    return std::vector<uint8_t>();
  auto temp = packet.data;
  state.aes.framePayloadEncryption(get_port(packet), DEVADDR, state.fCntUp,
                                   PktDir::UP, temp.begin() + 8 + fOptsLen + 1,
                                   packet.length - 8 - fOptsLen - 1 - 4);
  std::vector<uint8_t> values;
  for (uint8_t i = 8 + fOptsLen + 1; i < packet.length - 4; i++) {
    values.push_back(temp[i]);
  }
  return values;
}

uint8_t get_port(RadioFake::Packet const &packet) {
  auto position = 8 + get_opts_len(packet);
  if (position >= packet.length - 4) {
    return 0;
  }
  return packet.data[position];
}