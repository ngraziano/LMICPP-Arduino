#include <unity.h>
#include <vector>

#include "activation_pretest.h"
#include "dut.h"

void sp1_intial_join(TestServerState &server_state) {
  // Step 1: send join request
  auto const data = dut::wait_for_data(OsDeltaTime::from_sec(60));
  TEST_ASSERT(data.is_valid());
  TEST_ASSERT(is_join_request(data));
  auto devNonce = get_dev_nonce(data);

  auto joinResponse = make_join_response(server_state);
  read_join_key(devNonce, server_state);
  joinResponse.time = data.time + JOIN_ACCEPT_DELAY2;
  dut::send_data(joinResponse);

  // Step 2: send data
  auto const firstPacket = dut::wait_for_data(OsDeltaTime::from_sec(60));
  TEST_ASSERT(firstPacket.is_valid());
  TEST_ASSERT(is_data(firstPacket));
  TEST_ASSERT(check_is_next_packet(firstPacket, server_state));
  TEST_ASSERT(server_state.fCntUp == 0 || server_state.fCntUp == 1);

  auto firstResponse =
      make_data_response(224, std::vector<uint8_t>{0x01},
                         is_confirmed_uplink(firstPacket), server_state);
  firstResponse.time = firstPacket.time + RECEIVE_DELAY2;
  dut::send_data(firstResponse);

  // Step 3: second join
  auto const nextJoin = dut::wait_for_data(OsDeltaTime::from_sec(60));
  TEST_ASSERT(nextJoin.is_valid());
  TEST_ASSERT(is_join_request(nextJoin));
  auto devNonce2 = get_dev_nonce(nextJoin);
  // In lorawn version 1.0.4 the devNonce must increase
  // In lorawan version 1.0.3 the devNonce only must be different from the first
  // The devNonce must be greater than the previous one.
  TEST_ASSERT(devNonce2 > devNonce);
  // TEST_ASSERT_NOT_EQUAL_UINT16(devNonce, devNonce2);

  auto joinResponse2 = make_join_response(server_state);
  read_join_key(devNonce2, server_state);
  joinResponse2.time = nextJoin.time + JOIN_ACCEPT_DELAY1;
  dut::send_data(joinResponse2);

  // Step 4: send  DUT sends Confirmed or Unconfirmed frame
  auto const secondPacket = dut::wait_for_data(OsDeltaTime::from_sec(60));
  TEST_ASSERT(secondPacket.is_valid());
  TEST_ASSERT(is_data(secondPacket));
  TEST_ASSERT(check_is_next_packet(secondPacket, server_state));
  TEST_ASSERT(server_state.fCntUp == 0 || server_state.fCntUp == 1);
  auto m = server_state.fCntUp;

  auto secondResponse =
      make_data_response(224, std::vector<uint8_t>{0x06, 0x01},
                         is_confirmed_uplink(firstPacket), server_state);
  secondResponse.time = secondPacket.time + RECEIVE_DELAY2;
  dut::send_data(secondResponse);

  // Step 5: DUT sends Confirmed or Unconfirmed frame
  // FCntUp = m + 1
  // FPort = any allowed port except 224
  auto nextPacket = dut::wait_for_data(OsDeltaTime::from_sec(60));
  TEST_ASSERT(nextPacket.is_valid());
  TEST_ASSERT(check_is_next_packet(nextPacket, server_state));
  TEST_ASSERT(server_state.fCntUp == m + 1);
  // TEST_ASSERT(get_port(message) != 224);

  // If DUT sent a Confirmed frame, then
  // The TCL sends Unconfirmed frame
  // Else, this step must be skipped
  if (is_confirmed_uplink(nextPacket)) {
    auto nextResponse =
        make_data_response(224, std::vector<uint8_t>{0x07, 0x01},
                           is_confirmed_uplink(nextPacket), server_state);
    nextResponse.time = nextPacket.time + RECEIVE_DELAY2;
    dut::send_data(nextResponse);
  }

  // Step 6:DUT sends Unconfirmed frame
  // FCntUp = m + 2
  nextPacket = dut::wait_for_data(OsDeltaTime::from_sec(60));
  TEST_ASSERT(nextPacket.is_valid());
  TEST_ASSERT(check_is_next_packet(nextPacket, server_state));
  TEST_ASSERT(!is_confirmed_uplink(nextPacket));
  TEST_ASSERT(server_state.fCntUp == m + 2);

  // if FCtrl ADR Bit = false, then
  // The TCL sends Unconfirmed frame
  // Else, this step is skipped
  if (!is_adr(nextPacket)) {
    auto nextResponse = make_data_response(
        224, std::vector<uint8_t>{0x04, 0x01}, false, server_state);
    nextResponse.time = nextPacket.time + RECEIVE_DELAY2;
    dut::send_data(nextResponse);
  }

  // Step 7: DUT sends Unconfirmed frame
  // FCntUp = m + 3
  // FCtrl ADR bit = true
  nextPacket = dut::wait_for_data(OsDeltaTime::from_sec(60));
  TEST_ASSERT(nextPacket.is_valid());
  TEST_ASSERT(check_is_next_packet(nextPacket, server_state));
  TEST_ASSERT(!is_confirmed_uplink(nextPacket));
  TEST_ASSERT(server_state.fCntUp == m + 3);
  TEST_ASSERT(is_adr(nextPacket));
  auto lastTimeOfPacket = nextPacket.time;
  // The TCL sends Unconfirmed frame
  // MAC-CMD LinkADRReq
  // DataRate = Max125kHzDR,
  // Payload = [0x]03XXXXXXXX
  // ChMaskCntl:
  //  DC = 0,
  //  FC = 6
  // ChMask:
  //  DC - Enable only default channels
  //  FC = [0x]00FF

  // For EU868, the default channels are:
  //

  auto nextResponse = make_data_response(
      0, std::vector<uint8_t>{0x03, 0x05 << 4, 0x07, 0x00, 0x00}, false,
      server_state);
  nextResponse.time = nextPacket.time + RECEIVE_DELAY2;
  dut::send_data(nextResponse);

  // Step 8 : DUT sends Unconfirmed frame in 5 seconds
  // FCntUp = m + 4
  // AC-CMD LinkADRAns Payload = [0x]0307
  nextPacket = dut::wait_for_data(OsDeltaTime::from_sec(60));
  TEST_ASSERT(nextPacket.is_valid());
  TEST_ASSERT(check_is_next_packet(nextPacket, server_state));
  TEST_ASSERT(!is_confirmed_uplink(nextPacket));
  TEST_ASSERT(server_state.fCntUp == m + 4);
  auto val = get_mac_command_values(nextPacket, server_state);
  TEST_ASSERT(get_mac_command_values(nextPacket, server_state) ==
              (std::vector<uint8_t>{0x03, 0x07}));

  printf("Intervale PACKET %d\n", (nextPacket.time - lastTimeOfPacket).to_ms());
  lastTimeOfPacket = nextPacket.time;

  // The TCL sends Unconfirmed frame
  // CP-CMD DutVersionsReq
  // FPort = 224
  // Payload = [0x]7F
  nextResponse =
      make_data_response(224, std::vector<uint8_t>{0x7F}, false, server_state);
  nextResponse.time = nextPacket.time + RECEIVE_DELAY2;
  dut::send_data(nextResponse);

  // Step 9: DUT sends Unconfirmed frame
  // FCntUp = m + 5
  // CP-CMD DutVersionsAns
  // FPort = 224
  // Payload = [0x]7FXXXXXXXXXXXXXXXXXX

  nextPacket = dut::wait_for_data(OsDeltaTime::from_sec(60));
  TEST_ASSERT(nextPacket.is_valid());
  TEST_ASSERT(check_is_next_packet(nextPacket, server_state));
  TEST_ASSERT(!is_confirmed_uplink(nextPacket));
  TEST_ASSERT(server_state.fCntUp == m + 5);
  printf("Intervale PACKET %d\n", (nextPacket.time - lastTimeOfPacket).to_ms());
  lastTimeOfPacket = nextPacket.time;

  TEST_ASSERT_EQUAL_UINT8(224, get_port(nextPacket));
  auto versionPayload = get_payload(nextPacket, server_state);
  TEST_ASSERT_EQUAL_INT(13, versionPayload.size());
  printf("Fw version %d.%d.%d.%d\n", versionPayload[1], versionPayload[2],
         versionPayload[3], versionPayload[4]);
  printf("Lrwan version %d.%d.%d.%d\n", versionPayload[5], versionPayload[6],
         versionPayload[7], versionPayload[8]);
  printf("Lrwan RP version %d.%d.%d.%d\n", versionPayload[9],
         versionPayload[10], versionPayload[11], versionPayload[12]);
}
