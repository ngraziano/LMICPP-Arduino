#include "common.h"

void test_RX_timing_setup_req() {
  // Step 1
  // DUT sends Unconfirmed frame
  // FCntUp = y
  auto nextPacket = dut::wait_for_data(defaultWaitTime);
  TEST_ASSERT(check_is_next_packet(nextPacket, server_state));
  TEST_ASSERT(is_data(nextPacket));
  TEST_ASSERT_FALSE(is_confirmed_uplink(nextPacket));
  auto y = server_state.fCntUp;

  // The TCL sends Unconfirmed frame
  // MAC-CMD
  // RxTimingSetupReq
  // Payload = [0x]08XX
  // Delay (i) = [3-14]
  uint8_t newDelay = 7;
  auto nextResponse = make_data_response(
      0, std::vector<uint8_t>{0x08, newDelay}, false, server_state);
  nextResponse.time = nextPacket.time + RECEIVE_DELAY2;
  dut::send_data(nextResponse);

  // Step 2
  for (uint8_t i = 0; i < 3; i++) {
    // DUT sends Unconfirmed frame
    // Repeat up to 3 times until a downlink is received confirming the receipt
    // of the RxTimingSetupAns FCntUp >= y + n MAC-CMD RxTimingSetupAns Payload
    // = [0x]08
    nextPacket = dut::wait_for_data(defaultWaitTime);
    TEST_ASSERT(check_is_next_packet(nextPacket, server_state));
    TEST_ASSERT(is_data(nextPacket));
    TEST_ASSERT_FALSE(is_confirmed_uplink(nextPacket));
    auto macResponse = get_mac_command_values(nextPacket, server_state);
    TEST_ASSERT_EQUAL_UINT(1, macResponse.size());
    TEST_ASSERT_EQUAL_UINT8(0x08, macResponse[0]);
    TEST_ASSERT_GREATER_OR_EQUAL(y + i, server_state.fCntUp);
  }

  // Step 3
  // DUT sends Unconfirmed frame
  // MAC-CMD
  // RxTimingSetupAns
  // Payload = [0x]08
  nextPacket = dut::wait_for_data(defaultWaitTime);
  TEST_ASSERT(check_is_next_packet(nextPacket, server_state));
  TEST_ASSERT(is_data(nextPacket));
  TEST_ASSERT_FALSE(is_confirmed_uplink(nextPacket));
  auto macResponse = get_mac_command_values(nextPacket, server_state);
  TEST_ASSERT_EQUAL_UINT(1, macResponse.size());
  TEST_ASSERT_EQUAL_UINT8(0x08, macResponse[0]);

  // The TCL sends Unconfirmed frame on RX1 window
  // CP-CMD EchoPayloadReq
  // FPort = 224
  // Payload = [0x]08010203
  // TXDelay = (i) seconds
  nextResponse = make_data_response(
      224, std::vector<uint8_t>{0x08, 0x01, 0x02, 0x03}, false, server_state);
  nextResponse.time = nextPacket.time + OsDeltaTime::from_sec(newDelay) +
                      OsDeltaTime::from_ms(55);
  dut::send_data(nextResponse);

  // Step 4
  // DUT sends Unconfirmed frame
  // CP-CMD EchoPayloadAns
  // FPort = 224
  // Payload = [0x]08020304
  nextPacket = dut::wait_for_data(defaultWaitTime);
  TEST_ASSERT(check_is_next_packet(nextPacket, server_state));
  TEST_ASSERT(is_data(nextPacket));
  TEST_ASSERT_FALSE(is_confirmed_uplink(nextPacket));
  TEST_ASSERT_EQUAL_UINT8(224, get_port(nextPacket));
  auto payload = get_payload(nextPacket, server_state);
  TEST_ASSERT_EQUAL_UINT8(0x08, payload[0]);
  TEST_ASSERT_EQUAL_UINT8(0x02, payload[1]);
  TEST_ASSERT_EQUAL_UINT8(0x03, payload[2]);
  TEST_ASSERT_EQUAL_UINT8(0x04, payload[3]);
  macResponse = get_mac_command_values(nextPacket, server_state);
  TEST_ASSERT_EQUAL_UINT(0, macResponse.size());

  // The TCL sends Unconfirmed frame on RX2 window
  // CP-CMD EchoPayloadReq
  // FPort = 224
  // Payload = [0x]08121314
  // TXDelay = (i + 1) seconds
  nextResponse = make_data_response(
      224, std::vector<uint8_t>{0x08, 0x12, 0x13, 0x14}, false, server_state);
  nextResponse.time = nextPacket.time + OsDeltaTime::from_sec(newDelay + 1) +
                      OsDeltaTime::from_ms(55);
  dut::send_data(nextResponse);

  // Step 5
  // DUT sends Unconfirmed frame
  // CP-CMD EchoPayloadAns
  // FPort = 224
  // Payload = [0x]08131415
  nextPacket = dut::wait_for_data(defaultWaitTime);
  TEST_ASSERT(check_is_next_packet(nextPacket, server_state));
  TEST_ASSERT(is_data(nextPacket));
  TEST_ASSERT_FALSE(is_confirmed_uplink(nextPacket));
  TEST_ASSERT_EQUAL_UINT8(224, get_port(nextPacket));
  payload = get_payload(nextPacket, server_state);
  TEST_ASSERT_EQUAL_UINT8(0x08, payload[0]);
  TEST_ASSERT_EQUAL_UINT8(0x13, payload[1]);
  TEST_ASSERT_EQUAL_UINT8(0x14, payload[2]);
  TEST_ASSERT_EQUAL_UINT8(0x15, payload[3]);

  // The TCL sends Unconfirmed frame
  //  MAC-CMD RxTimingSetupReq
  // Payload = [0x]08XX
  // Delay = 2
  nextResponse =
      make_data_response(0, std::vector<uint8_t>{0x08, 2}, false, server_state);
  nextResponse.time = nextPacket.time + OsDeltaTime::from_sec(newDelay + 1) +
                      OsDeltaTime::from_ms(55);
  dut::send_data(nextResponse);

  // Step 6
  // DUT sends Unconfirmed frame
  // Repeat up to 3 times until a downlink is received confirming the receipt of
  // the RxTimingSetupAns MAC-CMD RxTimingSetupAns Payload = [0x]08
  for (uint8_t i = 0; i < 3; i++) {
    nextPacket = dut::wait_for_data(defaultWaitTime);
    TEST_ASSERT(check_is_next_packet(nextPacket, server_state));
    TEST_ASSERT(is_data(nextPacket));
    TEST_ASSERT_FALSE(is_confirmed_uplink(nextPacket));
    macResponse = get_mac_command_values(nextPacket, server_state);
    TEST_ASSERT_EQUAL_UINT(1, macResponse.size());
    TEST_ASSERT_EQUAL_UINT8(0x08, macResponse[0]);
    TEST_ASSERT_GREATER_OR_EQUAL(y + i, server_state.fCntUp);
  }

  // Step 7
  // DUT sends Unconfirmed frame
  // MAC-CMD
  // RxTimingSetupAns
  // Payload = [0x]08
  nextPacket = dut::wait_for_data(defaultWaitTime);
  TEST_ASSERT(check_is_next_packet(nextPacket, server_state));
  TEST_ASSERT(is_data(nextPacket));
  TEST_ASSERT_FALSE(is_confirmed_uplink(nextPacket));
  macResponse = get_mac_command_values(nextPacket, server_state);
  TEST_ASSERT_EQUAL_UINT(1, macResponse.size());
  TEST_ASSERT_EQUAL_UINT8(0x08, macResponse[0]);

  // The TCL sends Unconfirmed frame on RX1 window
  // CP-CMD EchoPayloadReq
  // FPort = 224
  // Payload = [0x]08010203
  // TXDelay = 2 sec
  nextResponse = make_data_response(
      224, std::vector<uint8_t>{0x08, 0x01, 0x02, 0x03}, false, server_state);
  nextResponse.time =
      nextPacket.time + OsDeltaTime::from_sec(2) + OsDeltaTime::from_ms(55);
  dut::send_data(nextResponse);

  // Step 8
  // DUT sends Unconfirmed frame
  // CP-CMD EchoPayloadAns
  // FPort = 224
  // Payload = [0x]08020304
  nextPacket = dut::wait_for_data(defaultWaitTime);
  TEST_ASSERT(check_is_next_packet(nextPacket, server_state));
  TEST_ASSERT(is_data(nextPacket));
  TEST_ASSERT_FALSE(is_confirmed_uplink(nextPacket));
  TEST_ASSERT_EQUAL_UINT8(224, get_port(nextPacket));
  payload = get_payload(nextPacket, server_state);
  TEST_ASSERT_EQUAL_UINT8(0x08, payload[0]);
  TEST_ASSERT_EQUAL_UINT8(0x02, payload[1]);
  TEST_ASSERT_EQUAL_UINT8(0x03, payload[2]);
  TEST_ASSERT_EQUAL_UINT8(0x04, payload[3]);
  macResponse = get_mac_command_values(nextPacket, server_state);
  TEST_ASSERT_EQUAL_UINT(0, macResponse.size());

  // The TCL sends Unconfirmed frame on RX2 window
  // CP-CMD EchoPayloadReq
  // FPort = 224
  // Payload = [0x]08121314
  // TXDelay = 3 seconds
  nextResponse = make_data_response(
      224, std::vector<uint8_t>{0x08, 0x12, 0x13, 0x14}, false, server_state);
  nextResponse.time =
      nextPacket.time + OsDeltaTime::from_sec(3) + OsDeltaTime::from_ms(55);
  dut::send_data(nextResponse);

  // Step 9
  // DUT sends Unconfirmed frame
  // CP-CMD EchoPayloadAns
  // FPort = 224
  // Payload = [0x]08131415
  nextPacket = dut::wait_for_data(defaultWaitTime);
  TEST_ASSERT(check_is_next_packet(nextPacket, server_state));
  TEST_ASSERT(is_data(nextPacket));
  TEST_ASSERT_FALSE(is_confirmed_uplink(nextPacket));
  TEST_ASSERT_EQUAL_UINT8(224, get_port(nextPacket));
  payload = get_payload(nextPacket, server_state);
  TEST_ASSERT_EQUAL_UINT8(0x08, payload[0]);
  TEST_ASSERT_EQUAL_UINT8(0x13, payload[1]);
  TEST_ASSERT_EQUAL_UINT8(0x14, payload[2]);
  TEST_ASSERT_EQUAL_UINT8(0x15, payload[3]);

  // The TCL sends Unconfirmed frame
  // MAC-CMD RxTimingSetupReq
  // Payload = [0x]08XX
  // Delay = 15
  nextResponse = make_data_response(0, std::vector<uint8_t>{0x08, 15}, false,
                                    server_state);
  nextResponse.time =
      nextPacket.time + OsDeltaTime::from_sec(3) + OsDeltaTime::from_ms(55);
  dut::send_data(nextResponse);

  // Step 10
  // DUT sends Unconfirmed frame
  // Repeat up to 3 times until a downlink  is received confirming the receipt
  // of the RxTimingSetupAns MAC-CMD RxTimingSetupAns Payload = [0x]08
  for (uint8_t i = 0; i < 3; i++) {
    nextPacket = dut::wait_for_data(defaultWaitTime);
    TEST_ASSERT(check_is_next_packet(nextPacket, server_state));
    TEST_ASSERT(is_data(nextPacket));
    TEST_ASSERT_FALSE(is_confirmed_uplink(nextPacket));
    macResponse = get_mac_command_values(nextPacket, server_state);
    TEST_ASSERT_EQUAL_UINT(1, macResponse.size());
    TEST_ASSERT_EQUAL_UINT8(0x08, macResponse[0]);
  }

  // Step 11
  // DUT sends Unconfirmed frame
  // MAC-CMD
  // RxTimingSetupAns
  // Payload = [0x]08
  nextPacket = dut::wait_for_data(defaultWaitTime);
  TEST_ASSERT(check_is_next_packet(nextPacket, server_state));
  TEST_ASSERT(is_data(nextPacket));
  TEST_ASSERT_FALSE(is_confirmed_uplink(nextPacket));
  macResponse = get_mac_command_values(nextPacket, server_state);
  TEST_ASSERT_EQUAL_UINT(1, macResponse.size());
  TEST_ASSERT_EQUAL_UINT8(0x08, macResponse[0]);

  // The TCL sends Unconfirmed frame on RX1 window
  // CP-CMD EchoPayloadReq
  // FPort = 224
  // Payload = [0x]08010203
  // TXDelay = 15 sec
  nextResponse = make_data_response(
      224, std::vector<uint8_t>{0x08, 0x01, 0x02, 0x03}, false, server_state);
  nextResponse.time =
      nextPacket.time + OsDeltaTime::from_sec(15) + OsDeltaTime::from_ms(55);
  dut::send_data(nextResponse);

  // Step 12
  // DUT sends Unconfirmed frame
  // CP-CMD EchoPayloadAns
  // FPort = 224
  // Payload = [0x]08020304
  nextPacket = dut::wait_for_data(defaultWaitTime);
  TEST_ASSERT(check_is_next_packet(nextPacket, server_state));
  TEST_ASSERT(is_data(nextPacket));
  TEST_ASSERT_FALSE(is_confirmed_uplink(nextPacket));
  TEST_ASSERT_EQUAL_UINT8(224, get_port(nextPacket));
  payload = get_payload(nextPacket, server_state);
  TEST_ASSERT_EQUAL_UINT8(0x08, payload[0]);
  TEST_ASSERT_EQUAL_UINT8(0x02, payload[1]);
  TEST_ASSERT_EQUAL_UINT8(0x03, payload[2]);
  TEST_ASSERT_EQUAL_UINT8(0x04, payload[3]);

  // The TCL sends Unconfirmed frame on RX2 window
  // CP-CMD EchoPayloadReq
  // FPort = 224
  // Payload = [0x]08121314
  // TXDelay = 16 seconds
  nextResponse = make_data_response(
      224, std::vector<uint8_t>{0x08, 0x12, 0x13, 0x14}, false, server_state);
  nextResponse.time =
      nextPacket.time + OsDeltaTime::from_sec(16) + OsDeltaTime::from_ms(55);
  dut::send_data(nextResponse);

  // Step 13
  // DUT sends Unconfirmed frame
  // CP-CMD EchoPayloadAns
  // FPort = 224
  // Payload = [0x]08131415
  nextPacket = dut::wait_for_data(defaultWaitTime);
  TEST_ASSERT(check_is_next_packet(nextPacket, server_state));
  TEST_ASSERT(is_data(nextPacket));
  TEST_ASSERT_FALSE(is_confirmed_uplink(nextPacket));
  TEST_ASSERT_EQUAL_UINT8(224, get_port(nextPacket));
  payload = get_payload(nextPacket, server_state);
  TEST_ASSERT_EQUAL_UINT8(0x08, payload[0]);
  TEST_ASSERT_EQUAL_UINT8(0x13, payload[1]);
  TEST_ASSERT_EQUAL_UINT8(0x14, payload[2]);
  TEST_ASSERT_EQUAL_UINT8(0x15, payload[3]);

  // The TCL sends Unconfirmed frame
  // MAC-CMD
  // RxTimingSetupReq
  // Payload = [0x]08XX
  // Delay = 0
  nextResponse =
      make_data_response(0, std::vector<uint8_t>{0x08, 0}, false, server_state);
  nextResponse.time =
      nextPacket.time + OsDeltaTime::from_sec(16) + OsDeltaTime::from_ms(55);
  dut::send_data(nextResponse);

  // Step 14
  // DUT sends Unconfirmed frame
  // Repeat up to 3 times until a downlink is received confirming the receipt of
  // the RxTimingSetupAns MAC-CMD RxTimingSetupAns Payload = [0x]08
  for (uint8_t i = 0; i < 3; i++) {
    nextPacket = dut::wait_for_data(defaultWaitTime);
    TEST_ASSERT(check_is_next_packet(nextPacket, server_state));
    TEST_ASSERT(is_data(nextPacket));
    TEST_ASSERT_FALSE(is_confirmed_uplink(nextPacket));
    macResponse = get_mac_command_values(nextPacket, server_state);
    TEST_ASSERT_EQUAL_UINT(1, macResponse.size());
    TEST_ASSERT_EQUAL_UINT8(0x08, macResponse[0]);
  }

  // Step 15
  // DUT sends Unconfirmed frame
  // MAC-CMD
  // RxTimingSetupAns
  // Payload = [0x]08
  nextPacket = dut::wait_for_data(defaultWaitTime);
  TEST_ASSERT(check_is_next_packet(nextPacket, server_state));
  TEST_ASSERT(is_data(nextPacket));
  TEST_ASSERT_FALSE(is_confirmed_uplink(nextPacket));
  macResponse = get_mac_command_values(nextPacket, server_state);
  TEST_ASSERT_EQUAL_UINT(1, macResponse.size());
  TEST_ASSERT_EQUAL_UINT8(0x08, macResponse[0]);

  // The TCL sends Unconfirmed frame on RX1 window
  // CP-CMD EchoPayloadReq
  // FPort = 224
  // Payload = [0x]08010203
  // TXDelay = 1 sec
  nextResponse = make_data_response(
      224, std::vector<uint8_t>{0x08, 0x01, 0x02, 0x03}, false, server_state);
  nextResponse.time =
      nextPacket.time + OsDeltaTime::from_sec(1) + OsDeltaTime::from_ms(55);
  dut::send_data(nextResponse);

  // Step 16
  // DUT sends Unconfirmed frame → CP-CMD EchoPayloadAns
  // FPort = 224
  // Payload = [0x]08020304
  nextPacket = dut::wait_for_data(defaultWaitTime);
  TEST_ASSERT(check_is_next_packet(nextPacket, server_state));
  TEST_ASSERT(is_data(nextPacket));
  TEST_ASSERT_FALSE(is_confirmed_uplink(nextPacket));
  TEST_ASSERT_EQUAL_UINT8(224, get_port(nextPacket));
  payload = get_payload(nextPacket, server_state);
  TEST_ASSERT_EQUAL_UINT8(0x08, payload[0]);
  TEST_ASSERT_EQUAL_UINT8(0x02, payload[1]);
  TEST_ASSERT_EQUAL_UINT8(0x03, payload[2]);
  TEST_ASSERT_EQUAL_UINT8(0x04, payload[3]);

  // Step 17
  // The TCL sends Unconfirmed frame on RX2 window
  // CP-CMD EchoPayloadReq
  // FPort = 224
  // Payload = [0x]08121314
  // TXDelay = 2 seconds
  nextResponse = make_data_response(
      224, std::vector<uint8_t>{0x08, 0x12, 0x13, 0x14}, false, server_state);
  nextResponse.time =
      nextPacket.time + OsDeltaTime::from_sec(2) + OsDeltaTime::from_ms(55);
  dut::send_data(nextResponse);

  // Step 18
  // DUT sends Unconfirmed frame
  // CP-CMD EchoPayloadAns
  // FPort = 224
  // Payload = [0x]08131415
  nextPacket = dut::wait_for_data(defaultWaitTime);
  TEST_ASSERT(check_is_next_packet(nextPacket, server_state));
  TEST_ASSERT(is_data(nextPacket));
  TEST_ASSERT_FALSE(is_confirmed_uplink(nextPacket));
  TEST_ASSERT_EQUAL_UINT8(224, get_port(nextPacket));
  payload = get_payload(nextPacket, server_state);
  TEST_ASSERT_EQUAL_UINT8(0x08, payload[0]);
  TEST_ASSERT_EQUAL_UINT8(0x13, payload[1]);
  TEST_ASSERT_EQUAL_UINT8(0x14, payload[2]);
  TEST_ASSERT_EQUAL_UINT8(0x15, payload[3]);
}
