#include "common.h"

void test_TX_param_setup_req() {
  // test_TX_param_setup_req is not implement in EU868
  // so use 2.5.6.a

  // Step 1
  // DUT sends Unconfirmed frame
  auto nextPacket = dut::wait_for_data(defaultWaitTime);
  TEST_ASSERT(check_is_next_packet(nextPacket, server_state));
  TEST_ASSERT(is_data(nextPacket));
  TEST_ASSERT_FALSE(is_confirmed_uplink(nextPacket));

  // The TCL sends Unconfirmed frame
  // MAC-CMD
  // TXParamSetupReq
  // Payload = [0x]09XX
  // UplinkDwellTime = 0
  auto nextResponse =
      make_data_response(0, std::vector<uint8_t>{0x09, 0}, false, server_state);
  nextResponse.time =
      nextPacket.time + OsDeltaTime::from_sec(16);
  dut::send_data(nextResponse);

  // Step 2
  // DUT sends Unconfirmed frame
  nextPacket = dut::wait_for_data(defaultWaitTime);
  TEST_ASSERT(check_is_next_packet(nextPacket, server_state));
  TEST_ASSERT(is_data(nextPacket));
  TEST_ASSERT_FALSE(is_confirmed_uplink(nextPacket));
  auto macResponse = get_mac_command_values(nextPacket, server_state);
  TEST_ASSERT_EQUAL_UINT(0, macResponse.size());
}
