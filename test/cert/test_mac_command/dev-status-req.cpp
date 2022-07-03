#include "common.h"

void test_dev_status_req()
{
  // Step 1
  // DUT sends Unconfirmed frame
  auto nextPacket = dut::wait_for_data(defaultWaitTime);
  TEST_ASSERT(check_is_next_packet(nextPacket, server_state));
  TEST_ASSERT(is_data(nextPacket));
  TEST_ASSERT_FALSE(is_confirmed_uplink(nextPacket));

  // The TCL sends Unconfirmed frame
  // MAC-CMD DevStatusReq
  // Payload [0x]06
  auto nextResponse =
      make_data_response(0, std::vector<uint8_t>{0x06}, false, server_state);
  nextResponse.time = nextPacket.time + RECEIVE_DELAY2;
  dut::send_data(nextResponse);

  // Step 2
  // DUT sends Unconfirmed frame
  // MAC-CMD DevStatusAns
  // RadioStatus >= - 32 and <= 31
  // Payload [0x]06XXXX
  nextPacket = dut::wait_for_data(defaultWaitTime);
  TEST_ASSERT(check_is_next_packet(nextPacket, server_state));
  TEST_ASSERT(is_data(nextPacket));
  TEST_ASSERT_FALSE(is_confirmed_uplink(nextPacket));
  auto devStatusAns = get_mac_command_values(nextPacket, server_state);
  TEST_ASSERT_EQUAL_UINT(3, devStatusAns.size());
  TEST_ASSERT_EQUAL_UINT8(0x06, devStatusAns[0]);
}

