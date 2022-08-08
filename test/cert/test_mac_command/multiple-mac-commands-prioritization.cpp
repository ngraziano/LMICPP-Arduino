#include "common.h"

void test_multiple_mac_commands_prioritization() {
  // Step 1
  // DUT sends Unconfirmed frame
  auto nextPacket = dut::wait_for_data(defaultWaitTime);
  TEST_ASSERT(check_is_next_packet(nextPacket, server_state));
  TEST_ASSERT(is_data(nextPacket));
  TEST_ASSERT_FALSE(is_confirmed_uplink(nextPacket));

  // The TCL sends Unconfirmed frame
  // FPort = 224
  // CP-CMD LinkCheckReq
  // MAC-CMD DevStatusReq
  // MAC-CMD LinkADRReq
  // DataRate = Max125kHzDR
  // Payload = [0x]20
  // FOpts = [0x]06[0x]03XXXXXXXX
  auto nextResponse = make_data_response(
      224, std::vector<uint8_t>{0x20}, false, server_state, false,
      std::vector<uint8_t>{0x06, 0x03, 0x05 << 4, 0x07, 0x00, 0x00});
  nextResponse.time = nextPacket.time + RECEIVE_DELAY2;

  dut::send_data(nextResponse);

  // Step 2
  //  DUT sends Unconfirmed frame
  // MAC-CMD DevStatusAns
  // MAC-CMD LinkADRAns
  // MAC-CMD LinkCheckReq
  // Payload = [0x]06XXXX[0x]0307[0x]02
  nextPacket = dut::wait_for_data(defaultWaitTime);
  TEST_ASSERT(check_is_next_packet(nextPacket, server_state));
  TEST_ASSERT(is_data(nextPacket));
  TEST_ASSERT_FALSE(is_confirmed_uplink(nextPacket));
  auto macResponse = get_mac_command_values(nextPacket, server_state);
  TEST_ASSERT_EQUAL_UINT(6, macResponse.size());
  TEST_ASSERT_EQUAL_UINT8(0x06, macResponse[0]);
  TEST_ASSERT_EQUAL_UINT8(0x03, macResponse[3]);
  TEST_ASSERT_EQUAL_UINT8(0x07, macResponse[4]);
  TEST_ASSERT_EQUAL_UINT8(0x02, macResponse[5]);

  // The TCL sends Unconfirmed frame
  // MAC-CMD LinkCheckAns
  // Payload = [0x]02XXXX
  nextResponse = make_data_response(0, std::vector<uint8_t>{0x02, 0X05, 0x06},
                                    false, server_state);
  nextResponse.time = nextPacket.time + RECEIVE_DELAY2;
  dut::send_data(nextResponse);

  // Step 3
  // DUT sends Unconfirmed frame
  nextPacket = dut::wait_for_data(defaultWaitTime);
  TEST_ASSERT(check_is_next_packet(nextPacket, server_state));
  TEST_ASSERT(is_data(nextPacket));
  TEST_ASSERT_FALSE(is_confirmed_uplink(nextPacket));

  // The TCL sends Unconfirmed frame
  // Note: This step is required for only regions with Dwell Time limitation

  // Step 4
  // DUT sends Unconfirmed frame
  // For regions with Dwell time limitation only
  // MAC-CMD
  // TXParamSetupAns
  // Payload = [0x]09
  // DataRate = Max125kHzDR
  nextPacket = dut::wait_for_data(defaultWaitTime);
  TEST_ASSERT(check_is_next_packet(nextPacket, server_state));
  TEST_ASSERT(is_data(nextPacket));
  TEST_ASSERT_FALSE(is_confirmed_uplink(nextPacket));

  /*
  ********* LMITATION TO 64 BYTES PREVENT TO TEST THE FOLLOWING STEPS **********

  // TCL also sends Unconfirmed frame
  // MAC-CMD1 DevStatusReq
  // MAC-CMD2 RxParamSetupReq
  // Repeat the MAC-CMD DevStatusReq until the MAC command uplink response
  buffer would be full for MinDR, refer [2]
  // MAC-CMDX LinkADRReq  (with DataRate = MinDR)
  // For example: For EU863-870, the Max payload size is 51 bytes for MinDR.
  Hence repeat the DevStatusReq
  // command 15 times to ensure the MAC command response buffer is greater than
  51 bytes.
  // Payload = [0x]06[0x]05XXXXXXXX[0x]06[Repeat as required][0x]03XXXXXXXX

  // repeat default parameter : RX2  869.525
  std::vector<uint8_t> payload = {0x06, 0x05, 0x00, 0x84, 0xAD, 0xD2};
  for (int i = 0; i < 14; i++)
  {
      payload.push_back(0x06);
  }
  payload.push_back(0x03);
  payload.push_back(0x00);
  payload.push_back(0x07);
  payload.push_back(0x00);
  payload.push_back(0x00);
  nextResponse = make_data_response(0, payload, false, server_state);
  nextResponse.time = nextPacket.time + RECEIVE_DELAY2;
  dut::send_data(nextResponse);

  // Step 5
  // DUT sends Unconfirmed frame
  // DataRate = MinDR
  // MAC-CMD1 DevStatusAns
  // MAC-CMD2 RxParamSetupAns
  // MAC-CMD3 DevStatusAns
  // ...
  // MAC-CMDX DevStatusAns
  // Payload = [0x]06XXXX[0x]0507[0x]06XXXX…[0x]06XXXX
  // DUT truncates the MAC command when max payload size is exceeded.
  // The sequence of the response must be exactly the same as described.
  // The LinkADRAns is not sent in the response as it must be truncated due to
  payload size restrictions. However, the DR must be set to MinDR nextPacket =
  dut::wait_for_data(defaultWaitTime);
  TEST_ASSERT(check_is_next_packet(nextPacket, server_state));
  TEST_ASSERT(is_data(nextPacket));
  TEST_ASSERT_FALSE(is_confirmed_uplink(nextPacket));
  macResponse = get_mac_command_values(nextPacket, server_state);
  TEST_ASSERT_EQUAL_UINT(51, macResponse.size());
  TEST_ASSERT_EQUAL_UINT8(0x06, macResponse[0]);
  TEST_ASSERT_EQUAL_UINT8(0x05, macResponse[3]);
  TEST_ASSERT_EQUAL_UINT8(0x07, macResponse[4]);
  TEST_ASSERT_EQUAL_UINT8(0x06, macResponse[5]);
  for (int i = 0; i < 14; i++)
  {
      TEST_ASSERT_EQUAL_UINT8(0x06, macResponse[6 + i]);
  }

  */
}