#include "common.h"

void test_multiple_mac_commands_prioritization()
{
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
    nextResponse = make_data_response(
        0, std::vector<uint8_t>{0x02, 0X05, 0x06}, false, server_state);
    nextResponse.time = nextPacket.time + RECEIVE_DELAY2;
    dut::send_data(nextResponse);

    
}