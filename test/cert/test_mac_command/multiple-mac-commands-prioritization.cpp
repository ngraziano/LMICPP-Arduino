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
        0, std::vector<uint8_t>{0x20}, false, server_state);
}