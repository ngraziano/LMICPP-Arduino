#ifndef cert_test_mac_command_common_h
#define cert_test_mac_command_common_h


#include <unity.h>
#include <vector>

#include "../activation_pretest.h"
#include "../dut.h"
#include "../packet_util.h"

extern TestServerState server_state;

constexpr OsDeltaTime defaultWaitTime = OsDeltaTime::from_sec(60);


// List of test :
void test_dev_status_req();
void test_new_channel_req();
void test_RX_timing_setup_req();
void test_TX_param_setup_req();


// §2.5.13
void test_incorrect_mac_command();
// §2.5.14
void test_multiple_mac_commands_prioritization();


#endif // cert_test_mac_command_common_h