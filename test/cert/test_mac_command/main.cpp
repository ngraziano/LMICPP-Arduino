#include <unity.h>
#include <vector>

#include "../activation_pretest.h"
#include "../dut.h"
#include "../packet_util.h"

TestServerState server_state;

constexpr OsDeltaTime defaultWaitTime = OsDeltaTime::from_sec(60);

void setUp(void)
{
  dut::reset();
  sp1_intial_join(server_state);
}

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

constexpr std::array<int, 3> DEFAULT_CHANNEL_FREQUENCY = {868100000, 868300000,
                                                          868500000};

constexpr std::array<int, 13> TEST_VALID_FREQ = {
    868200000, 868400000, 868600000, 868700000, 868900000, 869100000, 869300000,
    869500000, 869700000, 869900000, 863300000, 863500000};

void test_new_channel_req()
{
  // Step 1
  // DUT sends Unconfirmed frame
  auto nextPacket = dut::wait_for_data(defaultWaitTime);
  TEST_ASSERT(check_is_next_packet(nextPacket, server_state));
  TEST_ASSERT(is_data(nextPacket));
  TEST_ASSERT_FALSE(is_confirmed_uplink(nextPacket));

  // The TCL sends Unconfirmed frame  MAC-CMD NewChannelReq
  // ChIndex = 0
  // Freq = Any allowed frequency for the channel,
  // DRRange = Any valid range,
  // Payload =
  // [0x]07XXXXXXXXXX
  auto nextResponse = make_data_response(
      0,
      std::vector<uint8_t>{0x07, 0x00, (TEST_VALID_FREQ[0] / 100) & 0xFF,
                           (TEST_VALID_FREQ[0] / 100) >> 8 & 0xFF,
                           (TEST_VALID_FREQ[0] / 100) >> 16 & 0xFF, 0x50},
      false, server_state);
  nextResponse.time = nextPacket.time + RECEIVE_DELAY2;
  dut::send_data(nextResponse);

  for (uint8_t i = 1; i < DEFAULT_CHANNEL_FREQUENCY.size(); i++)
  {
    // Step 2
    // DUT sends Unconfirmed frame
    // MAC-CMD NewChannelAns
    // Payload NOT = [0x]0703
    nextPacket = dut::wait_for_data(defaultWaitTime);
    TEST_ASSERT(check_is_next_packet(nextPacket, server_state));
    TEST_ASSERT(is_data(nextPacket));
    TEST_ASSERT_FALSE(is_confirmed_uplink(nextPacket));
    auto newChannelAns = get_mac_command_values(nextPacket, server_state);
    TEST_ASSERT_EQUAL_UINT(2, newChannelAns.size());
    TEST_ASSERT_EQUAL_UINT8(0x07, newChannelAns[0]);
    TEST_ASSERT_NOT_EQUAL_UINT8(3, newChannelAns[1]);

    // The TCL sends Unconfirmed frame
    // MAC-CMD NewChannelReq
    // ChIndex = All other default
    // channels, refer [2]
    // Freq = Any allowed frequency for that channel,
    // DRRange = Any valid range,
    // Payload = [0x]07XXXXXXXXXX
    nextResponse = make_data_response(
        0,
        std::vector<uint8_t>{0x07, i, (TEST_VALID_FREQ[0] / 100) & 0xFF,
                             (TEST_VALID_FREQ[0] / 100) >> 8 & 0xFF,
                             (TEST_VALID_FREQ[0] / 100) >> 16 & 0xFF, 0x50},
        false, server_state);
    nextResponse.time = nextPacket.time + RECEIVE_DELAY2;
    dut::send_data(nextResponse);
  }

  // Step 3
  // DUT sends Unconfirmed frame
  // MAC-CMD NewChannelAns
  // Payload NOT = [0x]0703
  // DUT shall not change its channel plan or transmission behaviour
  nextPacket = dut::wait_for_data(defaultWaitTime);
  TEST_ASSERT(check_is_next_packet(nextPacket, server_state));
  TEST_ASSERT(is_data(nextPacket));
  TEST_ASSERT_FALSE(is_confirmed_uplink(nextPacket));
  auto newChannelAns = get_mac_command_values(nextPacket, server_state);
  TEST_ASSERT_EQUAL_UINT(2, newChannelAns.size());
  TEST_ASSERT_EQUAL_UINT8(0x07, newChannelAns[0]);
  TEST_ASSERT_NOT_EQUAL_UINT8(3, newChannelAns[1]);

  // The TCL sends Unconfirmed frame
  // FPort = 0
  // MAC-CMD NewChannelReq
  // ChIndex = 15
  // Freq = any applicable frequency
  // Payload = [0x]07XXXXXXXXXX
  nextResponse = make_data_response(
      0,
      std::vector<uint8_t>{0x07, 15, (TEST_VALID_FREQ[0] / 100) & 0xFF,
                           (TEST_VALID_FREQ[0] / 100) >> 8 & 0xFF,
                           (TEST_VALID_FREQ[0] / 100) >> 16 & 0xFF, 0x50},
      false, server_state);

  nextResponse.time = nextPacket.time + RECEIVE_DELAY2;
  dut::send_data(nextResponse);

  // Step 4
  // DUT sends Unconfirmed frame
  // MAC-CMD NewChannelAns
  // Payload = [0x]0703
  nextPacket = dut::wait_for_data(defaultWaitTime);
  TEST_ASSERT(check_is_next_packet(nextPacket, server_state));
  TEST_ASSERT(is_data(nextPacket));
  TEST_ASSERT_FALSE(is_confirmed_uplink(nextPacket));
  newChannelAns = get_mac_command_values(nextPacket, server_state);
  TEST_ASSERT_EQUAL_UINT(2, newChannelAns.size());
  TEST_ASSERT_EQUAL_UINT8(0x07, newChannelAns[0]);
  TEST_ASSERT_EQUAL_UINT8(3, newChannelAns[1]);

  // Step 5
  // Wait for a maximum of (5 * number of channels configured) uplink
  // packets, i.e. until the channel configured is used at least once
  // The new channel configured must be used at least once
  auto newChannelIsUsed = false;
  for (unsigned int i = 0; i < 5 * (DEFAULT_CHANNEL_FREQUENCY.size() + 1);
       i++)
  {
    nextPacket = dut::wait_for_data(defaultWaitTime);
    TEST_ASSERT(check_is_next_packet(nextPacket, server_state));
    TEST_ASSERT(is_data(nextPacket));
    if (nextPacket.freq == TEST_VALID_FREQ[0])
    {
      newChannelIsUsed = true;
      break;
    }
  }

  TEST_ASSERT_TRUE_MESSAGE(newChannelIsUsed, "New channel not used");

  // Step 6
  // DUT sends Unconfirmed frame
  nextPacket = dut::wait_for_data(defaultWaitTime);
  TEST_ASSERT(check_is_next_packet(nextPacket, server_state));
  TEST_ASSERT(is_data(nextPacket));
  TEST_ASSERT_FALSE(is_confirmed_uplink(nextPacket));

  // The TCL sends Unconfirmed frame
  // MAC-CMD NewChannelReq
  // ChIndex = 15
  // Freq = 0 MHz
  // Payload = [0x]07XXXXXXXXXX
  nextResponse = make_data_response(
      0, std::vector<uint8_t>{0x07, 15, 0, 0, 0, 0x50}, false, server_state);
  nextResponse.time = nextPacket.time + RECEIVE_DELAY2;
  dut::send_data(nextResponse);

  // Step 7
  // DUT sends Unconfirmed frame
  // MAC-CMD NewChannelAns
  // Payload = [0x]0703
  nextPacket = dut::wait_for_data(defaultWaitTime);
  TEST_ASSERT(check_is_next_packet(nextPacket, server_state));
  TEST_ASSERT(is_data(nextPacket));
  TEST_ASSERT_FALSE(is_confirmed_uplink(nextPacket));
  newChannelAns = get_mac_command_values(nextPacket, server_state);
  TEST_ASSERT_EQUAL_UINT(2, newChannelAns.size());
  TEST_ASSERT_EQUAL_UINT8(0x07, newChannelAns[0]);
  TEST_ASSERT_EQUAL_UINT8(0x03, newChannelAns[1]);

  // Step 8
  // Wait for 5 * number of channels configured uplink packets, to confirm
  // that the removed channel is not used.
  // The channel removed must not be used
  for (unsigned int i = 0; i < 5 * (DEFAULT_CHANNEL_FREQUENCY.size()); i++)
  {
    nextPacket = dut::wait_for_data(defaultWaitTime);
    TEST_ASSERT(check_is_next_packet(nextPacket, server_state));
    TEST_ASSERT(is_data(nextPacket));
    TEST_ASSERT_FALSE_MESSAGE(nextPacket.freq == TEST_VALID_FREQ[0],
                              "Channel removed used");
  }

  // Step 9
  // DUT sends Unconfirmed frame
  nextPacket = dut::wait_for_data(defaultWaitTime);
  TEST_ASSERT(check_is_next_packet(nextPacket, server_state));
  TEST_ASSERT(is_data(nextPacket));
  TEST_ASSERT_FALSE(is_confirmed_uplink(nextPacket));

  // The TCL sends Unconfirmed frame  FPort = 0
  // MAC-CMD NewChannelReq
  // ChIndex = all non-default channel indexes, refer [2]
  // Freq = any frequency applicable for that region, refer [2]. Each channel
  // must have a different frequency, as supported by the gateway. Payload =
  // [0x]07XXXXXXXXXX[0x]07XXXXXXXXXX… [repeat [0x]07XXXXXXXXXX up to (16 -
  // NbDefaultChannels)]
  // Note1: This downlink may be split into multiple downlinks so that the
  // maximum FRMPayload is not exceeded
  std::vector<uint8_t> newChannelReq;
  for (int i = DEFAULT_CHANNEL_FREQUENCY.size(); i < 9; i++)
  {
    newChannelReq.push_back(0x07);
    newChannelReq.push_back(i);
    newChannelReq.push_back(TEST_VALID_FREQ[i - 3] / 100 & 0xFF);
    newChannelReq.push_back(TEST_VALID_FREQ[i - 3] / 100 >> 8 & 0xFF);
    newChannelReq.push_back(TEST_VALID_FREQ[i - 3] / 100 >> 16 & 0xFF);
    newChannelReq.push_back(0x50);
  }
  nextResponse = make_data_response(
      0, newChannelReq, false, server_state);
  nextResponse.time = nextPacket.time + RECEIVE_DELAY2;
  dut::send_data(nextResponse);

  // Step 10
  // DUT sends Unconfirmed frame
  // FPort = 0
  // MAC-CMD NewChannelAns
  // Payload =  [0x]0703[0x]0703[0x]0703...[Repeat [0x]0703 "Y" times]
  // where Y is the number of channels configured
  // Note: This uplink may be split into multiple uplinks so that the maximum FRMPayload is not exceeded
  auto y = 9 - DEFAULT_CHANNEL_FREQUENCY.size();
  nextPacket = dut::wait_for_data(defaultWaitTime);
  TEST_ASSERT(check_is_next_packet(nextPacket, server_state));
  TEST_ASSERT(is_data(nextPacket));
  TEST_ASSERT_FALSE(is_confirmed_uplink(nextPacket));
  newChannelAns = get_mac_command_values(nextPacket, server_state);
  TEST_ASSERT_EQUAL_UINT(2 * y, newChannelAns.size());
  for (uint8_t i = 0; i < y; i++)
  {
    TEST_ASSERT_EQUAL_UINT8(0x07, newChannelAns[2 * i]);
    TEST_ASSERT_EQUAL_UINT8(0x03, newChannelAns[2 * i + 1]);
  }

  // TODO Continue with the test
}

void test_RX_timing_setup_req()
{
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
  for (uint8_t i = 0; i < 3; i++)
  {
    // DUT sends Unconfirmed frame
    // Repeat up to 3 times until a downlink is received confirming the receipt of the RxTimingSetupAns
    // FCntUp >= y + n
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
  nextResponse.time = nextPacket.time + OsDeltaTime::from_sec(newDelay) + OsDeltaTime::from_ms(55);
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
  nextResponse.time = nextPacket.time + OsDeltaTime::from_sec(newDelay + 1) + OsDeltaTime::from_ms(55);
  dut::send_data(nextResponse);

  // Step 5
  // DUT sends Unconfirmed frame
  // CP-CMD EchoPayloadAns
  // FPort = 224
  // Payload = [0x]08131415
}

void tearDown(void)
{
  // clean stuff up here
}

void runUnityTests(void)
{
  UNITY_BEGIN();
  RUN_TEST(test_dev_status_req);
  RUN_TEST(test_new_channel_req);
  RUN_TEST(test_RX_timing_setup_req);
  UNITY_END();
}

int main() { runUnityTests(); }

#ifdef ARDUINO
void setup()
{
  delay(2000);
  runUnityTests();
}

void loop()
{
#if defined(set_sleep_mode) && defined(sleep_mode)
  set_sleep_mode(SLEEP_MODE_PWR_DOWN);
  sleep_mode();
#endif
}
#endif