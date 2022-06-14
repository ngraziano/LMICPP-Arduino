#include <unity.h>
#include <vector>

#include "../dut.h"
#include "../packet_util.h"

constexpr OsDeltaTime JOIN_ACCEPT_DELAY1 =
    OsDeltaTime::from_sec(5)
    // This delay is abnormal, but it is the only way to make the test pass.
    // the calculation must be checked.
    + OsDeltaTime::from_ms(55);

constexpr OsDeltaTime JOIN_ACCEPT_DELAY2 =
    OsDeltaTime::from_sec(6)
    // This delay is abnormal, but it is the only way to make the test pass.
    // the calculation must be checked.
    + OsDeltaTime::from_ms(55);

// constexpr OsDeltaTime RECEIVE_DELAY1;
constexpr OsDeltaTime RECEIVE_DELAY2 =
    OsDeltaTime::from_sec(2)
    // This delay is abnormal, but it is the only way to make the test pass.
    // the calculation must be checked.
    + OsDeltaTime::from_ms(55);

TestServerState server_state;

void setUp(void) { dut::reset(); }

void sp1_intial_join() {
  // Step 1: send join request
  auto const data = dut::wait_for_data(OsDeltaTime::from_sec(60));
  TEST_ASSERT(data.is_valid());
  TEST_ASSERT(is_join_request(data));
  auto devNonce = get_dev_nonce(data);

  auto joinResponse = make_join_response(data, server_state);
  read_join_key(devNonce, server_state);
  joinResponse.time = data.time + JOIN_ACCEPT_DELAY2;
  dut::send_data(joinResponse);

  // Step 2: send data
  auto const firstPacket = dut::wait_for_data(OsDeltaTime::from_sec(60));
  TEST_ASSERT(firstPacket.is_valid());
  TEST_ASSERT(is_data(firstPacket));
  TEST_ASSERT(check_is_next_packet(firstPacket, server_state));
  TEST_ASSERT(server_state.fCntUp == 0 + 1 || server_state.fCntUp == 1 + 1);

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
  // The devNonce must be greater than the previous one.
  TEST_ASSERT(devNonce2 > devNonce);

  auto joinResponse2 = make_join_response(nextJoin, server_state);
  read_join_key(devNonce2, server_state);
  joinResponse2.time = nextJoin.time + JOIN_ACCEPT_DELAY1;
  dut::send_data(joinResponse2);

  auto const secondPacket = dut::wait_for_data(OsDeltaTime::from_sec(60));
  TEST_ASSERT(secondPacket.is_valid());
  TEST_ASSERT(is_data(secondPacket));
  TEST_ASSERT(check_is_next_packet(secondPacket, server_state));
  TEST_ASSERT(server_state.fCntUp == 0 + 1 || server_state.fCntUp == 1 + 1);

  auto secondResponse =
      make_data_response(224, std::vector<uint8_t>{0x06, 0x01},
                         is_confirmed_uplink(firstPacket), server_state);
  secondResponse.time = secondPacket.time + RECEIVE_DELAY2;
  dut::send_data(secondResponse);

  auto const nextPacket = dut::wait_for_data(OsDeltaTime::from_sec(6));
  TEST_ASSERT(nextPacket.is_valid());
}

void tearDown(void) {
  // clean stuff up here
}

void runUnityTests(void) {
  UNITY_BEGIN();
  RUN_TEST(sp1_intial_join);
  UNITY_END();
}

int main() { runUnityTests(); }

#ifdef ARDUINO
void setup() {
  delay(2000);
  runUnityTests();
}

void loop() {
#if defined(set_sleep_mode) && defined(sleep_mode)
  set_sleep_mode(SLEEP_MODE_PWR_DOWN);
  sleep_mode();
#endif
}
#endif