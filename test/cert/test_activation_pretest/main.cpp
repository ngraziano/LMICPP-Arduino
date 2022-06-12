#include <unity.h>

#include "../dut.h"
#include "../packet_util.h"

constexpr OsDeltaTime JOIN_ACCEPT_DELAY2 =
    OsDeltaTime::from_sec(6)
    // This delay is abnormal, but it is the only way to make the test pass.
    // the calculation must be checked.
    + OsDeltaTime::from_ms(55);

void setUp(void) { dut::reset(); }

void sp1_intial_join() {
  // Step 1: send join request
  auto const data = dut::wait_for_data(OsDeltaTime::from_sec(60));
  TEST_ASSERT(data.is_valid());
  TEST_ASSERT(is_join_request(data));
  auto devNonce = get_dev_nonce(data);


  auto joinResponse = make_join_response(data);
  joinResponse.time = data.time + JOIN_ACCEPT_DELAY2;
  dut::send_data(joinResponse);

  // Step 2: send data
  auto const firstPacket = dut::wait_for_data(OsDeltaTime::from_sec(60));
  TEST_ASSERT(firstPacket.is_valid());
  TEST_ASSERT(is_data(firstPacket));
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