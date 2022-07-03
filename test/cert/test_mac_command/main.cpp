#include "common.h"

void setUp(void)
{
  dut::reset();
  sp1_intial_join(server_state);
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
  RUN_TEST(test_TX_param_setup_req);
  RUN_TEST(test_incorrect_mac_command);
  RUN_TEST(test_multiple_mac_commands_prioritization);
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