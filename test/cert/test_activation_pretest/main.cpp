#include <unity.h>
#include <vector>

#include "../activation_pretest.h"
#include "../dut.h"
#include "../packet_util.h"

#ifdef ARDUINO
#include <Arduino.h>
#endif

TestServerState server_state;

void setUp(void) { dut::reset(); }

void initial_join() { sp1_intial_join(server_state); }

void tearDown(void) {
  // clean stuff up here
}

void runUnityTests(void) {
  UNITY_BEGIN();
  RUN_TEST(initial_join);
  UNITY_END();
}

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

#else

int main() { runUnityTests(); }

#endif