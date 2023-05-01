#include <unity.h>

#ifdef ARDUINO
#include <Arduino.h>
#endif

#include "test_aes.h"
#include "test_keyhandler.h"
#include "test_eu868channels.h"

void setUp(void) {
  // set stuff up here
}

void tearDown(void) {
  // clean stuff up here
}

int runUnityTests(void) {
  UNITY_BEGIN();
  test_keyhandler::run();
  test_aes::run();
  test_eu868channels::run();
  UNITY_END();
  return 0;
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
int main() { return runUnityTests(); }

#endif