#ifdef ARDUINO
#include <Arduino.h>
#include <unity.h>

#include "test_aes.h"

void setup() {
     UNITY_BEGIN();
     test_aes::run();
     UNITY_END();
}

void loop() {
    #if defined(set_sleep_mode) && defined(sleep_mode)
        set_sleep_mode(SLEEP_MODE_PWR_DOWN);
        sleep_mode();
    #endif
}
#endif