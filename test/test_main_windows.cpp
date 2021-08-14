#ifndef ARDUINO
#include <unity.h>

#include "test_aes.h"
#include "test_keyhandler.h"

int main() {
     UNITY_BEGIN();
     test_aes::run();
     test_keyhandler::run();
     UNITY_END();
}

#endif