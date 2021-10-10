#ifndef ARDUINO
#include <unity.h>

#include "test_aes.h"
#include "test_keyhandler.h"

int main() {
     UNITY_BEGIN();
     test_keyhandler::run();
     test_aes::run();
     UNITY_END();
}

#endif