#ifndef ARDUINO
#include <unity.h>

#include "test_aes.h"

int main() {
     UNITY_BEGIN();
     test_aes::run();
     UNITY_END();
}

#endif