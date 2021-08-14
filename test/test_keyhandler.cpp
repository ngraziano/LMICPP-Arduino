#include "test_keyhandler.h"
#include "keyhandler.h"
#include "unity.h"
#include <array>

namespace test_keyhandler {

void run() { RUN_TEST(test_eui_getter); }

constexpr char const appEui[] = "1112456789ABCDF0";
const std::array<uint8_t, 8> wanted_eui{0xF0, 0xCD, 0xAB, 0x89,
                                             0x67, 0x45, 0x12, 0x11};

void test_eui_getter() {
  EuiGetter<appEui> obj;

  std::array<uint8_t, 8> eui;
  obj.getEui(eui.begin());

  TEST_ASSERT_EQUAL_MEMORY(wanted_eui.begin(), eui.begin(),
                           wanted_eui.max_size());
}

} // namespace test_keyhandler
