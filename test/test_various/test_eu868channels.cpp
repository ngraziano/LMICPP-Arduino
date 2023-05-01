

#include "test_eu868channels.h"

#include "lmic/lmic.eu868.h"
#include <set>
#include <tuple>
#include <unity.h>

namespace test_eu868channels {

void run() { RUN_TEST(test_default_join_frequency); }

void test_default_join_frequency() {

  Aes aes;
  LmicRand rand{aes};
  Eu868RegionalChannelParams testObj(rand);

  testObj.initDefaultChannels();

  testObj.initJoinLoop();

  std::set<uint32_t> frequencies;
  std::set<sf_t> spreadingFactors;
  std::set<std::tuple<uint32_t, uint8_t>> frequenciesAndSpreadingFactors;

  for (int i = 0; i < 200; i++) {
    auto join = testObj.nextJoinState();

    // pour les premier 10 join, on ne doit pas encore
    // etre en echec
    if (i < 10) {
      TEST_ASSERT_EQUAL(true, join.status);
    }

    auto param = testObj.getTxParameter();

    // param.frequency is in the list (868.10 MHz, 868.30 MHz, 868.50 MHz)
    TEST_ASSERT_TRUE(param.frequency == 868100000 ||
                     param.frequency == 868300000 ||
                     param.frequency == 868500000);

    frequencies.insert(param.frequency);

    // datarate is in DR0, DR1, DR2, DR3, DR4, D5
    // so spreading factor is in SF12, SF11, SF10, SF9, SF8, SF7
    // and bandwidth is 125 kHz
    TEST_ASSERT_TRUE(param.rps.sf == SF12 || param.rps.sf == SF11 ||
                     param.rps.sf == SF10 || param.rps.sf == SF9 ||
                     param.rps.sf == SF8 || param.rps.sf == SF7);
    spreadingFactors.insert(param.rps.sf);

    sf_t sf = param.rps.sf;
    frequenciesAndSpreadingFactors.insert(
        std::make_tuple(param.frequency, sf));

    TEST_ASSERT_EQUAL(BandWidth::BW125, param.rps.getBw());
  }

  // all 3 default frequencies are used
  TEST_ASSERT_EQUAL(3, frequencies.size());
  // all 6 spreading factors are used
  TEST_ASSERT_EQUAL(6, spreadingFactors.size());

  // all 18 combinations of frequencies and spreading factors are used
  TEST_ASSERT_EQUAL(18, frequenciesAndSpreadingFactors.size());
}
} // namespace test_eu868channels