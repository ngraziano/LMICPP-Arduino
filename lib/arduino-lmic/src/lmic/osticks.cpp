
#include "osticks.h"
#include "lmicrand.h"


OsDeltaTime &OsDeltaTime::operator+=(const OsDeltaTime &a) {
  value += a.value;
  return *this;
}

OsDeltaTime &OsDeltaTime::operator-=(const OsDeltaTime &a) {
  value -= a.value;
  return *this;
}

OsTime &OsTime::operator+=(const OsDeltaTime &a) {
  value += a.tick();
  return *this;
}

OsTime &OsTime::operator-=(const OsDeltaTime &a) {
  value -= a.tick();
  return *this;
}


OsDeltaTime OsDeltaTime::rnd_delay(LmicRand& rand, uint8_t secSpan) {
  int16_t r = rand.uint16();
  int16_t delay = r;
  if (delay > OSTICKS_PER_SEC)
    delay = r % (uint16_t)OSTICKS_PER_SEC;
  if (secSpan > 0)
    delay += (r % secSpan) * OSTICKS_PER_SEC;
  return OsDeltaTime(delay);
}


// Some test

// diff
static_assert( OsTime(2) - OsTime(1) == OsDeltaTime(1), "Simple diff");
static_assert( OsTime(0x0000001) - OsTime(0xFFFFFFFF) == OsDeltaTime(2) , "diff with roll over");
static_assert( OsTime(0xFFFFFFFF) - OsTime(0x0000001) == OsDeltaTime(-2) , "diff with roll over");


// Comparaison
static_assert( OsTime(1) < OsTime(10) , "Comparaison small number");
static_assert( OsTime(0x7FFFFFFF) < OsTime(0x8FFFFFFF) , "Comparaison mid number");
static_assert( OsTime(0xFFFFFFFF) < OsTime(0x0000010) , "Comparaison roll over");


static_assert( OsTime(11) > OsTime(10) , "Comparaison small number");
static_assert( OsTime(0x8FFFFFFF) > OsTime(0x7FFFFFFF) , "Comparaison mid number");
static_assert( OsTime(0x0000010) > OsTime(0xFFFFFFFF) , "Comparaison roll over");
