
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
