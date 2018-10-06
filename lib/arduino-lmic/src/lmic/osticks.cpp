
#include "osticks.h"
#include "../hal/hal.h"
#include "lmicrand.h"

int32_t OsDeltaTime::tick() const { return value; }

OsDeltaTime &OsDeltaTime::operator+=(const OsDeltaTime &a) {
  value += a.value;
  return *this;
}

OsDeltaTime &OsDeltaTime::operator-=(const OsDeltaTime &a) {
  value -= a.value;
  return *this;
}
int32_t operator/(OsDeltaTime const &a, OsDeltaTime const &b) {
  return a.tick() / b.tick();
};

bool operator<(OsDeltaTime const &lhs, OsDeltaTime const &rhs) {
  return lhs.tick() < rhs.tick();
}
bool operator>(OsDeltaTime const &lhs, OsDeltaTime const &rhs) {
  return rhs < lhs;
}
bool operator<=(OsDeltaTime const &lhs, OsDeltaTime const &rhs) {
  return !(lhs > rhs);
}
bool operator>=(OsDeltaTime const &lhs, OsDeltaTime const &rhs) {
  return !(lhs < rhs);
}

uint32_t OsTime::tick() const { return value; }

OsDeltaTime operator+(OsDeltaTime const &a, OsDeltaTime const &b) {
  OsDeltaTime dup(a);
  dup += b;
  return dup;
}

OsTime operator+(OsTime const &a, OsDeltaTime const &b) {
  OsTime dup(a);
  dup += b;
  return dup;
}

OsTime operator-(OsTime const &a, OsDeltaTime const &b) {
  OsTime dup(a);
  dup -= b;
  return dup;
}

OsTime &OsTime::operator+=(const OsDeltaTime &a) {
  value += a.tick();
  return *this;
}

OsTime &OsTime::operator-=(const OsDeltaTime &a) {
  value -= a.tick();
  return *this;
}

OsDeltaTime operator*(int16_t const &a, OsDeltaTime const &b) {
  return OsDeltaTime(a * b.tick());
}

OsDeltaTime operator-(OsTime const &a, OsTime const &b) {
  return OsDeltaTime(a.tick() - b.tick());
}

OsDeltaTime OsDeltaTime::from_us(int64_t us) {
  return OsDeltaTime(us * OSTICKS_PER_SEC / 1000000);
}

OsDeltaTime OsDeltaTime::from_ms(int64_t ms) {
  return OsDeltaTime(ms * OSTICKS_PER_SEC / 1000);
}

OsDeltaTime OsDeltaTime::from_sec(int64_t sec) {
  return OsDeltaTime(sec * OSTICKS_PER_SEC);
}

OsDeltaTime OsDeltaTime::from_us_round(int64_t us) {
  return OsDeltaTime(us2osticksRound(us));
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

int32_t OsDeltaTime::to_ms() const {
  return value * (int64_t)1000 / OSTICKS_PER_SEC;
}

int32_t OsDeltaTime::to_us() const {
  return value * (int64_t)1000000 / OSTICKS_PER_SEC;
}


bool operator<(OsTime const &lhs, OsTime const &rhs) {
  return lhs - rhs < OsDeltaTime(0);
}
bool operator>(OsTime const &lhs, OsTime const &rhs) {
  return rhs < lhs;
}
bool operator<=(OsTime const &lhs, OsTime const &rhs) {
  return !(lhs > rhs);
}
bool operator>=(OsTime const &lhs, OsTime const &rhs) {
  return !(lhs < rhs);
}