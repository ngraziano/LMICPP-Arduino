#ifndef _osticks_h_
#define _osticks_h_

#include "config.h"
#include <stdint.h>

#ifndef OSTICKS_PER_SEC
#define OSTICKS_PER_SEC 32768
#elif OSTICKS_PER_SEC < 10000 || OSTICKS_PER_SEC > 64516
#error Illegal OSTICKS_PER_SEC - must be in range [10000:64516]. One tick must be 15.5us .. 100us long.
#endif

// FOR constant table
#define us2osticks(us) ((int32_t)(((int64_t)(us)*OSTICKS_PER_SEC) / 1000000))

constexpr int32_t us2osticksRound(int64_t us) {
  return ((int32_t)(((us)*OSTICKS_PER_SEC + 500000) / 1000000));
}
  


class LmicRand;

class OsDeltaTime {
public:
  constexpr explicit OsDeltaTime(int32_t init) : value(init){};
  constexpr OsDeltaTime() : value(0){};

  constexpr static OsDeltaTime from_us(int64_t us) {
    return OsDeltaTime(us * OSTICKS_PER_SEC / 1000000);
  };
  constexpr static OsDeltaTime from_ms(int64_t ms) {
    return OsDeltaTime(ms * OSTICKS_PER_SEC / 1000);
  };
  constexpr static OsDeltaTime from_sec(int64_t sec) {
    return OsDeltaTime(sec * OSTICKS_PER_SEC);
  };
  constexpr static OsDeltaTime from_us_round(int64_t us) {
    return OsDeltaTime(us2osticksRound(us));
  };
  static OsDeltaTime rnd_delay(LmicRand &rand, uint8_t sec_span);

  constexpr int32_t to_us() const {
    return value * (int64_t)1000000 / OSTICKS_PER_SEC;
  };
  constexpr int32_t to_ms() const {
    return value * (int64_t)1000 / OSTICKS_PER_SEC;
  };
  constexpr int32_t tick() const { return value; };

  OsDeltaTime &operator+=(const OsDeltaTime &a);
  OsDeltaTime &operator-=(const OsDeltaTime &a);

private:
  int32_t value;
};

class OsTime {
public:
  constexpr OsTime() : OsTime(0){};
  constexpr explicit OsTime(uint32_t init) : value(init){};
  constexpr uint32_t tick() const { return value; };

  OsTime &operator+=(const OsDeltaTime &a);
  OsTime &operator-=(const OsDeltaTime &a);

private:
  uint32_t value;
};

constexpr OsDeltaTime operator+(OsDeltaTime const &a, OsDeltaTime const &b) {
  return OsDeltaTime(a.tick() + b.tick());
};

constexpr OsDeltaTime operator*(int16_t const &a, OsDeltaTime const &b) {
  return OsDeltaTime(a * b.tick());
};

constexpr int32_t operator/(OsDeltaTime const &a, OsDeltaTime const &b) {
  return a.tick() / b.tick();
};

constexpr bool operator<(OsDeltaTime const &lhs, OsDeltaTime const &rhs) {
  return lhs.tick() < rhs.tick();
};
constexpr bool operator>(OsDeltaTime const &lhs, OsDeltaTime const &rhs) {
  return rhs < lhs;
};
constexpr bool operator<=(OsDeltaTime const &lhs, OsDeltaTime const &rhs) {
  return !(lhs > rhs);
};
constexpr bool operator>=(OsDeltaTime const &lhs, OsDeltaTime const &rhs) {
  return !(lhs < rhs);
};

constexpr OsTime operator+(OsTime const &a, OsDeltaTime const &b) {
  return OsTime(a.tick() + b.tick());
};
constexpr OsTime operator-(OsTime const &a, OsDeltaTime const &b) {
  return OsTime(a.tick() - b.tick());
};

constexpr OsDeltaTime operator-(OsTime const &a, OsTime const &b) {
  return OsDeltaTime(a.tick() - b.tick());
};

constexpr bool operator<(OsTime const &lhs, OsTime const &rhs) {
  return lhs - rhs < OsDeltaTime(0);
};

// Some test
static_assert( OsTime(1) < OsTime(10) , "Comparaison small number");
static_assert( OsTime(0x7FFFFFFF) < OsTime(0x8FFFFFFF) , "Comparaison mid number");
static_assert( OsTime(0xFFFFFFFF) < OsTime(0x0000010) , "Comparaison roll over");


constexpr bool operator>(OsTime const &lhs, OsTime const &rhs) {
  return rhs < lhs;
};

// Some test
static_assert( OsTime(11) > OsTime(10) , "Comparaison small number");
static_assert( OsTime(0x8FFFFFFF) > OsTime(0x7FFFFFFF) , "Comparaison mid number");
static_assert( OsTime(0x0000010) > OsTime(0xFFFFFFFF) , "Comparaison roll over");


constexpr bool operator<=(OsTime const &lhs, OsTime const &rhs) {
  return !(lhs > rhs);
};
constexpr bool operator>=(OsTime const &lhs, OsTime const &rhs) {
  return !(lhs < rhs);
};

#endif // _osticks_h_