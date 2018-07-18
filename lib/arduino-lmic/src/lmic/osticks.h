#ifndef _osticks_h_
#define _osticks_h_

#include "config.h"
#include <stdint.h>

#ifndef OSTICKS_PER_SEC
#define OSTICKS_PER_SEC 32768
#elif OSTICKS_PER_SEC < 10000 || OSTICKS_PER_SEC > 64516
#error Illegal OSTICKS_PER_SEC - must be in range [10000:64516]. One tick must be 15.5us .. 100us long.
#endif

class OsDeltaTime {
    public:
        OsDeltaTime(int32_t init) : value(init) {};
        OsDeltaTime() :value(0) {};
        
        static OsDeltaTime from_us(int64_t us);
        static OsDeltaTime from_ms(int64_t us);
        static OsDeltaTime from_sec(int64_t us);
        static OsDeltaTime from_us_round(int64_t us);

        int32_t to_us() const;
        int32_t to_ms() const;
        int32_t tick() const;
        OsDeltaTime& operator+=(const OsDeltaTime& a);
        OsDeltaTime& operator-=(const OsDeltaTime& a);
    private:
        int32_t value;
};

class OsTime {
    public:
        OsTime() : OsTime(0) {};
        OsTime(uint32_t init) : value(init) {};
        uint32_t tick() const;

        OsTime& operator+=(const OsDeltaTime& a);
        OsTime& operator-=(const OsDeltaTime& a);
    private:
        uint32_t value;
};

OsDeltaTime operator+(OsDeltaTime const& a, OsDeltaTime const& b);

OsDeltaTime operator*(int16_t const& a, OsDeltaTime const& b);
int32_t operator/(OsDeltaTime const& a, OsDeltaTime const& b);

bool operator< (OsDeltaTime const& lhs, OsDeltaTime const& rhs);
bool operator> (OsDeltaTime const& lhs, OsDeltaTime const& rhs);
bool operator<=(OsDeltaTime const& lhs, OsDeltaTime const& rhs);
bool operator>=(OsDeltaTime const& lhs, OsDeltaTime const& rhs);

OsTime operator+(OsTime const& a, OsDeltaTime const& b);
OsTime operator-(OsTime const& a, OsDeltaTime const& b);

OsDeltaTime operator-(OsTime const& a,OsTime  const& b);



#endif // _osticks_h_