#ifndef _osticks_h_
#define _osticks_h_

#include "config.h"
#include <stdint.h>

#ifndef OSTICKS_PER_SEC
#define OSTICKS_PER_SEC 32768
#elif OSTICKS_PER_SEC < 10000 || OSTICKS_PER_SEC > 64516
#error Illegal OSTICKS_PER_SEC - must be in range [10000:64516]. One tick must be 15.5us .. 100us long.
#endif

typedef int32_t  ostime_t;

#define us2osticks(us)   ((ostime_t)( ((int64_t)(us) * OSTICKS_PER_SEC) / 1000000))
#define ms2osticks(ms)   ((ostime_t)( ((int64_t)(ms) * OSTICKS_PER_SEC)    / 1000))
#define sec2osticks(sec) ((ostime_t)( (int64_t)(sec) * OSTICKS_PER_SEC))
#define osticks2ms(os)   ((int32_t)(((os)*(int64_t)1000    ) / OSTICKS_PER_SEC))
#define osticks2us(os)   ((int32_t)(((os)*(int64_t)1000000 ) / OSTICKS_PER_SEC))
// Special versions
#define us2osticksCeil(us)  ((ostime_t)( ((int64_t)(us) * OSTICKS_PER_SEC + 999999) / 1000000))
#define us2osticksRound(us) ((ostime_t)( ((int64_t)(us) * OSTICKS_PER_SEC + 500000) / 1000000))
#define ms2osticksCeil(ms)  ((ostime_t)( ((int64_t)(ms) * OSTICKS_PER_SEC + 999) / 1000))
#define ms2osticksRound(ms) ((ostime_t)( ((int64_t)(ms) * OSTICKS_PER_SEC + 500) / 1000))


#endif // _osticks_h_