
#ifndef _powersave_h_
#define _powersave_h_

class OsDeltaTime;

using stopsleepcb_t = bool (*)();

void powersave(OsDeltaTime maxTime, stopsleepcb_t interrupt);

#endif