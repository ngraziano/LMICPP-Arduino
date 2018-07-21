#ifndef _radio_h_
#define _radio_h_

#include <stdint.h>
#include "osticks.h"

void radio_init(void);
void os_radio(uint8_t mode);
void radio_irq_handler(uint8_t dio, OsTime const &trigger);
uint8_t radio_rand1(void);


#endif