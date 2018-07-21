#ifndef _radio_h_
#define _radio_h_

#include "osticks.h"
#include <stdint.h>

void radio_init(void);
void os_radio(uint8_t mode);
void radio_irq_handler(uint8_t dio, OsTime const &trigger);
void radio_init_random(uint8_t randbuf[16]);

#endif