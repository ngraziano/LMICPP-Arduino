#ifndef _radio_h_
#define _radio_h_

#include "lorabase.h"
#include "osticks.h"
#include <stdint.h>

void radio_init(void);

void radio_rst();
void radio_tx(uint32_t freq, rps_t rps, int8_t txpow, uint8_t *frame,
              uint8_t dataLen, OsTime *txendPtr);
void radio_rx(uint32_t freq, rps_t rps, uint8_t rxsyms, OsTime *rxtime,
              uint8_t *frame, uint8_t *dataLen);
void radio_rxon(uint32_t freq, rps_t rps, uint8_t rxsyms, OsTime *rxtime,
                uint8_t *frame, uint8_t *dataLen);

void radio_irq_handler(uint8_t dio, OsTime const &trigger);
void radio_init_random(uint8_t randbuf[16]);

#endif