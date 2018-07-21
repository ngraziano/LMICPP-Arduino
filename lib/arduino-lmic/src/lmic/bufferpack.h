#ifndef __bufferpack_h__
#define __bufferpack_h__

#include <stdint.h>

//! Read 32-bit quantity from given pointer in little endian byte order.
uint32_t rlsbf4(const uint8_t *buf);
//! Write 32-bit quntity into buffer in little endian byte order.
void wlsbf4(uint8_t *buf, uint32_t value);
//! Read 32-bit quantity from given pointer in big endian byte order.
uint32_t rmsbf4(const uint8_t *buf);
//! Write 32-bit quntity into buffer in big endian byte order.
void wmsbf4(uint8_t *buf, uint32_t value);
//! Read 16-bit quantity from given pointer in little endian byte order.
uint16_t rlsbf2(const uint8_t *buf);
//! Write 16-bit quntity into buffer in little endian byte order.
void wlsbf2(uint8_t *buf, uint16_t value);


#endif // __bufferpack_h__