/*******************************************************************************
 * Copyright (c) 2014-2015 IBM Corporation.
 * All rights reserved. This program and the accompanying materials
 * are made available under the terms of the Eclipse Public License v1.0
 * which accompanies this distribution, and is available at
 * http://www.eclipse.org/legal/epl-v10.html
 *
 * Contributors:
 *    IBM Zurich Research Lab - initial API, implementation and documentation
 *    Nicolas Graziano - cpp style.
 *******************************************************************************/

#ifndef __bufferpack_h__
#define __bufferpack_h__

#include <stdint.h>

#include "enumflagsvalue.h"
#include "osticks.h"

//! Read 24-bit quantity from given pointer in little endian byte order (but in
//! uint32_t).
uint32_t rlsbf3(const uint8_t *buf);
//! Read 32-bit quantity from given pointer in little endian byte order.
uint32_t rlsbf4(const uint8_t *buf);
//! Write 32-bit quantity into buffer in little endian byte order.
void wlsbf4(uint8_t *buf, uint32_t value);
//! Read 32-bit quantity from given pointer in big endian byte order.
uint32_t rmsbf4(const uint8_t *buf);
//! Write 32-bit quantity into buffer in big endian byte order.
void wmsbf4(uint8_t *buf, uint32_t value);
//! Read 16-bit quantity from given pointer in little endian byte order.
uint16_t rlsbf2(const uint8_t *buf);
//! Read 16-bit quantity from given pointer in big endian byte order.
uint16_t rmsbf2(const uint8_t *buf);
//! Write 16-bit quantity into buffer in little endian byte order.
void wlsbf2(uint8_t *buf, uint16_t value);

inline void write_to_buffer(uint8_t *&buf, uint8_t val) { *(buf++) = val; }

inline void write_to_buffer(uint8_t *&buf, int8_t val) { *(buf++) = val; }

inline void write_to_buffer(uint8_t *&buf, bool val) { *(buf++) = val; }

inline void write_to_buffer(uint8_t *&buf, uint16_t val) {
  *(reinterpret_cast<uint16_t *>(buf)) = val;
  buf += 2;
}

inline void write_to_buffer(uint8_t *&buf, uint32_t val) {
  *(reinterpret_cast<uint32_t *>(buf)) = val;
  buf += 4;
}

inline void write_to_buffer(uint8_t *&buf, int32_t val) {
  *(reinterpret_cast<int32_t *>(buf)) = val;
  buf += 4;
}

inline void write_to_buffer(uint8_t *&buf, OsTime const &val) {
  write_to_buffer(buf, val.tick());
}

inline void write_to_buffer(uint8_t *&buf, OsDeltaTime const &val) {
  write_to_buffer(buf, val.tick());
}

template <typename T>
inline void write_to_buffer(uint8_t *&buf, EnumFlagsValue<T> const &val) {
  write_to_buffer(buf, val.value);
}

inline void read_from_buffer(uint8_t const *&buf, uint8_t &val) {
  val = *(buf++);
}

inline void read_from_buffer(uint8_t const *&buf, int8_t &val) {
  val = *(buf++);
}

inline void read_from_buffer(uint8_t const *&buf, bool &val) { val = *(buf++); }

inline void read_from_buffer(uint8_t const *&buf, uint16_t &val) {
  val = *(reinterpret_cast<uint16_t const *>(buf));
  buf += 2;
}

inline void read_from_buffer(uint8_t const *&buf, uint32_t &val) {
  val = *(reinterpret_cast<uint32_t const *>(buf));
  buf += 4;
}

inline void read_from_buffer(uint8_t const *&buf, int32_t &val) {
  val = *(reinterpret_cast<int32_t const *>(buf));
  buf += 4;
}

inline void read_from_buffer(uint8_t const *&buf, OsTime &val) {
  int32_t tick;
  read_from_buffer(buf, tick);
  val = OsTime(tick);
}

inline void read_from_buffer(uint8_t const *&buf, OsDeltaTime &val) {
  uint32_t tick;
  read_from_buffer(buf, tick);
  val = OsDeltaTime(tick);
}

template <typename T>
inline void read_from_buffer(uint8_t const *&buf, EnumFlagsValue<T> &val) {
  read_from_buffer(buf, val.value);
}

#endif // __bufferpack_h__