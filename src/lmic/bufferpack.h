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

#include <cstring>
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

class StoringAbtract {
public:
  template <class T> void write(T const val) { store(&val, sizeof(T)); }

protected:
  virtual void store(void const *val, size_t size);
};

class RetrieveAbtract {
public:
  template <class T> void read(T &val) { retrieve(&val, sizeof(T)); }

protected:
  virtual void retrieve(void *val, size_t size);
};

class StoringBuffer final : public StoringAbtract {
public:
  StoringBuffer(uint8_t *const buffer) : original(buffer), current(buffer){};
  size_t length() const;

protected:
  void store(void const *val, size_t size) override;

private:
  uint8_t *const original;
  uint8_t *current;
};

class RetrieveBuffer final : public RetrieveAbtract {
public:
  RetrieveBuffer(uint8_t const *const buffer) : current(buffer){};
  size_t length() const;

protected:
  void retrieve(void *val, size_t size) override;

private:
  uint8_t const *current;
};

/**
 * If v compares less than lo, returns lo;
 * otherwise if hi compares less than v, returns hi; otherwise returns v.
 */
template <typename T>
constexpr T const &clamp(T const &v, T const &lo, T const &hi) {
  return v < lo ? lo : hi < v ? hi : v;
}

#endif // __bufferpack_h__