/*******************************************************************************
 * Copyright (c) 2014-2015 IBM Corporation.
 * All rights reserved. This program and the accompanying materials
 * are made available under the terms of the Eclipse Public License v1.0
 * which accompanies this distribution, and is available at
 * http://www.eclipse.org/legal/epl-v10.html
 *
 * Contributors:
 *    IBM Zurich Research Lab - initial API, implementation and documentation
 *******************************************************************************/

//! \file
#ifndef _oslmic_h_
#define _oslmic_h_

// Dependencies required for the LoRa MAC in C to run.
// These settings can be adapted to the underlying system.
// You should not, however, change the lmic.[hc]

#include "config.h"
#include "osticks.h"
#include <stdint.h>
#include <stdio.h>

#include "../hal/hal.h"
#include <string.h>

//================================================================================
//================================================================================

#if !defined(CFG_noassert)
#define ASSERT(cond)                                                           \
  if (!(cond))                                                                 \
  hal_failed(__FILE__, __LINE__)
#else
#define ASSERT(cond) /**/
#endif




void os_init(void);

//================================================================================

#ifndef RX_RAMPUP
#define RX_RAMPUP (OsDeltaTime::from_us(2000))
#endif
#ifndef TX_RAMPUP
#define TX_RAMPUP (OsDeltaTime::from_us(2000))
#endif

#ifndef HAS_os_calls

#ifndef os_getTime
OsTime os_getTime(void);
#endif
#ifndef os_getBattLevel
uint8_t os_getBattLevel(void);
#endif

//! Get random number (default impl for uint16_t).
#ifndef os_getRndU2
#define os_getRndU2() ((uint16_t)((radio_rand1() << 8) | radio_rand1()))
#endif

#endif // !HAS_os_calls

// ======================================================================
// Table support
// These macros for defining a table of constants and retrieving values
// from it makes it easier for other platforms (like AVR) to optimize
// table accesses.
// Use CONST_TABLE() whenever declaring or defining a table, and
// TABLE_GET_xx whenever accessing its values. The actual name of the
// declared variable will be modified to prevent accidental direct
// access. The accessor macros forward to an inline function to allow
// proper type checking of the array element type.

// Helper to add a prefix to the table name
#define RESOLVE_TABLE(table) constant_table_##table

// Accessors for table elements
#define TABLE_GET_U1(table, index) table_get_u1(RESOLVE_TABLE(table), index)
#define TABLE_GET_S1(table, index) table_get_s1(RESOLVE_TABLE(table), index)
#define TABLE_GET_U2(table, index) table_get_u2(RESOLVE_TABLE(table), index)
#define TABLE_GET_S2(table, index) table_get_s2(RESOLVE_TABLE(table), index)
#define TABLE_GET_U4(table, index) table_get_u4(RESOLVE_TABLE(table), index)
#define TABLE_GET_S4(table, index) table_get_s4(RESOLVE_TABLE(table), index)
#define TABLE_GET_OSTIME(table, index)                                         \
  table_get_ostime(RESOLVE_TABLE(table), index)
#define TABLE_GET_U1_TWODIM(table, index1, index2)                             \
  table_get_u1(RESOLVE_TABLE(table)[index1], index2)

#if defined(__AVR__)
#include <avr/pgmspace.h>
// Macro to define the getter functions. This loads data from
// progmem using pgm_read_xx, or accesses memory directly when the
// index is a constant so gcc can optimize it away;
#define TABLE_GETTER(postfix, type, pgm_type)                                  \
  inline type table_get##postfix(const type *table, size_t index) {            \
    if (__builtin_constant_p(table[index]))                                    \
      return table[index];                                                     \
    return pgm_read_##pgm_type(&table[index]);                                 \
  }

TABLE_GETTER(_u1, uint8_t, byte);
TABLE_GETTER(_s1, int8_t, byte);
TABLE_GETTER(_u2, uint16_t, word);
TABLE_GETTER(_s2, int16_t, word);
TABLE_GETTER(_u4, uint32_t, dword);
TABLE_GETTER(_s4, int32_t, dword);

// This assumes ostime is 4 bytes, so error out if it is not
// typedef int check_sizeof_ostime[(sizeof(ostime) == 4) ? 0 : -1];
TABLE_GETTER(_ostime, int32_t, dword);

// For AVR, store constants in PROGMEM, saving on RAM usage
#define CONST_TABLE(type, name) const type PROGMEM RESOLVE_TABLE(name)

#define lmic_printf(fmt, ...) printf_P(PSTR(fmt), ##__VA_ARGS__)

#else
inline uint8_t table_get_u1(const uint8_t *table, size_t index) {
  return table[index];
}
inline int8_t table_get_s1(const int8_t *table, size_t index) {
  return table[index];
}
inline uint16_t table_get_u2(const uint16_t *table, size_t index) {
  return table[index];
}
inline int16_t table_get_s2(const int16_t *table, size_t index) {
  return table[index];
}
inline uint32_t table_get_u4(const uint32_t *table, size_t index) {
  return table[index];
}
inline int32_t table_get_s4(const int32_t *table, size_t index) {
  return table[index];
}
inline ostime table_get_ostime(const ostime *table, size_t index) {
  return table[index];
}

// Declare a table
#define CONST_TABLE(type, name) const type RESOLVE_TABLE(name)
#define lmic_printf printf
#endif

#if LMIC_DEBUG_LEVEL > 0
#define PRINT_DEBUG_1(str, ...)                                                \
  lmic_printf("%lu: " str "\n", os_getTime(), ##__VA_ARGS__)
#else
#define PRINT_DEBUG_1(str, ...)
#endif

#if LMIC_DEBUG_LEVEL > 1
#define PRINT_DEBUG_2(str, ...)                                                \
  lmic_printf("%lu: " str "\n", os_getTime(), ##__VA_ARGS__)
#else
#define PRINT_DEBUG_2(str, ...)
#endif

class OsJobBase;
class OsJob;

using osjobcb_t = void (*)();

class OsScheduler {
  friend class OsJobBase;

private:
  OsJobBase *scheduledjobs = nullptr;
  OsJobBase *runnablejobs = nullptr;

public:
  OsDeltaTime runloopOnce();
};

extern OsScheduler OSS;

class OsJobBase {
  friend class OsScheduler;

private:
  OsScheduler *scheduler;
  OsJobBase *next = nullptr;
  OsTime deadline;

  static bool unlinkjob(OsJobBase **pnext, OsJobBase *job);

protected:
  virtual void call() = 0;

public:
  OsJobBase(OsScheduler &scheduler);
  OsJobBase() : OsJobBase(OSS){};

  void setRunnable();
  void clearCallback();

  void setTimed(OsTime const &time);
};

class OsJob : public OsJobBase {
protected:
  osjobcb_t func = nullptr;
  virtual void call();

public:
  void setCallbackFuture(osjobcb_t cb) { func = cb; };
  void setCallbackRunnable(osjobcb_t cb);
  void setTimedCallback(OsTime const &time, osjobcb_t cb);
};

template <class T> class OsJobType : public OsJobBase {
public:
  using osjobcbTyped_t = void (T::*)();

private:
  T *refClass;
  osjobcbTyped_t funcTyped;

protected:
  virtual void call() {
    PRINT_DEBUG_2("Run func %p on class %p", funcTyped, refClass);
    (refClass->*funcTyped)();
  };

public:
  OsJobType(T *ref) : OsJobBase() { refClass = ref; };
  OsJobType(T *ref, OsScheduler &scheduler) : OsJobBase(scheduler) {
    refClass = ref;
  };
  void setCallbackFuture(osjobcbTyped_t cb) {
    funcTyped = cb;
    PRINT_DEBUG_2("Job %p SetCallBack %p on class %p", this, funcTyped,
                  refClass);
  };
  void setCallbackRunnable(osjobcbTyped_t cb) {
    setCallbackFuture(cb);
    setRunnable();
  };
  void setTimedCallback(OsTime const &time, osjobcbTyped_t cb) {
    setCallbackFuture(cb);
    setTimed(time);
  };
};

#endif // _oslmic_h_
