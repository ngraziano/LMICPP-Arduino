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
#include "lmic_table.h"
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

constexpr OsDeltaTime RX_RAMPUP = OsDeltaTime::from_us(2000);
constexpr OsDeltaTime TX_RAMPUP = OsDeltaTime::from_us(2000);

#ifndef HAS_os_calls

#ifndef os_getTime
OsTime os_getTime(void);
#endif
#ifndef os_getBattLevel
uint8_t os_getBattLevel(void);
#endif

#endif // !HAS_os_calls

#if defined(__AVR__)
#define lmic_printf(fmt, ...) printf_P(PSTR(fmt), ##__VA_ARGS__)
#else
#define lmic_printf printf
#endif


#if LMIC_DEBUG_LEVEL > 0
#define PRINT_DEBUG_1(str, ...)                                                \
  lmic_printf("%lu: " str "\n", os_getTime().tick(), ##__VA_ARGS__)
#else
#define PRINT_DEBUG_1(str, ...)
#endif

#if LMIC_DEBUG_LEVEL > 1
#define PRINT_DEBUG_2(str, ...)                                                \
  lmic_printf("%lu: " str "\n", os_getTime().tick(), ##__VA_ARGS__)
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
  static void unlinkjob(OsJobBase **pnext, OsJobBase *job);
  void unlinkScheduledJobs(OsJobBase *job);
  void unlinkRunableJobs(OsJobBase *job);
  void linkScheduledJob(OsJobBase *job);
  void linkRunableJob(OsJobBase *job);

public:
  OsDeltaTime runloopOnce();
};

extern OsScheduler OSS;

class OsJobBase {
  friend class OsScheduler;

private:
  OsScheduler &scheduler;
  OsJobBase *next = nullptr;
  OsTime deadline;

protected:
  virtual void call() const = 0;

public:
  OsJobBase(OsScheduler &scheduler);
  OsJobBase() : OsJobBase(OSS){};

  void setRunnable();
  void clearCallback();

  void setTimed(OsTime time);
};

class OsJob final : public OsJobBase {
protected:
  osjobcb_t func = nullptr;
  void call() const override;

public:
  void setCallbackFuture(osjobcb_t cb) { func = cb; };
  void setCallbackRunnable(osjobcb_t cb);
  void setTimedCallback(OsTime time, osjobcb_t cb);
};

template <class T> class OsJobType final : public OsJobBase {
public:
  using osjobcbTyped_t = void (T::*)();

private:
  T &refClass;
  osjobcbTyped_t funcTyped;

protected:
  void call() const override { (refClass.*funcTyped)(); };

public:
  OsJobType(T &ref) : OsJobBase(), refClass(ref){};
  OsJobType(T &ref, OsScheduler &scheduler)
      : OsJobBase(scheduler), refClass(ref){};
  void setCallbackFuture(osjobcbTyped_t cb) { funcTyped = cb; };
  void setCallbackRunnable(osjobcbTyped_t cb) {
    setCallbackFuture(cb);
    setRunnable();
  };
  void setTimedCallback(OsTime time, osjobcbTyped_t cb) {
    setCallbackFuture(cb);
    setTimed(time);
  };
};

#endif // _oslmic_h_
