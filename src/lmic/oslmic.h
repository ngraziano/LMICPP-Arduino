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

#ifndef _oslmic_h_
#define _oslmic_h_

#include "../hal/hal.h"
#include "Arduino.h"
#include "config.h"
#include "osticks.h"
#include <stdint.h>
#include <stdio.h>

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

constexpr OsDeltaTime RX_RAMPUP = OsDeltaTime::from_us(40000);
constexpr OsDeltaTime TX_RAMPUP = OsDeltaTime::from_us(2000);

#ifndef HAS_os_calls

#ifndef os_getTime
OsTime os_getTime(void);
#endif

#endif // !HAS_os_calls


template <class T> class OsJobType final {
public:
  using osjobcbTyped_t = void (T::*)();

private:
  osjobcbTyped_t funcTyped;
  OsTime deadline;

public:
  // clear scheduled job
  void clearCallback() { funcTyped = nullptr; }

  void setCallbackFuture(osjobcbTyped_t cb) { funcTyped = cb; };
  void setCallbackRunnable(osjobcbTyped_t cb) {
    setCallbackFuture(cb);
    setRunnable();
  };
  void setRunnable() { setTimed(os_getTime()); }
  void setTimedCallback(OsTime time, osjobcbTyped_t cb) {
    setCallbackFuture(cb);
    setTimed(time);
  };
  // schedule timed job
  void setTimed(OsTime time) {
    // fill-in job
    deadline = time;
  }

  OsDeltaTime run(T &refClass) {

    if (funcTyped && deadline <= hal_ticks()) {
      auto called = funcTyped;
      funcTyped = nullptr;
      (refClass.*called)();
    }
    // functyped and deadline may have been updated
    if (funcTyped) {
      // return the time to wait
      auto timeToWait = deadline - hal_ticks();
      if (timeToWait < OsDeltaTime(0)) {
        return OsDeltaTime(0);
      } else {
        return timeToWait;
      }
    } else {

      // nothing to do
      return OsDeltaTime(-1);
    }
  }
};

#endif // _oslmic_h_
