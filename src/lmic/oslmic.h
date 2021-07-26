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
  OsJobType() : OsJobType(nullptr, OsTime()){};
  explicit OsJobType(osjobcbTyped_t cb) : OsJobType(cb, os_getTime()){};
  explicit OsJobType(osjobcbTyped_t cb, OsTime time) : funcTyped(cb), deadline(time){};

  OsDeltaTime run(T &refClass) {

    if (funcTyped && deadline <= hal_ticks()) {
      auto called = funcTyped;
      funcTyped = nullptr;
      (refClass.*called)();
    }
    // functyped and deadline may have been updated
    if (funcTyped) {
      // return before nex action
      return deadline - hal_ticks();
    } else {
      // nothing to do
      return OsInfiniteDeltaTime;
    }
  }
};

#endif // _oslmic_h_
