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

#include "../hal/print_debug.h"
#include "lmic.h"
#include <stdbool.h>

OsJobBase::OsJobBase(OsScheduler &scheduler) : scheduler(scheduler) {}

OsJob::OsJob(OsScheduler &scheduler) : OsJobBase(scheduler) {}

void OsJob::setCallbackRunnable(osjobcb_t cb) {
  setCallbackFuture(cb);
  setRunnable();
}

// schedule immediately runnable job
void OsJobBase::setRunnable() { setTimed(os_getTime()); }

void OsScheduler::linkScheduledJob(OsJobBase &job) {
  const OsTime time = job.deadline;
  job.next = nullptr;
  OsJobBase **pnext;
  // insert into schedule
  for (pnext = &scheduledjobs; *pnext; pnext = &((*pnext)->next)) {
    if ((*pnext)->deadline > time) {
      // enqueue before next element and stop
      job.next = *pnext;
      break;
    }
  }
  *pnext = &job;
}

void OsScheduler::unlinkScheduledJobs(OsJobBase &job) {
  for (OsJobBase **pnext = &scheduledjobs; *pnext; pnext = &((*pnext)->next)) {
    if (*pnext == &job) { // unlink
      *pnext = job.next;
    }
  }
}

// clear scheduled job
void OsJobBase::clearCallback() { scheduler.unlinkScheduledJobs(*this); }

void OsJob::setTimedCallback(OsTime time, osjobcb_t cb) {
  setCallbackFuture(cb);
  setTimed(time);
}

// schedule timed job
void OsJobBase::setTimed(OsTime time) {
  // remove if job was already queued
  scheduler.unlinkScheduledJobs(*this);
  // fill-in job
  deadline = time;
  scheduler.linkScheduledJob(*this);
  PRINT_DEBUG(2, F("Scheduled job %p, atRun %" PRIu32 ""), this, time);
}

void OsJob::call() const { func(); }

OsDeltaTime OsScheduler::runloopOnce() {

  OsJobBase const * const nextJob = scheduledjobs;
  if (nextJob && nextJob->deadline <= hal_ticks()) {
    // timed jobs runnable
    scheduledjobs = nextJob->next;
    // run job callback
    PRINT_DEBUG(2, F("Running job %p, deadline %" PRIu32 ""), nextJob,
            nextJob->deadline.tick());
    nextJob->call();
  }

  if (scheduledjobs) {
    // return the time to wait
    return scheduledjobs->deadline - hal_ticks();
  }
  // nothing to do
  return OsDeltaTime(0);
}

void os_init() { hal_init(); }

OsTime os_getTime() { return hal_ticks(); }
