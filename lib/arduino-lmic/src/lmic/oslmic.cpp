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

#include "lmic.h"
#include <stdbool.h>

OsScheduler OSS;

OsJobBase::OsJobBase(OsScheduler &scheduler) : scheduler(scheduler) {}

void OsJob::setCallbackRunnable(osjobcb_t cb) {
  setCallbackFuture(cb);
  setRunnable();
}

// schedule immediately runnable job
void OsJobBase::setRunnable() {
  hal_disableIRQs();
  // remove if job was already queued
  scheduler.unlinkScheduledJobs(this);
  scheduler.unlinkRunableJobs(this);
  // fill-in job
  next = nullptr;
  scheduler.linkRunableJob(this);
  hal_enableIRQs();

  PRINT_DEBUG_2("Scheduled job %p ASAP\n", this);
}

void OsScheduler::unlinkjob(OsJobBase **pnext, OsJobBase *job) {
  for (; *pnext; pnext = &((*pnext)->next)) {
    if (*pnext == job) { // unlink
      *pnext = job->next;
    }
  }
}

void OsScheduler::linkScheduledJob(OsJobBase *job) {
  const OsTime time = job->deadline;
  OsJobBase **pnext;
  // insert into schedule
  for (pnext = &scheduledjobs; *pnext; pnext = &((*pnext)->next)) {
    if ((*pnext)->deadline > time ) {
      // enqueue before next element and stop
      job->next = *pnext;
      break;
    }
  }
  *pnext = job;
}

void OsScheduler::linkRunableJob(OsJobBase *job) {
  // add to end of run queue
  OsJobBase **pnext;
  for (pnext = &runnablejobs; *pnext; pnext = &((*pnext)->next))
    ;
  *pnext = job;
}

void OsScheduler::unlinkScheduledJobs(OsJobBase *job) {
  unlinkjob(&scheduledjobs, job);
}

void OsScheduler::unlinkRunableJobs(OsJobBase *job) {
  unlinkjob(&runnablejobs, job);
}

// clear scheduled job
void OsJobBase::clearCallback() {
  hal_disableIRQs();
  scheduler.unlinkScheduledJobs(this);
  scheduler.unlinkRunableJobs(this);
  hal_enableIRQs();
}

void OsJob::setTimedCallback(OsTime time, osjobcb_t cb) {
  setCallbackFuture(cb);
  setTimed(time);
}

// schedule timed job
void OsJobBase::setTimed(OsTime time) {

  hal_disableIRQs();
  // remove if job was already queued
  scheduler.unlinkScheduledJobs(this);
  scheduler.unlinkRunableJobs(this);
  // fill-in job
  deadline = time;
  next = nullptr;
  scheduler.linkScheduledJob(this);
  hal_enableIRQs();
  PRINT_DEBUG_2("Scheduled job %p, atRun %lu\n", this, time);
}

void OsJob::call() const { func(); }

OsDeltaTime OsScheduler::runloopOnce() {
#if LMIC_DEBUG_LEVEL > 1
  bool has_deadline = false;
#endif
  OsJobBase *j = nullptr;
  hal_disableIRQs();
  // check for runnable jobs
  if (runnablejobs) {
    j = runnablejobs;
    runnablejobs = j->next;
  } else if (scheduledjobs &&
             hal_checkTimer(
                 scheduledjobs->deadline)) { // check for expired timed jobs
    j = scheduledjobs;
    scheduledjobs = j->next;
#if LMIC_DEBUG_LEVEL > 1
    has_deadline = true;
#endif
  }
  hal_enableIRQs();

  if (j) { // run job callback
    PRINT_DEBUG_2("Running job %p, deadline %lu\n", j,
                  has_deadline ? j->deadline : 0);
    j->call();
  }
  if (runnablejobs) {
    return OsDeltaTime(0);
  } else if (scheduledjobs) {
    // return the number of milisecond to wait ()
    return scheduledjobs->deadline - hal_ticks();
  }
  return OsDeltaTime(0);
}

void os_init() { hal_init(); }

OsTime os_getTime() { return hal_ticks(); }
