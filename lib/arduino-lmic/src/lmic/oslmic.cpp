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

OsJobBase::OsJobBase(OsScheduler& scheduler) {
    this->scheduler = &scheduler;
}

void OsJob::setCallbackRunnable (osjobcb_t cb) {
    setCallbackFuture(cb);
    setRunnable();
}

// schedule immediately runnable job
void OsJobBase::setRunnable () {
    hal_disableIRQs();
    // remove if job was already queued
    unlinkjob(&this->scheduler->scheduledjobs, this) || unlinkjob(&this->scheduler->runnablejobs, this);
    // fill-in job
    next = nullptr;
    // add to end of run queue
    OsJobBase** pnext;
    for(pnext=&this->scheduler->runnablejobs; *pnext; pnext=&((*pnext)->next));
    *pnext = this;
    hal_enableIRQs();
    #if LMIC_DEBUG_LEVEL > 1
        lmic_printf("%lu: Scheduled job %p ASAP\n", os_getTime(), this);
    #endif
}

bool OsJobBase::unlinkjob (OsJobBase** pnext, OsJobBase* job) {
    for( ; *pnext; pnext = &((*pnext)->next)) {
        if(*pnext == job) { // unlink
            *pnext = job->next;
            return true;
        }
    }
    return false;
}

// clear scheduled job
void OsJobBase::clearCallback () {
    hal_disableIRQs();
    bool res = unlinkjob(&this->scheduler->scheduledjobs, this) || unlinkjob(&this->scheduler->runnablejobs, this);
    hal_enableIRQs();
    #if LMIC_DEBUG_LEVEL > 1
        if (res)
            lmic_printf("%lu: Cleared job %p\n", os_getTime(), this);
    #endif
}

void OsJob::setTimedCallback (OsTime const& time, osjobcb_t cb) {
    setCallbackFuture(cb);
    setTimed(time);
}

// schedule timed job
void OsJobBase::setTimed (OsTime const& time) {
    OsJobBase** pnext;
    hal_disableIRQs();
    // remove if job was already queued
    clearCallback();
    // fill-in job
    deadline = time;
    next = nullptr;
    // insert into schedule
    for(pnext=&this->scheduler->scheduledjobs; *pnext; pnext=&((*pnext)->next)) {
        if((*pnext)->deadline - time > 0) { // (cmp diff, not abs!)
            // enqueue before next element and stop
            next = *pnext;
            break;
        }
    }
    *pnext = this;
    hal_enableIRQs();
    #if LMIC_DEBUG_LEVEL > 1
        lmic_printf("%lu: Scheduled job %p, atRun %lu\n", os_getTime(), this, time);
    #endif
}

void OsJob::call() {
    func(this);
}

OsDeltaTime OsScheduler::runloopOnce() {
    #if LMIC_DEBUG_LEVEL > 1
        bool has_deadline = false;
    #endif
    OsJobBase* j = nullptr;
    hal_disableIRQs();
    // check for runnable jobs
    if(runnablejobs) {
        j = runnablejobs;
        runnablejobs = j->next;
    } else if(scheduledjobs && hal_checkTimer(scheduledjobs->deadline)) { // check for expired timed jobs
        j = scheduledjobs;
        scheduledjobs = j->next;
        #if LMIC_DEBUG_LEVEL > 1
            has_deadline = true;
        #endif
    }
    hal_enableIRQs();
    // Instead of using proper interrupts (which are a bit tricky
    // and/or not available on all pins on AVR), just poll the pin
    // values. Here makes sure we check at least once every
    // loop.
    //
    // As an additional bonus, this prevents the can of worms that
    // we would otherwise get for running SPI transfers inside ISRs
    hal_io_check();
    if(j) { // run job callback
        #if LMIC_DEBUG_LEVEL > 1
            lmic_printf("%lu: Running job %p, deadline %lu\n", os_getTime(), j,  has_deadline ? j->deadline : 0);
        #endif
        j->call();
    } 
    if (runnablejobs) {
        return 0;
    } else {
        // return the number of milisecond to wait ()
        return scheduledjobs->deadline - hal_ticks();
    } 
}

void os_init () {
    hal_init();
    radio_init();
    LMIC.init();
}

OsTime os_getTime () {
    return hal_ticks();
}
