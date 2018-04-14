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

OsJob::OsJob(OsScheduler& scheduler) {
    this->scheduler = &scheduler;
}

// schedule immediately runnable job
void OsJob::setCallbackRunnable (osjobcb_t cb) {
    OsJob** pnext;
    hal_disableIRQs();
    // remove if job was already queued
    clearCallback();
    // fill-in job
    func = cb;
    next = nullptr;
    // add to end of run queue
    for(pnext=&this->scheduler->runnablejobs; *pnext; pnext=&((*pnext)->next));
    *pnext = this;
    hal_enableIRQs();
    #if LMIC_DEBUG_LEVEL > 1
        lmic_printf("%lu: Scheduled job %p, cb %p ASAP\n", os_getTime(), this, cb);
    #endif
}

bool OsJob::unlinkjob (OsJob** pnext, OsJob* job) {
    for( ; *pnext; pnext = &((*pnext)->next)) {
        if(*pnext == job) { // unlink
            *pnext = job->next;
            return true;
        }
    }
    return false;
}

// clear scheduled job
void OsJob::clearCallback () {
    hal_disableIRQs();
    bool res = unlinkjob(&this->scheduler->scheduledjobs, this) || unlinkjob(&this->scheduler->runnablejobs, this);
    hal_enableIRQs();
    #if LMIC_DEBUG_LEVEL > 1
        if (res)
            lmic_printf("%lu: Cleared job %p\n", os_getTime(), this);
    #endif
}

// schedule timed job
void OsJob::setTimedCallback (ostime_t time, osjobcb_t cb) {
    OsJob** pnext;
    hal_disableIRQs();
    // remove if job was already queued
    clearCallback();
    // fill-in job
    deadline = time;
    func = cb;
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
        lmic_printf("%lu: Scheduled job %p, cb %p at %lu\n", os_getTime(), this, cb, time);
    #endif
}

int32_t OsScheduler::runloopOnce() {
    #if LMIC_DEBUG_LEVEL > 1
        bool has_deadline = false;
    #endif
    OsJob* j = NULL;
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
            lmic_printf("%lu: Running job %p, cb %p, deadline %lu\n", os_getTime(), j, j->func, has_deadline ? j->deadline : 0);
        #endif
        j->func(j);
    } 
    if (runnablejobs) {
        return 0;
    } else {
        // return the number of milisecond to wait ()
        return osticks2ms(delta_time(scheduledjobs->deadline));
    } 
}

void os_init () {
    hal_init();
    radio_init();
    LMIC_init();
}

ostime_t os_getTime () {
    return hal_ticks();
}
