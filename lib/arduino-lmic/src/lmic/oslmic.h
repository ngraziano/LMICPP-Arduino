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
#include <stdint.h>
#include <stdio.h>

#include <string.h>
#include "hal.h"


//================================================================================
//================================================================================
// Target platform as C library
typedef unsigned int       uint;
typedef const char* str_t;

#if !defined(CFG_noassert)
#define ASSERT(cond) if(!(cond)) hal_failed(__FILE__, __LINE__)
#else
#define ASSERT(cond) /**/
#endif

#define os_clearMem(a,b)   memset(a,0,b)
#define os_copyMem(a,b,c)  memcpy(a,b,c)

typedef     struct osjob_t osjob_t;
typedef      struct band_t band_t;
typedef   struct chnldef_t chnldef_t;
typedef   struct rxsched_t rxsched_t;
typedef   struct bcninfo_t bcninfo_t;
typedef        const uint8_t* xref2cuint8_t;
typedef              uint8_t* xref2uint8_t;
#define TYPEDEF_xref2rps_t     typedef         rps_t* xref2rps_t
#define TYPEDEF_xref2rxsched_t typedef     rxsched_t* xref2rxsched_t
#define TYPEDEF_xref2chnldef_t typedef     chnldef_t* xref2chnldef_t
#define TYPEDEF_xref2band_t    typedef        band_t* xref2band_t


#define SIZEOFEXPR(x) sizeof(x)

#define ON_LMIC_EVENT(ev)  onEvent(ev)
#define DECL_ON_LMIC_EVENT void onEvent(ev_t e)

extern uint32_t AESAUX[];
extern uint32_t AESKEY[];
#define AESkey ((uint8_t*)AESKEY)
#define AESaux ((uint8_t*)AESAUX)

uint8_t radio_rand1 (void);
#define os_getRndU1() radio_rand1()


void radio_init (void);
void radio_irq_handler (uint8_t dio, int32_t trigger);
void os_init (void);

//================================================================================


#ifndef RX_RAMPUP
#define RX_RAMPUP  (us2osticks(2000))
#endif
#ifndef TX_RAMPUP
#define TX_RAMPUP  (us2osticks(2000))
#endif

#ifndef OSTICKS_PER_SEC
#define OSTICKS_PER_SEC 32768
#elif OSTICKS_PER_SEC < 10000 || OSTICKS_PER_SEC > 64516
#error Illegal OSTICKS_PER_SEC - must be in range [10000:64516]. One tick must be 15.5us .. 100us long.
#endif

typedef int32_t  ostime_t;

#if !HAS_ostick_conv
#define us2osticks(us)   ((ostime_t)( ((int64_t)(us) * OSTICKS_PER_SEC) / 1000000))
#define ms2osticks(ms)   ((ostime_t)( ((int64_t)(ms) * OSTICKS_PER_SEC)    / 1000))
#define sec2osticks(sec) ((ostime_t)( (int64_t)(sec) * OSTICKS_PER_SEC))
#define osticks2ms(os)   ((int32_t)(((os)*(int64_t)1000    ) / OSTICKS_PER_SEC))
#define osticks2us(os)   ((int32_t)(((os)*(int64_t)1000000 ) / OSTICKS_PER_SEC))
// Special versions
#define us2osticksCeil(us)  ((ostime_t)( ((int64_t)(us) * OSTICKS_PER_SEC + 999999) / 1000000))
#define us2osticksRound(us) ((ostime_t)( ((int64_t)(us) * OSTICKS_PER_SEC + 500000) / 1000000))
#define ms2osticksCeil(ms)  ((ostime_t)( ((int64_t)(ms) * OSTICKS_PER_SEC + 999) / 1000))
#define ms2osticksRound(ms) ((ostime_t)( ((int64_t)(ms) * OSTICKS_PER_SEC + 500) / 1000))
#endif

#ifndef HAS_os_calls

#ifndef os_getDevKey
void os_getDevKey (xref2uint8_t buf);
#endif
#ifndef os_getArtEui
void os_getArtEui (xref2uint8_t buf);
#endif
#ifndef os_getDevEui
void os_getDevEui (xref2uint8_t buf);
#endif
#ifndef os_getTime
ostime_t os_getTime (void);
#endif
#ifndef os_getTimeSecs
uint os_getTimeSecs (void);
#endif
#ifndef os_radio
void os_radio (uint8_t mode);
#endif
#ifndef os_getBattLevel
uint8_t os_getBattLevel (void);
#endif

#ifndef os_rlsbf4
//! Read 32-bit quantity from given pointer in little endian byte order.
uint32_t os_rlsbf4 (xref2cuint8_t buf);
#endif
#ifndef os_wlsbf4
//! Write 32-bit quntity into buffer in little endian byte order.
void os_wlsbf4 (xref2uint8_t buf, uint32_t value);
#endif
#ifndef os_rmsbf4
//! Read 32-bit quantity from given pointer in big endian byte order.
uint32_t os_rmsbf4 (xref2cuint8_t buf);
#endif
#ifndef os_wmsbf4
//! Write 32-bit quntity into buffer in big endian byte order.
void os_wmsbf4 (xref2uint8_t buf, uint32_t value);
#endif
#ifndef os_rlsbf2
//! Read 16-bit quantity from given pointer in little endian byte order.
uint16_t os_rlsbf2 (xref2cuint8_t buf);
#endif
#ifndef os_wlsbf2
//! Write 16-bit quntity into buffer in little endian byte order.
void os_wlsbf2 (xref2uint8_t buf, uint16_t value);
#endif

//! Get random number (default impl for uint16_t).
#ifndef os_getRndU2
#define os_getRndU2() ((uint16_t)((os_getRndU1()<<8)|os_getRndU1()))
#endif
#ifndef os_crc16
uint16_t os_crc16 (xref2uint8_t d, uint len);
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
#define RESOLVE_TABLE(table) constant_table_ ## table

// Accessors for table elements
#define TABLE_GET_U1(table, index) table_get_u1(RESOLVE_TABLE(table), index)
#define TABLE_GET_S1(table, index) table_get_s1(RESOLVE_TABLE(table), index)
#define TABLE_GET_U2(table, index) table_get_u2(RESOLVE_TABLE(table), index)
#define TABLE_GET_S2(table, index) table_get_s2(RESOLVE_TABLE(table), index)
#define TABLE_GET_U4(table, index) table_get_u4(RESOLVE_TABLE(table), index)
#define TABLE_GET_S4(table, index) table_get_s4(RESOLVE_TABLE(table), index)
#define TABLE_GET_OSTIME(table, index) table_get_ostime(RESOLVE_TABLE(table), index)
#define TABLE_GET_U1_TWODIM(table, index1, index2) table_get_u1(RESOLVE_TABLE(table)[index1], index2)

#if defined(__AVR__)
    #include <avr/pgmspace.h>
    // Macro to define the getter functions. This loads data from
    // progmem using pgm_read_xx, or accesses memory directly when the
    // index is a constant so gcc can optimize it away;
    #define TABLE_GETTER(postfix, type, pgm_type) \
        inline type table_get ## postfix(const type *table, size_t index) { \
            if (__builtin_constant_p(table[index])) \
                return table[index]; \
            return pgm_read_ ## pgm_type(&table[index]); \
        }

    TABLE_GETTER(_u1, uint8_t, byte);
    TABLE_GETTER(_s1, int8_t, byte);
    TABLE_GETTER(_u2, uint16_t, word);
    TABLE_GETTER(_s2, int16_t, word);
    TABLE_GETTER(_u4, uint32_t, dword);
    TABLE_GETTER(_s4, int32_t, dword);

    // This assumes ostime_t is 4 bytes, so error out if it is not
    typedef int check_sizeof_ostime_t[(sizeof(ostime_t) == 4) ? 0 : -1];
    TABLE_GETTER(_ostime, ostime_t, dword);

    // For AVR, store constants in PROGMEM, saving on RAM usage
    #define CONST_TABLE(type, name) const type PROGMEM RESOLVE_TABLE(name)

    #define lmic_printf(fmt, ...) printf_P(PSTR(fmt), ## __VA_ARGS__)

#else
    inline uint8_t table_get_u1(const uint8_t *table, size_t index) { return table[index]; }
    inline int8_t table_get_s1(const int8_t *table, size_t index) { return table[index]; }
    inline uint16_t table_get_u2(const uint16_t *table, size_t index) { return table[index]; }
    inline int16_t table_get_s2(const int16_t *table, size_t index) { return table[index]; }
    inline uint32_t table_get_u4(const uint32_t *table, size_t index) { return table[index]; }
    inline int32_t table_get_s4(const int32_t *table, size_t index) { return table[index]; }
    inline ostime_t table_get_ostime(const ostime_t *table, size_t index) { return table[index]; }

    // Declare a table
    #define CONST_TABLE(type, name) const type RESOLVE_TABLE(name)
    #define lmic_printf printf
#endif

#if LMIC_DEBUG_LEVEL > 0
    #define PRINT_DEBUG_1(str, ...) lmic_printf("%lu: " str "\n", os_getTime(), ## __VA_ARGS__)
#else
    #define PRINT_DEBUG_1(str, ...) 
#endif

#if LMIC_DEBUG_LEVEL > 1
    #define PRINT_DEBUG_2(str, ...) lmic_printf("%lu: " str "\n", os_getTime(), ##__VA_ARGS__)
#else
    #define PRINT_DEBUG_2(str, ...) 
#endif

// ======================================================================
// AES support
// !!Keep in sync with lorabase.hpp!!

#ifndef AES_ENC  // if AES_ENC is defined as macro all other values must be too
#define AES_ENC       0x00
#define AES_DEC       0x01
#define AES_MIC       0x02
#define AES_CTR       0x04
#define AES_MICNOAUX  0x08
#endif
#ifndef AESkey  // if AESkey is defined as macro all other values must be too
extern xref2uint8_t AESkey;
extern xref2uint8_t AESaux;
#endif
#ifndef os_aes
uint32_t os_aes (uint8_t mode, xref2uint8_t buf, uint16_t len);
#endif


class OsJob;

using osjobcb_t = void (*) (OsJob*);

class OsScheduler {
    friend class OsJob;
    private:
        OsJob* scheduledjobs = nullptr;
        OsJob* runnablejobs = nullptr;
    public:
        int32_t runloopOnce();

};

extern OsScheduler OSS;

class OsJob {
    friend class OsScheduler;
    private:
        OsScheduler* scheduler;
        OsJob* next = nullptr;
        ostime_t deadline = 0;
        
        

        static bool unlinkjob (OsJob** pnext, OsJob* job);
    protected:
        osjobcb_t  func = nullptr;
        virtual void call();
    
    public:
        OsJob(OsScheduler& scheduler);
        OsJob(): OsJob(OSS){};
        void setCallbackFuture(osjobcb_t cb) { func = cb; };
        void setCallbackRunnable(osjobcb_t cb);
        void setRunnable();
        void clearCallback();
        void setTimedCallback (ostime_t time, osjobcb_t cb);
        void setTimed(ostime_t time);
};

template<class T> class OsJobType : public OsJob {
    public:
        using osjobcbTyped_t = void (T::*) (OsJob*);
    private:
        T* refClass;
        osjobcbTyped_t funcTyped;
    protected:
        virtual void call() {
            PRINT_DEBUG_2("Run func %p on class %p", funcTyped, refClass);
            (refClass->*funcTyped)(this);
        };
    public:
        OsJobType(T* ref): OsJob() { refClass = ref; };
        OsJobType(T* ref, OsScheduler& scheduler): OsJob(scheduler) { refClass = ref; };
        void setCallbackFuture2(osjobcbTyped_t cb) { func = nullptr; funcTyped=cb; };
        void setCallbackRunnable2(osjobcbTyped_t cb) {     
            setCallbackFuture2(cb);
            setRunnable(); 
        };
        void setTimedCallback2 (ostime_t time, osjobcbTyped_t cb) {
            setCallbackFuture2(cb);
            setTimed(time);
        };
};

#endif // _oslmic_h_
