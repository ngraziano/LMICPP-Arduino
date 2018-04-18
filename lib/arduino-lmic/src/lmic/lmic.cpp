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
#include "lmic.h"

#if !defined(MINRX_SYMS)
#define MINRX_SYMS 5
#endif // !defined(MINRX_SYMS)
#define PAMBL_SYMS 8
#define PAMBL_FSK  5
#define PRERX_FSK  1
#define RXLEN_FSK  (1+5+2)

#define BCN_INTV_osticks       sec2osticks(BCN_INTV_sec)
#define TXRX_GUARD_osticks     ms2osticks(TXRX_GUARD_ms)
#define JOIN_GUARD_osticks     ms2osticks(JOIN_GUARD_ms)
#define DELAY_JACC1_osticks    sec2osticks(DELAY_JACC1)
#define DELAY_JACC2_osticks    sec2osticks(DELAY_JACC2)
#define DELAY_EXTDNW2_osticks  sec2osticks(DELAY_EXTDNW2)
#define BCN_RESERVE_osticks    ms2osticks(BCN_RESERVE_ms)
#define BCN_GUARD_osticks      ms2osticks(BCN_GUARD_ms)
#define BCN_WINDOW_osticks     ms2osticks(BCN_WINDOW_ms)
#define AIRTIME_BCN_osticks    us2osticks(AIRTIME_BCN)
#if defined(CFG_eu868)
#define DNW2_SAFETY_ZONE       ms2osticks(3000)
#endif
#if defined(CFG_us915)
#define DNW2_SAFETY_ZONE       ms2osticks(750)
#endif

// Special APIs - for development or testing
#define isTESTMODE() 0

Lmic LMIC;

// ================================================================================
// BEG OS - default implementations for certain OS suport functions

#if !defined(HAS_os_calls)

#if !defined(os_rlsbf2)
uint16_t os_rlsbf2 (xref2cuint8_t buf) {
    return (uint16_t)((uint16_t)buf[0] | ((uint16_t)buf[1]<<8));
}
#endif

#if !defined(os_rlsbf4)
uint32_t os_rlsbf4 (xref2cuint8_t buf) {
    return (uint32_t)((uint32_t)buf[0] | ((uint32_t)buf[1]<<8) | ((uint32_t)buf[2]<<16) | ((uint32_t)buf[3]<<24));
}
#endif


#if !defined(os_rmsbf4)
uint32_t os_rmsbf4 (xref2cuint8_t buf) {
    return (uint32_t)((uint32_t)buf[3] | ((uint32_t)buf[2]<<8) | ((uint32_t)buf[1]<<16) | ((uint32_t)buf[0]<<24));
}
#endif


#if !defined(os_wlsbf2)
void os_wlsbf2 (xref2uint8_t buf, uint16_t v) {
    buf[0] = v;
    buf[1] = v>>8;
}
#endif

#if !defined(os_wlsbf4)
void os_wlsbf4 (xref2uint8_t buf, uint32_t v) {
    buf[0] = v;
    buf[1] = v>>8;
    buf[2] = v>>16;
    buf[3] = v>>24;
}
#endif

#if !defined(os_wmsbf4)
void os_wmsbf4 (xref2uint8_t buf, uint32_t v) {
    buf[3] = v;
    buf[2] = v>>8;
    buf[1] = v>>16;
    buf[0] = v>>24;
}
#endif

#if !defined(os_getBattLevel)
uint8_t os_getBattLevel (void) {
    return MCMD_DEVS_BATT_NOINFO;
}
#endif

#if !defined(os_crc16)
// New CRC-16 CCITT(XMODEM) checksum for beacons:
uint16_t os_crc16 (xref2uint8_t data, uint len) {
    uint16_t remainder = 0;
    uint16_t polynomial = 0x1021;
    for( uint i = 0; i < len; i++ ) {
        remainder ^= data[i] << 8;
        for( uint8_t bit = 8; bit > 0; bit--) {
            if( (remainder & 0x8000) )
                remainder = (remainder << 1) ^ polynomial;
            else
                remainder <<= 1;
        }
    }
    return remainder;
}
#endif

#endif // !HAS_os_calls

// END OS - default implementations for certain OS suport functions
// ================================================================================

// ================================================================================
// BEG AES

static void micB0 (uint32_t devaddr, uint32_t seqno, int dndir, int len) {
    os_clearMem(AESaux,16);
    AESaux[0]  = 0x49;
    AESaux[5]  = dndir?1:0;
    AESaux[15] = len;
    os_wlsbf4(AESaux+ 6,devaddr);
    os_wlsbf4(AESaux+10,seqno);
}


static int aes_verifyMic (xref2cuint8_t key, uint32_t devaddr, uint32_t seqno, int dndir, xref2uint8_t pdu, int len) {
    micB0(devaddr, seqno, dndir, len);
    os_copyMem(AESkey,key,16);
    return os_aes(AES_MIC, pdu, len) == os_rmsbf4(pdu+len);
}


static void aes_appendMic (xref2cuint8_t key, uint32_t devaddr, uint32_t seqno, int dndir, xref2uint8_t pdu, int len) {
    micB0(devaddr, seqno, dndir, len);
    os_copyMem(AESkey,key,16);
    // MSB because of internal structure of AES
    os_wmsbf4(pdu+len, os_aes(AES_MIC, pdu, len));
}


static void aes_appendMic0 (xref2uint8_t pdu, int len) {
    os_getDevKey(AESkey);
    os_wmsbf4(pdu+len, os_aes(AES_MIC|AES_MICNOAUX, pdu, len));  // MSB because of internal structure of AES
}


static int aes_verifyMic0 (xref2uint8_t pdu, int len) {
    os_getDevKey(AESkey);
    return os_aes(AES_MIC|AES_MICNOAUX, pdu, len) == os_rmsbf4(pdu+len);
}


static void aes_encrypt (xref2uint8_t pdu, int len) {
    os_getDevKey(AESkey);
    os_aes(AES_ENC, pdu, len);
}


static void aes_cipher (xref2cuint8_t key, uint32_t devaddr, uint32_t seqno, int dndir, xref2uint8_t payload, int len) {
    if( len <= 0 )
        return;
    os_clearMem(AESaux, 16);
    AESaux[0] = AESaux[15] = 1; // mode=cipher / dir=down / block counter=1
    AESaux[5] = dndir?1:0;
    os_wlsbf4(AESaux+ 6,devaddr);
    os_wlsbf4(AESaux+10,seqno);
    os_copyMem(AESkey,key,16);
    os_aes(AES_CTR, payload, len);
}


static void aes_sessKeys (uint16_t devnonce, xref2cuint8_t artnonce, xref2uint8_t nwkkey, xref2uint8_t artkey) {
    os_clearMem(nwkkey, 16);
    nwkkey[0] = 0x01;
    os_copyMem(nwkkey+1, artnonce, LEN_ARTNONCE+LEN_NETID);
    os_wlsbf2(nwkkey+1+LEN_ARTNONCE+LEN_NETID, devnonce);
    os_copyMem(artkey, nwkkey, 16);
    artkey[0] = 0x02;

    os_getDevKey(AESkey);
    os_aes(AES_ENC, nwkkey, 16);
    os_getDevKey(AESkey);
    os_aes(AES_ENC, artkey, 16);
}

// END AES
// ================================================================================


// ================================================================================
// BEG LORA

#if defined(CFG_eu868) // ========================================

#define maxFrameLen(dr) ((dr)<=DR_SF9 ? TABLE_GET_U1(maxFrameLens, (dr)) : 0xFF)
CONST_TABLE(uint8_t, maxFrameLens) [] = { 64,64,64,123 };

CONST_TABLE(uint8_t, _DR2RPS_CRC)[] = {
    ILLEGAL_RPS,
    (uint8_t)MAKERPS(SF12, BW125, CR_4_5, 0, 0),
    (uint8_t)MAKERPS(SF11, BW125, CR_4_5, 0, 0),
    (uint8_t)MAKERPS(SF10, BW125, CR_4_5, 0, 0),
    (uint8_t)MAKERPS(SF9,  BW125, CR_4_5, 0, 0),
    (uint8_t)MAKERPS(SF8,  BW125, CR_4_5, 0, 0),
    (uint8_t)MAKERPS(SF7,  BW125, CR_4_5, 0, 0),
    (uint8_t)MAKERPS(SF7,  BW250, CR_4_5, 0, 0),
    (uint8_t)MAKERPS(FSK,  BW125, CR_4_5, 0, 0),
    ILLEGAL_RPS
};

static CONST_TABLE(int8_t, TXPOWLEVELS)[] = {
    20, 14, 11, 8, 5, 2, 0,0, 0,0,0,0, 0,0,0,0
};
#define pow2dBm(mcmd_ladr_p1) (TABLE_GET_S1(TXPOWLEVELS, (mcmd_ladr_p1&MCMD_LADR_POW_MASK)>>MCMD_LADR_POW_SHIFT))

#elif defined(CFG_us915) // ========================================

#define maxFrameLen(dr) ((dr)<=DR_SF11CR ? TABLE_GET_U1(maxFrameLens, (dr)) : 0xFF)
CONST_TABLE(uint8_t, maxFrameLens) [] = { 24,66,142,255,255,255,255,255,  66,142 };

CONST_TABLE(uint8_t, _DR2RPS_CRC)[] = {
    ILLEGAL_RPS,
    MAKERPS(SF10, BW125, CR_4_5, 0, 0),
    MAKERPS(SF9 , BW125, CR_4_5, 0, 0),
    MAKERPS(SF8 , BW125, CR_4_5, 0, 0),
    MAKERPS(SF7 , BW125, CR_4_5, 0, 0),
    MAKERPS(SF8 , BW500, CR_4_5, 0, 0),
    ILLEGAL_RPS ,
    ILLEGAL_RPS ,
    ILLEGAL_RPS ,
    MAKERPS(SF12, BW500, CR_4_5, 0, 0),
    MAKERPS(SF11, BW500, CR_4_5, 0, 0),
    MAKERPS(SF10, BW500, CR_4_5, 0, 0),
    MAKERPS(SF9 , BW500, CR_4_5, 0, 0),
    MAKERPS(SF8 , BW500, CR_4_5, 0, 0),
    MAKERPS(SF7 , BW500, CR_4_5, 0, 0),
    ILLEGAL_RPS
};

#define pow2dBm(mcmd_ladr_p1) ((int8_t)(30 - (((mcmd_ladr_p1)&MCMD_LADR_POW_MASK)<<1)))

#endif // ================================================

static CONST_TABLE(uint8_t, SENSITIVITY)[7][3] = {
    // ------------bw----------
    // 125kHz    250kHz    500kHz
    { 141-109,  141-109, 141-109 },  // FSK
    { 141-127,  141-124, 141-121 },  // SF7
    { 141-129,  141-126, 141-123 },  // SF8
    { 141-132,  141-129, 141-126 },  // SF9
    { 141-135,  141-132, 141-129 },  // SF10
    { 141-138,  141-135, 141-132 },  // SF11
    { 141-141,  141-138, 141-135 }   // SF12
};

int getSensitivity (rps_t rps) {
    return -141 + TABLE_GET_U1_TWODIM(SENSITIVITY, getSf(rps), getBw(rps));
}

ostime_t calcAirTime (rps_t rps, uint8_t plen) {
    uint8_t bw = getBw(rps);  // 0,1,2 = 125,250,500kHz
    uint8_t sf = getSf(rps);  // 0=FSK, 1..6 = SF7..12
    #if !defined(DISABLE_FSK)
    if( sf == FSK ) {
        return (plen+/*preamble*/5+/*syncword*/3+/*len*/1+/*crc*/2) * /*bits/byte*/8
            * (int32_t)OSTICKS_PER_SEC / /*kbit/s*/50000;
    }
    #endif
    uint8_t sfx = 4*(sf+(7-SF7));
    uint8_t q = sfx - (sf >= SF11 ? 8 : 0);
    int tmp = 8*plen - sfx + 28 + (getNocrc(rps)?0:16) - (getIh(rps)?20:0);
    if( tmp > 0 ) {
        tmp = (tmp + q - 1) / q;
        tmp *= getCr(rps)+5;
        tmp += 8;
    } else {
        tmp = 8;
    }
    tmp = (tmp<<2) + /*preamble*/49 /* 4 * (8 + 4.25) */;
    // bw = 125000 = 15625 * 2^3
    //      250000 = 15625 * 2^4
    //      500000 = 15625 * 2^5
    // sf = 7..12
    //
    // osticks =  tmp * OSTICKS_PER_SEC * 1<<sf / bw
    //
    // 3 => counter reduced divisor 125000/8 => 15625
    // 2 => counter 2 shift on tmp
    sfx = sf+(7-SF7) - (3+2) - bw;
    int div = 15625;
    if( sfx > 4 ) {
        // prevent 32bit signed int overflow in last step
        div >>= sfx-4;
        sfx = 4;
    }
    // Need 32bit arithmetic for this last step
    return (((ostime_t)tmp << sfx) * OSTICKS_PER_SEC + div/2) / div;
}

extern inline rps_t updr2rps (dr_t dr);
extern inline rps_t dndr2rps (dr_t dr);
extern inline int isFasterDR (dr_t dr1, dr_t dr2);
extern inline int isSlowerDR (dr_t dr1, dr_t dr2);
extern inline dr_t  incDR    (dr_t dr);
extern inline dr_t  decDR    (dr_t dr);
extern inline dr_t  assertDR (dr_t dr);
extern inline bool  validDR  (dr_t dr);
extern inline dr_t  lowerDR  (dr_t dr, uint8_t n);

extern inline sf_t  getSf    (rps_t params);
extern inline rps_t setSf    (rps_t params, sf_t sf);
extern inline bw_t  getBw    (rps_t params);
extern inline rps_t setBw    (rps_t params, bw_t cr);
extern inline cr_t  getCr    (rps_t params);
extern inline rps_t setCr    (rps_t params, cr_t cr);
extern inline int   getNocrc (rps_t params);
extern inline rps_t setNocrc (rps_t params, int nocrc);
extern inline int   getIh    (rps_t params);
extern inline rps_t setIh    (rps_t params, int ih);
extern inline rps_t makeRps  (sf_t sf, bw_t bw, cr_t cr, int ih, int nocrc);
extern inline int   sameSfBw (rps_t r1, rps_t r2);

// END LORA
// ================================================================================


// Adjust DR for TX retries
//  - indexed by retry count
//  - return steps to lower DR
static CONST_TABLE(uint8_t, DRADJUST)[2+TXCONF_ATTEMPTS] = {
    // normal frames - 1st try / no retry
    0,
    // confirmed frames
    0,0,1,0,1,0,1,0,0
};


// Table below defines the size of one symbol as
//   symtime = 256us * 2^T(sf,bw)
// 256us is called one symunit.
//                 SF:
//      BW:      |__7___8___9__10__11__12
//      125kHz   |  2   3   4   5   6   7
//      250kHz   |  1   2   3   4   5   6
//      500kHz   |  0   1   2   3   4   5
//
// Times for half symbol per DR
// Per DR table to minimize rounding errors
static CONST_TABLE(ostime_t, DR2HSYM_osticks)[] = {
#if defined(CFG_eu868)
#define dr2hsym(dr) (TABLE_GET_OSTIME(DR2HSYM_osticks, (dr)))
    us2osticksRound(128<<7),  // DR_SF12
    us2osticksRound(128<<6),  // DR_SF11
    us2osticksRound(128<<5),  // DR_SF10
    us2osticksRound(128<<4),  // DR_SF9
    us2osticksRound(128<<3),  // DR_SF8
    us2osticksRound(128<<2),  // DR_SF7
    us2osticksRound(128<<1),  // DR_SF7B
    us2osticksRound(80)       // FSK -- not used (time for 1/2 byte)
#elif defined(CFG_us915)
#define dr2hsym(dr) (TABLE_GET_OSTIME(DR2HSYM_osticks, (dr)&7))  // map DR_SFnCR -> 0-6
    us2osticksRound(128<<5),  // DR_SF10   DR_SF12CR
    us2osticksRound(128<<4),  // DR_SF9    DR_SF11CR
    us2osticksRound(128<<3),  // DR_SF8    DR_SF10CR
    us2osticksRound(128<<2),  // DR_SF7    DR_SF9CR
    us2osticksRound(128<<1),  // DR_SF8C   DR_SF8CR
    us2osticksRound(128<<0)   // ------    DR_SF7CR
#endif
};


static ostime_t rndDelay (uint8_t secSpan) {
    uint16_t r = os_getRndU2();
    ostime_t delay = r;
    if( delay > OSTICKS_PER_SEC )
        delay = r % (uint16_t)OSTICKS_PER_SEC;
    if( secSpan > 0 )
        delay += ((uint8_t)r % secSpan) * OSTICKS_PER_SEC;
    return delay;
}


void Lmic::txDelay (ostime_t reftime, uint8_t secSpan) {
    reftime += rndDelay(secSpan);
    if( globalDutyRate == 0  ||  (reftime - globalDutyAvail) > 0 ) {
        globalDutyAvail = reftime;
        opmode |= OP_RNDTX;
    }
}


void Lmic::setDrJoin (dr_t dr) {
    datarate = dr;
}


void Lmic::setDrTxpow (uint8_t dr, int8_t pow) {
    if( pow != KEEP_TXPOW )
        adrTxPow = pow;
    if( datarate != dr ) {
        datarate = dr;
        opmode |= OP_NEXTCHNL;
    }
}


#if defined(CFG_eu868)
// ================================================================================
//
// BEG: EU868 related stuff
//
enum { NUM_DEFAULT_CHANNELS=3 };
static CONST_TABLE(uint32_t, iniChannelFreq)[6] = {
    // Join frequencies and duty cycle limit (0.1%)
    EU868_F1|BAND_MILLI, EU868_F2|BAND_MILLI, EU868_F3|BAND_MILLI,
    // Default operational frequencies
    EU868_F1|BAND_CENTI, EU868_F2|BAND_CENTI, EU868_F3|BAND_CENTI,
};

void Lmic::initDefaultChannels (bool join) {
    PRINT_DEBUG_2("Init Default Channel join?=%d",join);
    os_clearMem(&channelFreq, sizeof(channelFreq));
    os_clearMem(&channelDrMap, sizeof(channelDrMap));
    os_clearMem(&bands, sizeof(bands));
    

    channelMap = 0x07;
    uint8_t su = join ? 0 : 3;
    for( uint8_t fu=0; fu<3; fu++,su++ ) {
        channelFreq[fu]  = TABLE_GET_U4(iniChannelFreq, su);
        channelDrMap[fu] = DR_RANGE_MAP(DR_SF12,DR_SF7);
    }

    bands[BAND_MILLI].txcap    = 1000;  // 0.1%
    bands[BAND_MILLI].txpow    = 14;
    bands[BAND_MILLI].lastchnl = os_getRndU1() % MAX_CHANNELS;
    bands[BAND_CENTI].txcap    = 100;   // 1%
    bands[BAND_CENTI].txpow    = 14;
    bands[BAND_CENTI].lastchnl = os_getRndU1() % MAX_CHANNELS;
    bands[BAND_DECI ].txcap    = 10;    // 10%
    bands[BAND_DECI ].txpow    = 27;
    bands[BAND_DECI ].lastchnl = os_getRndU1() % MAX_CHANNELS;
    bands[BAND_MILLI].avail = os_getTime();
    bands[BAND_CENTI].avail = os_getTime();
    bands[BAND_DECI ].avail = os_getTime();
}

bool Lmic::setupBand (uint8_t bandidx, int8_t txpow, uint16_t txcap) {
    if( bandidx > BAND_AUX ) 
        return false;
    band_t* b = &bands[bandidx];
    b->txpow = txpow;
    b->txcap = txcap;
    b->avail = os_getTime();
    b->lastchnl = os_getRndU1() % MAX_CHANNELS;
    return true;
}

bool Lmic::setupChannel (uint8_t chidx, uint32_t newfreq, uint16_t drmap, int8_t band) {
    if( chidx >= MAX_CHANNELS )
        return false;
    if( band == -1 ) {
        if( newfreq >= 869400000 && newfreq <= 869650000 )
            newfreq |= BAND_DECI;   // 10% 27dBm
        else if( (newfreq >= 868000000 && newfreq <= 868600000) ||
                 (newfreq >= 869700000 && newfreq <= 870000000)  )
            newfreq |= BAND_CENTI;  // 1% 14dBm
        else
            newfreq |= BAND_MILLI;  // 0.1% 14dBm
    } else {
        if( band > BAND_AUX ) return 0;
        newfreq = (newfreq&~3) | band;
    }
    channelFreq [chidx] = newfreq;
    channelDrMap[chidx] = drmap==0 ? DR_RANGE_MAP(DR_SF12,DR_SF7) : drmap;
    channelMap |= 1<<chidx;  // enabled right away
    return true;
}

void Lmic::disableChannel (uint8_t channel) {
    channelFreq[channel] = 0;
    channelDrMap[channel] = 0;
    channelMap &= ~(1<<channel);
}

static uint32_t convFreq (xref2uint8_t ptr) {
    uint32_t newfreq = (os_rlsbf4(ptr-1) >> 8) * 100;
    if( newfreq < EU868_FREQ_MIN || newfreq > EU868_FREQ_MAX )
        newfreq = 0;
    return newfreq;
}

uint8_t Lmic::mapChannels (uint8_t chpage, uint16_t chmap) {
    // Bad page, disable all channel, enable non-existent
    if( chpage != 0 || chmap==0 || (chmap & ~channelMap) != 0 )
        return 0;  // illegal input
    for( uint8_t chnl=0; chnl<MAX_CHANNELS; chnl++ ) {
        if( (chmap & (1<<chnl)) != 0 && channelFreq[chnl] == 0 )
            chmap &= ~(1<<chnl); // ignore - channel is not defined
    }
    channelMap = chmap;
    return 1;
}


void Lmic::updateTx (ostime_t txbeg) {
    uint32_t newfreq = channelFreq[txChnl];
    // Update global/band specific duty cycle stats
    ostime_t airtime = calcAirTime(rps, dataLen);
    // Update channel/global duty cycle stats
    xref2band_t band = &bands[freq & 0x3];
    freq  = newfreq & ~(uint32_t)3;
    txpow = band->txpow;
    band->avail = txbeg + airtime * band->txcap;
    if( globalDutyRate != 0 )
        globalDutyAvail = txbeg + (airtime<<globalDutyRate);
    #if LMIC_DEBUG_LEVEL > 1
        lmic_printf("%lu: Updating info for TX at %lu, airtime will be %lu. Setting available time for band %d to %lu\n", os_getTime(), txbeg, airtime, freq & 0x3, band->avail);
        if( globalDutyRate != 0 )
            lmic_printf("%lu: Updating global duty avail to %lu\n", os_getTime(), globalDutyAvail);
    #endif
}

ostime_t Lmic::nextTx (ostime_t now) {
    uint8_t bmap=0xF;
    do {
        ostime_t mintime = now + /*8h*/sec2osticks(28800);
        uint8_t band=0;
        for( uint8_t bi=0; bi<4; bi++ ) {
            if( (bmap & (1<<bi)) && mintime - bands[bi].avail > 0 ) {
                #if LMIC_DEBUG_LEVEL > 1
                    lmic_printf("%lu: Considering band %d, which is available at %lu\n", os_getTime(), bi, bands[bi].avail);
                #endif
                mintime = bands[band = bi].avail;
            }
        }
        // Find next channel in given band
        uint8_t chnl = bands[band].lastchnl;
        for( uint8_t ci=0; ci<MAX_CHANNELS; ci++ ) {
            if( (chnl = (chnl+1)) >= MAX_CHANNELS )
                chnl -=  MAX_CHANNELS;
            if( (channelMap & (1<<chnl)) != 0  &&  // channel enabled
                (channelDrMap[chnl] & (1<<(datarate&0xF))) != 0  &&
                band == (channelFreq[chnl] & 0x3) ) { // in selected band
                txChnl = bands[band].lastchnl = chnl;
                return mintime;
            }
        }
        #if LMIC_DEBUG_LEVEL > 1
            lmic_printf("%lu: No channel found in band %d\n", os_getTime(), band);
        #endif
        if( (bmap &= ~(1<<band)) == 0 ) {
            // No feasible channel  found!
            return mintime;
        }
    } while(1);
}

void Lmic::setRx1Params() {
 /*freq/rps remain unchanged*/
}

#if !defined(DISABLE_JOIN)
void Lmic::initJoinLoop () {
    txChnl = os_getRndU1() % 3;
    adrTxPow = 14;
    setDrJoin(DR_SF7);
    initDefaultChannels(true);
    ASSERT((opmode & OP_NEXTCHNL)==0);
    txend = bands[BAND_MILLI].avail + rndDelay(8);
    PRINT_DEBUG_1("Init Join loop : avail=%lu txend=%lu", bands[BAND_MILLI].avail, txend);
}


ostime_t Lmic::nextJoinState (void) {
    uint8_t failed = 0;

    // Try 869.x and then 864.x with same DR
    // If both fail try next lower datarate
    if( ++txChnl == 3 )
        txChnl = 0;
    if( (++txCnt & 1) == 0 ) {
        // Lower DR every 2nd try (having tried 868.x and 864.x with the same DR)
        if( datarate == DR_SF12 )
            failed = 1; // we have tried all DR - signal EV_JOIN_FAILED
        else
            setDrJoin(decDR(datarate));
    }
    // Clear NEXTCHNL because join state engine controls channel hopping
    opmode &= ~OP_NEXTCHNL;
    // Move txend to randomize synchronized concurrent joins.
    // Duty cycle is based on txend.
    ostime_t time = os_getTime();
    if( time - bands[BAND_MILLI].avail < 0 )
        time = bands[BAND_MILLI].avail;
    txend = time +
        (isTESTMODE()
         // Avoid collision with JOIN ACCEPT @ SF12 being sent by GW (but we missed it)
         ? DNW2_SAFETY_ZONE
         // Otherwise: randomize join (street lamp case):
         // SF12:255, SF11:127, .., SF7:8secs
         : DNW2_SAFETY_ZONE+rndDelay(255>>datarate));
    #if LMIC_DEBUG_LEVEL > 1
        if (failed)
            lmic_printf("%lu: Join failed\n", os_getTime());
        else
            lmic_printf("%lu: Scheduling next join at %lu\n", os_getTime(), txend);
    #endif
    // 1 - triggers EV_JOIN_FAILED event
    return failed;
}
#endif // !DISABLE_JOIN

//
// END: EU868 related stuff
//
// ================================================================================
#elif defined(CFG_us915)
// ================================================================================
//
// BEG: US915 related stuff
//

void Lmic::initDefaultChannels () {
    for( uint8_t i=0; i<4; i++ )
        channelMap[i] = 0xFFFF;
    channelMap[4] = 0x00FF;
}

static uint32_t convFreq (xref2uint8_t ptr) {
    uint32_t freq = (os_rlsbf4(ptr-1) >> 8) * 100;
    if( freq < US915_FREQ_MIN || freq > US915_FREQ_MAX )
        freq = 0;
    return freq;
}

bool Lmic::setupChannel (uint8_t chidx, uint32_t freq, uint16_t drmap, int8_t band) {
    if( chidx < 72 || chidx >= 72+MAX_XCHANNELS )
        return false; // channels 0..71 are hardwired
    chidx -= 72;
    xchFreq[chidx] = freq;
    xchDrMap[chidx] = drmap==0 ? DR_RANGE_MAP(DR_SF10,DR_SF8C) : drmap;
    channelMap[chidx>>4] |= (1<<(chidx&0xF));
    return true;
}

void Lmic::disableChannel (uint8_t channel) {
    if( channel < 72+MAX_XCHANNELS )
        channelMap[channel>>4] &= ~(1<<(channel&0xF));
}

void Lmic::enableChannel (uint8_t channel) {
    if( channel < 72+MAX_XCHANNELS )
        channelMap[channel>>4] |= (1<<(channel&0xF));
}

void  Lmic::enableSubBand (uint8_t band) {
  ASSERT(band < 8);
  uint8_t start = band * 8;
  uint8_t end = start + 8;
  for (int channel=start; channel < end; ++channel )
      enableChannel(channel);
}
void  Lmic::disableSubBand (uint8_t band) {
  ASSERT(band < 8);
  uint8_t start = band * 8;
  uint8_t end = start + 8;
  for (int channel=start; channel < end; ++channel )
      disableChannel(channel);
}
void  Lmic::selectSubBand (uint8_t band) {
  ASSERT(band < 8);
  for (int b=0; b<8; ++b) {
    if (band==b)
      enableSubBand(b);
    else
      disableSubBand(b);
  }
}

uint8_t Lmic::mapChannels (uint8_t chpage, uint16_t chmap) {
    if( chpage == MCMD_LADR_CHP_125ON || chpage == MCMD_LADR_CHP_125OFF ) {
        uint16_t en125 = chpage == MCMD_LADR_CHP_125ON ? 0xFFFF : 0x0000;
        for( uint8_t u=0; u<4; u++ )
            channelMap[u] = en125;
        channelMap[64/16] = chmap;
    } else {
        if( chpage >= (72+MAX_XCHANNELS+15)/16 )
            return 0;
        channelMap[chpage] = chmap;
    }
    return 1;
}

void Lmic::updateTx (ostime_t txbeg) {
    uint8_t chnl = txChnl;
    if( chnl < 64 ) {
        freq = US915_125kHz_UPFBASE + chnl*US915_125kHz_UPFSTEP;
        txpow = 30;
        return;
    }
    txpow = 26;
    if( chnl < 64+8 ) {
        freq = US915_500kHz_UPFBASE + (chnl-64)*US915_500kHz_UPFSTEP;
    } else {
        ASSERT(chnl < 64+8+MAX_XCHANNELS);
        freq = xchFreq[chnl-72];
    }

    // Update global duty cycle stats
    if( globalDutyRate != 0 ) {
        ostime_t airtime = calcAirTime(rps, dataLen);
        globalDutyAvail = txbeg + (airtime<<globalDutyRate);
    }
}

// US does not have duty cycling - return now as earliest TX time
ostime_t Lmic::nextTx (ostime_t now) {
    if( chRnd==0 )
        chRnd = os_getRndU1() & 0x3F;
    if( datarate >= DR_SF8C ) { // 500kHz
        uint8_t map = channelMap[64/16]&0xFF;
        for( uint8_t i=0; i<8; i++ ) {
            if( (map & (1<<(++chRnd & 7))) != 0 ) {
                txChnl = 64 + (chRnd & 7);
                return now;
            }
        }
    } else { // 125kHz
        for( uint8_t i=0; i<64; i++ ) {
            uint8_t chnl = ++chRnd & 0x3F;
            if( (channelMap[(chnl >> 4)] & (1<<(chnl & 0xF))) != 0 ) {
                txChnl = chnl;
                return now;
            }
        }
    }
    // No feasible channel  found! Keep old one.
    return now;
}

void  Lmic::setRx1Params() {                                                
    freq = US915_500kHz_DNFBASE + (txChnl & 0x7) * US915_500kHz_DNFSTEP; 
    if( /* TX datarate */dndr < DR_SF8C )                          
        dndr += DR_SF10CR - DR_SF10;                               
    else if( dndr == DR_SF8C )                                     
        dndr = DR_SF7CR;                                           
    rps = dndr2rps(dndr);                                     
}

#if !defined(DISABLE_JOIN)
void Lmic::initJoinLoop (void) {
    chRnd = 0;
    txChnl = 0;
    adrTxPow = 20;
    ASSERT((opmode & OP_NEXTCHNL)==0);
    txend = os_getTime();
    setDrJoin(DR_SF7);
}

ostime_t Lmic::nextJoinState (void) {
    // Try the following:
    //   SF7/8/9/10  on a random channel 0..63
    //   SF8C        on a random channel 64..71
    //
    uint8_t failed = 0;
    if( datarate != DR_SF8C ) {
        txChnl = 64+(txChnl&7);
        setDrJoin(DR_SF8C);
    } else {
        txChnl = os_getRndU1() & 0x3F;
        int8_t dr = DR_SF7 - ++txCnt;
        if( dr < DR_SF10 ) {
            dr = DR_SF10;
            failed = 1; // All DR exhausted - signal failed
        }
        setDrJoin(dr);
    }
    opmode &= ~OP_NEXTCHNL;
    txend = os_getTime() +
        (isTESTMODE()
         // Avoid collision with JOIN ACCEPT being sent by GW (but we missed it - GW is still busy)
         ? DNW2_SAFETY_ZONE
         // Otherwise: randomize join (street lamp case):
         // SF10:16, SF9=8,..SF8C:1secs
         : rndDelay(16>>datarate));
    // 1 - triggers EV_JOIN_FAILED event
    return failed;
}
#endif // !DISABLE_JOIN

//
// END: US915 related stuff
//
// ================================================================================
#else
#error Unsupported frequency band!
#endif


void Lmic::runEngineUpdate (OsJobBase* osjob) {
    engineUpdate();
}


void Lmic::reportEvent (ev_t ev) {
    ON_LMIC_EVENT(ev);
    engineUpdate();
}


void Lmic::runReset (OsJobBase* osjob) {
    // Disable session
    reset();
#if !defined(DISABLE_JOIN)
    startJoining();
#endif // !DISABLE_JOIN
    reportEvent(EV_RESET);
}

void Lmic::stateJustJoined () {
    seqnoDn = 0;
    seqnoUp = 0;
    rejoinCnt = 0;
    dnConf = 0;
    adrChanged =  0;
    ladrAns = false;
    devsAns = false;
#if !defined(DISABLE_MCMD_SNCH_REQ)
    snchAns     = 0;
#endif
#if !defined(DISABLE_MCMD_DN2P_SET)
    dn2Ans      = 0;
#endif
    moreData    = 0;
#if !defined(DISABLE_MCMD_DCAP_REQ)
    dutyCapAns  = 0;
#endif
    upRepeat    = 0;
    adrAckReq   = LINK_CHECK_INIT;
    dn2Dr       = DR_DNW2;
    dn2Freq     = FREQ_DNW2;
}


// ================================================================================
// Decoding frames
bool Lmic::decodeFrame () {
    xref2uint8_t d = frame;
    uint8_t hdr    = d[0];
    uint8_t ftype  = hdr & HDR_FTYPE;
    int  dlen   = dataLen;
    const char *window = (txrxFlags & TXRX_DNW1) ? "RX1" : ((txrxFlags & TXRX_DNW2) ? "RX2" : "Other");
    if( dlen < OFF_DAT_OPTS+4 ||
        (hdr & HDR_MAJOR) != HDR_MAJOR_V1 ||
        (ftype != HDR_FTYPE_DADN  &&  ftype != HDR_FTYPE_DCDN) ) {
        // Basic sanity checks failed
        PRINT_DEBUG_1("Invalid downlink, window=%s", window);
        dataLen = 0;
        return false;
    }
    // Validate exact frame length
    // Note: device address was already read+evaluated in order to arrive here.
    int  fct   = d[OFF_DAT_FCT];
    uint32_t addr  = os_rlsbf4(&d[OFF_DAT_ADDR]);
    uint32_t seqno = os_rlsbf2(&d[OFF_DAT_SEQNO]);
    int  olen  = fct & FCT_OPTLEN;
    int  ackup = (fct & FCT_ACK) != 0 ? 1 : 0;   // ACK last up frame
    int  poff  = OFF_DAT_OPTS+olen;
    int  pend  = dlen-4;  // MIC

    if( addr != devaddr ) {
        PRINT_DEBUG_1("Invalid address, window=%s", window);
        dataLen = 0;
        return false;
    }
    if( poff > pend ) {
        PRINT_DEBUG_1("Invalid offset, window=%s", window);
        dataLen = 0;
        return false;
    }

    int port = -1;
    int replayConf = 0;

    if( pend > poff )
        port = d[poff++];

    seqno = seqnoDn + (uint16_t)(seqno - seqnoDn);

    if( !aes_verifyMic(nwkKey, devaddr, seqno, /*dn*/1, d, pend) ) {
        PRINT_DEBUG_1("Fail to verify aes mic, window=%s", window);
        dataLen = 0;
        return false;
    }
    if( seqno < seqnoDn ) {
        if( (int32_t)seqno > (int32_t)seqnoDn ) {
            dataLen = 0;
            return false;
        }
        if( seqno != seqnoDn-1 || !dnConf || ftype != HDR_FTYPE_DCDN ) {
            dataLen = 0;
            return false;
        }
        // Replay of previous sequence number allowed only if
        // previous frame and repeated both requested confirmation
        replayConf = 1;
    }
    else {
        if( seqno > seqnoDn ) {
            // skip in sequence number
            // log ?
        }
        seqnoDn = seqno+1;  // next number to be expected
        // DN frame requested confirmation - provide ACK once with next UP frame
        dnConf = (ftype == HDR_FTYPE_DCDN ? FCT_ACK : 0);
    }

    if( dnConf || (fct & FCT_MORE) )
        opmode |= OP_POLL;

    // We heard from network
    adrChanged = rejoinCnt = 0;
    if( adrAckReq != LINK_CHECK_OFF )
        adrAckReq = LINK_CHECK_INIT;

    // Process OPTS
    int m = rssi - RSSI_OFF - getSensitivity(rps);
    margin = m < 0 ? 0 : m > 254 ? 254 : m;

    uint8_t* opts = &d[OFF_DAT_OPTS];
    int oidx = 0;
    while( oidx < olen ) {
        switch( opts[oidx] ) {
        case MCMD_LCHK_ANS: {
            //int gwmargin = opts[oidx+1];
            //int ngws = opts[oidx+2];
            oidx += 3;
            continue;
        }
        case MCMD_LADR_REQ: {
            uint8_t p1     = opts[oidx+1];            // txpow + DR
            uint16_t chmap  = os_rlsbf2(&opts[oidx+2]);// list of enabled channels
            uint8_t chpage = opts[oidx+4] & MCMD_LADR_CHPAGE_MASK;     // channel page
            uint8_t uprpt  = opts[oidx+4] & MCMD_LADR_REPEAT_MASK;     // up repeat count
            oidx += 5;

            ladrAns = 0x80 |     // Include an answer into next frame up
                MCMD_LADR_ANS_POWACK | MCMD_LADR_ANS_CHACK | MCMD_LADR_ANS_DRACK;
            if( !mapChannels(chpage, chmap) )
                ladrAns &= ~MCMD_LADR_ANS_CHACK;
            dr_t dr = (dr_t)(p1>>MCMD_LADR_DR_SHIFT);
            if( !validDR(dr) ) {
                ladrAns &= ~MCMD_LADR_ANS_DRACK;
            }
            if( (ladrAns & 0x7F) == (MCMD_LADR_ANS_POWACK | MCMD_LADR_ANS_CHACK | MCMD_LADR_ANS_DRACK) ) {
                // Nothing went wrong - use settings
                upRepeat = uprpt;
                setDrTxpow(dr, pow2dBm(p1));
            }
            adrChanged = 1;  // Trigger an ACK to NWK
            continue;
        }
        case MCMD_DEVS_REQ: {
            devsAns = 1;
            oidx += 1;
            continue;
        }
        case MCMD_DN2P_SET: {
#if !defined(DISABLE_MCMD_DN2P_SET)
            dr_t dr = (dr_t)(opts[oidx+1] & 0x0F);
            uint32_t newfreq = convFreq(&opts[oidx+2]);
            dn2Ans = 0x80;   // answer pending
            if( validDR(dr) )
                dn2Ans |= MCMD_DN2P_ANS_DRACK;
            if( newfreq != 0 )
                dn2Ans |= MCMD_DN2P_ANS_CHACK;
            if( dn2Ans == (0x80|MCMD_DN2P_ANS_DRACK|MCMD_DN2P_ANS_CHACK) ) {
                dn2Dr = dr;
                dn2Freq = newfreq;
            }
#endif // !DISABLE_MCMD_DN2P_SET
            oidx += 5;
            continue;
        }
        case MCMD_DCAP_REQ: {
#if !defined(DISABLE_MCMD_DCAP_REQ)
            uint8_t cap = opts[oidx+1];
            // A value cap=0xFF means device is OFF unless enabled again manually.
            if( cap==0xFF )
                opmode |= OP_SHUTDOWN;  // stop any sending
            globalDutyRate  = cap & 0xF;
            globalDutyAvail = os_getTime();
            dutyCapAns = 1;
            oidx += 2;
#endif // !DISABLE_MCMD_DCAP_REQ
            continue;
        }
        case MCMD_SNCH_REQ: {
#if !defined(DISABLE_MCMD_SNCH_REQ)
            uint8_t chidx = opts[oidx+1];  // channel
            uint32_t newfreq  = convFreq(&opts[oidx+2]); // freq
            uint8_t drs   = opts[oidx+5];  // datarate span
            snchAns = 0x80;
            if( newfreq != 0 && setupChannel(chidx, newfreq, DR_RANGE_MAP(drs&0xF,drs>>4), -1) )
                snchAns |= MCMD_SNCH_ANS_DRACK|MCMD_SNCH_ANS_FQACK;
#endif // !DISABLE_MCMD_SNCH_REQ
            oidx += 6;
            continue;
        }
        case MCMD_PING_SET: {
            oidx += 4;
            continue;
        }
        case MCMD_BCNI_ANS: {
            oidx += 4;
            continue;
        }
        }
        break;
    }
    if( oidx != olen ) {
        // corrupted frame
        // log ?
    }

    if( !replayConf ) {
        // Handle payload only if not a replay
        // Decrypt payload - if any
        if( port >= 0  &&  pend-poff > 0 )
            aes_cipher(port <= 0 ? nwkKey : artKey, devaddr, seqno, /*dn*/1, d+poff, pend-poff);
    } else {
        // replay
        // not handle
    }

    if( // NWK acks but we don't have a frame pending
        (ackup && txCnt == 0) ||
        // We sent up confirmed and we got a response in DNW1/DNW2
        // BUT it did not carry an ACK - this should never happen
        // Do not resend and assume frame was not ACKed.
        (!ackup && txCnt != 0) ) {
            //suspirious hack
    }

    if( txCnt != 0 ) // we requested an ACK
        txrxFlags |= ackup ? TXRX_ACK : TXRX_NACK;

    if( port < 0 ) {
        txrxFlags |= TXRX_NOPORT;
        dataBeg = poff;
        dataLen = 0;
    } else {
        txrxFlags |= TXRX_PORT;
        dataBeg = poff;
        dataLen = pend-poff;
    }
    PRINT_DEBUG_1("Received downlink, window=%s, port=%d, ack=%d", window, port, ackup);
    return true;
}


// ================================================================================
// TX/RX transaction support


void Lmic::setupRx2 () {
    txrxFlags = TXRX_DNW2;
    rps = dndr2rps(dn2Dr);
    freq = dn2Freq;
    dataLen = 0;
    os_radio(RADIO_RX);
}


void Lmic::schedRx12 (ostime_t delay, OsJobType<Lmic>::osjobcbTyped_t func, uint8_t dr) {
    ostime_t hsym = dr2hsym(dr);

    rxsyms = MINRX_SYMS;

    // If a clock error is specified, compensate for it by extending the
    // receive window
    if (clockError != 0) {
        // Calculate how much the clock will drift maximally after delay has
        // passed. This indicates the amount of time we can be early
        // _or_ late.
        ostime_t drift = (int64_t)delay * clockError / MAX_CLOCK_ERROR;

        // Increase the receive window by twice the maximum drift (to
        // compensate for a slow or a fast clock).
        // decrease the rxtime to compensate for. Note that hsym is a
        // *half* symbol time, so the factor 2 is hidden. First check if
        // this would overflow (which can happen if the drift is very
        // high, or the symbol time is low at high datarates).
        if ((255 - rxsyms) * hsym < drift)
            rxsyms = 255;
        else
            rxsyms += drift / hsym;
    }

    // Center the receive window on the center of the expected preamble
    // (again note that hsym is half a sumbol time, so no /2 needed)
    rxtime = txend + delay + PAMBL_SYMS * hsym - rxsyms * hsym;

    osjob.setTimedCallback2(rxtime - RX_RAMPUP, func);
}

void Lmic::setupRx1 (OsJobType<Lmic>::osjobcbTyped_t func) {
    txrxFlags = TXRX_DNW1;
    // Turn rps from TX over to RX
    rps = setNocrc(rps,1);
    dataLen = 0;
    osjob.setCallbackFuture2(func);
    os_radio(RADIO_RX);
}


// Called by HAL once TX complete and delivers exact end of TX time stamp in rxtime
void Lmic::txDone (ostime_t delay, OsJobType<Lmic>::osjobcbTyped_t func) {
    // Change RX frequency / rps (US only) before we increment txChnl
    setRx1Params();
    // rxsyms carries the TX datarate (can be != datarate [confirm retries etc.])
    // Setup receive - rxtime is preloaded with 1.5 symbols offset to tune
    // into the middle of the 8 symbols preamble.
#if defined(CFG_eu868)
    if( /* TX datarate */rxsyms == DR_FSK ) {
        rxtime = txend + delay - PRERX_FSK*us2osticksRound(160);
        rxsyms = RXLEN_FSK;
        osjob.setTimedCallback2(rxtime - RX_RAMPUP, func);
    }
    else
#endif
    {
        schedRx12(delay, func, dndr);
    }
}


// ======================================== Join frames


#if !defined(DISABLE_JOIN)
void Lmic::onJoinFailed (OsJobBase* osjob) {
    // Notify app - must call reset() to stop joining
    // otherwise join procedure continues.
    reportEvent(EV_JOIN_FAILED);
}

bool Lmic::processJoinAcceptNoJoinFrame() {
        if( (opmode & OP_JOINING) == 0 ) {
            ASSERT((opmode & OP_REJOIN) != 0);
            // REJOIN attempt for roaming
            opmode &= ~(OP_REJOIN|OP_TXRXPEND);
            if( rejoinCnt < 10 )
                rejoinCnt++;
            reportEvent(EV_REJOIN_FAILED);
            return true;
        } 
        opmode &= ~OP_TXRXPEND;
        ostime_t delay = nextJoinState();
        // Build next JOIN REQUEST with next engineUpdate call
        // Optionally, report join failed.
        // Both after a random/chosen amount of ticks.

        // this delay is suspissisous FIXME GZOGZO

        osjob.setTimedCallback2(os_getTime()+delay,
                            (delay&1) != 0
                            ? &Lmic::onJoinFailed      // one JOIN iteration done and failed
                            : &Lmic::runEngineUpdate); // next step to be delayed
        return true;
}
bool Lmic::processJoinAccept () {
    ASSERT(txrxFlags != TXRX_DNW1 || dataLen != 0);
    ASSERT((opmode & OP_TXRXPEND)!=0);

    if( dataLen == 0 ) {
      return processJoinAcceptNoJoinFrame();
    }

    uint8_t hdr  = frame[0];
    uint8_t dlen = dataLen;
    uint32_t mic  = os_rlsbf4(&frame[dlen-4]); // safe before modified by encrypt!
    if( (dlen != LEN_JA && dlen != LEN_JAEXT)
        || (hdr & (HDR_FTYPE|HDR_MAJOR)) != (HDR_FTYPE_JACC|HDR_MAJOR_V1) ) {
            //unexpected frame
        if( (txrxFlags & TXRX_DNW1) != 0 )
            return false;
        return processJoinAcceptNoJoinFrame();
    }
    aes_encrypt(frame+1, dlen-1);
    if( !aes_verifyMic0(frame, dlen-4) ) {
        //bad mic
        if( (txrxFlags & TXRX_DNW1) != 0 )
            return false;
        return processJoinAcceptNoJoinFrame();
    }

    uint32_t addr = os_rlsbf4(frame+OFF_JA_DEVADDR);
    devaddr = addr;
    netid = os_rlsbf4(&frame[OFF_JA_NETID]) & 0xFFFFFF;

#if defined(CFG_eu868)
    initDefaultChannels(false);
#endif
    if( dlen > LEN_JA ) {
#if defined(CFG_us915)
        if( (txrxFlags & TXRX_DNW1) != 0 )
            return false;
        return processJoinAcceptNoJoinFrame();
#endif
        dlen = OFF_CFLIST;
        for( uint8_t chidx=3; chidx<8; chidx++, dlen+=3 ) {
            uint32_t newfreq = convFreq(&frame[dlen]);
            if( newfreq ) {
                setupChannel(chidx, freq, 0, -1);
#if LMIC_DEBUG_LEVEL > 1
                lmic_printf("%lu: Setup channel, idx=%d, freq=%lu\n", os_getTime(), chidx, (unsigned long)newfreq);
#endif
            }
        }
    }

    // already incremented when JOIN REQ got sent off
    aes_sessKeys(devNonce-1, &frame[OFF_JA_ARTNONCE], nwkKey, artKey);

    ASSERT((opmode & (OP_JOINING|OP_REJOIN))!=0);
    if( (opmode & OP_REJOIN) != 0 ) {
        // Lower DR every try below current UP DR
        datarate = lowerDR(datarate, rejoinCnt);
    }
    opmode &= ~(OP_JOINING|OP_TRACK|OP_REJOIN|OP_TXRXPEND|OP_PINGINI) | OP_NEXTCHNL;
    txCnt = 0;
    stateJustJoined();
    dn2Dr = frame[OFF_JA_DLSET] & 0x0F;
    rxDelay = frame[OFF_JA_RXDLY];
    if (rxDelay == 0) rxDelay = 1;
    reportEvent(EV_JOINED);
    return true;
}


void Lmic::processRx2Jacc (OsJobBase* osjob) {
    if( dataLen == 0 )
        txrxFlags = 0;  // nothing in 1st/2nd DN slot
    processJoinAccept();
}


void Lmic::setupRx2Jacc (OsJobBase* osjob) {
    this->osjob.setCallbackFuture2(&Lmic::processRx2Jacc);
    setupRx2();
}


void Lmic::processRx1Jacc (OsJobBase* osjob) {
    if( dataLen == 0 || !processJoinAccept() )
        schedRx12(DELAY_JACC2_osticks, &Lmic::setupRx2Jacc, dn2Dr);
}


void Lmic::setupRx1Jacc (OsJobBase* osjob) {
    setupRx1(&Lmic::processRx1Jacc);
}


void Lmic::jreqDone (OsJobBase* osjob) {
    txDone(DELAY_JACC1_osticks, &Lmic::setupRx1Jacc);
}

#endif // !DISABLE_JOIN

// ======================================== Data frames

void Lmic::processRx2DnData (OsJobBase* osjob) {
    if( dataLen == 0 ) {
        txrxFlags = 0;  // nothing in 1st/2nd DN slot
        // It could be that the gateway *is* sending a reply, but we
        // just didn't pick it up. To avoid TX'ing again while the
        // gateay is not listening anyway, delay the next transmission
        // until DNW2_SAFETY_ZONE from now, and add up to 2 seconds of
        // extra randomization.
        txDelay(os_getTime() + DNW2_SAFETY_ZONE, 2);
    }
    processDnData();
}


void Lmic::setupRx2DnData (OsJobBase* osjob) {
    this->osjob.setCallbackFuture2(&Lmic::processRx2DnData);
    setupRx2();
}


void Lmic::processRx1DnData (OsJobBase* osjob) {
    if( dataLen == 0 || !processDnData() )
        schedRx12(sec2osticks(rxDelay +(int)DELAY_EXTDNW2), &Lmic::setupRx2DnData, dn2Dr);
}


void Lmic::setupRx1DnData (OsJobBase* osjob) {
    setupRx1(&Lmic::processRx1DnData);
}


void Lmic::updataDone (OsJobBase* osjob) {
    txDone(sec2osticks(rxDelay), &Lmic::setupRx1DnData);
}

// ========================================


void Lmic::buildDataFrame () {
    bool txdata = ((opmode & (OP_TXDATA|OP_POLL)) != OP_POLL);
    uint8_t dlen = txdata ? pendTxLen : 0;

    // Piggyback MAC options
    // Prioritize by importance
    int  end = OFF_DAT_OPTS;
#if !defined(DISABLE_MCMD_DCAP_REQ)
    if( dutyCapAns ) {
        frame[end] = MCMD_DCAP_ANS;
        end += 1;
        dutyCapAns = 0;
    }
#endif // !DISABLE_MCMD_DCAP_REQ
#if !defined(DISABLE_MCMD_DN2P_SET)
    if( dn2Ans ) {
        frame[end+0] = MCMD_DN2P_ANS;
        frame[end+1] = dn2Ans & ~MCMD_DN2P_ANS_RFU;
        end += 2;
        dn2Ans = 0;
    }
#endif // !DISABLE_MCMD_DN2P_SET
    if( devsAns ) {  // answer to device status
        frame[end+0] = MCMD_DEVS_ANS;
        frame[end+1] = os_getBattLevel();
        frame[end+2] = margin;
        end += 3;
        devsAns = 0;
    }
    if( ladrAns ) {  // answer to ADR change
        frame[end+0] = MCMD_LADR_ANS;
        frame[end+1] = ladrAns & ~MCMD_LADR_ANS_RFU;
        end += 2;
        ladrAns = 0;
    }
    if( adrChanged ) {
        if( adrAckReq < 0 )
            adrAckReq = 0;
        adrChanged = 0;
    }
#if !defined(DISABLE_MCMD_SNCH_REQ)
    if( snchAns ) {
        frame[end+0] = MCMD_SNCH_ANS;
        frame[end+1] = snchAns & ~MCMD_SNCH_ANS_RFU;
        end += 2;
        snchAns = 0;
    }
#endif // !DISABLE_MCMD_SNCH_REQ
    ASSERT(end <= OFF_DAT_OPTS+16);

    uint8_t flen = end + (txdata ? 5+dlen : 4);
    if( flen > MAX_LEN_FRAME ) {
        // Options and payload too big - delay payload
        txdata = 0;
        flen = end+4;
    }
    frame[OFF_DAT_HDR] = HDR_FTYPE_DAUP | HDR_MAJOR_V1;
    frame[OFF_DAT_FCT] = (dnConf | adrEnabled
                              | (adrAckReq >= 0 ? FCT_ADRARQ : 0)
                              | (end-OFF_DAT_OPTS));
    os_wlsbf4(frame+OFF_DAT_ADDR,  devaddr);

    if( txCnt == 0 ) {
        seqnoUp += 1;
    } else {
    }
    os_wlsbf2(frame+OFF_DAT_SEQNO, seqnoUp-1);

    // Clear pending DN confirmation
    dnConf = 0;

    if( txdata ) {
        if( pendTxConf ) {
            // Confirmed only makes sense if we have a payload (or at least a port)
            frame[OFF_DAT_HDR] = HDR_FTYPE_DCUP | HDR_MAJOR_V1;
            if( txCnt == 0 ) txCnt = 1;
        }
        frame[end] = pendTxPort;
        os_copyMem(frame+end+1, pendTxData, dlen);
        aes_cipher(pendTxPort==0 ? nwkKey : artKey,
                   devaddr, seqnoUp-1,
                   /*up*/0, frame+end+1, dlen);
    }
    aes_appendMic(nwkKey, devaddr, seqnoUp-1, /*up*/0, frame, flen-4);

    dataLen = flen;
}


// ================================================================================
//
// Join stuff
//
// ================================================================================

#if !defined(DISABLE_JOIN)
void Lmic::buildJoinRequest (uint8_t ftype) {
    // Do not use pendTxData since we might have a pending
    // user level frame in there. Use RX holding area instead.
    xref2uint8_t d = frame;
    d[OFF_JR_HDR] = ftype;
    os_getArtEui(d + OFF_JR_ARTEUI);
    os_getDevEui(d + OFF_JR_DEVEUI);
    os_wlsbf2(d + OFF_JR_DEVNONCE, devNonce);
    aes_appendMic0(d, OFF_JR_MIC);

    dataLen = LEN_JR;
    devNonce++;
}

void Lmic::startJoining (OsJobBase* osjob) {
    reportEvent(EV_JOINING);
}

// Start join procedure if not already joined.
bool Lmic::startJoining () {
    if( devaddr == 0 ) {
        // There should be no TX/RX going on
        ASSERT((opmode & (OP_POLL|OP_TXRXPEND)) == 0);
        // Lift any previous duty limitation
        globalDutyRate = 0;
        // Cancel scanning
        opmode &= ~(OP_SCAN|OP_REJOIN|OP_LINKDEAD|OP_NEXTCHNL);
        // Setup state
        rejoinCnt = txCnt = 0;
        initJoinLoop();
        opmode |= OP_JOINING;
        // reportEvent will call engineUpdate which then starts sending JOIN REQUESTS
        osjob.setCallbackRunnable2(&Lmic::startJoining);
        return true;
    }
    return false; // already joined
}
#endif // !DISABLE_JOIN


// ================================================================================
//
//
//
// ================================================================================



bool Lmic::processDnData () {
    ASSERT((opmode & OP_TXRXPEND)!=0);

    if( dataLen == 0 ) {
      norx:
        if( txCnt != 0 ) {
            if( txCnt < TXCONF_ATTEMPTS ) {
                txCnt += 1;
                setDrTxpow(lowerDR(datarate, TABLE_GET_U1(DRADJUST, txCnt)), KEEP_TXPOW);
                // Schedule another retransmission
                txDelay(rxtime, RETRY_PERIOD_secs);
                opmode &= ~OP_TXRXPEND;
                engineUpdate();
                return 1;
            }
            txrxFlags = TXRX_NACK | TXRX_NOPORT;
        } else {
            // Nothing received - implies no port
            txrxFlags = TXRX_NOPORT;
        }
        if( adrAckReq != LINK_CHECK_OFF )
            adrAckReq += 1;
        dataBeg = dataLen = 0;
      txcomplete:
        opmode &= ~(OP_TXDATA|OP_TXRXPEND);
        if( (txrxFlags & (TXRX_DNW1|TXRX_DNW2|TXRX_PING)) != 0  &&  (opmode & OP_LINKDEAD) != 0 ) {
            opmode &= ~OP_LINKDEAD;
            reportEvent(EV_LINK_ALIVE);
        }
        reportEvent(EV_TXCOMPLETE);
        // If we haven't heard from NWK in a while although we asked for a sign
        // assume link is dead - notify application and keep going
        if( adrAckReq > LINK_CHECK_DEAD ) {
            // We haven't heard from NWK for some time although we
            // asked for a response for some time - assume we're disconnected. Lower DR one notch.
            setDrTxpow(decDR(datarate), KEEP_TXPOW);
            adrAckReq = LINK_CHECK_CONT;
            opmode |= OP_REJOIN|OP_LINKDEAD;
            reportEvent(EV_LINK_DEAD);
        }
        return 1;
    }
    if( !decodeFrame() ) {
        if( (txrxFlags & TXRX_DNW1) != 0 )
            return 0;
        goto norx;
    }
    goto txcomplete;
}

// Decide what to do next for the MAC layer of a device
void Lmic::engineUpdate () {
#if LMIC_DEBUG_LEVEL > 0
    lmic_printf("%lu: engineUpdate, opmode=0x%x\n", os_getTime(), opmode);
#endif
    // Check for ongoing state: scan or TX/RX transaction
    if( (opmode & (OP_SCAN|OP_TXRXPEND|OP_SHUTDOWN)) != 0 )
        return;

#if !defined(DISABLE_JOIN)
    if( devaddr == 0 && (opmode & OP_JOINING) == 0 ) {
        startJoining();
        return;
    }
#endif // !DISABLE_JOIN

    ostime_t now    = os_getTime();
    ostime_t txbeg  = 0;

    if( (opmode & (OP_JOINING|OP_REJOIN|OP_TXDATA|OP_POLL)) != 0 ) {
        // Need to TX some data...
        // Assuming txChnl points to channel which first becomes available again.
        bool jacc = ((opmode & (OP_JOINING|OP_REJOIN)) != 0 ? 1 : 0);
        #if LMIC_DEBUG_LEVEL > 1
            if (jacc)
                lmic_printf("%lu: Uplink join pending\n", os_getTime());
            else
                lmic_printf("%lu: Uplink data pending\n", os_getTime());
        #endif
        // Find next suitable channel and return availability time
        if( (opmode & OP_NEXTCHNL) != 0 ) {
            txbeg = txend = nextTx(now);
            opmode &= ~OP_NEXTCHNL;
            #if LMIC_DEBUG_LEVEL > 1
                lmic_printf("%lu: Airtime available at %lu (channel duty limit)\n", os_getTime(), txbeg);
            #endif
        } else {
            txbeg = txend;
            #if LMIC_DEBUG_LEVEL > 1
                lmic_printf("%lu: Airtime available at %lu (previously determined)\n", os_getTime(), txbeg);
            #endif
        }
        // Delayed TX or waiting for duty cycle?
        if( (globalDutyRate != 0 || (opmode & OP_RNDTX) != 0)  &&  (txbeg - globalDutyAvail) < 0 ) {
            txbeg = globalDutyAvail;
            #if LMIC_DEBUG_LEVEL > 1
                lmic_printf("%lu: Airtime available at %lu (global duty limit)\n", os_getTime(), txbeg);
            #endif
        }
        // Earliest possible time vs overhead to setup radio
        if( txbeg - (now + TX_RAMPUP) < 0 ) {
            #if LMIC_DEBUG_LEVEL > 1
                lmic_printf("%lu: Ready for uplink\n", os_getTime());
            #endif
            // We could send right now!
            txbeg = now;
            dr_t txdr = (dr_t)datarate;
#if !defined(DISABLE_JOIN)
            if( jacc ) {
                uint8_t ftype;
                if( (opmode & OP_REJOIN) != 0 ) {
                    txdr = lowerDR(txdr, rejoinCnt);
                    ftype = HDR_FTYPE_REJOIN;
                } else {
                    ftype = HDR_FTYPE_JREQ;
                }
                buildJoinRequest(ftype);
                osjob.setCallbackFuture2(&Lmic::jreqDone);
            } else
#endif // !DISABLE_JOIN
            {
                if( seqnoDn >= 0xFFFFFF80 ) {
                    // Imminent roll over - proactively reset MAC
                    // Device has to react! NWK will not roll over and just stop sending.
                    // Thus, we have N frames to detect a possible lock up.
                    osjob.setCallbackRunnable2(&Lmic::runReset);
                    return;
                }
                if( (txCnt==0 && seqnoUp == 0xFFFFFFFF) ) {
                    // Roll over of up seq counter
                    // Do not run RESET event callback from here!
                    // App code might do some stuff after send unaware of RESET.
                    osjob.setCallbackRunnable2(&Lmic::runReset);
                    return;
                }
                buildDataFrame();
                osjob.setCallbackFuture2(&Lmic::updataDone);
            }
            rps    = setCr(updr2rps(txdr), (cr_t)errcr);
            dndr   = txdr;  // carry TX datarate (can be != datarate) over to txDone/setupRx1
            opmode = (opmode & ~(OP_POLL|OP_RNDTX)) | OP_TXRXPEND | OP_NEXTCHNL;
            updateTx(txbeg);
            os_radio(RADIO_TX);
            return;
        }
        #if LMIC_DEBUG_LEVEL > 1
            lmic_printf("%lu: Uplink delayed until %lu\n", os_getTime(), txbeg);
        #endif
        // Cannot yet TX
        if( (opmode & OP_TRACK) == 0 )
            goto txdelay; // We don't track the beacon - nothing else to do - so wait for the time to TX
        // Consider RX tasks
        if( txbeg == 0 ) // zero indicates no TX pending
            txbeg += 1;  // TX delayed by one tick (insignificant amount of time)
    } else {
        // No TX pending - no scheduled RX
        if( (opmode & OP_TRACK) == 0 )
            return;
    }

  txdelay:
    osjob.setTimedCallback2(txbeg-TX_RAMPUP, &Lmic::runEngineUpdate);
}


void Lmic::setAdrMode (bool enabled) {
    adrEnabled = enabled ? FCT_ADREN : 0;
}

void Lmic::shutdown () {
    osjob.clearCallback();
    os_radio(RADIO_RST);
    opmode |= OP_SHUTDOWN;
}


void Lmic::reset () {
    os_radio(RADIO_RST);
    osjob.clearCallback();

    // TODO proper reset
   // os_clearMem((xref2uint8_t)&LMIC,SIZEOFEXPR(LMIC));
    devaddr      =  0;
    devNonce     =  os_getRndU2();
    opmode       =  OP_NONE;
    errcr        =  CR_4_5;
    adrEnabled   =  FCT_ADREN;
    dn2Dr        =  DR_DNW2;   // we need this for 2nd DN window of join accept
    dn2Freq      =  FREQ_DNW2; // ditto
    rxDelay      =  DELAY_DNW1;
#if defined(CFG_us915)
    initDefaultChannels();
#endif
}


void Lmic::init (void) {
    opmode = OP_SHUTDOWN;
}


void Lmic::clrTxData (void) {
    opmode &= ~(OP_TXDATA|OP_TXRXPEND|OP_POLL);
    pendTxLen = 0;
    if( (opmode & (OP_JOINING|OP_SCAN)) != 0 ) // do not interfere with JOINING
        return;
    osjob.clearCallback();
    os_radio(RADIO_RST);
    engineUpdate();
}

void Lmic::setTxData (void) {
    opmode |= OP_TXDATA;
    if( (opmode & OP_JOINING) == 0 )
        txCnt = 0;             // cancel any ongoing TX/RX retries
    engineUpdate();
}

//
int Lmic::setTxData2 (uint8_t port, xref2uint8_t data, uint8_t dlen, uint8_t confirmed) {
    if( dlen > SIZEOFEXPR(pendTxData) )
        return -2;
    if( data != (xref2uint8_t)0 )
        os_copyMem(pendTxData, data, dlen);
    pendTxConf = confirmed;
    pendTxPort = port;
    pendTxLen  = dlen;
    setTxData();
    return 0;
}


// Send a payload-less message to signal device is alive
void Lmic::sendAlive () {
    opmode |= OP_POLL;
    engineUpdate();
}


// Check if other networks are around.
void Lmic::tryRejoin (void) {
    opmode |= OP_REJOIN;
    engineUpdate();
}

//! \brief Setup given session keys
//! and put the MAC in a state as if
//! a join request/accept would have negotiated just these keys.
//! It is crucial that the combinations `devaddr/nwkkey` and `devaddr/artkey`
//! are unique within the network identified by `netid`.
//! NOTE: on Harvard architectures when session keys are in flash:
//!  Caller has to fill in {nwk,art}Key  before and pass {nwk,art}Key are NULL
//! \param netid a 24 bit number describing the network id this device is using
//! \param devaddr the 32 bit session address of the device. It is strongly recommended
//!    to ensure that different devices use different numbers with high probability.
//! \param nwkKey  the 16 byte network session key used for message integrity.
//!     If NULL the caller has copied the key into `nwkKey` before.
//! \param artKey  the 16 byte application router session key used for message confidentiality.
//!     If NULL the caller has copied the key into `artKey` before.
void Lmic::setSession (uint32_t netid, devaddr_t devaddr, xref2uint8_t nwkKey, xref2uint8_t artKey) {
    this->netid = netid;
    this->devaddr = devaddr;
    if( nwkKey != (xref2uint8_t)0 )
        os_copyMem(this->nwkKey, nwkKey, 16);
    if( artKey != (xref2uint8_t)0 )
        os_copyMem(this->artKey, artKey, 16);

#if defined(CFG_eu868)
    initDefaultChannels(false);
#endif

    opmode &= ~(OP_JOINING|OP_TRACK|OP_REJOIN|OP_TXRXPEND|OP_PINGINI);
    opmode |= OP_NEXTCHNL;
    stateJustJoined();
}

// Enable/disable link check validation.
// LMIC sets the ADRACKREQ bit in UP frames if there were no DN frames
// for a while. It expects the network to provide a DN message to prove
// connectivity with a span of UP frames. If this no such prove is coming
// then the datarate is lowered and a LINK_DEAD event is generated.
// This mode can be disabled and no connectivity prove (ADRACKREQ) is requested
// nor is the datarate changed.
// This must be called only if a session is established (e.g. after EV_JOINED)
void Lmic::setLinkCheckMode (bool enabled) {
    adrChanged = 0;
    adrAckReq = enabled ? LINK_CHECK_INIT : LINK_CHECK_OFF;
}

// Sets the max clock error to compensate for (defaults to 0, which
// allows for +/- 640 at SF7BW250). MAX_CLOCK_ERROR represents +/-100%,
// so e.g. for a +/-1% error you would pass MAX_CLOCK_ERROR * 1 / 100.
void Lmic::setClockError(uint16_t error) {
    clockError = error;
}

void Lmic::nextTask() {
    osjob.setRunnable();
}
