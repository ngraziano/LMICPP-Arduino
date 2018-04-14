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

//! @file
//! @brief LMIC API

#ifndef _lmic_h_
#define _lmic_h_

#include "oslmic.h"
#include "lorabase.h"


// LMIC version
#define LMIC_VERSION_MAJOR 1
#define LMIC_VERSION_MINOR 5
#define LMIC_VERSION_BUILD 1431528305

enum { MAX_FRAME_LEN      =  64 };   //!< Library cap on max frame length
enum { TXCONF_ATTEMPTS    =   8 };   //!< Transmit attempts for confirmed frames
enum { MAX_MISSED_BCNS    =  20 };   // threshold for triggering rejoin requests
enum { MAX_RXSYMS         = 100 };   // stop tracking beacon beyond this

enum { LINK_CHECK_CONT    =  12 ,    // continue with this after reported dead link
       LINK_CHECK_DEAD    =  24 ,    // after this UP frames and no response from NWK assume link is dead
       LINK_CHECK_INIT    = -12 ,    // UP frame count until we inc datarate
       LINK_CHECK_OFF     =-128 };   // link check disabled

enum { TIME_RESYNC        = 6*128 }; // secs
enum { TXRX_GUARD_ms      =  6000 };  // msecs - don't start TX-RX transaction before beacon
enum { JOIN_GUARD_ms      =  9000 };  // msecs - don't start Join Req/Acc transaction before beacon
enum { TXRX_BCNEXT_secs   =     2 };  // secs - earliest start after beacon time
enum { RETRY_PERIOD_secs  =     3 };  // secs - random period for retrying a confirmed send

#if defined(CFG_eu868) // EU868 spectrum ====================================================

enum { MAX_CHANNELS = 16 };      //!< Max supported channels
enum { MAX_BANDS    =  4 };

enum { LIMIT_CHANNELS = (1<<4) };   // EU868 will never have more channels
//! \internal
struct band_t {
    uint16_t     txcap;     // duty cycle limitation: 1/txcap
    int8_t     txpow;     // maximum TX power
    uint8_t     lastchnl;  // last used channel
    ostime_t avail;     // channel is blocked until this time
};
TYPEDEF_xref2band_t; //!< \internal

#elif defined(CFG_us915)  // US915 spectrum =================================================

enum { MAX_XCHANNELS = 2 };      // extra channels in RAM, channels 0-71 are immutable
enum { MAX_TXPOW_125kHz = 30 };

#endif // ==========================================================================

// Keep in sync with evdefs.hpp::drChange
enum { DRCHG_SET, DRCHG_NOJACC, DRCHG_NOACK, DRCHG_NOADRACK, DRCHG_NWKCMD };
enum { KEEP_TXPOW = -128 };

// purpose of receive window - lmic_t.rxState
enum { RADIO_RST=0, RADIO_TX=1, RADIO_RX=2, RADIO_RXON=3 };
// Netid values /  lmic_t.netid
enum { NETID_NONE=(int)~0U, NETID_MASK=(int)0xFFFFFF };
// MAC operation modes (lmic_t.opmode).
enum { OP_NONE     = 0x0000,
       OP_SCAN     = 0x0001, // radio scan to find a beacon
       OP_TRACK    = 0x0002, // track my networks beacon (netid)
       OP_JOINING  = 0x0004, // device joining in progress (blocks other activities)
       OP_TXDATA   = 0x0008, // TX user data (buffered in pendTxData)
       OP_POLL     = 0x0010, // send empty UP frame to ACK confirmed DN/fetch more DN data
       OP_REJOIN   = 0x0020, // occasionally send JOIN REQUEST
       OP_SHUTDOWN = 0x0040, // prevent MAC from doing anything
       OP_TXRXPEND = 0x0080, // TX/RX transaction pending
       OP_RNDTX    = 0x0100, // prevent TX lining up after a beacon
       OP_PINGINI  = 0x0200, // pingable is initialized and scheduling active
       OP_PINGABLE = 0x0400, // we're pingable
       OP_NEXTCHNL = 0x0800, // find a new channel
       OP_LINKDEAD = 0x1000, // link was reported as dead
       OP_TESTMODE = 0x2000, // developer test mode
};
// TX-RX transaction flags - report back to user
enum { TXRX_ACK    = 0x80,   // confirmed UP frame was acked
       TXRX_NACK   = 0x40,   // confirmed UP frame was not acked
       TXRX_NOPORT = 0x20,   // set if a frame with a port was RXed, clr if no frame/no port
       TXRX_PORT   = 0x10,   // set if a frame with a port was RXed, LMIC.frame[LMIC.dataBeg-1] => port
       TXRX_DNW1   = 0x01,   // received in 1st DN slot
       TXRX_DNW2   = 0x02,   // received in 2dn DN slot
       TXRX_PING   = 0x04 }; // received in a scheduled RX slot
// Event types for event callback
enum _ev_t { EV_SCAN_TIMEOUT=1, EV_BEACON_FOUND,
             EV_BEACON_MISSED, EV_BEACON_TRACKED, EV_JOINING,
             EV_JOINED, EV_RFU1, EV_JOIN_FAILED, EV_REJOIN_FAILED,
             EV_TXCOMPLETE, EV_LOST_TSYNC, EV_RESET,
             EV_RXCOMPLETE, EV_LINK_DEAD, EV_LINK_ALIVE };
typedef enum _ev_t ev_t;

enum {
        // This value represents 100% error in LMIC.clockError
        MAX_CLOCK_ERROR = 65536,
};

#if defined(CFG_eu868)
enum { BAND_MILLI=0, BAND_CENTI=1, BAND_DECI=2, BAND_AUX=3 };
#endif

class Lmic {
public:
    // Radio settings TX/RX (also accessed by HAL)
    ostime_t    txend;
    ostime_t    rxtime;
    uint32_t        freq;
    int8_t        rssi;
    int8_t        snr;
    rps_t       rps;
    uint8_t        rxsyms;
    uint8_t        dndr;
    int8_t        txpow;     // dBm

    OsJobType<Lmic>     osjob {  this, OSS };

    // Channel scheduling
#if defined(CFG_eu868)
    band_t      bands[MAX_BANDS];
    uint32_t        channelFreq[MAX_CHANNELS];
    uint16_t        channelDrMap[MAX_CHANNELS];
    uint16_t        channelMap;
#elif defined(CFG_us915)
    uint32_t        xchFreq[MAX_XCHANNELS];    // extra channel frequencies (if device is behind a repeater)
    uint16_t        xchDrMap[MAX_XCHANNELS];   // extra channel datarate ranges  ---XXX: ditto
    uint16_t        channelMap[(72+MAX_XCHANNELS+15)/16];  // enabled bits
    uint16_t        chRnd;        // channel randomizer
#endif
    uint8_t        txChnl;          // channel for next TX
    uint8_t        globalDutyRate;  // max rate: 1/2^k
    ostime_t    globalDutyAvail; // time device can send again

    uint32_t        netid;        // current network id (~0 - none)
    uint16_t        opmode;
    uint8_t        upRepeat;     // configured up repeat
    int8_t        adrTxPow;     // ADR adjusted TX power
    dr_t        datarate;     // current data rate
    uint8_t        errcr;        // error coding rate (used for TX only)
    uint8_t        rejoinCnt;    // adjustment for rejoin datarate

    uint16_t        clockError; // Inaccuracy in the clock. CLOCK_ERROR_MAX
                            // represents +/-100% error

    uint8_t        pendTxPort;
    uint8_t        pendTxConf;   // confirmed data
    uint8_t        pendTxLen;    // +0x80 = confirmed
    uint8_t        pendTxData[MAX_LEN_PAYLOAD];

    uint16_t        devNonce;     // last generated nonce
    uint8_t        nwkKey[16];   // network session key
    uint8_t        artKey[16];   // application router session key
    devaddr_t   devaddr;
    uint32_t        seqnoDn;      // device level down stream seqno
    uint32_t        seqnoUp;

    uint8_t        dnConf;       // dn frame confirm pending: LORA::FCT_ACK or 0
    int8_t        adrAckReq;    // counter until we reset data rate (0=off)
    uint8_t        adrChanged;

    uint8_t        rxDelay;      // Rx delay after TX
    
    uint8_t        margin;
    bool       ladrAns;      // link adr adapt answer pending
    bool       devsAns;      // device status answer pending
    uint8_t        adrEnabled;
    uint8_t        moreData;     // NWK has more data pending
#if !defined(DISABLE_MCMD_DCAP_REQ)
    bool       dutyCapAns;   // have to ACK duty cycle settings
#endif
#if !defined(DISABLE_MCMD_SNCH_REQ)
    uint8_t        snchAns;      // answer set new channel
#endif
    // 2nd RX window (after up stream)
    uint8_t        dn2Dr;
    uint32_t        dn2Freq;
#if !defined(DISABLE_MCMD_DN2P_SET)
    uint8_t        dn2Ans;       // 0=no answer pend, 0x80+ACKs
#endif


    // Public part of MAC state
    uint8_t        txCnt;
    uint8_t        txrxFlags;  // transaction flags (TX-RX combo)
    uint8_t        dataBeg;    // 0 or start of data (dataBeg-1 is port)
    uint8_t        dataLen;    // 0 no data or zero length data, >0 byte count of data
    uint8_t        frame[MAX_LEN_FRAME];

private:


    // callbacks

    void processRx1DnData (OsJob* osjob);
    void setupRx1 (OsJobType<Lmic>::osjobcbTyped_t func);
    void setupRx2 ();
    void schedRx12 (ostime_t delay, OsJobType<Lmic>::osjobcbTyped_t func, uint8_t dr);
    
    void txDone (ostime_t delay, OsJobType<Lmic>::osjobcbTyped_t func);


    void runReset (OsJob* osjob);
    void runEngineUpdate (OsJob* osjob);

    #if !defined(DISABLE_JOIN)
    void onJoinFailed (OsJob* osjob);
    bool processJoinAcceptNoJoinFrame();
    bool processJoinAccept();
    void processRx1Jacc (OsJob* osjob);
    void processRx2Jacc (OsJob* osjob);
    void setupRx1Jacc (OsJob* osjob);
    void setupRx2Jacc (OsJob* osjob);
    void jreqDone (OsJob* osjob);
    void startJoining (OsJob* osjob);

    void buildJoinRequest (uint8_t ftype);
    
    #endif

    void processRx2DnData (OsJob* osjob);

    void setupRx1DnData (OsJob* osjob);
    void setupRx2DnData (OsJob* osjob);
    
    void updataDone (OsJob* osjob);
    


    void stateJustJoined();
    

    void reportEvent (ev_t ev);
    
    void buildDataFrame ();
    void engineUpdate();
    bool decodeFrame ();
    bool processDnData();

    #if defined(CFG_us915)
    void initDefaultChannels ();
    #endif
    #if defined(CFG_eu868)
    void initDefaultChannels (bool join);
    #endif
    
    uint8_t mapChannels (uint8_t chpage, uint16_t chmap);
    void updateTx (ostime_t txbeg);

    // GZO Maybe public
    ostime_t nextTx (ostime_t now);
    
    void setRx1Params();

    void txDelay (ostime_t reftime, uint8_t secSpan);
    
    void setDrJoin (dr_t dr);
    
    #if !defined(DISABLE_JOIN)
    void initJoinLoop ();
    ostime_t nextJoinState (void);
    #endif
public:
    // set default/start DR/txpow
    void setDrTxpow (uint8_t dr, int8_t pow);
    void setLinkCheckMode (bool enabled);
    void setSession (uint32_t netid, devaddr_t devaddr, xref2uint8_t nwkKey, xref2uint8_t artKey);

    bool setupChannel (uint8_t channel, uint32_t freq, uint16_t drmap, int8_t band);
    void disableChannel (uint8_t channel);

    // set ADR mode (if mobile turn off)
    void  setAdrMode   (bool enabled);        
    
    #if defined(CFG_us915)
    void  enableChannel (uint8_t channel);
    void  enableSubBand (uint8_t band);
    void  disableSubBand (uint8_t band);
    void  selectSubBand (uint8_t band);
    #endif

    #if defined(CFG_eu868)
    bool setupBand (uint8_t bandidx, int8_t txpow, uint16_t txcap);
    #endif

    #if !defined(DISABLE_JOIN)
    bool startJoining ();
    void tryRejoin();

    #endif

    void init();
    void shutdown();
    void reset();
    
    void clrTxData();
    void setTxData();
    int setTxData2(uint8_t port, xref2uint8_t data, uint8_t dlen, uint8_t confirmed);
    void sendAlive();
    void setClockError(uint16_t error);
};
//! \var struct lmic_t LMIC
//! The state of LMIC MAC layer is encapsulated in this variable.
extern struct Lmic LMIC; //!< \internal

//! Construct a bit map of allowed datarates from drlo to drhi (both included).
#define DR_RANGE_MAP(drlo,drhi) (((uint16_t)0xFFFF<<(drlo)) & ((uint16_t)0xFFFF>>(15-(drhi))))


// Declare onEvent() function, to make sure any definition will have the
// C conventions, even when in a C++ file.
DECL_ON_LMIC_EVENT;



#endif // _lmic_h_
