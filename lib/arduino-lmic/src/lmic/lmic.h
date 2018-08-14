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

#include "../aes/aes.h"
#include "lorabase.h"
#include "oslmic.h"

// LMIC version
#define LMIC_VERSION_MAJOR 1
#define LMIC_VERSION_MINOR 5
#define LMIC_VERSION_BUILD 1431528305

enum { MAX_FRAME_LEN = 64 };   //!< Library cap on max frame length
enum { TXCONF_ATTEMPTS = 8 };  //!< Transmit attempts for confirmed frames
enum { MAX_MISSED_BCNS = 20 }; // threshold for triggering rejoin requests
enum { MAX_RXSYMS = 100 };     // stop tracking beacon beyond this

enum {
  LINK_CHECK_CONT = 12, // continue with this after reported dead link
  LINK_CHECK_DEAD =
      24, // after this UP frames and no response from NWK assume link is dead
  LINK_CHECK_INIT = -12, // UP frame count until we inc datarate
  LINK_CHECK_OFF = -128
}; // link check disabled

enum { TIME_RESYNC = 6 * 128 }; // secs
enum {
  TXRX_GUARD_ms = 6000
}; // msecs - don't start TX-RX transaction before beacon
enum {
  JOIN_GUARD_ms = 9000
}; // msecs - don't start Join Req/Acc transaction before beacon
enum { TXRX_BCNEXT_secs = 2 }; // secs - earliest start after beacon time
enum {
  RETRY_PERIOD_secs = 3
}; // secs - random period for retrying a confirmed send

#if defined(CFG_eu868) // EU868 spectrum
                       // ====================================================

enum { MAX_CHANNELS = 16 }; //!< Max supported channels
enum { MAX_BANDS = 4 };

enum { LIMIT_CHANNELS = (1 << 4) }; // EU868 will never have more channels
//! \internal
struct band_t {
  uint16_t txcap;   // duty cycle limitation: 1/txcap
  int8_t txpow;     // maximum TX power
  uint8_t lastchnl; // last used channel
  OsTime avail;     // channel is blocked until this time
};

#elif defined(CFG_us915) // US915 spectrum
                         // =================================================

enum {
  MAX_XCHANNELS = 2
}; // extra channels in RAM, channels 0-71 are immutable
enum { MAX_TXPOW_125kHz = 30 };

#endif // ==========================================================================

// Keep in sync with evdefs.hpp::drChange
enum { DRCHG_SET, DRCHG_NOJACC, DRCHG_NOACK, DRCHG_NOADRACK, DRCHG_NWKCMD };
enum { KEEP_TXPOW = -128 };

// Netid values /  lmic_t.netid
enum { NETID_NONE = (int)~0U, NETID_MASK = (int)0xFFFFFF };
// MAC operation modes (lmic_t.opmode).
enum {
  OP_NONE = 0x0000,
  OP_SCAN = 0x0001,    // radio scan to find a beacon
  OP_TRACK = 0x0002,   // track my networks beacon (netid)
  OP_JOINING = 0x0004, // device joining in progress (blocks other activities)
  OP_TXDATA = 0x0008,  // TX user data (buffered in pendTxData)
  OP_POLL =
      0x0010, // send empty UP frame to ACK confirmed DN/fetch more DN data
  OP_REJOIN = 0x0020,   // occasionally send JOIN REQUEST
  OP_SHUTDOWN = 0x0040, // prevent MAC from doing anything
  OP_TXRXPEND = 0x0080, // TX/RX transaction pending
  OP_RNDTX = 0x0100,    // prevent TX lining up after a beacon
  OP_PINGINI = 0x0200,  // pingable is initialized and scheduling active
  OP_PINGABLE = 0x0400, // we're pingable
  OP_NEXTCHNL = 0x0800, // find a new channel
  OP_LINKDEAD = 0x1000, // link was reported as dead
  OP_TESTMODE = 0x2000, // developer test mode
};
// TX-RX transaction flags - report back to user
enum {
  TXRX_ACK = 0x80,  // confirmed UP frame was acked
  TXRX_NACK = 0x40, // confirmed UP frame was not acked
  TXRX_NOPORT =
      0x20, // set if a frame with a port was RXed, clr if no frame/no port
  TXRX_PORT = 0x10, // set if a frame with a port was RXed,
                    // LMIC.frame[LMIC.dataBeg-1] => port
  TXRX_DNW1 = 0x01, // received in 1st DN slot
  TXRX_DNW2 = 0x02, // received in 2dn DN slot
  TXRX_PING = 0x04
}; // received in a scheduled RX slot
// Event types for event callback
enum _ev_t {
  EV_SCAN_TIMEOUT = 1,
  EV_BEACON_FOUND,
  EV_BEACON_MISSED,
  EV_BEACON_TRACKED,
  EV_JOINING,
  EV_JOINED,
  EV_RFU1,
  EV_JOIN_FAILED,
  EV_REJOIN_FAILED,
  EV_TXCOMPLETE,
  EV_LOST_TSYNC,
  EV_RESET,
  EV_RXCOMPLETE,
  EV_LINK_DEAD,
  EV_LINK_ALIVE
};
typedef enum _ev_t ev_t;

using eventCallback_t = void (*)(ev_t);
using keyCallback_t = void (*)(uint8_t *);
enum {
  // This value represents 100% error in LMIC.clockError
  MAX_CLOCK_ERROR = 65536,
};

#if defined(CFG_eu868)
enum { BAND_MILLI = 0, BAND_CENTI = 1, BAND_DECI = 2, BAND_AUX = 3 };
#endif

struct ChannelDetail {
  // three low bit of freq is used to store band.
  uint32_t freq;
  uint16_t drMap;
};

class Lmic {
public:
  Aes aes;
  // Radio settings TX/RX (also accessed by HAL)
  OsTime txend;
  OsTime rxtime;
  uint32_t freq = 0;
  int8_t rssi = 0;
  int8_t snr = 0;
  rps_t rps = 0;
  uint8_t rxsyms = 0;
  uint8_t dndr = 0;
  int8_t txpow = 0; // dBm

private:
  OsJobType<Lmic> osjob{this, OSS};
  eventCallback_t eventCallBack = nullptr;
  keyCallback_t devEuiCallBack = nullptr;
  keyCallback_t artEuiCallBack = nullptr;

  // Channel scheduling
#if defined(CFG_eu868)
  band_t bands[MAX_BANDS]{};
  ChannelDetail channels[MAX_CHANNELS] = {};
  uint16_t channelMap = 0;
#elif defined(CFG_us915)
  uint32_t xchFreq[MAX_XCHANNELS]; // extra channel frequencies (if device is
                                   // behind a repeater)
  uint16_t
      xchDrMap[MAX_XCHANNELS]; // extra channel datarate ranges  ---XXX: ditto
  uint16_t channelMap[(72 + MAX_XCHANNELS + 15) / 16]; // enabled bits
  uint16_t chRnd;                                      // channel randomizer
#endif
  uint8_t txChnl = 0;         // channel for next TX
  uint8_t globalDutyRate = 0; // max rate: 1/2^k
  OsTime globalDutyAvail;     // time device can send again

  uint32_t netid; // current network id (~0 - none)
  // curent opmode set at init
  uint16_t opmode;
  // configured up repeat for unconfirmed message, reset after join.
  // Not handle properly  cf: LoRaWAN™ Specification §5.2
  uint8_t upRepeat;  
  // ADR adjusted TX power not used ?
  int8_t adrTxPow = 0;
  dr_t datarate = 0;     // current data rate
  // error coding rate (used for TX only), init at reset
  cr_t errcr;
  // adjustment for rejoin datarate
  uint8_t rejoinCnt; 

  uint16_t clockError = 0; // Inaccuracy in the clock. CLOCK_ERROR_MAX
                           // represents +/-100% error

  // pending data length
  uint8_t pendTxLen = 0;
  // pending data ask for confirmation
  bool pendTxConf;
  // pending data port
  uint8_t pendTxPort;
  // pending data
  uint8_t pendTxData[MAX_LEN_PAYLOAD];

  // last generated nonce
  // set at random value at reset.
  uint16_t devNonce;      
  
  // device address, set at 0 at reset.
  devaddr_t devaddr;
  // device level down stream seqno, reset after join.
  uint32_t seqnoDn;
  // device level up stream seqno, reset after join.
  uint32_t seqnoUp;
  // dn frame confirm pending: LORA::FCT_ACK or 0, reset after join
  uint8_t dnConf;
  // counter until we reset data rate (-128=off), reset after join
  // ask for confirmation if > 0
  // lower data rate if > LINK_CHECK_DEAD
  int8_t adrAckReq;

  // // Rx delay after TX, init at reset
  OsDeltaTime rxDelay; 

  uint8_t margin = 0;
  // link adr adapt answer pending, init after join
  bool ladrAns;
  // device status answer pending, init after join 
  bool devsAns;
  // adr Mode, init at reset
  uint8_t adrEnabled;
#if !defined(DISABLE_MCMD_DCAP_REQ)
  // have to ACK duty cycle settings, init after join
  bool dutyCapAns; 
#endif
#if !defined(DISABLE_MCMD_SNCH_REQ)
 // answer set new channel, init afet join.
  uint8_t snchAns;
#endif
  // 2nd RX window (after up stream), init at reset
  uint8_t dn2Dr;
  uint32_t dn2Freq;
#if !defined(DISABLE_MCMD_DN2P_SET)
  // 0=no answer pend, 0x80+ACKs, init after join
  uint8_t dn2Ans;
#endif

public:
  // Public part of MAC state
  uint8_t txCnt = 0;
  uint8_t txrxFlags = 0; // transaction flags (TX-RX combo)
  uint8_t dataBeg = 0;   // 0 or start of data (dataBeg-1 is port)
  uint8_t dataLen = 0;   // 0 no data or zero length data, >0 byte count of data
  uint8_t frame[MAX_LEN_FRAME] = {};

private:
  // callbacks
  static uint32_t convFreq(const uint8_t *ptr);
  static int8_t pow2dBm(uint8_t mcmd_ladr_p1);
  static OsDeltaTime getDwn2SafetyZone();
  static OsDeltaTime dr2hsym(dr_t dr);

  void processRx1DnData();
  void setupRx1();
  void setupRx2();
  void schedRx12(OsDeltaTime const &delay,
                 uint8_t dr);

  void txDone(OsDeltaTime const &delay);

  void runReset();
  void runEngineUpdate();

#if !defined(DISABLE_JOIN)
  void onJoinFailed();
  bool processJoinAcceptNoJoinFrame();
  bool processJoinAccept();
  void processRx1Jacc();
  void processRx2Jacc();
  void setupRx1Jacc();
  void setupRx2Jacc();
  void jreqDone();
  void startJoiningCallBack();

  void buildJoinRequest(uint8_t ftype);

#endif

  void processRx2DnData();

  void setupRx1DnData();
  void setupRx2DnData();

  void updataDone();

  void stateJustJoined();

  void reportEvent(ev_t ev);

  void buildDataFrame();
  void engineUpdate();
  void parseMacCommands (const uint8_t* opts, uint8_t olen);
  bool decodeFrame();
  bool processDnData();

#if defined(CFG_us915)
  void initDefaultChannels();
  void enableChannel(uint8_t channel);
  void enableSubBand(uint8_t band);
  void disableSubBand(uint8_t band);
  void selectSubBand(uint8_t band);
#endif
#if defined(CFG_eu868)
  void initDefaultChannels(bool join);
  uint32_t getFreq(uint8_t channel);
  uint8_t getBand(uint8_t channel);
  bool setupBand(uint8_t bandidx, int8_t txpow, uint16_t txcap);
#endif

  uint8_t mapChannels(uint8_t chpage, uint16_t chmap);
  void updateTx(OsTime const &txbeg);

  OsTime nextTx(OsTime const &now);

  void setRx1Params();

  void txDelay(OsTime const &reftime, uint8_t secSpan);

  void setDrJoin(dr_t dr);

#if !defined(DISABLE_JOIN)
  void initJoinLoop();
  bool nextJoinState();
#endif
public:
  // set default/start DR/txpow
  void setDrTxpow(uint8_t dr, int8_t pow);
  void setLinkCheckMode(bool enabled);
  void setSession(uint32_t netid, devaddr_t devaddr, uint8_t *nwkSKey,
                  uint8_t *artKey);

  bool setupChannel(uint8_t channel, uint32_t newfreq, uint16_t drmap,
                    int8_t band);
  void disableChannel(uint8_t channel);

  // set ADR mode (if mobile turn off)
  void setAdrMode(bool enabled);

#if !defined(DISABLE_JOIN)
  bool startJoining();
  void tryRejoin();

#endif

  void init();
  void shutdown();
  void reset();

  void clrTxData();
  void setTxData();
  int setTxData2(uint8_t port, uint8_t *data, uint8_t dlen, bool confirmed);
  void sendAlive();
  void setClockError(uint16_t error);

  uint16_t getOpMode() { return opmode; };

  void setEventCallBack(eventCallback_t callback) { eventCallBack = callback; };
  void setDevEuiCallback(keyCallback_t callback) { devEuiCallBack = callback; };
  void setArtEuiCallback(keyCallback_t callback) { artEuiCallBack = callback; };

  // for radio to wakeup processing.
  void nextTask();
};
// The state of LMIC MAC layer is encapsulated in this class.
extern Lmic LMIC; 

//! Construct a bit map of allowed datarates from drlo to drhi (both included).
#define DR_RANGE_MAP(drlo, drhi)                                               \
  (((uint16_t)0xFFFF << (drlo)) & ((uint16_t)0xFFFF >> (15 - (drhi))))

#endif // _lmic_h_
