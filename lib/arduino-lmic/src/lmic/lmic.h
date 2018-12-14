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
#include "enumflagsvalue.h"
#include "lmicrand.h"
#include "lorabase.h"
#include "oslmic.h"
#include "radio.h"

// LMIC version
#define LMIC_VERSION_MAJOR 1
#define LMIC_VERSION_MINOR 5

//!< Transmit attempts for confirmed frames
const uint8_t TXCONF_ATTEMPTS = 8;

// secs - random period for retrying a confirmed send
const uint8_t RETRY_PERIOD_secs = 3;

const int8_t KEEP_TXPOW = -128;

// MAC operation modes (lmic_t.opmode).
enum class OpState : uint8_t {
  // device joining in progress (blocks other activities)
  JOINING = 0x00,
  // TX user data (buffered in pendTxData)
  TXDATA,
  // send empty UP frame to ACK confirmed DN/fetch more DN data
  POLL,
  // occasionally send JOIN REQUEST
  REJOIN,
  // prevent MAC from doing anything
  SHUTDOWN,
  // TX/RX transaction pending
  TXRXPEND,
  // find a new channel
  NEXTCHNL,
  // link was reported as dead
  LINKDEAD,
};
using OpStateValue = EnumFlagsValue<OpState>;

// TX-RX transaction flags - report back to user
enum class TxRxStatus : uint8_t {
  // received in 1st DN slot
  DNW1,
  // received in 2dn DN slot
  DNW2,
  // set if a frame with a port was RXed,
  // LMIC.frame[LMIC.dataBeg-1] => port
  PORT,
  // set if a frame with a port was RXed, clr if no frame/no port
  NOPORT,
  // confirmed UP frame was acked
  ACK,
  // confirmed UP frame was not acked
  NACK,
}; // received in a scheduled RX slot

using TxRxStatusValue = EnumFlagsValue<TxRxStatus>;

// Event types for event callback
enum class EventType : uint8_t {
  JOINING = 1,
  JOINED,
  JOIN_FAILED,
  REJOIN_FAILED,
  TXCOMPLETE,
  RESET,
  LINK_DEAD,
  LINK_ALIVE
};

using eventCallback_t = void (*)(EventType);
using keyCallback_t = void (*)(uint8_t *);

// This value represents 100% error in LMIC.clockError
const uint8_t MAX_CLOCK_ERROR = 255;

const int8_t ADR_ACK_DELAY = 32;
const int8_t ADR_ACK_LIMIT = 64;

// continue with this after reported dead link
const int8_t LINK_CHECK_CONT = 0;
// after this UP frames and no response from NWK assume link is dead
const int8_t LINK_CHECK_DEAD = ADR_ACK_DELAY;
// UP frame count until we ask for ADRACKReq
const int8_t LINK_CHECK_INIT = -ADR_ACK_LIMIT;
// link check disabled
const int8_t LINK_CHECK_OFF = -128;

class Lmic {
public:
  static OsDeltaTime calcAirTime(rps_t rps, uint8_t plen);

private:
  Radio radio;
  OsJobType<Lmic> osjob{*this, OSS};
  // Radio settings TX/RX (also accessed by HAL)
  OsTime rxtime;

  // radio parameters set
  rps_t rps;
  uint8_t rxsyms = 0;

  eventCallback_t eventCallBack = nullptr;
  keyCallback_t devEuiCallBack = nullptr;
  keyCallback_t artEuiCallBack = nullptr;

protected:
  OsTime txend;
  uint8_t dndr = 0;
  uint32_t freq = 0;
  uint8_t txChnl = 0;         // channel for next TX
  uint8_t globalDutyRate = 0; // max rate: 1/2^k
  OsTime globalDutyAvail;     // time device can send again

  // ADR adjusted TX power, limit power to this value.
  int8_t adrTxPow;
  int8_t txpow = 0; // dBm
  int8_t antennaPowerAdjustment = 0;

  dr_t datarate = 0; // current data rate
  // curent opmode set at init
  OpStateValue opmode;
private:
  uint32_t netid; // current network id (~0 - none)
  // configured up repeat for unconfirmed message, reset after join.
  // Not handle properly  cf: LoRaWAN™ Specification §5.2
  uint8_t upRepeat;

  // adjustment for rejoin datarate
  uint8_t rejoinCnt;

  uint8_t clockError = 0; // Inaccuracy in the clock. CLOCK_ERROR_MAX
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

  // link adr adapt answer pending, init after join
  // use bit 15 as flag, other as value for acq
  uint8_t ladrAns;
  // device status answer pending, init after join
  bool devsAns;
  // RX timing setup answer pending, init after join
  bool rxTimingSetupAns;
#if !defined(DISABLE_MCMD_DCAP_REQ)
  // have to ACK duty cycle settings, init after join
  bool dutyCapAns;
#endif
#if !defined(DISABLE_MCMD_SNCH_REQ)
  // answer set new channel, init afet join.
  uint8_t snchAns;
#endif
protected:
  // 1 RX window DR offset
  uint8_t rx1DrOffset;

private:
  // 2nd RX window (after up stream), init at reset
  dr_t dn2Dr;
  uint32_t dn2Freq;
#if !defined(DISABLE_MCMD_DN2P_SET)
  // 0=no answer pend, 0x80+ACKs, init after join
  uint8_t dn2Ans;
#endif

  // Public part of MAC state
  uint8_t frame[MAX_LEN_FRAME];
  // transaction flags (TX-RX combo)
  TxRxStatusValue txrxFlags;
  // 0 no data or zero length data, >0 byte count of data
  uint8_t dataLen = 0;
  // 0 or start of data (dataBeg-1 is port)
  uint8_t dataBeg = 0;

  Aes aes;

protected:
  uint8_t txCnt = 0;
  LmicRand rand;

private:
  // callbacks
  void processRx1DnData();
  void setupRx1();
  void setupRx2();
  void schedRx12(OsDeltaTime delay, uint8_t dr);

  void txDone(OsDeltaTime delay);

  void runReset();
  void runEngineUpdate();

#if !defined(DISABLE_JOIN)
  void onJoinFailed();
  void processJoinAcceptNoJoinFrame();
  bool processJoinAccept();
  void processRx1Jacc();
  void processRx2Jacc();
  void setupRx1Jacc();
  void setupRx2Jacc();
  void jreqDone();
  void startJoiningCallBack();

  void buildJoinRequest();

#endif

  void processRx2DnData();

  void setupRx1DnData();
  void setupRx2DnData();

  void updataDone();

  void stateJustJoined();

  void reportEvent(EventType ev);

  void buildDataFrame();
  void engineUpdate();
  void parseMacCommands(const uint8_t *opts, uint8_t olen);
  bool decodeFrame();
  void processDnData();
  void txDelay(OsTime reftime, uint8_t secSpan);
  void resetAdrCount();
  void incrementAdrCount();
public:
  void setDrJoin(dr_t dr);
  // set default/start DR/txpow
  void setDrTxpow(uint8_t dr, int8_t pow);
  // set ADR mode (if mobile turn off)
  // It activate ADR bit and LINK Check.
  void setLinkCheckMode(bool enabled);
  void setSession(uint32_t netid, devaddr_t devaddr, uint8_t *nwkSKey,
                  uint8_t *artKey);

  /**
   * Adjust output power by this amount (for antenna gain)
   */
  void setAntennaPowerAdjustment(int8_t power);
#if !defined(DISABLE_JOIN)
  bool startJoining();
  void tryRejoin();

#endif

  void init();
  void shutdown();
  void reset();
  void setDevKey(uint8_t const key[16]) { aes.setDevKey(key); };

  void clrTxData();
  void setTxData();
  int8_t setTxData2(uint8_t port, uint8_t *data, uint8_t dlen, bool confirmed);
  void sendAlive();
  void setClockError(uint8_t error);

  OpStateValue getOpMode() const { return opmode; };
  TxRxStatusValue getTxRxFlags() const { return txrxFlags; };
  uint8_t getDataLen() const { return dataLen; };
  uint8_t const *getData() const {
    return dataBeg ? frame + dataBeg : nullptr;
  };
  uint8_t getPort() const {
    return txrxFlags.test(TxRxStatus::PORT) ? frame[dataBeg - 1] : 0;
  };

  void setEventCallBack(eventCallback_t callback) { eventCallBack = callback; };
  void setDevEuiCallback(keyCallback_t callback) { devEuiCallBack = callback; };
  void setArtEuiCallback(keyCallback_t callback) { artEuiCallBack = callback; };

protected:
  virtual uint8_t getRawRps(dr_t dr) const = 0;

  virtual int8_t pow2dBm(uint8_t powerIndex) const = 0;
  virtual OsDeltaTime getDwn2SafetyZone() const = 0;
  virtual OsDeltaTime dr2hsym(dr_t dr) const = 0;
  virtual uint32_t convFreq(const uint8_t *ptr) const = 0;
  virtual bool validRx1DrOffset(uint8_t drOffset) const = 0;

  virtual void initDefaultChannels(bool join) = 0;
  virtual bool setupChannel(uint8_t channel, uint32_t newfreq, uint16_t drmap,
                            int8_t band) = 0;
  virtual void disableChannel(uint8_t channel) = 0;
  virtual void handleCFList(const uint8_t *ptr) = 0;

  virtual bool mapChannels(uint8_t chpage, uint16_t chmap) = 0;
  virtual void updateTx(OsTime txbeg, OsDeltaTime airtime) = 0;
  virtual OsTime nextTx(OsTime now) = 0;
  virtual void setRx1Params() = 0;
#if !defined(DISABLE_JOIN)
  virtual void initJoinLoop() = 0;
  virtual bool nextJoinState() = 0;
#endif
  virtual dr_t defaultRX2Dr() const = 0;
  virtual uint32_t defaultRX2Freq() const = 0;

  rps_t updr2rps(dr_t dr) const;
  rps_t dndr2rps(dr_t dr) const;
  bool isFasterDR(dr_t dr1, dr_t dr2) const;
  bool isSlowerDR(dr_t dr1, dr_t dr2) const;
  // increase data rate
  dr_t incDR(dr_t dr) const;
  // decrease data rate
  dr_t decDR(dr_t dr) const;
  // in range
  bool validDR(dr_t dr) const;
  // decrease data rate by n steps
  dr_t lowerDR(dr_t dr, uint8_t n) const;

public:
  explicit Lmic(lmic_pinmap const &pins);
  void io_check();
  void store_trigger();
};

// Construct a bit map of allowed datarates from drlo to drhi (both included).
template <typename T> constexpr uint16_t dr_range_map(T drlo, T drhi) {
  return (((uint16_t)0xFFFF << static_cast<uint8_t>(drlo)) &
          ((uint16_t)0xFFFF >> (15 - static_cast<uint8_t>(drhi))));
}

#endif // _lmic_h_
