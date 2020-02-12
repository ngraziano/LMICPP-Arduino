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

#ifndef _lmic_h_
#define _lmic_h_

#include "../aes/lmic_aes.h"
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
  TXCOMPLETE,
  RESET,
  LINK_DEAD,
  LINK_ALIVE
};

using eventCallback_t = void (*)(EventType);
using keyCallback_t = void (*)(uint8_t *);

// This value represents 100% error in LMIC.clockError
constexpr uint8_t MAX_CLOCK_ERROR = 255;

constexpr int8_t ADR_ACK_DELAY = 32;
constexpr int8_t ADR_ACK_LIMIT = 64;

// continue with this after reported dead link
constexpr int8_t LINK_CHECK_CONT = 0;
// after this UP frames and no response from NWK assume link is dead
constexpr int8_t LINK_CHECK_DEAD = ADR_ACK_DELAY;
// UP frame count until we ask for ADRACKReq
constexpr int8_t LINK_CHECK_INIT = -ADR_ACK_LIMIT;
// link check disabled
constexpr int8_t LINK_CHECK_OFF = -128;

struct FrequencyAndRate {
  uint32_t frequency;
  dr_t datarate;
};

class Lmic {
public:
  static OsDeltaTime calcAirTime(rps_t rps, uint8_t plen);

private:
  Radio &radio;
  OsJobType<Lmic> osjob;
  // Radio settings TX/RX (also accessed by HAL)
  OsTime rxtime;
  // time of detect of change of state of radio module
  OsTime last_int_trigger;
  uint8_t rxsyms = 0;

  eventCallback_t eventCallBack = nullptr;
  keyCallback_t devEuiCallBack = nullptr;
  keyCallback_t artEuiCallBack = nullptr;

protected:
  OsTime txend;

  // ADR adjusted TX power, limit power to this value.
  // dBm
  int8_t adrTxPow;
  dr_t datarate = 0; // current data rate

private:
  // curent opmode set at init
  OpStateValue opmode;

  int8_t antennaPowerAdjustment = 0;
  // last time we increase duty rate for back-off
  OsTime lastDutyRateBackOff;
  // max rate: 1/2^k
  uint8_t globalDutyRate = 0;
  // time device can send again
  OsTime globalDutyAvail;
  // current network id (~0 - none)
  uint32_t netid;
  // configured up repeat for unconfirmed message, reset after join.
  // Not handle properly  cf: LoRaWAN™ Specification §5.2
  uint8_t upRepeat;

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

  // Rx delay after TX, init at reset
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
  // answer set new channel, init after join.
  uint8_t snchAns;
#endif

private:
  // 2nd RX window (after up stream), init at reset
  FrequencyAndRate rx2Parameter;

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
  // 1 RX window DR offset
  uint8_t rx1DrOffset;

  uint8_t txCnt = 0;
  LmicRand rand;

private:
  // callbacks
  void processRxDnData();
  void processRx1DnData();
  void processRx2DnData();
  void setupRx1();
  void setupRx2();
  OsTime schedRx12(OsDeltaTime delay, dr_t dr);

  void txDone(OsDeltaTime delay);

  void runReset();
  void runEngineUpdate();

  void onJoinFailed();
  void processJoinAcceptNoJoinFrame();
  bool processJoinAccept();
  void processRxJacc();

  void jreqDone();
  void startJoiningCallBack();

  void buildJoinRequest();

  void updataDone();

  void stateJustJoined();

  void reportEvent(EventType ev);

  uint8_t *add_opt_dcap(uint8_t *buffer_pos);
  uint8_t *add_opt_dn2p(uint8_t *buffer_pos);
  uint8_t *add_opt_devs(uint8_t *buffer_pos);
  uint8_t *add_opt_adr(uint8_t *buffer_pos);
  uint8_t *add_opt_rxtiming(uint8_t *buffer_pos);
  uint8_t *add_opt_snch(uint8_t *buffer_pos);

  void buildDataFrame();
  void engineUpdate();
  void parse_ladr(const uint8_t *const opts);
  void parse_dn2p(const uint8_t *const opts);
  void parse_dcap(const uint8_t *const opts);
  void parse_snch(const uint8_t *const opts);
  void parse_rx_timing_setup(const uint8_t *const opts);
  void parseMacCommands(const uint8_t *opts, uint8_t olen);
  enum class SeqNoValidity : uint8_t {
    invalid,
    previous,
    ok,
  };
  uint32_t read_seqno(const uint8_t *const buffer) const;
  SeqNoValidity check_seq_no(const uint32_t seqNo, const uint8_t ftype) const;

  bool decodeFrame();
  void processDnData();
  void txDelay(OsTime reftime, uint8_t secSpan);
  void resetAdrCount();
  void incrementAdrCount();

public:
  void setDrJoin(dr_t dr);
  // set default/start DR/txpow
  void setDrTx(uint8_t dr);
  void setRx2Parameter(uint32_t rx2frequency, dr_t rx2datarate);
  void setDutyRate(uint8_t duty_rate);
  // set ADR mode (if mobile turn off)
  // It activate ADR bit and LINK Check.
  void setLinkCheckMode(bool enabled);
  void setSession(uint32_t netid, devaddr_t devaddr, AesKey const &nwkSKey,
                  AesKey const &artKey);

  /**
   * Adjust output power by this amount (for antenna gain)
   */
  void setAntennaPowerAdjustment(int8_t power);
  bool startJoining();

  void init();
  void shutdown();
  void reset();
  void setDevKey(const AesKey &key) { aes.setDevKey(key); };

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
  virtual uint32_t getTxFrequency() const = 0;
  virtual int8_t getTxPower() const = 0;
  virtual FrequencyAndRate getRx1Parameter() const = 0;
  virtual uint8_t getRawRps(dr_t dr) const = 0;

  int8_t const InvalidPower = -128;
  /**
   * Return InvalidPower if passed value is invalid
   */
  virtual int8_t pow2dBm(uint8_t powerIndex) const = 0;
  virtual OsDeltaTime getDwn2SafetyZone() const = 0;
  virtual OsDeltaTime dr2hsym(dr_t dr) const = 0;
  virtual uint32_t convFreq(const uint8_t *ptr) const = 0;
  virtual bool validRx1DrOffset(uint8_t drOffset) const = 0;

  virtual void initDefaultChannels() = 0;
  virtual bool setupChannel(uint8_t channel, uint32_t newfreq,
                            uint16_t drmap) = 0;
  virtual void disableChannel(uint8_t channel) = 0;
  virtual void handleCFList(const uint8_t *ptr) = 0;

  virtual bool validMapChannels(uint8_t chpage, uint16_t chmap) = 0;
  virtual void mapChannels(uint8_t chpage, uint16_t chmap) = 0;
  virtual void updateTxTimes(OsDeltaTime airtime) = 0;
  virtual OsTime nextTx(OsTime now) = 0;
  virtual void initJoinLoop() = 0;
  virtual bool nextJoinState() = 0;
  virtual FrequencyAndRate defaultRX2Parameter() const = 0;

#if defined(ENABLE_SAVE_RESTORE)

  virtual void saveState(StoringAbtract &store) const;
  virtual void saveStateWithoutTimeData(StoringAbtract &store) const;
  virtual void loadState(RetrieveAbtract &strore);
  virtual void loadStateWithoutTimeData(RetrieveAbtract &strore);
#endif

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

  OsTime int_trigger_time() const;
  void wait_end_rx();
  void wait_end_tx();

public:
  explicit Lmic(Radio &radio, OsScheduler &scheduler);
  void store_trigger();
};

// Construct a bit map of allowed datarates from drlo to drhi (both included).
template <typename T> constexpr uint16_t dr_range_map(T drlo, T drhi) {
  return (((uint16_t)0xFFFF << static_cast<uint8_t>(drlo)) &
          ((uint16_t)0xFFFF >> (15 - static_cast<uint8_t>(drhi))));
}

#endif // _lmic_h_
