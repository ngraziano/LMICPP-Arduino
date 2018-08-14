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
#include "../aes/aes.h"
#include "bufferpack.h"
#include "radio.h"
#include <algorithm>

#if !defined(MINRX_SYMS)
#define MINRX_SYMS 5
#endif // !defined(MINRX_SYMS)
#define PAMBL_SYMS 8
#define PAMBL_FSK 5
#define PRERX_FSK 1
#define RXLEN_FSK (1 + 5 + 2)

#define BCN_INTV_osticks OsDeltaTime::from_sec(BCN_INTV_sec)
#define TXRX_GUARD_osticks OsDeltaTime::from_ms(TXRX_GUARD_ms)
#define JOIN_GUARD_osticks OsDeltaTime::from_ms(JOIN_GUARD_ms)
#define DELAY_JACC1_osticks OsDeltaTime::from_sec(DELAY_JACC1)
#define DELAY_JACC2_osticks OsDeltaTime::from_sec(DELAY_JACC2)
#define DELAY_EXTDNW2_osticks OsDeltaTime::from_sec(DELAY_EXTDNW2)
#define BCN_RESERVE_osticks OsDeltaTime::from_ms(BCN_RESERVE_ms)
#define BCN_GUARD_osticks OsDeltaTime::from_ms(BCN_GUARD_ms)
#define BCN_WINDOW_osticks OsDeltaTime::from_ms(BCN_WINDOW_ms)
#define AIRTIME_BCN_osticks OsDeltaTime::from_us(AIRTIME_BCN)

Lmic LMIC;

// ================================================================================
// BEG OS - default implementations for certain OS suport functions

#if !defined(HAS_os_calls)

#if !defined(os_getBattLevel)
uint8_t os_getBattLevel(void) { return MCMD_DEVS_BATT_NOINFO; }
#endif

#endif // !HAS_os_calls

// END OS - default implementations for certain OS suport functions
// ================================================================================

// ================================================================================
// BEG LORA

static CONST_TABLE(uint8_t, SENSITIVITY)[7][3] = {
    // ------------bw----------
    // 125kHz    250kHz    500kHz
    {141 - 109, 141 - 109, 141 - 109}, // FSK
    {141 - 127, 141 - 124, 141 - 121}, // SF7
    {141 - 129, 141 - 126, 141 - 123}, // SF8
    {141 - 132, 141 - 129, 141 - 126}, // SF9
    {141 - 135, 141 - 132, 141 - 129}, // SF10
    {141 - 138, 141 - 135, 141 - 132}, // SF11
    {141 - 141, 141 - 138, 141 - 135}  // SF12
};

int getSensitivity(rps_t rps) {
  return -141 + TABLE_GET_U1_TWODIM(SENSITIVITY, getSf(rps), getBw(rps));
}

OsDeltaTime calcAirTime(rps_t rps, uint8_t plen) {
  uint8_t bw = getBw(rps); // 0,1,2 = 125,250,500kHz
  uint8_t sf = getSf(rps); // 0=FSK, 1..6 = SF7..12
  uint8_t sfx = 4 * (sf + (7 - SF7));
  uint8_t q = sfx - (sf >= SF11 ? 8 : 0);
  int tmp =
      8 * plen - sfx + 28 + (getNocrc(rps) ? 0 : 16) - (getIh(rps) ? 20 : 0);
  if (tmp > 0) {
    tmp = (tmp + q - 1) / q;
    tmp *= getCr(rps) + 5;
    tmp += 8;
  } else {
    tmp = 8;
  }
  tmp = (tmp << 2) + /*preamble*/ 49 /* 4 * (8 + 4.25) */;
  // bw = 125000 = 15625 * 2^3
  //      250000 = 15625 * 2^4
  //      500000 = 15625 * 2^5
  // sf = 7..12
  //
  // osticks =  tmp * OSTICKS_PER_SEC * 1<<sf / bw
  //
  // 3 => counter reduced divisor 125000/8 => 15625
  // 2 => counter 2 shift on tmp
  sfx = sf + (7 - SF7) - (3 + 2) - bw;
  int div = 15625;
  if (sfx > 4) {
    // prevent 32bit signed int overflow in last step
    div >>= sfx - 4;
    sfx = 4;
  }
  // Need 32bit arithmetic for this last step
  return (((int32_t)tmp << sfx) * OSTICKS_PER_SEC + div / 2) / div;
}

extern inline rps_t updr2rps(dr_t dr);
extern inline rps_t dndr2rps(dr_t dr);
extern inline bool isFasterDR(dr_t dr1, dr_t dr2);
extern inline bool isSlowerDR(dr_t dr1, dr_t dr2);
extern inline dr_t incDR(dr_t dr);
extern inline dr_t decDR(dr_t dr);
extern inline dr_t assertDR(dr_t dr);
extern inline bool validDR(dr_t dr);
extern inline dr_t lowerDR(dr_t dr, uint8_t n);

extern inline sf_t getSf(rps_t params);
extern inline rps_t setSf(rps_t params, sf_t sf);
extern inline bw_t getBw(rps_t params);
extern inline rps_t setBw(rps_t params, bw_t cr);
extern inline cr_t getCr(rps_t params);
extern inline rps_t setCr(rps_t params, cr_t cr);
extern inline int getNocrc(rps_t params);
extern inline rps_t setNocrc(rps_t params, int nocrc);
extern inline int getIh(rps_t params);
extern inline rps_t setIh(rps_t params, int ih);
extern inline rps_t makeRps(sf_t sf, bw_t bw, cr_t cr, int ih, int nocrc);
extern inline int sameSfBw(rps_t r1, rps_t r2);

// END LORA
// ================================================================================

// Adjust DR for TX retries
//  - indexed by retry count
//  - return steps to lower DR
static CONST_TABLE(uint8_t, DRADJUST)[2 + TXCONF_ATTEMPTS] = {
    // normal frames - 1st try / no retry
    0,
    // confirmed frames
    0, 0, 1, 0, 1, 0, 1, 0, 0};

void Lmic::txDelay(OsTime const &reftime, uint8_t secSpan) {
  auto delayRef = reftime + OsDeltaTime::rnd_delay(secSpan);
  if (globalDutyRate == 0 || (delayRef - globalDutyAvail) > OsDeltaTime(0)) {
    globalDutyAvail = delayRef;
    opmode |= OP_RNDTX;
  }
}

void Lmic::setDrJoin(dr_t dr) { datarate = dr; }

void Lmic::setDrTxpow(uint8_t dr, int8_t pow) {
  if (pow != KEEP_TXPOW)
    adrTxPow = pow;
  if (datarate != dr) {
    datarate = dr;
    opmode |= OP_NEXTCHNL;
  }
}

void Lmic::runEngineUpdate() { engineUpdate(); }

void Lmic::reportEvent(ev_t ev) {
  if (eventCallBack)
    eventCallBack(ev);
  engineUpdate();
}

void Lmic::runReset() {
  // Disable session
  reset();
#if !defined(DISABLE_JOIN)
  startJoining();
#endif // !DISABLE_JOIN
  reportEvent(EV_RESET);
}

void Lmic::stateJustJoined() {
  seqnoDn = 0;
  seqnoUp = 0;
  rejoinCnt = 0;
  dnConf = 0;
  ladrAns = false;
  devsAns = false;
#if !defined(DISABLE_MCMD_SNCH_REQ)
  snchAns = 0;
#endif
#if !defined(DISABLE_MCMD_DN2P_SET)
  dn2Ans = 0;
#endif
#if !defined(DISABLE_MCMD_DCAP_REQ)
  dutyCapAns = false;
#endif
  upRepeat = 0;
  adrAckReq = LINK_CHECK_INIT;
  rx1DrOffset = 0;
  dn2Dr = DR_DNW2;
  dn2Freq = FREQ_DNW2;
}

void Lmic::parseMacCommands(const uint8_t *opts, uint8_t olen) {
  uint8_t oidx = 0;
  while (oidx < olen) {
    switch (opts[oidx]) {
    // LinkCheckReq LoRaWAN™ Specification §5.1
    case MCMD_LCHK_ANS: {
      // int gwmargin = opts[oidx+1];
      // int ngws = opts[oidx+2];
      oidx += 3;
      continue;
    }
    // LinkADRReq LoRaWAN™ Specification §5.2
    case MCMD_LADR_REQ: {
      // FIXME multiple LinkAdrReq not handled properly in continous block
      // must be handle atomic.
      uint8_t p1 = opts[oidx + 1];               // txpow + DR
      uint16_t chMask = rlsbf2(&opts[oidx + 2]); // list of enabled channels
      uint8_t chMaskCntl =
          opts[oidx + 4] & MCMD_LADR_CHPAGE_MASK; // channel page
      uint8_t nbTrans =
          opts[oidx + 4] & MCMD_LADR_REPEAT_MASK; // up repeat count
      oidx += 5;

      ladrAns = 0x80 | // Include an answer into next frame up
                MCMD_LADR_ANS_POWACK | MCMD_LADR_ANS_CHACK |
                MCMD_LADR_ANS_DRACK;
      if (!regionLMic.mapChannels(chMaskCntl, chMask))
        ladrAns &= ~MCMD_LADR_ANS_CHACK;
      dr_t dr = (dr_t)(p1 >> MCMD_LADR_DR_SHIFT);
      if (!validDR(dr)) {
        ladrAns &= ~MCMD_LADR_ANS_DRACK;
      }
      if ((ladrAns & 0x7F) ==
          (MCMD_LADR_ANS_POWACK | MCMD_LADR_ANS_CHACK | MCMD_LADR_ANS_DRACK)) {
        // Nothing went wrong - use settings
        upRepeat = nbTrans;
        setDrTxpow(dr, regionLMic.pow2dBm(p1));
      }
      if (adrAckReq != LINK_CHECK_OFF) {
        // force ack to NWK.
        adrAckReq = 0;
      }
      continue;
    }
    // DevStatusReq LoRaWAN™ Specification §5.5
    case MCMD_DEVS_REQ: {
      devsAns = true;
      oidx += 1;
      continue;
    }
    // RXParamSetupReq LoRaWAN™ Specification §5.4
    case MCMD_DN2P_SET: {
#if !defined(DISABLE_MCMD_DN2P_SET)
      dr_t dr = (dr_t)(opts[oidx + 1] & 0x0F);
      uint8_t newRx1DrOffset = ((opts[oidx+1] & 0x70) >> 4);
      uint32_t newfreq = regionLMic.convFreq(&opts[oidx + 2]);
      dn2Ans = 0x80; // answer pending
      if(regionLMic.validRx1DrOffset(newRx1DrOffset))
        dn2Ans |= MCMD_DN2P_ANS_RX1DrOffsetAck;
      if (validDR(dr))
        dn2Ans |= MCMD_DN2P_ANS_DRACK;
      if (newfreq != 0)
        dn2Ans |= MCMD_DN2P_ANS_CHACK;
      if (dn2Ans == (0x80 | MCMD_DN2P_ANS_RX1DrOffsetAck | MCMD_DN2P_ANS_DRACK | MCMD_DN2P_ANS_CHACK)) {
        dn2Dr = dr;
        dn2Freq = newfreq;
        rx1DrOffset = newRx1DrOffset;
      }
#endif // !DISABLE_MCMD_DN2P_SET
      oidx += 5;
      continue;
    }
    case MCMD_DCAP_REQ: {
#if !defined(DISABLE_MCMD_DCAP_REQ)
      uint8_t cap = opts[oidx + 1];
      // A value cap=0xFF means device is OFF unless enabled again manually.
      if (cap == 0xFF)
        opmode |= OP_SHUTDOWN; // stop any sending
      globalDutyRate = cap & 0xF;
      globalDutyAvail = os_getTime();
      dutyCapAns = true;
#endif // !DISABLE_MCMD_DCAP_REQ
      oidx += 2;
      continue;
    }
    case MCMD_SNCH_REQ: {
#if !defined(DISABLE_MCMD_SNCH_REQ)
      uint8_t chidx = opts[oidx + 1];                          // channel
      uint32_t newfreq = regionLMic.convFreq(&opts[oidx + 2]); // freq
      uint8_t drs = opts[oidx + 5];                            // datarate span
      snchAns = 0x80;
      if (newfreq != 0 &&
          regionLMic.setupChannel(chidx, newfreq,
                                  DR_RANGE_MAP(drs & 0xF, drs >> 4), -1))
        snchAns |= MCMD_SNCH_ANS_DRACK | MCMD_SNCH_ANS_FQACK;
#endif // !DISABLE_MCMD_SNCH_REQ
      oidx += 6;
      continue;
    }
    case MCMD_RXTimingSetup_REQ: {
      // NOT IMPLEMENTED !
      oidx += 2;
      continue;
    }
    case MCMD_TxParamSetup_REQ: {
      // NOT IMPLEMENTED / NOT NEED IN EU868
      oidx += 2;
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
  if (oidx != olen) {
    // corrupted frame or unknown command
    PRINT_DEBUG_1("Parse of MAC command incompleted.");
  }
}

// ================================================================================
// Decoding frames
bool Lmic::decodeFrame() {
#if LMIC_DEBUG_LEVEL > 0
  const char *window = (txrxFlags & TXRX_DNW1)
                           ? "RX1"
                           : ((txrxFlags & TXRX_DNW2) ? "RX2" : "Other");
#endif

  if (dataLen == 0) {
    PRINT_DEBUG_1("No downlink data, window=%s", window);
    return false;
  }

  uint8_t *d = frame;
  uint8_t hdr = d[0];
  uint8_t ftype = hdr & HDR_FTYPE;
  uint8_t dlen = dataLen;

  if (dlen < OFF_DAT_OPTS + 4 || (hdr & HDR_MAJOR) != HDR_MAJOR_V1 ||
      (ftype != HDR_FTYPE_DADN && ftype != HDR_FTYPE_DCDN)) {
    // Basic sanity checks failed
    PRINT_DEBUG_1("Invalid downlink, window=%s", window);
    dataLen = 0;
    return false;
  }
  // Validate exact frame length
  // Note: device address was already read+evaluated in order to arrive here.
  uint8_t fct = d[OFF_DAT_FCT];
  uint32_t addr = rlsbf4(&d[OFF_DAT_ADDR]);
  uint32_t seqno = rlsbf2(&d[OFF_DAT_SEQNO]);
  uint8_t olen = fct & FCT_OPTLEN;
  bool ackup = (fct & FCT_ACK) != 0 ? true : false; // ACK last up frame
  uint8_t poff = OFF_DAT_OPTS + olen;
  uint8_t pend = dlen - MIC_LEN; // MIC

  if (addr != devaddr) {
    PRINT_DEBUG_1("Invalid address, window=%s", window);
    dataLen = 0;
    return false;
  }
  if (poff > pend) {
    PRINT_DEBUG_1("Invalid offset, window=%s", window);
    dataLen = 0;
    return false;
  }

  bool replayConf = false;

  seqno = seqnoDn + (uint16_t)(seqno - seqnoDn);

  if (!aes.verifyMic(devaddr, seqno, DIR_DOWN, d, dlen)) {
    PRINT_DEBUG_1("Fail to verify aes mic, window=%s", window);
    dataLen = 0;
    return false;
  }
  if (seqno < seqnoDn) {
    if ((int32_t)seqno > (int32_t)seqnoDn) {
      dataLen = 0;
      return false;
    }
    if (seqno != seqnoDn - 1 || !dnConf || ftype != HDR_FTYPE_DCDN) {
      dataLen = 0;
      return false;
    }
    // Replay of previous sequence number allowed only if
    // previous frame and repeated both requested confirmation
    replayConf = true;
  } else {
    if (seqno > seqnoDn) {
      // skip in sequence number
      // log ?
    }
    seqnoDn = seqno + 1; // next number to be expected
    // DN frame requested confirmation - provide ACK once with next UP frame
    dnConf = (ftype == HDR_FTYPE_DCDN ? FCT_ACK : 0);
  }

  if (dnConf || (fct & FCT_MORE))
    opmode |= OP_POLL;

  // We heard from network
  rejoinCnt = 0;
  if (adrAckReq != LINK_CHECK_OFF)
    adrAckReq = LINK_CHECK_INIT;

  // Process OPTS
  int16_t m = rssi - RSSI_OFF - getSensitivity(rps);
  margin = m < 0 ? 0 : m > 254 ? 254 : m;

  parseMacCommands(d + OFF_DAT_OPTS, olen);

  uint8_t port = -1;
  if (!replayConf) {
    // Handle payload only if not a replay
    if (pend > poff) {
      port = d[poff++];
      // Decrypt payload - if any
      aes.framePayloadEncryption(port, devaddr, seqno, DIR_DOWN, d + poff,
                                 pend - poff);
      txrxFlags |= TXRX_PORT;
      dataBeg = poff;
      dataLen = pend - poff;
      if (port == 0) {
        parseMacCommands(d + poff, pend - poff);
      }
    } else {
      txrxFlags |= TXRX_NOPORT;
      dataBeg = poff;
      dataLen = 0;
    }
  } else {
    // replay
    // not handle
  }

  if ( // NWK acks but we don't have a frame pending
      (ackup && txCnt == 0) ||
      // We sent up confirmed and we got a response in DNW1/DNW2
      // BUT it did not carry an ACK - this should never happen
      // Do not resend and assume frame was not ACKed.
      (!ackup && txCnt != 0)) {
    // suspirious hack
  }

  if (txCnt != 0) // we requested an ACK
    txrxFlags |= ackup ? TXRX_ACK : TXRX_NACK;

#if !defined(DISABLE_MCMD_DN2P_SET)
  // stop sending RXParamSetupAns when receive dowlink message
  dn2Ans = 0;
#endif

  PRINT_DEBUG_1("Received downlink, window=%s, port=%d, ack=%d", window, port,
                ackup);
  return true;
}

// ================================================================================
// TX/RX transaction support

void Lmic::setupRx2() {
  txrxFlags = TXRX_DNW2;
  rps = dndr2rps(dn2Dr);
  freq = dn2Freq;
  dataLen = 0;
  radio_rx();
}

void Lmic::schedRx12(OsDeltaTime const &delay, uint8_t dr) {
  PRINT_DEBUG_2("SchedRx RX1/2.");

  // Half symbol time for the data rate.
  OsDeltaTime hsym = regionLMic.dr2hsym(dr);

  rxsyms = MINRX_SYMS;

  // If a clock error is specified, compensate for it by extending the
  // receive window
  if (clockError != 0) {
    // Calculate how much the clock will drift maximally after delay has
    // passed. This indicates the amount of time we can be early
    // _or_ late.
    OsDeltaTime drift = OsDeltaTime(
        int32_t((int64_t)delay.tick() * clockError / MAX_CLOCK_ERROR));

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
  rxtime = txend + (delay + (PAMBL_SYMS - rxsyms) * hsym);

  osjob.setTimed(rxtime - RX_RAMPUP);
}

void Lmic::setupRx1() {
  txrxFlags = TXRX_DNW1;
  // Turn rps from TX over to RX
  rps = setNocrc(rps, 1);
  dataLen = 0;
  radio_rx();
}

// Called by HAL once TX complete and delivers exact end of TX time stamp in
// rxtime
void Lmic::txDone(OsDeltaTime const &delay) {
  // Change RX frequency / rps (US only) before we increment txChnl
  regionLMic.setRx1Params(txChnl, rx1DrOffset, dndr, freq, rps);
  schedRx12(delay, dndr);
}

// ======================================== Join frames

#if !defined(DISABLE_JOIN)
void Lmic::onJoinFailed() {
  // Notify app - must call reset() to stop joining
  // otherwise join procedure continues.
  reportEvent(EV_JOIN_FAILED);
}

bool Lmic::processJoinAcceptNoJoinFrame() {
  if ((opmode & OP_JOINING) == 0) {
    ASSERT((opmode & OP_REJOIN) != 0);
    // REJOIN attempt for roaming
    opmode &= ~(OP_REJOIN | OP_TXRXPEND);
    if (rejoinCnt < 10)
      rejoinCnt++;
    reportEvent(EV_REJOIN_FAILED);
    return true;
  }
  opmode &= ~OP_TXRXPEND;
  // Clear NEXTCHNL because join state engine controls channel hopping
  opmode &= ~OP_NEXTCHNL;
  bool succes = regionLMic.nextJoinState(txChnl, txCnt, datarate, txend);
  // Build next JOIN REQUEST with next engineUpdate call
  // Optionally, report join failed.
  // Both after a random/chosen amount of ticks.

  osjob.setCallbackRunnable(
      succes ? &Lmic::runEngineUpdate // next step to be delayed
             : &Lmic::onJoinFailed);  // one JOIN iteration done and failed
  return true;
}
bool Lmic::processJoinAccept() {
  PRINT_DEBUG_2("Process join accept.");

  ASSERT(txrxFlags != TXRX_DNW1 || dataLen != 0);
  ASSERT((opmode & OP_TXRXPEND) != 0);

  if (dataLen == 0) {
    return processJoinAcceptNoJoinFrame();
  }

  uint8_t hdr = frame[0];
  uint8_t dlen = dataLen;

  if ((dlen != LEN_JA && dlen != LEN_JAEXT) ||
      (hdr & (HDR_FTYPE | HDR_MAJOR)) != (HDR_FTYPE_JACC | HDR_MAJOR_V1)) {
    // unexpected frame
    if ((txrxFlags & TXRX_DNW1) != 0)
      return false;
    return processJoinAcceptNoJoinFrame();
  }
  aes.encrypt(frame + 1, dlen - 1);
  if (!aes.verifyMic0(frame, dlen)) {
    // bad mic
    if ((txrxFlags & TXRX_DNW1) != 0)
      return false;
    return processJoinAcceptNoJoinFrame();
  }

  uint32_t addr = rlsbf4(frame + OFF_JA_DEVADDR);
  devaddr = addr;
  netid = rlsbf4(&frame[OFF_JA_NETID]) & 0xFFFFFF;

  regionLMic.initDefaultChannels(false);

  if (dlen > LEN_JA) {
#if defined(CFG_us915)
    if ((txrxFlags & TXRX_DNW1) != 0)
      return false;
    return processJoinAcceptNoJoinFrame();
#endif
    dlen = OFF_CFLIST;
    for (uint8_t chidx = 3; chidx < 8; chidx++, dlen += 3) {
      uint32_t newfreq = regionLMic.convFreq(&frame[dlen]);
      if (newfreq) {
        regionLMic.setupChannel(chidx, newfreq, 0, -1);
#if LMIC_DEBUG_LEVEL > 1
        lmic_printf("%lu: Setup channel, idx=%d, freq=%lu\n", os_getTime(),
                    chidx, (unsigned long)newfreq);
#endif
      }
    }
  }

  // already incremented when JOIN REQ got sent off
  aes.sessKeys(devNonce - 1, &frame[OFF_JA_ARTNONCE]);

  ASSERT((opmode & (OP_JOINING | OP_REJOIN)) != 0);
  if ((opmode & OP_REJOIN) != 0) {
    // Lower DR every try below current UP DR
    datarate = lowerDR(datarate, rejoinCnt);
  }
  opmode &= ~(OP_JOINING | OP_TRACK | OP_REJOIN | OP_TXRXPEND | OP_PINGINI) |
            OP_NEXTCHNL;
  txCnt = 0;
  stateJustJoined();
  dn2Dr = frame[OFF_JA_DLSET] & 0x0F;
  rx1DrOffset = (LMIC.frame[OFF_JA_DLSET] >> 4) & 0x7;

  if (frame[OFF_JA_RXDLY] == 0) {
    rxDelay = OsDeltaTime::from_sec(DELAY_DNW1);
  } else {
    rxDelay = OsDeltaTime::from_sec(frame[OFF_JA_RXDLY]);
  }
  reportEvent(EV_JOINED);
  return true;
}

void Lmic::processRx2Jacc() {
  if (dataLen == 0)
    txrxFlags = 0; // nothing in 1st/2nd DN slot
  processJoinAccept();
}

void Lmic::setupRx2Jacc() {
  PRINT_DEBUG_2("Setup RX2 join accept.");
  this->osjob.setCallbackFuture(&Lmic::processRx2Jacc);
  setupRx2();
}

void Lmic::processRx1Jacc() {
  PRINT_DEBUG_2("Result RX1 join accept datalen=%i.", dataLen);
  if (dataLen == 0 || !processJoinAccept()) {
    osjob.setCallbackFuture(&Lmic::setupRx2Jacc);
    schedRx12(DELAY_JACC2_osticks, dn2Dr);
  }
}

void Lmic::setupRx1Jacc() {
  PRINT_DEBUG_2("Setup RX1 join accept.");
  this->osjob.setCallbackFuture(&Lmic::processRx1Jacc);
  setupRx1();
}

void Lmic::jreqDone() {
  osjob.setCallbackFuture(&Lmic::setupRx1Jacc);
  txDone(DELAY_JACC1_osticks);
}

#endif // !DISABLE_JOIN

// ======================================== Data frames

void Lmic::processRx2DnData() {
  if (dataLen == 0) {
    txrxFlags = 0; // nothing in 1st/2nd DN slot
    // It could be that the gateway *is* sending a reply, but we
    // just didn't pick it up. To avoid TX'ing again while the
    // gateay is not listening anyway, delay the next transmission
    // until DNW2_SAFETY_ZONE from now, and add up to 2 seconds of
    // extra randomization.
    txDelay(os_getTime() + regionLMic.getDwn2SafetyZone(), 2);
  }
  processDnData();
}

void Lmic::setupRx2DnData() {
  osjob.setCallbackFuture(&Lmic::processRx2DnData);
  setupRx2();
}

void Lmic::processRx1DnData() {
  if (!processDnData()) {
    osjob.setCallbackFuture(&Lmic::setupRx2DnData);
    schedRx12(rxDelay + OsDeltaTime::from_sec(DELAY_EXTDNW2), dn2Dr);
  }
}

void Lmic::setupRx1DnData() {
  osjob.setCallbackFuture(&Lmic::processRx1DnData);
  setupRx1();
}

void Lmic::updataDone() {
  osjob.setCallbackFuture(&Lmic::setupRx1DnData);
  txDone(rxDelay);
}

// ========================================

void Lmic::buildDataFrame() {
  bool txdata = ((opmode & (OP_TXDATA | OP_POLL)) != OP_POLL);

  // Piggyback MAC options
  // Prioritize by importance
  uint8_t end = OFF_DAT_OPTS;
#if !defined(DISABLE_MCMD_DCAP_REQ)
  if (dutyCapAns) {
    frame[end] = MCMD_DCAP_ANS;
    end += 1;
    dutyCapAns = false;
  }
#endif // !DISABLE_MCMD_DCAP_REQ
#if !defined(DISABLE_MCMD_DN2P_SET)
  // RXParamSetupAns LoRaWAN™ Specification §5.4
  if (dn2Ans) {
    frame[end + 0] = MCMD_DN2P_ANS;
    frame[end + 1] = dn2Ans & ~MCMD_DN2P_ANS_RFU;
    end += 2;
    // dn2Ans reset when downlink packet receive
  }
#endif // !DISABLE_MCMD_DN2P_SET
  // DevStatusAns LoRaWAN™ Specification §5.5
  if (devsAns) { // answer to device status
    frame[end + 0] = MCMD_DEVS_ANS;
    frame[end + 1] = os_getBattLevel();
    // TODO check margin calculation (normaly 6bit signed integer)
    frame[end + 2] = margin;
    end += 3;
    devsAns = false;
  }
  if (ladrAns) { // answer to ADR change
    frame[end + 0] = MCMD_LADR_ANS;
    frame[end + 1] = ladrAns & ~MCMD_LADR_ANS_RFU;
    end += 2;
    ladrAns = 0;
  }
#if !defined(DISABLE_MCMD_SNCH_REQ)
  if (snchAns) {
    frame[end + 0] = MCMD_SNCH_ANS;
    frame[end + 1] = snchAns & ~MCMD_SNCH_ANS_RFU;
    end += 2;
    snchAns = 0;
  }
#endif // !DISABLE_MCMD_SNCH_REQ
  ASSERT(end <= OFF_DAT_OPTS + 16);

  uint8_t flen = end + (txdata ? 5 + pendTxLen : 4);
  if (flen > MAX_LEN_FRAME) {
    // Options and payload too big - delay payload
    txdata = 0;
    flen = end + 4;
  }
  frame[OFF_DAT_HDR] = HDR_FTYPE_DAUP | HDR_MAJOR_V1;
  frame[OFF_DAT_FCT] =
      (dnConf | adrEnabled | (adrAckReq >= 0 ? FCT_ADRARQ : 0) |
       (end - OFF_DAT_OPTS));
  wlsbf4(frame + OFF_DAT_ADDR, devaddr);

  if (txCnt == 0) {
    seqnoUp += 1;
  } else {
  }
  wlsbf2(frame + OFF_DAT_SEQNO, seqnoUp - 1);

  // Clear pending DN confirmation
  dnConf = 0;

  if (txdata) {
    if (pendTxConf) {
      // Confirmed only makes sense if we have a payload (or at least a port)
      frame[OFF_DAT_HDR] = HDR_FTYPE_DCUP | HDR_MAJOR_V1;
      if (txCnt == 0)
        txCnt = 1;
    }
    frame[end] = pendTxPort;
    std::copy(pendTxData, pendTxData + pendTxLen, frame + end + 1);
    aes.framePayloadEncryption(pendTxPort, devaddr, seqnoUp - 1, DIR_UP,
                               frame + end + 1, pendTxLen);
  }
  aes.appendMic(devaddr, seqnoUp - 1, DIR_UP, frame, flen);

  dataLen = flen;
}

// ================================================================================
//
// Join stuff
//
// ================================================================================

#if !defined(DISABLE_JOIN)
void Lmic::buildJoinRequest(uint8_t ftype) {
  // Do not use pendTxData since we might have a pending
  // user level frame in there. Use RX holding area instead.
  uint8_t *d = frame;
  d[OFF_JR_HDR] = ftype;
  artEuiCallBack(d + OFF_JR_ARTEUI);
  devEuiCallBack(d + OFF_JR_DEVEUI);
  wlsbf2(d + OFF_JR_DEVNONCE, devNonce);
  aes.appendMic0(d, LEN_JR);

  dataLen = LEN_JR;
  devNonce++;
}

void Lmic::startJoiningCallBack() { reportEvent(EV_JOINING); }

// Start join procedure if not already joined.
bool Lmic::startJoining() {
  if (devaddr == 0) {
    // There should be no TX/RX going on
    ASSERT((opmode & (OP_POLL | OP_TXRXPEND)) == 0);
    // Lift any previous duty limitation
    globalDutyRate = 0;
    // Cancel scanning
    opmode &= ~(OP_SCAN | OP_REJOIN | OP_LINKDEAD | OP_NEXTCHNL);
    // Setup state
    rejoinCnt = txCnt = 0;
    // remove rx 1 offset
    rx1DrOffset = 0;
    dr_t newDr;
    regionLMic.initJoinLoop(txChnl, adrTxPow, newDr, txend);
    setDrJoin(newDr);
    opmode |= OP_JOINING;
    // reportEvent will call engineUpdate which then starts sending JOIN
    // REQUESTS
    osjob.setCallbackRunnable(&Lmic::startJoiningCallBack);
    return true;
  }
  return false; // already joined
}
#endif // !DISABLE_JOIN

bool Lmic::processDnData() {
  ASSERT((opmode & OP_TXRXPEND) != 0);

  if (!decodeFrame()) {
    // first RX windows, do nothing wait for second windows.
    if ((txrxFlags & TXRX_DNW1) != 0)
      return false;

    // retry send if need
    if (txCnt != 0) {
      if (txCnt < TXCONF_ATTEMPTS) {
        txCnt += 1;
        setDrTxpow(lowerDR(datarate, TABLE_GET_U1(DRADJUST, txCnt)),
                   KEEP_TXPOW);
        // Schedule another retransmission
        txDelay(rxtime, RETRY_PERIOD_secs);
        opmode &= ~OP_TXRXPEND;
        engineUpdate();
        return true;
      }
      txrxFlags = TXRX_NACK | TXRX_NOPORT;
    } else {
      // Nothing received - implies no port
      txrxFlags = TXRX_NOPORT;
    }
    if (adrAckReq != LINK_CHECK_OFF)
      adrAckReq += 1;
    dataBeg = dataLen = 0;
  }

  opmode &= ~(OP_TXDATA | OP_TXRXPEND);
  if ((txrxFlags & (TXRX_DNW1 | TXRX_DNW2 | TXRX_PING)) != 0 &&
      (opmode & OP_LINKDEAD) != 0) {
    opmode &= ~OP_LINKDEAD;
    reportEvent(EV_LINK_ALIVE);
  }
  reportEvent(EV_TXCOMPLETE);
  // If we haven't heard from NWK in a while although we asked for a sign
  // assume link is dead - notify application and keep going
  if (adrAckReq > LINK_CHECK_DEAD) {
    // We haven't heard from NWK for some time although we
    // asked for a response for some time - assume we're disconnected. Lower
    // DR one notch.
    setDrTxpow(decDR(datarate), KEEP_TXPOW);
    adrAckReq = LINK_CHECK_CONT;
    opmode |= OP_REJOIN | OP_LINKDEAD;
    reportEvent(EV_LINK_DEAD);
  }
  return true;
}

// Decide what to do next for the MAC layer of a device
void Lmic::engineUpdate() {
#if LMIC_DEBUG_LEVEL > 0
  lmic_printf("%lu: engineUpdate, opmode=0x%x\n", os_getTime(), opmode);
#endif
  // Check for ongoing state: scan or TX/RX transaction
  if ((opmode & (OP_SCAN | OP_TXRXPEND | OP_SHUTDOWN)) != 0)
    return;

#if !defined(DISABLE_JOIN)
  if (devaddr == 0 && (opmode & OP_JOINING) == 0) {
    startJoining();
    return;
  }
#endif // !DISABLE_JOIN

  OsTime now = os_getTime();
  OsTime txbeg = now;

  if ((opmode & (OP_JOINING | OP_REJOIN | OP_TXDATA | OP_POLL)) != 0) {
    // Need to TX some data...
    // Assuming txChnl points to channel which first becomes available again.
    bool jacc = ((opmode & (OP_JOINING | OP_REJOIN)) != 0 ? 1 : 0);
#if LMIC_DEBUG_LEVEL > 1
    if (jacc)
      lmic_printf("%lu: Uplink join pending\n", os_getTime());
    else
      lmic_printf("%lu: Uplink data pending\n", os_getTime());
#endif
    // Find next suitable channel and return availability time
    if ((opmode & OP_NEXTCHNL) != 0) {
      txbeg = txend = regionLMic.nextTx(now, datarate, txChnl);
      opmode &= ~OP_NEXTCHNL;
#if LMIC_DEBUG_LEVEL > 1
      lmic_printf("%lu: Airtime available at %lu (channel duty limit)\n",
                  os_getTime(), txbeg);
#endif
    } else {
      txbeg = txend;
#if LMIC_DEBUG_LEVEL > 1
      lmic_printf("%lu: Airtime available at %lu (previously determined)\n",
                  os_getTime(), txbeg);
#endif
    }
    // Delayed TX or waiting for duty cycle?
    if ((globalDutyRate != 0 || (opmode & OP_RNDTX) != 0) &&
        (txbeg - globalDutyAvail) < 0) {
      txbeg = globalDutyAvail;
#if LMIC_DEBUG_LEVEL > 1
      lmic_printf("%lu: Airtime available at %lu (global duty limit)\n",
                  os_getTime(), txbeg);
#endif
    }
    // Earliest possible time vs overhead to setup radio
    if (txbeg - (now + TX_RAMPUP) < 0) {
#if LMIC_DEBUG_LEVEL > 1
      lmic_printf("%lu: Ready for uplink\n", os_getTime());
#endif
      // We could send right now!
      txbeg = now;
      dr_t txdr = datarate;
#if !defined(DISABLE_JOIN)
      if (jacc) {
        uint8_t ftype;
        if ((opmode & OP_REJOIN) != 0) {
          txdr = lowerDR(txdr, rejoinCnt);
          ftype = HDR_FTYPE_REJOIN;
        } else {
          ftype = HDR_FTYPE_JREQ;
        }
        buildJoinRequest(ftype);
        osjob.setCallbackFuture(&Lmic::jreqDone);
      } else
#endif // !DISABLE_JOIN
      {
        if (seqnoDn >= 0xFFFFFF80) {
          // Imminent roll over - proactively reset MAC
          // Device has to react! NWK will not roll over and just stop sending.
          // Thus, we have N frames to detect a possible lock up.
          osjob.setCallbackRunnable(&Lmic::runReset);
          return;
        }
        if ((txCnt == 0 && seqnoUp == 0xFFFFFFFF)) {
          // Roll over of up seq counter
          // Do not run RESET event callback from here!
          // App code might do some stuff after send unaware of RESET.
          osjob.setCallbackRunnable(&Lmic::runReset);
          return;
        }
        buildDataFrame();
        osjob.setCallbackFuture(&Lmic::updataDone);
      }
      rps = setCr(updr2rps(txdr), errcr);
      dndr = txdr; // carry TX datarate (can be != datarate) over to
                   // txDone/setupRx1
      opmode = (opmode & ~(OP_POLL | OP_RNDTX)) | OP_TXRXPEND | OP_NEXTCHNL;
      OsDeltaTime airtime = calcAirTime(rps, dataLen);
      regionLMic.updateTx(txbeg, globalDutyRate, airtime, txChnl, freq, txpow,
                          globalDutyAvail);
      radio_tx();
      return;
    }
#if LMIC_DEBUG_LEVEL > 1
    lmic_printf("%lu: Uplink delayed until %lu\n", os_getTime(), txbeg);
#endif
    // Cannot yet TX
    if ((opmode & OP_TRACK) == 0)
      goto txdelay; // We don't track the beacon - nothing else to do - so wait
                    // for the time to TX
                    // Consider RX tasks
                    // if( txbeg == 0 ) // zero indicates no TX pending
    //    txbeg += 1;  // TX delayed by one tick (insignificant amount of time)
  } else {
    // No TX pending - no scheduled RX
    if ((opmode & OP_TRACK) == 0)
      return;
  }

txdelay:
  osjob.setTimedCallback(txbeg - TX_RAMPUP, &Lmic::runEngineUpdate);
}

void Lmic::setAdrMode(bool enabled) { adrEnabled = enabled ? FCT_ADREN : 0; }

void Lmic::shutdown() {
  osjob.clearCallback();
  radio_rst();
  opmode |= OP_SHUTDOWN;
}

void Lmic::reset() {
  radio_rst();
  osjob.clearCallback();

  devaddr = 0;
  devNonce = hal_rand2();
  opmode = OP_NONE;
  errcr = CR_4_5;
  adrEnabled = FCT_ADREN;
  dn2Dr = DR_DNW2;     // we need this for 2nd DN window of join accept
  dn2Freq = FREQ_DNW2; // ditto
  rxDelay = OsDeltaTime::from_sec(DELAY_DNW1);

  regionLMic.initDefaultChannels(true);
}

void Lmic::init(void) { opmode = OP_SHUTDOWN; }

void Lmic::clrTxData(void) {
  opmode &= ~(OP_TXDATA | OP_TXRXPEND | OP_POLL);
  pendTxLen = 0;
  if ((opmode & (OP_JOINING | OP_SCAN)) != 0) // do not interfere with JOINING
    return;
  osjob.clearCallback();
  radio_rst();
  engineUpdate();
}

void Lmic::setTxData(void) {
  opmode |= OP_TXDATA;
  if ((opmode & OP_JOINING) == 0)
    txCnt = 0; // cancel any ongoing TX/RX retries
  engineUpdate();
}

//
int Lmic::setTxData2(uint8_t port, uint8_t *data, uint8_t dlen,
                     bool confirmed) {
  if (dlen > sizeof(pendTxData))
    return -2;
  if (data)
    std::copy(data, data + dlen, pendTxData);
  pendTxConf = confirmed;
  pendTxPort = port;
  pendTxLen = dlen;
  setTxData();
  return 0;
}

// Send a payload-less message to signal device is alive
void Lmic::sendAlive() {
  opmode |= OP_POLL;
  engineUpdate();
}

// Check if other networks are around.
void Lmic::tryRejoin(void) {
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
//! \param devaddr the 32 bit session address of the device. It is strongly
//! recommended
//!    to ensure that different devices use different numbers with high
//!    probability.
//! \param nwkKey  the 16 byte network session key used for message integrity.
//! \param artKey  the 16 byte application router session key used for message
//! confidentiality.
void Lmic::setSession(uint32_t netid, devaddr_t devaddr, uint8_t *nwkKey,
                      uint8_t *artKey) {
  this->netid = netid;
  this->devaddr = devaddr;
  if (nwkKey)
    aes.setNetworkSessionKey(nwkKey);
  if (artKey)
    aes.setApplicationSessionKey(artKey);

  regionLMic.initDefaultChannels(false);

  opmode &= ~(OP_JOINING | OP_TRACK | OP_REJOIN | OP_TXRXPEND | OP_PINGINI);
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
void Lmic::setLinkCheckMode(bool enabled) {
  adrAckReq = enabled ? LINK_CHECK_INIT : LINK_CHECK_OFF;
}

// Sets the max clock error to compensate for (defaults to 0, which
// allows for +/- 640 at SF7BW250). MAX_CLOCK_ERROR represents +/-100%,
// so e.g. for a +/-1% error you would pass MAX_CLOCK_ERROR * 1 / 100.
void Lmic::setClockError(uint16_t error) { clockError = error; }

void Lmic::nextTask() { osjob.setRunnable(); }
