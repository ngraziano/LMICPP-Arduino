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
#include "lorawanpacket.h"
#include "radio.h"
#include <algorithm>

using namespace lorawan;

#if !defined(MINRX_SYMS)
#define MINRX_SYMS 5
#endif // !defined(MINRX_SYMS)
#define PAMBL_SYMS 8
#define PAMBL_FSK 5
#define PRERX_FSK 1
#define RXLEN_FSK (1 + 5 + 2)

#define DELAY_JACC1_osticks OsDeltaTime::from_sec(DELAY_JACC1)
#define DELAY_JACC2_osticks OsDeltaTime::from_sec(DELAY_JACC2)

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

int16_t getSensitivity(rps_t rps) {
  return -141 + TABLE_GET_U1_TWODIM(SENSITIVITY, rps.sf, rps.bwRaw);
}

OsDeltaTime Lmic::calcAirTime(rps_t rps, uint8_t plen) {
  // 0,1,2 = 125,250,500kHz
  const uint8_t bw = rps.bwRaw;
  //  7..12 = SF7..12
  const uint8_t sf = 7 + rps.sf - SF7;
  const uint8_t sfx = 4 * sf;
  const uint8_t optimiseLowSf = (rps.sf >= SF11 ? 8 : 0);
  const uint8_t q = sfx - optimiseLowSf;

  int16_t tmp = 8 * plen - sfx + 28 + (rps.nocrc ? 0 : 16) - (rps.ih ? 20 : 0);
  if (tmp > 0) {
    tmp = (tmp + q - 1) / q;
    tmp *= (rps.crRaw + 5);
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
  uint8_t sfx2 = sf - (3 + 2) - bw;
  uint16_t div = 15625;
  if (sfx2 > 4) {
    // prevent 32bit signed int overflow in last step
    div >>= sfx2 - 4;
    sfx2 = 4;
  }
  // Need 32bit arithmetic for this last step
  OsDeltaTime val = OsDeltaTime((((int32_t)tmp << sfx2) * OSTICKS_PER_SEC + div / 2) / div);
  PRINT_DEBUG_1("Time on air : %i ms", val.to_ms());
  return val;
}

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

void Lmic::txDelay(OsTime reftime, uint8_t secSpan) {
  const auto delayRef = reftime + OsDeltaTime::rnd_delay(rand, secSpan);
  if (globalDutyRate == 0 || (delayRef - globalDutyAvail) > OsDeltaTime(0)) {
    globalDutyAvail = delayRef;
  }
}

void Lmic::setDrJoin(dr_t dr) { datarate = dr; }

void Lmic::setDrTxpow(uint8_t dr, int8_t pow) {
  if (pow != KEEP_TXPOW)
    adrTxPow = pow;
  if (datarate != dr) {
    datarate = dr;
    opmode |= OpState::NEXTCHNL;
  }
}

void Lmic::runEngineUpdate() { engineUpdate(); }

void Lmic::reportEvent(EventType ev) {
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
  reportEvent(EventType::RESET);
}

void Lmic::stateJustJoined() {
  seqnoDn = 0;
  seqnoUp = 0;
  rejoinCnt = 0;
  dnConf = 0;
  ladrAns = 0;
  devsAns = false;
  rxTimingSetupAns = false;
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
  dn2Dr = defaultRX2Dr();
  dn2Freq = defaultRX2Freq();
}

void Lmic::parseMacCommands(const uint8_t *const opts, uint8_t const olen) {
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
      const uint8_t p1 = opts[oidx + 1]; // txpow + DR
      const uint16_t chMask =
          rlsbf2(&opts[oidx + 2]); // list of enabled channels
      const uint8_t chMaskCntl =
          opts[oidx + 4] & MCMD_LADR_CHPAGE_MASK; // channel page
      const uint8_t nbTrans =
          opts[oidx + 4] & MCMD_LADR_REPEAT_MASK; // up repeat count
      oidx += 5;

      ladrAns = 0x80 | // Include an answer into next frame up
                MCMD_LADR_ANS_POWACK | MCMD_LADR_ANS_CHACK |
                MCMD_LADR_ANS_DRACK;
      if (!mapChannels(chMaskCntl, chMask)) {
        PRINT_DEBUG_1("ADR REQ Invalid map channel maskCtnl=%i, mask=%i",
                      chMaskCntl, chMask);
        ladrAns &= ~MCMD_LADR_ANS_CHACK;
      }
      dr_t dr = (dr_t)(p1 >> MCMD_LADR_DR_SHIFT);
      if (!validDR(dr)) {
        PRINT_DEBUG_1("ADR REQ Invalid dr %i", dr);
        ladrAns &= ~MCMD_LADR_ANS_DRACK;
      }
      if ((ladrAns & 0x7F) ==
          (MCMD_LADR_ANS_POWACK | MCMD_LADR_ANS_CHACK | MCMD_LADR_ANS_DRACK)) {
        // Nothing went wrong - use settings
        upRepeat = nbTrans;
        PRINT_DEBUG_1("ADR REQ Change dr to %i, power to %i", dr,
                      p1 & MCMD_LADR_POW_MASK);
        setDrTxpow(dr, pow2dBm(p1));
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
      const dr_t dr = (dr_t)(opts[oidx + 1] & 0x0F);
      const uint8_t newRx1DrOffset = ((opts[oidx + 1] & 0x70) >> 4);
      const uint32_t newfreq = convFreq(&opts[oidx + 2]);
      dn2Ans = 0x80; // answer pending
      if (validRx1DrOffset(newRx1DrOffset))
        dn2Ans |= MCMD_DN2P_ANS_RX1DrOffsetAck;
      if (validDR(dr))
        dn2Ans |= MCMD_DN2P_ANS_DRACK;
      if (newfreq != 0)
        dn2Ans |= MCMD_DN2P_ANS_CHACK;
      if (dn2Ans == (0x80 | MCMD_DN2P_ANS_RX1DrOffsetAck | MCMD_DN2P_ANS_DRACK |
                     MCMD_DN2P_ANS_CHACK)) {
        dn2Dr = dr;
        dn2Freq = newfreq;
        rx1DrOffset = newRx1DrOffset;
      }
#endif // !DISABLE_MCMD_DN2P_SET
      oidx += 5;
      continue;
    }
    // DutyCycleReq LoRaWAN™ Specification §5.3
    case MCMD_DCAP_REQ: {
#if !defined(DISABLE_MCMD_DCAP_REQ)
      const uint8_t cap = opts[oidx + 1];

      // cap=0xFF is not in specification...
      // A value cap=0xFF means device is OFF unless enabled again manually.
      if (cap == 0xFF)
        opmode |= OpState::SHUTDOWN; // stop any sending

      globalDutyRate = cap & 0xF;
      globalDutyAvail = os_getTime();
      dutyCapAns = true;
#endif // !DISABLE_MCMD_DCAP_REQ
      oidx += 2;
      continue;
    }
    // NewChannelReq LoRaWAN™ Specification §5.6
    case MCMD_SNCH_REQ: {
#if !defined(DISABLE_MCMD_SNCH_REQ)
      const uint8_t chidx = opts[oidx + 1];               // channel
      const uint32_t newfreq = convFreq(&opts[oidx + 2]); // freq
      const uint8_t drs = opts[oidx + 5];                 // datarate span
      snchAns = 0x80;
      if (newfreq != 0 &&
          setupChannel(chidx, newfreq, DR_RANGE_MAP(drs & 0xF, drs >> 4), -1))
        snchAns |= MCMD_SNCH_ANS_DRACK | MCMD_SNCH_ANS_FQACK;
#endif // !DISABLE_MCMD_SNCH_REQ
      oidx += 6;
      continue;
    }
    // RXTimingSetupReq LoRaWAN™ Specification §5.7
    case MCMD_RXTimingSetup_REQ: {
      uint8_t newDelay = opts[oidx + 1] & 0x0F;
      if (newDelay == 0)
        newDelay = 1;
      rxDelay = OsDeltaTime::from_sec(newDelay);
      rxTimingSetupAns = true;
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
  const char *window = (txrxFlags & TxRxStatus::DNW1)
                           ? "RX1"
                           : ((txrxFlags & TxRxStatus::DNW2) ? "RX2" : "Other");
#endif

  if (dataLen == 0) {
    PRINT_DEBUG_1("No downlink data, window=%s", window);
    return false;
  }

  uint8_t *const d = frame;
  const uint8_t hdr = d[0];
  const uint8_t ftype = hdr & HDR_FTYPE;
  const uint8_t dlen = dataLen;

  if (dlen < OFF_DAT_OPTS + 4 || (hdr & HDR_MAJOR) != HDR_MAJOR_V1 ||
      (ftype != HDR_FTYPE_DADN && ftype != HDR_FTYPE_DCDN)) {
    // Basic sanity checks failed
    PRINT_DEBUG_1("Invalid downlink, window=%s", window);
    dataLen = 0;
    return false;
  }
  // Validate exact frame length
  // Note: device address was already read+evaluated in order to arrive here.
  const uint8_t fct = d[OFF_DAT_FCT];
  const uint32_t addr = rlsbf4(&d[OFF_DAT_ADDR]);

  const uint8_t olen = fct & FCT_OPTLEN;
  const bool ackup = (fct & FCT_ACK) != 0 ? true : false; // ACK last up frame
  const uint8_t poff = OFF_DAT_OPTS + olen;
  const uint8_t pend = dlen - MIC_LEN; // MIC

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

  uint32_t seqno = rlsbf2(&d[OFF_DAT_SEQNO]);
  seqno = seqnoDn + (uint16_t)(seqno - seqnoDn);

  if (!aes.verifyMic(devaddr, seqno, PktDir::DOWN, d, dlen)) {
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
    opmode |= OpState::POLL;

  // We heard from network
  rejoinCnt = 0;
  if (adrAckReq != LINK_CHECK_OFF)
    adrAckReq = LINK_CHECK_INIT;

  // Process OPTS
  const int16_t m = rssi - RSSI_OFF - getSensitivity(rps);
  margin = m < 0 ? 0 : m > 254 ? 254 : m;

  parseMacCommands(d + OFF_DAT_OPTS, olen);

  uint8_t port = -1;
  if (!replayConf) {
    // Handle payload only if not a replay
    if (pend > poff) {
      port = d[poff];
      dataBeg = poff + 1;
      dataLen = pend - dataBeg;
      // Decrypt payload - if any
      aes.framePayloadEncryption(port, devaddr, seqno, PktDir::DOWN,
                                 d + dataBeg, dataLen);
      txrxFlags |= TxRxStatus::PORT;

      if (port == 0) {
        parseMacCommands(d + dataBeg, dataLen);
      }
    } else {
      txrxFlags |= TxRxStatus::NOPORT;
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
    txrxFlags |= ackup ? TxRxStatus::ACK : TxRxStatus::NACK;

#if !defined(DISABLE_MCMD_DN2P_SET)
  // stop sending RXParamSetupAns when receive dowlink message
  dn2Ans = 0;
#endif
  // stop sending rxTimingSetupAns when receive dowlink message
  rxTimingSetupAns = false;

  PRINT_DEBUG_1("Received downlink, window=%s, port=%d, ack=%d", window, port,
                ackup);
  return true;
}

// ================================================================================
// TX/RX transaction support

void Lmic::setupRx2() {
  txrxFlags = TxRxStatus::DNW2;
  rps = dndr2rps(dn2Dr);
  freq = dn2Freq;
  dataLen = 0;
  radio.rx(freq, rps, rxsyms, rxtime);
}

void Lmic::schedRx12(OsDeltaTime delay, uint8_t dr) {
  PRINT_DEBUG_2("SchedRx RX1/2.");

  // Half symbol time for the data rate.
  const OsDeltaTime hsym = dr2hsym(dr);

  rxsyms = MINRX_SYMS;

  // If a clock error is specified, compensate for it by extending the
  // receive window
  if (clockError != 0) {
    // Calculate how much the clock will drift maximally after delay has
    // passed. This indicates the amount of time we can be early
    // _or_ late.
    const OsDeltaTime drift =
        OsDeltaTime(delay.tick() * clockError / MAX_CLOCK_ERROR);

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
  PRINT_DEBUG_1("Rx delay : %i ms", (rxtime - txend).to_ms());

  osjob.setTimed(rxtime - RX_RAMPUP);
}

void Lmic::setupRx1() {
  txrxFlags = TxRxStatus::DNW1;
  dataLen = 0;
  radio.rx(freq, rps, rxsyms, rxtime);
}

// Called by HAL once TX complete and delivers exact end of TX time stamp in
// rxtime
void Lmic::txDone(OsDeltaTime delay) {
  // Change RX frequency / rps (US only) before we increment txChnl
  setRx1Params();
  rps = dndr2rps(dndr);
  schedRx12(delay, dndr);
}

// ======================================== Join frames

#if !defined(DISABLE_JOIN)
void Lmic::onJoinFailed() {
  // Notify app - must call reset() to stop joining
  // otherwise join procedure continues.
  reportEvent(EventType::JOIN_FAILED);
}

bool Lmic::processJoinAcceptNoJoinFrame() {
  if (opmode & OpState::REJOIN) {
    // REJOIN attempt for roaming
    // rejoin fail, continue normal operation
    opmode &= ~OpState::REJOIN;
    opmode &= ~OpState::TXRXPEND;
    if (rejoinCnt < 10)
      rejoinCnt++;
    reportEvent(EventType::REJOIN_FAILED);
    return true;
  }
  opmode &= ~OpState::TXRXPEND;
  // Clear NEXTCHNL because join state engine controls channel hopping
  opmode &= ~OpState::NEXTCHNL;
  const bool succes = nextJoinState();
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
  ASSERT(opmode & OpState::TXRXPEND);

  if (dataLen == 0) {
    return processJoinAcceptNoJoinFrame();
  }

  const uint8_t hdr = frame[0];
  const uint8_t dlen = dataLen;

  if ((dlen != join_accept::lengths::total &&
       dlen != join_accept::lengths::totalWithOptional) ||
      (hdr & (HDR_FTYPE | HDR_MAJOR)) != (HDR_FTYPE_JACC | HDR_MAJOR_V1)) {
    PRINT_DEBUG_1("Join Accept BAD Length %i or bad header %i ", dlen, hdr);

    // unexpected frame
    if (txrxFlags & TxRxStatus::DNW1)
      return false;
    return processJoinAcceptNoJoinFrame();
  }
  aes.encrypt(frame + 1, dlen - 1);
  if (!aes.verifyMic0(frame, dlen)) {
    PRINT_DEBUG_1("Join Accept BAD MIC", dlen);

    // bad mic
    if (txrxFlags & TxRxStatus::DNW1)
      return false;
    return processJoinAcceptNoJoinFrame();
  }

  devaddr = rlsbf4(frame + join_accept::offset::devAddr);
  netid = rlsbf4(frame + join_accept::offset::netId) & 0xFFFFFF;

  initDefaultChannels(false);

  if (dlen > join_accept::lengths::total) {
    // some region just ignore cflist.
    handleCFList(frame + join_accept::offset::cfList);
  }

  // already incremented when JOIN REQ got sent off
  aes.sessKeys(devNonce - 1, frame + join_accept::offset::appNonce);

  ASSERT(opmode & (OpState::JOINING | OpState::REJOIN));
  if (opmode & OpState::REJOIN) {
    // Lower DR every try below current UP DR
    // so adjust the current datarate to success join
    datarate = lowerDR(datarate, rejoinCnt);
  }
  opmode &= ~(OpState::JOINING | OpState::REJOIN | OpState::TXRXPEND) |
            OpState::NEXTCHNL;
  txCnt = 0;
  stateJustJoined();

  const uint8_t dlSettings = frame[join_accept::offset::dlSettings];
  dn2Dr = dlSettings & 0x0F;
  rx1DrOffset = (dlSettings >> 4) & 0x7;

  const uint8_t configuredRxDelay = frame[join_accept::offset::rxDelay];
  if (configuredRxDelay == 0) {
    rxDelay = OsDeltaTime::from_sec(DELAY_DNW1);
  } else {
    rxDelay = OsDeltaTime::from_sec(configuredRxDelay);
  }
  reportEvent(EventType::JOINED);
  return true;
}

void Lmic::processRx2Jacc() {
  if (dataLen == 0)
    txrxFlags = TxRxStatus::NONE; // nothing in 1st/2nd DN slot
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
    // nothing in 1st/2nd DN slot
    // It could be that the gateway *is* sending a reply, but we
    // just didn't pick it up. To avoid TX'ing again while the
    // gateay is not listening anyway, delay the next transmission
    // until DNW2_SAFETY_ZONE from now, and add up to 2 seconds of
    // extra randomization.
    txDelay(os_getTime() + getDwn2SafetyZone(), 2);
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
  bool txdata = static_cast<bool>(opmode & OpState::TXDATA);

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
  if (rxTimingSetupAns) {
    frame[end + 0] = MCMD_RXTimingSetup_ANS;
    end += 1;
    // rxTimingSetupAns reset when downlink packet receive
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
      (dnConf | (adrAckReq != LINK_CHECK_OFF ? FCT_ADREN : 0) |
       (adrAckReq >= 0 ? FCT_ADRARQ : 0) | (end - OFF_DAT_OPTS));
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
    aes.framePayloadEncryption(pendTxPort, devaddr, seqnoUp - 1, PktDir::UP,
                               frame + end + 1, pendTxLen);
  }
  aes.appendMic(devaddr, seqnoUp - 1, PktDir::UP, frame, flen);

  dataLen = flen;
}

// ================================================================================
//
// Join stuff
//
// ================================================================================

#if !defined(DISABLE_JOIN)
void Lmic::buildJoinRequest() {
  // Do not use pendTxData since we might have a pending
  // user level frame in there. Use RX holding area instead.
  frame[join_request::offset::MHDR] = HDR_FTYPE_JREQ;
  artEuiCallBack(frame + join_request::offset::appEUI);
  devEuiCallBack(frame + join_request::offset::devEUI);
  wlsbf2(frame + join_request::offset::devNonce, devNonce);
  aes.appendMic0(frame, join_request::lengths::totalWithMic);

  dataLen = join_request::lengths::totalWithMic;
  devNonce++;
}

void Lmic::startJoiningCallBack() { reportEvent(EventType::JOINING); }

// Start join procedure if not already joined.
bool Lmic::startJoining() {
  if (devaddr == 0) {
    // There should be no TX/RX going on
    ASSERT((opmode & (OpState::POLL | OpState::TXRXPEND)) == 0);
    // Lift any previous duty limitation
    globalDutyRate = 0;
    // Cancel scanning
    opmode &= ~(OpState::REJOIN | OpState::LINKDEAD | OpState::NEXTCHNL);
    // Setup state
    rejoinCnt = txCnt = 0;
    // remove rx 1 offset
    rx1DrOffset = 0;

    initJoinLoop();

    opmode |= OpState::JOINING;
    // reportEvent will call engineUpdate which then starts sending JOIN
    // REQUESTS
    osjob.setCallbackRunnable(&Lmic::startJoiningCallBack);
    return true;
  }
  return false; // already joined
}
#endif // !DISABLE_JOIN

bool Lmic::processDnData() {
  ASSERT((opmode & OpState::TXRXPEND) != 0);

  if (!decodeFrame()) {
    // first RX windows, do nothing wait for second windows.
    if (txrxFlags & TxRxStatus::DNW1)
      return false;

    // retry send if need
    if (txCnt != 0) {
      if (txCnt < TXCONF_ATTEMPTS) {
        txCnt += 1;
        setDrTxpow(lowerDR(datarate, TABLE_GET_U1(DRADJUST, txCnt)),
                   KEEP_TXPOW);
        // Schedule another retransmission
        txDelay(rxtime, RETRY_PERIOD_secs);
        opmode &= ~OpState::TXRXPEND;
        engineUpdate();
        return true;
      }
      txrxFlags = TxRxStatus::NACK | TxRxStatus::NOPORT;
    } else {
      // Nothing received - implies no port
      txrxFlags = TxRxStatus::NOPORT;
    }
    if (adrAckReq != LINK_CHECK_OFF)
      adrAckReq += 1;
    dataBeg = dataLen = 0;
  }

  opmode &= ~(OpState::TXDATA | OpState::TXRXPEND);
  if ((txrxFlags & (TxRxStatus::DNW1 | TxRxStatus::DNW2)) &&
      (opmode & OpState::LINKDEAD)) {
    opmode &= ~OpState::LINKDEAD;
    reportEvent(EventType::LINK_ALIVE);
  }
  reportEvent(EventType::TXCOMPLETE);
  // If we haven't heard from NWK in a while although we asked for a sign
  // assume link is dead - notify application and keep going
  if (adrAckReq > LINK_CHECK_DEAD) {
    // We haven't heard from NWK for some time although we
    // asked for a response for some time - assume we're disconnected.
    // Restore max power if it not the case
    if (adrTxPow != pow2dBm(0)) {
      setDrTxpow(datarate, pow2dBm(0));
      opmode |= OpState::LINKDEAD;
    } else if (decDR(datarate) != datarate) {
      // Lower DR one notch.
      setDrTxpow(decDR(datarate), KEEP_TXPOW);
      opmode |= OpState::LINKDEAD;
    } else {
      // we are at max pow and max DR
      opmode |= OpState::REJOIN | OpState::LINKDEAD;
    }
    adrAckReq = LINK_CHECK_CONT;
    reportEvent(EventType::LINK_DEAD);
  }
  return true;
}

// Decide what to do next for the MAC layer of a device
void Lmic::engineUpdate() {
  PRINT_DEBUG_1("engineUpdate, opmode=0x%hhx.", opmode);
  // Check for ongoing state: scan or TX/RX transaction
  if (opmode & (OpState::TXRXPEND | OpState::SHUTDOWN))
    return;

#if !defined(DISABLE_JOIN)
  if (devaddr == 0 && !(opmode & OpState::JOINING)) {
    startJoining();
    return;
  }
#endif // !DISABLE_JOIN

  const OsTime now = os_getTime();
  OsTime txbeg = now;

  if (opmode &
      (OpState::JOINING | OpState::REJOIN | OpState::TXDATA | OpState::POLL)) {
    // Need to TX some data...
    // Assuming txChnl points to channel which first becomes available again.
    const bool jacc =
        static_cast<bool>(opmode & (OpState::JOINING | OpState::REJOIN));
#if LMIC_DEBUG_LEVEL > 1
    if (jacc)
      lmic_printf("%lu: Uplink join pending\n", os_getTime());
    else
      lmic_printf("%lu: Uplink data pending\n", os_getTime());
#endif
    // Find next suitable channel and return availability time
    if (opmode & OpState::NEXTCHNL) {
      txbeg = txend = nextTx(now);
      opmode &= ~OpState::NEXTCHNL;
      PRINT_DEBUG_2("Airtime available at %lu (channel duty limit)", txbeg);
    } else {
      txbeg = txend;
      PRINT_DEBUG_2("Airtime available at %lu (previously determined)", txbeg);
    }
    // Delayed TX or waiting for duty cycle?
    if ((txbeg - globalDutyAvail) < OsDeltaTime(0)) {
      txbeg = globalDutyAvail;
      PRINT_DEBUG_2("Airtime available at %lu (global duty limit)", txbeg);
    }
    // Earliest possible time vs overhead to setup radio
    if (txbeg - (now + TX_RAMPUP) < OsDeltaTime(0)) {
      PRINT_DEBUG_1("Ready for uplink");
      // We could send right now!
      txbeg = now;
      dr_t txdr = datarate;
#if !defined(DISABLE_JOIN)
      if (jacc) {
        if (opmode & OpState::REJOIN) {
          txdr = lowerDR(txdr, rejoinCnt);
        }
        buildJoinRequest();
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
      rps = updr2rps(txdr);
      dndr = txdr; // carry TX datarate (can be != datarate) over to
                   // txDone/setupRx1

      opmode =
          (opmode & ~(OpState::POLL)) | OpState::TXRXPEND | OpState::NEXTCHNL;
      OsDeltaTime airtime = calcAirTime(rps, dataLen);
      updateTx(txbeg, airtime);
      radio.tx(freq, rps, txpow + antennaPowerAdjustment, frame, dataLen);
      return;
    }
    PRINT_DEBUG_1("Uplink delayed until %lu", txbeg);
    // Cannot yet TX
    //  wait for the time to TX
    osjob.setTimedCallback(txbeg - TX_RAMPUP, &Lmic::runEngineUpdate);
  } else {
    // No TX pending - no scheduled RX
    return;
  }
}


void Lmic::setAntennaPowerAdjustment(int8_t power) {
  antennaPowerAdjustment = power;
}

void Lmic::shutdown() {
  osjob.clearCallback();
  radio.rst();
  opmode |= OpState::SHUTDOWN;
}

void Lmic::reset() {
  radio.rst();
  osjob.clearCallback();
  rps = rps_t(0);
  devaddr = 0;
  devNonce = rand.uint16();
  opmode = OpState::NONE;
  // we need this for 2nd DN window of join accept
  dn2Dr = defaultRX2Dr();
  dn2Freq = defaultRX2Freq();
  rxDelay = OsDeltaTime::from_sec(DELAY_DNW1);

  initDefaultChannels(true);
}

void Lmic::init(void) {
  radio.init();
  rand.init(radio);
  opmode = OpState::SHUTDOWN;
}

void Lmic::clrTxData(void) {
  opmode &= ~(OpState::TXDATA | OpState::TXRXPEND | OpState::POLL);
  pendTxLen = 0;
  if (opmode & OpState::JOINING) // do not interfere with JOINING
    return;
  osjob.clearCallback();
  radio.rst();
  engineUpdate();
}

void Lmic::setTxData(void) {
  opmode |= OpState::TXDATA;
  if (!(opmode & OpState::JOINING))
    txCnt = 0; // cancel any ongoing TX/RX retries
  engineUpdate();
}

//
int8_t Lmic::setTxData2(uint8_t port, uint8_t *data, uint8_t dlen,
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
  opmode |= OpState::POLL;
  engineUpdate();
}

// Check if other networks are around.
void Lmic::tryRejoin(void) {
  opmode |= OpState::REJOIN;
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

  initDefaultChannels(false);

  opmode &= ~(OpState::JOINING | OpState::REJOIN | OpState::TXRXPEND);
  opmode |= OpState::NEXTCHNL;
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
void Lmic::setClockError(uint8_t error) { clockError = error; }

rps_t Lmic::updr2rps(dr_t dr) const {
  rps_t result(getRawRps(dr));
  return result;
}

rps_t Lmic::dndr2rps(dr_t dr) const {
  auto val = updr2rps(dr);
  val.nocrc = 1;
  return val;
}

bool Lmic::isFasterDR(dr_t dr1, dr_t dr2) const { return dr1 > dr2; }

bool Lmic::isSlowerDR(dr_t dr1, dr_t dr2) const { return dr1 < dr2; }

// increase data rate
dr_t Lmic::incDR(dr_t dr) const { return validDR(dr + 1) ? dr + 1 : dr; }

// decrease data rate
dr_t Lmic::decDR(dr_t dr) const { return validDR(dr - 1) ? dr - 1 : dr; }

// in range
bool Lmic::validDR(dr_t dr) const { return getRawRps(dr) != ILLEGAL_RPS; }

// decrease data rate by n steps
dr_t Lmic::lowerDR(dr_t dr, uint8_t n) const {
  while (n--) {
    dr = decDR(dr);
  };
  return dr;
}

void Lmic::io_check() {
  if (radio.io_check(frame, dataLen, txend, rxtime, rps)) {
    // if radio task ended, activate next job.
    osjob.setRunnable();
  }
}

void Lmic::store_trigger() { radio.store_trigger(); }

Lmic::Lmic(lmic_pinmap const &pins) : radio(pins), rand(aes) {}