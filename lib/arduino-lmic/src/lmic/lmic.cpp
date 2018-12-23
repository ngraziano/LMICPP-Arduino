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

const uint8_t MINRX_SYMS = 5;
const uint8_t PAMBL_SYMS = 8;

// ================================================================================
// BEG OS - default implementations for certain OS suport functions

#if !defined(os_getBattLevel)
uint8_t os_getBattLevel() { return MCMD_DEVS_BATT_NOINFO; }
#endif

// END OS - default implementations for certain OS suport functions
// ================================================================================

// ================================================================================
// BEG LORA

static CONST_TABLE(uint8_t, SENSITIVITY)[7][3] = {
    // TODO check where this value come from.
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
  OsDeltaTime val =
      OsDeltaTime((((int32_t)tmp << sfx2) * OSTICKS_PER_SEC + div / 2) / div);
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
  if (globalDutyRate == 0 || (delayRef > globalDutyAvail)) {
    globalDutyAvail = delayRef;
  }
}

void Lmic::setDrJoin(dr_t dr) { datarate = dr; }

void Lmic::setDrTxpow(uint8_t dr) {
  if (datarate != dr) {
    datarate = dr;
    opmode.set(OpState::NEXTCHNL);
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
      // TODO add a test on power validPower
      if ((ladrAns & 0x7F) ==
          (MCMD_LADR_ANS_POWACK | MCMD_LADR_ANS_CHACK | MCMD_LADR_ANS_DRACK)) {
        // Nothing went wrong - use settings
        upRepeat = nbTrans;
        const uint8_t txPowerIndex =
            (p1 & MCMD_LADR_POW_MASK) >> MCMD_LADR_POW_SHIFT;
        PRINT_DEBUG_1("ADR REQ Change dr to %i, power to %i", dr, txPowerIndex);
        adrTxPow = pow2dBm(txPowerIndex);
        setDrTxpow(dr);
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
          setupChannel(chidx, newfreq, dr_range_map(drs & 0xF, drs >> 4)))
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
  const char *window =
      (txrxFlags.test(TxRxStatus::DNW1))
          ? "RX1"
          : ((txrxFlags.test(TxRxStatus::DNW2)) ? "RX2" : "Other");
#endif

  if (dataLen == 0) {
    PRINT_DEBUG_1("No downlink data, window=%s", window);
    return false;
  }

  uint8_t *const d = frame;
  const uint8_t hdr = d[0];
  const uint8_t ftype = hdr & mhdr::ftype_mask;
  const uint8_t dlen = dataLen;

  if (dlen < mac_payload::offsets::fopts + lengths::MIC ||
      (hdr & mhdr::major_mask) != mhdr::major_v1 ||
      (ftype != mhdr::ftype_data_down && ftype != mhdr::ftype_data_conf_down)) {
    // Basic sanity checks failed
    PRINT_DEBUG_1("Invalid downlink, window=%s", window);
    return false;
  }
  // Validate exact frame length
  // Note: device address was already read+evaluated in order to arrive here.
  const uint8_t fct = d[mac_payload::offsets::fctrl];
  const uint32_t addr = rlsbf4(&d[mac_payload::offsets::devAddr]);

  const uint8_t olen = fct & FCT_OPTLEN;
  const bool ackup = (fct & FCT_ACK) != 0 ? true : false; // ACK last up frame
  const uint8_t poff = mac_payload::offsets::fopts + olen;
  const uint8_t pend = dlen - lengths::MIC; // MIC

  if (addr != devaddr) {
    PRINT_DEBUG_1("Invalid address, window=%s", window);
    return false;
  }
  if (poff > pend) {
    PRINT_DEBUG_1("Invalid offset, window=%s", window);
    return false;
  }

  bool replayConf = false;

  uint32_t seqno = rlsbf2(&d[mac_payload::offsets::fcnt]);
  // reconstruct 32 bit value.
  seqno = seqnoDn + (uint16_t)(seqno - seqnoDn);

  if (!aes.verifyMic(devaddr, seqno, PktDir::DOWN, d, dlen)) {
    PRINT_DEBUG_1("Fail to verify aes mic, window=%s", window);
    return false;
  }
  if (seqno < seqnoDn) {
    if ((int32_t)seqno > (int32_t)seqnoDn) {
      return false;
    }
    if (seqno != seqnoDn - 1 || !dnConf ||
        ftype != mhdr::ftype_data_conf_down) {
      return false;
    }
    // Replay of previous sequence number allowed only if
    // previous frame and repeated both requested confirmation
    replayConf = true;
  } else {
    if (seqno > seqnoDn) {
      // skip in sequence number
      PRINT_DEBUG_1("Current packet receive %lu expected %ld", seqno, seqnoDn);
    }
    seqnoDn = seqno + 1; // next number to be expected
    // DN frame requested confirmation - provide ACK once with next UP frame
    dnConf = (ftype == mhdr::ftype_data_conf_down ? FCT_ACK : 0);
  }

  if (dnConf || (fct & FCT_MORE))
    opmode.set(OpState::POLL);

  // We heard from network
  rejoinCnt = 0;

  parseMacCommands(d + mac_payload::offsets::fopts, olen);

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
      txrxFlags.set(TxRxStatus::PORT);

      if (port == 0) {
        parseMacCommands(d + dataBeg, dataLen);
      }
    } else {
      txrxFlags.set(TxRxStatus::NOPORT);
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
    txrxFlags.set(ackup ? TxRxStatus::ACK : TxRxStatus::NACK);

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
  txrxFlags.reset().set(TxRxStatus::DNW2);
  dataLen = 0;
  rps_t rps = dndr2rps(dn2Dr);
  radio.rx(dn2Freq, rps, rxsyms, rxtime);
}

void Lmic::schedRx12(OsDeltaTime delay, uint8_t dr) {
  PRINT_DEBUG_2("SchedRx RX1/2");

  // Half symbol time for the data rate.
  const OsDeltaTime hsym = dr2hsym(dr);

  // If a clock error is specified, compensate for it by extending the
  // receive window
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
  if ((255 - MINRX_SYMS) * hsym < drift)
    rxsyms = 255;
  else
    rxsyms = MINRX_SYMS + (drift / hsym);

  // Center the receive window on the center of the expected preamble
  // (again note that hsym is half a sumbol time, so no /2 needed)
  rxtime = txend + (delay + (PAMBL_SYMS - rxsyms) * hsym);
  PRINT_DEBUG_1("Rx delay : %i ms", (rxtime - txend).to_ms());

  osjob.setTimed(rxtime - RX_RAMPUP);
}

void Lmic::setupRx1() {
  txrxFlags.reset().set(TxRxStatus::DNW1);
  dataLen = 0;
  rps_t rps = dndr2rps(dndr);
  radio.rx(freq, rps, rxsyms, rxtime);
}

// Called by HAL once TX complete and delivers exact end of TX time stamp in
// rxtime
void Lmic::txDone(OsDeltaTime delay) {
  // Change RX frequency / rps (US only) before we increment txChnl
  setRx1Params();
  schedRx12(delay, dndr);
}

// ======================================== Join frames

#if !defined(DISABLE_JOIN)
void Lmic::onJoinFailed() {
  // Notify app - must call reset() to stop joining
  // otherwise join procedure continues.
  reportEvent(EventType::JOIN_FAILED);
}

void Lmic::processJoinAcceptNoJoinFrame() {
  if (opmode.test(OpState::REJOIN)) {
    // REJOIN attempt for roaming
    // rejoin fail, continue normal operation
    opmode.reset(OpState::REJOIN);
    opmode.reset(OpState::TXRXPEND);
    if (rejoinCnt < 10)
      rejoinCnt++;
    reportEvent(EventType::REJOIN_FAILED);
  } else {
    opmode.reset(OpState::TXRXPEND);
    // Clear NEXTCHNL because join state engine controls channel hopping
    opmode.reset(OpState::NEXTCHNL);
    const bool succes = nextJoinState();
    // Build next JOIN REQUEST with next engineUpdate call
    // Optionally, report join failed.
    // Both after a random/chosen amount of ticks.

    osjob.setCallbackRunnable(
        succes ? &Lmic::runEngineUpdate // next step to be delayed
               : &Lmic::onJoinFailed);  // one JOIN iteration done and failed
  }
}

bool Lmic::processJoinAccept() {
  PRINT_DEBUG_2("Process join accept.");
  ASSERT(opmode.test(OpState::TXRXPEND));

  const uint8_t hdr = frame[0];
  const uint8_t dlen = dataLen;

  if (dataLen == 0) {
    return false;
  }

  if ((dlen != join_accept::lengths::total &&
       dlen != join_accept::lengths::totalWithOptional) ||
      (hdr & (mhdr::ftype_mask | mhdr::major_mask)) !=
          (mhdr::ftype_join_acc | mhdr::major_v1)) {
    PRINT_DEBUG_1("Join Accept BAD Length %i or bad header %i ", dlen, hdr);

    // unexpected frame
    return false;
  }
  aes.encrypt(frame + 1, dlen - 1);
  if (!aes.verifyMic0(frame, dlen)) {
    PRINT_DEBUG_1("Join Accept BAD MIC");

    // bad mic
    return false;
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

  ASSERT(opmode.test(OpState::JOINING) || opmode.test(OpState::REJOIN));
  if (opmode.test(OpState::REJOIN)) {
    // Lower DR every try below current UP DR
    // so adjust the current datarate to success join
    datarate = lowerDR(datarate, rejoinCnt);
  }
  opmode.reset(OpState::JOINING)
      .reset(OpState::REJOIN)
      .reset(OpState::TXRXPEND);
  opmode.set(OpState::NEXTCHNL);

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
    txrxFlags.reset(); // nothing in 1st/2nd DN slot
  if (!processJoinAccept()) {
    processJoinAcceptNoJoinFrame();
  };
}

void Lmic::setupRx2Jacc() {
  PRINT_DEBUG_2("Setup RX2 join accept.");
  this->osjob.setCallbackFuture(&Lmic::processRx2Jacc);
  setupRx2();
}

void Lmic::processRx1Jacc() {
  PRINT_DEBUG_2("Result RX1 join accept datalen=%i.", dataLen);
  if (!processJoinAccept()) {
    osjob.setCallbackFuture(&Lmic::setupRx2Jacc);
    schedRx12(OsDeltaTime::from_sec(DELAY_JACC2), dn2Dr);
  }
}

void Lmic::setupRx1Jacc() {
  PRINT_DEBUG_2("Setup RX1 join accept.");
  this->osjob.setCallbackFuture(&Lmic::processRx1Jacc);
  setupRx1();
}

void Lmic::jreqDone() {
  osjob.setCallbackFuture(&Lmic::setupRx1Jacc);
  txDone(OsDeltaTime::from_sec(DELAY_JACC1));
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
  if (!decodeFrame()) {
    dataBeg = 0;
    dataLen = 0;

    // retry send if need
    if (txCnt != 0) {
      if (txCnt < TXCONF_ATTEMPTS) {
        txCnt++;
        setDrTxpow(lowerDR(datarate, TABLE_GET_U1(DRADJUST, txCnt)));
        // Schedule another retransmission
        txDelay(rxtime, RETRY_PERIOD_secs);
        opmode.reset(OpState::TXRXPEND);
        engineUpdate();
        return;
      }
      txrxFlags.reset().set(TxRxStatus::NACK).set(TxRxStatus::NOPORT);
    } else {
      // Nothing received - implies no port
      txrxFlags.reset().set(TxRxStatus::NOPORT);
    }
    incrementAdrCount();

  } else {
    resetAdrCount();
  }
  processDnData();
}

void Lmic::setupRx2DnData() {
  osjob.setCallbackFuture(&Lmic::processRx2DnData);
  setupRx2();
}

void Lmic::processRx1DnData() {
  if (!decodeFrame()) {
    dataBeg = 0;
    dataLen = 0;
    // if nothing receive, wait for RX2 before take actions
    osjob.setCallbackFuture(&Lmic::setupRx2DnData);
    schedRx12(rxDelay + OsDeltaTime::from_sec(DELAY_EXTDNW2), dn2Dr);
  } else {
    resetAdrCount();
    processDnData();
  }
}

void Lmic::setupRx1DnData() {
  osjob.setCallbackFuture(&Lmic::processRx1DnData);
  setupRx1();
}

void Lmic::incrementAdrCount() {
  if (adrAckReq != LINK_CHECK_OFF)
    adrAckReq += 1;
  // If we haven't heard from NWK in a while although we asked for a sign
  // assume link is dead - notify application and keep going
  if (adrAckReq > LINK_CHECK_DEAD) {
    // We haven't heard from NWK for some time although we
    // asked for a response for some time - assume we're disconnected.
    // Restore max power if it not the case
    if (adrTxPow != pow2dBm(0)) {
      adrTxPow = pow2dBm(0);
      opmode.set(OpState::LINKDEAD);
    } else if (decDR(datarate) != datarate) {
      // Lower DR one notch.
      setDrTxpow(decDR(datarate));
      opmode.set(OpState::LINKDEAD);
    } else {
      // we are at max pow and max DR
      opmode.set(OpState::LINKDEAD).set(OpState::REJOIN);
    }
    adrAckReq = LINK_CHECK_CONT;
    reportEvent(EventType::LINK_DEAD);
  }
}

void Lmic::resetAdrCount() {
  if (adrAckReq != LINK_CHECK_OFF)
    adrAckReq = LINK_CHECK_INIT;
  if (opmode.test(OpState::LINKDEAD)) {
    opmode.reset(OpState::LINKDEAD);
    reportEvent(EventType::LINK_ALIVE);
  }
}

void Lmic::updataDone() {
  osjob.setCallbackFuture(&Lmic::setupRx1DnData);
  txDone(rxDelay);
}

// ========================================

void Lmic::buildDataFrame() {
  bool txdata = opmode.test(OpState::TXDATA);

  // Piggyback MAC options
  // Prioritize by importance
  uint8_t end = mac_payload::offsets::fopts;
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
    // lorawan 1.0.2 §5.5. the margin is the SNR.
    // Convert to real SNR; rounding towards zero.
    const int8_t snr = (radio.get_last_packet_snr_x4() + 2) / 4;
    frame[end + 2] = static_cast<uint8_t>(
        (0x3F & (snr <= -32 ? -32 : snr >= 31 ? 31 : snr)));

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
  ASSERT(end <= mac_payload::offsets::fopts + 16);

  uint8_t flen = end + (txdata ? 5 + pendTxLen : 4);
  if (flen > MAX_LEN_FRAME) {
    // Options and payload too big - delay payload
    txdata = 0;
    flen = end + 4;
  }
  frame[offsets::MHDR] = mhdr::ftype_data_up | mhdr::major_v1;
  frame[mac_payload::offsets::fctrl] =
      (dnConf | (adrAckReq != LINK_CHECK_OFF ? FCT_ADREN : 0) |
       (adrAckReq >= 0 ? FCT_ADRARQ : 0) | (end - mac_payload::offsets::fopts));
  wlsbf4(frame + mac_payload::offsets::devAddr, devaddr);

  if (txCnt == 0) {
    seqnoUp++;
  }
  const uint32_t current_seq_no = seqnoUp - 1;
  wlsbf2(frame + mac_payload::offsets::fcnt, current_seq_no);

  // Clear pending DN confirmation
  dnConf = 0;

  if (txdata) {
    if (pendTxConf) {
      // Confirmed only makes sense if we have a payload (or at least a port)
      frame[offsets::MHDR] = mhdr::ftype_data_conf_up | mhdr::major_v1;
      if (txCnt == 0)
        txCnt = 1;
    }
    frame[end] = pendTxPort;
    std::copy(pendTxData, pendTxData + pendTxLen, frame + end + 1);
    aes.framePayloadEncryption(pendTxPort, devaddr, current_seq_no, PktDir::UP,
                               frame + end + 1, pendTxLen);
  }
  aes.appendMic(devaddr, current_seq_no, PktDir::UP, frame, flen);

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
  frame[join_request::offset::MHDR] = mhdr::ftype_join_req | mhdr::major_v1;
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
    ASSERT(opmode.test(OpState::POLL) || opmode.test(OpState::TXRXPEND));
    // Lift any previous duty limitation
    globalDutyRate = 0;
    // Cancel scanning
    opmode.reset(OpState::REJOIN)
        .reset(OpState::LINKDEAD)
        .reset(OpState::LINKDEAD)
        .set(OpState::JOINING);
    // Setup state
    rejoinCnt = txCnt = 0;
    // remove rx 1 offset
    rx1DrOffset = 0;

    initJoinLoop();

    // reportEvent will call engineUpdate which then starts sending JOIN
    // REQUESTS
    osjob.setCallbackRunnable(&Lmic::startJoiningCallBack);
    return true;
  }
  return false; // already joined
}
#endif // !DISABLE_JOIN

void Lmic::processDnData() {
  opmode.reset(OpState::TXDATA).reset(OpState::TXRXPEND);

  if ((txrxFlags.test(TxRxStatus::DNW1) || txrxFlags.test(TxRxStatus::DNW2)) &&
      opmode.test(OpState::LINKDEAD)) {
    opmode.reset(OpState::LINKDEAD);
    reportEvent(EventType::LINK_ALIVE);
  }

  reportEvent(EventType::TXCOMPLETE);
}

// Decide what to do next for the MAC layer of a device
void Lmic::engineUpdate() {
  // PRINT_DEBUG_1("engineUpdate, opmode=0x%hhx.", static_cast
  // <uint8_t>(opmode)); Check for ongoing state: scan or TX/RX transaction
  if (opmode.test(OpState::TXRXPEND) || opmode.test(OpState::SHUTDOWN))
    return;

#if !defined(DISABLE_JOIN)
  if (devaddr == 0 && !opmode.test(OpState::JOINING)) {
    startJoining();
    return;
  }
#endif // !DISABLE_JOIN

  const OsTime now = os_getTime();
  OsTime txbeg = now;

  if (opmode.test(OpState::JOINING) || opmode.test(OpState::REJOIN) ||
      opmode.test(OpState::TXDATA) || opmode.test(OpState::POLL)) {
    // Need to TX some data...
    // Assuming txChnl points to channel which first becomes available again.
    const bool jacc =
        opmode.test(OpState::JOINING) || opmode.test(OpState::REJOIN);
#if LMIC_DEBUG_LEVEL > 1
    if (jacc)
      lmic_printf("%lu: Uplink join pending\n", os_getTime().tick());
    else
      lmic_printf("%lu: Uplink data pending\n", os_getTime().tick());
#endif
    // Find next suitable channel and return availability time
    if (opmode.test(OpState::NEXTCHNL)) {
      txbeg = txend = nextTx(now);
      opmode.reset(OpState::NEXTCHNL);
      PRINT_DEBUG_2("Airtime available at %lu (channel duty limit)",
                    txbeg.tick());
    } else {
      txbeg = txend;
      PRINT_DEBUG_2("Airtime available at %lu (previously determined)",
                    txbeg.tick());
    }
    // Delayed TX or waiting for duty cycle?
    if (txbeg < globalDutyAvail) {
      txbeg = globalDutyAvail;
      PRINT_DEBUG_2("Airtime available at %lu (global duty limit)",
                    txbeg.tick());
    }
    // Earliest possible time vs overhead to setup radio
    if (txbeg >= (now + TX_RAMPUP)) {
      PRINT_DEBUG_1("Uplink delayed until %lu", txbeg.tick());
      // Cannot yet TX
      //  wait for the time to TX
      osjob.setTimedCallback(txbeg - TX_RAMPUP, &Lmic::runEngineUpdate);
      return;
    }

    PRINT_DEBUG_1("Ready for uplink");
    // We could send right now!
    txbeg = now;
    dr_t txdr = datarate;
#if !defined(DISABLE_JOIN)
    if (jacc) {
      if (opmode.test(OpState::REJOIN)) {
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
    rps_t rps = updr2rps(txdr);
    dndr = txdr; // carry TX datarate (can be != datarate) over to
                 // txDone/setupRx1

    opmode.reset(OpState::POLL);
    opmode.set(OpState::TXRXPEND);
    opmode.set(OpState::NEXTCHNL);
    OsDeltaTime airtime = calcAirTime(rps, dataLen);

    updateTx(txbeg, airtime);
    if (globalDutyRate != 0) {
      globalDutyAvail = txbeg + OsDeltaTime(airtime.tick() << globalDutyRate);
      PRINT_DEBUG_2("Updating global duty avail to %lu", globalDutyAvail);
    }

    radio.tx(freq, rps, txpow + antennaPowerAdjustment, frame, dataLen);
  }
  // No TX pending - no scheduled RX
}

void Lmic::setAntennaPowerAdjustment(int8_t power) {
  antennaPowerAdjustment = power;
}

void Lmic::shutdown() {
  osjob.clearCallback();
  radio.rst();
  opmode.set(OpState::SHUTDOWN);
}

void Lmic::reset() {
  radio.rst();
  osjob.clearCallback();
  devaddr = 0;
  devNonce = rand.uint16();
  opmode.reset();
  // we need this for 2nd DN window of join accept
  dn2Dr = defaultRX2Dr();
  dn2Freq = defaultRX2Freq();
  rxDelay = OsDeltaTime::from_sec(DELAY_DNW1);

  initDefaultChannels(true);
}

void Lmic::init() {
  radio.init();
  rand.init(radio);
  opmode.reset().set(OpState::SHUTDOWN);
}

void Lmic::clrTxData() {
  opmode.reset(OpState::TXDATA).reset(OpState::TXRXPEND).reset(OpState::POLL);
  pendTxLen = 0;
  if (opmode.test(OpState::JOINING)) // do not interfere with JOINING
    return;
  osjob.clearCallback();
  radio.rst();
  engineUpdate();
}

void Lmic::setTxData() {
  opmode.set(OpState::TXDATA);
  if (!opmode.test(OpState::JOINING))
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
  opmode.set(OpState::POLL);
  engineUpdate();
}

// Check if other networks are around.
void Lmic::tryRejoin(void) {
  opmode.set(OpState::REJOIN);
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

  opmode.reset(OpState::JOINING);
  opmode.reset(OpState::REJOIN);
  opmode.reset(OpState::TXRXPEND);
  opmode.set(OpState::NEXTCHNL);
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
  if (radio.io_check(frame, dataLen, txend, rxtime)) {
    // if radio task ended, activate next job.
    osjob.setRunnable();
  }
}

void Lmic::store_trigger() { radio.store_trigger(); }

Lmic::Lmic(lmic_pinmap const &pins) : radio(pins), rand(aes) {}