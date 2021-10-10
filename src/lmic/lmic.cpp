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

//! \file
#include "lmic.h"
#include "../aes/lmic_aes.h"
#include "../hal/print_debug.h"
#include "bufferpack.h"
#include "lmic_table.h"
#include "lorawanpacket.h"
#include "radio.h"
#include <algorithm>

using namespace lorawan;

constexpr uint8_t MINRX_SYMS = 5;
constexpr uint8_t PAMBL_SYMS = 8;

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
  // BW 0,1,2 = 125,250,500kHz
  const uint8_t bw = rps.bwRaw;
  // SF 7..12 = SF7..12
  const uint8_t sf = 7 + rps.sf - SF7;
  const uint8_t sfx = 4 * sf;
  const uint8_t optimiseLowSf = (rps.sf >= SF11 ? 8 : 0);
  const uint8_t q = sfx - optimiseLowSf;

  int16_t tmp = 8 * plen - sfx + 28 + (rps.nocrc ? 0 : 16);
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
  PRINT_DEBUG(1, F("Time on air : %i ms"), val.to_ms());
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
  if (delayRef > globalDutyAvail) {
    globalDutyAvail = delayRef;
  }
}

/**
 * Set the battery level from :
 *  0 : external power source
 *  1-254 : battery level
 *  255 : no info
 * To be call when  txrxFlags have NEED_BATTERY_LEVEL flag
 **/
void Lmic::setBatteryLevel(uint8_t level) { battery_level = level; }

void Lmic::setDrJoin(dr_t dr) { datarate = dr; }

void Lmic::setDrTx(uint8_t dr) {
  if (datarate != dr) {
    datarate = dr;
    opmode.set(OpState::NEXTCHNL);
  }
}

void Lmic::setRx2Parameter(uint32_t rx2frequency, dr_t rx2datarate) {
  rx2Parameter = {rx2frequency, rx2datarate};
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
  startJoining();
  reportEvent(EventType::RESET);
}

void Lmic::stateJustJoined() {
  seqnoDn = 0;
  seqnoUp = 0;
  dnConf = 0;
  ladrAns = 0;
  devsAns = false;
  rxTimingSetupAns = false;
  globalDutyRate = 0;
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
}

void Lmic::parse_ladr(const uint8_t *const opts) {
  // FIXME multiple LinkAdrReq not handled properly in continous block
  // must be handle atomic.
  const uint8_t p1 = opts[1];               // txpow + DR
  const uint16_t chMask = rlsbf2(&opts[2]); // list of enabled channels
  // channel page
  const uint8_t chMaskCntl =
      (opts[4] & MCMD_LADR_CHPAGE_MASK) >> MCMD_LADR_CHPAGE_OFFSET;
  // up repeat count
  const uint8_t nbTrans = opts[4] & MCMD_LADR_REPEAT_MASK;

  // Include an answer into next frame up
  ladrAns =
      0x80 | MCMD_LADR_ANS_POWACK | MCMD_LADR_ANS_CHACK | MCMD_LADR_ANS_DRACK;
  if (!validMapChannels(chMaskCntl, chMask)) {
    PRINT_DEBUG(1, F("ADR REQ Invalid map channel maskCtnl=%i, mask=%i"),
                chMaskCntl, chMask);
    ladrAns &= ~MCMD_LADR_ANS_CHACK;
  }
  const dr_t dr = (dr_t)(p1 >> MCMD_LADR_DR_SHIFT);
  if (!validDR(dr)) {
    PRINT_DEBUG(1, F("ADR REQ Invalid dr %i"), dr);
    ladrAns &= ~MCMD_LADR_ANS_DRACK;
  }

  uint8_t const txPowerIndex = (p1 & MCMD_LADR_POW_MASK) >> MCMD_LADR_POW_SHIFT;
  auto const newPower = pow2dBm(txPowerIndex);
  if (newPower == InvalidPower) {
    PRINT_DEBUG(1, F("ADR REQ Invalid power index %i"), txPowerIndex);
    ladrAns &= ~MCMD_LADR_ANS_POWACK;
  }

  if ((ladrAns & 0x7F) ==
      (MCMD_LADR_ANS_POWACK | MCMD_LADR_ANS_CHACK | MCMD_LADR_ANS_DRACK)) {
    // Nothing went wrong - use settings
    upRepeat = nbTrans;
    mapChannels(chMaskCntl, chMask);
    PRINT_DEBUG(1, F("ADR REQ Change dr to %i, power to %i"), dr, txPowerIndex);
    adrTxPow = newPower;
    setDrTx(dr);
  }
}

void Lmic::parse_dn2p(const uint8_t *const opts) {
#if !defined(DISABLE_MCMD_DN2P_SET)
  const dr_t dr = (dr_t)(opts[1] & 0x0F);
  const uint8_t newRx1DrOffset = ((opts[1] & 0x70) >> 4);
  const uint32_t newfreq = convFreq(&opts[2]);
  dn2Ans = 0x80; // answer pending
  if (validRx1DrOffset(newRx1DrOffset))
    dn2Ans |= MCMD_DN2P_ANS_RX1DrOffsetAck;
  if (validDR(dr))
    dn2Ans |= MCMD_DN2P_ANS_DRACK;
  if (newfreq != 0)
    dn2Ans |= MCMD_DN2P_ANS_CHACK;
  // Only take parameter into account if all parameter are ok.
  if (dn2Ans == (0x80 | MCMD_DN2P_ANS_RX1DrOffsetAck | MCMD_DN2P_ANS_DRACK |
                 MCMD_DN2P_ANS_CHACK)) {
    rx2Parameter.datarate = dr;
    rx2Parameter.frequency = newfreq;
    rx1DrOffset = newRx1DrOffset;
  }
#endif // !DISABLE_MCMD_DN2P_SET
}

void Lmic::setDutyRate(uint8_t rate) {
  globalDutyRate = rate;
  globalDutyAvail = os_getTime();
}

void Lmic::parse_dcap(const uint8_t *const opts) {
#if !defined(DISABLE_MCMD_DCAP_REQ)
  const uint8_t cap = opts[1];
  globalDutyRate = cap & 0xF;
  globalDutyAvail = os_getTime();
  dutyCapAns = true;
#endif // !DISABLE_MCMD_DCAP_REQ
}

void Lmic::parse_snch(const uint8_t *const opts) {
#if !defined(DISABLE_MCMD_SNCH_REQ)
  const uint8_t chidx = opts[1];               // channel
  const uint32_t newfreq = convFreq(&opts[2]); // freq
  const uint8_t drs = opts[5];                 // datarate span
  snchAns = 0x80;
  if (newfreq != 0 &&
      setupChannel(chidx, newfreq, dr_range_map(drs & 0xF, drs >> 4)))
    snchAns |= MCMD_SNCH_ANS_DRACK | MCMD_SNCH_ANS_FQACK;
#endif // !DISABLE_MCMD_SNCH_REQ
}

void Lmic::parse_rx_timing_setup(const uint8_t *const opts) {
  uint8_t newDelay = opts[1] & 0x0F;
  if (newDelay == 0)
    newDelay = 1;
  rxDelay = OsDeltaTime::from_sec(newDelay);
  rxTimingSetupAns = true;
}

void Lmic::parseMacCommands(const uint8_t *const opts, uint8_t const olen) {
  uint8_t oidx = 0;
  while (oidx < olen) {
    PRINT_DEBUG(1, F("Parse Mac command %d"), opts[oidx]);
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
      parse_ladr(opts + oidx);
      oidx += 5;
      continue;
    }
    // DevStatusReq LoRaWAN™ Specification §5.5
    case MCMD_DEVS_REQ: {
      txrxFlags.set(TxRxStatus::NEED_BATTERY_LEVEL);
      devsAns = true;
      oidx += 1;
      continue;
    }
    // RXParamSetupReq LoRaWAN™ Specification §5.4
    case MCMD_DN2P_SET: {
      parse_dn2p(opts + oidx);
      oidx += 5;
      continue;
    }
    // DutyCycleReq LoRaWAN™ Specification §5.3
    case MCMD_DCAP_REQ: {
      parse_dcap(opts + oidx);
      oidx += 2;
      continue;
    }
    // NewChannelReq LoRaWAN™ Specification §5.6
    case MCMD_SNCH_REQ: {
      parse_snch(opts + oidx);
      oidx += 6;
      continue;
    }
    // RXTimingSetupReq LoRaWAN™ Specification §5.7
    case MCMD_RXTimingSetup_REQ: {
      parse_rx_timing_setup(opts + oidx);
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
    PRINT_DEBUG(1, F("Parse of MAC command incompleted."));
  }
}

uint32_t Lmic::read_seqno(uint8_t const *const buffer) const {
  const uint32_t seqno = rlsbf2(buffer);
  // reconstruct 32 bit value.
  return seqnoDn + (uint16_t)(seqno - seqnoDn);
}

Lmic::SeqNoValidity Lmic::check_seq_no(const uint32_t seqno,
                                       const uint8_t ftype) const {
  const auto diff = static_cast<int32_t>(seqno - seqnoDn);

  if (diff == 0)
    return SeqNoValidity::ok;

  if (diff > 0) {
    // skip in sequence number, missed packet
    PRINT_DEBUG(1, F("Current packet receive %" PRIu32 " expected %" PRIu32),
                seqno, seqnoDn);
    return SeqNoValidity::ok;
  }

  // Replay of previous sequence number allowed only if
  // previous frame and repeated both requested confirmation
  // TODO validity of this test ? check dnConf may be reset to zero by the
  // sending of the ack
  if (diff == -1 && dnConf && ftype == mhdr::ftype_data_conf_down) {
    return SeqNoValidity::previous;
  }

  return SeqNoValidity::invalid;
}

// ================================================================================
// Decoding frames
bool Lmic::decodeFrame() {

  if (txrxFlags.test(TxRxStatus::DNW1)) {
    PRINT_DEBUG(1, F("Decode Frame RX1"));
  } else if (txrxFlags.test(TxRxStatus::DNW2)) {
    PRINT_DEBUG(1, F("Decode Frame RX2"));
  } else {
    PRINT_DEBUG(1, F("Decode Frame unk"));
  }

  if (dataLen == 0) {
    PRINT_DEBUG(1, F("No downlink data"));
    return false;
  }

  const uint8_t hdr = frame[0];
  const uint8_t ftype = hdr & mhdr::ftype_mask;
  const uint8_t dlen = dataLen;

  if (dlen < mac_payload::offsets::fopts + lengths::MIC ||
      (hdr & mhdr::major_mask) != mhdr::major_v1 ||
      (ftype != mhdr::ftype_data_down && ftype != mhdr::ftype_data_conf_down)) {
    // Basic sanity checks failed
    PRINT_DEBUG(1, F("Invalid downlink"));
    return false;
  }

  const uint32_t addr = rlsbf4(frame.cbegin() + mac_payload::offsets::devAddr);
  if (addr != devaddr) {
    PRINT_DEBUG(1, F("Invalid address"));
    return false;
  }

  const uint8_t fct = frame[mac_payload::offsets::fctrl];
  const uint8_t olen = fct & FCT_OPTLEN;
  const bool ackup = (fct & FCT_ACK) != 0 ? true : false; // ACK last up frame
  const uint8_t poff = mac_payload::offsets::fopts + olen;
  const uint8_t pend = dlen - lengths::MIC; // MIC

  if (poff > pend) {
    PRINT_DEBUG(1, F("Invalid data offset"));
    return false;
  }

  const uint32_t seqno =
      read_seqno(frame.cbegin() + mac_payload::offsets::fcnt);

  if (!aes.verifyMic(devaddr, seqno, PktDir::DOWN, frame.cbegin(), dlen)) {
    PRINT_DEBUG(1, F("Fail to verify aes mic"));
    return false;
  }

  const auto checkseqnoresult = check_seq_no(seqno, ftype);
  if (checkseqnoresult == SeqNoValidity::invalid)
    return false;
  const bool replayConf = checkseqnoresult == SeqNoValidity::previous;

  // next number to be expected
  seqnoDn = seqno + 1;

  // DN frame requested confirmation - provide ACK once with next UP frame
  dnConf = ftype == mhdr::ftype_data_conf_down ? FCT_ACK : 0;
  if (dnConf || (fct & FCT_MORE))
    opmode.set(OpState::POLL);

  parseMacCommands(frame.cbegin() + mac_payload::offsets::fopts, olen);

  if (!replayConf) {
    // Handle payload only if not a replay
    if (pend > poff) {
      const auto port = frame[poff];
      dataBeg = poff + 1;
      dataLen = pend - dataBeg;
      // Decrypt payload - if any
      aes.framePayloadEncryption(port, devaddr, seqno, PktDir::DOWN,
                                 frame.begin() + dataBeg, dataLen);
      txrxFlags.set(TxRxStatus::PORT);

      if (port == 0) {
        parseMacCommands(frame.cbegin() + dataBeg, dataLen);
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

  PRINT_DEBUG(1, F("Received downlink, port=%d, ack=%d"), getPort(), ackup);
  return true;
}

// ================================================================================
// TX/RX transaction support

void Lmic::setupRx1() {
  txrxFlags.reset().set(TxRxStatus::DNW1);
  dataLen = 0;
  auto parameters = getRx1Parameter();
  rps_t rps = dndr2rps(parameters.datarate);
  radio.rx(parameters.frequency, rps, rxsyms, rxtime);
  wait_end_rx();
}

void Lmic::setupRx2() {
  txrxFlags.reset().set(TxRxStatus::DNW2);
  dataLen = 0;
  rps_t const rps = dndr2rps(rx2Parameter.datarate);
  radio.rx(rx2Parameter.frequency, rps, rxsyms, rxtime);
  wait_end_rx();
}

OsTime Lmic::schedRx12(OsDeltaTime delay, dr_t dr) {
  PRINT_DEBUG(2, F("SchedRx RX1/2"));

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
  PRINT_DEBUG(1, F("Rx delay : %i ms"), (rxtime - txend).to_ms());

  return (rxtime - RX_RAMPUP);
}

// Called by HAL once TX complete and delivers exact end of TX time stamp in
// rxtime. Schedule first receive.
void Lmic::txDone() {
  auto waitime = schedRx12(rxDelay, getRx1Parameter().datarate);
  next_job = Job(&Lmic::setupRx1, waitime);
}

// ======================================== Join frames

void Lmic::onJoinFailed() {
  // Notify app - must call reset() to stop joining
  // otherwise join procedure continues.
  reportEvent(EventType::JOIN_FAILED);
}

void Lmic::processJoinAcceptNoJoinFrame() {
  opmode.reset(OpState::TXRXPEND);
  // Clear NEXTCHNL because join state engine controls channel hopping
  opmode.reset(OpState::NEXTCHNL);
  const bool succes = nextJoinState();

  // in §7 of lorawan 1.0.3
  // the backoff for join is describe
  // Duty rate of 2^14 is enought to respect 8.7s by 24h
  // and divide by every 4000s, from 2^7 to 2^14, is enougth to
  // respect 36s during first hour and
  // 36s during next 10h

  if (globalDutyRate < 14 &&
      os_getTime() - lastDutyRateBackOff > OsDeltaTime::from_sec(4000)) {
    PRINT_DEBUG(1, F("Reduce join DutyRate: %i"), globalDutyRate);
    globalDutyRate++;
    lastDutyRateBackOff = os_getTime();
  }

  // Build next JOIN REQUEST with next engineUpdate call
  // Optionally, report join failed.
  // Both after a random/chosen amount of ticks.
  if (txend < globalDutyAvail) {
    txend = globalDutyAvail;
  }
  txend += getDwn2SafetyZone();
  txend += OsDeltaTime::rnd_delay(rand, 255 >> datarate);

  PRINT_DEBUG(1, F("Next Join delay : %i s"), (txend - os_getTime()).to_s());

  if (succes) {
    // next step to be delayed
    next_job = Job(&Lmic::runEngineUpdate);
  } else {
    // one JOIN iteration done and failed
    next_job = Job(&Lmic::onJoinFailed);
  }
}

bool Lmic::processJoinAccept() {
  PRINT_DEBUG(2, F("Process join accept."));
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
    PRINT_DEBUG(1, F("Join Accept BAD Length %i or bad header %i "), dlen, hdr);

    // unexpected frame
    return false;
  }

  aes.encrypt(frame.begin() + 1, dlen - 1);
  if (!aes.verifyMic0(frame.cbegin(), dlen)) {
    PRINT_DEBUG(1, F("Join Accept BAD MIC"));

    // bad mic
    return false;
  }

  devaddr = rlsbf4(frame.cbegin() + join_accept::offset::devAddr);
  netid = rlsbf4(frame.cbegin() + join_accept::offset::netId) & 0xFFFFFF;

  if (dlen > join_accept::lengths::total) {
    // some region just ignore cflist.
    handleCFList(frame.cbegin() + join_accept::offset::cfList);
  }

  // already incremented when JOIN REQ got sent off
  aes.sessKeys(devNonce - 1, frame.cbegin() + join_accept::offset::appNonce);

  ASSERT(opmode.test(OpState::JOINING));

  opmode.reset(OpState::JOINING).reset(OpState::TXRXPEND);
  opmode.set(OpState::NEXTCHNL);

  txCnt = 0;
  stateJustJoined();

  const uint8_t dlSettings = frame[join_accept::offset::dlSettings];
  rx2Parameter.datarate = dlSettings & 0x0F;
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

void Lmic::processRxJacc() {
  PRINT_DEBUG(2, F("Result RX join accept datalen=%i."), dataLen);
  if (!processJoinAccept()) {
    if (txrxFlags.test(TxRxStatus::DNW1)) {
      // wait for RX2
      auto waitime =
          schedRx12(OsDeltaTime::from_sec(DELAY_JACC2), rx2Parameter.datarate);
      next_job = Job(&Lmic::setupRx2, waitime);
    } else {
      // nothing in 1st/2nd DN slot
      txrxFlags.reset();
      processJoinAcceptNoJoinFrame();
    }
  }
}

// ======================================== Data frames

void Lmic::processRxDnData() {
  if (txrxFlags.test(TxRxStatus::DNW1)) {
    processRx1DnData();
  } else {
    processRx2DnData();
  }
}

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
        setDrTx(lowerDR(datarate, TABLE_GET_U1(DRADJUST, txCnt)));
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

void Lmic::processRx1DnData() {
  if (!decodeFrame()) {
    dataBeg = 0;
    dataLen = 0;
    // if nothing receive, wait for RX2 before take actions
    auto waitime = schedRx12(rxDelay + OsDeltaTime::from_sec(DELAY_EXTDNW2),
                             rx2Parameter.datarate);
    next_job = Job(&Lmic::setupRx2, waitime);

  } else {
    resetAdrCount();
    processDnData();
  }
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
    } else {
      // Lower DR one notch.
      setDrTx(decDR(datarate));
    }

    opmode.set(OpState::LINKDEAD);
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


uint8_t *Lmic::add_opt_dcap(uint8_t *pos) {
#if !defined(DISABLE_MCMD_DCAP_REQ)
  if (dutyCapAns) {
    *(pos++) = MCMD_DCAP_ANS;
    dutyCapAns = false;
  }
#endif // !DISABLE_MCMD_DCAP_REQ
  return pos;
}

uint8_t *Lmic::add_opt_dn2p(uint8_t *pos) {
#if !defined(DISABLE_MCMD_DN2P_SET)
  // RXParamSetupAns LoRaWAN™ Specification §5.4
  if (dn2Ans) {
    *(pos++) = MCMD_DN2P_ANS;
    *(pos++) = dn2Ans & ~MCMD_DN2P_ANS_RFU;
    // dn2Ans reset when downlink packet receive
  }
#endif // !DISABLE_MCMD_DN2P_SET
  return pos;
}

uint8_t *Lmic::add_opt_devs(uint8_t *pos) {
  // DevStatusAns LoRaWAN™ Specification §5.5
  if (devsAns) { // answer to device status
    *(pos++) = MCMD_DEVS_ANS;
    *(pos++) = battery_level;
    // lorawan 1.0.2 §5.5. the margin is the SNR.
    // Convert to real SNR; rounding towards zero.
    const int8_t snr = (radio.get_last_packet_snr_x4() + 2) / 4;
    *(pos++) = static_cast<uint8_t>((0x3F & (snr <= -32  ? -32
                                             : snr >= 31 ? 31
                                                         : snr)));
    devsAns = false;
  }
  return pos;
}

uint8_t *Lmic::add_opt_adr(uint8_t *pos) {
  if (ladrAns) { // answer to ADR change
    *(pos++) = MCMD_LADR_ANS;
    *(pos++) = ladrAns & ~MCMD_LADR_ANS_RFU;
    ladrAns = 0;
  }
  return pos;
}

uint8_t *Lmic::add_opt_rxtiming(uint8_t *pos) {
  if (rxTimingSetupAns) {
    *(pos++) = MCMD_RXTimingSetup_ANS;
    // rxTimingSetupAns reset when downlink packet receive
  }
  return pos;
}

uint8_t *Lmic::add_opt_snch(uint8_t *pos) {
#if !defined(DISABLE_MCMD_SNCH_REQ)
  if (snchAns) {
    *(pos++) = MCMD_SNCH_ANS;
    *(pos++) = snchAns & ~MCMD_SNCH_ANS_RFU;
    snchAns = 0;
  }
#endif // !DISABLE_MCMD_SNCH_REQ
  return pos;
}

void Lmic::buildDataFrame() {

  // Piggyback MAC options
  // Prioritize by importance
  uint8_t *pos = frame.begin() + mac_payload::offsets::fopts;
  pos = add_opt_dcap(pos);
  pos = add_opt_dn2p(pos);
  pos = add_opt_devs(pos);
  pos = add_opt_adr(pos);
  pos = add_opt_rxtiming(pos);
  pos = add_opt_snch(pos);

  const uint8_t end = pos - frame.cbegin();

  ASSERT(end <= mac_payload::offsets::fopts + 16);

  bool txdata = opmode.test(OpState::TXDATA);
  uint8_t flen = end + (txdata ? 1 + lengths::MIC + pendTxLen : lengths::MIC);
  if (flen > frame.max_size()) {
    // Options and payload too big - delay payload
    txdata = false;
    flen = end + lengths::MIC;
  }

  frame[offsets::MHDR] = mhdr::ftype_data_up | mhdr::major_v1;
  frame[mac_payload::offsets::fctrl] =
      (dnConf | (adrAckReq != LINK_CHECK_OFF ? FCT_ADREN : 0) |
       (adrAckReq >= 0 ? FCT_ADRARQ : 0) | (end - mac_payload::offsets::fopts));
  wlsbf4(frame.begin() + mac_payload::offsets::devAddr, devaddr);

  // if not a resend
  if (txCnt == 0) {
    seqnoUp++;
  }
  const uint32_t current_seq_no = seqnoUp - 1;
  wlsbf2(frame.begin() + mac_payload::offsets::fcnt, current_seq_no);

  // Clear pending DN confirmation
  dnConf = 0;

  if (txdata) {
    if (pendTxConf) {
      // Confirmed only makes sense if we have a payload (or at least a port)
      frame[offsets::MHDR] = mhdr::ftype_data_conf_up | mhdr::major_v1;
      if (txCnt == 0)
        txCnt = 1;
    }
    uint8_t *buffer_pos = frame.begin() + end;

    *(buffer_pos++) = pendTxPort;
    std::copy(begin(pendTxData), begin(pendTxData) + pendTxLen, buffer_pos);
    aes.framePayloadEncryption(pendTxPort, devaddr, current_seq_no, PktDir::UP,
                               buffer_pos, pendTxLen);
  }
  aes.appendMic(devaddr, current_seq_no, PktDir::UP, frame.begin(), flen);

  dataLen = flen;

  PRINT_DEBUG(1, F("Build pkt # %" PRIu32), current_seq_no);
}

// ================================================================================
//
// Join stuff
//
// ================================================================================

void Lmic::buildJoinRequest() {
  // Do not use pendTxData since we might have a pending
  // user level frame in there. Use RX holding area instead.
  frame[join_request::offset::MHDR] = mhdr::ftype_join_req | mhdr::major_v1;
  artEuiCallBack(frame.begin() + join_request::offset::appEUI);
  devEuiCallBack(frame.begin() + join_request::offset::devEUI);
  wlsbf2(frame.begin() + join_request::offset::devNonce, devNonce);
  aes.appendMic0(frame.begin(), join_request::lengths::totalWithMic);

  dataLen = join_request::lengths::totalWithMic;
  devNonce++;
}

void Lmic::startJoiningCallBack() { reportEvent(EventType::JOINING); }

// Start join procedure if not already joined.
bool Lmic::startJoining() {
  if (devaddr == 0) {
    // There should be no TX/RX going on
    ASSERT(opmode.test(OpState::POLL) || opmode.test(OpState::TXRXPEND));
    // Reset Duty rate limitation to respect retransmission backoff
    // (max 36 during first hour)
    globalDutyRate = 7;
    lastDutyRateBackOff = os_getTime();

    // Cancel scanning
    opmode.reset(OpState::LINKDEAD)
        .reset(OpState::NEXTCHNL)
        .set(OpState::JOINING);
    // Setup state
    txCnt = 0;
    rxDelay = OsDeltaTime::from_sec(DELAY_JACC1);

    initJoinLoop();

    // reportEvent will call engineUpdate which then starts sending JOIN
    // REQUESTS
    next_job = Job(&Lmic::startJoiningCallBack);
    return true;
  }
  return false; // already joined
}

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
  // PRINT_DEBUG(1,F("engineUpdate, opmode=0x%hhx."), static_cast
  // <uint8_t>(opmode)); Check for ongoing state: scan or TX/RX transaction
  if (opmode.test(OpState::TXRXPEND) || opmode.test(OpState::SHUTDOWN))
    return;

  if (devaddr == 0 && !opmode.test(OpState::JOINING)) {
    startJoining();
    return;
  }

  if (!opmode.test(OpState::JOINING) && !opmode.test(OpState::TXDATA) &&
      !opmode.test(OpState::POLL)) {
    // No TX pending - no scheduled RX
    return;
  }

  // Need to TX some data...
  // Assuming txChnl points to channel which first becomes available again.
  const bool jacc = opmode.test(OpState::JOINING);
  if (jacc) {
    PRINT_DEBUG(2, F("Uplink join pending"));
  } else {
    PRINT_DEBUG(2, F("Uplink data pending"));
  }

  const OsTime now = os_getTime();
  OsTime txbeg;
  // Find next suitable channel and return availability time
  if (opmode.test(OpState::NEXTCHNL)) {
    txbeg = nextTx(now);
    opmode.reset(OpState::NEXTCHNL);
    PRINT_DEBUG(2, F("Airtime available at %" PRIu32 " (channel duty limit)"),
                txbeg.tick());
  } else {
    txbeg = txend;
    PRINT_DEBUG(2,
                F("Airtime available at %" PRIu32 " (previously determined)"),
                txbeg.tick());
  }
  // Delayed TX or waiting for duty cycle?
  if (txbeg < globalDutyAvail) {
    txbeg = globalDutyAvail;
    PRINT_DEBUG(2, F("Airtime available at %" PRIu32 " (global duty limit)"),
                txbeg.tick());
  }

  // Earliest possible time vs overhead to setup radio
  if (txbeg >= (now + TX_RAMPUP)) {
    PRINT_DEBUG(1, F("Uplink delayed until %" PRIu32), txbeg.tick());
    // Cannot yet TX
    //  wait for the time to TX
    next_job = Job(&Lmic::runEngineUpdate, txbeg - TX_RAMPUP);
    txend = txbeg;
    return;
  }

  PRINT_DEBUG(1, F("Ready for uplink"));
  // We could send right now!
  if (jacc) {
    buildJoinRequest();
  } else {
    if (seqnoDn >= 0xFFFFFF80) {
      // Imminent roll over - proactively reset MAC
      // Device has to react! NWK will not roll over and just stop sending.
      // Thus, we have N frames to detect a possible lock up.
      next_job = Job(&Lmic::runReset);
      return;
    }
    if ((txCnt == 0 && seqnoUp == 0xFFFFFFFF)) {
      // Roll over of up seq counter
      // Do not run RESET event callback from here!
      // App code might do some stuff after send unaware of RESET.
      next_job = Job(&Lmic::runReset);
      return;
    }
    buildDataFrame();
  }

  opmode.reset(OpState::POLL);
  opmode.set(OpState::TXRXPEND);
  opmode.set(OpState::NEXTCHNL);

  rps_t rps = updr2rps(datarate);
  OsDeltaTime airtime = calcAirTime(rps, dataLen);
  updateTxTimes(airtime);

  // if globalDutyRate==0 send available just after transmit.
  globalDutyAvail = os_getTime() + (airtime << globalDutyRate);
  PRINT_DEBUG(2, F("Updating global duty avail to %" PRIu32 ""),
              globalDutyAvail.tick());

  radio.tx(getTxFrequency(), rps, getTxPower() + antennaPowerAdjustment,
           frame.cbegin(), dataLen);
  wait_end_tx();
}

void Lmic::setAntennaPowerAdjustment(int8_t power) {
  antennaPowerAdjustment = power;
}

void Lmic::shutdown() {
  next_job = {};
  radio.rst();
  opmode.set(OpState::SHUTDOWN);
}

void Lmic::reset() {
  radio.rst();
  next_job = {};
  devaddr = 0;
  devNonce = rand.uint16();
  opmode.reset();
  rx1DrOffset = 0;
  // we need this for 2nd DN window of join accept
  rx2Parameter = defaultRX2Parameter();
  rxDelay = OsDeltaTime::from_sec(DELAY_DNW1);
  globalDutyAvail = os_getTime();
  initDefaultChannels();
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
  next_job = {};
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
  if (dlen > pendTxData.max_size())
    return -2;
  if (data)
    std::copy(data, data + dlen, begin(pendTxData));
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
void Lmic::setSession(uint32_t const newnetid, devaddr_t const newdevaddr,
                      AesKey const &nwkKey, AesKey const &artKey) {
  netid = newnetid;
  devaddr = newdevaddr;
  aes.setNetworkSessionKey(nwkKey);
  aes.setApplicationSessionKey(artKey);

  opmode.reset(OpState::JOINING);
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
void Lmic::setLinkCheckMode(bool const enabled) {
  adrAckReq = enabled ? LINK_CHECK_INIT : LINK_CHECK_OFF;
}

// Sets the max clock error to compensate for (defaults to 0, which
// allows for +/- 640 at SF7BW250). MAX_CLOCK_ERROR represents +/-100%,
// so e.g. for a +/-1% error you would pass MAX_CLOCK_ERROR * 1 / 100.
void Lmic::setClockError(uint8_t const error) { clockError = error; }

rps_t Lmic::updr2rps(dr_t const dr) const { return rps_t(getRawRps(dr)); }

rps_t Lmic::dndr2rps(dr_t const dr) const {
  auto val = updr2rps(dr);
  val.nocrc = true;
  return val;
}

bool Lmic::isFasterDR(dr_t const dr1, dr_t const dr2) const {
  return dr1 > dr2;
}

bool Lmic::isSlowerDR(dr_t const dr1, dr_t const dr2) const {
  return dr1 < dr2;
}

// increase data rate
dr_t Lmic::incDR(dr_t const dr) const { return validDR(dr + 1) ? dr + 1 : dr; }

// decrease data rate
dr_t Lmic::decDR(dr_t const dr) const { return validDR(dr - 1) ? dr - 1 : dr; }

// in range
bool Lmic::validDR(dr_t const dr) const { return getRawRps(dr) != ILLEGAL_RPS; }

// decrease data rate by n steps
dr_t Lmic::lowerDR(dr_t dr, uint8_t n) const {
  while (n--) {
    dr = decDR(dr);
  };
  return dr;
}

OsTime Lmic::int_trigger_time() const {
  OsTime const now = os_getTime();
  auto const diff = now - last_int_trigger;
  if (diff > OsDeltaTime(0) && diff < OsDeltaTime::from_sec(1)) {
    return last_int_trigger;
  } else {
    PRINT_DEBUG(1, F("Not using interupt trigger %" PRIu32 ""),
                last_int_trigger.tick());
    return now;
  }
}

void Lmic::wait_end_rx() {
  if (radio.io_check()) {
    const auto now = int_trigger_time();

    dataLen = radio.handle_end_rx(frame);

    PRINT_DEBUG(1, F("End RX - Start RX : %" PRIi32 " us "),
                (now - rxtime).to_us());
    rxtime = now;

    // if radio task ended, activate job.
    if (opmode.test(OpState::JOINING)) {
      processRxJacc();
    } else {
      processRxDnData();
    }
  } else {
    // if radio has not finish come back later (loop).
    next_job = Job(&Lmic::wait_end_rx);
  }
}

void Lmic::wait_end_tx() {
  if (radio.io_check()) {
    // save exact tx time
    txend = int_trigger_time();

    radio.handle_end_tx();

    PRINT_DEBUG(1, F("End TX  %" PRIu32 ""), txend.tick());
    txDone();
  } else {
    // if radio has not finish come back later (loop).
    next_job = Job(&Lmic::wait_end_tx);
  }
}

void Lmic::store_trigger() { last_int_trigger = os_getTime(); }

#if defined(ENABLE_SAVE_RESTORE)
void Lmic::saveState(StoringAbtract &store) const {
  Lmic::saveStateWithoutTimeData(store);
  store.write(globalDutyAvail);
}

void Lmic::saveStateWithoutTimeData(StoringAbtract &store) const {
  // TODO radio RSSI,SNR
  // TODO check if we can avoid storing rxsyms
  store.write(rxsyms);

  store.write(globalDutyRate);

  store.write(netid);
  store.write(opmode);
  store.write(upRepeat);
  store.write(adrTxPow);
  store.write(datarate);
  store.write(devNonce);
  store.write(devaddr);
  store.write(seqnoDn);
  store.write(seqnoUp);
  store.write(dnConf);
  store.write(adrAckReq);
  store.write(rxDelay);
  store.write(ladrAns);
  store.write(devsAns);
  store.write(rxTimingSetupAns);
#if !defined(DISABLE_MCMD_DCAP_REQ)
  store.write(dutyCapAns);
#endif
#if !defined(DISABLE_MCMD_SNCH_REQ)
  // answer set new channel, init afet join.
  store.write(snchAns);
#endif
  store.write(rx1DrOffset);
  store.write(rx2Parameter.datarate);
  store.write(rx2Parameter.frequency);
#if !defined(DISABLE_MCMD_DN2P_SET)
  store.write(dn2Ans);
#endif
  aes.saveState(store);
}

void Lmic::loadState(RetrieveAbtract &store) {
  Lmic::loadStateWithoutTimeData(store);
  store.read(globalDutyAvail);
}

void Lmic::loadStateWithoutTimeData(RetrieveAbtract &store) {
  // TODO radio RSSI,SNR
  // TODO check if we can avoid storing rxsyms
  store.read(rxsyms);

  store.read(globalDutyRate);

  store.read(netid);
  store.read(opmode);
  store.read(upRepeat);
  store.read(adrTxPow);
  store.read(datarate);
  store.read(devNonce);
  store.read(devaddr);
  store.read(seqnoDn);
  store.read(seqnoUp);
  store.read(dnConf);
  store.read(adrAckReq);
  store.read(rxDelay);
  store.read(ladrAns);
  store.read(devsAns);
  store.read(rxTimingSetupAns);
#if !defined(DISABLE_MCMD_DCAP_REQ)
  store.read(dutyCapAns);
#endif
#if !defined(DISABLE_MCMD_SNCH_REQ)
  // answer set new channel, init afet join.
  store.read(snchAns);
#endif
  store.read(rx1DrOffset);
  store.read(rx2Parameter.datarate);
  store.read(rx2Parameter.frequency);
#if !defined(DISABLE_MCMD_DN2P_SET)
  store.read(dn2Ans);
#endif
  aes.loadState(store);
}

#endif

OsDeltaTime Lmic::run() { return next_job.run(*this); }

Lmic::Lmic(Radio &aradio) : radio(aradio), rand(aes) {}
