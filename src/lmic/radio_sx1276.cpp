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

#include "radio_sx1276.h"
#include "../hal/print_debug.h"

#include "../aes/lmic_aes.h"
#include "lmic_table.h"

namespace {
// ----------------------------------------
// Registers Mapping
constexpr uint8_t RegFifo = 0x00;     // common
constexpr uint8_t RegOpMode = 0x01;   // common
constexpr uint8_t RegFrfMsb = 0x06;   // common
constexpr uint8_t RegFrfMid = 0x07;   // common
constexpr uint8_t RegFrfLsb = 0x08;   // common
constexpr uint8_t RegPaConfig = 0x09; // common
constexpr uint8_t RegPaRamp = 0x0A;   // common
constexpr uint8_t RegOcp = 0x0B;      // common
constexpr uint8_t RegLna = 0x0C;      // common
constexpr uint8_t LORARegFifoAddrPtr = 0x0D;
constexpr uint8_t LORARegFifoTxBaseAddr = 0x0E;
constexpr uint8_t LORARegFifoRxBaseAddr = 0x0F;
constexpr uint8_t LORARegFifoRxCurrentAddr = 0x10;
constexpr uint8_t LORARegIrqFlagsMask = 0x11;
constexpr uint8_t LORARegIrqFlags = 0x12;
constexpr uint8_t LORARegRxNbBytes = 0x13;
constexpr uint8_t LORARegRxHeaderCntValueMsb = 0x14;
constexpr uint8_t LORARegRxHeaderCntValueLsb = 0x15;
constexpr uint8_t LORARegRxPacketCntValueMsb = 0x16;
constexpr uint8_t LORARegRxpacketCntValueLsb = 0x17;
constexpr uint8_t LORARegModemStat = 0x18;
constexpr uint8_t LORARegPktSnrValue = 0x19;
constexpr uint8_t LORARegPktRssiValue = 0x1A;
constexpr uint8_t LORARegRssiValue = 0x1B;
constexpr uint8_t LORARegHopChannel = 0x1C;
constexpr uint8_t LORARegModemConfig1 = 0x1D;
constexpr uint8_t LORARegModemConfig2 = 0x1E;
constexpr uint8_t LORARegSymbTimeoutLsb = 0x1F;
constexpr uint8_t LORARegPreambleMsb = 0x20;
constexpr uint8_t LORARegPreambleLsb = 0x21;
constexpr uint8_t LORARegPayloadLength = 0x22;
constexpr uint8_t LORARegPayloadMaxLength = 0x23;
constexpr uint8_t LORARegHopPeriod = 0x24;
constexpr uint8_t LORARegFifoRxByteAddr = 0x25;
constexpr uint8_t LORARegModemConfig3 = 0x26;
constexpr uint8_t LORARegFeiMsb = 0x28;
constexpr uint8_t LORAFeiMib = 0x29;
constexpr uint8_t LORARegFeiLsb = 0x2A;
constexpr uint8_t LORARegRssiWideband = 0x2C;
constexpr uint8_t LORARegDetectOptimize = 0x31;
constexpr uint8_t LORARegInvertIQ = 0x33;
constexpr uint8_t LORARegDetectionThreshold = 0x37;
constexpr uint8_t LORARegSyncWord = 0x39;
constexpr uint8_t RegDioMapping1 = 0x40; // common
constexpr uint8_t RegDioMapping2 = 0x41; // common
constexpr uint8_t RegVersion = 0x42;     // common
constexpr uint8_t RegPaDac = 0x4D;       // common

// ----------------------------------------
// spread factors and mode for RegModemConfig2
constexpr uint8_t sf_to_mc2(sf_t sf) { return (7 - SF7 + sf) << 4; }
// sx1276 RegModemConfig2
constexpr uint8_t MC2_RX_PAYLOAD_CRCON = 1 << 2;

constexpr uint8_t MC1_BW_OFFSET = 4;
constexpr uint8_t MC1_IDX_BW_125 = 7;
constexpr uint8_t MC1_IDX_BW_250 = 8;
constexpr uint8_t MC1_IDX_BW_500 = 9;
// sx1276 RegModemConfig1
constexpr uint8_t bw_to_mc1(BandWidth bw) {
  return (MC1_IDX_BW_125 - static_cast<uint8_t>(BandWidth::BW125) +
          static_cast<uint8_t>(bw))
         << MC1_BW_OFFSET;
}

constexpr uint8_t MC1_CR_OFFSET = 1;
constexpr uint8_t MC1_IDX_CR_4_5 = 1;
constexpr uint8_t MC1_IDX_CR_4_6 = 2;
constexpr uint8_t MC1_IDX_CR_4_7 = 3;
constexpr uint8_t MC1_IDX_CR_4_8 = 4;

constexpr uint8_t cr_to_mc1(CodingRate cr) {
  return (MC1_IDX_CR_4_5 - static_cast<uint8_t>(CodingRate::CR_4_5) +
          static_cast<uint8_t>(cr))
         << MC1_CR_OFFSET;
}
constexpr uint8_t MC1_IMPLICIT_HEADER_MODE_ON = 0x01;

// RegModemConfig3
constexpr uint8_t MC3_LOW_DATA_RATE_OPTIMIZE = 0x08;
constexpr uint8_t MC3_AGCAUTO = 0x04;

// preamble for lora networks (nibbles swapped)
constexpr uint8_t LORA_MAC_PREAMBLE = 0x34;

constexpr uint8_t RXLORA_RXMODE_RSSI_REG_MODEM_CONFIG1 = 0x0A;
constexpr uint8_t RXLORA_RXMODE_RSSI_REG_MODEM_CONFIG2 = 0x70;

// ----------------------------------------
// Constants for radio registers
constexpr uint8_t OPMODE_LORA = 0x80;
constexpr uint8_t OPMODE_MASK = 0x07;
constexpr uint8_t OPMODE_SLEEP = 0x00;
constexpr uint8_t OPMODE_STANDBY = 0x01;
constexpr uint8_t OPMODE_FSTX = 0x02;
constexpr uint8_t OPMODE_TX = 0x03;
constexpr uint8_t OPMODE_FSRX = 0x04;
constexpr uint8_t OPMODE_RX = 0x05;
constexpr uint8_t OPMODE_RX_SINGLE = 0x06;
constexpr uint8_t OPMODE_CAD = 0x07;

// ----------------------------------------
// Bits masking the corresponding IRQs from the radio
constexpr uint8_t IRQ_LORA_RXTOUT_MASK = 0x80;
constexpr uint8_t IRQ_LORA_RXDONE_MASK = 0x40;
constexpr uint8_t IRQ_LORA_CRCERR_MASK = 0x20;
constexpr uint8_t IRQ_LORA_HEADER_MASK = 0x10;
constexpr uint8_t IRQ_LORA_TXDONE_MASK = 0x08;
constexpr uint8_t IRQ_LORA_CDDONE_MASK = 0x04;
constexpr uint8_t IRQ_LORA_FHSSCH_MASK = 0x02;
constexpr uint8_t IRQ_LORA_CDDETD_MASK = 0x01;

// ----------------------------------------
// DIO function mappings                D0D1D2D3
constexpr uint8_t MAP_DIO0_LORA_RXDONE = 0x00; // 00------
constexpr uint8_t MAP_DIO0_LORA_TXDONE = 0x40; // 01------
constexpr uint8_t MAP_DIO0_LORA_NOP = 0xC0;    // 11------
constexpr uint8_t MAP_DIO1_LORA_RXTOUT = 0x00; // --00----
constexpr uint8_t MAP_DIO1_LORA_NOP = 0x30;    // --11----
constexpr uint8_t MAP_DIO2_LORA_NOP = 0x0C;    // ----11--

constexpr uint8_t LNA_RX_GAIN = (0x20 | 0x03);

constexpr uint8_t crForLog(rps_t const rps) {
  return (5 - static_cast<uint8_t>(CodingRate::CR_4_5) +
          static_cast<uint8_t>(rps.getCr()));
}

CONST_TABLE(uint16_t, BW_ENUM_TO_VAL)[] = {125, 250, 500, 0};

uint16_t bwForLog(rps_t const rps) {
  auto index = static_cast<uint8_t>(rps.getBw());
  return TABLE_GET_U2(BW_ENUM_TO_VAL, index);
}

} // namespace

void RadioSx1276::write_list_of_reg(uint16_t const *const listcmd,
                                    uint8_t nb_cmd) const {
  for (uint8_t i = 0; i < nb_cmd; i++) {
    RegSet cmd{table_get_u2(listcmd, i)};
    hal.write_reg(cmd.reg, cmd.val);
  }
}

void RadioSx1276::opmode(uint8_t const mode) const {
  hal.write_reg(RegOpMode, (hal.read_reg(RegOpMode) & ~OPMODE_MASK) | mode);
}

void RadioSx1276::opmodeLora() const {
  uint8_t u = OPMODE_LORA;
  hal.write_reg(RegOpMode, u);
}

// configure LoRa modem (cfg1, cfg2)
void RadioSx1276::configLoraModem(rps_t rps) {
  sf_t const sf = rps.sf;

  uint8_t mc1 = bw_to_mc1(rps.getBw());
  mc1 |= cr_to_mc1(rps.getCr());
  // set ModemConfig1
  hal.write_reg(LORARegModemConfig1, mc1);

  uint8_t mc2 = sf_to_mc2(sf);
  if (!rps.nocrc) {
    mc2 |= MC2_RX_PAYLOAD_CRCON;
  }
  hal.write_reg(LORARegModemConfig2, mc2);

  uint8_t mc3 = MC3_AGCAUTO;
  if (((sf == SF11 || sf == SF12) && rps.getBw() == BandWidth::BW125) ||
      (sf == SF12 && rps.getBw() == BandWidth::BW250)) {
    mc3 |= MC3_LOW_DATA_RATE_OPTIMIZE;
  }
  hal.write_reg(LORARegModemConfig3, mc3);
}

void RadioSx1276::configChannel(uint32_t const freq) const {
  // set frequency: FQ = (FRF * 32 Mhz) / (2 ^ 19)
  uint64_t const frf = ((uint64_t)freq << 19) / 32000000;
  hal.write_reg(RegFrfMsb, (uint8_t)(frf >> 16));
  hal.write_reg(RegFrfMid, (uint8_t)(frf >> 8));
  hal.write_reg(RegFrfLsb, (uint8_t)(frf >> 0));
}

#define PA_BOOST_PIN 1

void RadioSx1276::configPower(int8_t pw) const {

#if PA_BOOST_PIN
  // no boost +20dB used for now
  if (pw > 17) {
    pw = 17;
  } else if (pw < 2) {
    pw = 2;
  }
  PRINT_DEBUG(1, F("Config power to %i on PA_BOOST"), pw);

  pw -= 2;
  // check board type for output pin
  // output on PA_BOOST for RFM95W
  hal.write_reg(RegPaConfig, (uint8_t)(0x80 | pw));
  // no boost +20dB
  hal.write_reg(RegPaDac, (hal.read_reg(RegPaDac) & 0xF8) | 0x4);

#else
  // output on rfo pin
  // Bit 6-4 Select max output power: Pmax=10.8+0.6*MaxPower [dBm]
  // Bit 0-3 Pout=Pmax-(15-OutputPower) if PaSelect = 0 (RFO pin)

  if (pw > 15) {
    pw = 15;
  } else if (pw < -4) {
    pw = -4;
  }

  PRINT_DEBUG(1, F("Config power to %i on RFO"), pw);

  uint8_t pa = 0;
  if (pw >= 0) {
    pa = 7 << 4;
    pa += pw;
  } else {
    pa = 0 << 4;
    // take 11 instead of 10.8
    pa += 15 - 11 + pw;
  }

  hal.write_reg(RegPaConfig, pa);
  // no boost +20dB
  hal.write_reg(RegPaDac, (hal.read_reg(RegPaDac) & 0xF8) | 0x4);
#endif
}

// start LoRa receiver
void RadioSx1276::rxrssi() const {
  // select LoRa modem (from sleep mode)
  opmodeLora();
  // enter standby mode (warm up))
  opmode(OPMODE_STANDBY);
  // don't use MAC settings at startup
  // use fixed settings for rssi scan
  hal.write_reg(LORARegModemConfig1, RXLORA_RXMODE_RSSI_REG_MODEM_CONFIG1);
  hal.write_reg(LORARegModemConfig2, RXLORA_RXMODE_RSSI_REG_MODEM_CONFIG2);
  // set LNA gain
  hal.write_reg(RegLna, LNA_RX_GAIN);

  clear_irq();
  // enable antenna switch for RX
  hal.pin_rxtx(0);
  // now instruct the radio to receive
  // continous rx
  opmode(OPMODE_RX);
  PRINT_DEBUG(1, F("RXMODE_RSSI"));
}

void RadioSx1276::init() {
  DisableIRQsGard irqguard;
  hal.init();
  // manually reset radio
  // drive RST pin low
  hal.pin_rst(0);
  // wait >100us for SX127x to detect reset
  hal_wait(OsDeltaTime::from_ms(1));
  // configure RST pin floating!
  hal.pin_rst(2);
  // wait 5ms after reset
  hal_wait(OsDeltaTime::from_ms(5));

  // some sanity checks, e.g., read version number
  uint8_t const v = hal.read_reg(RegVersion);
  PRINT_DEBUG(1, F("Chip version : %i"), v);
  ASSERT(v == 0x12);

  /* TODO add a parameter
  // Configure max curent
  // limit current to 45mA
  constexpr uint8_t limit = 0;
  hal.write_reg(RegOcp, 0x20 | limit);
  */
  opmode(OPMODE_SLEEP);
}

// get random seed from wideband noise rssi
void RadioSx1276::init_random(uint8_t randbuf[16]) {
  DisableIRQsGard irqguard;

  // seed 15-byte randomness via noise rssi
  rxrssi();
  while ((hal.read_reg(RegOpMode) & OPMODE_MASK) != OPMODE_RX)
    ; // continuous rx
  for (uint8_t i = 1; i < 16; i++) {
    for (uint8_t j = 0; j < 8; j++) {
      uint8_t b; // wait for two non-identical subsequent least-significant bits
      while ((b = hal.read_reg(LORARegRssiWideband) & 0x01) ==
             (hal.read_reg(LORARegRssiWideband) & 0x01))
        ;
      randbuf[i] = (randbuf[i] << 1) | b;
    }
  }

  // stop RX
  opmode(OPMODE_SLEEP);
}

uint8_t RadioSx1276::rssi() const {
  DisableIRQsGard irqguard;
  uint8_t const r = hal.read_reg(LORARegRssiValue);
  return r;
}

// called by hal ext IRQ handler
// (radio goes to stanby mode after tx/rx operations)
uint8_t RadioSx1276::handle_end_rx(uint8_t *const framePtr) {

  uint8_t const flags = hal.read_reg(LORARegIrqFlags);
  PRINT_DEBUG(2, F("irq: flags: 0x%x\n"), flags);

  uint8_t length = 0;
  if (flags & IRQ_LORA_RXDONE_MASK) {
    // read the PDU and inform the MAC that we received something
    length = hal.read_reg(LORARegRxNbBytes);

    // for security clamp length of data
    length = length < MAX_LEN_FRAME ? length : MAX_LEN_FRAME;

    // set FIFO read address pointer
    hal.write_reg(LORARegFifoAddrPtr, hal.read_reg(LORARegFifoRxCurrentAddr));
    // now read the FIFO
    hal.read_buffer(RegFifo, framePtr, length);

    // read rx quality parameters
    // SNR [dB] * 4
    last_packet_snr_reg = static_cast<int8_t>(hal.read_reg(LORARegPktSnrValue));
    // RSSI [dBm]  - 139
    last_packet_rssi_reg = hal.read_reg(LORARegPktRssiValue);
  } else if (flags & IRQ_LORA_RXTOUT_MASK) {
    // indicate timeout
    PRINT_DEBUG(1, F("RX timeout"));
  }
  clear_irq();
  // go from stanby to sleep
  opmode(OPMODE_SLEEP);

  // 0 in case of timeout.
  return length;
}

void RadioSx1276::handle_end_tx() const {
  clear_irq();
  // go from stanby to sleep
  opmode(OPMODE_SLEEP);
}

void RadioSx1276::clear_irq() const {
  // mask all radio IRQs
  hal.write_reg(LORARegIrqFlagsMask, 0xFF);
  // clear radio IRQ flags
  hal.write_reg(LORARegIrqFlags, 0xFF);
}

void RadioSx1276::rst() const {
  DisableIRQsGard irqguard;
  // put radio to sleep
  opmode(OPMODE_SLEEP);
}

CONST_TABLE(uint16_t, TX_INIT_CMD)
[] = {
    // set sync word
    RegSet(LORARegSyncWord, LORA_MAC_PREAMBLE).raw(),
    // set the IRQ mapping DIO0=TxDone DIO1=NOP DIO2=NOP
    RegSet(RegDioMapping1,
           MAP_DIO0_LORA_TXDONE | MAP_DIO1_LORA_NOP | MAP_DIO2_LORA_NOP)
        .raw(),
    // clear all radio IRQ flags
    RegSet(LORARegIrqFlags, 0xFF).raw(),
    // mask all IRQs but TxDone
    RegSet(LORARegIrqFlagsMask, ~IRQ_LORA_TXDONE_MASK).raw(),

    // initialize the payload size and address pointers
    RegSet(LORARegFifoTxBaseAddr, 0x00).raw(),
    RegSet(LORARegFifoAddrPtr, 0x00).raw(),
};

constexpr uint8_t NB_TX_INIT_CMD =
    sizeof(RESOLVE_TABLE(TX_INIT_CMD)) / sizeof(RESOLVE_TABLE(TX_INIT_CMD)[0]);

void RadioSx1276::tx(uint32_t const freq, rps_t const rps, int8_t const txpow,
                     uint8_t const *const framePtr, uint8_t const frameLength) {
  DisableIRQsGard irqguard;
  // select LoRa modem (from sleep mode)
  opmodeLora();
  // enter standby mode (required for FIFO loading))
  opmode(OPMODE_STANDBY);
  // configure LoRa modem (cfg1, cfg2)
  configLoraModem(rps);
  // configure frequency
  configChannel(freq);
  // configure output power
  // set PA ramp-up time 50 uSec
  hal.write_reg(RegPaRamp, (hal.read_reg(RegPaRamp) & 0xF0) | 0x08);
  configPower(txpow);

  write_list_of_reg(RESOLVE_TABLE(TX_INIT_CMD), NB_TX_INIT_CMD);

  hal.write_reg(LORARegPayloadLength, frameLength);

  // download buffer to the radio FIFO
  hal.write_buffer(RegFifo, framePtr, frameLength);

  // enable antenna switch for TX
  hal.pin_rxtx(1);

  // now we actually start the transmission
  opmode(OPMODE_TX);

  PRINT_DEBUG(1, F("TXMODE, freq=%" PRIu32 ", len=%d, SF=%d, BW=%d, CR=4/%d"),
              freq, frameLength, rps.sf + 6, bwForLog(rps), crForLog(rps));
  // the radio will go back to STANDBY mode as soon as the TX is finished
  // the corresponding IRQ will inform us about completion.
}

CONST_TABLE(uint16_t, RX_INIT_CMD)
[] = {
    // set LNA gain
    RegSet(RegLna, LNA_RX_GAIN).raw(),
    // set max payload size
    RegSet(LORARegPayloadMaxLength, 64).raw(),
    // set sync word
    RegSet(LORARegSyncWord, LORA_MAC_PREAMBLE).raw(),
    // configure DIO mapping DIO0=RxDone DIO1=RxTout DIO2=NOP
    RegSet(RegDioMapping1,
           MAP_DIO0_LORA_RXDONE | MAP_DIO1_LORA_RXTOUT | MAP_DIO2_LORA_NOP)
        .raw(),
    // clear all radio IRQ flags
    RegSet(LORARegIrqFlags, 0xFF).raw(),
    // enable required radio IRQs
    RegSet(LORARegIrqFlagsMask,
           (uint8_t) ~(IRQ_LORA_RXDONE_MASK | IRQ_LORA_RXTOUT_MASK))
        .raw(),

};

constexpr uint8_t NB_RX_INIT_CMD =
    sizeof(RESOLVE_TABLE(RX_INIT_CMD)) / sizeof(RESOLVE_TABLE(RX_INIT_CMD)[0]);

void RadioSx1276::rx(uint32_t const freq, rps_t const rps, uint8_t const rxsyms,
                     OsTime const rxtime) {
  DisableIRQsGard irqguard;
  // receive frame now (exactly at rxtime)
  // select LoRa modem (from sleep mode)
  opmodeLora();
  ASSERT((hal.read_reg(RegOpMode) & OPMODE_LORA) != 0);
  // enter standby mode (warm up))
  opmode(OPMODE_STANDBY);
  // don't use MAC settings at startup
  // configure LoRa modem (cfg1, cfg2)
  configLoraModem(rps);
  // configure frequency
  configChannel(freq);

  // set symbol timeout (for single rx)
  hal.write_reg(LORARegSymbTimeoutLsb, rxsyms);
#if !defined(DISABLE_INVERT_IQ_ON_RX)
  // use inverted I/Q signal (prevent mote-to-mote communication)
  hal.write_reg(LORARegInvertIQ, hal.read_reg(LORARegInvertIQ) | (1 << 6));
#endif
  write_list_of_reg(RESOLVE_TABLE(RX_INIT_CMD), NB_RX_INIT_CMD);

  // enable antenna switch for RX
  hal.pin_rxtx(0);

  // now instruct the radio to receive
  // busy wait until exact rx time
  hal_waitUntil(rxtime); 
  // single rx
  opmode(OPMODE_RX_SINGLE);

  PRINT_DEBUG(
      1, F("RXMODE_SINGLE, freq=%" PRIu32 ", SF=%d, BW=%d, CR=4/%d, IH=%d"),
      freq, rps.sf + 6, bwForLog(rps), crForLog(rps));
  // the radio will go back to STANDBY mode as soon as the RX is finished
  // or timed out, and the corresponding IRQ will inform us about completion.
}

RadioSx1276::RadioSx1276(lmic_pinmap const &pins) : Radio(pins) {}
