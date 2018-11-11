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

#include "radio.h"
#include "../aes/aes.h"
#include "lmic.h"

// ----------------------------------------
// Registers Mapping
#define RegFifo 0x00     // common
#define RegOpMode 0x01   // common
#define RegFrfMsb 0x06   // common
#define RegFrfMid 0x07   // common
#define RegFrfLsb 0x08   // common
#define RegPaConfig 0x09 // common
#define RegPaRamp 0x0A   // common
#define RegOcp 0x0B      // common
#define RegLna 0x0C      // common
#define LORARegFifoAddrPtr 0x0D
#define LORARegFifoTxBaseAddr 0x0E
#define LORARegFifoRxBaseAddr 0x0F
#define LORARegFifoRxCurrentAddr 0x10
#define LORARegIrqFlagsMask 0x11
#define LORARegIrqFlags 0x12
#define LORARegRxNbBytes 0x13
#define LORARegRxHeaderCntValueMsb 0x14
#define LORARegRxHeaderCntValueLsb 0x15
#define LORARegRxPacketCntValueMsb 0x16
#define LORARegRxpacketCntValueLsb 0x17
#define LORARegModemStat 0x18
#define LORARegPktSnrValue 0x19
#define LORARegPktRssiValue 0x1A
#define LORARegRssiValue 0x1B
#define LORARegHopChannel 0x1C
#define LORARegModemConfig1 0x1D
#define LORARegModemConfig2 0x1E
#define LORARegSymbTimeoutLsb 0x1F
#define LORARegPreambleMsb 0x20
#define LORARegPreambleLsb 0x21
#define LORARegPayloadLength 0x22
#define LORARegPayloadMaxLength 0x23
#define LORARegHopPeriod 0x24
#define LORARegFifoRxByteAddr 0x25
#define LORARegModemConfig3 0x26
#define LORARegFeiMsb 0x28
#define LORAFeiMib 0x29
#define LORARegFeiLsb 0x2A
#define LORARegRssiWideband 0x2C
#define LORARegDetectOptimize 0x31
#define LORARegInvertIQ 0x33
#define LORARegDetectionThreshold 0x37
#define LORARegSyncWord 0x39
#define RegDioMapping1 0x40 // common
#define RegDioMapping2 0x41 // common
#define RegVersion 0x42     // common
// #define RegAgcRef                                  0x43 // common
// #define RegAgcThresh1                              0x44 // common
// #define RegAgcThresh2                              0x45 // common
// #define RegAgcThresh3                              0x46 // common
// #define RegPllHop                                  0x4B // common
// #define RegTcxo                                    0x58 // common
#define RegPaDac 0x4D // common
// #define RegPll                                     0x5C // common
// #define RegPllLowPn                                0x5E // common
// #define RegFormerTemp                              0x6C // common
// #define RegBitRateFrac                             0x70 // common

// ----------------------------------------
// spread factors and mode for RegModemConfig2
#define SX1272_MC2_FSK 0x00
#define SX1272_MC2_SF7 0x70
#define SX1272_MC2_SF8 0x80
#define SX1272_MC2_SF9 0x90
#define SX1272_MC2_SF10 0xA0
#define SX1272_MC2_SF11 0xB0
#define SX1272_MC2_SF12 0xC0
// bandwidth for RegModemConfig1
#define SX1272_MC1_BW_125 0x00
#define SX1272_MC1_BW_250 0x40
#define SX1272_MC1_BW_500 0x80
// coding rate for RegModemConfig1
#define SX1272_MC1_CR_4_5 0x08
#define SX1272_MC1_CR_4_6 0x10
#define SX1272_MC1_CR_4_7 0x18
#define SX1272_MC1_CR_4_8 0x20
#define SX1272_MC1_IMPLICIT_HEADER_MODE_ON 0x04 // required for receive
#define SX1272_MC1_RX_PAYLOAD_CRCON 0x02
#define SX1272_MC1_LOW_DATA_RATE_OPTIMIZE 0x01 // mandated for SF11 and SF12
// transmit power configuration for RegPaConfig
#define SX1272_PAC_PA_SELECT_PA_BOOST 0x80
#define SX1272_PAC_PA_SELECT_RFIO_PIN 0x00

// sx1276 RegModemConfig1
#define SX1276_MC1_BW_125 0x70
#define SX1276_MC1_BW_250 0x80
#define SX1276_MC1_BW_500 0x90
#define SX1276_MC1_CR_4_5 0x02
#define SX1276_MC1_CR_4_6 0x04
#define SX1276_MC1_CR_4_7 0x06
#define SX1276_MC1_CR_4_8 0x08

#define SX1276_MC1_IMPLICIT_HEADER_MODE_ON 0x01

// sx1276 RegModemConfig2
#define SX1276_MC2_RX_PAYLOAD_CRCON 0x04

// sx1276 RegModemConfig3
#define SX1276_MC3_LOW_DATA_RATE_OPTIMIZE 0x08
#define SX1276_MC3_AGCAUTO 0x04

// preamble for lora networks (nibbles swapped)
#define LORA_MAC_PREAMBLE 0x34

#define RXLORA_RXMODE_RSSI_REG_MODEM_CONFIG1 0x0A
#ifdef CFG_sx1276_radio
#define RXLORA_RXMODE_RSSI_REG_MODEM_CONFIG2 0x70
#elif CFG_sx1272_radio
#define RXLORA_RXMODE_RSSI_REG_MODEM_CONFIG2 0x74
#endif

// ----------------------------------------
// Constants for radio registers
#define OPMODE_LORA 0x80
#define OPMODE_MASK 0x07
#define OPMODE_SLEEP 0x00
#define OPMODE_STANDBY 0x01
#define OPMODE_FSTX 0x02
#define OPMODE_TX 0x03
#define OPMODE_FSRX 0x04
#define OPMODE_RX 0x05
#define OPMODE_RX_SINGLE 0x06
#define OPMODE_CAD 0x07

// ----------------------------------------
// Bits masking the corresponding IRQs from the radio
#define IRQ_LORA_RXTOUT_MASK 0x80
#define IRQ_LORA_RXDONE_MASK 0x40
#define IRQ_LORA_CRCERR_MASK 0x20
#define IRQ_LORA_HEADER_MASK 0x10
#define IRQ_LORA_TXDONE_MASK 0x08
#define IRQ_LORA_CDDONE_MASK 0x04
#define IRQ_LORA_FHSSCH_MASK 0x02
#define IRQ_LORA_CDDETD_MASK 0x01

// ----------------------------------------
// DIO function mappings                D0D1D2D3
#define MAP_DIO0_LORA_RXDONE 0x00 // 00------
#define MAP_DIO0_LORA_TXDONE 0x40 // 01------
#define MAP_DIO0_LORA_NOP 0xC0    // 11------
#define MAP_DIO1_LORA_RXTOUT 0x00 // --00----
#define MAP_DIO1_LORA_NOP 0x30    // --11----
#define MAP_DIO2_LORA_NOP 0x0C    // ----11--

#define LNA_RX_GAIN (0x20 | 0x03)

void Radio::writeReg(uint8_t addr, uint8_t data) const {
  hal.pin_nss(0);
  hal.spi(addr | 0x80);
  hal.spi(data);
  hal.pin_nss(1);
}

uint8_t Radio::readReg(uint8_t addr) const {
  hal.pin_nss(0);
  hal.spi(addr & 0x7F);
  uint8_t val = hal.spi(0x00);
  hal.pin_nss(1);
  return val;
}

void Radio::writeBuf(uint8_t addr, uint8_t *buf, uint8_t len) const {
  hal.pin_nss(0);
  hal.spi(addr | 0x80);
  for (uint8_t i = 0; i < len; i++) {
    hal.spi(buf[i]);
  }
  hal.pin_nss(1);
}

void Radio::readBuf(uint8_t addr, uint8_t *buf, uint8_t len) const {
  hal.pin_nss(0);
  hal.spi(addr & 0x7F);
  for (uint8_t i = 0; i < len; i++) {
    buf[i] = hal.spi(0x00);
  }
  hal.pin_nss(1);
}

void Radio::opmode(uint8_t mode) const {
  writeReg(RegOpMode, (readReg(RegOpMode) & ~OPMODE_MASK) | mode);
}

void Radio::opmodeLora() const {
  uint8_t u = OPMODE_LORA;
#ifdef CFG_sx1276_radio
  u |= 0x8; // TBD: sx1276 high freq
#endif
  writeReg(RegOpMode, u);
}

// configure LoRa modem (cfg1, cfg2)
void Radio::configLoraModem(rps_t rps) const {
  sf_t sf = rps.sf;

#ifdef CFG_sx1276_radio
  uint8_t mc1 = 0, mc2 = 0, mc3 = 0;

  switch (rps.getBw()) {
  case BandWidth::BW125:
    mc1 |= SX1276_MC1_BW_125;
    break;
  case BandWidth::BW250:
    mc1 |= SX1276_MC1_BW_250;
    break;
  case BandWidth::BW500:
    mc1 |= SX1276_MC1_BW_500;
    break;
  default:
    ASSERT(0);
  }
  switch (rps.getCr()) {
  case CodingRate::CR_4_5:
    mc1 |= SX1276_MC1_CR_4_5;
    break;
  case CodingRate::CR_4_6:
    mc1 |= SX1276_MC1_CR_4_6;
    break;
  case CodingRate::CR_4_7:
    mc1 |= SX1276_MC1_CR_4_7;
    break;
  case CodingRate::CR_4_8:
    mc1 |= SX1276_MC1_CR_4_8;
    break;
  default:
    ASSERT(0);
  }

  if (rps.ih) {
    mc1 |= SX1276_MC1_IMPLICIT_HEADER_MODE_ON;
    writeReg(LORARegPayloadLength, rps.ih); // required length
  }
  // set ModemConfig1
  writeReg(LORARegModemConfig1, mc1);

  mc2 = (SX1272_MC2_SF7 + ((sf - 1) << 4));
  if (!rps.nocrc) {
    mc2 |= SX1276_MC2_RX_PAYLOAD_CRCON;
  }
  writeReg(LORARegModemConfig2, mc2);

  mc3 = SX1276_MC3_AGCAUTO;
  if ((sf == SF11 || sf == SF12) && rps.getBw() == BandWidth::BW125) {
    mc3 |= SX1276_MC3_LOW_DATA_RATE_OPTIMIZE;
  }
  writeReg(LORARegModemConfig3, mc3);
#elif CFG_sx1272_radio
  uint8_t mc1 = (rps.bw << 6);

  switch (rps.cr) {
  case CR_4_5:
    mc1 |= SX1272_MC1_CR_4_5;
    break;
  case CR_4_6:
    mc1 |= SX1272_MC1_CR_4_6;
    break;
  case CR_4_7:
    mc1 |= SX1272_MC1_CR_4_7;
    break;
  case CR_4_8:
    mc1 |= SX1272_MC1_CR_4_8;
    break;
  }

  if ((sf == SF11 || sf == SF12) && rps.bw == BW125) {
    mc1 |= SX1272_MC1_LOW_DATA_RATE_OPTIMIZE;
  }

  if (rps.nocrc == 0) {
    mc1 |= SX1272_MC1_RX_PAYLOAD_CRCON;
  }

  if (rps.ih) {
    mc1 |= SX1272_MC1_IMPLICIT_HEADER_MODE_ON;
    writeReg(LORARegPayloadLength, rps.ih); // required length
  }
  // set ModemConfig1
  writeReg(LORARegModemConfig1, mc1);

  // set ModemConfig2 (sf, AgcAutoOn=1 SymbTimeoutHi=00)
  writeReg(LORARegModemConfig2, (SX1272_MC2_SF7 + ((sf - 1) << 4)) | 0x04);
#else
#error Missing CFG_sx1272_radio/CFG_sx1276_radio
#endif /* CFG_sx1272_radio */
}

void Radio::configChannel(uint32_t freq) const {
  // set frequency: FQ = (FRF * 32 Mhz) / (2 ^ 19)
  uint64_t frf = ((uint64_t)freq << 19) / 32000000;
  writeReg(RegFrfMsb, (uint8_t)(frf >> 16));
  writeReg(RegFrfMid, (uint8_t)(frf >> 8));
  writeReg(RegFrfLsb, (uint8_t)(frf >> 0));
}

#define PA_BOOST_PIN 1

void Radio::configPower(int8_t pw) const {
#ifdef CFG_sx1276_radio

#if PA_BOOST_PIN
  // no boost +20dB used for now
  if (pw > 17) {
    pw = 17;
  } else if (pw < 2) {
    pw = 2;
  }
  pw -= 2;
  // check board type for output pin
  // output on PA_BOOST for RFM95W
  writeReg(RegPaConfig, (uint8_t)(0x80 | pw));
  // no boost +20dB
  writeReg(RegPaDac, (readReg(RegPaDac) & 0xF8) | 0x4);
#else
  // output on rfo pin
  // Bit 6-4 Select max output power: Pmax=10.8+0.6*MaxPower [dBm]
  // Bit 0-3 Pout=Pmax-(15-OutputPower) if PaSelect = 0 (RFO pin)

  if (pw > 15) {
    pw = 15;
  } else if (pw < -4) {
    pw = -4;
  }

  uint8_t pa = 0;
  if (pw >= 0) {
    pa = 7 << 4;
    pa += pw;
  } else {
    pa = 0 << 4;
    // take 11 instead of 10.8
    pa += 15 - 11 + pw;
  }

  writeReg(RegPaConfig, pa);
  // no boost +20dB
  writeReg(RegPaDac, (readReg(RegPaDac) & 0xF8) | 0x4);
#endif
#elif CFG_sx1272_radio
  // set PA config (2-17 dBm using PA_BOOST)
  if (pw > 17) {
    pw = 17;
  } else if (pw < 2) {
    pw = 2;
  }
  writeReg(RegPaConfig, (uint8_t)(0x80 | (pw - 2)));
#else
#error Missing CFG_sx1272_radio/CFG_sx1276_radio
#endif /* CFG_sx1272_radio */
}

uint8_t crForLog(rps_t rps) {
  switch (rps.getCr()) {
  case CodingRate::CR_4_5:
    return 5;
  case CodingRate::CR_4_6:
    return 6;
  case CodingRate::CR_4_7:
    return 7;
  case CodingRate::CR_4_8:
    return 8;
  }
  return 0;
}

uint16_t bwForLog(rps_t rps) {
  switch (rps.getBw()) {
  case BandWidth::BW125:
    return 125;
  case BandWidth::BW250:
    return 250;
  case BandWidth::BW500:
    return 500;
  default:
    return 0;
  }
}

void Radio::txlora(uint32_t freq, rps_t rps, int8_t txpow, uint8_t *frame,
                   uint8_t dataLen) const {
  // select LoRa modem (from sleep mode)
  // writeReg(RegOpMode, OPMODE_LORA);
  opmodeLora();
  ASSERT((readReg(RegOpMode) & OPMODE_LORA) != 0);

  // enter standby mode (required for FIFO loading))
  opmode(OPMODE_STANDBY);
  // configure LoRa modem (cfg1, cfg2)
  configLoraModem(rps);
  // configure frequency
  configChannel(freq);
  // configure output power
  writeReg(RegPaRamp,
           (readReg(RegPaRamp) & 0xF0) | 0x08); // set PA ramp-up time 50 uSec
  configPower(txpow);
  // set sync word
  writeReg(LORARegSyncWord, LORA_MAC_PREAMBLE);

  // set the IRQ mapping DIO0=TxDone DIO1=NOP DIO2=NOP
  writeReg(RegDioMapping1,
           MAP_DIO0_LORA_TXDONE | MAP_DIO1_LORA_NOP | MAP_DIO2_LORA_NOP);
  // clear all radio IRQ flags
  writeReg(LORARegIrqFlags, 0xFF);
  // mask all IRQs but TxDone
  writeReg(LORARegIrqFlagsMask, ~IRQ_LORA_TXDONE_MASK);

  // initialize the payload size and address pointers
  writeReg(LORARegFifoTxBaseAddr, 0x00);
  writeReg(LORARegFifoAddrPtr, 0x00);
  writeReg(LORARegPayloadLength, dataLen);

  // download buffer to the radio FIFO
  writeBuf(RegFifo, frame, dataLen);

  // enable antenna switch for TX
  hal.pin_rxtx(1);

  // now we actually start the transmission
  opmode(OPMODE_TX);
  hal_forbid_sleep();

#if LMIC_DEBUG_LEVEL > 0
  uint8_t sf = rps.sf + 6; // 1 == SF7
  lmic_printf("%lu: TXMODE, freq=%lu, len=%d, SF=%d, BW=%d, CR=4/%d, IH=%d\n",
              os_getTime().tick(), freq, dataLen, sf, bwForLog(rps), crForLog(rps),
              rps.ih);
#endif
}

// start transmitter
void Radio::starttx(uint32_t freq, rps_t rps, int8_t txpow, uint8_t *frame,
                    uint8_t dataLen) const {
  ASSERT((readReg(RegOpMode) & OPMODE_MASK) == OPMODE_SLEEP);
  txlora(freq, rps, txpow, frame, dataLen);
  // the radio will go back to STANDBY mode as soon as the TX is finished
  // the corresponding IRQ will inform us about completion.
}

enum { RXMODE_SINGLE, RXMODE_SCAN, RXMODE_RSSI };

static CONST_TABLE(uint8_t, rxlorairqmask)[] = {
    [RXMODE_SINGLE] = IRQ_LORA_RXDONE_MASK | IRQ_LORA_RXTOUT_MASK,
    [RXMODE_SCAN] = IRQ_LORA_RXDONE_MASK,
    [RXMODE_RSSI] = 0x00,
};

// start LoRa receiver
void Radio::rxlora(uint8_t rxmode, uint32_t freq, rps_t rps, uint8_t rxsyms,
                   OsTime rxtime) const {
  // select LoRa modem (from sleep mode)
  opmodeLora();
  ASSERT((readReg(RegOpMode) & OPMODE_LORA) != 0);
  // enter standby mode (warm up))
  opmode(OPMODE_STANDBY);
  // don't use MAC settings at startup
  if (rxmode == RXMODE_RSSI) { // use fixed settings for rssi scan
    writeReg(LORARegModemConfig1, RXLORA_RXMODE_RSSI_REG_MODEM_CONFIG1);
    writeReg(LORARegModemConfig2, RXLORA_RXMODE_RSSI_REG_MODEM_CONFIG2);
  } else { // single or continuous rx mode
    // configure LoRa modem (cfg1, cfg2)
    configLoraModem(rps);
    // configure frequency
    configChannel(freq);
  }
  // set LNA gain
  writeReg(RegLna, LNA_RX_GAIN);
  // set max payload size
  writeReg(LORARegPayloadMaxLength, 64);
#if !defined(DISABLE_INVERT_IQ_ON_RX)
  // use inverted I/Q signal (prevent mote-to-mote communication)
  writeReg(LORARegInvertIQ, readReg(LORARegInvertIQ) | (1 << 6));
#endif
  // set symbol timeout (for single rx)
  writeReg(LORARegSymbTimeoutLsb, rxsyms);
  // set sync word
  writeReg(LORARegSyncWord, LORA_MAC_PREAMBLE);

  // configure DIO mapping DIO0=RxDone DIO1=RxTout DIO2=NOP
  writeReg(RegDioMapping1,
           MAP_DIO0_LORA_RXDONE | MAP_DIO1_LORA_RXTOUT | MAP_DIO2_LORA_NOP);
  // clear all radio IRQ flags
  writeReg(LORARegIrqFlags, 0xFF);
  // enable required radio IRQs
  writeReg(LORARegIrqFlagsMask, ~TABLE_GET_U1(rxlorairqmask, rxmode));

  // enable antenna switch for RX
  hal.pin_rxtx(0);

  // now instruct the radio to receive
  if (rxmode == RXMODE_SINGLE) { // single rx
    hal_waitUntil(rxtime);       // busy wait until exact rx time
    opmode(OPMODE_RX_SINGLE);
  } else { // continous rx (scan or rssi)
    opmode(OPMODE_RX);
  }
  hal_forbid_sleep();

#if LMIC_DEBUG_LEVEL > 0
  if (rxmode == RXMODE_RSSI) {
    lmic_printf("RXMODE_RSSI\n");
  } else {
    uint8_t sf = rps.sf + 6; // 1 == SF7
    lmic_printf("%lu: %s, freq=%lu, SF=%d, BW=%d, CR=4/%d, IH=%d\n",
                os_getTime().tick(),
                rxmode == RXMODE_SINGLE
                    ? "RXMODE_SINGLE"
                    : (rxmode == RXMODE_SCAN ? "RXMODE_SCAN" : "UNKNOWN_RX"),
                freq, sf, bwForLog(rps), crForLog(rps), rps.ih);
  }
#endif
}

void Radio::startrx(uint8_t rxmode, uint32_t freq, rps_t rps, uint8_t rxsyms,
                    OsTime rxtime) const {
  ASSERT((readReg(RegOpMode) & OPMODE_MASK) == OPMODE_SLEEP);
  rxlora(rxmode, freq, rps, rxsyms, rxtime);
  // the radio will go back to STANDBY mode as soon as the RX is finished
  // or timed out, and the corresponding IRQ will inform us about completion.
}

void Radio::init() {
  hal_disableIRQs();
  hal.init();
  // manually reset radio
#ifdef CFG_sx1276_radio
  hal.pin_rst(0); // drive RST pin low
#else
  hal.pin_rst(1); // drive RST pin high
#endif
  // wait >100us
  hal_wait(OsDeltaTime::from_ms(1));
  hal.pin_rst(2); // configure RST pin floating!
  // wait 5ms
  hal_wait(OsDeltaTime::from_ms(5));

  opmode(OPMODE_SLEEP);

#if !defined(CFG_noassert) || LMIC_DEBUG_LEVEL > 0
  // some sanity checks, e.g., read version number
  uint8_t v = readReg(RegVersion);
  PRINT_DEBUG_1("Chip version : %i", v);
#endif
#ifdef CFG_sx1276_radio
  ASSERT(v == 0x12);
#elif CFG_sx1272_radio
  ASSERT(v == 0x22);
#else
#error Missing CFG_sx1272_radio/CFG_sx1276_radio
#endif

#ifdef CFG_sx1276mb1_board
  // chain calibration
  writeReg(RegPaConfig, 0);

  // Launch Rx chain calibration for LF band
  writeReg(FSKRegImageCal,
           (readReg(FSKRegImageCal) & RF_IMAGECAL_IMAGECAL_MASK) |
               RF_IMAGECAL_IMAGECAL_START);
  while ((readReg(FSKRegImageCal) & RF_IMAGECAL_IMAGECAL_RUNNING) ==
         RF_IMAGECAL_IMAGECAL_RUNNING) {
    ;
  }

  // Sets a Frequency in HF band
  uint32_t frf = 868000000;
  writeReg(RegFrfMsb, (uint8_t)(frf >> 16));
  writeReg(RegFrfMid, (uint8_t)(frf >> 8));
  writeReg(RegFrfLsb, (uint8_t)(frf >> 0));

  // Launch Rx chain calibration for HF band
  writeReg(FSKRegImageCal,
           (readReg(FSKRegImageCal) & RF_IMAGECAL_IMAGECAL_MASK) |
               RF_IMAGECAL_IMAGECAL_START);
  while ((readReg(FSKRegImageCal) & RF_IMAGECAL_IMAGECAL_RUNNING) ==
         RF_IMAGECAL_IMAGECAL_RUNNING) {
    ;
  }
#endif /* CFG_sx1276mb1_board */

  opmode(OPMODE_SLEEP);
  hal_allow_sleep();

  hal_enableIRQs();
}

// get random seed from wideband noise rssi
void Radio::init_random(uint8_t randbuf[16]) {
  hal_disableIRQs();

  // seed 15-byte randomness via noise rssi
  // freq and rps not used
  rps_t dumyrps;
  rxlora(RXMODE_RSSI, 0, dumyrps, 1, hal_ticks());
  while ((readReg(RegOpMode) & OPMODE_MASK) != OPMODE_RX)
    ; // continuous rx
  for (uint8_t i = 1; i < 16; i++) {
    for (uint8_t j = 0; j < 8; j++) {
      uint8_t b; // wait for two non-identical subsequent least-significant bits
      while ((b = readReg(LORARegRssiWideband) & 0x01) ==
             (readReg(LORARegRssiWideband) & 0x01))
        ;
      randbuf[i] = (randbuf[i] << 1) | b;
    }
  }
  randbuf[0] = 16; // set initial index
  opmode(OPMODE_SLEEP);
  hal_enableIRQs();
}

uint8_t Radio::rssi() const {
  hal_disableIRQs();
  uint8_t r = readReg(LORARegRssiValue);
  hal_enableIRQs();
  return r;
}

CONST_TABLE(int32_t, LORA_RXDONE_FIXUP)
[] = {
    [FSK] = OsDeltaTime::from_us(0).tick(), // (   0 ticks)
    [SF7] = OsDeltaTime::from_us(0).tick(), // (   0 ticks)
    [SF8] = OsDeltaTime::from_us(1648).tick(),
    [SF9] = OsDeltaTime::from_us(3265).tick(),
    [SF10] = OsDeltaTime::from_us(7049).tick(),
    [SF11] = OsDeltaTime::from_us(13641).tick(),
    [SF12] = OsDeltaTime::from_us(31189).tick(),
};

OsTime Radio::int_trigger_time() const {
  OsTime now = os_getTime();
  if (now - last_int_trigger < OsDeltaTime::from_sec(1)) {
    return last_int_trigger;
  } else {
    PRINT_DEBUG_1("Not using interupt trigger %lu", last_int_trigger);
    return now;
  }
}

// called by hal ext IRQ handler
// (radio goes to stanby mode after tx/rx operations)
void Radio::irq_handler(uint8_t dio, uint8_t *framePtr, uint8_t &frameLength,
                        OsTime &txEnd, OsTime &rxTime, rps_t currentRps) const {
    OsTime now = int_trigger_time();

  if ((readReg(RegOpMode) & OPMODE_LORA) != 0) { // LORA modem
    uint8_t flags = readReg(LORARegIrqFlags);

    PRINT_DEBUG_2("irq: dio: 0x%x flags: 0x%x\n", dio, flags);

    if (flags & IRQ_LORA_TXDONE_MASK) {
      // save exact tx time
      txEnd = now; 
      PRINT_DEBUG_1("End TX  %li", txEnd);
      // forbid sleep to keep precise time counting.
      // hal_forbid_sleep();
      hal_allow_sleep();

    } else if (flags & IRQ_LORA_RXDONE_MASK) {
      // save exact rx time
      if (currentRps.getBw() == BandWidth::BW125) {
        now -= OsDeltaTime(TABLE_GET_S4(LORA_RXDONE_FIXUP, currentRps.sf));
      }
      PRINT_DEBUG_1("End RX -  Start RX : %li us ", (now - rxTime).to_us());
      rxTime = now;

      // read the PDU and inform the MAC that we received something
      uint8_t length =
          (readReg(LORARegModemConfig1) & SX1272_MC1_IMPLICIT_HEADER_MODE_ON)
              ? readReg(LORARegPayloadLength)
              : readReg(LORARegRxNbBytes);

      // for security clamp length of data
      length = length < MAX_LEN_FRAME ? length : MAX_LEN_FRAME;

      // set FIFO read address pointer
      writeReg(LORARegFifoAddrPtr, readReg(LORARegFifoRxCurrentAddr));
      // now read the FIFO
      readBuf(RegFifo, framePtr, length);
      frameLength = length;

      // read rx quality parameters
      // TODO restore
      // LMIC.snr = readReg(LORARegPktSnrValue); // SNR [dB] * 4
      // LMIC.rssi =
      //    readReg(LORARegPktRssiValue) - 125 + 64; // RSSI [dBm] (-196...+63)
      hal_allow_sleep();
    } else if (flags & IRQ_LORA_RXTOUT_MASK) {
      PRINT_DEBUG_1("RX timeout  %li", now);

      // indicate timeout
      frameLength = 0;
      hal_allow_sleep();
    }
    // mask all radio IRQs
    writeReg(LORARegIrqFlagsMask, 0xFF);
    // clear radio IRQ flags
    writeReg(LORARegIrqFlags, 0xFF);
  }
  // go from stanby to sleep
  opmode(OPMODE_SLEEP);
}

void Radio::rst() const {
  hal_disableIRQs();
  // put radio to sleep
  opmode(OPMODE_SLEEP);
  hal_allow_sleep();
  hal_enableIRQs();
}

void Radio::tx(uint32_t freq, rps_t rps, int8_t txpow, uint8_t *framePtr,
               uint8_t frameLength) const {
  hal_disableIRQs();
  // transmit frame now
  starttx(freq, rps, txpow, framePtr, frameLength);
  hal_enableIRQs();
}

void Radio::rx(uint32_t freq, rps_t rps, uint8_t rxsyms, OsTime rxtime) const {
  hal_disableIRQs();
  // receive frame now (exactly at rxtime)
  startrx(RXMODE_SINGLE, freq, rps, rxsyms, rxtime);
  hal_enableIRQs();
}

void Radio::rxon(uint32_t freq, rps_t rps, uint8_t rxsyms,
                 OsTime rxtime) const {
  hal_disableIRQs();
  // start scanning for beacon now
  startrx(RXMODE_SCAN, freq, rps, rxsyms, rxtime);
  hal_enableIRQs();
}

/**
 *  Return true if the radio has finish it's operation 
 */
bool Radio::io_check(uint8_t *framePtr, uint8_t &frameLength, OsTime &txEnd,
                     OsTime &rxTime, rps_t currentRps) {
  auto pinInInt = hal.io_check();
  if (pinInInt < NUM_DIO) {
    irq_handler(pinInInt, framePtr, frameLength, txEnd, rxTime, currentRps);
    return true;
  }
  return false;
}

void Radio::store_trigger() { last_int_trigger = os_getTime(); }

Radio::Radio(lmic_pinmap const &pins) : hal(pins) {}
