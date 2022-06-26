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

#ifndef _lorabase_h_
#define _lorabase_h_

#include "oslmic.h"
#include <array>

enum class CodingRate : uint8_t { CR_4_5 = 0, CR_4_6, CR_4_7, CR_4_8 };
enum _sf_t { FSK = 0, SF7, SF8, SF9, SF10, SF11, SF12, SFrfu };
enum class BandWidth { BW125 = 0, BW250, BW500 };

typedef uint8_t sf_t;
typedef uint8_t dr_t;

// Radio parameter set (encodes SF/BW/CR/NOCRC)
struct rps_t {
  sf_t sf : 3;
  uint8_t bwRaw : 2;
  uint8_t crRaw : 2;
  bool nocrc : 1;

  constexpr BandWidth getBw() const { return static_cast<BandWidth>(bwRaw); };
  constexpr CodingRate getCr() const { return static_cast<CodingRate>(crRaw); };

  constexpr operator uint8_t() const {
    return (sf | (bwRaw << 3) | (crRaw << 5) | (nocrc ? (1 << 7) : 0));
  }

  constexpr rps_t(sf_t asf, BandWidth bw, CodingRate cr, bool anocrc = false)
      : sf(asf), bwRaw(static_cast<uint8_t>(bw)),
        crRaw(static_cast<uint8_t>(cr)), nocrc(anocrc){};
  explicit constexpr rps_t(uint8_t rawValue)
      : sf(rawValue & 0x07), bwRaw((rawValue >> 3) & 0x03),
        crRaw((rawValue >> 5) & 0x03), nocrc(rawValue & (1 << 7)){};
  constexpr rps_t() : rps_t(0){};
};

constexpr uint8_t ILLEGAL_RPS = 0xFF;

// Global maximum frame length
constexpr uint8_t STD_PREAMBLE_LEN = 8;
constexpr uint8_t MAX_LEN_FRAME = 64;
constexpr uint8_t MAX_LEN_FOPTS = 15;
constexpr uint8_t DELAY_JACC1 = 5;   // in secs
constexpr uint8_t DELAY_DNW1 = 1;    // in secs down window #1
constexpr uint8_t DELAY_EXTDNW2 = 1; // in secs
constexpr uint8_t DELAY_JACC2 = DELAY_JACC1 + DELAY_EXTDNW2; // in secs
constexpr uint8_t DELAY_DNW2 =
    DELAY_DNW1 + DELAY_EXTDNW2; // in secs down window #1

using FrameBuffer = std::array<uint8_t, MAX_LEN_FRAME>;

enum class PktDir : uint8_t {
  UP = 0,
  DOWN = 1,
};
constexpr uint8_t MAX_LEN_PAYLOAD = MAX_LEN_FRAME - 8 - 4;

enum {
  // Bitfields in frame control octet
  FCT_ADREN = 0x80,
  FCT_ADRARQ = 0x40,
  FCT_ACK = 0x20,
  // Fpending
  FCT_MORE = 0x10,
  FCT_OPTLEN = 0x0F,
};
enum {
  // In UP direction: signals class B enabled
  FCT_CLASSB = FCT_MORE
};
enum { NWKID_MASK = (int)0xFE000000, NWKID_BITS = 7 };

// MAC uplink commands   downwlink too
enum {
  // Class A
  MCMD_LCHK_REQ = 0x02, // -  link check request : -
  MCMD_LADR_ANS =
      0x03, // -  link ADR answer    : u1:7-3:RFU, 3/2/1: pow/DR/Ch ACK
  MCMD_DCAP_ANS = 0x04, // -  duty cycle answer  : -
  MCMD_DN2P_ANS =
      0x05, // -  2nd DN slot status : u1:7-2:RFU  1/0:datarate/channel ack
  // -  device status ans  : u1:battery 0,1-254,255=?,
  // u1:7-6:RFU,5-0:margin(-32..31)
  MCMD_DEVS_ANS = 0x06,
  // -  set new channel    : u1: 7-2=RFU, 1/0:DR/freq ACK
  MCMD_SNCH_ANS = 0x07,
  // Ack to new RX 1 timing.
  MCMD_RXTimingSetup_ANS = 0x08,
  MCMD_TxParamSetup_ANS = 0x09,
  MCMD_DlChannel_ANS = 0x0A,
  // Device time req
  MCMD_DeviceTime_REQ = 0x0D,
  // Class B
  MCMD_PING_IND =
      0x10, // -  pingability indic  : u1: 7=RFU, 6-4:interval, 3-0:datarate
  MCMD_PING_ANS = 0x11, // -  ack ping freq      : u1: 7-1:RFU, 0:freq ok
  MCMD_BCNI_REQ = 0x12, // -  next beacon start  : -

};

// MAC downlink commands
enum {
  // Class A
  // link check answer  : u1:margin 0-254,255=unknown margin / u1:gwcnt
  MCMD_LCHK_ANS = 0x02,
  // link ADR request   : u1:DR/TXPow, u2:chmask, u1:chpage/repeat
  MCMD_LADR_REQ = 0x03,
  // duty cycle cap     : u1:255 dead [7-4]:RFU, [3-0]:cap 2^-k
  MCMD_DCAP_REQ = 0x04,
  // 2nd DN window param: u1:7-4:RFU/3-0:datarate, u3:freq
  MCMD_DN2P_SET = 0x05,
  // device status req  : -
  MCMD_DEVS_REQ = 0x06,
  // set new channel    : u1:chidx, u3:freq, u1:DRrange
  MCMD_NewChannel_REQ = 0x07,
  // Sets the timing of the of the reception slots  u1: [7-4]:RFU, [3-0]:del
  MCMD_RXTimingSetup_REQ = 0x08,
  //  set the maximum allowed dwell time
  MCMD_TxParamSetup_REQ = 0x09,
};

enum {
  MCMD_LADR_ANS_RFU = 0xF8,    // RFU bits
  MCMD_LADR_ANS_POWACK = 0x04, // 0=not supported power level
  MCMD_LADR_ANS_DRACK = 0x02,  // 0=unknown data rate
  MCMD_LADR_ANS_CHACK = 0x01,  // 0=unknown channel enabled
};
enum {
  MCMD_DN2P_ANS_RFU = 0xF8,            // RFU bits
  MCMD_DN2P_ANS_RX1DrOffsetAck = 0x04, // 0=dr2 not allowed
  MCMD_DN2P_ANS_DRACK = 0x02,          // 0=unknown data rate
  MCMD_DN2P_ANS_CHACK = 0x01,          // 0=unknown channel enabled
};
enum {
  MCMD_SNCH_ANS_RFU = 0xFC,   // RFU bits
  MCMD_SNCH_ANS_DRACK = 0x02, // 0=unknown data rate
  MCMD_SNCH_ANS_FQACK = 0x01, // 0=rejected channel frequency
};

enum {
  MCMD_DEVS_EXT_POWER = 0x00,   // external power supply
  MCMD_DEVS_BATT_MIN = 0x01,    // min battery value
  MCMD_DEVS_BATT_MAX = 0xFE,    // max battery value
  MCMD_DEVS_BATT_NOINFO = 0xFF, // unknown battery level
};

// Bit fields byte#3 of MCMD_LADR_REQ payload

constexpr uint8_t MCMD_LADR_CHPAGE_MASK = 0x70;
constexpr uint8_t MCMD_LADR_CHPAGE_OFFSET = 4;
constexpr uint8_t MCMD_LADR_REPEAT_MASK = 0x0F;

// Bit fields byte#0 of MCMD_LADR_REQ payload
enum {
  MCMD_LADR_DR_MASK = 0xF0,
  MCMD_LADR_POW_MASK = 0x0F,
  MCMD_LADR_DR_SHIFT = 4,
  MCMD_LADR_POW_SHIFT = 0,
};

// Device address
typedef uint32_t devaddr_t;

// RX quality (device)
enum { SNR_SCALEUP = 4 };

#endif // _lorabase_h_
