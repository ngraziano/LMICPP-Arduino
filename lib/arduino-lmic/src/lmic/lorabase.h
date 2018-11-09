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

#ifndef _lorabase_h_
#define _lorabase_h_

#include "oslmic.h"

// ================================================================================
// BEG: Keep in sync with lorabase.hpp
//

enum class CodingRate : uint8_t { CR_4_5 = 0, CR_4_6, CR_4_7, CR_4_8 };
enum _sf_t { FSK = 0, SF7, SF8, SF9, SF10, SF11, SF12, SFrfu };
enum class BandWidth { BW125 = 0, BW250, BW500, BWrfu };

typedef uint8_t sf_t;
typedef uint8_t dr_t;

// Radio parameter set (encodes SF/BW/CR/IH/NOCRC)

struct rps_t {
  sf_t sf : 3;
  uint8_t bwRaw : 2;
  uint8_t crRaw : 2;
  bool nocrc : 1;
  uint8_t ih : 8;

  constexpr BandWidth getBw() { return static_cast<BandWidth>(bwRaw); };
  constexpr CodingRate getCr() { return static_cast<CodingRate>(crRaw); };
  constexpr uint16_t rawValue() {
    return (sf | (bwRaw << 3) | (crRaw << 5) | (nocrc ? (1 << 7) : 0) |
            (ih << 8));
  }

  constexpr rps_t(sf_t sf, BandWidth bw, CodingRate cr, bool nocrc, uint8_t ih)
      : sf(sf), bwRaw(static_cast<uint8_t>(bw)),
        crRaw(static_cast<uint8_t>(cr)), nocrc(nocrc), ih(ih){};
  constexpr rps_t(uint16_t rawValue)
      : sf(rawValue & 0x07), bwRaw((rawValue >> 3) & 0x03),
        crRaw((rawValue >> 5) & 0x03), nocrc(rawValue & (1 << 7)),
        ih((rawValue >> 8) & 0xFF)
        //: rawValue(rawValue)
        {};
  rps_t(){};
};


const uint8_t ILLEGAL_RPS = 0xFF;

// Global maximum frame length
enum { STD_PREAMBLE_LEN = 8 };
enum { MAX_LEN_FRAME = 64 };
enum { LEN_DEVNONCE = 2 };
enum { LEN_ARTNONCE = 3 };
enum { LEN_NETID = 3 };
enum { DELAY_JACC1 = 5 };   // in secs
enum { DELAY_DNW1 = 1 };    // in secs down window #1
enum { DELAY_EXTDNW2 = 1 }; // in secs
enum { DELAY_JACC2 = DELAY_JACC1 + (int)DELAY_EXTDNW2 }; // in secs
enum { DELAY_DNW2 = DELAY_DNW1 + (int)DELAY_EXTDNW2 }; // in secs down window #1

enum {
  // Data frame format
  OFF_DAT_HDR = 0,
  OFF_DAT_ADDR = 1,
  OFF_DAT_FCT = 5,
  OFF_DAT_SEQNO = 6,
  OFF_DAT_OPTS = 8,
};

const uint8_t MIC_LEN = 4;

enum class PktDir : uint8_t {
  UP = 0,
  DOWN = 1,
};
enum { MAX_LEN_PAYLOAD = MAX_LEN_FRAME - (int)OFF_DAT_OPTS - 4 };
enum {
  // Bitfields in frame format octet
  HDR_FTYPE = 0xE0,
  HDR_RFU = 0x1C,
  HDR_MAJOR = 0x03
};
enum { HDR_FTYPE_DNFLAG = 0x20 }; // flags DN frame except for HDR_FTYPE_PROP
enum {
  // Values of frame type bit field
  HDR_FTYPE_JREQ = 0x00,
  HDR_FTYPE_JACC = 0x20,
  HDR_FTYPE_DAUP = 0x40, // data (unconfirmed) up
  HDR_FTYPE_DADN = 0x60, // data (unconfirmed) dn
  HDR_FTYPE_DCUP = 0x80, // data confirmed up
  HDR_FTYPE_DCDN = 0xA0, // data confirmed dn
  HDR_FTYPE_PROP = 0xE0
};
enum {
  HDR_MAJOR_V1 = 0x00,
};
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
  MCMD_DEVS_ANS = 0x06, // -  device status ans  : u1:battery 0,1-254,255=?,
                        // u1:7-6:RFU,5-0:margin(-32..31)
  MCMD_SNCH_ANS = 0x07, // -  set new channel    : u1: 7-2=RFU, 1/0:DR/freq ACK
  // Ack to new RX 1 timing.
  MCMD_RXTimingSetup_ANS = 0x08,
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
  MCMD_SNCH_REQ = 0x07,
  // Sets the timing of the of the reception slots  u1: [7-4]:RFU, [3-0]:del
  MCMD_RXTimingSetup_REQ = 0x08,
  //  set the maximum allowed dwell time
  MCMD_TxParamSetup_REQ = 0x09,
  // Class B
  MCMD_PING_SET = 0x11, // set ping freq      : u3: freq
  MCMD_BCNI_ANS =
      0x12, // next beacon start  : u2: delay(in TUNIT millis), u1:channel
};

enum {
  MCMD_BCNI_TUNIT = 30 // time unit of delay value in millis
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
enum { MCMD_PING_ANS_RFU = 0xFE, MCMD_PING_ANS_FQACK = 0x01 };

enum {
  MCMD_DEVS_EXT_POWER = 0x00,   // external power supply
  MCMD_DEVS_BATT_MIN = 0x01,    // min battery value
  MCMD_DEVS_BATT_MAX = 0xFE,    // max battery value
  MCMD_DEVS_BATT_NOINFO = 0xFF, // unknown battery level
};

// Bit fields byte#3 of MCMD_LADR_REQ payload
enum {
  MCMD_LADR_CHP_125ON =
      0x60, // special channel page enable, bits applied to 64..71
  MCMD_LADR_CHP_125OFF = 0x70, //  ditto
  MCMD_LADR_N3RFU_MASK = 0x80,
  MCMD_LADR_CHPAGE_MASK = 0xF0,
  MCMD_LADR_REPEAT_MASK = 0x0F,
  MCMD_LADR_REPEAT_1 = 0x01,
  MCMD_LADR_CHPAGE_1 = 0x10
};
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
enum { RSSI_OFF = 64, SNR_SCALEUP = 4 };

#endif // _lorabase_h_
