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

enum _cr_t { CR_4_5 = 0, CR_4_6, CR_4_7, CR_4_8 };
enum _sf_t { FSK = 0, SF7, SF8, SF9, SF10, SF11, SF12, SFrfu };
enum _bw_t { BW125 = 0, BW250, BW500, BWrfu };
typedef uint8_t cr_t;
typedef uint8_t sf_t;
typedef uint8_t bw_t;
typedef uint8_t dr_t;
// Radio parameter set (encodes SF/BW/CR/IH/NOCRC)
// typedef uint16_t rps_t;
union rps_t {
  uint16_t rawValue;
  struct {
    sf_t sf : 3;
    bw_t bw : 2;
    cr_t cr : 2;
    bool nocrc : 1;
    uint8_t ih : 8;
  };
};

enum { ILLEGAL_RPS = 0xFF };
enum { DR_PAGE_EU868 = 0x00 };
enum { DR_PAGE_US915 = 0x10 };

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
enum { BCN_INTV_exp = 7 };
enum { BCN_INTV_sec = 1 << BCN_INTV_exp };
enum { BCN_INTV_ms = BCN_INTV_sec * 1000L };
enum { BCN_INTV_us = BCN_INTV_ms * 1000L };
enum { BCN_RESERVE_ms = 2120 }; // space reserved for beacon and NWK management
enum {
  BCN_GUARD_ms = 3000
}; // end of beacon period to prevent interference with beacon
enum { BCN_SLOT_SPAN_ms = 30 }; // 2^12 reception slots a this span
enum { BCN_WINDOW_ms = BCN_INTV_ms - (int)BCN_GUARD_ms - (int)BCN_RESERVE_ms };
enum { BCN_RESERVE_us = 2120000 };
enum { BCN_GUARD_us = 3000000 };
enum { BCN_SLOT_SPAN_us = 30000 };

enum {
  // Join Request frame format
  OFF_JR_HDR = 0,
  OFF_JR_ARTEUI = 1,
  OFF_JR_DEVEUI = 9,
  OFF_JR_DEVNONCE = 17,
  OFF_JR_MIC = 19,
  LEN_JR = 23
};
enum {
  // Join Accept frame format
  OFF_JA_HDR = 0,
  OFF_JA_ARTNONCE = 1,
  OFF_JA_NETID = 4,
  OFF_JA_DEVADDR = 7,
  OFF_JA_RFU = 11,
  OFF_JA_DLSET = 11,
  OFF_JA_RXDLY = 12,
  OFF_CFLIST = 13,
  LEN_JA = 17,
  LEN_JAEXT = 17 + 16
};
enum {
  // Data frame format
  OFF_DAT_HDR = 0,
  OFF_DAT_ADDR = 1,
  OFF_DAT_FCT = 5,
  OFF_DAT_SEQNO = 6,
  OFF_DAT_OPTS = 8,
};
enum { MIC_LEN = 4 };
enum class PktDir: uint8_t {
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
  HDR_FTYPE_DAUP = 0x40,   // data (unconfirmed) up
  HDR_FTYPE_DADN = 0x60,   // data (unconfirmed) dn
  HDR_FTYPE_DCUP = 0x80,   // data confirmed up
  HDR_FTYPE_DCDN = 0xA0,   // data confirmed dn
  HDR_FTYPE_REJOIN = 0xC0, // rejoin for roaming
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
  FCT_MORE = 0x10, // also in DN direction: Class B indicator
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

#define MAKERPS(sf, bw, cr, ih, nocrc)                                         \
  (((sf) | ((bw) << 3) | ((cr) << 5) | ((nocrc) ? (1 << 7) : 0) |              \
    ((ih & 0xFF) << 8)))
// Two frames with params r1/r2 would interfere on air: same SFx + BWx
inline bool sameSfBw(rps_t r1, rps_t r2) {
  return (r1.sf == r2.sf) && (r1.bw == r2.bw);
}




//
// BEG: Keep in sync with lorabase.hpp
// ================================================================================

// Calculate airtime
OsDeltaTime calcAirTime(uint8_t plen);
// Sensitivity at given SF/BW
int16_t getSensitivity(rps_t rps);

#endif // _lorabase_h_
