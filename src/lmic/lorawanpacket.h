#ifndef _lorawanpacket_h_
#define _lorawanpacket_h_
#pragma once

#include <stdint.h>

namespace lorawan {

namespace lengths {
constexpr uint8_t MHDR = 1;
constexpr uint8_t MIC = 4;
} // namespace lengths

namespace offsets {
constexpr uint8_t MHDR = 0;
}

namespace mac_payload {
namespace lengths {
constexpr uint8_t devAddr = 4;
constexpr uint8_t fctrl = 1;
constexpr uint8_t fcnt = 2;
} // namespace lengths

namespace offsets {
constexpr uint8_t devAddr = lorawan::offsets::MHDR + lorawan::lengths::MHDR;
constexpr uint8_t fctrl = devAddr + lengths::devAddr;
constexpr uint8_t fcnt = fctrl + lengths::fctrl;
constexpr uint8_t fopts = fcnt + lengths::fcnt;
} // namespace offsets
} // namespace mac_payload

// Join Request frame format
namespace join_request {
namespace lengths {
// cf § 6.2.4 Join-request message
constexpr uint8_t appEUI = 8;
constexpr uint8_t devEUI = 8;
constexpr uint8_t devNonce = 2;
constexpr uint8_t totalWithMic = 1 + appEUI + devEUI + devNonce + 4;
} // namespace lengths

namespace offset {
constexpr uint8_t MHDR = 0;
constexpr uint8_t appEUI = 1;
constexpr uint8_t devEUI = appEUI + lengths::appEUI;
constexpr uint8_t devNonce = devEUI + lengths::devEUI;
constexpr uint8_t MIC = devNonce + lengths::devNonce;
} // namespace offset

} // namespace join_request

namespace join_accept {
namespace lengths {
constexpr uint8_t appNonce = 3;
constexpr uint8_t netId = 3;
constexpr uint8_t devAddr = 4;
constexpr uint8_t dlSettings = 1;
constexpr uint8_t rxDelay = 1;
constexpr uint8_t cfList = 16;
constexpr uint8_t total = 1 + appNonce + netId + devAddr + dlSettings +
                          rxDelay + lorawan::lengths::MIC;
constexpr uint8_t totalWithOptional = total + cfList;

} // namespace lengths

namespace offset {
constexpr uint8_t MHDR = 0;
constexpr uint8_t appNonce = 1;
constexpr uint8_t netId = appNonce + lengths::appNonce;
constexpr uint8_t devAddr = netId + lengths::netId;
constexpr uint8_t dlSettings = devAddr + lengths::devAddr;
constexpr uint8_t rxDelay = dlSettings + lengths::dlSettings;
constexpr uint8_t cfList = rxDelay + lengths::rxDelay;
} // namespace offset
} // namespace join_accept

namespace mhdr {
constexpr uint8_t ftype_offset = 5;
constexpr uint8_t ftype_mask = 0x07 << ftype_offset;

// Values of frame type bit field
// join request
constexpr uint8_t ftype_join_req = 0x00 << ftype_offset;
// join accept
constexpr uint8_t ftype_join_acc = 0x01 << ftype_offset;
// data unconfirmed up
constexpr uint8_t ftype_data_up = 0x02 << ftype_offset;
// data unconfirmed dn
constexpr uint8_t ftype_data_down = 0x03 << ftype_offset;
// data confirmed up
constexpr uint8_t ftype_data_conf_up = 0x04 << ftype_offset;
// data confirmed dn
constexpr uint8_t ftype_data_conf_down = 0x05 << ftype_offset;
// Proprietary
constexpr uint8_t ftype_proprietary = 0x07 << ftype_offset;

constexpr uint8_t major_mask = 0x03;
constexpr uint8_t major_v1 = 0x00;



} // namespace mhdr

} // namespace lorawan

#endif