#ifndef _lorawanpacket_h_
#define _lorawanpacket_h_
#pragma once

#include <stdint.h>

namespace lorawan {

namespace lengths {
    const uint8_t MHDR = 1;
    const uint8_t MIC = 4;
}

namespace offsets {
    const uint8_t MHDR = 0;
}

namespace mac_payload {
namespace lengths {
    const uint8_t devAddr = 4;
    const uint8_t fctrl = 1;
    const uint8_t fcnt = 2;
}

namespace offsets {
    const uint8_t devAddr = lorawan::offsets::MHDR + lorawan::lengths::MHDR;
    const uint8_t fctrl = devAddr + lengths::devAddr;
    const uint8_t fcnt = fctrl + lengths::fctrl;
    const uint8_t fopts = fcnt + lengths::fcnt;
}
}

// Join Request frame format
namespace join_request {
namespace lengths {
// cf § 6.2.4 Join-request message
const uint8_t appEUI = 8;
const uint8_t devEUI = 8;
const uint8_t devNonce = 2;
const uint8_t totalWithMic = 1 + appEUI + devEUI + devNonce + 4;
} // namespace lengths

namespace offset {
const uint8_t MHDR = 0;
const uint8_t appEUI = 1;
const uint8_t devEUI = appEUI + lengths::appEUI;
const uint8_t devNonce = devEUI + lengths::devEUI;
const uint8_t MIC = devNonce + lengths::devNonce;
} // namespace offset

} // namespace join_request

namespace join_accept {
namespace lengths {
const uint8_t appNonce = 3;
const uint8_t netId = 3;
const uint8_t devAddr = 4;
const uint8_t dlSettings = 1;
const uint8_t rxDelay = 1;
const uint8_t cfList = 16;
const uint8_t total = 1 + appNonce + netId + devAddr + dlSettings + rxDelay + lorawan::lengths::MIC;
const uint8_t totalWithOptional = total + cfList;

} // namespace lengths

namespace offset {
const uint8_t MHDR = 0;
const uint8_t appNonce = 1;
const uint8_t netId = appNonce + lengths::appNonce;
const uint8_t devAddr = netId + lengths::netId;
const uint8_t dlSettings = devAddr + lengths::devAddr;
const uint8_t rxDelay = dlSettings + lengths::dlSettings;
const uint8_t cfList = rxDelay + lengths::rxDelay;
} // namespace offset
} // namespace join_accept

} // namespace lorawan

#endif