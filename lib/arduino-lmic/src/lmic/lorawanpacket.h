#ifndef _lorawanpacket_h_
#define _lorawanpacket_h_
#pragma once

#include <stdint.h>

namespace lorawan {
// Join Request frame format
namespace join_request {
namespace lengths {
// cf § 6.2.4 Join-request message
const uint8_t appEUI = 8;
const uint8_t devEUI = 8;
const uint8_t devNonce = 2;
const uint8_t total = 1 + appEUI + devEUI + devNonce + 4;
} // namespace lengths

namespace offset {
const uint8_t MHDR = 0;
const uint8_t appEUI = 1;
const uint8_t devEUI = appEUI + lengths::appEUI;
const uint8_t devNonce = devEUI + lengths::devEUI;
const uint8_t MIC = devNonce + lengths::devNonce;
} // namespace offset

} // namespace join_request

} // namespace lorawan

#endif