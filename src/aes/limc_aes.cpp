/*******************************************************************************
 * Copyright (c) 2016 Matthijs Kooijman
 *               2019 Nicolas Graziano
 *
 * LICENSE
 *
 * Permission is hereby granted, free of charge, to anyone
 * obtaining a copy of this document and accompanying files,
 * to do whatever they want with them without any restriction,
 * including, but not limited to, copying, modification and
 * redistribution.
 *
 * NO WARRANTY OF ANY KIND IS PROVIDED.
 *******************************************************************************/

#include "../lmic/lorabase.h"
#include "../lmic/lorawanpacket.h"
#include "lmic_aes.h"
#include <algorithm>

using namespace lorawan;

void Aes::setDevKey(AesKey const &key) { AESDevKey = key; }
void Aes::setNetworkSessionKey(AesKey const &key) { nwkSKey = key; }
void Aes::setApplicationSessionKey(AesKey const &key) { appSKey = key; }

// Get B0 value in buf
void Aes::micB0(const uint32_t devaddr, const uint32_t seqno,
                const PktDir dndir, const uint8_t len,
                uint8_t buf[AES_BLCK_SIZE]) {
  buf[0] = 0x49;
  buf[1] = 0;
  buf[2] = 0;
  buf[3] = 0;
  buf[4] = 0;
  buf[5] = static_cast<uint8_t>(dndir);
  wlsbf4(buf + 6, devaddr);
  wlsbf4(buf + 10, seqno);
  buf[14] = 0;
  buf[15] = len;
}

/**
 * Verify MIC
 * len : total length (MIC included)
 */
bool Aes::verifyMic(const uint32_t devaddr, const uint32_t seqno,
                    const PktDir dndir, const uint8_t *const pdu,
                    const uint8_t len) const {
  uint8_t buf[AES_BLCK_SIZE];
  const uint8_t lenWithoutMic = len - lengths::MIC;
  micB0(devaddr, seqno, dndir, lenWithoutMic, buf);
  aes_cmac(pdu, lenWithoutMic, true, nwkSKey, buf);
  return std::equal(buf, buf + lengths::MIC, pdu + lenWithoutMic);
}

/**
 * Append MIC
 * len : total length (MIC included)
 */
void Aes::appendMic(const uint32_t devaddr, const uint32_t seqno,
                    const PktDir dndir, uint8_t *const pdu,
                    const uint8_t len) const {
  uint8_t buf[AES_BLCK_SIZE];
  const uint8_t lenWithoutMic = len - lengths::MIC;
  micB0(devaddr, seqno, dndir, lenWithoutMic, buf);
  aes_cmac(pdu, lenWithoutMic, true, nwkSKey, buf);
  // Copy MIC at the end
  std::copy(buf, buf + lengths::MIC, pdu + lenWithoutMic);
}

/**
 * Append join MIC
 * len : total length (MIC included)
 */
void Aes::appendMic0(uint8_t *const pdu, const uint8_t len) const {
  uint8_t buf[AES_BLCK_SIZE] = {0};
  const uint8_t lenWithoutMic = len - lengths::MIC;
  aes_cmac(pdu, lenWithoutMic, false, AESDevKey, buf);
  // Copy MIC0 at the end
  std::copy(buf, buf + lengths::MIC, pdu + lenWithoutMic);
}

/**
 * Verify join MIC
 * len : total length (MIC included)
 */
bool Aes::verifyMic0(const uint8_t *const pdu, const uint8_t len) const {
  uint8_t buf[AES_BLCK_SIZE] = {0};
  const uint8_t lenWithoutMic = len - lengths::MIC;
  aes_cmac(pdu, lenWithoutMic, 0, AESDevKey, buf);
  return std::equal(buf, buf + lengths::MIC, pdu + lenWithoutMic);
}

void Aes::encrypt(uint8_t *const pdu, const uint8_t len) const {
  // TODO: Check / handle when len is not a multiple of AES_BLCK_SIZE
  for (uint8_t i = 0; i < len; i += AES_BLCK_SIZE)
    aes_128_encrypt(pdu + i, AESDevKey);
}

/**
 *  Encrypt data frame payload.
 */
void Aes::framePayloadEncryption(const uint8_t port, const uint32_t devaddr,
                                 const uint32_t seqno, const PktDir dndir,
                                 uint8_t *payload, uint8_t len) const {
  const auto &key = port == 0 ? nwkSKey : appSKey;
  // Generate
  uint8_t blockAi[AES_BLCK_SIZE];
  blockAi[0] = 1; // mode=cipher
  blockAi[1] = 0;
  blockAi[2] = 0;
  blockAi[3] = 0;
  blockAi[4] = 0;
  blockAi[5] = static_cast<uint8_t>(dndir); // direction (0=up 1=down)
  wlsbf4(blockAi + 6, devaddr);
  wlsbf4(blockAi + 10, seqno);
  blockAi[14] = 0;
  blockAi[15] = 0; // block counter

  while (len) {
    uint8_t blockSi[AES_BLCK_SIZE];

    // Increment the block index byte
    blockAi[15]++;
    // Encrypt the counter block with the selected key
    std::copy(blockAi, blockAi + AES_BLCK_SIZE, blockSi);
    aes_128_encrypt(blockSi, key);

    // Xor the payload with the resulting ciphertext
    for (uint8_t i = 0; i < AES_BLCK_SIZE && len > 0; i++, len--, payload++)
      *payload ^= blockSi[i];
  }
}

// Extract session keys
void Aes::sessKeys(const uint16_t devnonce, const uint8_t *const artnonce) {
  nwkSKey.data[0] = 0x01;
  std::copy(artnonce,
            artnonce + join_accept::lengths::appNonce +
                join_accept::lengths::netId,
            nwkSKey.data + 1);
  wlsbf2(nwkSKey.data + 1 + join_accept::lengths::appNonce +
             join_accept::lengths::netId,
         devnonce);
  // add pading
  std::fill(nwkSKey.data + 1 + join_accept::lengths::appNonce +
                join_accept::lengths::netId + join_request::lengths::devNonce,
            nwkSKey.data + AES_BLCK_SIZE, 0);

  appSKey = nwkSKey;
  appSKey.data[0] = 0x02;

  aes_128_encrypt(nwkSKey.data, AESDevKey);
  aes_128_encrypt(appSKey.data, AESDevKey);
}

// Shift the given buffer left one bit
static void shift_left(uint8_t *buf, uint8_t len) {
  while (len--) {
    uint8_t next = len ? buf[1] : 0;

    uint8_t val = (*buf << 1);
    if (next & 0x80)
      val |= 1;
    *buf++ = val;
  }
}

// Apply RFC4493 CMAC. If prepend_aux is true,
// result is prepended to the message. result is used as working memory,
// it can be set to "B0" for MIC. The CMAC result is returned in result
// as well.
void Aes::aes_cmac(const uint8_t *buf, uint8_t len, const bool prepend_aux,
                   AesKey const &key, uint8_t result[AES_BLCK_SIZE]) {
  if (prepend_aux)
    aes_128_encrypt(result, key);

  while (len > 0) {
    uint8_t need_padding = 0;
    for (uint8_t i = 0; i < AES_BLCK_SIZE; ++i, ++buf, --len) {
      if (len == 0) {
        // The message is padded with 0x80 and then zeroes.
        // Since zeroes are no-op for xor, we can just skip them
        // and leave AESAUX unchanged for them.
        result[i] ^= 0x80;
        need_padding = 1;
        break;
      }
      result[i] ^= *buf;
    }

    if (len == 0) {
      // Final block, xor with K1 or K2. K1 and K2 are calculated
      // by encrypting the all-zeroes block and then applying some
      // shifts and xor on that.
      uint8_t final_key[AES_BLCK_SIZE];
      std::fill(final_key, final_key + AES_BLCK_SIZE, 0);
      aes_128_encrypt(final_key, key);

      // Calculate K1
      uint8_t msb = final_key[0] & 0x80;
      shift_left(final_key, sizeof(final_key));
      if (msb)
        final_key[sizeof(final_key) - 1] ^= 0x87;

      // If the final block was not complete, calculate K2 from K1
      if (need_padding) {
        msb = final_key[0] & 0x80;
        shift_left(final_key, sizeof(final_key));
        if (msb)
          final_key[sizeof(final_key) - 1] ^= 0x87;
      }

      // Xor with K1 or K2
      for (uint8_t i = 0; i < sizeof(final_key); ++i)
        result[i] ^= final_key[i];
    }

    aes_128_encrypt(result, key);
  }
}

void Aes::saveState(StoringAbtract &store) const {
  // Do not save devkey (should be fix value)
  // save 2 keys
  store.write(nwkSKey);
  store.write(appSKey);
}

void Aes::loadState(RetrieveAbtract& store) {
  // Do not load devkey (should be fix valuse)
  // save 2 keys
  store.read(nwkSKey);
  store.read(appSKey);
}
