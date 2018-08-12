/*******************************************************************************
 * Copyright (c) 2016 Matthijs Kooijman
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

#include "aes.h"
#include "../lmic/bufferpack.h"
#include "../lmic/lorabase.h"
#include <algorithm>

void Aes::setDevKey(uint8_t key[16]) { std::copy(key, key + 16, AESDevKey); }

// Get B0 value in buf
void Aes::micB0(uint32_t devaddr, uint32_t seqno, uint8_t dndir, uint8_t len,
                uint8_t buf[16]) {
  buf[0] = 0x49;
  buf[1] = 0;
  buf[2] = 0;
  buf[3] = 0;
  buf[4] = 0;
  buf[5] = dndir ? 1 : 0;
  wlsbf4(buf + 6, devaddr);
  wlsbf4(buf + 10, seqno);
  buf[14] = 0;
  buf[15] = len;

}

// Verify MIC
bool Aes::verifyMic(const uint8_t *key, uint32_t devaddr, uint32_t seqno,
                    uint8_t dndir, uint8_t *pdu, uint8_t len) {
  uint8_t buf[16];
  micB0(devaddr, seqno, dndir, len, buf);
  os_aes_cmac(pdu, len, 1, key, buf);
  return std::equal(buf, buf + MIC_LEN, pdu+len);
}

// Append MIC
void Aes::appendMic(const uint8_t *key, uint32_t devaddr, uint32_t seqno,
                    uint8_t dndir, uint8_t *pdu, uint8_t len) {
  uint8_t buf[16];
  micB0(devaddr, seqno, dndir, len, buf);
  os_aes_cmac(pdu, len, true, key, buf);
  // Copy MIC at the end
  std::copy(buf, buf + MIC_LEN, pdu + len);
}

// Append join MIC
void Aes::appendMic0(uint8_t *pdu, uint8_t len) {
  uint8_t buf[16] = {0};
  os_aes_cmac(pdu, len, false, AESDevKey, buf);
  // Copy MIC0 at the end
  std::copy(buf, buf + MIC_LEN, pdu + len);
}

// Verify join MIC
bool Aes::verifyMic0(uint8_t *pdu, uint8_t len) {
  uint8_t buf[16] = {0};
  os_aes_cmac(pdu, len, 0, AESDevKey, buf);
  return std::equal(buf, buf + MIC_LEN, pdu+len);
}

void Aes::encrypt(uint8_t *pdu, uint8_t len) {
  // TODO: Check / handle when len is not a multiple of 16
  for (uint8_t i = 0; i < len; i += 16)
    lmic_aes_encrypt(pdu + i, AESDevKey);
}

// cipher a buffer with corresponding data
void Aes::cipher(const uint8_t *key, uint32_t devaddr, uint32_t seqno,
                 uint8_t dndir, uint8_t *payload, uint8_t len) {

  uint8_t buf[16];
  buf[0] = 1; // mode=cipher
  buf[1] = 0;
  buf[2] = 0;
  buf[3] = 0;
  buf[4] = 0;
  buf[5] = dndir ? 1 : 0;
  wlsbf4(buf + 6, devaddr);
  wlsbf4(buf + 10, seqno);
  buf[14] = 0;
  buf[15] = 1; // block counter=1


  uint8_t ctr[16];
  while (len) {
    // Encrypt the counter block with the selected key
    memcpy(ctr, buf, sizeof(ctr));
    lmic_aes_encrypt(ctr, key);

    // Xor the payload with the resulting ciphertext
    for (uint8_t i = 0; i < 16 && len > 0; i++, len--, payload++)
      *payload ^= ctr[i];

    // Increment the block index byte
    buf[15]++;
  }

}

// Extract session keys
void Aes::sessKeys(uint16_t devnonce, const uint8_t *artnonce, uint8_t nwkkey[16],
                   uint8_t artkey[16]) {
  std::fill(nwkkey, nwkkey + 16, 0);
  nwkkey[0] = 0x01;
  std::copy(artnonce, artnonce + LEN_ARTNONCE + LEN_NETID, nwkkey + 1);
  wlsbf2(nwkkey + 1 + LEN_ARTNONCE + LEN_NETID, devnonce);
  std::copy(nwkkey, nwkkey + 16, artkey);
  artkey[0] = 0x02;

  lmic_aes_encrypt(nwkkey, AESDevKey);
  lmic_aes_encrypt(artkey, AESDevKey);
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
void Aes::os_aes_cmac(const uint8_t *buf, uint8_t len, bool prepend_aux,
                      const uint8_t key[16], uint8_t result[16]) {
  if (prepend_aux)
    lmic_aes_encrypt(result, key);

  while (len > 0) {
    uint8_t need_padding = 0;
    for (uint8_t i = 0; i < 16; ++i, ++buf, --len) {
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
      uint8_t final_key[16];
      std::fill(final_key, final_key + 16, 0);
      lmic_aes_encrypt(final_key, key);

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

    lmic_aes_encrypt(result, key);
  }
}

