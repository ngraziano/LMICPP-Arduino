
#ifndef __aes_h__
#define __aes_h__
#include "../lmic/bufferpack.h"
#include "../lmic/config.h"
#include "../lmic/lorabase.h"
#include "aes_encrypt.h"

#include <stdint.h>

// ======================================================================
// AES support

constexpr uint8_t AES_BLCK_SIZE = 16;
using AesBlock = std::array<uint8_t, AES_BLCK_SIZE>;

class Aes {
private:
  AesKey AESDevKey;
  // network session key
  AesKey nwkSKey;
  // application session key
  AesKey appSKey;

  static AesBlock micB0(uint32_t devaddr, uint32_t seqno, PktDir dndir, uint8_t len);
  static void aes_cmac(const uint8_t *buf, uint8_t len, bool prepend_aux,
                       AesKey const &key, AesBlock &result);

public:
  /* Set device key
   * Key is copied.
   */
  void setDevKey(AesKey const &key);
  void setNetworkSessionKey(AesKey const &key);
  void setApplicationSessionKey(AesKey const &key);
  bool verifyMic(uint32_t devaddr, uint32_t seqno, PktDir dndir,
                 const uint8_t *pdu, uint8_t len) const;
  bool verifyMic0(uint8_t const *pdu, uint8_t len) const;
  void framePayloadEncryption(uint8_t port, uint32_t devaddr, uint32_t seqno,
                              PktDir dndir, uint8_t *payload,
                              uint8_t len) const;
  void encrypt(uint8_t *pdu, uint8_t len) const;
  void sessKeys(uint16_t devnonce, uint8_t const *artnonce);
  void appendMic(uint32_t devaddr, uint32_t seqno, PktDir dndir, uint8_t *pdu,
                 uint8_t len) const;
  void appendMic0(uint8_t *pdu, uint8_t len) const;
  void saveState(StoringAbtract &buffer) const;
  void loadState(RetrieveAbtract &store);
};

#endif // __aes_h__