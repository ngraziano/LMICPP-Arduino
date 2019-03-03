
#ifndef __aes_h__
#define __aes_h__
#include "../lmic/config.h"
#include "../lmic/lorabase.h"
#include"aes_encrypt.h"
#include <stdint.h>

// ======================================================================
// AES support

// void lmic_aes_encrypt(uint8_t *data, const uint8_t *key);

const uint8_t AES_BLCK_SIZE = 16;

class Aes {
private:
  AesKey AESDevKey;
  // network session key
  AesKey nwkSKey;
  // application session key
  AesKey appSKey;

  static void micB0(uint32_t devaddr, uint32_t seqno, PktDir dndir, uint8_t len,
                    uint8_t buf[AES_BLCK_SIZE]);
  static void aes_cmac(const uint8_t *buf, uint8_t len, bool prepend_aux,
                       AesKey const &key, uint8_t result[AES_BLCK_SIZE]);

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
  size_t saveState(uint8_t* buffer) const;
  size_t loadState(uint8_t const* buffer);

};

#endif // __aes_h__