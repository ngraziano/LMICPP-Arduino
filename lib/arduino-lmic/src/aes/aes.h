
#ifndef __aes_h__
#define __aes_h__
#include "../lmic/config.h"
#include <stdint.h>
#include "../lmic/lorabase.h"

// ======================================================================
// AES support

void lmic_aes_encrypt(uint8_t *data, const uint8_t *key);

#define AES_BLCK_SIZE 16

class Aes {
private:
  uint8_t AESDevKey[16];
  // network session key
  uint8_t nwkSKey[16];
  // application session key
  uint8_t appSKey[16];

  static void micB0(uint32_t devaddr, uint32_t seqno, PktDir dndir,
                    uint8_t len, uint8_t buf[16]);
  static void aes_cmac(const uint8_t *buf, uint8_t len, bool prepend_aux,
                       const uint8_t key[16], uint8_t result[16]);

public:
  /* Set device key
   * Key is copied.
   */
  void setDevKey(uint8_t key[16]);
  void setNetworkSessionKey(uint8_t key[16]);
  void setApplicationSessionKey(uint8_t key[16]);
  bool verifyMic(uint32_t devaddr, uint32_t seqno, PktDir dndir, const uint8_t *pdu,
                 uint8_t len) const;
  bool verifyMic0(const uint8_t *pdu, uint8_t len) const;
  void framePayloadEncryption(uint8_t port, uint32_t devaddr, uint32_t seqno,
                              PktDir dndir, uint8_t *payload,
                              uint8_t len) const;
  void encrypt(uint8_t *pdu, uint8_t len) const;
  void sessKeys(uint16_t devnonce, const uint8_t *artnonce);
  void appendMic(uint32_t devaddr, uint32_t seqno, PktDir dndir, uint8_t *pdu,
                 uint8_t len) const;
  void appendMic0(uint8_t *pdu, uint8_t len) const;
};

#endif // __aes_h__