
#ifndef __aes_h__
#define __aes_h__
#include "../lmic/config.h"
#include <stdint.h>

// ======================================================================
// AES support

void lmic_aes_encrypt(uint8_t *data, const uint8_t *key);

#define AES_BLCK_SIZE 16

class Aes {
private:
  uint8_t AESDevKey[16];

  static void micB0(uint32_t devaddr, uint32_t seqno, uint8_t dndir, uint8_t len,
                    uint8_t buf[16]);
  static void aes_cmac(const uint8_t *buf, uint8_t len, bool prepend_aux,
                          const uint8_t key[16], uint8_t result[16]);

public:
  /* Set device key
   * Key is copied.
   */
  void setDevKey(uint8_t key[16]);
  bool verifyMic(const uint8_t *key, uint32_t devaddr, uint32_t seqno,
                 uint8_t dndir, uint8_t *pdu, uint8_t len);
  bool verifyMic0(uint8_t *pdu, uint8_t len);
  void framePayloadEncryption(const uint8_t *key, uint32_t devaddr, uint32_t seqno, uint8_t dndir,
              uint8_t *payload, uint8_t len);
  void encrypt(uint8_t *pdu, uint8_t len);
  void sessKeys(uint16_t devnonce, const uint8_t *artnonce, uint8_t nwkkey[16],
                uint8_t artkey[16]);
  void appendMic(const uint8_t *key, uint32_t devaddr, uint32_t seqno,
                 uint8_t dndir, uint8_t *pdu, uint8_t len);
  void appendMic0(uint8_t *pdu, uint8_t len);
};

#endif // __aes_h__