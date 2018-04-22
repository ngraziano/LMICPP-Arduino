
#ifndef __aes_h__
#define __aes_h__
#include "../lmic/config.h"
#include <stdint.h>

#ifndef os_getDevKey
void os_getDevKey (uint8_t* buf);
#endif

// ======================================================================
// AES support
// !!Keep in sync with lorabase.hpp!!

#ifndef AES_ENC  // if AES_ENC is defined as macro all other values must be too
#define AES_ENC       0x00
#define AES_DEC       0x01
#define AES_MIC       0x02
#define AES_CTR       0x04
#define AES_MICNOAUX  0x08
#endif


// This should be defined elsewhere
void lmic_aes_encrypt(uint8_t *data, uint8_t *key);


extern uint32_t AESAUX[];
extern uint32_t AESKEY[];
#define AESkey ((uint8_t*)AESKEY)
#define AESaux ((uint8_t*)AESAUX)


int aes_verifyMic (const uint8_t* key, uint32_t devaddr, uint32_t seqno, int dndir, uint8_t* pdu, int len);
int aes_verifyMic0 (uint8_t* pdu, int len);
void aes_cipher (const uint8_t* key, uint32_t devaddr, uint32_t seqno, int dndir, uint8_t* payload, int len);
void aes_encrypt (uint8_t* pdu, int len);
void aes_sessKeys (uint16_t devnonce, const uint8_t* artnonce, uint8_t* nwkkey, uint8_t* artkey);
void aes_appendMic (const uint8_t* key, uint32_t devaddr, uint32_t seqno, int dndir, uint8_t* pdu, int len);
void aes_appendMic0 (uint8_t* pdu, int len);


uint32_t os_aes (uint8_t mode, uint8_t* buf, uint16_t len);

#endif // __aes_h__