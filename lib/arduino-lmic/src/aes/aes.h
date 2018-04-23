
#ifndef __aes_h__
#define __aes_h__
#include "../lmic/config.h"
#include <stdint.h>


// ======================================================================
// AES support


void lmic_aes_encrypt(uint8_t *data, const uint8_t *key);

class Aes {
    private:
    uint8_t AESDevKey[16];



    static void micB0 (uint32_t devaddr, uint32_t seqno, int dndir, int len, uint8_t buf[16]);

    static void os_aes_ctr (uint8_t* buf, uint16_t len, const uint8_t key[16], uint8_t result[16]);
    static void os_aes_cmac(const uint8_t* buf, uint16_t len, bool prepend_aux, const uint8_t key[16], uint8_t result[16]);

    public:
    /* Set device key
    * Key is copied.
    */
    void setDevKey(uint8_t key[16]);
    int verifyMic (const uint8_t* key, uint32_t devaddr, uint32_t seqno, int dndir, uint8_t* pdu, int len);
    int verifyMic0 (uint8_t* pdu, int len);
    void cipher (const uint8_t* key, uint32_t devaddr, uint32_t seqno, int dndir, uint8_t* payload, int len);
    void encrypt (uint8_t* pdu, int len);
    void sessKeys (uint16_t devnonce, const uint8_t* artnonce, uint8_t* nwkkey, uint8_t* artkey);
    void appendMic (const uint8_t* key, uint32_t devaddr, uint32_t seqno, int dndir, uint8_t* pdu, int len);
    void appendMic0 (uint8_t* pdu, int len);
};

#endif // __aes_h__