#include "aes.h"
#include "../lmic/oslmic.h"
#include "../lmic/lorabase.h"
#include "../lmic/bufferpack.h"
#include <algorithm>


// ================================================================================
// BEG AES

static void micB0 (uint32_t devaddr, uint32_t seqno, int dndir, int len) {
    std::fill(AESaux, AESaux+16, 0);
    AESaux[0]  = 0x49;
    AESaux[5]  = dndir?1:0;
    AESaux[15] = len;
    wlsbf4(AESaux+ 6,devaddr);
    wlsbf4(AESaux+10,seqno);
}


int aes_verifyMic (const uint8_t* key, uint32_t devaddr, uint32_t seqno, int dndir, uint8_t* pdu, int len) {
    micB0(devaddr, seqno, dndir, len);
    std::copy(key, key+16, AESkey);
    return os_aes(AES_MIC, pdu, len) == rmsbf4(pdu+len);
}


void aes_appendMic (const uint8_t* key, uint32_t devaddr, uint32_t seqno, int dndir, uint8_t* pdu, int len) {
    micB0(devaddr, seqno, dndir, len);
    std::copy(key, key+16, AESkey);    
    // MSB because of internal structure of AES
    wmsbf4(pdu+len, os_aes(AES_MIC, pdu, len));
}


void aes_appendMic0 (uint8_t* pdu, int len) {
    os_getDevKey(AESkey);
    wmsbf4(pdu+len, os_aes(AES_MIC|AES_MICNOAUX, pdu, len));  // MSB because of internal structure of AES
}


int aes_verifyMic0 (uint8_t* pdu, int len) {
    os_getDevKey(AESkey);
    return os_aes(AES_MIC|AES_MICNOAUX, pdu, len) == rmsbf4(pdu+len);
}


void aes_encrypt (uint8_t* pdu, int len) {
    os_getDevKey(AESkey);
    os_aes(AES_ENC, pdu, len);
}


void aes_cipher (const uint8_t* key, uint32_t devaddr, uint32_t seqno, int dndir, uint8_t* payload, int len) {
    if( len <= 0 )
        return;
    std::fill(AESaux, AESaux+16, 0);
    AESaux[0] = AESaux[15] = 1; // mode=cipher / dir=down / block counter=1
    AESaux[5] = dndir?1:0;
    wlsbf4(AESaux+ 6,devaddr);
    wlsbf4(AESaux+10,seqno);
    std::copy(key, key+16, AESkey);    
    os_aes(AES_CTR, payload, len);
}


void aes_sessKeys (uint16_t devnonce, const uint8_t* artnonce, uint8_t* nwkkey, uint8_t* artkey) {
    std::fill(nwkkey,nwkkey+16,0);
    nwkkey[0] = 0x01;
    std::copy(artnonce, artnonce+LEN_ARTNONCE+LEN_NETID,nwkkey+1);
    wlsbf2(nwkkey+1+LEN_ARTNONCE+LEN_NETID, devnonce);
    std::copy(nwkkey, nwkkey+16, artkey);
    artkey[0] = 0x02;

    os_getDevKey(AESkey);
    os_aes(AES_ENC, nwkkey, 16);
    os_getDevKey(AESkey);
    os_aes(AES_ENC, artkey, 16);
}

// END AES
// ================================================================================
