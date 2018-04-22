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

/*
 * The original LMIC AES implementation integrates raw AES encryption
 * with CMAC and AES-CTR in a single piece of code. Most other AES
 * implementations (only) offer raw single block AES encryption, so this
 * file contains an implementation of CMAC and AES-CTR, and offers the
 * same API through the os_aes() function as the original AES
 * implementation. This file assumes that there is an encryption
 * function available with this signature:
 *
 *      extern "C" void lmic_aes_encrypt(uint8_t *data, uint8_t *key);
 *
 *  That takes a single 16-byte buffer and encrypts it wit the given
 *  16-byte key.
 */

#include "aes.h"
#include "../lmic/oslmic.h"
#include "../lmic/lorabase.h"
#include "../lmic/bufferpack.h"
#include <algorithm>

uint8_t AESkey[16];
uint8_t AESaux[16];


void Aes::micB0 (uint32_t devaddr, uint32_t seqno, int dndir, int len) {
    std::fill(AESaux, AESaux+16, 0);
    AESaux[0]  = 0x49;
    AESaux[5]  = dndir?1:0;
    AESaux[15] = len;
    wlsbf4(AESaux+ 6,devaddr);
    wlsbf4(AESaux+10,seqno);
}


int Aes::verifyMic (const uint8_t* key, uint32_t devaddr, uint32_t seqno, int dndir, uint8_t* pdu, int len) {
    micB0(devaddr, seqno, dndir, len);
    std::copy(key, key+16, AESkey);
    return os_aes(AES_MIC, pdu, len) == rmsbf4(pdu+len);
}


void Aes::appendMic (const uint8_t* key, uint32_t devaddr, uint32_t seqno, int dndir, uint8_t* pdu, int len) {
    micB0(devaddr, seqno, dndir, len);
    std::copy(key, key+16, AESkey);    
    // MSB because of internal structure of AES
    wmsbf4(pdu+len, os_aes(AES_MIC, pdu, len));
}


void Aes::appendMic0 (uint8_t* pdu, int len) {
    os_getDevKey(AESkey);
    wmsbf4(pdu+len, os_aes(AES_MIC|AES_MICNOAUX, pdu, len));  // MSB because of internal structure of AES
}


int Aes::verifyMic0 (uint8_t* pdu, int len) {
    os_getDevKey(AESkey);
    return os_aes(AES_MIC|AES_MICNOAUX, pdu, len) == rmsbf4(pdu+len);
}


void Aes::encrypt (uint8_t* pdu, int len) {
    os_getDevKey(AESkey);
    os_aes(AES_ENC, pdu, len);
}


void Aes::cipher (const uint8_t* key, uint32_t devaddr, uint32_t seqno, int dndir, uint8_t* payload, int len) {
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


void Aes::sessKeys (uint16_t devnonce, const uint8_t* artnonce, uint8_t* nwkkey, uint8_t* artkey) {
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


// Shift the given buffer left one bit
static void shift_left(uint8_t* buf, uint8_t len) {
    while (len--) {
        uint8_t next = len ? buf[1] : 0;

        uint8_t val = (*buf << 1);
        if (next & 0x80)
            val |= 1;
        *buf++ = val;
    }
}

// Apply RFC4493 CMAC, using AESKEY as the key. If prepend_aux is true,
// AESAUX is prepended to the message. AESAUX is used as working memory
// in any case. The CMAC result is returned in AESAUX as well.
void Aes::os_aes_cmac(const uint8_t* buf, uint16_t len, uint8_t prepend_aux) {
    if (prepend_aux)
        lmic_aes_encrypt(AESaux, AESkey);
    else
        std::fill(AESaux, AESaux+16, 0);

    while (len > 0) {
        uint8_t need_padding = 0;
        for (uint8_t i = 0; i < 16; ++i, ++buf, --len) {
            if (len == 0) {
                // The message is padded with 0x80 and then zeroes.
                // Since zeroes are no-op for xor, we can just skip them
                // and leave AESAUX unchanged for them.
                AESaux[i] ^= 0x80;
                need_padding = 1;
                break;
            }
            AESaux[i] ^= *buf;
        }

        if (len == 0) {
            // Final block, xor with K1 or K2. K1 and K2 are calculated
            // by encrypting the all-zeroes block and then applying some
            // shifts and xor on that.
            uint8_t final_key[16];
            std::fill(final_key,final_key+16,0);            
            lmic_aes_encrypt(final_key, AESkey);

            // Calculate K1
            uint8_t msb = final_key[0] & 0x80;
            shift_left(final_key, sizeof(final_key));
            if (msb)
                final_key[sizeof(final_key)-1] ^= 0x87;

            // If the final block was not complete, calculate K2 from K1
            if (need_padding) {
                msb = final_key[0] & 0x80;
                shift_left(final_key, sizeof(final_key));
                if (msb)
                    final_key[sizeof(final_key)-1] ^= 0x87;
            }

            // Xor with K1 or K2
            for (uint8_t i = 0; i < sizeof(final_key); ++i)
                AESaux[i] ^= final_key[i];
        }

        lmic_aes_encrypt(AESaux, AESkey);
    }
}

// Run AES-CTR using the key in AESKEY and using AESAUX as the
// counter block. The last byte of the counter block will be incremented
// for every block. The given buffer will be encrypted in place.
void Aes::os_aes_ctr (uint8_t* buf, uint16_t len) {
    uint8_t ctr[16];
    while (len) {
        // Encrypt the counter block with the selected key
        memcpy(ctr, AESaux, sizeof(ctr));
        lmic_aes_encrypt(ctr, AESkey);

        // Xor the payload with the resulting ciphertext
        for (uint8_t i = 0; i < 16 && len > 0; i++, len--, buf++)
            *buf ^= ctr[i];

        // Increment the block index byte
        AESaux[15]++;
    }
}

uint32_t Aes::os_aes (uint8_t mode, uint8_t* buf, uint16_t len) {
    switch (mode & ~AES_MICNOAUX) {
        case AES_MIC:
            os_aes_cmac(buf, len, /* prepend_aux */ !(mode & AES_MICNOAUX));
            return rmsbf4(AESaux);

        case AES_ENC:
            // TODO: Check / handle when len is not a multiple of 16
            for (uint8_t i = 0; i < len; i += 16)
                lmic_aes_encrypt(buf+i, AESkey);
            break;

        case AES_CTR:
            os_aes_ctr(buf, len);
            break;
    }
    return 0;
}