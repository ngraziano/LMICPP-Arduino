
#ifdef ARDUINO_ARCH_ESP32
#include "aes_encrypt.h"
#include "mbedtls/aes.h"


void aes_esp_128_encrypt(uint8_t *buffer, AesKey const &key) {
    mbedtls_aes_context keyCtx;
    mbedtls_aes_init(&keyCtx);
    mbedtls_aes_setkey_enc(&keyCtx, key.data, 128);
    mbedtls_aes_crypt_ecb(&keyCtx, ESP_AES_ENCRYPT, buffer, buffer);
    mbedtls_aes_free(&keyCtx);
}

#endif