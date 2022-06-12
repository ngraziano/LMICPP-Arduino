#include "aesdecrypt.h"
#include "aes/aes_encrypt.h"

void aes128_decrypt(std::array<uint8_t, 16> const &key, uint8_t *data,
                    uint8_t len) {
  for (uint8_t i = 0; i < len; i += 16)
    aes_tiny_128_decrypt(data + i, key);
}
