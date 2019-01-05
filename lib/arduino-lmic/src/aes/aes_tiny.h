
#ifndef __aes_tiny_h__
#define __aes_tiny_h__

#include <stdint.h>
struct AesKey {
  static const uint8_t key_size = 16;
  uint8_t data[key_size];
  AesKey() = default;
};

void aes_tiny_128_encrypt(uint8_t * buffer,AesKey const &key);

#endif