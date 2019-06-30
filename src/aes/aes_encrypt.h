
#ifndef __aes_tiny_h__
#define __aes_tiny_h__

#include <stdint.h>
struct AesKey {
  static const uint8_t key_size = 16;
  uint8_t data[key_size];

  uint8_t *begin() { return data; };
  uint8_t *end() { return data + key_size; };


  uint8_t const *begin() const { return data; };
  uint8_t const *end() const { return data + key_size; };


  AesKey() = default;
};


void aes_tiny_128_encrypt(uint8_t *buffer, AesKey const &key);

#ifdef ARDUINO_ARCH_ESP32
void aes_esp_128_encrypt(uint8_t *buffer, AesKey const &key);
constexpr auto aes_128_encrypt=aes_esp_128_encrypt;
#else
constexpr auto aes_128_encrypt=aes_tiny_128_encrypt;
#endif

#endif