#ifndef cert_aesdecrypt_h
#define cert_aesdecrypt_h

#include <array>
#include <cstdint>

void aes128_decrypt(std::array<uint8_t, 16> const &key, uint8_t *data,
                    uint8_t len);

#endif