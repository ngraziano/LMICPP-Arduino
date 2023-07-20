/*
 * Copyright (C) 2015,2018 Southern Storm Software, Pty Ltd.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a
 * copy of this software and associated documentation files (the "Software"),
 * to deal in the Software without restriction, including without limitation
 * the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included
 * in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS
 * OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
 * DEALINGS IN THE SOFTWARE.
 */

/*
Extract of https://github.com/rweather/arduinolibs/

Modified in 2019 by Nicolas Graziano

Change to use this project type and adapt to some C++ type.

*/

#include "aes_encrypt.h"
#include <algorithm>
#include <array>
#include <stdint.h>

#ifdef ARDUINO
#ifdef __AVR__
#include <avr/pgmspace.h>
#else
#include <pgmspace.h>
#endif
#else
#define PROGMEM
#define pgm_read_byte(addr) (*(const unsigned char *)(addr))
#endif
namespace {

struct Indices {
  const uint8_t col;
  const uint8_t row;
};

struct DataBlock {
  static constexpr uint8_t data_size = 16;

  uint8_t *begin() { return data.begin(); };
  uint8_t const *begin() const { return data.begin(); };

  uint8_t *end() { return data.end(); };
  uint8_t const *end() const { return data.end(); };

  uint8_t &operator[](Indices const i) { return data[i.col * 4 + i.row]; }
  uint8_t *column(uint8_t const col) { return data.begin() + (4 * col); }

private:
  std::array<uint8_t, data_size> data;
};

constexpr uint8_t sbox[256] PROGMEM = {
    0x63, 0x7C, 0x77, 0x7B, 0xF2, 0x6B, 0x6F, 0xC5, // 0x00
    0x30, 0x01, 0x67, 0x2B, 0xFE, 0xD7, 0xAB, 0x76,
    0xCA, 0x82, 0xC9, 0x7D, 0xFA, 0x59, 0x47, 0xF0, // 0x10
    0xAD, 0xD4, 0xA2, 0xAF, 0x9C, 0xA4, 0x72, 0xC0,
    0xB7, 0xFD, 0x93, 0x26, 0x36, 0x3F, 0xF7, 0xCC, // 0x20
    0x34, 0xA5, 0xE5, 0xF1, 0x71, 0xD8, 0x31, 0x15,
    0x04, 0xC7, 0x23, 0xC3, 0x18, 0x96, 0x05, 0x9A, // 0x30
    0x07, 0x12, 0x80, 0xE2, 0xEB, 0x27, 0xB2, 0x75,
    0x09, 0x83, 0x2C, 0x1A, 0x1B, 0x6E, 0x5A, 0xA0, // 0x40
    0x52, 0x3B, 0xD6, 0xB3, 0x29, 0xE3, 0x2F, 0x84,
    0x53, 0xD1, 0x00, 0xED, 0x20, 0xFC, 0xB1, 0x5B, // 0x50
    0x6A, 0xCB, 0xBE, 0x39, 0x4A, 0x4C, 0x58, 0xCF,
    0xD0, 0xEF, 0xAA, 0xFB, 0x43, 0x4D, 0x33, 0x85, // 0x60
    0x45, 0xF9, 0x02, 0x7F, 0x50, 0x3C, 0x9F, 0xA8,
    0x51, 0xA3, 0x40, 0x8F, 0x92, 0x9D, 0x38, 0xF5, // 0x70
    0xBC, 0xB6, 0xDA, 0x21, 0x10, 0xFF, 0xF3, 0xD2,
    0xCD, 0x0C, 0x13, 0xEC, 0x5F, 0x97, 0x44, 0x17, // 0x80
    0xC4, 0xA7, 0x7E, 0x3D, 0x64, 0x5D, 0x19, 0x73,
    0x60, 0x81, 0x4F, 0xDC, 0x22, 0x2A, 0x90, 0x88, // 0x90
    0x46, 0xEE, 0xB8, 0x14, 0xDE, 0x5E, 0x0B, 0xDB,
    0xE0, 0x32, 0x3A, 0x0A, 0x49, 0x06, 0x24, 0x5C, // 0xA0
    0xC2, 0xD3, 0xAC, 0x62, 0x91, 0x95, 0xE4, 0x79,
    0xE7, 0xC8, 0x37, 0x6D, 0x8D, 0xD5, 0x4E, 0xA9, // 0xB0
    0x6C, 0x56, 0xF4, 0xEA, 0x65, 0x7A, 0xAE, 0x08,
    0xBA, 0x78, 0x25, 0x2E, 0x1C, 0xA6, 0xB4, 0xC6, // 0xC0
    0xE8, 0xDD, 0x74, 0x1F, 0x4B, 0xBD, 0x8B, 0x8A,
    0x70, 0x3E, 0xB5, 0x66, 0x48, 0x03, 0xF6, 0x0E, // 0xD0
    0x61, 0x35, 0x57, 0xB9, 0x86, 0xC1, 0x1D, 0x9E,
    0xE1, 0xF8, 0x98, 0x11, 0x69, 0xD9, 0x8E, 0x94, // 0xE0
    0x9B, 0x1E, 0x87, 0xE9, 0xCE, 0x55, 0x28, 0xDF,
    0x8C, 0xA1, 0x89, 0x0D, 0xBF, 0xE6, 0x42, 0x68, // 0xF0
    0x41, 0x99, 0x2D, 0x0F, 0xB0, 0x54, 0xBB, 0x16};

static inline uint8_t readsbox(uint8_t val) {
  return pgm_read_byte(sbox + val);
}

// AES inverse S-box (http://en.wikipedia.org/wiki/Rijndael_S-box)
constexpr uint8_t sbox_inverse[256] PROGMEM = {
    0x52, 0x09, 0x6A, 0xD5, 0x30, 0x36, 0xA5, 0x38, // 0x00
    0xBF, 0x40, 0xA3, 0x9E, 0x81, 0xF3, 0xD7, 0xFB,
    0x7C, 0xE3, 0x39, 0x82, 0x9B, 0x2F, 0xFF, 0x87, // 0x10
    0x34, 0x8E, 0x43, 0x44, 0xC4, 0xDE, 0xE9, 0xCB,
    0x54, 0x7B, 0x94, 0x32, 0xA6, 0xC2, 0x23, 0x3D, // 0x20
    0xEE, 0x4C, 0x95, 0x0B, 0x42, 0xFA, 0xC3, 0x4E,
    0x08, 0x2E, 0xA1, 0x66, 0x28, 0xD9, 0x24, 0xB2, // 0x30
    0x76, 0x5B, 0xA2, 0x49, 0x6D, 0x8B, 0xD1, 0x25,
    0x72, 0xF8, 0xF6, 0x64, 0x86, 0x68, 0x98, 0x16, // 0x40
    0xD4, 0xA4, 0x5C, 0xCC, 0x5D, 0x65, 0xB6, 0x92,
    0x6C, 0x70, 0x48, 0x50, 0xFD, 0xED, 0xB9, 0xDA, // 0x50
    0x5E, 0x15, 0x46, 0x57, 0xA7, 0x8D, 0x9D, 0x84,
    0x90, 0xD8, 0xAB, 0x00, 0x8C, 0xBC, 0xD3, 0x0A, // 0x60
    0xF7, 0xE4, 0x58, 0x05, 0xB8, 0xB3, 0x45, 0x06,
    0xD0, 0x2C, 0x1E, 0x8F, 0xCA, 0x3F, 0x0F, 0x02, // 0x70
    0xC1, 0xAF, 0xBD, 0x03, 0x01, 0x13, 0x8A, 0x6B,
    0x3A, 0x91, 0x11, 0x41, 0x4F, 0x67, 0xDC, 0xEA, // 0x80
    0x97, 0xF2, 0xCF, 0xCE, 0xF0, 0xB4, 0xE6, 0x73,
    0x96, 0xAC, 0x74, 0x22, 0xE7, 0xAD, 0x35, 0x85, // 0x90
    0xE2, 0xF9, 0x37, 0xE8, 0x1C, 0x75, 0xDF, 0x6E,
    0x47, 0xF1, 0x1A, 0x71, 0x1D, 0x29, 0xC5, 0x89, // 0xA0
    0x6F, 0xB7, 0x62, 0x0E, 0xAA, 0x18, 0xBE, 0x1B,
    0xFC, 0x56, 0x3E, 0x4B, 0xC6, 0xD2, 0x79, 0x20, // 0xB0
    0x9A, 0xDB, 0xC0, 0xFE, 0x78, 0xCD, 0x5A, 0xF4,
    0x1F, 0xDD, 0xA8, 0x33, 0x88, 0x07, 0xC7, 0x31, // 0xC0
    0xB1, 0x12, 0x10, 0x59, 0x27, 0x80, 0xEC, 0x5F,
    0x60, 0x51, 0x7F, 0xA9, 0x19, 0xB5, 0x4A, 0x0D, // 0xD0
    0x2D, 0xE5, 0x7A, 0x9F, 0x93, 0xC9, 0x9C, 0xEF,
    0xA0, 0xE0, 0x3B, 0x4D, 0xAE, 0x2A, 0xF5, 0xB0, // 0xE0
    0xC8, 0xEB, 0xBB, 0x3C, 0x83, 0x53, 0x99, 0x61,
    0x17, 0x2B, 0x04, 0x7E, 0xBA, 0x77, 0xD6, 0x26, // 0xF0
    0xE1, 0x69, 0x14, 0x63, 0x55, 0x21, 0x0C, 0x7D};

static inline uint8_t readsbox_inv(uint8_t val) {
  return pgm_read_byte(sbox_inverse + val);
}

// Rcon(i), 2^(i+1) in the Rijndael finite field, for i = 0..9.
// http://en.wikipedia.org/wiki/Rijndael_key_schedule
constexpr uint8_t const rcon[10] PROGMEM = {0x01, 0x02, 0x04, 0x08, 0x10,
                                            0x20, 0x40, 0x80, 0x1B, 0x36};

static inline std::array<uint8_t, 4> keyScheduleCore(const uint8_t *input,
                                   uint8_t iteration) {
  return std::array<uint8_t, 4> {
    (uint8_t)(readsbox(input[1]) ^ pgm_read_byte(rcon + iteration)),
    readsbox(input[2]),
    readsbox(input[3]),
    readsbox(input[0])
  };
}

static inline void subBytesAndShiftRows(DataBlock &buffer) {

  std::transform(buffer.begin(), buffer.end(), buffer.begin(),
                 [](uint8_t c) -> uint8_t { return readsbox(c); });

  // shift row
  /*
  buffer[{0, 0}] = buffer[{0, 0}];
  buffer[{1, 0}] = buffer[{1, 0}];
  buffer[{2, 0}] = buffer[{2, 0}];
  buffer[{3, 0}] = buffer[{3, 0}];
  */

  const uint8_t temp01 = buffer[{0, 1}];
  buffer[{0, 1}] = buffer[{1, 1}];
  buffer[{1, 1}] = buffer[{2, 1}];
  buffer[{2, 1}] = buffer[{3, 1}];
  buffer[{3, 1}] = temp01;

  const uint8_t temp02 = buffer[{0, 2}];
  buffer[{0, 2}] = buffer[{2, 2}];
  buffer[{2, 2}] = temp02;

  const uint8_t temp03 = buffer[{0, 3}];
  buffer[{0, 3}] = buffer[{3, 3}];
  buffer[{3, 3}] = buffer[{2, 3}];
  buffer[{2, 3}] = buffer[{1, 3}];
  buffer[{1, 3}] = temp03;

  const uint8_t temp12 = buffer[{1, 2}];
  buffer[{1, 2}] = buffer[{3, 2}];
  buffer[{3, 2}] = temp12;
}

// Multiply x by 2 in the Galois field, to achieve the effect of the following:
//
//     if (x & 0x80)
//         return (x << 1) ^ 0x1B;
//     else
//         return (x << 1);
//
// However, we don't want to use runtime conditionals if we can help it
// to avoid leaking timing information from the implementation.
// In this case, multiplication is slightly faster than table lookup on AVR.
uint8_t gmul2(uint8_t const x) {
  uint8_t const t = x << 1;
  uint8_t const xorval = 0x1B * (x >> 7);
  return t ^ xorval;
}

void mixColumn(uint8_t *buffer) {
  uint8_t const a = buffer[0];
  uint8_t const b = buffer[1];
  uint8_t const c = buffer[2];
  uint8_t const d = buffer[3];
  uint8_t const a2 = gmul2(a);
  uint8_t const b2 = gmul2(b);
  uint8_t const c2 = gmul2(c);
  uint8_t const d2 = gmul2(d);
  buffer[0] = a2 ^ b2 ^ b ^ c ^ d;
  buffer[1] = a ^ b2 ^ c2 ^ c ^ d;
  buffer[2] = a ^ b ^ c2 ^ d2 ^ d;
  buffer[3] = a2 ^ a ^ b ^ c ^ d2;
}

void kcore(uint8_t n, AesKey &schedule) {
  std::array<uint8_t, 4> temp = keyScheduleCore(schedule.begin() + 12, n);
  schedule[0] ^= temp[0];
  schedule[1] ^= temp[1];
  schedule[2] ^= temp[2];
  schedule[3] ^= temp[3];
}

void kxor(uint8_t const a, uint8_t const b, AesKey &schedule) {
  schedule[a * 4] ^= schedule[b * 4];
  schedule[a * 4 + 1] ^= schedule[b * 4 + 1];
  schedule[a * 4 + 2] ^= schedule[b * 4 + 2];
  schedule[a * 4 + 3] ^= schedule[b * 4 + 3];
}

void expand_key(AesKey &schedule, uint8_t round) {
  kcore(round, schedule);
  kxor(1, 0, schedule);
  kxor(2, 1, schedule);
  kxor(3, 2, schedule);
}

void xorbuffer(uint8_t const *source1, AesKey &source2, uint8_t *dest) {
  std::transform(source1, source1 + 16, source2.begin(), dest,
                 [](uint8_t a, uint8_t b) { return a ^ b; });
}

void xorbuffer(DataBlock const &source1, AesKey &source2, uint8_t *dest) {
  xorbuffer(source1.begin(), source2, dest);
}

void xorbuffer(uint8_t const *source1, AesKey &source2, DataBlock &dest) {
  xorbuffer(source1, source2, dest.begin());
}

void xorbuffer(DataBlock const &source1, AesKey &source2, DataBlock &dest) {
  xorbuffer(source1.begin(), source2, dest.begin());
}

} // namespace

void aes_tiny_128_encrypt(uint8_t *buffer, AesKey const &key) {

  // Start with the key in the schedule buffer.
  AesKey schedule = key;

  DataBlock state1;
  // Copy the input into the state and XOR with the key schedule.
  xorbuffer(buffer, schedule, state1);
  // Perform the first 9 rounds of the cipher.
  for (uint8_t round = 0; round < 9; ++round) {
    // Expand the next 16 bytes of the key schedule.
    expand_key(schedule, round);
    // Encrypt using the key schedule.
    subBytesAndShiftRows(state1);
    mixColumn(state1.column(0));
    mixColumn(state1.column(1));
    mixColumn(state1.column(2));
    mixColumn(state1.column(3));
    xorbuffer(state1, schedule, state1);
  }

  // Expand the final 16 bytes of the key schedule.
  expand_key(schedule, 9);

  // Perform the final round.
  subBytesAndShiftRows(state1);
  xorbuffer(state1, schedule, buffer);
}

void invShiftRowsAndSubBytes(DataBlock &buffer) {

  std::transform(buffer.begin(), buffer.end(), buffer.begin(),
                 [](uint8_t c) -> uint8_t { return readsbox_inv(c); });

  // shift row
  /*
  buffer[{0, 0}] = buffer[{0, 0}];
  buffer[{1, 0}] = buffer[{1, 0}];
  buffer[{2, 0}] = buffer[{2, 0}];
  buffer[{3, 0}] = buffer[{3, 0}];
  */

  const uint8_t temp01 = buffer[{0, 1}];
  buffer[{0, 1}] = buffer[{3, 1}];
  buffer[{3, 1}] = buffer[{2, 1}];
  buffer[{2, 1}] = buffer[{1, 1}];
  buffer[{1, 1}] = temp01;

  const uint8_t temp02 = buffer[{0, 2}];
  buffer[{0, 2}] = buffer[{2, 2}];
  buffer[{2, 2}] = temp02;

  const uint8_t temp03 = buffer[{0, 3}];
  buffer[{0, 3}] = buffer[{1, 3}];
  buffer[{1, 3}] = buffer[{2, 3}];
  buffer[{2, 3}] = buffer[{3, 3}];
  buffer[{3, 3}] = temp03;

  const uint8_t temp12 = buffer[{1, 2}];
  buffer[{1, 2}] = buffer[{3, 2}];
  buffer[{3, 2}] = temp12;
}

void invMixColumns(uint8_t *buffer) {
  // Inverse mix columns.
  uint8_t const a = buffer[0];
  uint8_t const b = buffer[1];
  uint8_t const c = buffer[2];
  uint8_t const d = buffer[3];
  uint8_t const a2 = gmul2(a);
  uint8_t const b2 = gmul2(b);
  uint8_t const c2 = gmul2(c);
  uint8_t const d2 = gmul2(d);
  uint8_t const a4 = gmul2(a2);
  uint8_t const b4 = gmul2(b2);
  uint8_t const c4 = gmul2(c2);
  uint8_t const d4 = gmul2(d2);
  uint8_t const a8 = gmul2(a4);
  uint8_t const b8 = gmul2(b4);
  uint8_t const c8 = gmul2(c4);
  uint8_t const d8 = gmul2(d4);
  buffer[0] = a8 ^ a4 ^ a2 ^ b8 ^ b2 ^ b ^ c8 ^ c4 ^ c ^ d8 ^ d;
  buffer[1] = a8 ^ a ^ b8 ^ b4 ^ b2 ^ c8 ^ c2 ^ c ^ d8 ^ d4 ^ d;
  buffer[2] = a8 ^ a4 ^ a ^ b8 ^ b ^ c8 ^ c4 ^ c2 ^ d8 ^ d2 ^ d;
  buffer[3] = a8 ^ a2 ^ a ^ b8 ^ b4 ^ b ^ c8 ^ c ^ d8 ^ d4 ^ d2;
}

void inv_expand_key(AesKey &schedule, uint8_t round) {
  kxor(3, 2, schedule);
  kxor(2, 1, schedule);
  kxor(1, 0, schedule);
  kcore(round, schedule);
}

void aes_tiny_128_decrypt(uint8_t *buffer, AesKey const &key) {
  // Start with the key in the schedule buffer.
  // get the last schedule
  AesKey schedule = key;
  for (uint8_t round = 0; round < 10; ++round) {
    expand_key(schedule, round);
  }

  DataBlock state1;
  // reverse the final round
  xorbuffer(buffer, schedule, state1);
  invShiftRowsAndSubBytes(state1);
  // reverse the schedule for the final round
  inv_expand_key(schedule, 9);

  // Perform the next 9 rounds of the decryption process.
  for (int8_t round = 8; round >= 0; --round) {
    // Decrypt using the key schedule.
    xorbuffer(state1, schedule, state1);
    invMixColumns(state1.column(3));
    invMixColumns(state1.column(2));
    invMixColumns(state1.column(1));
    invMixColumns(state1.column(0));
    invShiftRowsAndSubBytes(state1);
    // Expand the next 16 bytes of the key schedule in reverse.
    inv_expand_key(schedule, round);
  }

  // Reverse the initial round and create the output words.
  xorbuffer(state1, schedule, buffer);
}