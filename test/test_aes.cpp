#include "test_aes.h"

#include <algorithm>
#include "aes/aes_encrypt.h"
#include <unity.h>

namespace
{
constexpr uint8_t HexCharToInt(char const char1)
{

    return (char1 >= '0' && char1 <= '9')
               ? char1 - '0'
               : (char1 >= 'A' && char1 <= 'F')
                     ? char1 - 'A' + 0x0A
                     : (char1 >= 'a' && char1 <= 'f') ? char1 - 'a' + 0x0A : 0;
}
constexpr uint8_t HexCharToInt(char const char1, char const char2)
{

    return (HexCharToInt(char1) * 0x10) + HexCharToInt(char2);
}

class ValGetter
{
public:
    static const uint8_t size = 16;
    uint8_t const val[size];

    uint8_t const *begin() const { return val; };
    uint8_t const *end() const { return val + size; };

    ValGetter(char const *VAL) : val{HexCharToInt(VAL[0], VAL[1]), HexCharToInt(VAL[2], VAL[3]),
                                     HexCharToInt(VAL[4], VAL[5]), HexCharToInt(VAL[6], VAL[7]),
                                     HexCharToInt(VAL[8], VAL[9]), HexCharToInt(VAL[10], VAL[11]),
                                     HexCharToInt(VAL[12], VAL[13]), HexCharToInt(VAL[14], VAL[15]),
                                     HexCharToInt(VAL[16], VAL[17]), HexCharToInt(VAL[18], VAL[19]),
                                     HexCharToInt(VAL[20], VAL[21]), HexCharToInt(VAL[22], VAL[23]),
                                     HexCharToInt(VAL[24], VAL[25]), HexCharToInt(VAL[26], VAL[27]),
                                     HexCharToInt(VAL[28], VAL[29]), HexCharToInt(VAL[30], VAL[31])} {};
};

} // namespace

namespace test_aes
{

void run()
{
    RUN_TEST(test_aes_key);
    RUN_TEST(test_aes_encript_with_key0);
    RUN_TEST(test_aes_encript_with_buff0);
}

static ValGetter fake_key("000102030405060708090A0B0C0D0E0F");

/**
 * Test AES Key object for copy
 */
void test_aes_key()
{
    AesKey test_key;
    TEST_ASSERT_EQUAL(16, test_key.max_size());

    std::copy(fake_key.begin(), fake_key.end(), test_key.begin());
    AesKey copy_key = test_key;
    std::fill(test_key.begin(), test_key.end(), 0);
    TEST_ASSERT_EQUAL_MEMORY(fake_key.val, copy_key.begin(), copy_key.max_size());
}

void encrypt_run_key0(ValGetter const &plaintext, ValGetter const &result)
{

    AesKey key;
    std::fill(key.begin(), key.end(), 0);
    uint8_t buffer[16];

    std::copy(plaintext.begin(), plaintext.end(), buffer);
    aes_128_encrypt(buffer, key);
    TEST_ASSERT_EQUAL_MEMORY(result.val, buffer, result.size);
}

static ValGetter plaintext0("f34481ec3cc627bacd5dc3fb08f273e6");
static ValGetter result0("0336763e966d92595a567cc9ce537f5e");
static ValGetter plaintext1("9798c4640bad75c7c3227db910174e72");
static ValGetter result1("a9a1631bf4996954ebc093957b234589");
static ValGetter plaintext2("96ab5c2ff612d9dfaae8c31f30c42168");
static ValGetter result2("ff4f8391a6a40ca5b25d23bedd44a597");

/**
 * Test known value with a key all equal to 0
 */
void test_aes_encript_with_key0()
{
    encrypt_run_key0(plaintext0, result0);
    encrypt_run_key0(plaintext1, result1);
    encrypt_run_key0(plaintext2, result2);
}

void encrypt_run_buff0(ValGetter const &key_val, ValGetter const &result)
{

    AesKey key;
    std::copy(key_val.begin(), key_val.end(), key.begin());
    uint8_t buffer[16];
    std::fill(buffer, buffer + 16, 0);

    aes_128_encrypt(buffer, key);
    TEST_ASSERT_EQUAL_MEMORY(result.val, buffer, result.size);
}

static ValGetter test_key10("10a58869d74be5a374cf867cfb473859");
static ValGetter result10("6d251e6944b051e04eaa6fb4dbf78465");
static ValGetter test_key11("caea65cdbb75e9169ecd22ebe6e54675");
static ValGetter result11("6e29201190152df4ee058139def610bb");
static ValGetter test_key12("a2e2fa9baf7d20822ca9f0542f764a41");
static ValGetter result12("c3b44b95d9d2f25670eee9a0de099fa3");

/**
 * Test known value with a buffer all equal to  zeron
 */
void test_aes_encript_with_buff0()
{

    encrypt_run_buff0(test_key10, result10);
    encrypt_run_buff0(test_key11, result11);
    encrypt_run_buff0(test_key12, result12);
}

} // namespace test_aes