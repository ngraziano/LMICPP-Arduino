#ifndef _lorakeyhandler_h_
#define _lorakeyhandler_h_

#include <Arduino.h>
#include <lmic.h>

constexpr uint8_t HexCharToInt(char const char1)
{

    return (char1 >= '0' && char1 <= '9') ? char1 - '0' : (char1 >= 'A' && char1 <= 'F') ? char1 - 'A' + 0x0A : (char1 >= 'a' && char1 <= 'f') ? char1 - 'a' + 0x0A : 0;
}

constexpr uint8_t HexCharToInt(char const char1, char const char2)
{

    return (HexCharToInt(char1) * 0x10) + HexCharToInt(char2);
}

template <char const *KEY>
class EuiGetter
{
  private:
    static constexpr size_t SIZE = 8;
    static uint8_t const PROGMEM key[SIZE];

  public:
    static void getEui(uint8_t *buf) { memcpy_P(buf, key, SIZE); }
};

template <char const *KEY>
uint8_t const PROGMEM EuiGetter<KEY>::key[SIZE] = {
    HexCharToInt(KEY[14], KEY[15]),
    HexCharToInt(KEY[12], KEY[13]),
    HexCharToInt(KEY[10], KEY[11]),
    HexCharToInt(KEY[8], KEY[9]),
    HexCharToInt(KEY[6], KEY[7]),
    HexCharToInt(KEY[4], KEY[5]),
    HexCharToInt(KEY[2], KEY[3]),
    HexCharToInt(KEY[0], KEY[1]),
};

template <char const *KEY>
class KeyGetter
{
  private:
    static constexpr size_t SIZE = 16;
    static uint8_t const PROGMEM key[SIZE];

  public:
    static AesKey getKey()
    {
        AesKey lmicKey;
        memcpy_P(lmicKey.data, key, SIZE);
        return lmicKey;
    }
};

template <char const *KEY>
uint8_t const PROGMEM KeyGetter<KEY>::key[SIZE] = {
    HexCharToInt(KEY[0], KEY[1]),
    HexCharToInt(KEY[2], KEY[3]),
    HexCharToInt(KEY[4], KEY[5]),
    HexCharToInt(KEY[6], KEY[7]),
    HexCharToInt(KEY[8], KEY[9]),
    HexCharToInt(KEY[10], KEY[11]),
    HexCharToInt(KEY[12], KEY[13]),
    HexCharToInt(KEY[14], KEY[15]),
    HexCharToInt(KEY[16], KEY[17]),
    HexCharToInt(KEY[18], KEY[19]),
    HexCharToInt(KEY[20], KEY[21]),
    HexCharToInt(KEY[22], KEY[23]),
    HexCharToInt(KEY[24], KEY[25]),
    HexCharToInt(KEY[26], KEY[27]),
    HexCharToInt(KEY[28], KEY[29]),
    HexCharToInt(KEY[30], KEY[31]),
};

template <char const *APPEUI, char const *DEVEUI, char const *APPKEY>
class SetupLmicKey
{
  private:
    static EuiGetter<APPEUI> const appEuiGetter;
    static EuiGetter<DEVEUI> const devEuiGetter;
    static KeyGetter<APPKEY> const appKeyGetter;

  public:
    static void setup(Lmic &lmic)
    {
        lmic.setDevKey(appKeyGetter.getKey());
        lmic.setDevEuiCallback(devEuiGetter.getEui);
        lmic.setArtEuiCallback(appEuiGetter.getEui);
    };
};

#endif