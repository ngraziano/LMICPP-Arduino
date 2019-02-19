# LORAWAN Librairie for Arduino and SX1276/2 lora chip

* Based on LMIC librairy.
* Modified to get C++ style.
* Only class A device (no class B)
* Add some sleep of arduino board.

Tested with Arduino Pro Mini and RFM95 on EU868 frequencies.

## Usage

Work with platformio.

In ``src`` directory create a file named ``lorakeys.h`` wich contain the keys declared in network (for exemple <https://www.thethingsnetwork.org>)

Exemple of file:

```c
#include <Arduino.h>
// This EUI must be in little-endian format, so least-significant-byte
// first. When copying an EUI from ttnctl output, this means to reverse
// the bytes. For TTN issued EUIs the last bytes should be 0xD5, 0xB3,
// 0x70.
static const uint8_t PROGMEM APPEUI[8]={ 0xXX, 0xXX, 0xXX, 0xXX, 0xXX, 0xD5, 0xB3, 0x70 };

// This should also be in little endian format, see above.
static const uint8_t PROGMEM DEVEUI[8]={ 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };

// This key should be in big endian format (or, since it is not really a
// number but a block of memory, endianness does not really apply). In
// practice, a key taken from ttnctl can be copied as-is.
// The key shown here is the semtech default key.
static const uint8_t PROGMEM APPKEY[16] = { 0x2B, 0x7E, 0x15, 0x16, 0x28, 0xAE, 0xD2, 0xA6, 0xAB, 0xF7, 0x15, 0x88, 0x09, 0xCF, 0x4F, 0x3C };
```

In ``main.cpp`` replace the content of ``do_send()`` with the data you want to send.

## Main functional change from LMIC

* Try to implement ADR a little more correctl:
  * Handle Txpower
  * ADR_ACK_LIMIT set to 64
  * ADR_ACK_DELAY set to 32
* Correct set of power for SX1276

## License

Most source files in this repository are made available under the Eclipse Public License v1.0.
Some of the AES code is available under MIT like license. Refer to each individual source file for more details.
