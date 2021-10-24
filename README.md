# LORAWAN Librairie for Arduino and SX1276 , SX1262 lora chip

* Based on LMIC librairy.
* Modified to get C++ style.
* Only class A and C device (no class B)
* Add some sleep of arduino board and ESP.
* Add SX1262 chip

## Limitation

This library try to comply with lorawan 1.0.3 but do not implement all feature. For example FSK datarate is not implemented.

For a more complete Lorawan library based on lmic, please have a look to <https://github.com/mcci-catena/arduino-lmic>

:warning: This library do not compile in Arduino IDE due to dependency to STL (ciband/avr_stl in case of AVR platform).
It need PlatfomIO for the dependencies to be handle correctly.

For the SX1262 it only support board with TCXO.

## AVR examples

Tested with Arduino Pro Mini and RFM95 on EU868 frequencies. Examples:

* [simple](examples/simple/) Minimal example, send one analog value.
* [balise](examples/balise/) Example, send one analog value with deepsleep using watchdog.
* [tempsensor](examples/tempsensor/) Example, send temps reads from onewire with deepsleep using watchdog.

## ESP32 examples

Tested with HelTec

* [esp32](examples/esp32/) Minimal example, send one value.
* [esp32-deepsleep](examples/esp32-deepsleep/) Example, send one analog value with deepsleep using RTC and state save in RTC RAM.

## RAK811 
* [simple_rak811](examples/simple_rak811) Minimal example for RAK811

## Usage

Work with platformio with Arduino framework.

Copy balise exemple in a new directory.
Open with platformio (VSCODE with Platformio extension)
In ``src`` directory create a file named ``lorakeys.h`` wich contain the keys declared in network (for exemple <https://www.thethingsnetwork.org>)

Exemple of file:

```cpp
// Application in string format.
// For TTN issued EUIs the first bytes should be 70B3D5
constexpr char const appEui[] = "70B3D5XXXXXXXXXX";

// Device EUI in string format.
constexpr char const devEui[] = "XXXXXXXXXXXXXXXX";
// Application key in string format.
constexpr char const appKey[] = "XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX";

```

Use define in platformio.ini `build_flags` to change activated part.

* ENABLE_SAVE_RESTORE enable save and restore functions
* LMIC_DEBUG_LEVEL set to 0,1 or 2 for different log levels (default value 1)

In ``main.cpp`` replace the content of ``do_send()`` with the data you want to send.


## License

Most source files in this repository are made available under the Eclipse Public License v1.0.
Some of the AES code is available under MIT like license. Refer to each individual source file for more details.
