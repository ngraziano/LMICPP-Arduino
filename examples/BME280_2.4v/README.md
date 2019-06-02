# Sample of LMICPP-Arduino for Arduino Pro mini with deepsleep at 2Mhz and with custom bootloader

*Warning* : Not standart bootloader **must** be installed to handle watchdog and low voltage

Tested with Arduino Pro Mini and RFM95 on EU868 frequencies.

## Usage

Work with platformio.

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

Calibrate deepsleep duration (see below)

Int 1 / Pin 3 is use to wake a with a button linked to ground.

## Calibration of deep sleep

During start there is a test of deepsleep time.
You need to calibrate it.
Use a terminal which print the time the message are receive (YAT for example) and mesure time between message `Start Test sleep time.` and `End Test sleep time.` divide this time by the time in `Test Time should be :` message and ajust `sleepAdj` acordingly.
