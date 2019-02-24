# LORAWAN Librairie for Arduino and SX1276/2 lora chip

* Based on LMIC librairy.
* Modified to get C++ style.
* Only class A device (no class B)
* Add some sleep of arduino board.

Tested with Arduino Pro Mini and RFM95 on EU868 frequencies.

## Usage

Work with platformio.

Copy balise exemple in a new directory.
Open with platformio (VSCODE with Platformio extension)
In ``src`` directory create a file named ``lorakeys.h`` wich contain the keys declared in network (for exemple <https://www.thethingsnetwork.org>)

Exemple of file:

```c
// Application in string format.
// For TTN issued EUIs the first bytes should be 70B3D5
constexpr char const appEui[] = "70B3D5XXXXXXXXXX";

// Device EUI in string format.
constexpr char const devEui[] = "XXXXXXXXXXXXXXXX";
// Application key in string format.
constexpr char const appKey[] = "XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX";

```

In ``main.cpp`` replace the content of ``do_send()`` with the data you want to send.

## Main functional change from LMIC

* Try to implement ADR a little more correctl:
  * Handle Txpower
  * ADR_ACK_LIMIT set to 64
  * ADR_ACK_DELAY set to 32
* Correct set of power for SX1276
* Various coding style fix (remove goto ...)

## License

Most source files in this repository are made available under the Eclipse Public License v1.0.
Some of the AES code is available under MIT like license. Refer to each individual source file for more details.
