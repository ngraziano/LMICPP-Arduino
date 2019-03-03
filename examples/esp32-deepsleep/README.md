# Example for ESP32 with DEEP SLEEP

This is an exemple for ESP32 with arduino framework.

The system send a value, save the LMIC state and go to deep sleep.
At wake up  if a value was saved the systeme restore the saved state.

This example was tested with HELTEC board.

Only work on EU868.

## Specific to ESP32

Use:

- hardware AES for encoding.
- system random generator.
- RTC clock for timing.

Go deep sleep.


## Usage

Work with platformio.

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

In ``main.cpp`` replace the content of ``do_send()`` with the data you want to send.

Check the pin configuration for your board in ``lmic_pins`` structure.