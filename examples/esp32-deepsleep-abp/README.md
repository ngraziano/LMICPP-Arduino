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
