
#include "powersave.h"
#include <Arduino.h>
#include <hal/print_debug.h>
#include <lmic.h>
#include <sleepandwatchdog.h>

const int64_t sleepAdj = 1080;

void powersave(OsDeltaTime maxTime, stopsleepcb_t interrupt) {
  OsDeltaTime duration_selected;
  Sleep period_selected;
  // these value are base on test
  if (maxTime > OsDeltaTime::from_ms(8700)) {
    duration_selected = OsDeltaTime::from_ms(8000 * sleepAdj / 1000);
    period_selected = Sleep::P8S;
  } else if (maxTime > OsDeltaTime::from_ms(4600)) {
    duration_selected = OsDeltaTime::from_ms(4000 * sleepAdj / 1000);
    period_selected = Sleep::P4S;
  } else if (maxTime > OsDeltaTime::from_ms(2600)) {
    duration_selected = OsDeltaTime::from_ms(2000 * sleepAdj / 1000);
    period_selected = Sleep::P2S;
  } else if (maxTime > OsDeltaTime::from_ms(1500)) {
    duration_selected = OsDeltaTime::from_ms(1000 * sleepAdj / 1000);
    period_selected = Sleep::P1S;
  } else if (maxTime > OsDeltaTime::from_ms(800)) {
    duration_selected = OsDeltaTime::from_ms(500 * sleepAdj / 1000);
    period_selected = Sleep::P500MS;
  } else if (maxTime > OsDeltaTime::from_ms(500)) {
    duration_selected = OsDeltaTime::from_ms(250 * sleepAdj / 1000);
    period_selected = Sleep::P250MS;
  } else {
    return;
  }

  PRINT_DEBUG(1, F("Sleep (ostick) :%lix%i"), duration_selected.to_ms(),
              maxTime / duration_selected);
  if (debugLevel > 0) {
    Serial.flush();
  }

  bool stopsleep = false;
  for (uint16_t nbsleep = maxTime / duration_selected;
       nbsleep > 0 && !stopsleep; nbsleep--) {
    powerDown(period_selected);
    hal_add_time_in_sleep(duration_selected);
    stopsleep = interrupt();
  }
  PRINT_DEBUG(1, F("Wakeup"));
}
