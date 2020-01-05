#ifndef lmic_eu868_h
#define lmic_eu868_h

#include <stdint.h>

#include "bufferpack.h"
#include "osticks.h"

class BandsEu868 {
public:
  void init();
  void updateBandAvailability(uint8_t band, OsTime lastusage,
                              OsDeltaTime duration);
  void print_state() const;
  OsTime getAvailability(uint8_t band) { return avail[band]; };

  static constexpr uint8_t MAX_BAND = 3;
  static uint8_t getBandForFrequency(uint32_t frequency);

#if defined(ENABLE_SAVE_RESTORE)

  void saveState(StoringAbtract &store) const;
  void loadState(RetrieveAbtract &store);
#endif

private:
  OsTime avail[MAX_BAND];
};

#endif