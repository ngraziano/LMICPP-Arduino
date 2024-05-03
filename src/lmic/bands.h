#ifndef lmic_bands_h
#define lmic_bands_h

#include <stdint.h>

#include "bufferpack.h"
#include "osticks.h"
#include "oslmic.h"
#include "../hal/print_debug.h"

class Bands {
public:
  virtual void updateBandAvailability(uint8_t band, OsTime lastusage,
                                      OsDeltaTime duration) = 0;
  virtual OsTime getAvailability(uint8_t band) const = 0;

  virtual uint8_t getBandForFrequency(uint32_t frequency) const = 0;

#if defined(ENABLE_SAVE_RESTORE)

  virtual void saveState(StoringAbtract &store) const = 0;
  virtual void loadState(RetrieveAbtract &store) = 0;
#endif
};

template <uint16_t dutyCycle> class BandSingle : public Bands {
public:
  BandSingle() : avail{os_getTime()} {};

  void updateBandAvailability(uint8_t, OsTime lastusage,
                              OsDeltaTime duration) final {
    avail = lastusage + dutyCycle * duration;

    PRINT_DEBUG(2, F("Setting  available time for bandto %" PRIu32 ""),
                avail.tick());
  };
  void print_state() const  {
    PRINT_DEBUG(2, F("Band , available at %" PRIu32 "."), avail.tick());
  };
  OsTime getAvailability(uint8_t) const final { return avail; };

  static constexpr uint8_t MAX_BAND = 1;
  uint8_t getBandForFrequency(uint32_t) const final { return 0; };

#if defined(ENABLE_SAVE_RESTORE)

  void saveState(StoringAbtract &store) const final { store.write(avail); };
  void loadState(RetrieveAbtract &store) final { store.read(avail); };
  static constexpr uint16_t getStateSize() { return sizeof(avail); };
#endif

private:
  OsTime avail;
};

#endif