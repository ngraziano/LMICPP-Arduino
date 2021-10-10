#ifndef lmic_bands_h
#define lmic_bands_h

#include <stdint.h>

#include "bufferpack.h"
#include "osticks.h"

class Bands {
public:
  virtual void init() = 0;
  virtual void updateBandAvailability(uint8_t band, OsTime lastusage,
                                      OsDeltaTime duration) = 0;
  virtual void print_state() const = 0;
  virtual OsTime getAvailability(uint8_t band) const = 0;

  virtual uint8_t getBandForFrequency(uint32_t frequency) const = 0;

#if defined(ENABLE_SAVE_RESTORE)

  virtual void saveState(StoringAbtract &store) const = 0;
  virtual void loadState(RetrieveAbtract &store) = 0;
#endif
};

class BandSingle : public Bands {
public:
  explicit BandSingle(uint16_t duty);
  void init() final;
  void updateBandAvailability(uint8_t band, OsTime lastusage,
                              OsDeltaTime duration) final;
  void print_state() const final;
  OsTime getAvailability(uint8_t) const final { return avail; };

  static constexpr uint8_t MAX_BAND = 1;
  uint8_t getBandForFrequency(uint32_t frequency) const final;

#if defined(ENABLE_SAVE_RESTORE)

  void saveState(StoringAbtract &store) const final;
  void loadState(RetrieveAbtract &store) final;
#endif

private:
  const uint16_t dutyCycle;
  OsTime avail;
};

#endif