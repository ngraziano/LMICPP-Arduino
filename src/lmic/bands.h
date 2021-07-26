#ifndef lmic_bands_h
#define lmic_bands_h

#include <stdint.h>

#include "bufferpack.h"
#include "osticks.h"

class Bands {
public:
  virtual void init();
  virtual void updateBandAvailability(uint8_t band, OsTime lastusage,
                                      OsDeltaTime duration);
  virtual void print_state() const;
  virtual OsTime getAvailability(uint8_t band) const;

  virtual uint8_t getBandForFrequency(uint32_t frequency) const;

#if defined(ENABLE_SAVE_RESTORE)

  virtual void saveState(StoringAbtract &store) const;
  virtual void loadState(RetrieveAbtract &store);
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