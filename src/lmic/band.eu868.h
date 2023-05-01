#ifndef lmic_eu868_h
#define lmic_eu868_h

#include <stdint.h>

#include "bands.h"
#include "bufferpack.h"
#include "osticks.h"
#include <array>

class BandsEu868 : public Bands {
public:
  BandsEu868();
  void updateBandAvailability(uint8_t band, OsTime lastusage,
                              OsDeltaTime duration) final;
  void print_state() const final;
  OsTime getAvailability(uint8_t band) const final { return avail[band]; };

  static constexpr uint8_t MAX_BAND = 3;
  uint8_t getBandForFrequency(uint32_t frequency) const final;

#if defined(ENABLE_SAVE_RESTORE)

  void saveState(StoringAbtract &store) const final;
  void loadState(RetrieveAbtract &store) final;
  static constexpr uint16_t getStateSize();
#endif

private:
  std::array<OsTime,MAX_BAND> avail;
};

#endif