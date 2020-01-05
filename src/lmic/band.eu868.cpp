#include "band.eu868.h"
#include "../hal/print_debug.h"
#include "bufferpack.h"
#include "oslmic.h"

namespace {
constexpr uint32_t MIN_BAND1_CENTI = 868000000;
constexpr uint32_t MAX_BAND1_CENTI = 868600000;
constexpr uint32_t MIN_BAND_DECI = 869400000;
constexpr uint32_t MAX_BAND_DECI = 869650000;
constexpr uint32_t MIN_BAND2_CENTI = 869700000;
constexpr uint32_t MAX_BAND2_CENTI = 870000000;

enum { BAND_MILLI = 0, BAND_CENTI = 1, BAND_DECI = 2 };

} // namespace

void BandsEu868::init() {
  auto now = os_getTime();
  for (uint8_t i = 0; i < MAX_BAND; i++) {
    avail[i] = now;
  }
}

void BandsEu868::updateBandAvailability(uint8_t const band,
                                        OsTime const lastusage,
                                        OsDeltaTime const duration) {
  uint16_t cap;
  if (band == BAND_MILLI) {
    cap = 1000;
  } else if (band == BAND_CENTI) {
    cap = 100;
  } else {
    // DECI
    cap = 10;
  }
  avail[band] = lastusage + cap * duration;

  PRINT_DEBUG(2, F("Setting  available time for band %d to %" PRIu32 ""), band,
              avail[band].tick());
}

void BandsEu868::print_state() const {

  for (uint8_t band_index = 0; band_index < MAX_BAND; band_index++) {
    PRINT_DEBUG(2, F("Band %d, available at %" PRIu32 "."), band_index,
                avail[band_index].tick());
  }
}

uint8_t BandsEu868::getBandForFrequency(uint32_t const frequency) {

  if (frequency >= MIN_BAND_DECI && frequency <= MAX_BAND_DECI)
    return BAND_DECI; // 10%
  else if ((frequency >= MIN_BAND1_CENTI && frequency <= MAX_BAND1_CENTI) ||
           (frequency >= MIN_BAND2_CENTI && frequency <= MAX_BAND2_CENTI))
    return BAND_CENTI; // 1%
  else
    return BAND_MILLI; // 0.1%
}

#if defined(ENABLE_SAVE_RESTORE)

void BandsEu868::saveState(StoringAbtract &store) const {
  for (int i = 0; i < MAX_BAND; i++) {
    store.write(avail[i]);
  }
}

void BandsEu868::loadState(RetrieveAbtract &store) {
  for (int i = 0; i < MAX_BAND; i++) {
    store.read(avail[i]);
  }
}

#endif
