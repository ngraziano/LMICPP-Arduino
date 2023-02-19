#include "bands.h"
#include "../hal/print_debug.h"
#include "bufferpack.h"
#include "oslmic.h"

BandSingle::BandSingle(uint16_t duty) : dutyCycle{duty} {}

void BandSingle::init() { avail = os_getTime(); }

void BandSingle::updateBandAvailability(uint8_t const, OsTime const lastusage,
                                        OsDeltaTime const duration) {
  avail = lastusage + dutyCycle * duration;

  PRINT_DEBUG(2, F("Setting  available time for bandto %" PRIu32 ""),
              avail.tick());
}

void BandSingle::print_state() const {

  PRINT_DEBUG(2, F("Band , available at %" PRIu32 "."), avail.tick());
}

uint8_t BandSingle::getBandForFrequency(uint32_t const) const { return 0; }

#if defined(ENABLE_SAVE_RESTORE)

void BandSingle::saveState(StoringAbtract &store) const { store.write(avail); }

void BandSingle::loadState(RetrieveAbtract &store) { store.read(avail); }

#endif
