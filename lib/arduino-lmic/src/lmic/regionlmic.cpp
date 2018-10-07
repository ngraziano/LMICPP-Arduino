
#include "regionlmic.h"

rps_t RegionLmic::updr2rps(dr_t dr) const {
  rps_t result;
  result.rawValue = getRawRps(dr);
  return result;
}

rps_t RegionLmic::dndr2rps(dr_t dr) const {
  auto val = updr2rps(dr);
  val.nocrc = 1;
  return val;
}

bool RegionLmic::isFasterDR(dr_t dr1, dr_t dr2) const { return dr1 > dr2; }

bool RegionLmic::isSlowerDR(dr_t dr1, dr_t dr2) const { return dr1 < dr2; }

// increase data rate
dr_t RegionLmic::incDR(dr_t dr) const { return validDR(dr + 1) ? dr + 1 : dr; }

// decrease data rate
dr_t RegionLmic::decDR(dr_t dr) const { return validDR(dr - 1) ? dr - 1 : dr; }

// in range
bool RegionLmic::validDR(dr_t dr) const { return getRawRps(dr) != ILLEGAL_RPS; }

// decrease data rate by n steps
dr_t RegionLmic::lowerDR(dr_t dr, uint8_t n) const {
  while (n--) {
    dr = decDR(dr);
  };
  return dr;
}