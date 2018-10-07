#ifndef _regionlmic_h_
#define _regionlmic_h_

#include "lorabase.h"
#include <stdint.h>

class RegionLmic {
protected:
  virtual uint8_t getRawRps(dr_t dr) const = 0;
public:
  rps_t updr2rps(dr_t dr) const;
  rps_t dndr2rps(dr_t dr) const;
  bool isFasterDR(dr_t dr1, dr_t dr2) const;
  bool isSlowerDR(dr_t dr1, dr_t dr2) const;
  // increase data rate
  dr_t incDR(dr_t dr) const;
  // decrease data rate 
  dr_t decDR(dr_t dr) const;
  // in range
  bool validDR(dr_t dr) const;
  // decrease data rate by n steps
  dr_t lowerDR(dr_t dr, uint8_t n) const;

  virtual uint8_t defaultRX2Dr() const = 0;
  virtual uint32_t defaultRX2Freq() const = 0;

};

#endif