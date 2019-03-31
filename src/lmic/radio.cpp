#include "radio.h"

int16_t Radio::get_last_packet_rssi() const {
  // see documentation for -139
  // do not handle snr > 0
  return -139 + last_packet_rssi_reg;
}

int8_t Radio::get_last_packet_snr_x4() const {
  return last_packet_snr_reg;
}

/**
 * Check the IO pin.
 * Return true if the radio has finish it's operation
 */
bool Radio::io_check() const { return hal.io_check(); }


Radio::Radio(lmic_pinmap const &pins) : hal(pins) {}
