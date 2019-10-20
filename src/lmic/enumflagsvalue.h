#ifndef _enumflagsvalue_h_
#define _enumflagsvalue_h_

#include <stdint.h>

template <typename T> class EnumFlagsValue final {
public:
  uint8_t value;

  EnumFlagsValue &set(T bit) {
    value |= (1 << static_cast<uint8_t>(bit));
    return *this;
  };

  EnumFlagsValue &reset() {
    value = 0;
    return *this;
  };

  EnumFlagsValue &reset(T bit) {
    value &= ~(1 << static_cast<uint8_t>(bit));
    return *this;
  };

  constexpr bool test(T bit) const {
    return (value & (1 << static_cast<uint8_t>(bit))) != 0;
  };
};
#endif