#ifndef BME280_h
#define BME280_h

#include "Arduino.h"
#include <stdint.h>

enum class BME280Registers : uint8_t {
  dig_T1 = 0x88,
  dig_T2 = 0x8A,
  dig_T3 = 0x8C,

  dig_P1 = 0x8E,
  dig_P2 = 0x90,
  dig_P3 = 0x92,
  dig_P4 = 0x94,
  dig_P5 = 0x96,
  dig_P6 = 0x98,
  dig_P7 = 0x9A,
  dig_P8 = 0x9C,
  dig_P9 = 0x9E,

  dig_H1 = 0xA1,

  chipid = 0xD0,

  dig_H2 = 0xE1,
  dig_H3 = 0xE3,
  dig_H4 = 0xE4,
  dig_H45 = 0xE5,
  dig_H5 = 0xE6,
  dig_H6 = 0xE7,

  ctrl_hum = 0xF2,
  status = 0xF3,
  ctrl_meas = 0xF4,

  press_msb = 0xF7,
  press_lsb = 0xF8,
  press_xlsb = 0xF9,

  temp_msb = 0xFA,
  temp_lsb = 0xFB,
  temp_xlsb = 0xFC,

};

struct BME280_Result {
  int32_t T;
  uint32_t P;
  uint32_t H;
};

struct BME280_Result16 {
  int16_t T;
  uint16_t P;
  uint16_t H;
};

class BME280 {
public:
  static const uint8_t BME280_ADDRESS1 = 0x76;
  static const uint8_t BME280_ADDRESS2 = 0x77;

  BME280() = default;
  BME280(uint8_t i2cAddress) : i2c_address(i2cAddress){};
  bool begin();
  void singleMeasure() const;
  void waitMeasureDone() const;

  BME280_Result getValues() const;
  BME280_Result16 getValues16() const;

private:
  const uint8_t i2c_address = BME280_ADDRESS1;

  uint8_t read8(BME280Registers bmeregister) const;
  void write8(BME280Registers bmeregister, uint8_t value) const;
  uint16_t read16(BME280Registers bmeregister) const;
  int16_t read16s(BME280Registers bmeregister) const;
  void readBuffer(BME280Registers bmeregister, uint8_t *buffer, uint8_t size) const;

  void readTrimmingParameters();

  int32_t compensate_T(int32_t adc_T) const;
  uint32_t compensate_P(int32_t adc_P, int32_t t_fine) const;
  uint32_t compensate_H(int32_t adc_H, int32_t t_fine) const;

  uint16_t dig_T1;
  int16_t dig_T2;
  int16_t dig_T3;

  uint16_t dig_P1;
  int16_t dig_P2;
  int16_t dig_P3;
  int16_t dig_P4;
  int16_t dig_P5;
  int16_t dig_P6;
  int16_t dig_P7;
  int16_t dig_P8;
  int16_t dig_P9;

  uint8_t dig_H1;
  int16_t dig_H2;
  uint8_t dig_H3;
  int16_t dig_H4;
  int16_t dig_H5;
  int8_t dig_H6;
};

#endif
