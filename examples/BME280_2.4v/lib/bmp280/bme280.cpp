#include "bme280.h"
#include <Wire.h>

uint8_t BME280::read8(BME280Registers const reg) const {
  Wire.beginTransmission(i2c_address);
  Wire.write(static_cast<uint8_t>(reg));
  Wire.endTransmission();
  Wire.requestFrom(i2c_address, static_cast<uint8_t>(1));
  return Wire.read();
}

void BME280::write8(BME280Registers const reg, uint8_t const value) const {
  Wire.beginTransmission(i2c_address);
  Wire.write(static_cast<uint8_t>(reg));
  Wire.write(value);
  Wire.endTransmission();
}

uint16_t BME280::read16(BME280Registers const reg) const {
  Wire.beginTransmission(i2c_address);
  Wire.write(static_cast<uint8_t>(reg));
  Wire.endTransmission();
  Wire.requestFrom(i2c_address, static_cast<uint8_t>(2));
  return Wire.read() | (Wire.read() << 8);
}

int16_t BME280::read16s(BME280Registers const reg) const {
  return static_cast<int16_t>(read16(reg));
}

void BME280::readBuffer(BME280Registers const reg, uint8_t *buffer,
                        uint8_t const size) const {
  Wire.beginTransmission(i2c_address);
  Wire.write(static_cast<uint8_t>(reg));
  Wire.endTransmission();
  Wire.requestFrom(i2c_address, static_cast<uint8_t>(size));

  for (uint8_t i = 0; i < size; i++) {
    buffer[i] = static_cast<uint8_t>(Wire.read());
  }
}

bool BME280::begin() {

  if (read8(BME280Registers::chipid) != 0x60)
    return false;

  readTrimmingParameters();

  // write8(BME280Registers::ctrl_hum, 0x3F);
  // write8(BME280Registers::ctrl_meas, 0x3F);
  return true;
}

void BME280::singleMeasure() const {

  write8(BME280Registers::ctrl_hum, 0x01);
  write8(BME280Registers::ctrl_meas, (1 << 5) | (1 << 2) | 1);
}

void BME280::waitMeasureDone() const {
  bool measuring;
  do {
    auto status = read8(BME280Registers::status);
    measuring = (status & (1 << 3)) != 0;
  } while (measuring);
}

// Retrieve calibration data from device:
void BME280::readTrimmingParameters() {
  dig_T1 = read16(BME280Registers::dig_T1);
  dig_T2 = read16s(BME280Registers::dig_T2);
  dig_T3 = read16s(BME280Registers::dig_T3);

  dig_P1 = read16(BME280Registers::dig_P1);
  dig_P2 = read16s(BME280Registers::dig_P2);
  dig_P3 = read16s(BME280Registers::dig_P3);
  dig_P4 = read16s(BME280Registers::dig_P4);
  dig_P5 = read16s(BME280Registers::dig_P5);
  dig_P6 = read16s(BME280Registers::dig_P6);
  dig_P7 = read16s(BME280Registers::dig_P7);
  dig_P8 = read16s(BME280Registers::dig_P8);
  dig_P9 = read16s(BME280Registers::dig_P9);

  dig_H1 = read8(BME280Registers::dig_H1);
  dig_H2 = read16s(BME280Registers::dig_H2);
  dig_H3 = read8(BME280Registers::dig_H3);

  auto const tmp1 = read8(BME280Registers::dig_H4);
  auto const tmp2 = read8(BME280Registers::dig_H45);
  auto const tmp3 = read8(BME280Registers::dig_H5);

  dig_H4 = (tmp1 << 8) >> 4 | (tmp2 & 0x0F);
  dig_H5 = (tmp3 << 8) >> 4 | (tmp2 >> 4);

  dig_H6 = read8(BME280Registers::dig_H6);
}

int32_t BME280::compensate_T(int32_t adc_T) const {
  int32_t var1 =
      ((((adc_T >> 3) - ((int32_t)dig_T1 << 1))) * ((int32_t)dig_T2)) >> 11;
  int32_t var2 = (((((adc_T >> 4) - ((int32_t)dig_T1)) *
                    ((adc_T >> 4) - ((int32_t)dig_T1))) >>
                   12) *
                  ((int32_t)dig_T3)) >>
                 14;
  return var1 + var2;
}

uint32_t BME280::compensate_P(int32_t adc_P, int32_t t_fine) const {
  int64_t var1 = ((int64_t)t_fine) - 128000;
  int64_t var2 = var1 * var1 * (int64_t)dig_P6;
  var2 = var2 + ((var1 * (int64_t)dig_P5) << 17);
  var2 = var2 + (((int64_t)dig_P4) << 35);
  var1 =
      ((var1 * var1 * (int64_t)dig_P3) >> 8) + ((var1 * (int64_t)dig_P2) << 12);
  var1 = (((((int64_t)1) << 47) + var1)) * ((int64_t)dig_P1) >> 33;
  if (var1 == 0) {
    return 0; // avoid exception caused by division by zero
  }
  int64_t p = 1048576 - adc_P;
  p = (((p << 31) - var2) * 3125) / var1;
  var1 = (((int64_t)dig_P9) * (p >> 13) * (p >> 13)) >> 25;
  var2 = (((int64_t)dig_P8) * p) >> 19;
  p = ((p + var1 + var2) >> 8) + (((int64_t)dig_P7) << 4);
  return (uint32_t)p;
}

uint32_t BME280::compensate_H(int32_t adc_H, int32_t t_fine) const {
  int32_t v_x1_u32r;
  v_x1_u32r = (t_fine - ((int32_t)76800));
  v_x1_u32r =
      (((((adc_H << 14) - (((int32_t)dig_H4) << 20) -
          (((int32_t)dig_H5) * v_x1_u32r)) +
         ((int32_t)16384)) >>
        15) *
       (((((((v_x1_u32r * ((int32_t)dig_H6)) >> 10) *
            (((v_x1_u32r * ((int32_t)dig_H3)) >> 11) + ((int32_t)32768))) >>
           10) +
          ((int32_t)2097152)) *
             ((int32_t)dig_H2) +
         8192) >>
        14));
  v_x1_u32r =
      (v_x1_u32r -
       (((((v_x1_u32r >> 15) * (v_x1_u32r >> 15)) >> 7) * ((int32_t)dig_H1)) >>
        4));
  v_x1_u32r = (v_x1_u32r < 0 ? 0 : v_x1_u32r);
  v_x1_u32r = (v_x1_u32r > 419430400 ? 419430400 : v_x1_u32r);
  return (uint32_t)(v_x1_u32r >> 12);
}

BME280_Result BME280::getValues() const {
  uint8_t rawbuffer[8];

  // read on one pass to have consistant value
  readBuffer(BME280Registers::press_msb, rawbuffer, 8);

  int32_t adValueTemp = rawbuffer[3];
  adValueTemp <<= 8;
  adValueTemp |= rawbuffer[4];
  adValueTemp <<= 4;
  adValueTemp |= rawbuffer[5] >> 4;

  auto const t_fine = compensate_T(adValueTemp);
  // Temperature in DegC, resolution is 0.01 DegC. Output value of “5123”
  // equals 51.23 DegC.
  int32_t const T = (t_fine * 5 + 128) >> 8;

  int32_t adValuePress = rawbuffer[0];
  adValuePress <<= 8;
  adValuePress |= rawbuffer[1];
  adValuePress <<= 4;
  adValuePress |= rawbuffer[2] >> 4;

  // Returns pressure in Pa as unsigned 32 bit integer in Q24.8 format (24
  // integer bits and 8 fractional bits). Output value of “24674867” represents
  // 24674867/256 = 96386.2 Pa = 963.862 hPa
  uint32_t const P = compensate_P(adValuePress, t_fine);

  int32_t adValueHum = rawbuffer[6];
  adValueHum <<= 8;
  adValuePress |= rawbuffer[7];

  // Returns humidity in %RH as unsigned 32 bit integer in Q22.10 format (22
  // integer and 10 fractional bits). Output value of “47445” represents
  // 47445/1024 = 46.333 %RH
  uint32_t const H = compensate_H(adValueHum, t_fine);

  return {T, P, H};
}

BME280_Result16 BME280::getValues16() const {
  uint8_t rawbuffer[8];

  // read on one pass to have consistant value
  readBuffer(BME280Registers::press_msb, rawbuffer, 8);

  int32_t adValueTemp = rawbuffer[3];
  adValueTemp <<= 8;
  adValueTemp |= rawbuffer[4];
  adValueTemp <<= 4;
  adValueTemp |= rawbuffer[5] >> 4;

  auto const t_fine = compensate_T(adValueTemp);

  // temperature range -40 +85  (16bit signed => -327 / +327 °C)
  // Temperature in DegC, resolution is 0.01 DegC. Output value of “5123”
  // equals 51.23 DegC.
  int16_t const T = static_cast<int16_t>((t_fine * 5 + 128) >> 8);

  int32_t adValuePress = rawbuffer[0];
  adValuePress <<= 8;
  adValuePress |= rawbuffer[1];
  adValuePress <<= 4;
  adValuePress |= rawbuffer[2] >> 4;

  // Absolute rating 300 -> 1100 hPa
  // Returns pressure in Pa as unsigned 16 bit integer
  // 1 = 1/50 hPa
  auto const P = static_cast<uint16_t>(compensate_P(adValuePress, t_fine) >> 9);

  int32_t adValueHum = rawbuffer[6];
  adValueHum <<= 8;
  adValuePress |= rawbuffer[7];

  // Returns humidity in %RH as unsigned 16 bit
  // 1 => 1/256 %RH
  auto const H = static_cast<uint16_t>(compensate_H(adValueHum, t_fine) >> 2);

  return {T, P, H};
}
