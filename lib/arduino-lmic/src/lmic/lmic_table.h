#ifndef _lmic_table_h_
#define _lmic_table_h_

#pragma once
#include "Arduino.h"
#include <stdint.h>

// ======================================================================
// Table support
// These macros for defining a table of constants and retrieving values
// from it makes it easier for other platforms (like AVR) to optimize
// table accesses.
// Use CONST_TABLE() whenever declaring or defining a table, and
// TABLE_GET_xx whenever accessing its values. The actual name of the
// declared variable will be modified to prevent accidental direct
// access. The accessor macros forward to an inline function to allow
// proper type checking of the array element type.

// Helper to add a prefix to the table name
#define RESOLVE_TABLE(table) constant_table_##table

// Accessors for table elements
#define TABLE_GET_U1(table, index) table_get_u1(RESOLVE_TABLE(table), index)
#define TABLE_GET_S1(table, index) table_get_s1(RESOLVE_TABLE(table), index)
#define TABLE_GET_U4(table, index) table_get_u4(RESOLVE_TABLE(table), index)
#define TABLE_GET_S4(table, index) table_get_s4(RESOLVE_TABLE(table), index)
#define TABLE_GET_U1_TWODIM(table, index1, index2)                             \
  table_get_u1(RESOLVE_TABLE(table)[index1], index2)

#if defined(__AVR__)
#include <avr/pgmspace.h>
// Macro to define the getter functions. This loads data from
// progmem using pgm_read_xx, or accesses memory directly when the
// index is a constant so gcc can optimize it away;
#define TABLE_GETTER(postfix, type, pgm_type)                                  \
  inline type table_get##postfix(const type *table, size_t index) {            \
    if (__builtin_constant_p(table[index]))                                    \
      return table[index];                                                     \
    return pgm_read_##pgm_type(&table[index]);                                 \
  }

TABLE_GETTER(_u1, uint8_t, byte);
TABLE_GETTER(_s1, int8_t, byte);
TABLE_GETTER(_u4, uint32_t, dword);
TABLE_GETTER(_s4, int32_t, dword);


// For AVR, store constants in PROGMEM, saving on RAM usage
#define CONST_TABLE(type, name) const type PROGMEM RESOLVE_TABLE(name)

#else
inline uint8_t table_get_u1(const uint8_t *table, size_t index) {
  return table[index];
}
inline int8_t table_get_s1(const int8_t *table, size_t index) {
  return table[index];
}
inline uint32_t table_get_u4(const uint32_t *table, size_t index) {
  return table[index];
}
inline int32_t table_get_s4(const int32_t *table, size_t index) {
  return table[index];
}

// Declare a table
#define CONST_TABLE(type, name) const type RESOLVE_TABLE(name)

#endif

#endif