#ifndef boardconfig_h
#define boardconfig_h

#define LMIC_GENERIC 1
#define LMIC_ARDUINO 2
#define LMIC_ESP32 3

#ifdef ARDUINO


#ifndef LMIC_HAL_IO
#define LMIC_HAL_IO LMIC_ARDUINO
#endif


// ESP32 using Arduino
#ifdef ARDUINO_ARCH_ESP32
#ifndef LMIC_HAL
#define LMIC_HAL LMIC_ESP32
#endif

// Other arduino
#else
#ifndef LMIC_HAL
#define LMIC_HAL LMIC_ARDUINO
#endif

#endif

// not arduino
#else
#ifndef LMIC_HAL
#define LMIC_HAL LMIC_GENERIC
#endif


#ifndef LMIC_HAL_IO
#define LMIC_HAL_IO LMIC_GENERIC
#endif


#endif

#endif