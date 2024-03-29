#ifndef _lmic_config_h_
#define _lmic_config_h_

// In the original LMIC code, these config values were defined on the
// gcc commandline. Since Arduino does not allow easily modifying the
// compiler commandline, use this file instead.

#ifdef LMIC_104_EXPERIMENTAL
constexpr bool lorawan_v104 = true;
#else
constexpr bool lorawan_v104 = false;
#endif

// 16 μs per tick
// LMIC requires ticks to be 15.5μs - 100 μs long
#define US_PER_OSTICK_EXPONENT 4
#define US_PER_OSTICK (1 << US_PER_OSTICK_EXPONENT)
#define OSTICKS_PER_SEC (1000000 / US_PER_OSTICK)

// Set this to 1 to enable some basic debug output (using printf) about
// RF settings used during transmission and reception. Set to 2 to
// enable more verbose output. Make sure that printf is actually
// configured (e.g. on AVR it is not by default), otherwise using it can
// cause crashing.

#ifdef LMIC_DEBUG_LEVEL
constexpr int debugLevel = LMIC_DEBUG_LEVEL;
#else
constexpr int debugLevel = 1;
#endif

// Define time to prepare radio for RX in ms
// depend on MCU
#ifndef LMIC_RX_RAMPUP_MS
#define LMIC_RX_RAMPUP_MS 40000
#endif

// Define time to prepare radio for TX in ms
// depend on MCU
#ifndef LMIC_TX_RAMPUP_MS
#define LMIC_TX_RAMPUP_MS 2000
#endif

// Enable this to allow using printf() to print to the given serial port
// (or any other Print object).
#ifndef LMIC_PRINTF_TO
#define LMIC_PRINTF_TO Serial
#endif

#ifndef LMIC_MAX_BUFFER_LENGTH
#define LMIC_MAX_BUFFER_LENGTH 64
#endif

// Any runtime assertion failures are printed to this serial port (or
// any other Print object). If this is unset, any failures just silently
// halt execution.
// #define LMIC_FAILURE_TO Serial

// In LoRaWAN, a gateway applies I/Q inversion on TX, and nodes do the
// same on RX. This ensures that gateways can talk to nodes and vice
// versa, but gateways will not hear other gateways and nodes will not
// hear other nodes. By uncommenting this macro, this inversion is
// disabled and this node can hear other nodes. If two nodes both have
// this macro set, they can talk to each other (but they can no longer
// hear gateways). This should probably only be used when debugging
// and/or when talking to the radio directly (e.g. like in the "raw"
// example).
//#define DISABLE_INVERT_IQ_ON_RX

#define CFG_noassert

#endif // _lmic_config_h_
