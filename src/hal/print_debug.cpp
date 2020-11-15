#include "../lmic/config.h"

#include "print_debug.h"

#include <Arduino.h>
#include <stdio.h>

#ifdef ARDUINO_ARCH_AVR
static int uart_putchar(char c, FILE *) {
  LMIC_PRINTF_TO.write(c);
  return 0;
}

void hal_printf_init() {

  // create a FILE structure to reference our UART output function
  static FILE uartout = {};

  // fill in the UART file descriptor with pointer to writer.
  fdev_setup_stream(&uartout, uart_putchar, NULL, _FDEV_SETUP_WRITE);

  // The uart is the standard output device STDOUT.
  stdout = &uartout;
}
#else
void hal_printf_init() {
  // no init for other than AVR
}
#endif // defined(LMIC_PRINTF_TO)
