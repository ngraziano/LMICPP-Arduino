/*******************************************************************************
 * Copyright (c) 2014-2015 IBM Corporation.
 * All rights reserved. This program and the accompanying materials
 * are made available under the terms of the Eclipse Public License v1.0
 * which accompanies this distribution, and is available at
 * http://www.eclipse.org/legal/epl-v10.html
 *
 * Contributors:
 *    IBM Zurich Research Lab - initial API, implementation and documentation
 *    Nicolas Graziano - cpp style.
 *******************************************************************************/


#include "bufferpack.h"

uint16_t rlsbf2(const uint8_t *const buf) {
  return (uint16_t)((uint16_t)buf[0] | ((uint16_t)buf[1] << 8));
}

uint32_t rlsbf3(const uint8_t *const buf) {
  return (uint32_t)buf[0] | ((uint32_t)buf[1] << 8) | ((uint32_t)buf[2] << 16);
}

uint32_t rlsbf4(const uint8_t *const buf) {
  return (uint32_t)((uint32_t)buf[0] | ((uint32_t)buf[1] << 8) |
                    ((uint32_t)buf[2] << 16) | ((uint32_t)buf[3] << 24));
}

uint16_t rmsbf2(const uint8_t *const buf) {
  return (uint16_t)((uint16_t)buf[1] | ((uint16_t)buf[0] << 8));
}

uint32_t rmsbf4(const uint8_t *const buf) {
  return (uint32_t)((uint32_t)buf[3] | ((uint32_t)buf[2] << 8) |
                    ((uint32_t)buf[1] << 16) | ((uint32_t)buf[0] << 24));
}

void wlsbf2(uint8_t *const buf, uint16_t const v) {
  buf[0] = v;
  buf[1] = v >> 8;
}

void wlsbf4(uint8_t *const buf, uint32_t const v) {
  buf[0] = v;
  buf[1] = v >> 8;
  buf[2] = v >> 16;
  buf[3] = v >> 24;
}

void wmsbf4(uint8_t *const buf, uint32_t const v) {
  buf[3] = v;
  buf[2] = v >> 8;
  buf[1] = v >> 16;
  buf[0] = v >> 24;
}
