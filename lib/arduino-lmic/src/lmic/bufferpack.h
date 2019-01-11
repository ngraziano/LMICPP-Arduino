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

#ifndef __bufferpack_h__
#define __bufferpack_h__

#include <stdint.h>

//! Read 24-bit quantity from given pointer in little endian byte order (but in uint32_t).
uint32_t rlsbf3(const uint8_t *buf);
//! Read 32-bit quantity from given pointer in little endian byte order.
uint32_t rlsbf4(const uint8_t *buf);
//! Write 32-bit quntity into buffer in little endian byte order.
void wlsbf4(uint8_t *buf, uint32_t value);
//! Read 32-bit quantity from given pointer in big endian byte order.
uint32_t rmsbf4(const uint8_t *buf);
//! Write 32-bit quntity into buffer in big endian byte order.
void wmsbf4(uint8_t *buf, uint32_t value);
//! Read 16-bit quantity from given pointer in little endian byte order.
uint16_t rlsbf2(const uint8_t *buf);
//! Write 16-bit quntity into buffer in little endian byte order.
void wlsbf2(uint8_t *buf, uint16_t value);


#endif // __bufferpack_h__