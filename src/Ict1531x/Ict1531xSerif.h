/*
 *
 * Copyright (c) [2023] by InvenSense, Inc.
 * 
 * Permission to use, copy, modify, and/or distribute this software for any
 * purpose with or without fee is hereby granted.
 * 
 * THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
 * WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY
 * SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
 * WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN ACTION
 * OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF OR IN
 * CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.
 *
 */

/** @defgroup DriverIct1531xSerif Ict1531x driver serif
 *  @brief Interface for low-level serial (I2C/SPI) access
 *  @ingroup  DriverIct1531x
 *  @{
 */

#ifndef _INV_ICT1531X_SERIF_H_
#define _INV_ICT1531X_SERIF_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "Invn/InvError.h"

#include <stdint.h>

/** @brief Ict1531x serial interface
 */
struct inv_ict1531x_serif {
	void *context;
	int (*read_reg)(void *serif, uint8_t reg, uint8_t *buf, uint32_t len);
	int (*write_reg)(void *serif, uint8_t reg, const uint8_t *buf, uint32_t len);
	uint32_t max_read;
	uint32_t max_write;
	uint8_t  is_first_transaction; /* indicates if the upcoming transaction will be the first one */
};

static inline uint32_t inv_ict1531x_serif_max_read(struct inv_ict1531x_serif *s)
{
	if (!s)
		return INV_ERROR;

	return s->max_read;
}

static inline uint32_t inv_ict1531x_serif_max_write(struct inv_ict1531x_serif *s)
{
	if (!s)
		return INV_ERROR;

	return s->max_write;
}

static inline int inv_ict1531x_serif_read_reg(struct inv_ict1531x_serif *s, uint8_t reg,
                                              uint8_t *buf, uint32_t len)
{
	int rc;

	if (!s)
		return INV_ERROR;

	if (len > s->max_read)
		return INV_ERROR_SIZE;

	rc = s->read_reg(s->context, reg, buf, len);

	/* 
	 * After device power-up, the user may receive NACK for the very first I2C transaction.
	 * The user should perform one retry on the very first I2C transaction if it receives a NACK.
	 */
	if (s->is_first_transaction) {
		s->is_first_transaction = 0;
		if (rc != 0)
			rc = s->read_reg(s->context, reg, buf, len);
	}

	if (rc != 0)
		return INV_ERROR_TRANSPORT;

	return 0;
}

static inline int inv_ict1531x_serif_write_reg(struct inv_ict1531x_serif *s, uint8_t reg,
                                               const uint8_t *buf, uint32_t len)
{
	int rc;

	if (!s)
		return INV_ERROR;

	if (len > s->max_write)
		return INV_ERROR_SIZE;

	rc = s->write_reg(s->context, reg, buf, len);

	/* 
	 * After device power-up, the user may receive NACK for the very first I2C transaction.
	 * The user should perform one retry on the very first I2C transaction if it receives a NACK.
	 */
	if (s->is_first_transaction) {
		s->is_first_transaction = 0;
		if (rc != 0)
			rc = s->write_reg(s->context, reg, buf, len);
	}

	if (rc != 0)
		return INV_ERROR_TRANSPORT;

	return 0;
}

#ifdef __cplusplus
}
#endif

#endif /* _INV_ICT1531X_SERIF_H_ */

/** @} */
