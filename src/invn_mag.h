/*
 *
 * Copyright (c) [2020] by InvenSense, Inc.
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

#ifndef __INVN_MAG_H__
#define __INVN_MAG_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "Invn/InvError.h"

/* Driver */
#include "imu/inv_imu_driver.h"
#include "imu/inv_imu_i2cm.h"


/** @brief System usecase definition for Magnetometer initialization */
typedef enum { INVN_MAG_USECASE_IMG_OVER_SIF } invn_mag_usecase_t;

int invn_read_whoami(inv_imu_device_t *s);

/**
 * \brief Initialize and configure appropriately IMU I2CM for targeted magnetometer usage and check magnetometer is connected to I2CM IOs.
 *  @param[in] s   Pointer to IMU device.
 *  @return        0 on success, negative value on error.
 */
int invn_mag_init(inv_imu_device_t *s);

/**
 * \brief Load magnetometer-related EDMP image in IMU RAM.
 *  @param[in] s       Pointer to IMU device.
 *  @param[in] usecase Usecase for which RAM image is to be loaded, refer to @sa invn_mag_usecase_t.
 *  @return            0 on success, negative value on error.
 */
int invn_mag_load_ram_image(inv_imu_device_t *s, invn_mag_usecase_t usecase);

/**
 * \brief Execute MRM once.
 *  @param[in] s       Pointer to IMU device.
 *  @return            0 on success, negative value on error.
 */
int invn_mag_run_mrm(inv_imu_device_t *s);

/**
 * \brief Enable automatic MRM.
 *  @param[in] s       Pointer to IMU device.
 *  @return            0 on success, negative value on error.
 */
int invn_mag_enable_automrm(inv_imu_device_t *s);

/**
 * \brief Disable automatic MRM.
 *  @param[in] s       Pointer to IMU device.
 *  @return            0 on success, negative value on error.
 */
int invn_mag_disable_automrm(inv_imu_device_t *s);

int invn_mag_enable(int flag);


#ifdef __cplusplus
}
#endif

#endif /* !__INVN_MAG_H__ */
