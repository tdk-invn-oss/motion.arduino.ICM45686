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

/** @defgroup DriverIct1531x Ict1531x driver
 *  @brief    Low-level driver for Ict1531x devices
 *  @ingroup  Drivers
 *  @{
 */

#ifndef _INV_ICT1531X_H_
#define _INV_ICT1531X_H_

#include "Ict1531xSerif.h"

#ifdef __cplusplus
extern "C" {
#endif

#define ICT1531X_WHOAMI 0x45 /* Chip ID */

/* Register definitions */
#define ICT1531X_MANUF_ID_REG 0x00 /* Manufacturer ID. Reset value = 0xe7 */
#define ICT1531X_CHIP_ID_REG  0x01 /* Chip ID. Reset value = 0x45 */

#define ICT1531X_CHIP_CONFIG_REG 0x03 /* Chip Config */
#define ICT1531X_CHIP_CONFIG_REG_TEMP_SEL_MASK 0x08
#define ICT1531X_CHIP_CONFIG_REG_TEMP_SEL_POS  3

#define ICT1531X_MODE_CTRL_REG           0x04 /* Mode Control */
#define ICT1531X_MODE_CTRL_REG_ODR_MASK  0x70
#define ICT1531X_MODE_CTRL_REG_ODR_POS   4
#define ICT1531X_MODE_CTRL_REG_MODE_MASK 0x03
#define ICT1531X_MODE_CTRL_REG_MODE_POS  0

#define ICT1531X_MODE_STATUS_REG             0x05 /* Mode Status */
#define ICT1531X_MODE_STATUS_REG_STATUS_MASK 0x03
#define ICT1531X_MODE_STATUS_REG_STATUS_POS  0

#define ICT1531X_STATUS_REG                 0x06 /* Status Register */
#define ICT1531X_STATUS_REG_DATA_READY_MASK 0x01
#define ICT1531X_STATUS_REG_DATA_READY_POS  0

#define ICT1531X_FRAME_CNT_REG                    0x07 /* Frame Counter */
#define ICT1531X_FRAME_CNT_REG_FRAME_COUNTER_MASK 0x07
#define ICT1531X_FRAME_CNT_REG_FRAME_COUNTER_POS  0

#define ICT1531X_TEMP_DATA_LSB  0x08 /* Temp data LSB */
#define ICT1531X_TEMP_DATA_MSB  0x09 /* Temp data MSB */
#define ICT1531X_MAG_DATA_X_LSB 0x0a /* Mag X axis LSB */
#define ICT1531X_MAG_DATA_X_MSB 0x0b /* Mag X axis MSB */
#define ICT1531X_MAG_DATA_Y_LSB 0x0c /* Mag Y axis LSB */
#define ICT1531X_MAG_DATA_Y_MSB 0x0d /* Mag Y axis MSB */
#define ICT1531X_MAG_DATA_Z_LSB 0x0e /* Mag Z axis LSB */
#define ICT1531X_MAG_DATA_Z_MSB 0x0f /* Mag Z axis MSB */

#define ICT1531X_CONF_CALIB_REG                     0x21
#define ICT1531X_SEQUENCER_CTRL_REG                 0x7c
#define ICT1531X_SEQUENCER_CTRL_REG_SOFT_RESET_MASK 0x80
#define ICT1531X_SEQUENCER_CTRL_REG_SOFT_RESET_POS  7
#define ICT1531X_GLOBAL_LOCK_REG                    0x7f

/** @brief mode
 *  @details Mode selection
 */
typedef enum {
	ICT1531X_MODE_CTRL_REG_MODE_STANDBY     = 0,
	ICT1531X_MODE_CTRL_REG_MODE_PULSED      = 1,
	ICT1531X_MODE_CTRL_REG_MODE_SINGLE_SHOT = 2,
	ICT1531X_MODE_CTRL_REG_MODE_MRM         = 3
} inv_ict1531x_mode_t;

/** @brief mode_status
 *  @details Mode status
 */
typedef enum {
	ICT1531X_MODE_STATUS_REG_MODE_STANDBY     = 0,
	ICT1531X_MODE_STATUS_REG_MODE_PULSED      = 1,
	ICT1531X_MODE_STATUS_REG_MODE_SINGLE_SHOT = 2,
	ICT1531X_MODE_STATUS_REG_MODE_MRM         = 3
} inv_ict1531x_mode_status_t;


typedef enum {
	ICT1531X_CHIP_CONFIG_REG_TEMP_SEL_FILTERED = 0 << ICT1531X_CHIP_CONFIG_REG_TEMP_SEL_POS,
	ICT1531X_CHIP_CONFIG_REG_TEMP_SEL_RAW      = 1 << ICT1531X_CHIP_CONFIG_REG_TEMP_SEL_POS
} inv_ict1531x_chip_config_temp_sel_t;

/** @brief Defines how many samples we collect in each window for self-test */
#define NUMBER_OF_SAMPLES_FOR_SELFTEST 100

/** @brief Error cases for self-test */
#define ICT1531X_SELFTEST_SUCCESS        0
#define ICT1531X_SELFTEST_ERROR_PP_WIN_X (1 << 0)
#define ICT1531X_SELFTEST_ERROR_PP_WIN_Y (1 << 1)
#define ICT1531X_SELFTEST_ERROR_PP_WIN_Z (1 << 2)

/** @brief ICT1531X driver states definition
 */
typedef struct inv_ict1531x_selftest_status {
	/* Overall self-test status */
	int status;

	/* Stats */
	int16_t max[3];
	int16_t min[3];
	int32_t pp[3];

} inv_ict1531x_selftest_status_t;

/** @brief ICT1531X driver states definition
 */
typedef struct inv_ict1531x {
	struct inv_ict1531x_serif serif;
	uint8_t                   compass_en;
	int                       frame_cnt;
} inv_ict1531x_t;

/** @brief Hook for low-level system time() function to be implemented by upper layer
 *  @return monotonic timestamp in us
 *  @details
 *  When running self-tests, this function is used to measure a duration.
 *  It can also be used as a way of getting the current time.
 */
extern uint64_t inv_ict1531x_get_time_us(void);

/** @brief Reset and initialize driver states
 *  @param[in] s handle to driver states structure
 *  @param[in] serif handle to SERIF object for underlying register access
 */
void inv_ict1531x_reset_states(struct inv_ict1531x *s, const struct inv_ict1531x_serif *serif);

/** @brief Check and retrieve for new data
 *  @param[out] compass_data_lsb raw compass data
 *  @param[out] temp_data_lsb raw temperature data
 *  @return     0 on success, negative value on error
 */
int inv_ict1531x_poll_data(struct inv_ict1531x *s, int16_t *compass_data_lsb,
                           int16_t *temp_data_lsb);

/** @brief return WHOAMI value
 *  @param[out] whoami WHOAMI for device
 *  @return     0 on success, negative value on error
 */
int inv_ict1531x_get_whoami(struct inv_ict1531x *s, uint8_t *whoami);

/** @brief Perform a soft reset of the device
 *  @return 0 on success, negative value on error.
 */
int inv_ict1531x_soft_reset(struct inv_ict1531x *s);

/** @brief Set the compass to different mode
 * @param[in] mode one of @sa inv_ict1531x_mode_t
 * @return 0 in case of success, negative value on error
 */
int inv_ict1531x_set_mode(struct inv_ict1531x *s, inv_ict1531x_mode_t mode);

/** @brief return current mode status
 *  @param[out] cur_mode current mode, one of @sa inv_ict1531x_mode_t
 *  @return     0 on success, negative value on error
 */
int inv_ict1531x_get_mode(struct inv_ict1531x *s, inv_ict1531x_mode_t *cur_mode);

/** @brief Set temperature mode.
 *         Must be called when the sensor is in Standby/Sleep mode.
 *  @param[in] temp_mode Mode to apply to temperature sensor.
 *  @return    0 on success, negative value on error
 */
int inv_ict1531x_set_temperature_mode(struct inv_ict1531x *s,
                                      inv_ict1531x_chip_config_temp_sel_t temp_mode);

/** @brief Enables / disables the compass sensor
 * @param[in] enable			0=off, 1=on
 * @return 0 in case of success, negative value on error
 */
int inv_ict1531x_enable_sensor(struct inv_ict1531x *s, uint8_t en);

/** @brief Trigger magnetic reset mode
 * @return 0 in case of success, negative value on error
 */
int inv_ict1531x_set_mrm(struct inv_ict1531x *s);

/** @brief Lock protected registers
 * @param[in] lock			0=unlock, 1=lock
 * @return 0 in case of success, negative value on error
 */
int inv_ict1531x_global_lock(struct inv_ict1531x *s, uint8_t lock);

/** @brief Perform self-test
 * @param[in] s           Pointer to device.
 * @param[out] st_status  Status of the self-test (see `ICT1531X_SELFTEST_*` defines).
 * @return 0 in case of success, negative value on error.
 * @note The return code doesn't indicate the status of the self-test. It indicates if the
 *       function completed its execution or not. The self-test status should be read in 
 *       `st_status` if the function returned 0.
 */
int inv_ict1531x_selftest(struct inv_ict1531x *s, inv_ict1531x_selftest_status_t *st_status);

#ifdef __cplusplus
}
#endif

#endif /* _INV_ICT1531X_H_ */

/** @} */
