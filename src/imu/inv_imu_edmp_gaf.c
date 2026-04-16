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

#include "imu/inv_imu_edmp.h"
#include "imu/inv_imu_edmp_gaf.h"
#include "imu/inv_imu_edmp_defs.h"
#include "imu/inv_imu_edmp_ram_gaf_defs.h"
#include "imu/inv_imu_edmp_ram_gaf_memmap.h"

#define GYRO_2000DPS_Q30_TO_1DPS_Q16 131072000LL /* 2000dps * (1<<16) */

/* Fits in eDMP memory at EDMP_GAF_STATE_CONFIG address */
typedef struct {
	int32_t acc_fsr;
	int32_t gyr_fsr;

	union {
		uint32_t acc_odr_us;
		uint32_t gyr_odr_us;
	};

	int32_t temp_sensitivity;
	int32_t temp_offset;

	int8_t acc_accuracy;
	int8_t gyr_accuracy;

	int8_t clock_variation;

	union {
		uint32_t acc_pdr_us;
		uint32_t gyr_pdr_us;
	};

	int32_t stationary_angle_enable;
	int32_t stationary_angle_duration_us;
	int32_t stationary_angle_threshold_deg_q16;

	int32_t gyr_cal_stationary_duration_us;
	int32_t gyr_cal_threshold_metric1;
	int32_t gyr_cal_threshold_metric2;

	int32_t fus_high_speed_drift;
	int32_t fus_low_speed_drift_roll_pitch;
	int32_t fus_measurement_covariance_acc;
	int32_t fus_acceleration_rejection;
} internal_gaf_parameters_t;

static int inv_imu_edmp_gaf_do_init(inv_imu_device_t *s);

int inv_imu_edmp_gaf_init(inv_imu_device_t *s)
{
	int            status = INV_IMU_OK;
	uint8_t        value;
	static uint8_t ram_img[] = {
#include "imu/edmp_prgm_gaf.h"
	};

	status |= inv_imu_edmp_init_apex(s);

	/* Clear SRAM data */
	value = 0;
	for (int i = 0; i < RAM_GAF_DATA_SIZE; i++)
		status |= inv_imu_write_sram(s, (uint32_t)RAM_GAF_DATA_BASE + i, 1, &value);

	status |= inv_imu_write_sram(s, (uint32_t)RAM_GAF_PRGM_BASE, sizeof(ram_img), ram_img);

	return status;
}

int inv_imu_edmp_gaf_init_parameters(inv_imu_device_t *s, inv_imu_edmp_gaf_parameters_t *gaf_params)
{
	int status = INV_IMU_OK;

	status |= inv_imu_read_reg(s, SW_PLL1_TRIM, 1, (uint8_t *)&gaf_params->clock_variation);

	gaf_params->pdr_us                             = 10000;
	gaf_params->stationary_angle_enable            = 0;
	gaf_params->stationary_angle_duration_us       = 500000;
	gaf_params->stationary_angle_threshold_deg_q16 = 65536;
	gaf_params->gyr_cal_stationary_duration_us     = 500000;
	gaf_params->gyr_cal_threshold_metric1          = 300;
	gaf_params->gyr_cal_threshold_metric2          = 400;
	gaf_params->fus_high_speed_drift               = 262144;
	gaf_params->fus_low_speed_drift_roll_pitch     = 20;
	gaf_params->fus_measurement_covariance_acc     = 32768;
	gaf_params->fus_acceleration_rejection         = 1073741824;
	/* Identity matrix in Q30 */
	(void)memset(gaf_params->gain_mmatrixA, 0, sizeof(gaf_params->gain_mmatrixA));
	(void)memset(gaf_params->gain_mmatrixG, 0, sizeof(gaf_params->gain_mmatrixG));
	gaf_params->gain_mmatrixA[0] = 1 << 30;
	gaf_params->gain_mmatrixG[0] = 1 << 30;
	gaf_params->gain_mmatrixA[4] = 1 << 30;
	gaf_params->gain_mmatrixG[4] = 1 << 30;
	gaf_params->gain_mmatrixA[8] = 1 << 30;
	gaf_params->gain_mmatrixG[8] = 1 << 30;

	return status;
}

int inv_imu_edmp_gaf_get_parameters(inv_imu_device_t *s, inv_imu_edmp_gaf_parameters_t *gaf_params)
{
	int                       status = INV_IMU_OK;
	internal_gaf_parameters_t params;

	status |= INV_IMU_READ_EDMP_SRAM(s, EDMP_GAF_STATE_CONFIG, (uint8_t *)&params);

	gaf_params->clock_variation                    = params.clock_variation;
	gaf_params->pdr_us                             = params.acc_pdr_us;
	gaf_params->stationary_angle_enable            = params.stationary_angle_enable;
	gaf_params->stationary_angle_duration_us       = params.stationary_angle_duration_us;
	gaf_params->stationary_angle_threshold_deg_q16 = params.stationary_angle_threshold_deg_q16;
	gaf_params->gyr_cal_stationary_duration_us     = params.gyr_cal_stationary_duration_us;
	gaf_params->gyr_cal_threshold_metric1          = params.gyr_cal_threshold_metric1;
	gaf_params->gyr_cal_threshold_metric2          = params.gyr_cal_threshold_metric2;
	gaf_params->fus_high_speed_drift               = params.fus_high_speed_drift;
	gaf_params->fus_low_speed_drift_roll_pitch     = params.fus_low_speed_drift_roll_pitch;
	gaf_params->fus_measurement_covariance_acc     = params.fus_measurement_covariance_acc;
	gaf_params->fus_acceleration_rejection         = params.fus_acceleration_rejection;

	status |= INV_IMU_READ_EDMP_SRAM(s, EDMP_GAF_GAIN_MOUNTING_MATRIX_A,
	                                 (uint8_t *)gaf_params->gain_mmatrixA);
	status |= INV_IMU_READ_EDMP_SRAM(s, EDMP_GAF_GAIN_MOUNTING_MATRIX_G,
	                                 (uint8_t *)gaf_params->gain_mmatrixG);

	return status;
}

int inv_imu_edmp_gaf_set_parameters(inv_imu_device_t *                   s,
                                    const inv_imu_edmp_gaf_parameters_t *gaf_params)
{
	int                       status = INV_IMU_OK;
	internal_gaf_parameters_t params = {
		.acc_fsr                            = 32,
		.gyr_fsr                            = 4000,
		.temp_sensitivity                   = 8388608,
		.temp_offset                        = 1638400,
		.acc_accuracy                       = 0,
		.gyr_accuracy                       = gaf_params->gyr_bias_accuracy,
		.clock_variation                    = gaf_params->clock_variation,
		.acc_pdr_us                         = gaf_params->pdr_us,
		.stationary_angle_enable            = gaf_params->stationary_angle_enable,
		.stationary_angle_duration_us       = gaf_params->stationary_angle_duration_us,
		.stationary_angle_threshold_deg_q16 = gaf_params->stationary_angle_threshold_deg_q16,
		.gyr_cal_stationary_duration_us     = gaf_params->gyr_cal_stationary_duration_us,
		.gyr_cal_threshold_metric1          = gaf_params->gyr_cal_threshold_metric1,
		.gyr_cal_threshold_metric2          = gaf_params->gyr_cal_threshold_metric2,
		.fus_high_speed_drift               = gaf_params->fus_high_speed_drift,
		.fus_low_speed_drift_roll_pitch     = gaf_params->fus_low_speed_drift_roll_pitch,
		.fus_measurement_covariance_acc     = gaf_params->fus_measurement_covariance_acc,
		.fus_acceleration_rejection         = gaf_params->fus_acceleration_rejection
	};

	status |= INV_IMU_WRITE_EDMP_SRAM(s, EDMP_GAF_STATE_CONFIG, (uint8_t *)&params);

	status |= INV_IMU_WRITE_EDMP_SRAM(s, EDMP_GAF_GAIN_MOUNTING_MATRIX_A,
	                                  (uint8_t *)gaf_params->gain_mmatrixA);
	status |= INV_IMU_WRITE_EDMP_SRAM(s, EDMP_GAF_GAIN_MOUNTING_MATRIX_G,
	                                  (uint8_t *)gaf_params->gain_mmatrixG);

	return status;
}

static int inv_imu_edmp_gaf_do_init(inv_imu_device_t *s)
{
	int     status = INV_IMU_OK;
	uint8_t value;
	uint8_t prev_mask_reg;

	/* Mask ISR0 interrupt not to trigger any processing */
	status |= inv_imu_read_reg(s, STATUS_MASK_PIN_0_7, 1, &prev_mask_reg);
	value = 0x3F;
	status |= inv_imu_write_reg(s, STATUS_MASK_PIN_0_7, 1, &value);

	/* Trigger EDMP with on-demand mode */
	status |= inv_imu_edmp_run_ondemand(s, INV_IMU_EDMP_INT0);

	/* Wait 200 us to give enough time for EMDP to start running */
	inv_imu_sleep_us(s, 200);

	/* Wait for DMP execution to complete */
	status |= inv_imu_edmp_wait_for_idle(s);

	status |= inv_imu_edmp_disable(s);

	/* Reset states */
	status |= inv_imu_write_reg(s, STATUS_MASK_PIN_0_7, 1, &prev_mask_reg);

	return status;
}

int inv_imu_edmp_gaf_enable(inv_imu_device_t *s)
{
	int             status   = INV_IMU_OK;
	uint8_t         gaf_init = 0;
	fifo_config0_t  fifo_config0;
	edmp_apex_en1_t edmp_apex_en1;

	/*
	 * Force reinitilization of algorithm because user might have changed GAF parameters between
	 * call	to `inv_imu_init_gaf()` and call to `inv_imu_edmp_gaf_enable()`.
	 * If this is not done, new user parameters won't be applied.
	 */
	status |= INV_IMU_WRITE_EDMP_SRAM(s, EDMP_GAF_STATE_INIT, (uint8_t *)&gaf_init);

	/* Only 1.2k FIFO is supported if GAF RAM image is loaded */
	status |= inv_imu_read_reg(s, FIFO_CONFIG0, 1, (uint8_t *)&fifo_config0);
	if (fifo_config0.fifo_depth > FIFO_CONFIG0_FIFO_DEPTH_GAF)
		return INV_IMU_ERROR;

	/*
	 * Enable RAM feature in EDMP_APEX_EN1 so that ROM executes GAF RAM image.
	 */
	status |= inv_imu_read_reg(s, EDMP_APEX_EN1, 1, (uint8_t *)&edmp_apex_en1);
	edmp_apex_en1.feature3_en = INV_IMU_ENABLE;
	status |= inv_imu_write_reg(s, EDMP_APEX_EN1, 1, (uint8_t *)&edmp_apex_en1);

	/* Trigger the init on edmp side now that everything has been readied */
	status |= inv_imu_edmp_gaf_do_init(s);

	return status;
}

int inv_imu_edmp_gaf_disable(inv_imu_device_t *s)
{
	int             status = INV_IMU_OK;
	edmp_apex_en1_t edmp_apex_en1;

	status |= inv_imu_read_reg(s, EDMP_APEX_EN1, 1, (uint8_t *)&edmp_apex_en1);
	edmp_apex_en1.feature3_en = INV_IMU_DISABLE;
	status |= inv_imu_write_reg(s, EDMP_APEX_EN1, 1, (uint8_t *)&edmp_apex_en1);

	return status;
}

int inv_imu_edmp_gaf_build_outputs(inv_imu_device_t *s, const uint8_t frame_es0[9],
                                   const uint8_t               frame_es1[6],
                                   inv_imu_edmp_gaf_outputs_t *gaf_outputs)
{
	int status = INV_IMU_OK;

	/* Decode quaternion based on ES0 bitfield of FIFO content */
	gaf_outputs->grv_quat_q14[0] =
	    (int16_t)(((int16_t)frame_es0[1] << 8) + ((int16_t)frame_es0[0]));
	gaf_outputs->grv_quat_q14[1] =
	    (int16_t)(((int16_t)frame_es0[3] << 8) + ((int16_t)frame_es0[2]));
	gaf_outputs->grv_quat_q14[2] =
	    (int16_t)(((int16_t)frame_es0[5] << 8) + ((int16_t)frame_es0[4]));
	gaf_outputs->grv_quat_q14[3] =
	    (int16_t)(((int16_t)frame_es0[7] << 8) + ((int16_t)frame_es0[6]));

	/* Decode gyro bias and accuracy flag based on ES1 bitfield of FIFO content */
	gaf_outputs->gyr_bias_q12[0] =
	    (int16_t)(((int16_t)frame_es1[0] << 8) + ((int16_t)frame_es0[8]));
	gaf_outputs->gyr_bias_q12[1] =
	    (int16_t)(((int16_t)frame_es1[2] << 8) + ((int16_t)frame_es1[1]));
	gaf_outputs->gyr_bias_q12[2] =
	    (int16_t)(((int16_t)frame_es1[4] << 8) + ((int16_t)frame_es1[3]));
	gaf_outputs->gyr_accuracy_flag = frame_es1[5] & 0x03;

	/* Decode stationary flag value based on ES1 bitfield of FIFO content */
	switch (frame_es1[5] & 0xfc) {
	case 0xfc:
		gaf_outputs->stationary_flag = -1;
		break;
	case 0x0:
		gaf_outputs->stationary_flag = 0;
		break;
	case 0x4:
		gaf_outputs->stationary_flag = 1;
		break;
	default:
		status = -1;
		break;
	}

	return status;
}

int inv_imu_edmp_gaf_set_biases(inv_imu_device_t *s, const int32_t acc_bias[3],
                                const int32_t gyr_bias[3])
{
	/* Wait for GAF to fully initialise */
	int     status   = INV_IMU_OK;
	uint8_t gaf_init = 0;
	/* Input bias is given in 1dps q16 format and
	 * needs to be converted to 2000dps q30 format
	 * The logic to convert a value, is as follows:
	 *     val_q30 = (val_q16 << 30) / (2000dps << 16)
	 * <=> val_q30 = val_q16 * (2^14/2000)
	 * <=> val_q30 = val_q16 * 8.192
	 * <=> val_q30 = (val_q16 << 30) / (2^30/8.192)
	 */
	const int32_t gyr_bias_edmp[3] = {
		((int64_t)gyr_bias[0] << 30LL) / GYRO_2000DPS_Q30_TO_1DPS_Q16,
		((int64_t)gyr_bias[1] << 30LL) / GYRO_2000DPS_Q30_TO_1DPS_Q16,
		((int64_t)gyr_bias[2] << 30LL) / GYRO_2000DPS_Q30_TO_1DPS_Q16
	};

	/* GAF init must have been successful */
	status |= INV_IMU_READ_EDMP_SRAM(s, EDMP_GAF_STATE_INIT, (uint8_t *)&gaf_init);

	if (0 == gaf_init) {
		status |= INV_IMU_ERROR;
	} else {
		if (gyr_bias) {
			status |= INV_IMU_WRITE_EDMP_SRAM(s, EDMP_GAF_STATE_GYRO_BIAS,
			                                  (const uint8_t *)gyr_bias_edmp);
		}
		if (acc_bias) {
			status |=
			    INV_IMU_WRITE_EDMP_SRAM(s, EDMP_GAF_STATE_ACC_BIAS, (const uint8_t *)acc_bias);
		}
	}
	return status;
}
