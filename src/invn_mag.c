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

#include "invn_mag.h"

#if defined(ICM45608) || defined(ICM45689)
#include "imu/inv_imu_edmp_mrm.h"
#endif

/* Magnetometer drivers */
#include "Ict1531x/Ict1531x.h"

/*
 * Static functions definition
 */
static int check_end_of_i2c_master_ops(void);
static int mag_read_reg(void *serif, uint8_t reg, uint8_t *buf, uint32_t len);
static int mag_write_reg(void *serif, uint8_t reg, const uint8_t *buf, uint32_t len);

/* I2C configuration for Mag I/O */
#define MAG_I2C_ADDR 0x1E
static inv_ict1531x_t mag_dev; /* Ict1531x driver object */

static inv_imu_device_t *self;

static int check_end_of_i2c_master_ops(void)
{
	int                 status    = 0;
	inv_imu_int_state_t int_state = { 0 };
	i2cm_status_t       i2cm_status;

	while ((!int_state.INV_I2CM_DONE) && (status == 0))
		status |= inv_imu_get_int_status(self, INV_IMU_INT1, &int_state);

	/* Check the status, non-zero means there was an error */
	status |= inv_imu_read_reg(self, I2CM_STATUS, 1, (uint8_t *)&i2cm_status);
	if (i2cm_status.i2cm_busy || i2cm_status.i2cm_timeout_err || i2cm_status.i2cm_srst_err ||
	    i2cm_status.i2cm_scl_err || i2cm_status.i2cm_sda_err)
		return -1;

	return status;
}

static int mag_read_reg(void *serif, uint8_t reg, uint8_t *buf, uint32_t len)
{
	int status = 0;
	/* Configuration to read from reg to buf */
	inv_imu_i2c_master_cfg_t cfg = { .i2c_addr = MAG_I2C_ADDR,
		                             .op_cnt   = 1,
		                             .op       = { { .r_n_w = 1, .reg_addr = reg, .len = len } } };

	(void)serif;

	status |= inv_imu_configure_i2cm(self, &cfg, NULL);
	status |= inv_imu_start_i2cm_ops(self, 1 /* fast_mode */);
	status |= check_end_of_i2c_master_ops();
	status |= inv_imu_get_i2cm_data(self, buf, len);

	return status;
}

static int mag_write_reg(void *serif, uint8_t reg, const uint8_t *buf, uint32_t len)
{
	int status = 0;
	/* Configuration to write buf at reg */
	inv_imu_i2c_master_cfg_t cfg = {
		.i2c_addr = MAG_I2C_ADDR,
		.op_cnt   = 1,
		.op       = { { .r_n_w = 0, .reg_addr = reg, .len = len, .wdata = buf } }
	};

	(void)serif;

	status |= inv_imu_configure_i2cm(self, &cfg, NULL);
	status |= inv_imu_start_i2cm_ops(self, 1 /* fast_mode */);
	status |= check_end_of_i2c_master_ops();

	return status;
}

int invn_read_whoami(inv_imu_device_t *s)
{
  	int                       rc = 0;
	uint8_t                   data;
	struct inv_ict1531x_serif mag_serif;
	inv_imu_int_state_t       int1_config;

	self = s;

	rc |= inv_imu_init_i2cm(s);

    if(rc != INV_IMU_OK)
        return 1;

	/* Enable I2C master done IRQ, to get data from IRQ and inv_imu_get_i2cm_data()
	 * for initial mag configuration, until eDMP is enabled */
	rc |= inv_imu_get_config_int(s, INV_IMU_INT1, &int1_config);
	int1_config.INV_I2CM_DONE = INV_IMU_ENABLE;
	rc |= inv_imu_set_config_int(s, INV_IMU_INT1, &int1_config);

	mag_serif.read_reg  = mag_read_reg;
	mag_serif.write_reg = mag_write_reg;
	mag_serif.max_read  = 21;
	mag_serif.max_write = 6;

    if(rc != INV_IMU_OK)
        return 2;

	/* Reset ict1531x driver states */
	inv_ict1531x_reset_states(&mag_dev, &mag_serif);

	/* Init ict1531x device */
	rc = inv_ict1531x_soft_reset(&mag_dev);
	if (rc != INV_ERROR_SUCCESS) {
		return 3;
	}

	rc = inv_ict1531x_get_whoami(&mag_dev, &data);
	if (rc != INV_ERROR_SUCCESS) {
		return rc;
	}

    return data;
}


int invn_mag_enable(int flag)
{
	int rc = 0;

	rc |= inv_ict1531x_enable_sensor(&mag_dev, flag);

	return rc;
}


int invn_mag_init(inv_imu_device_t *s)
{
	int                       rc = 0;
	uint8_t                   data;
	struct inv_ict1531x_serif mag_serif;
	inv_imu_int_state_t       int1_config;

	self = s;

    rc |= inv_imu_init_i2cm(s);

	/* Enable I2C master done IRQ, to get data from IRQ and inv_imu_get_i2cm_data()
	 * for initial mag configuration, until eDMP is enabled */
	rc |= inv_imu_get_config_int(s, INV_IMU_INT1, &int1_config);
	int1_config.INV_I2CM_DONE = INV_IMU_ENABLE;
	rc |= inv_imu_set_config_int(s, INV_IMU_INT1, &int1_config);

	mag_serif.read_reg  = mag_read_reg;
	mag_serif.write_reg = mag_write_reg;
	mag_serif.max_read  = 21;
	mag_serif.max_write = 6;

	/* Reset ict1531x driver states */
	inv_ict1531x_reset_states(&mag_dev, &mag_serif);

	/* Init ict1531x device */
	rc = inv_ict1531x_soft_reset(&mag_dev);
	if (rc != INV_ERROR_SUCCESS) {
		return rc;
	}

	rc = inv_ict1531x_get_whoami(&mag_dev, &data);
	if (rc != INV_ERROR_SUCCESS) {
		return rc;
	}

	if (data != ICT1531X_WHOAMI) {
		return INV_ERROR;
	}

	/* Read ICT1531X status register so that latest IMU I2CM transaction is a read in case EDMP
	 * needs to start with a I2CM_RDATA content analysis expecting ICT1531X status bit
	 */
	rc |= mag_serif.read_reg(&mag_dev.serif, ICT1531X_STATUS_REG, &data, 1);

	/*
	 * Disable I2C master done IRQ, ES data will be read automatically by eDMP
	 * and host will get them by FIFO from now on.
	 */
	rc |= inv_imu_get_config_int(s, INV_IMU_INT1, &int1_config);
	int1_config.INV_I2CM_DONE = INV_IMU_DISABLE;
	rc |= inv_imu_set_config_int(s, INV_IMU_INT1, &int1_config);
	{
		/* I2CM device profile[2] = [Read addr, deviceID] */
		const uint8_t i2cm_dev_prof[2] = { ICT1531X_STATUS_REG, MAG_I2C_ADDR };
		/* I2CM write buffer[4] is initialized with 2 couples of write_addr/data */
		const uint8_t i2cm_wrb[4] = { ICT1531X_MODE_CTRL_REG,
			                          ICT1531X_MODE_CTRL_REG_MODE_SINGLE_SHOT,
			                          ICT1531X_MODE_CTRL_REG, ICT1531X_MODE_CTRL_REG_MODE_STANDBY };
		/* I2CM cmd buffer[1] is programmed for a 2 bytes write, last operation (I2C transfer is
		 * stopped after it).
		 * note: I2CM cmd buffer[0] is handled by eDMP FW.
		 */
		const uint8_t i2cm_cmd1 = 0x82;

		inv_imu_write_reg(s, I2CM_DEV_PROFILE0, sizeof(i2cm_dev_prof), i2cm_dev_prof);
		inv_imu_write_reg(s, I2CM_WR_DATA0, sizeof(i2cm_wrb), i2cm_wrb);
		inv_imu_write_reg(s, I2CM_COMMAND_1, sizeof(i2cm_cmd1), &i2cm_cmd1);
	}
	return rc;
}

#if defined(ICM45608) || defined(ICM45689)
int invn_mag_load_ram_image(inv_imu_device_t *s, invn_mag_usecase_t usecase)
{
	int rc = 0;

	if (usecase == INVN_MAG_USECASE_IMG_OVER_SIF)
		rc |= inv_imu_edmp_mrm_init(s, INV_IMU_EDMP_MRM_INIT_OVER_SIF);
	else
		rc = INV_ERROR_BAD_ARG;

	return rc;
}

int invn_mag_run_mrm(inv_imu_device_t *s)
{
	int rc = 0;

	rc |= inv_imu_edmp_mrm_request(s);

	return rc;
}

int invn_mag_enable_automrm(inv_imu_device_t *s)
{
	int rc = 0;

	rc |= inv_imu_edmp_mrm_enable_auto(s);

	return rc;
}

int invn_mag_disable_automrm(inv_imu_device_t *s)
{
	int rc = 0;

	rc |= inv_imu_edmp_mrm_disable_auto(s);

	return rc;
}
#endif /* defined(ICM45608) || defined(ICM45689) */

uint64_t inv_ict1531x_get_time_us(void)
{
	return inv_imu_get_time_us();
}
