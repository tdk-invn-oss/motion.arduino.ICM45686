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
 
#include "Arduino.h"
#include "ICM45686.h"
#if (INV_DEVICE_TYPE == INV_TYPE_A2)
#include "imu/inv_imu_edmp_gaf.h"
#else
#include "invn_mag.h"
#include "Ict1531x/Ict1531x.h"
/* 
 * Soft-iron matrix applied to mag data in EDMP
 * Align mag axis to IMU
 */
static int32_t soft_iron_matrix[3][3] = {
	{ (1 << 30), 0, 0 },
	{ 0, (1 << 30), 0 },
	{ 0, 0, (1 << 30) },
};

#endif

#if (INV_DEVICE_TYPE == INV_TYPE_A1)
#include "imu/inv_imu_edmp_compass.h"
#include "imu/inv_imu_edmp_wearable.h"
#endif

#if (INV_DEVICE_TYPE == INV_TYPE_C1)
#include "imu/inv_imu_edmp_extended_features.h"
#endif

static int i2c_write(uint8_t reg, const uint8_t * wbuffer, uint32_t wlen);
static int i2c_read(uint8_t reg, uint8_t * rbuffer, uint32_t rlen);
static int spi_write(uint8_t reg, const uint8_t * wbuffer, uint32_t wlen);
static int spi_read(uint8_t reg, uint8_t * rbuffer, uint32_t rlen);
static void sleep_us(uint32_t us);
static void sensor_event_cb(inv_imu_sensor_event_t *event);

// i2c and SPI interfaces are used from C driver callbacks, without any knowledge of the object
// As they are declared as static, they will be overriden each time a new ICM456xx object is created
// i2c
uint8_t i2c_address = 0;
static TwoWire *i2c = NULL;
#define I2C_DEFAULT_CLOCK 400000
#define I2C_MAX_CLOCK 1000000
#define ICM456xx_I2C_ADDRESS 0x68
#define ARDUINO_I2C_BUFFER_LENGTH 32
// spi
static SPIClass *spi = NULL;
static uint8_t chip_select_id = 0;
#define SPI_READ 0x80
#define SPI_DEFAULT_CLOCK 6000000
#define SPI_MAX_CLOCK 24000000

// i2c/spi clock frequency
static uint32_t clk_freq = 0;

bool I2CM_EXAM = false;
int32_t accel_data[3];
int32_t gyro_data[3];
float mag_data[3];

/*
 * WOM threshold value in mg.
 * 1g/256 resolution (wom_th = mg * 256 / 1000)
 */
#define DEFAULT_WOM_THS_MG     (52 >> 2) // 52 mg
#define DEFAULT_WOM_AID_THS_MG (24 >> 2) // 24 mg

#define Q11_DIV (1<<11)
#define Q14_DIV (1<<14)
#define Q16_DIV (1<<16)
#define Q30_DIV (1<<30)

#define RAW_MAG_SCALE 0.075f

#if (INV_DEVICE_TYPE == INV_TYPE_A2) || (INV_DEVICE_TYPE == INV_TYPE_B1) || (INV_DEVICE_TYPE == INV_TYPE_C1)
// GAF output aggregartion
static inv_imu_edmp_gaf_outputs_t gaf_outputs_internal = {0};
static int gaf_status = 0;
#endif

// This is used by the getDataFromFifo callback (not object aware), declared static
static inv_imu_device_t *icm_driver_ptr = NULL;

// ICM456xx constructor for I2c interface
ICM456xx::ICM456xx(TwoWire &i2c_ref,bool lsb, uint32_t freq) {
  i2c = &i2c_ref; 
  i2c_address = ICM456xx_I2C_ADDRESS | (lsb ? 0x1 : 0);
  if ((freq <= I2C_MAX_CLOCK) && (freq >= 100000))
  {
    clk_freq = freq;
  } else {
    clk_freq = I2C_DEFAULT_CLOCK;
  }
}

// ICM456xx constructor for I2c interface, default frequency
ICM456xx::ICM456xx(TwoWire &i2c_ref,bool lsb) {
  i2c = &i2c_ref; 
  i2c_address = ICM456xx_I2C_ADDRESS | (lsb ? 0x1 : 0);
  clk_freq = I2C_DEFAULT_CLOCK;
}

// ICM456xx constructor for spi interface
ICM456xx::ICM456xx(SPIClass &spi_ref,uint8_t cs_id, uint32_t freq) {
  spi = &spi_ref;
  chip_select_id = cs_id; 
  if ((freq <= SPI_MAX_CLOCK) && (freq >= 100000))
  {
    clk_freq = freq;
  } else {
    clk_freq = SPI_DEFAULT_CLOCK;
  }
}

// ICM456xx constructor for spi interface, default frequency
ICM456xx::ICM456xx(SPIClass &spi_ref,uint8_t cs_id) {
  spi = &spi_ref;
  chip_select_id = cs_id; 
  clk_freq = SPI_DEFAULT_CLOCK;
}

/* starts communication with the ICM456xx */
int ICM456xx::begin() {
  int rc = 0;
  uint8_t who_am_i;

  if (i2c != NULL) {
    i2c->begin();
    i2c->setClock(clk_freq);
    icm_driver.transport.serif_type = UI_I2C;
    icm_driver.transport.read_reg  = i2c_read;
    icm_driver.transport.write_reg = i2c_write;
  } else {
    spi->begin();
    pinMode(chip_select_id,OUTPUT);
    digitalWrite(chip_select_id,HIGH);
    icm_driver.transport.serif_type = UI_SPI4;
    icm_driver.transport.read_reg  = spi_read;
    icm_driver.transport.write_reg = spi_write;
  }
  icm_driver.transport.sleep_us = sleep_us;
  
  /* Disable APEX features */
  for(int i=0; i < ICM456XX_APEX_MAX; i++)
  {
    apex_enable[i] = false;
  }
  /* Set ODR to the minimum, APEX init will select the highest required ODR */
  apex_edmp_odr = DMP_EXT_SEN_ODR_CFG_APEX_ODR_25_HZ;
  apex_accel_odr = ACCEL_CONFIG0_ACCEL_ODR_1_5625_HZ;
  
  /* Init sensor event callback */
  ((inv_imu_adv_var_t *)&icm_driver.adv_var)->sensor_event_cb = sensor_event_cb;
  icm_driver_ptr = &icm_driver;

  sleep_us(3000);

  return inv_imu_adv_init(&icm_driver);
}

int ICM456xx::startAccel(uint16_t odr, uint16_t fsr) {
  int rc = 0;
  rc |= inv_imu_set_accel_fsr(&icm_driver, accel_fsr_g_to_param(fsr));
  rc |= inv_imu_set_accel_frequency(&icm_driver, accel_freq_to_param(odr));
  rc |= inv_imu_set_accel_mode(&icm_driver, PWR_MGMT0_ACCEL_MODE_LN);
  return rc;
}

int ICM456xx::startGyro(uint16_t odr, uint16_t fsr) {
  int rc = 0;
  rc |= inv_imu_set_gyro_fsr(&icm_driver, gyro_fsr_dps_to_param(fsr));
  rc |= inv_imu_set_gyro_frequency(&icm_driver, gyro_freq_to_param(odr));
  rc |= inv_imu_set_gyro_mode(&icm_driver, PWR_MGMT0_GYRO_MODE_LN);
  return rc;
}

int ICM456xx::stopAccel(void) {
  return inv_imu_set_accel_mode(&icm_driver, PWR_MGMT0_ACCEL_MODE_OFF);
}

int ICM456xx::stopGyro(void) {
  return inv_imu_set_gyro_mode(&icm_driver, PWR_MGMT0_GYRO_MODE_OFF);
}

int ICM456xx::getDataFromRegisters(inv_imu_sensor_data_t& data) {
    return inv_imu_get_register_data(&icm_driver, &data);
}

int ICM456xx::setup_irq(uint8_t intpin, ICM456xx_irq_handler handler)
{
  int rc = 0;
  inv_imu_int_state_t it_conf;
  const inv_imu_int_pin_config_t it_pins = {
    .int_polarity=INTX_CONFIG2_INTX_POLARITY_HIGH,
    .int_mode=INTX_CONFIG2_INTX_MODE_PULSE,
    .int_drive=INTX_CONFIG2_INTX_DRIVE_PP
  };
  memset(&it_conf, INV_IMU_DISABLE, sizeof(it_conf));
  it_conf.INV_FIFO_THS = INV_IMU_ENABLE;
//  it_conf.INV_UI_DRDY  = INV_IMU_ENABLE;
  pinMode(intpin,INPUT);
  rc |= inv_imu_set_pin_config_int(&icm_driver, INV_IMU_INT1, &it_pins);
  rc |= inv_imu_set_config_int(&icm_driver, INV_IMU_INT1, &it_conf);
  attachInterrupt(intpin,handler,HIGH);
  return rc;
}

static int inv_init_i2cm_bypass(inv_imu_device_t *s)
{
  int status = INV_IMU_OK;
  ioc_pad_scenario_aux_ovrd_t ioc_pad_scenario_aux_ovrd;

  uint8_t                   data = 0x1b;

  /* Force AUX1 in I2CM master mode */
  status |=
    inv_imu_read_reg(s, IOC_PAD_SCENARIO_AUX_OVRD, 1, (uint8_t *)&ioc_pad_scenario_aux_ovrd);
  ioc_pad_scenario_aux_ovrd.aux1_mode_ovrd     = 1;
  ioc_pad_scenario_aux_ovrd.aux1_mode_ovrd_val = 2;
  status |=
    inv_imu_write_reg(s, IOC_PAD_SCENARIO_AUX_OVRD, 1, (uint8_t *)&ioc_pad_scenario_aux_ovrd);

  return status;
}

int ICM456xx::enableFifoInterrupt(uint8_t intpin, ICM456xx_irq_handler handler, uint8_t fifo_watermark) {
  int rc = 0;
  inv_imu_int_state_t it_conf;
  const inv_imu_fifo_config_t fifo_config = {
    .gyro_en=true,
    .accel_en=true,
    .hires_en=false,
    .fifo_wm_th=fifo_watermark,
    .fifo_mode=FIFO_CONFIG0_FIFO_MODE_SNAPSHOT,
    .fifo_depth=FIFO_CONFIG0_FIFO_DEPTH_MAX
  };
  if(handler == NULL) {
    return -1;
  }

  rc |= inv_imu_set_fifo_config(&icm_driver, &fifo_config);

  rc |= setup_irq(intpin, handler);
  return rc;
}


int ICM456xx::getDataFromFifo(inv_imu_fifo_data_t& data) {
  return inv_imu_get_fifo_frame(&icm_driver,&data);
}

#if (INV_DEVICE_TYPE == INV_TYPE_A2) || (INV_DEVICE_TYPE == INV_TYPE_B1) || (INV_DEVICE_TYPE == INV_TYPE_C1)
int ICM456xx::startGaf(uint8_t intpin, ICM456xx_irq_handler handler, uint8_t algo)
{
  int rc = 0;
  int gyro_is_on = 1;
  int mag_is_on = 0;  
  inv_imu_edmp_gaf_parameters_t gaf_params;
  inv_imu_edmp_int_state_t apex_int_config;
  //int8_t clk_variation = 0;
  inv_imu_adv_fifo_config_t fifo_config = {
    .base_conf = {
      .gyro_en    = INV_IMU_DISABLE, // raw data are read from sensor registers
      .accel_en   = INV_IMU_DISABLE, // raw data are read from sensor registers      .fifo_mode  = FIFO_CONFIG0_FIFO_MODE_SNAPSHOT,
      .hires_en   = INV_IMU_DISABLE,
      .fifo_wm_th = 4,
      .fifo_mode = FIFO_CONFIG0_FIFO_MODE_SNAPSHOT,
    },
    .fifo_wr_wm_gt_th     = FIFO_CONFIG2_FIFO_WR_WM_EQ_OR_GT_TH,
    .tmst_fsync_en        = INV_IMU_DISABLE,
    .es1_en               = INV_IMU_DISABLE,
    .es0_en               = INV_IMU_ENABLE,
    .es0_6b_9b            = FIFO_CONFIG4_FIFO_ES0_9B,
    .comp_en              = INV_IMU_DISABLE,
    .comp_nc_flow_cfg     = FIFO_CONFIG4_FIFO_COMP_NC_FLOW_CFG_DIS,
    .gyro_dec             = ODR_DECIMATE_CONFIG_GYRO_FIFO_ODR_DEC_1,
    .accel_dec            = ODR_DECIMATE_CONFIG_ACCEL_FIFO_ODR_DEC_1
  };

  stopAccel();
  stopGyro();
  rc = inv_imu_edmp_disable(&icm_driver);

#if (INV_DEVICE_TYPE == INV_TYPE_C1)
  if(algo == ALGO_GRV) // enable GRV when enable 6-axis(AG)
  { 
    mag_is_on = 0;
  }
  else if(algo == ALGO_GMRV) // enable GMRV when enable 6-axis(AM)
  {
    mag_is_on = 1;
	gyro_is_on = 0;
  }
  else if(algo == ALGO_RV) // enable RV when enable 9-axis(AGM)
  {
    mag_is_on = 1;
	gyro_is_on = 1;
  }

  if (mag_is_on) {
  	rc = invn_mag_init(&icm_driver);

    if (rc != 0)
	{
      mag_is_on = 0;
	  return rc;
    }
  }
#endif

  memset(&gaf_outputs_internal, 0, sizeof(gaf_outputs_internal));
  
  rc |= inv_imu_edmp_set_frequency(&icm_driver, DMP_EXT_SEN_ODR_CFG_APEX_ODR_100_HZ);

#if (INV_DEVICE_TYPE == INV_TYPE_A2)
  fifo_config.base_conf.fifo_depth = FIFO_CONFIG0_FIFO_DEPTH_GAF;

  /* Load APEX algorithm GAF in DMP RAM */
  rc |= inv_imu_edmp_gaf_init(&icm_driver);
    
  /* Configure GAF parameters */
  rc |= inv_imu_edmp_gaf_init_parameters(&icm_driver, &gaf_params);

  //gaf_params.clock_variation = clk_variation;
  gaf_params.pdr_us          = 10000;
  rc |= inv_imu_edmp_gaf_set_parameters(&icm_driver, &gaf_params);
#else
  fifo_config.base_conf.gyro_en    = INV_IMU_ENABLE;
  fifo_config.base_conf.accel_en   = INV_IMU_ENABLE;
  fifo_config.base_conf.fifo_depth = FIFO_CONFIG0_FIFO_DEPTH_APEX;
  fifo_config.base_conf.fifo_wm_th = 1;
  fifo_config.es1_en			   = INV_IMU_ENABLE;
  fifo_config.es0_en               = INV_IMU_ENABLE;
  fifo_config.tmst_fsync_en 	   = INV_IMU_ENABLE;
  fifo_config.es0_6b_9b            = FIFO_CONFIG4_FIFO_ES0_9B;

  rc |= inv_imu_edmp_init(&icm_driver);
  rc |= inv_imu_edmp_get_gaf_parameters(&icm_driver, &gaf_params);
#if (INV_DEVICE_TYPE == INV_TYPE_C1)
  gaf_params.pdr_us = 20000;
  gaf_params.run_spherical = true;
  if (mag_is_on)
    gaf_params.mag_dt_us = 20000;
  else
    gaf_params.mag_dt_us = 0;
#else
  gaf_params.pdr_us	= 10000;
#endif

  int32_t acc_bias_q16[3]= {0,0,0};
  int16_t gyr_bias_q16[3]= {0,0,0};
  int32_t mag_bias_q16[3]= {0,0,0};

  rc |= inv_imu_edmp_set_frequency(&icm_driver, DMP_EXT_SEN_ODR_CFG_APEX_ODR_100_HZ);
  rc |= inv_imu_set_accel_frequency(&icm_driver, ACCEL_CONFIG0_ACCEL_ODR_100_HZ);
  rc |= inv_imu_set_gyro_frequency(&icm_driver, GYRO_CONFIG0_GYRO_ODR_100_HZ);
  rc |= inv_imu_edmp_set_gaf_acc_bias(&icm_driver, acc_bias_q16);
  rc |= inv_imu_edmp_set_gaf_gyr_bias(&icm_driver, gyr_bias_q16, 0, 0);
  rc |= inv_imu_edmp_set_gaf_parameters(&icm_driver, &gaf_params);
#if (INV_DEVICE_TYPE == INV_TYPE_C1)
  rc |= inv_imu_edmp_set_gaf_mag_bias(&icm_driver, mag_bias_q16, 0);
  rc |= inv_imu_edmp_set_gaf_mode(&icm_driver, gyro_is_on, mag_is_on);

  if (mag_is_on)
  {
	/* Configure properly soft iron matrix in eDMP image */
	rc |= inv_imu_edmp_set_gaf_soft_iron_cor_matrix(&icm_driver, soft_iron_matrix);
	rc |= inv_imu_edmp_enable_gaf_soft_iron_cor(&icm_driver);
  }

  if(!gyro_is_on) // For GMRV
  {
    rc |= inv_imu_set_accel_mode(&icm_driver, PWR_MGMT0_ACCEL_MODE_LP);
	rc |= inv_imu_select_accel_lp_clk(&icm_driver, SMC_CONTROL_0_ACCEL_LP_CLK_RCOSC);
  }
  
  rc |= invn_mag_load_ram_image(&icm_driver, INVN_MAG_USECASE_IMG_OVER_SIF);
  rc |= inv_imu_edmp_start_gaf_fifo_push(&icm_driver);
#endif
  rc |= inv_imu_adv_disable_wom(&icm_driver);
  rc |= inv_imu_adv_reset_fifo(&icm_driver);
#endif

  if (rc != 0) {
    return rc;
  }
  rc |= startAccel(100,8);
  if(gyro_is_on)
    rc |= startGyro(100,2000);
  sleep_us(GYR_STARTUP_TIME_US);
  
  /* Only 2k FIFO is supported if GAF RAM image is loaded */
  rc |= inv_imu_adv_set_fifo_config(&icm_driver, &fifo_config);
  rc |= setup_irq(intpin,handler);

#if (INV_DEVICE_TYPE == INV_TYPE_A2)
  rc |= inv_imu_edmp_gaf_enable(&icm_driver);
#else
  rc |= inv_imu_edmp_enable_gaf(&icm_driver);
#endif
  rc |= inv_imu_edmp_enable(&icm_driver);
  return rc;
}

int ICM456xx::getGafData(inv_imu_edmp_gaf_outputs_t& gaf_outputs)
{
  int rc = 0;
  uint16_t fifo_count;
  uint8_t fifo_data[FIFO_MIRRORING_SIZE]; /* Memory where to store FIFO data */
  uint8_t count = 0;
  while((gaf_status != 1)&&(rc==0)&&(count++ <100))
  {
    rc |= inv_imu_adv_get_data_from_fifo(&icm_driver, fifo_data, &fifo_count);
    rc |= inv_imu_adv_parse_fifo_data(&icm_driver, fifo_data, fifo_count);
  }
  if(gaf_status == 1)
  {
    memcpy(&gaf_outputs, &gaf_outputs_internal, sizeof(inv_imu_edmp_gaf_outputs_t));
    gaf_status = 0;
  } else {
    memset(&gaf_outputs, 0, sizeof(inv_imu_edmp_gaf_outputs_t));
    return -1;
  }

  return 0;
}

int ICM456xx::getGaf_GRVData(float& quatW, float& quatX, float& quatY, float& quatZ)
{
  int rc = 0;
  inv_imu_edmp_gaf_outputs_t gaf_outputs;
  rc = getGafData(gaf_outputs);

#if (INV_DEVICE_TYPE == INV_TYPE_C1)
  if(gaf_outputs.grv_quat_valid && !rc)
    rc = 0;
#endif

#if (INV_DEVICE_TYPE == INV_TYPE_B1) || (INV_DEVICE_TYPE == INV_TYPE_C1)
  if(rc == 0) {
    quatW = (float)gaf_outputs.grv_quat_q14[0] / (float)Q14_DIV;
    quatX = (float)gaf_outputs.grv_quat_q14[1] / (float)Q14_DIV;
    quatY = (float)gaf_outputs.grv_quat_q14[2] / (float)Q14_DIV;
    quatZ = (float)gaf_outputs.grv_quat_q14[3] / (float)Q14_DIV;
  } 
#else
  if(rc == 0) {
    quatW = (float)gaf_outputs.grv_quat_q30[0] / (float)Q30_DIV;
    quatX = (float)gaf_outputs.grv_quat_q30[1] / (float)Q30_DIV;
    quatY = (float)gaf_outputs.grv_quat_q30[2] / (float)Q30_DIV;
    quatZ = (float)gaf_outputs.grv_quat_q30[3] / (float)Q30_DIV;
  }
#endif
  else {
    quatW = 0;
    quatX = 0;
    quatY = 0;
    quatZ = 0;
  }
  return rc;
}

int ICM456xx::getGaf_GMRVData(float& quatW,float& quatX,float& quatY,float& quatZ, float& accuracy)
{
#if (INV_DEVICE_TYPE == INV_TYPE_C1)
  inv_imu_edmp_gaf_outputs_t gaf_outputs;

  accuracy = 0;
  getGafData(gaf_outputs);
  
  if(gaf_outputs.gmrv_quat_valid)
  {
    quatW = (int32_t)gaf_outputs.gmrv_quat_q14[0] / (float)Q14_DIV;
    quatX = (int32_t)gaf_outputs.gmrv_quat_q14[1] / (float)Q14_DIV;
    quatY = (int32_t)gaf_outputs.gmrv_quat_q14[2] / (float)Q14_DIV;
    quatZ = (int32_t)gaf_outputs.gmrv_quat_q14[3] / (float)Q14_DIV;
	if(gaf_outputs.gmrv_heading_valid)
		accuracy = (int32_t)gaf_outputs.gmrv_heading_q11 / (float)Q11_DIV;
  } else {
    quatW = 0;
    quatX = 0;
    quatY = 0;
    quatZ = 0;
  }
#endif
  return 0;
}

int ICM456xx::getGaf_RVData(float& quatW, float& quatX, float& quatY, float& quatZ, float& accuracy)
{
#if (INV_DEVICE_TYPE == INV_TYPE_C1)
  inv_imu_edmp_gaf_outputs_t gaf_outputs;

  accuracy = 0;
  getGafData(gaf_outputs);

  if(gaf_outputs.rv_quat_valid)
  {
    quatW = (int32_t)gaf_outputs.rv_quat_q14[0] / (float)Q14_DIV;
    quatX = (int32_t)gaf_outputs.rv_quat_q14[1] / (float)Q14_DIV;
    quatY = (int32_t)gaf_outputs.rv_quat_q14[2] / (float)Q14_DIV;
    quatZ = (int32_t)gaf_outputs.rv_quat_q14[3] / (float)Q14_DIV;
	if(gaf_outputs.rv_heading_valid)
		accuracy = (int32_t)gaf_outputs.rv_heading_q11 / (float)Q11_DIV;
  }else{
    quatW = 0;
    quatX = 0;
    quatY = 0;
    quatZ = 0;
  }
#endif
  return 0;
}

int ICM456xx::getGaf_RMData(float& mX, float& mY, float& mZ)
{
#if (INV_DEVICE_TYPE == INV_TYPE_C1)
  mX = (int32_t)gaf_outputs_internal.rmag[0] * RAW_MAG_SCALE;
  mY = (int32_t)gaf_outputs_internal.rmag[1] * RAW_MAG_SCALE;
  mZ = (int32_t)gaf_outputs_internal.rmag[2] * RAW_MAG_SCALE;
#endif
  return 0;
}

int ICM456xx::getGaf_BiasData(int type, int& q12_X,int& q12_Y,int& q12_Z,int& accuracy)
{
#if (INV_DEVICE_TYPE == INV_TYPE_A2)
  if(type != GYRO)
	return -1;

  q12_X = gaf_outputs_internal.gyr_bias_q16[0] >> 4;
  q12_Y = gaf_outputs_internal.gyr_bias_q16[1] >> 4;
  q12_Z = gaf_outputs_internal.gyr_bias_q16[2] >> 4;
  accuracy = gaf_outputs_internal.gyr_accuracy_flag;
#else
  switch(type)
  {
    case GYRO:
      q12_X = gaf_outputs_internal.gyr_bias_q12[0];
      q12_Y = gaf_outputs_internal.gyr_bias_q12[1];
      q12_Z = gaf_outputs_internal.gyr_bias_q12[2];
      accuracy = gaf_outputs_internal.gyr_accuracy_flag;
      break;
#if (INV_DEVICE_TYPE == INV_TYPE_C1)
    case MAG:
      q12_X = gaf_outputs_internal.mag_bias_q16[0] >> 4;
      q12_Y = gaf_outputs_internal.mag_bias_q16[1] >> 4;
      q12_Z = gaf_outputs_internal.mag_bias_q16[2] >> 4;
      accuracy = gaf_outputs_internal.mag_accuracy_flag;
      break;
#endif
    default:
      return -1;
  }
#endif
  return 0;
}

#endif /* INV_TYPE_A2 & INV_TYPE_C1 */

#if (INV_DEVICE_TYPE == INV_TYPE_B1)

/* Default matrix to be used: identity */
static int8_t mounting_matrix[9] = { 1, 0, 0, 0, 1, 0, 0, 0, 1 };

int ICM456xx::startVocalVibDet(uint8_t intpin, ICM456xx_irq_handler handler)
{
  int rc = 0;
  inv_imu_edmp_int_state_t apex_int_config;
  inv_imu_edmp_vvd_parameters_t vvd_params;
  inv_imu_int_state_t int_config;
  inv_imu_int_pin_config_t	int_pin_config;

  pinMode(intpin,INPUT);
  attachInterrupt(intpin,handler,RISING);

  sleep_us(3000);

  int_pin_config.int_polarity = INTX_CONFIG2_INTX_POLARITY_HIGH;
  int_pin_config.int_mode     = INTX_CONFIG2_INTX_MODE_PULSE;
  int_pin_config.int_drive    = INTX_CONFIG2_INTX_DRIVE_PP;
  rc |= inv_imu_set_pin_config_int(&icm_driver, INV_IMU_INT1, &int_pin_config);

  /* Stop all sensors in case they were previously ON */
  rc |= inv_imu_edmp_disable(&icm_driver);
  rc |= inv_imu_set_accel_mode(&icm_driver, PWR_MGMT0_ACCEL_MODE_OFF);
  if (rc != 0) {
    return rc;
  }

  /* Set ODR */
  rc |= inv_imu_edmp_set_frequency(&icm_driver, DMP_EXT_SEN_ODR_CFG_APEX_ODR_800_HZ);
  rc |= inv_imu_set_accel_frequency(&icm_driver, ACCEL_CONFIG0_ACCEL_ODR_800_HZ);
  if (rc != 0) {
    return rc;
  }

  /* Set sensor FSR */
  rc |= inv_imu_set_accel_fsr(&icm_driver, ACCEL_CONFIG0_ACCEL_UI_FS_SEL_8_G);
  if (rc != 0) {
    return rc;
  }
  
  /* Initialize APEX for VVD */
  rc |= inv_imu_edmp_init(&icm_driver);
  rc |= inv_imu_edmp_get_vvd_parameters(&icm_driver, &vvd_params);

  if(nb_samples_before_decision)
    vvd_params.nbSamplesBeforeDecision = nb_samples_before_decision;
  else
    nb_samples_before_decision = vvd_params.nbSamplesBeforeDecision;

  if(vvd_thresh)
    vvd_params.dynEnvVVDThresh = vvd_thresh;
  else
    vvd_thresh = vvd_params.dynEnvVVDThresh;
  
  rc |= inv_imu_edmp_set_vvd_parameters(&icm_driver, &vvd_params);

  rc |= inv_imu_edmp_set_vvd_model(&icm_driver);
  if (rc != 0) {
    return rc;
  }
  
  /* Set accel in low-noise mode with no filter as ODR/2 is not reachable */
  rc |= inv_imu_set_accel_ln_bw(&icm_driver, IPREG_SYS2_REG_131_ACCEL_UI_LPFBW_NO_FILTER);
  rc |= inv_imu_set_accel_mode(&icm_driver, PWR_MGMT0_ACCEL_MODE_LN);
  if (rc != 0) {
    return rc;
  }
  
  /* Wait for accel startup time */
  sleep_us(ACC_STARTUP_TIME_US);
  
  /* Set EDMP mounting matrix */
  rc |= inv_imu_edmp_set_mounting_matrix(&icm_driver, mounting_matrix);
  if (rc != 0) {
    return rc;
  }
  
  memset(&int_config, INV_IMU_DISABLE, sizeof(int_config));
  int_config.INV_EDMP_EVENT = INV_IMU_ENABLE;

  rc |= inv_imu_set_config_int(&icm_driver, INV_IMU_INT1, &int_config);
  if (rc != 0) {
    return rc;
  }
  
  /* Enable VVD */
  rc |= inv_imu_edmp_enable_vvd(&icm_driver);
  if (rc != 0) {
    return rc;
  }
  
  /* Disable all APEX interrupt and enable only the one we need */
  memset(&apex_int_config, INV_IMU_DISABLE, sizeof(apex_int_config));
  apex_int_config.INV_VVD = INV_IMU_ENABLE;
  rc |= inv_imu_edmp_set_config_int_apex(&icm_driver, &apex_int_config);
  if (rc != 0) {
    return rc;
  }
  
  /* Enable EDMP */
  rc |= inv_imu_edmp_enable(&icm_driver);
  if (rc != 0) {
    return rc;
  }

  return rc;
}

int ICM456xx::GetVocalVibDet_thresh(void)
{
  return vvd_thresh;
}

void ICM456xx::SetVocalVibDet_thresh(uint32_t thresh)
{
  vvd_thresh = thresh;
}

int ICM456xx::SetVocalVibDet_Dynthresh(uint32_t thresh)
{
  return inv_imu_edmp_request_vvd_update_thresh(&icm_driver, thresh);
}

int ICM456xx::GetVocalVibDet_nb_samples(void)
{
  return nb_samples_before_decision;
}

void ICM456xx::SetVocalVibDet_nb_samples(uint32_t sample)
{
  nb_samples_before_decision = sample;
}

int ICM456xx::CheckVocalVibDet_thresh(void)
{
  uint8_t status_update=0;
  inv_imu_edmp_check_vvd_thresh(&icm_driver, &status_update);
  return status_update;
}

#endif

#if (INV_DEVICE_TYPE == INV_TYPE_A1) || (INV_DEVICE_TYPE == INV_TYPE_B1) || (INV_DEVICE_TYPE == INV_TYPE_C1)

#define MAG_I2C_ADDR 0x1E

static int wait_for_i2c_master_complete(inv_imu_device_t *icm_driver)
{
  uint64_t start;
  uint64_t cur = 0;
  inv_imu_int_state_t int_state;

  while (1) {
    inv_imu_get_int_status(icm_driver, INV_IMU_INT1, &int_state);
    if (int_state.INV_I2CM_DONE) {
      i2cm_status_t i2cm_status;

      inv_imu_read_reg(icm_driver, I2CM_STATUS, 1, (uint8_t *)&i2cm_status);
      if (i2cm_status.i2cm_busy || i2cm_status.i2cm_timeout_err ||
          i2cm_status.i2cm_srst_err || i2cm_status.i2cm_scl_err ||
          i2cm_status.i2cm_sda_err)
        return -1;
      return 0;
    }
  }

  return -2;
}

int ICM456xx::setI2CM(void)
{
  int rc = 0;
  uint8_t data  =0;
  inv_imu_int_state_t int1_config;

  rc = inv_imu_init_i2cm(&icm_driver);
  rc |= inv_imu_get_config_int(&icm_driver, INV_IMU_INT1, &int1_config);
  int1_config.INV_I2CM_DONE = INV_IMU_ENABLE;
  rc |= inv_imu_set_config_int(&icm_driver, INV_IMU_INT1, &int1_config);  
  return rc;
}

int ICM456xx::getDataFromI2CM(uint8_t reg, uint8_t& data)
{
  int rc;
  /* read register from ICT1531 */
  inv_imu_i2c_master_cfg_t cfg = { .op_cnt   = 1,
                                   .i2c_addr = MAG_I2C_ADDR,
                                   .op 	  = { /* op[0] */
                                             {
                                               .r_n_w	   = 1,
                                               .reg_addr = reg,
                                               .len      = 1
                                  } } };
  
  rc |= inv_imu_configure_i2cm(&icm_driver, &cfg, NULL);
  rc |= inv_imu_start_i2cm_ops(&icm_driver, 1 /* fast_mode */);
  rc |= wait_for_i2c_master_complete(&icm_driver);
  rc |= inv_imu_get_i2cm_data(&icm_driver, &data, 1);

  return rc;
}

int ICM456xx::setI2CMPassThrough(void)
{
  return inv_init_i2cm_bypass(&icm_driver);
}

int ICM456xx::getDataFromPassThrough(uint8_t reg, uint8_t& data)
{
  int rc;
  /* directly read register from ICT1531 */
  i2c_address = MAG_I2C_ADDR;
  rc = inv_imu_read_reg(&icm_driver, reg, 1, (uint8_t *)&data);
  i2c_address = ICM456xx_I2C_ADDRESS;
  return rc;
}

static uint8_t fifo_data[FIFO_MIRRORING_SIZE]; /* Memory where to store FIFO data */

int ICM456xx::adv_getDataFromFifo(void)
{
  uint16_t fifo_count = 0;
  inv_imu_adv_get_data_from_fifo(&icm_driver, fifo_data, &fifo_count);
  inv_imu_adv_parse_fifo_data(&icm_driver, fifo_data, fifo_count);
  return 0;
}

#if (INV_DEVICE_TYPE == INV_TYPE_A1)
int ICM456xx::setI2CM_FIFO(uint8_t intpin, ICM456xx_irq_handler handler)
{
  int rc = 0;
  uint8_t data  =0;
  inv_imu_int_state_t int1_config;
  inv_imu_int_pin_config_t  int_pin_config;
  inv_imu_adv_fifo_config_t fifo_config;
  inv_imu_edmp_apex_parameters_t apex_inputs;
  uint8_t wdata = 0x12;

  inv_imu_i2c_master_cfg_t cfg = { .op_cnt	= 2,
                                   .i2c_addr = MAG_I2C_ADDR,
                                   .op	  = { /* op[0] */
                                            {
                                                 .r_n_w    = 1,
                                                 .reg_addr = ICT1531X_TEMP_DATA_MSB,
                                                 .len	   = 7
                                            },
                                            {
                                                 .r_n_w    = 0,
                                                 .reg_addr = ICT1531X_MODE_CTRL_REG,
                                                 .len	   = 1,
                                                 .wdata = &wdata
                                            }
                                  } };

  int_pin_config.int_polarity = INTX_CONFIG2_INTX_POLARITY_HIGH;
  int_pin_config.int_mode     = INTX_CONFIG2_INTX_MODE_PULSE;
  int_pin_config.int_drive    = INTX_CONFIG2_INTX_DRIVE_PP;
  rc |= inv_imu_set_pin_config_int(&icm_driver, INV_IMU_INT1, &int_pin_config);

  /* Interrupts configuration */
  memset(&int1_config, INV_IMU_DISABLE, sizeof(int1_config));
  int1_config.INV_FIFO_THS = INV_IMU_ENABLE;
  rc |= inv_imu_set_config_int(&icm_driver, INV_IMU_INT1, &int1_config);

  /* FIFO configuration */
  rc |= inv_imu_adv_get_fifo_config(&icm_driver, &fifo_config);
  fifo_config.base_conf.gyro_en    = 1;
  fifo_config.base_conf.accel_en   = 1;
  fifo_config.base_conf.hires_en   = 0;
  fifo_config.base_conf.fifo_depth = FIFO_CONFIG0_FIFO_DEPTH_APEX;
  fifo_config.base_conf.fifo_wm_th = 1;
  fifo_config.base_conf.fifo_mode  = FIFO_CONFIG0_FIFO_MODE_SNAPSHOT;
  fifo_config.tmst_fsync_en        = INV_IMU_ENABLE;
  fifo_config.fifo_wr_wm_gt_th     = FIFO_CONFIG2_FIFO_WR_WM_EQ_OR_GT_TH;
  fifo_config.es0_en               = 1;
  fifo_config.es1_en               = 0;
  /* eDMP image will read external data and output 6 bytes in FIFO ES0 frame bitfield */
  fifo_config.es0_6b_9b = FIFO_CONFIG4_FIFO_ES0_6B;
  rc |= inv_imu_adv_set_fifo_config(&icm_driver, &fifo_config);

  rc = invn_mag_init(&icm_driver);
  rc |= inv_imu_edmp_compass_set_frequency(&icm_driver, DMP_EXT_SEN_ODR_CFG_EXT_ODR_25_HZ);
  rc |= inv_imu_configure_i2cm(&icm_driver, &cfg, NULL);
  
  /* Run eDMP init and load RAM image */
  rc |= inv_imu_edmp_compass_init(&icm_driver);

  /* Configure eDMP to get compass data from external sensor */
  rc |= inv_imu_edmp_compass_enable_es(&icm_driver);
  
  /* Reset FIFO */
  rc |= inv_imu_adv_reset_fifo(&icm_driver);
  
  /* Enable eDMP */
  rc |= inv_imu_edmp_enable(&icm_driver);
  
  /* Let dmp see ODR ready on ISR1, ISR0 all off */
  rc |= inv_imu_edmp_unmask_int_src(&icm_driver, INV_IMU_EDMP_INT1, EDMP_INT_SRC_EXT_ODR_DRDY_MASK);

  rc |= setup_irq(intpin, handler);
  
  return rc;  
}

int ICM456xx::getAdvDataFromFifo(int32_t *accel, int32_t *gyro, float *external)
{
  int rc = 0;
  uint16_t fifo_count;
  const fifo_header_t *    header;
  const fifo_header2_t *   header2;

  rc |= inv_imu_adv_get_data_from_fifo(&icm_driver, fifo_data, &fifo_count);

  /* patch for es0 vld setting fail */
  header  = (const fifo_header_t *)&fifo_data[0];
  if(header->bits.ext_header)
  {
    header2 = (const fifo_header2_t *)&fifo_data[1];    
    if (header2->bits.es0_en)
      fifo_data[1] |= (1 << 2);
  }

  rc |= inv_imu_adv_parse_fifo_data(&icm_driver, fifo_data, fifo_count);

  memcpy(accel, accel_data, sizeof(accel_data));
  memcpy(gyro, gyro_data, sizeof(gyro_data));
  memcpy(external, mag_data, sizeof(mag_data));

  return rc;
}

int ICM456xx::initMag(void)
{
  int ret =0;
  ret = invn_read_whoami(&icm_driver);
  return ret;
}

int ICM456xx::enableMag(int flag)
{
  int ret =0;
  ret = invn_mag_enable(flag);
  return ret;
}

int ICM456xx::getExternalMagData(float *mag)
{
  int rc;
  uint8_t data[6];
  int16_t external_data[3];
   
  /* read register from ICT1531 */
  inv_imu_i2c_master_cfg_t cfg = { .op_cnt   = 1,
                                   .i2c_addr = MAG_I2C_ADDR,
                                   .op 	  = { /* op[0] */
                                            {
                                               .r_n_w	   = 1,
                                               .reg_addr = ICT1531X_MAG_DATA_X_LSB,
                                               .len      = 6
                                 } } };
  
  rc |= inv_imu_configure_i2cm(&icm_driver, &cfg, NULL);
  rc |= inv_imu_start_i2cm_ops(&icm_driver, 1 /* fast_mode */);
  rc |= wait_for_i2c_master_complete(&icm_driver);
  rc |= inv_imu_get_i2cm_data(&icm_driver, data, 6);

  external_data[0] = (((int16_t)data[1]) << 8) + data[0];
  external_data[1] = (((int16_t)data[3]) << 8) + data[2];
  external_data[2] = (((int16_t)data[5]) << 8) + data[4];

  mag_data[0] = external_data[0] * RAW_MAG_SCALE;
  mag_data[1] = external_data[1] * RAW_MAG_SCALE;
  mag_data[2] = external_data[2] * RAW_MAG_SCALE;
  memcpy(mag, mag_data, sizeof(mag_data));

  return rc;
}

#endif /* INV_TYPE_A1 */
#endif /* INV_TYPE_A1 & INV_TYPE_C1 */

/* FIFO sensor event callback */
static void sensor_event_cb(inv_imu_sensor_event_t *event)
{
  if(event->sensor_mask & (1 << INV_SENSOR_ACCEL))
  {
    accel_data[0] = event->accel[0];
    accel_data[1] = event->accel[1];
    accel_data[2] = event->accel[2];
  }
	
  if(event->sensor_mask & (1 << INV_SENSOR_GYRO))
  {
    gyro_data[0] = event->gyro[0];
    gyro_data[1] = event->gyro[1];
    gyro_data[2] = event->gyro[2];  
  }

#if (INV_DEVICE_TYPE == INV_TYPE_A2)
  if(event->sensor_mask & (1 << INV_SENSOR_ES0))
  {
    gaf_status = inv_imu_edmp_gaf_build_outputs(icm_driver_ptr, (const uint8_t *)event->es0, &gaf_outputs_internal);
  }
#elif (INV_DEVICE_TYPE == INV_TYPE_A1)
  if(event->sensor_mask & (1 << INV_SENSOR_ES0))
  {
    int16_t external_data[3];
	
    external_data[0] = (((int16_t)event->es0[1]) << 8) + event->es0[0];
    external_data[1] = (((int16_t)event->es0[3]) << 8) + event->es0[2];
    external_data[2] = (((int16_t)event->es0[5]) << 8) + event->es0[4];

    mag_data[0] = external_data[0] * RAW_MAG_SCALE;
    mag_data[1] = external_data[1] * RAW_MAG_SCALE;
    mag_data[2] = external_data[2] * RAW_MAG_SCALE;
  }

#elif (INV_DEVICE_TYPE == INV_TYPE_B1) || (INV_DEVICE_TYPE == INV_TYPE_C1)
  if (event->sensor_mask & ((1 << INV_SENSOR_ES0) | (1 << INV_SENSOR_ES1)) == ((1 << INV_SENSOR_ES0) | (1 << INV_SENSOR_ES1)))
  {
    inv_imu_edmp_gaf_decode_fifo(icm_driver_ptr, (const uint8_t *)event->es0,
                                 (const uint8_t *)event->es1, &gaf_outputs_internal);
#if (INV_DEVICE_TYPE == INV_TYPE_C1)
    gaf_status = gaf_outputs_internal.frame_complete;
#else
    gaf_status = 1;
#endif
  }
#endif
}

int ICM456xx::setApexInterrupt(uint8_t intpin, ICM456xx_irq_handler handler)
{
  int rc = 0;
  inv_imu_int_state_t config_int;
  inv_imu_int_pin_config_t int_pin_config;
  inv_imu_edmp_int_state_t apex_int_config;

  if (handler == NULL)
    return 0;
  
  pinMode(intpin,INPUT);
  attachInterrupt(intpin,handler,RISING);
  
  /*
   * Configure interrupts pins
   * - Polarity High
   * - Pulse mode
   * - Push-Pull drive
   */
  int_pin_config.int_polarity = INTX_CONFIG2_INTX_POLARITY_HIGH;
  int_pin_config.int_mode     = INTX_CONFIG2_INTX_MODE_PULSE;
  int_pin_config.int_drive    = INTX_CONFIG2_INTX_DRIVE_PP;
  rc |= inv_imu_set_pin_config_int(&icm_driver, INV_IMU_INT1, &int_pin_config);
  
  /* Interrupts configuration: Enable only EDMP interrupt */
  inv_imu_get_config_int(&icm_driver,INV_IMU_INT1, &config_int);
  config_int.INV_WOM_X = INV_IMU_DISABLE;
  config_int.INV_WOM_Y = INV_IMU_DISABLE;
  config_int.INV_WOM_Z = INV_IMU_DISABLE;
  config_int.INV_FIFO_THS = INV_IMU_DISABLE;
  config_int.INV_EDMP_EVENT = INV_IMU_ENABLE;
  inv_imu_set_config_int(&icm_driver,INV_IMU_INT1, &config_int);
        
  /* Apply interrupt configuration */
  memset(&apex_int_config, INV_IMU_DISABLE, sizeof(apex_int_config));
  if(apex_enable[ICM456XX_APEX_TAP])
    apex_int_config.INV_TAP = INV_IMU_ENABLE;
  if(apex_enable[ICM456XX_APEX_FF])
    apex_int_config.INV_FF = INV_IMU_ENABLE;
  if(apex_enable[ICM456XX_APEX_LOWG])
    apex_int_config.INV_LOWG = INV_IMU_ENABLE;
  if(apex_enable[ICM456XX_APEX_HIGHG])
    apex_int_config.INV_HIGHG = INV_IMU_ENABLE;
  if(apex_enable[ICM456XX_APEX_B2S])
  {
    apex_int_config.INV_B2S = INV_IMU_ENABLE;
    apex_int_config.INV_B2S_REV = INV_IMU_ENABLE;
  }
#if (INV_DEVICE_TYPE == INV_TYPE_A1) || (INV_DEVICE_TYPE == INV_TYPE_A2)
  if(apex_enable[ICM456XX_APEX_TILT])
    apex_int_config.INV_TILT_DET = INV_IMU_ENABLE;
  if(apex_enable[ICM456XX_APEX_PEDOMETER])
  {
    apex_int_config.INV_STEP_DET = INV_IMU_ENABLE;
    apex_int_config.INV_STEP_CNT_OVFL = INV_IMU_ENABLE;
  }
  if(apex_enable[ICM456XX_APEX_R2W])
  { 
    apex_int_config.INV_R2W = INV_IMU_ENABLE;
    apex_int_config.INV_R2W_SLEEP = INV_IMU_ENABLE;
  }
#elif (INV_DEVICE_TYPE == INV_TYPE_B1)
  if(apex_enable[ICM456XX_APEX_AID])
  {
    apex_int_config.INV_AID_DEVICE = INV_IMU_ENABLE;
    apex_int_config.INV_AID_HUMAN = INV_IMU_ENABLE;
  }
#endif
  rc |= inv_imu_edmp_set_config_int_apex(&icm_driver, &apex_int_config);

  return rc;
}

int ICM456xx::startAPEX(dmp_ext_sen_odr_cfg_apex_odr_t edmp_odr, accel_config0_accel_odr_t accel_odr)
{
  int rc = 0;

  if(edmp_odr == NULL && accel_odr == NULL)
  {
    if(apex_enable[ICM456XX_APEX_FF] || apex_enable[ICM456XX_APEX_LOWG] || apex_enable[ICM456XX_APEX_HIGHG])
    {
#if (INV_DEVICE_TYPE == INV_TYPE_A1) || (INV_DEVICE_TYPE == INV_TYPE_A2)
      edmp_odr = DMP_EXT_SEN_ODR_CFG_APEX_ODR_800_HZ;
      accel_odr = ACCEL_CONFIG0_ACCEL_ODR_800_HZ;
#else
      edmp_odr = DMP_EXT_SEN_ODR_CFG_APEX_ODR_400_HZ;
      accel_odr = ACCEL_CONFIG0_ACCEL_ODR_400_HZ;
#endif
    }
	else if(apex_enable[ICM456XX_APEX_TAP])
    {
      edmp_odr = DMP_EXT_SEN_ODR_CFG_APEX_ODR_400_HZ;
      accel_odr = ACCEL_CONFIG0_ACCEL_ODR_400_HZ;
    }
    else
    {
      edmp_odr = DMP_EXT_SEN_ODR_CFG_APEX_ODR_50_HZ;
      accel_odr = ACCEL_CONFIG0_ACCEL_ODR_50_HZ;
    }
  }
  
#if (INV_DEVICE_TYPE == INV_TYPE_C1) || (INV_DEVICE_TYPE == INV_TYPE_B1)
  inv_imu_edmp_powersave_parameters_t power_parameters;
  rc |= inv_imu_edmp_init(&icm_driver);
#else
  inv_imu_edmp_apex_parameters_t apex_parameters;
  rc |= inv_imu_edmp_init_apex(&icm_driver);
#endif

  if(apex_edmp_odr < edmp_odr)
      apex_edmp_odr = edmp_odr;
  /* Accel ODR higher value means lower ODR */
  if(apex_accel_odr > accel_odr)
      apex_accel_odr = accel_odr;

  rc |= inv_imu_edmp_set_frequency(&icm_driver, apex_edmp_odr);
  rc |= inv_imu_set_accel_frequency(&icm_driver, apex_accel_odr);

  /* Set BW = ODR/4 */
  rc |= inv_imu_set_accel_ln_bw(&icm_driver, IPREG_SYS2_REG_131_ACCEL_UI_LPFBW_DIV_4);

  /* Select WUOSC clock to have accel in ULP (lowest power mode) */
  rc |= inv_imu_select_accel_lp_clk(&icm_driver, SMC_CONTROL_0_ACCEL_LP_CLK_WUOSC);

  /* Set AVG to 1x */
  rc |= inv_imu_set_accel_lp_avg(&icm_driver, IPREG_SYS2_REG_129_ACCEL_LP_AVG_1);

  /* Stop APEX in case it was previously ON */
#if (INV_DEVICE_TYPE == INV_TYPE_A1) || (INV_DEVICE_TYPE == INV_TYPE_A2)
  rc |= inv_imu_edmp_disable_pedometer(&icm_driver);
  rc |= inv_imu_edmp_disable_tilt(&icm_driver);
  rc |= inv_imu_edmp_disable_r2w(&icm_driver);
#elif (INV_DEVICE_TYPE == INV_TYPE_B1)
  rc |= inv_imu_edmp_disable_b2s(&icm_driver);
  rc |= inv_imu_edmp_disable_aid(&icm_driver);
#endif
  rc |= inv_imu_edmp_disable_tap(&icm_driver);
  rc |= inv_imu_adv_disable_wom(&icm_driver);
  rc |= inv_imu_edmp_disable_ff(&icm_driver);
  rc |= inv_imu_edmp_disable(&icm_driver);

#if (INV_DEVICE_TYPE == INV_TYPE_B1) || (INV_DEVICE_TYPE == INV_TYPE_C1)
  rc |= inv_imu_edmp_recompute_decimation(&icm_driver);

  rc |= inv_imu_edmp_get_powersave_parameters(&icm_driver, &power_parameters);
  power_parameters.power_save_en = 0;
  rc |= inv_imu_edmp_set_powersave_parameters(&icm_driver, &power_parameters);
#else /* INV_TYPE_B1 & INV_TYPE_C1 */
  rc |= inv_imu_edmp_recompute_apex_decimation(&icm_driver);
  
  rc |= inv_imu_edmp_get_apex_parameters(&icm_driver, &apex_parameters);
  apex_parameters.power_save_en = 0;
  rc |= inv_imu_edmp_set_apex_parameters(&icm_driver, &apex_parameters);
#endif /* INV_TYPE_A1 & INV_TYPE_A2*/

  rc |= inv_imu_set_accel_mode(&icm_driver, PWR_MGMT0_ACCEL_MODE_LN);

#if (INV_DEVICE_TYPE == INV_TYPE_B1) || (INV_DEVICE_TYPE == INV_TYPE_C1)
  if(apex_enable[ICM456XX_APEX_TAP])
  {
    inv_imu_edmp_tap_parameters_t tap_parameters;
    rc |= inv_imu_edmp_get_tap_parameters(&icm_driver, &tap_parameters);
    tap_parameters.tap_tmax			 = TAP_TMAX_400HZ;
    tap_parameters.tap_tmin			 = TAP_TMIN_400HZ;
    tap_parameters.tap_smudge_reject_th = TAP_SMUDGE_REJECT_THR_400HZ;
    tap_parameters.tap_max = 3; // we want to be able to detect triple tap
    tap_parameters.tap_min = 1; // we want to be able to detect single tap
    rc |= inv_imu_edmp_set_tap_parameters(&icm_driver, &tap_parameters);
    rc |= inv_imu_edmp_enable_tap(&icm_driver);
  }

  if(apex_enable[ICM456XX_APEX_B2S])
  {
    inv_imu_edmp_b2s_parameters_t b2s_params;
#if (INV_DEVICE_TYPE == INV_TYPE_B1)
    rc |= inv_imu_edmp_get_b2s_parameters(&icm_driver, &b2s_params);
    b2s_params.b2s_mounting_matrix = 0;
    rc |= inv_imu_edmp_set_b2s_parameters(&icm_driver, &b2s_params);
    rc |= inv_imu_edmp_enable_b2s(&icm_driver);
#else /* INV_TYPE_B1 */
    inv_imu_edmp_b2s_init_t usecase;
    rc |= inv_imu_edmp_b2s_init_over_sif(&icm_driver, &usecase);
    rc |= inv_imu_edmp_b2s_get_parameters(&icm_driver, &b2s_params, usecase);
    b2s_params.b2s_mounting_matrix = 0; /* identity matrix */
    rc |= inv_imu_edmp_b2s_set_parameters(&icm_driver, &b2s_params, usecase);
    rc |= inv_imu_edmp_b2s_enable(&icm_driver);
#endif /*INV_TYPE_C1*/
  }

#if (INV_DEVICE_TYPE == INV_TYPE_B1)
  if(apex_enable[ICM456XX_APEX_AID])
  {
    inv_imu_edmp_aid_parameters_t aid_params;
    rc |= inv_imu_edmp_get_aid_parameters(&icm_driver, &aid_params);
    /* AID device config */
    aid_params.aid_device.aid_update_win                 = 50;
    aid_params.aid_device.aid_alert_timer                = 600;
    aid_params.aid_device.aid_output_enable              = 7;
    aid_params.aid_device.aid_disable_multiple_interrupt = 1;
    /* AID human config */
    aid_params.aid_human.aid_update_win                 = 350;
    aid_params.aid_human.aid_alert_timer                = 3750;
    aid_params.aid_human.aid_output_enable              = 7;
    aid_params.aid_human.aid_disable_multiple_interrupt = 1;

    rc |= inv_imu_edmp_set_aid_parameters(&icm_driver, &aid_params);
    rc |= inv_imu_adv_configure_wom(&icm_driver, 6, 6, 6, // 6 * 1/256mg ~= 23.44mg for each axis
                                    TMST_WOM_CONFIG_WOM_INT_MODE_ORED,
                                    TMST_WOM_CONFIG_WOM_INT_DUR_1_SMPL);
    rc |= inv_imu_adv_enable_wom(&icm_driver);
    rc |= inv_imu_edmp_enable_aid(&icm_driver);
  }
#endif /* INV_TYPE_B1 */
#else /* INV_TYPE_B1 & INV_TYPE_C1*/
  if(apex_enable[ICM456XX_APEX_TILT])
  {
    rc |= inv_imu_edmp_enable_tilt(&icm_driver);
  }
  if(apex_enable[ICM456XX_APEX_PEDOMETER])
  {
    rc |= inv_imu_edmp_enable_pedometer(&icm_driver);
  }
  if(apex_enable[ICM456XX_APEX_R2W])
  {
    rc |= inv_imu_edmp_get_apex_parameters(&icm_driver, &apex_parameters);
    apex_parameters.r2w_sleep_time_out = 6400; /* 6.4 s */
    rc |= inv_imu_edmp_set_apex_parameters(&icm_driver, &apex_parameters);
    rc |= inv_imu_edmp_enable_r2w(&icm_driver);
  }  
  if(apex_enable[ICM456XX_APEX_TAP])
  {
    rc |= inv_imu_edmp_get_apex_parameters(&icm_driver, &apex_parameters);
    apex_parameters.tap_tmax             = TAP_TMAX_400HZ;
    apex_parameters.tap_tmin             = TAP_TMIN_400HZ;
    apex_parameters.tap_smudge_reject_th = TAP_SMUDGE_REJECT_THR_400HZ;
    rc |= inv_imu_edmp_set_apex_parameters(&icm_driver, &apex_parameters);
    rc |= inv_imu_edmp_enable_tap(&icm_driver);
  }
#endif /* INV_TYPE_A1 & INV_TYPE_A2*/
  if(apex_enable[ICM456XX_APEX_FF] || 
     apex_enable[ICM456XX_APEX_HIGHG] || 
     apex_enable[ICM456XX_APEX_LOWG])
  {
    rc |= inv_imu_edmp_enable_ff(&icm_driver);
  }

  rc |= inv_imu_edmp_enable(&icm_driver);
  rc |= inv_imu_adv_enable_accel_ln(&icm_driver);

  return rc;
}

int ICM456xx::startTiltDetection(uint8_t intpin, ICM456xx_irq_handler handler)
{
  int rc = 0;

#if (INV_DEVICE_TYPE == INV_TYPE_B1) || (INV_DEVICE_TYPE == INV_TYPE_C1)
  return 0;
#endif

  apex_enable[ICM456XX_APEX_TILT] = true;

  if(handler != NULL)
  {
    rc |= setApexInterrupt(intpin, handler);
    rc |= startAPEX(DMP_EXT_SEN_ODR_CFG_APEX_ODR_50_HZ,ACCEL_CONFIG0_ACCEL_ODR_50_HZ);
  }
  return rc;
}

int ICM456xx::startPedometer(uint8_t intpin, ICM456xx_irq_handler handler)
{
  int rc = 0;
  
#if (INV_DEVICE_TYPE == INV_TYPE_B1) || (INV_DEVICE_TYPE == INV_TYPE_C1)
	return 0;
#endif

  apex_enable[ICM456XX_APEX_PEDOMETER] = true;

  step_cnt_ovflw = 0;
  if(handler != NULL)
  {
    rc |= setApexInterrupt(intpin, handler);
    rc |= startAPEX(DMP_EXT_SEN_ODR_CFG_APEX_ODR_50_HZ,ACCEL_CONFIG0_ACCEL_ODR_50_HZ);
  }
  return rc;
}

int ICM456xx::startRaiseToWake(uint8_t intpin, ICM456xx_irq_handler handler)
{
  int rc = 0;

#if (INV_DEVICE_TYPE == INV_TYPE_B1) || (INV_DEVICE_TYPE == INV_TYPE_C1)
	return 0;
#endif

  apex_enable[ICM456XX_APEX_R2W] = true;

  if(handler != NULL)
  {
    rc |= setApexInterrupt(intpin, handler);
    rc |= startAPEX(DMP_EXT_SEN_ODR_CFG_APEX_ODR_100_HZ,ACCEL_CONFIG0_ACCEL_ODR_100_HZ);
  }
  return rc;
}

int ICM456xx::getTilt(void)
{
  if(apex_enable[ICM456XX_APEX_TILT] == false)
    return INV_ERROR;

#if (INV_DEVICE_TYPE == INV_TYPE_A1) || (INV_DEVICE_TYPE == INV_TYPE_A2)
  updateApex();
  if (apex_status.INV_TILT_DET)
  {
    apex_status.INV_TILT_DET = 0;
    return true;
  }
  return false;
#endif
}

int ICM456xx::getPedometer(uint32_t& step_count, float& step_cadence, char*& activity)
{
  int rc = 0;

  if(apex_enable[ICM456XX_APEX_PEDOMETER] == false)
    return INV_ERROR;

#if (INV_DEVICE_TYPE == INV_TYPE_A1) || (INV_DEVICE_TYPE == INV_TYPE_A2)
  rc |= updateApex();

  /* Pedometer */
  if (apex_status.INV_STEP_CNT_OVFL) {
    apex_status.INV_STEP_CNT_OVFL = 0;
    step_cnt_ovflw++;
  }
  if (apex_status.INV_STEP_DET) {
    inv_imu_edmp_pedometer_data_t ped_data;
    apex_status.INV_STEP_DET = 0;

    rc |= inv_imu_edmp_get_pedometer_data(&icm_driver, &ped_data);
    if (rc == INV_IMU_OK) {
      step_count = (uint32_t)ped_data.step_cnt + (step_cnt_ovflw * UINT16_MAX);  
      if(ped_data.step_cadence != 0)
      {
        step_cadence = 200 / (float)ped_data.step_cadence;
      } else {
        step_cadence = 0;
      }
      activity = ped_data.activity_class == INV_IMU_EDMP_RUN ?
                                   (char*)"Run" :
                                   (ped_data.activity_class == INV_IMU_EDMP_WALK ?
                                        (char*)"Walk" :
                                        /* INV_IMU_EDMP_UNKOWN */ (char*)"Unknown");
      return true;
    } else {
        return INV_ERROR;
    }
  } else {
    return false;
  }
#endif /* INV_TYPE_A1 & INV_TYPE_A2 */
}

int ICM456xx::getRaiseToWake(void)
{
  if(apex_enable[ICM456XX_APEX_R2W] == false)
    return INV_ERROR;

#if (INV_DEVICE_TYPE == INV_TYPE_A1) || (INV_DEVICE_TYPE == INV_TYPE_A2)
  updateApex();
  if (apex_status.INV_R2W)
  {
    apex_status.INV_R2W = 0;
    return 1;
  } else if (apex_status.INV_R2W_SLEEP) {
    apex_status.INV_R2W_SLEEP = 0;
    return 2;
  } else {
    return false;
  }
#endif
}

int ICM456xx::startFreeFall(uint8_t intpin, ICM456xx_irq_handler handler)
{
  int rc = 0;

  apex_enable[ICM456XX_APEX_FF] = true;

  if(handler != NULL)
  {
    rc |= setApexInterrupt(intpin, handler);
    rc |= startAPEX(DMP_EXT_SEN_ODR_CFG_APEX_ODR_400_HZ,ACCEL_CONFIG0_ACCEL_ODR_400_HZ);
  }
  return rc;
}

int ICM456xx::startHighG(uint8_t intpin, ICM456xx_irq_handler handler)
{
  int rc = 0;

  apex_enable[ICM456XX_APEX_HIGHG] = true;

  if(handler != NULL)
  {  
    rc |= setApexInterrupt(intpin, handler);
    rc |= startAPEX(DMP_EXT_SEN_ODR_CFG_APEX_ODR_400_HZ,ACCEL_CONFIG0_ACCEL_ODR_400_HZ);
  }
  return rc;
}

int ICM456xx::startLowG(uint8_t intpin, ICM456xx_irq_handler handler)
{
  int rc = 0;

  apex_enable[ICM456XX_APEX_LOWG] = true;

  if(handler != NULL)
  {
    rc |= setApexInterrupt(intpin, handler);
    rc |= startAPEX(DMP_EXT_SEN_ODR_CFG_APEX_ODR_400_HZ,ACCEL_CONFIG0_ACCEL_ODR_400_HZ);
  }
  return rc;
}

int ICM456xx::startWakeOnMotion(uint8_t intpin, ICM456xx_irq_handler handler)
{
  int rc = 0;
  inv_imu_int_state_t config_int;

  rc |= setApexInterrupt(intpin, handler);

  inv_imu_get_config_int(&icm_driver,INV_IMU_INT1, &config_int);
  config_int.INV_WOM_X = INV_IMU_ENABLE;
  config_int.INV_WOM_Y = INV_IMU_ENABLE;
  config_int.INV_WOM_Z = INV_IMU_ENABLE;
  config_int.INV_EDMP_EVENT = INV_IMU_DISABLE;
  inv_imu_set_config_int(&icm_driver,INV_IMU_INT1, &config_int);

  /* Disabled All APEX features */
  for(int i=0; i < ICM456XX_APEX_MAX; i++)
  {
    apex_enable[i] = false;
  }
  
  rc |= startAPEX(DMP_EXT_SEN_ODR_CFG_APEX_ODR_50_HZ,ACCEL_CONFIG0_ACCEL_ODR_50_HZ);
  rc |= inv_imu_adv_configure_wom(&icm_driver, DEFAULT_WOM_THS_MG, DEFAULT_WOM_THS_MG,
          DEFAULT_WOM_THS_MG, TMST_WOM_CONFIG_WOM_INT_MODE_ANDED,
          TMST_WOM_CONFIG_WOM_INT_DUR_1_SMPL);
  rc |= inv_imu_adv_enable_wom(&icm_driver);
  rc |= inv_imu_edmp_enable(&icm_driver);
  rc |= inv_imu_adv_enable_accel_ln(&icm_driver);

  return rc;
}

int ICM456xx::startTap(uint8_t intpin, ICM456xx_irq_handler handler)
{
  int rc = 0;

  apex_enable[ICM456XX_APEX_TAP] = true;

  if(handler != NULL)
  {
    rc |= setApexInterrupt(intpin, handler);
    rc |= startAPEX(DMP_EXT_SEN_ODR_CFG_APEX_ODR_400_HZ,ACCEL_CONFIG0_ACCEL_ODR_400_HZ);
  }
  return rc;
}

int ICM456xx::startB2S(uint8_t intpin, ICM456xx_irq_handler handler)
{
  int rc = 0;

  apex_enable[ICM456XX_APEX_B2S] = true;

  if(handler != NULL)
  {  
    rc |= setApexInterrupt(intpin, handler);
    rc |= startAPEX(DMP_EXT_SEN_ODR_CFG_APEX_ODR_400_HZ,ACCEL_CONFIG0_ACCEL_ODR_400_HZ);
  }
  return rc;
}

int ICM456xx::startAID(uint8_t intpin, ICM456xx_irq_handler handler)
{
  int rc = 0;

  apex_enable[ICM456XX_APEX_AID] = true;

  if(handler != NULL)
  {  
    rc |= setApexInterrupt(intpin, handler);
    rc |= startAPEX(DMP_EXT_SEN_ODR_CFG_APEX_ODR_400_HZ,ACCEL_CONFIG0_ACCEL_ODR_400_HZ);
  }
  return rc;
}


int ICM456xx::getTap(uint8_t& tap_count, uint8_t& axis, uint8_t& direction)
{
  int rc = 0;

  if(apex_enable[ICM456XX_APEX_TAP] == false)
    return INV_ERROR;

  rc |= updateApex();

  if (apex_status.INV_TAP) {
    inv_imu_edmp_tap_data_t tap_data;
    apex_status.INV_TAP = 0;
    rc |= inv_imu_edmp_get_tap_data(&icm_driver, &tap_data);
    tap_count = tap_data.num;
    axis = tap_data.axis;
    direction = tap_data.direction;
    return true;
  } else {
    return false;
  }
}

int ICM456xx::getFreefall(uint32_t& duration_ms)
{
  int rc = 0;

  if(apex_enable[ICM456XX_APEX_FF] == false)
    return INV_ERROR;

  rc |= updateApex();

  if (apex_status.INV_FF) {
	uint16_t duration;
    apex_status.INV_FF = 0;
    rc |= inv_imu_edmp_get_ff_data(&icm_driver, &duration);
	duration_ms = (duration * 2500) / 1000;
    return true;
  } else {
    return false;
  }
}

int ICM456xx::getHighG(void)
{
  int rc = 0;

  if(apex_enable[ICM456XX_APEX_HIGHG] == false)
    return INV_ERROR;

  rc |= updateApex();

  if (apex_status.INV_HIGHG) {
    apex_status.INV_HIGHG = 0;
    return true;
  } else {
    return false;
  }
}

int ICM456xx::getLowG(void)
{
  int rc = 0;

  if(apex_enable[ICM456XX_APEX_LOWG] == false)
    return INV_ERROR;

  rc |= updateApex();

  if (apex_status.INV_LOWG) {
    apex_status.INV_LOWG = 0;  	
    return true;
  } else {
    return false;
  }
}

int ICM456xx::getB2S(void)
{
  int rc = 0;

  if(apex_enable[ICM456XX_APEX_B2S] == false)
    return INV_ERROR;

  rc |= updateApex();

  if (apex_status.INV_B2S) {
    apex_status.INV_B2S = 0;
    return 1;
  } 

  if(apex_status.INV_B2S_REV) {
    apex_status.INV_B2S_REV= 0;
    return 2;
  }

  return false;
}

int ICM456xx::getAID_Device(void)
{
  int rc = 0;
  uint8_t output_state;

  if(apex_enable[ICM456XX_APEX_AID] == false)
    return INV_ERROR;

  rc |= updateApex();

#if (INV_DEVICE_TYPE == INV_TYPE_B1)
  if (apex_status.INV_AID_DEVICE) {
    apex_status.INV_AID_DEVICE = 0;
	rc |= inv_imu_edmp_get_aid_data_device(&icm_driver, &output_state);
	/*	Decision taken by the algorithm,
	    1 => activity is detected, 
	    2=> inactivity is detected, 
	    6=> inactivity and sedentary alert are detected */
    return output_state;
  }
#endif
  return false;
}

int ICM456xx::getAID_Human(void)
{
  int rc = 0;
  uint8_t output_state;

  if(apex_enable[ICM456XX_APEX_AID] == false)
    return INV_ERROR;

  rc |= updateApex();

#if (INV_DEVICE_TYPE == INV_TYPE_B1)
  if (apex_status.INV_AID_HUMAN) {
    apex_status.INV_AID_HUMAN = 0;
	rc |= inv_imu_edmp_get_aid_data_human(&icm_driver, &output_state);
	/*	Decision taken by the algorithm,
	    1 => activity is detected, 
	    2=> inactivity is detected, 
	    6=> inactivity and sedentary alert are detected */
    return output_state;
  }
#endif
  return false;
}

int ICM456xx::updateApex(void)
{
  int rc = 0;
  inv_imu_int_state_t      int_state;
  inv_imu_edmp_int_state_t apex_state = { 0 };

  /* Read interrupt status */
  rc |= inv_imu_get_int_status(&icm_driver, INV_IMU_INT1, &int_state);

  /* If APEX status is set */
  if (int_state.INV_EDMP_EVENT) {
    /* Read APEX interrupt status */
    rc |= inv_imu_edmp_get_int_apex_status(&icm_driver, &apex_state);
#if (INV_DEVICE_TYPE == INV_TYPE_A1) || (INV_DEVICE_TYPE == INV_TYPE_A2)
    apex_status.INV_STEP_CNT_OVFL |= apex_state.INV_STEP_CNT_OVFL;
    apex_status.INV_STEP_DET |= apex_state.INV_STEP_DET;
    apex_status.INV_TILT_DET |= apex_state.INV_TILT_DET;
    apex_status.INV_R2W |= apex_state.INV_R2W;
    apex_status.INV_R2W_SLEEP |= apex_state.INV_R2W_SLEEP;
#elif (INV_DEVICE_TYPE == INV_TYPE_B1) || (INV_DEVICE_TYPE == INV_TYPE_C1)
    apex_status.INV_AID_DEVICE |= apex_state.INV_AID_DEVICE;
    apex_status.INV_AID_HUMAN  |= apex_state.INV_AID_HUMAN;
#endif
    apex_status.INV_FF      |= apex_state.INV_FF;
    apex_status.INV_HIGHG   |= apex_state.INV_HIGHG;
    apex_status.INV_LOWG    |= apex_state.INV_LOWG;
    apex_status.INV_TAP     |= apex_state.INV_TAP;
    apex_status.INV_B2S     |= apex_state.INV_B2S;
    apex_status.INV_B2S_REV |= apex_state.INV_B2S_REV;

  }
  return rc;
}


static int i2c_write(uint8_t reg, const uint8_t * wbuffer, uint32_t wlen) {
  i2c->beginTransmission(i2c_address);
  i2c->write(reg);
  for(uint8_t i = 0; i < wlen; i++) {
    i2c->write(wbuffer[i]);
  }
  i2c->endTransmission();
  return 0;
}

static int i2c_read(uint8_t reg, uint8_t * rbuffer, uint32_t rlen) {
  uint16_t offset = 0;

  i2c->beginTransmission(i2c_address);
  i2c->write(reg);
  i2c->endTransmission(false);
  while(offset < rlen)
  {
    uint16_t rx_bytes = 0;
    uint8_t length = ((rlen - offset) > ARDUINO_I2C_BUFFER_LENGTH) ? ARDUINO_I2C_BUFFER_LENGTH : (rlen - offset) ;
    rx_bytes = i2c->requestFrom(i2c_address, length);
    if (rx_bytes == length) {
      for(uint8_t i = 0; i < length; i++) {
        rbuffer[offset+i] = i2c->read();
      }
      offset += length;
    }
  }

  if(offset == rlen)
  {
    return 0;
  } else {
    return -1;
  }
}

static int spi_write(uint8_t reg, const uint8_t * wbuffer, uint32_t wlen) {
  spi->beginTransaction(SPISettings(clk_freq, MSBFIRST, SPI_MODE3));
  digitalWrite(chip_select_id,LOW);
  spi->transfer(reg);
  for(uint8_t i = 0; i < wlen; i++) {
    spi->transfer(wbuffer[i]);
  }
  digitalWrite(chip_select_id,HIGH);
  spi->endTransaction();
  return 0;
}

static int spi_read(uint8_t reg, uint8_t * rbuffer, uint32_t rlen) {
  spi->beginTransaction(SPISettings(clk_freq, MSBFIRST, SPI_MODE3));
  digitalWrite(chip_select_id,LOW);
  spi->transfer(reg | SPI_READ);
  spi->transfer(rbuffer,rlen);
  digitalWrite(chip_select_id,HIGH);
  spi->endTransaction();
  return 0;
}

accel_config0_accel_ui_fs_sel_t ICM456xx::accel_fsr_g_to_param(uint16_t accel_fsr_g) {
  accel_config0_accel_ui_fs_sel_t ret = ACCEL_CONFIG0_ACCEL_UI_FS_SEL_16_G;

  switch(accel_fsr_g) {
  case 2:  ret = ACCEL_CONFIG0_ACCEL_UI_FS_SEL_2_G;  break;
  case 4:  ret = ACCEL_CONFIG0_ACCEL_UI_FS_SEL_4_G;  break;
  case 8:  ret = ACCEL_CONFIG0_ACCEL_UI_FS_SEL_8_G;  break;
  case 16: ret = ACCEL_CONFIG0_ACCEL_UI_FS_SEL_16_G; break;
#if INV_IMU_HIGH_FSR_SUPPORTED
  case 32: ret = ACCEL_CONFIG0_ACCEL_UI_FS_SEL_32_G; break;
#endif
  default:
    /* Unknown accel FSR. Set to default 16G */
    break;
  }
  return ret;
}

gyro_config0_gyro_ui_fs_sel_t ICM456xx::gyro_fsr_dps_to_param(uint16_t gyro_fsr_dps) {
  gyro_config0_gyro_ui_fs_sel_t ret = GYRO_CONFIG0_GYRO_UI_FS_SEL_2000_DPS;

  switch(gyro_fsr_dps) {
  case 15:  ret = GYRO_CONFIG0_GYRO_UI_FS_SEL_15_625_DPS;  break;
  case 31:  ret = GYRO_CONFIG0_GYRO_UI_FS_SEL_31_25_DPS;  break;
  case 62:  ret = GYRO_CONFIG0_GYRO_UI_FS_SEL_62_5_DPS;  break;
  case 125:  ret = GYRO_CONFIG0_GYRO_UI_FS_SEL_125_DPS;  break;
  case 250:  ret = GYRO_CONFIG0_GYRO_UI_FS_SEL_250_DPS;  break;
  case 500:  ret = GYRO_CONFIG0_GYRO_UI_FS_SEL_500_DPS;  break;
  case 1000: ret = GYRO_CONFIG0_GYRO_UI_FS_SEL_1000_DPS; break;
  case 2000: ret = GYRO_CONFIG0_GYRO_UI_FS_SEL_2000_DPS; break;
#if INV_IMU_HIGH_FSR_SUPPORTED
  case 4000: ret = GYRO_CONFIG0_GYRO_UI_FS_SEL_4000_DPS; break;
#endif
  default:
    /* Unknown gyro FSR. Set to default 2000dps" */
    break;
  }
  return ret;
}

accel_config0_accel_odr_t ICM456xx::accel_freq_to_param(uint16_t accel_freq_hz) {
  accel_config0_accel_odr_t ret = ACCEL_CONFIG0_ACCEL_ODR_100_HZ;

  switch(accel_freq_hz) {
  case 1:    ret = ACCEL_CONFIG0_ACCEL_ODR_1_5625_HZ;  break;
  case 3:    ret = ACCEL_CONFIG0_ACCEL_ODR_3_125_HZ;  break;
  case 6:    ret = ACCEL_CONFIG0_ACCEL_ODR_6_25_HZ;  break;
  case 12:   ret = ACCEL_CONFIG0_ACCEL_ODR_12_5_HZ;  break;
  case 25:   ret = ACCEL_CONFIG0_ACCEL_ODR_25_HZ;  break;
  case 50:   ret = ACCEL_CONFIG0_ACCEL_ODR_50_HZ;  break;
  case 100:  ret = ACCEL_CONFIG0_ACCEL_ODR_100_HZ; break;
  case 200:  ret = ACCEL_CONFIG0_ACCEL_ODR_200_HZ; break;
  case 400:  ret = ACCEL_CONFIG0_ACCEL_ODR_400_HZ; break;
  case 800:  ret = ACCEL_CONFIG0_ACCEL_ODR_800_HZ; break;
  case 1600: ret = ACCEL_CONFIG0_ACCEL_ODR_1600_HZ;  break;
  case 3200: ret = ACCEL_CONFIG0_ACCEL_ODR_3200_HZ;  break;
  case 6400: ret = ACCEL_CONFIG0_ACCEL_ODR_6400_HZ;  break;
  default:
    /* Unknown accel frequency. Set to default 100Hz */
    break;
  }
  return ret;
}

gyro_config0_gyro_odr_t ICM456xx::gyro_freq_to_param(uint16_t gyro_freq_hz) {
  gyro_config0_gyro_odr_t ret = GYRO_CONFIG0_GYRO_ODR_100_HZ;

  switch(gyro_freq_hz) {
  case 1:   ret = GYRO_CONFIG0_GYRO_ODR_1_5625_HZ;  break;
  case 3:   ret = GYRO_CONFIG0_GYRO_ODR_3_125_HZ;  break;
  case 6:   ret = GYRO_CONFIG0_GYRO_ODR_6_25_HZ;  break;
  case 12:   ret = GYRO_CONFIG0_GYRO_ODR_12_5_HZ;  break;
  case 25:   ret = GYRO_CONFIG0_GYRO_ODR_25_HZ;  break;
  case 50:   ret = GYRO_CONFIG0_GYRO_ODR_50_HZ;  break;
  case 100:  ret = GYRO_CONFIG0_GYRO_ODR_100_HZ; break;
  case 200:  ret = GYRO_CONFIG0_GYRO_ODR_200_HZ; break;
  case 400:  ret = GYRO_CONFIG0_GYRO_ODR_400_HZ; break;
  case 800:  ret = GYRO_CONFIG0_GYRO_ODR_800_HZ; break;
  case 1600: ret = GYRO_CONFIG0_GYRO_ODR_1600_HZ;  break;
  case 3200: ret = GYRO_CONFIG0_GYRO_ODR_3200_HZ;  break;
  case 6400: ret = GYRO_CONFIG0_GYRO_ODR_6400_HZ;  break;
  default:
    /* Unknown gyro ODR. Set to default 100Hz */
    break;
  }
  return ret;
}

static void sleep_us(uint32_t us)
{
    delayMicroseconds(us);
}


