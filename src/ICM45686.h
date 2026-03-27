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
 
#ifndef ICM456xx_H
#define ICM456xx_H

#include "Arduino.h"
#include "SPI.h"
#include "Wire.h"

extern "C" {
  #include "imu/inv_imu_driver_advanced.h"
  #include "imu/inv_imu_edmp.h"
  #include "Invn/InvError.h"
}

/* Device Type Definitions */
#define INV_TYPE_A1    0
#define INV_TYPE_A2    1
#define INV_TYPE_B1    2
#define INV_TYPE_C1    4

#if defined(ICM45605) || defined(ICM45686) || defined(ICM45688P)
  #define INV_DEVICE_TYPE INV_TYPE_A1
#elif  defined(ICM45605S) || defined(ICM45686S)
  #define INV_DEVICE_TYPE INV_TYPE_A2
#elif defined(ICM45606) || defined(ICM45687) || defined(ICM45685)
  #define INV_DEVICE_TYPE INV_TYPE_B1
#elif defined(ICM45689) || defined(ICM45608)
  #define INV_DEVICE_TYPE INV_TYPE_C1
#endif

#if (INV_DEVICE_TYPE == INV_TYPE_A2)
  #include "imu/inv_imu_edmp_gaf.h"
#endif

enum {
  ACCEL = 0,
  GYRO,
  MAG
};

// algo 0, enable GRV when enable 6-axis(AG)
// algo 1, enable GMRV when enable 6-axis(AM)
// algo 2, enable RV when enable 9-axis(AGM)
enum {
  ALGO_GRV = 0,
  ALGO_GMRV,
  ALGO_RV
};

enum {
  ICM456XX_APEX_TILT=0,
  ICM456XX_APEX_PEDOMETER,
  ICM456XX_APEX_TAP,
  ICM456XX_APEX_R2W,
  ICM456XX_APEX_B2S,
  ICM456XX_APEX_AID,
  ICM456XX_APEX_FF,
  ICM456XX_APEX_LOWG,
  ICM456XX_APEX_HIGHG,
  ICM456XX_APEX_MAX
};

// This defines the handler called when retrieving a sample from the FIFO
//typedef void (*ICM456xx_sensor_event_cb)(inv_imu_sensor_data_t *event);
// This defines the handler called when receiving an irq
typedef void (*ICM456xx_irq_handler)(void);

class ICM456xx {
  public:
    ICM456xx(TwoWire &i2c,bool address_lsb, uint32_t freq);
    ICM456xx(TwoWire &i2c,bool address_lsb);
    ICM456xx(SPIClass &spi,uint8_t chip_select_id, uint32_t freq);
    ICM456xx(SPIClass &spi,uint8_t chip_select_id);
    int begin();
    int startAccel(uint16_t odr, uint16_t fsr);
    int startGyro(uint16_t odr, uint16_t fsr);
    int getDataFromRegisters(inv_imu_sensor_data_t& data);
    int enableFifoInterrupt(uint8_t intpin, ICM456xx_irq_handler handler, uint8_t fifo_watermark);
    int getDataFromFifo(inv_imu_fifo_data_t& data);
#if (INV_DEVICE_TYPE == INV_TYPE_A2) || (INV_DEVICE_TYPE == INV_TYPE_B1) || (INV_DEVICE_TYPE == INV_TYPE_C1)
    int startGaf(uint8_t intpin, ICM456xx_irq_handler handler, uint8_t algo);
    int getGafData(inv_imu_edmp_gaf_outputs_t& gaf_outputs);
    int getGaf_GRVData(float& quatW,float& quatX,float& quatY,float& quatZ);
	int getGaf_GMRVData(float& quatW, float& quatX, float& quatY, float& quatZ) {
        float _dummy;
        return getGaf_GMRVData(quatW, quatX, quatY, quatZ, _dummy);
    }
    int getGaf_GMRVData(float& quatW,float& quatX,float& quatY,float& quatZ,float& accuracy);
	int getGaf_RVData(float& quatW, float& quatX, float& quatY, float& quatZ) {
        float _dummy;
        return getGaf_RVData(quatW, quatX, quatY, quatZ, _dummy);
    }
    int getGaf_RVData(float& quatW,float& quatX,float& quatY,float& quatZ,float& accuracy);
    int getGaf_RMData(float& mX,float& mY, float& mZ);
    int getGaf_BiasData(int type, int& q12_X,int& q12_Y,int& q12_Z,int& accuracy);
#endif

#if (INV_DEVICE_TYPE == INV_TYPE_B1)
    int startVocalVibDet(uint8_t intpin, ICM456xx_irq_handler handler);
    int SetVocalVibDet_Dynthresh(uint32_t thresh);
    int GetVocalVibDet_nb_samples(void);
    void SetVocalVibDet_nb_samples(uint32_t sample);
    int CheckVocalVibDet_thresh(void);
    int GetVocalVibDet_thresh(void);
    void SetVocalVibDet_thresh(uint32_t thresh);
#endif

#if (INV_DEVICE_TYPE == INV_TYPE_A1) || (INV_DEVICE_TYPE == INV_TYPE_B1) || (INV_DEVICE_TYPE == INV_TYPE_C1)
    int adv_getDataFromFifo(void);
    int setI2CM(void);
    int setI2CMPassThrough(void);
    int getDataFromI2CM(uint8_t reg, uint8_t& data);
    int getDataFromPassThrough(uint8_t reg, uint8_t& data);
#endif
#if (INV_DEVICE_TYPE == INV_TYPE_A1)
    int setI2CM_FIFO(uint8_t intpin, ICM456xx_irq_handler handler);
    int getAdvDataFromFifo(int32_t *accel, int32_t *gyro, float *external);
    int getExternalMagData(float *mag);
    int initMag(void);
    int enableMag(int flag);
#endif
    int stopAccel(void);
    int stopGyro(void);
    int startTiltDetection(uint8_t intpin=2, ICM456xx_irq_handler handler=NULL);
    int startPedometer(uint8_t intpin=2, ICM456xx_irq_handler handler=NULL);
    int startFreeFall(uint8_t intpin=2, ICM456xx_irq_handler handler=NULL);
    int startHighG(uint8_t intpin=2, ICM456xx_irq_handler handler=NULL);
    int startLowG(uint8_t intpin=2, ICM456xx_irq_handler handler=NULL);
    int getPedometer(uint32_t& step_count, float& step_cadence, char*& activity);
    int startWakeOnMotion(uint8_t intpin, ICM456xx_irq_handler handler);
    int startTap(uint8_t intpin=2, ICM456xx_irq_handler handler=NULL);
    int startRaiseToWake(uint8_t intpin=2, ICM456xx_irq_handler handler=NULL);
    int startB2S(uint8_t intpin=2, ICM456xx_irq_handler handler=NULL);
    int startAID(uint8_t intpin=2, ICM456xx_irq_handler handler=NULL);

    int getTilt(void);
    int getTap(uint8_t& tap_count, uint8_t& axis, uint8_t& direction);
    int getRaiseToWake(void);
    int getFreefall(uint32_t& duration_ms);
    int getHighG(void);
    int getLowG(void);
    int getAID_Human(void);
    int getAID_Device(void);
    int getB2S(void);

    int updateApex(void);
    int setApexInterrupt(uint8_t intpin, ICM456xx_irq_handler handler);
    int startAPEX(dmp_ext_sen_odr_cfg_apex_odr_t edmp_odr=(dmp_ext_sen_odr_cfg_apex_odr_t)NULL, 
                      accel_config0_accel_odr_t accel_odr=(accel_config0_accel_odr_t)NULL);

    inv_imu_edmp_int_state_t apex_status;

  protected:
    inv_imu_device_t icm_driver;
    accel_config0_accel_odr_t accel_freq_to_param(uint16_t accel_freq_hz);
    gyro_config0_gyro_odr_t gyro_freq_to_param(uint16_t gyro_freq_hz);
    accel_config0_accel_ui_fs_sel_t accel_fsr_g_to_param(uint16_t accel_fsr_g);
    gyro_config0_gyro_ui_fs_sel_t gyro_fsr_dps_to_param(uint16_t gyro_fsr_dps);
    int setup_irq(uint8_t intpin, ICM456xx_irq_handler handler);
    uint32_t step_cnt_ovflw;
    bool apex_enable[ICM456XX_APEX_MAX];
    dmp_ext_sen_odr_cfg_apex_odr_t apex_edmp_odr;
    accel_config0_accel_odr_t apex_accel_odr;
#if (INV_DEVICE_TYPE == INV_TYPE_B1)
    int32_t	nb_samples_before_decision=0; /* To compute delay before notifying VVD window's end */
    int32_t	vvd_thresh=0;
#endif

};

#endif // ICM456xx_H
