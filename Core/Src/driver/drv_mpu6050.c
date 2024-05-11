/**
 * @file       drv_mpu6050.c
 * @copyright  Copyright (C) 2023 QuyLe Co., Ltd. All rights reserved.
 * @license    This project is released under the QuyLe License.
 * @version    v1.0.0
 * @date       2024-05-02
 * @author     Quy Le
 *
 * @brief      driver mup6050
 *
 * @note
 */

/* Includes ----------------------------------------------------------- */
#include "drv_mpu6050.h"

#include "kalman_filter.h"

#include <math.h>
/* Private defines ---------------------------------------------------- */

#define PI         3.14
#define RAD_TO_DEG 59.29577951

#define DRV_MPU6050_CHECK_ERROR(status_1, status_2) \
  if (!(status_1))                                  \
    return (status_2);

#define DRV_MPU6050_CHECK_PARA(status) \
  if (!(status))                       \
    return DRV_MPU6050_ERROR_PARAMETER;

/* Private enumerate/structure ---------------------------------------- */

/* Private macros ----------------------------------------------------- */

/* Public variables --------------------------------------------------- */

/* Private variables -------------------------------------------------- */

kalman_filter_t kalman_X;
kalman_filter_t kalman_Y;

static uint32_t prev_millis = 0;
static double   acc_X, acc_Y, gyro_X, gyro_Y, gyro_Z;
static double   gyro_X_offset, gyro_Y_offset, gyro_Z_offset;

/* Private function prototypes ---------------------------------------- */

static drv_mpu6050_err_t drv_mpu6050_read_data(drv_mpu6050_config_t *drv_mpu6050, uint8_t reg_read, uint8_t *data_read, uint16_t size_data)
{
  DRV_MPU6050_CHECK_ERROR((drv_mpu6050->i2c_read_at(drv_mpu6050->device_address, reg_read, data_read, size_data) == true), DRV_MPU6050_ERROR)
  return DRV_MPU6050_OK;
}

static drv_mpu6050_err_t drv_mpu6050_write_data(drv_mpu6050_config_t *drv_mpu6050, uint8_t reg_write, uint8_t *data_write, uint16_t size_data)
{
  DRV_MPU6050_CHECK_ERROR((drv_mpu6050->i2c_write_at(drv_mpu6050->device_address, reg_write, data_write, size_data) == true), DRV_MPU6050_ERROR)
  return DRV_MPU6050_OK;
}

static drv_mpu6050_err_t drv_mpu6050_is_ready(drv_mpu6050_config_t *drv_mpu6050)
{
  DRV_MPU6050_CHECK_PARA(drv_mpu6050 != NULL)
  DRV_MPU6050_CHECK_ERROR((drv_mpu6050->i2c_is_device_ready(drv_mpu6050->device_address) == true), DRV_MPU6050_ERROR)
  return DRV_MPU6050_OK;
}

/* Function definitions ----------------------------------------------- */

drv_mpu6050_err_t drv_mpu6050_init(drv_mpu6050_config_t *mpu6050_data, uint8_t device_number)
{
  uint8_t temp = 0;

  /* Format I2C address */
  mpu6050_data->address = MPU6050_I2C_ADDR | (uint8_t) device_number;

  /* Init I2C */
  DRV_MPU6050_CHECK_PARA(mpu6050_data != NULL && mpu6050_data->i2c_is_device_ready != NULL && mpu6050_data->i2c_read_at != NULL
                         && mpu6050_data->i2c_write_at != NULL)

  /* Check if device is connected */
  DRV_MPU6050_CHECK_ERROR((drv_mpu6050_is_ready(mpu6050_data) == DRV_MPU6050_OK), DRV_MPU6050_ERROR)

  /* Check who I am */
  drv_mpu6050_read_data(mpu6050_data, DRV_MPU6050_WHO_AM_I, &temp, 1);
  if (temp != DRV_MPU6050_I_AM)
  {
    /* Return error */
    return DRV_MPU6050_DEVICE_INVALID;
  }

  /* Wakeup MPU6050 */
  temp = 0x00;
  drv_mpu6050_write_data(mpu6050_data, DRV_MPU6050_PWR_MGMT_1, &temp, 1);

  /* 0x07 Set the sample rate to 1000Hz - 8kHz/(7+1) = 1000Hz */
  temp = 0x07;
  drv_mpu6050_write_data(mpu6050_data, DRV_MPU6050_SMPLRT_DIV, &temp, 1);

  /* Config MPU6050 */
  temp = 0x00;
  drv_mpu6050_write_data(mpu6050_data, DRV_MPU6050_CONFIG, &temp, 1);

  /* Config accelerometer */
  drv_mpu6050_read_data(mpu6050_data, DRV_MPU6050_ACCEL_CONFIG, &temp, 1);
  temp = (temp & 0xE7) | (uint8_t) mpu6050_data->accelerometer_config << 3;
  drv_mpu6050_write_data(mpu6050_data, DRV_MPU6050_ACCEL_CONFIG, &temp, 1);

  /* Config gyroscope */
  drv_mpu6050_read_data(mpu6050_data, DRV_MPU6050_GYRO_CONFIG, &temp, 1);
  temp = (temp & 0xE7) | (uint8_t) mpu6050_data->gyroscope_config << 3;
  drv_mpu6050_write_data(mpu6050_data, DRV_MPU6050_GYRO_CONFIG, &temp, 1);

  /* User control MPU6050 */
  temp = 0x00;
  drv_mpu6050_write_data(mpu6050_data, DRV_MPU6050_USER_CTRL, &temp, 1);

  /* Wakeup MPU6050 */
  temp = 0x01;
  drv_mpu6050_write_data(mpu6050_data, DRV_MPU6050_PWR_MGMT_1, &temp, 1);

  /* Set sensitivities for multiplying gyro and accelerometer data */
  switch (mpu6050_data->accelerometer_config)
  {
  case DRV_MPU6050_ACCELEROMETER_2G: mpu6050_data->acce_mult = (float) 1 / DRV_MPU6050_ACCE_SENS_2; break;
  case DRV_MPU6050_ACCELEROMETER_4G: mpu6050_data->acce_mult = (float) 1 / DRV_MPU6050_ACCE_SENS_4; break;
  case DRV_MPU6050_ACCELEROMETER_8G: mpu6050_data->acce_mult = (float) 1 / DRV_MPU6050_ACCE_SENS_8; break;
  case DRV_MPU6050_ACCELEROMETER_16G: mpu6050_data->acce_mult = (float) 1 / DRV_MPU6050_ACCE_SENS_16;
  default: break;
  }

  switch (mpu6050_data->gyroscope_config)
  {
  case DRV_MPU6050_GYROSCOPE_250s: mpu6050_data->gyro_mult = (float) 1 / DRV_MPU6050_GYRO_SENS_250; break;
  case DRV_MPU6050_GYROSCOPE_500s: mpu6050_data->gyro_mult = (float) 1 / DRV_MPU6050_GYRO_SENS_500; break;
  case DRV_MPU6050_GYROSCOPE_1000s: mpu6050_data->gyro_mult = (float) 1 / DRV_MPU6050_GYRO_SENS_1000; break;
  case DRV_MPU6050_GYROSCOPE_2000s: mpu6050_data->gyro_mult = (float) 1 / DRV_MPU6050_GYRO_SENS_2000;
  default: break;
  }

  /* Setup kalman filter config */
  kalman_filter_init(&kalman_X);
  kalman_filter_init(&kalman_Y);

  drv_mpu6050_read_angles(mpu6050_data);

  /* start angle filter */
  kalman_X.angle = mpu6050_data->angle_X;
  kalman_Y.angle = mpu6050_data->angle_Y;

  /* Return OK */
  return DRV_MPU6050_OK;
}

drv_mpu6050_err_t drv_mpu6050_read_accelerometer(drv_mpu6050_config_t *mpu6050_data)
{
  uint8_t data[6];

  /* Read accelerometer data */
  drv_mpu6050_read_data(mpu6050_data, DRV_MPU6050_ACCEL_XOUT_H, (uint8_t *) data, 6);

  /* Format */
  mpu6050_data->accelerometer_X = (int16_t) (data[0] << 8 | data[1]);
  mpu6050_data->accelerometer_Y = (int16_t) (data[2] << 8 | data[3]);
  mpu6050_data->accelerometer_Z = (int16_t) (data[4] << 8 | data[5]);

  /* Return OK */
  return DRV_MPU6050_OK;
}

drv_mpu6050_err_t drv_mpu6050_read_gyroscope(drv_mpu6050_config_t *mpu6050_data)
{
  uint8_t data[6];

  /* Read gyroscope data */
  drv_mpu6050_read_data(mpu6050_data, DRV_MPU6050_GYRO_XOUT_H, (uint8_t *) data, 6);

  /* Format */
  mpu6050_data->gyroscope_X = (int16_t) (data[0] << 8 | data[1]);
  mpu6050_data->gyroscope_Y = (int16_t) (data[2] << 8 | data[3]);
  mpu6050_data->gyroscope_Z = (int16_t) (data[4] << 8 | data[5]);

  /* Return OK */
  return DRV_MPU6050_OK;
}

drv_mpu6050_err_t drv_mpu6050_read_temperature(drv_mpu6050_config_t *mpu6050_data)
{
  uint8_t data[2];
  int16_t temp;

  /* Read temperature */
  drv_mpu6050_read_data(mpu6050_data, DRV_MPU6050_TEMP_OUT_H, (uint8_t *) data, 2);

  /* Format temperature */
  temp                      = (data[0] << 8 | data[1]);
  mpu6050_data->temperature = (float) ((int16_t) temp / (float) 340.0 + (float) 36.53);

  /* Return OK */
  return DRV_MPU6050_OK;
}

drv_mpu6050_err_t drv_mpu6050_read_all(drv_mpu6050_config_t *mpu6050_data)
{
  uint8_t data[14];
  int16_t temp;

  /* Read full raw data, 14bytes */
  drv_mpu6050_read_data(mpu6050_data, DRV_MPU6050_ACCEL_XOUT_H, (uint8_t *) data, 14);

  /* Format accelerometer data */
  mpu6050_data->accelerometer_X = (int16_t) (data[0] << 8 | data[1]);
  mpu6050_data->accelerometer_Y = (int16_t) (data[2] << 8 | data[3]);
  mpu6050_data->accelerometer_Z = (int16_t) (data[4] << 8 | data[5]);

  /* Format temperature */
  temp                      = (data[6] << 8 | data[7]);
  mpu6050_data->temperature = (float) ((float) ((int16_t) temp) / (float) 340.0 + (float) 36.53);

  /* Format gyroscope data */
  mpu6050_data->gyroscope_X = (int16_t) (data[8] << 8 | data[9]);
  mpu6050_data->gyroscope_Y = (int16_t) (data[10] << 8 | data[11]);
  mpu6050_data->gyroscope_Z = (int16_t) (data[12] << 8 | data[13]);

  /* Return OK */
  return DRV_MPU6050_OK;
}

drv_mpu6050_err_t drv_mpu6050_read_angles(drv_mpu6050_config_t *mpu6050_data)
{
  uint8_t data[14];
  int16_t temp;

  /* Check parameter */
  DRV_MPU6050_CHECK_PARA(mpu6050_data != NULL && mpu6050_data->i2c_is_device_ready != NULL && mpu6050_data->i2c_read_at != NULL
                         && mpu6050_data->i2c_write_at != NULL)

  /* Check if device is connected */
  DRV_MPU6050_CHECK_ERROR((drv_mpu6050_is_ready(mpu6050_data) == DRV_MPU6050_OK), DRV_MPU6050_ERROR)

  /* Read full raw data, 14bytes */
  drv_mpu6050_read_data(mpu6050_data, DRV_MPU6050_ACCEL_XOUT_H, (uint8_t *) data, 14);

  /* Format accelerometer data */
  mpu6050_data->accelerometer_X = (int16_t) (data[0] << 8 | data[1]);
  mpu6050_data->accelerometer_Y = (int16_t) (data[2] << 8 | data[3]);
  mpu6050_data->accelerometer_Z = (int16_t) (data[4] << 8 | data[5]);

  /* Format temperature */
  temp                      = (data[6] << 8 | data[7]);
  mpu6050_data->temperature = (float) ((float) ((int16_t) temp) / (float) 340.0 + (float) 36.53);

  /* Format gyroscope data */
  mpu6050_data->gyroscope_X = (int16_t) (data[8] << 8 | data[9]);
  mpu6050_data->gyroscope_Y = (int16_t) (data[10] << 8 | data[11]);
  mpu6050_data->gyroscope_Z = (int16_t) (data[12] << 8 | data[13]);

  acc_X = atan((mpu6050_data->accelerometer_Y * mpu6050_data->acce_mult)
               / sqrt(pow((mpu6050_data->accelerometer_X * mpu6050_data->acce_mult), 2) + pow((mpu6050_data->accelerometer_Z * mpu6050_data->acce_mult), 2)))
          * RAD_TO_DEG;
  acc_Y = atan(-1 * (mpu6050_data->accelerometer_X * mpu6050_data->acce_mult)
               / sqrt(pow((mpu6050_data->accelerometer_Y * mpu6050_data->acce_mult), 2) + pow((mpu6050_data->accelerometer_Z * mpu6050_data->acce_mult), 2)))
          * RAD_TO_DEG;

  gyro_X = (mpu6050_data->gyroscope_X + gyro_X_offset) * mpu6050_data->gyro_mult;
  gyro_Y = (mpu6050_data->gyroscope_Y + gyro_Y_offset) * mpu6050_data->gyro_mult;
  gyro_Z = (mpu6050_data->gyroscope_Z + gyro_Z_offset) * mpu6050_data->gyro_mult;

  uint32_t curMillis = HAL_GetTick();
  double   duration  = (curMillis - prev_millis) * 1e-3;
  prev_millis        = curMillis;

  mpu6050_data->angle_X = 0.98 * (mpu6050_data->angle_X + gyro_X * duration) + 0.02 * acc_X;
  mpu6050_data->angle_Y = 0.98 * (mpu6050_data->angle_Y + gyro_Y * duration) + 0.02 * acc_Y;
  mpu6050_data->angle_Z = mpu6050_data->angle_Z + gyro_Z * duration;

  /* Return OK */
  return DRV_MPU6050_OK;
}

drv_mpu6050_err_t drv_mpu6050_read_angles_filter(drv_mpu6050_config_t *mpu6050_data)
{
  uint8_t data[14];
  int16_t temp;

  /* Check parameter */
  DRV_MPU6050_CHECK_PARA(mpu6050_data != NULL && mpu6050_data->i2c_is_device_ready != NULL && mpu6050_data->i2c_read_at != NULL
                         && mpu6050_data->i2c_write_at != NULL)

  /* Check if device is connected */
  DRV_MPU6050_CHECK_ERROR((drv_mpu6050_is_ready(mpu6050_data) == DRV_MPU6050_OK), DRV_MPU6050_ERROR)

  /* Read full raw data, 14bytes */
  drv_mpu6050_read_data(mpu6050_data, DRV_MPU6050_ACCEL_XOUT_H, (uint8_t *) data, 14);

  /* Format accelerometer data */
  mpu6050_data->accelerometer_X = (int16_t) (data[0] << 8 | data[1]);
  mpu6050_data->accelerometer_Y = (int16_t) (data[2] << 8 | data[3]);
  mpu6050_data->accelerometer_Z = (int16_t) (data[4] << 8 | data[5]);

  /* Format temperature */
  temp                      = (data[6] << 8 | data[7]);
  mpu6050_data->temperature = (float) ((float) ((int16_t) temp) / (float) 340.0 + (float) 36.53);

  /* Format gyroscope data */
  mpu6050_data->gyroscope_X = (int16_t) (data[8] << 8 | data[9]);
  mpu6050_data->gyroscope_Y = (int16_t) (data[10] << 8 | data[11]);
  mpu6050_data->gyroscope_Z = (int16_t) (data[12] << 8 | data[13]);

  acc_X = atan((mpu6050_data->accelerometer_Y * mpu6050_data->acce_mult)
               / sqrt(pow((mpu6050_data->accelerometer_X * mpu6050_data->acce_mult), 2) + pow((mpu6050_data->accelerometer_Z * mpu6050_data->acce_mult), 2)))
          * RAD_TO_DEG;
  acc_Y = atan(-1 * (mpu6050_data->accelerometer_X * mpu6050_data->acce_mult)
               / sqrt(pow((mpu6050_data->accelerometer_Y * mpu6050_data->acce_mult), 2) + pow((mpu6050_data->accelerometer_Z * mpu6050_data->acce_mult), 2)))
          * RAD_TO_DEG;

  gyro_X = (mpu6050_data->gyroscope_X + gyro_X_offset) * mpu6050_data->gyro_mult;
  gyro_Y = (mpu6050_data->gyroscope_Y + gyro_Y_offset) * mpu6050_data->gyro_mult;
  gyro_Z = (mpu6050_data->gyroscope_Z + gyro_Z_offset) * mpu6050_data->gyro_mult;

  uint32_t curMillis = HAL_GetTick();
  double   duration  = (curMillis - prev_millis) * 1e-3;
  prev_millis        = curMillis;

  // This fixes the transition problem when the accelerometer angle jumps between -180 and 180 degrees
  if ((acc_Y < -90 && mpu6050_data->kalman_angle_Y > 90) || (acc_Y > 90 && mpu6050_data->kalman_angle_Y < -90))
  {
    kalman_Y.angle               = acc_Y;
    mpu6050_data->kalman_angle_Y = acc_Y;
  }
  else
    mpu6050_data->kalman_angle_Y = kalman_filter_get_angle(&kalman_Y, acc_Y, gyro_Y, duration);  // Calculate the angle using a Kalman filter

  if (fabs(mpu6050_data->kalman_angle_Y) > 90)
    gyro_X = -gyro_X;                                                                          // Invert rate, so it fits the restriced accelerometer reading
  mpu6050_data->kalman_angle_X = kalman_filter_get_angle(&kalman_X, acc_X, gyro_X, duration);  // Calculate the angle using a Kalman filter

  /* Return OK */
  return DRV_MPU6050_OK;
}

drv_mpu6050_err_t drv_mpu6050_read_angles_atan2(drv_mpu6050_config_t *mpu6050_data)
{
  uint8_t data[14];
  int16_t temp;

  /* Check parameter */
  DRV_MPU6050_CHECK_PARA(mpu6050_data != NULL && mpu6050_data->i2c_is_device_ready != NULL && mpu6050_data->i2c_read_at != NULL
                         && mpu6050_data->i2c_write_at != NULL)

  /* Check if device is connected */
  DRV_MPU6050_CHECK_ERROR((drv_mpu6050_is_ready(mpu6050_data) == DRV_MPU6050_OK), DRV_MPU6050_ERROR)

  /* Read full raw data, 14bytes */
  drv_mpu6050_read_data(mpu6050_data, DRV_MPU6050_ACCEL_XOUT_H, (uint8_t *) data, 14);

  /* Format accelerometer data */
  mpu6050_data->accelerometer_X = (int16_t) (data[0] << 8 | data[1]);
  mpu6050_data->accelerometer_Y = (int16_t) (data[2] << 8 | data[3]);
  mpu6050_data->accelerometer_Z = (int16_t) (data[4] << 8 | data[5]);

  /* Format temperature */
  temp                      = (data[6] << 8 | data[7]);
  mpu6050_data->temperature = (float) ((float) ((int16_t) temp) / (float) 340.0 + (float) 36.53);

  /* Format gyroscope data */
  mpu6050_data->gyroscope_X = (int16_t) (data[8] << 8 | data[9]);
  mpu6050_data->gyroscope_Y = (int16_t) (data[10] << 8 | data[11]);
  mpu6050_data->gyroscope_Z = (int16_t) (data[12] << 8 | data[13]);

  /* Calculate angles using atan2 method*/
  acc_X = atan2(mpu6050_data->accelerometer_Y * mpu6050_data->acce_mult, mpu6050_data->accelerometer_Z * mpu6050_data->acce_mult) * RAD_TO_DEG;
  acc_Y = atan2(mpu6050_data->accelerometer_X * mpu6050_data->acce_mult, mpu6050_data->accelerometer_Z * mpu6050_data->acce_mult) * RAD_TO_DEG;

  gyro_X = (mpu6050_data->gyroscope_X + gyro_X_offset) * mpu6050_data->gyro_mult;
  gyro_Y = (mpu6050_data->gyroscope_Y + gyro_Y_offset) * mpu6050_data->gyro_mult;
  gyro_Z = (mpu6050_data->gyroscope_Z + gyro_Z_offset) * mpu6050_data->gyro_mult;

  uint32_t curMillis = HAL_GetTick();
  double   duration  = (curMillis - prev_millis) * 1e-3;
  prev_millis        = curMillis;

  mpu6050_data->angle_X = 0.98 * (mpu6050_data->angle_X + gyro_X * duration) + 0.02 * acc_X;
  mpu6050_data->angle_Y = 0.98 * (mpu6050_data->angle_Y + gyro_Y * duration) + 0.02 * acc_Y;
  mpu6050_data->angle_Z += gyro_Z * duration;

  /* Return OK */
  return DRV_MPU6050_OK;
}

/* End of file -------------------------------------------------------- */
