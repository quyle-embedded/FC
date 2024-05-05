/**
 * @file       system_angle.c
 * @copyright  Copyright (C) 2019 QuyLe Co., Ltd. All rights reserved.
 * @license    This project is released under the QuyLe License.
 * @version    v1.0.0
 * @date       2024-05-03
 * @author     QuyLe
 * @author     QuyLe
 *
 * @brief      System manager angle data
 *
 * @note       None
 * @example    None
 */

/* Includes ----------------------------------------------------------- */
#include "system_angle.h"

#include "drv_mpu6050.h"

/* Private defines ---------------------------------------------------- */

#define SYSTEM_ANGLE_CHECK_ERROR(status_1, status_2) \
  if (!(status_1))                                   \
    return (status_2);

/* Private enumerate/structure ---------------------------------------- */

/* Private macros ----------------------------------------------------- */

/* Public variables --------------------------------------------------- */

/* Private variables -------------------------------------------------- */

drv_mpu6050_config_t mpu6050;

/* Private function prototypes ---------------------------------------- */

/* Function definitions ----------------------------------------------- */

system_angle_error_t system_angle_init(void)
{
  mpu6050.device_address      = 0xD0;
  mpu6050.i2c_is_device_ready = bsp_i2c1_is_device_ready;
  mpu6050.i2c_read_at         = bsp_i2c1_read_mem;
  mpu6050.i2c_write_at        = bsp_i2c1_write_mem;

  mpu6050.accelerometer_config = DRV_MPU6050_ACCELEROMETER_2G;
  mpu6050.gyroscope_config = DRV_MPU6050_GYROSCOPE_250s;

  SYSTEM_ANGLE_CHECK_ERROR(drv_mpu6050_init(&mpu6050, 0) == DRV_MPU6050_OK, SYSTEM_ANGLE_ERROR)
  return SYSTEM_ANGLE_OK;
}

system_angle_error_t system_angle_get_value(float *angle_X, float *angle_Y, float *angle_Z)
{
  SYSTEM_ANGLE_CHECK_ERROR(drv_mpu6050_read_angles(&mpu6050) == DRV_MPU6050_OK, SYSTEM_ANGLE_ERROR)

  *angle_X = mpu6050.angle_X;
  *angle_Y = mpu6050.angle_Y;
  *angle_Z = mpu6050.angle_Z;

  return SYSTEM_ANGLE_OK;
}

/* End of file -------------------------------------------------------- */
