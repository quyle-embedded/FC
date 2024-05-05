/**
 * @file       drv_mpu6050.h
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

/* Define to prevent recursive inclusion ------------------------------ */
#ifndef __DRV_MPU6050_H
#define __DRV_MPU6050_H

/* Includes ----------------------------------------------------------- */
#include "bsp_i2c.h"

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
/* Public defines ----------------------------------------------------- */

/**
 * @defgroup MPU6050 LIB Macros
 * @brief    Library defines
 * @{
 */

/* Default I2C address */
#define MPU6050_I2C_ADDR              0xD0

/* Who I am register value */
#define DRV_MPU6050_I_AM              0x68

/* MPU6050 registers */
#define DRV_MPU6050_AUX_VDDIO         0x01
#define DRV_MPU6050_SMPLRT_DIV        0x19
#define DRV_MPU6050_CONFIG            0x1A
#define DRV_MPU6050_GYRO_CONFIG       0x1B
#define DRV_MPU6050_ACCEL_CONFIG      0x1C
#define DRV_MPU6050_MOTION_THRESH     0x1F
#define DRV_MPU6050_INT_PIN_CFG       0x37
#define DRV_MPU6050_INT_ENABLE        0x38
#define DRV_MPU6050_INT_STATUS        0x3A
#define DRV_MPU6050_ACCEL_XOUT_H      0x3B
#define DRV_MPU6050_ACCEL_XOUT_L      0x3C
#define DRV_MPU6050_ACCEL_YOUT_H      0x3D
#define DRV_MPU6050_ACCEL_YOUT_L      0x3E
#define DRV_MPU6050_ACCEL_ZOUT_H      0x3F
#define DRV_MPU6050_ACCEL_ZOUT_L      0x40
#define DRV_MPU6050_TEMP_OUT_H        0x41
#define DRV_MPU6050_TEMP_OUT_L        0x42
#define DRV_MPU6050_GYRO_XOUT_H       0x43
#define DRV_MPU6050_GYRO_XOUT_L       0x44
#define DRV_MPU6050_GYRO_YOUT_H       0x45
#define DRV_MPU6050_GYRO_YOUT_L       0x46
#define DRV_MPU6050_GYRO_ZOUT_H       0x47
#define DRV_MPU6050_GYRO_ZOUT_L       0x48
#define DRV_MPU6050_MOT_DETECT_STATUS 0x61
#define DRV_MPU6050_SIGNAL_PATH_RESET 0x68
#define DRV_MPU6050_MOT_DETECT_CTRL   0x69
#define DRV_MPU6050_USER_CTRL         0x6A
#define DRV_MPU6050_PWR_MGMT_1        0x6B
#define DRV_MPU6050_PWR_MGMT_2        0x6C
#define DRV_MPU6050_FIFO_COUNTH       0x72
#define DRV_MPU6050_FIFO_COUNTL       0x73
#define DRV_MPU6050_FIFO_R_W          0x74
#define DRV_MPU6050_WHO_AM_I          0x75

/* Gyro sensitivities in /s */
#define DRV_MPU6050_GYRO_SENS_250     ((float) 131)
#define DRV_MPU6050_GYRO_SENS_500     ((float) 65.5)
#define DRV_MPU6050_GYRO_SENS_1000    ((float) 32.8)
#define DRV_MPU6050_GYRO_SENS_2000    ((float) 16.4)

/* Acce sensitivities in /g */
#define DRV_MPU6050_ACCE_SENS_2       ((float) 16384)
#define DRV_MPU6050_ACCE_SENS_4       ((float) 8192)
#define DRV_MPU6050_ACCE_SENS_8       ((float) 4096)
#define DRV_MPU6050_ACCE_SENS_16      ((float) 2048)

/* Public enumerate/structure ----------------------------------------- */

/**
 * @brief MPU6050 error codes
 */
typedef enum
{
  DRV_MPU6050_OK,
  DRV_MPU6050_ERROR,
  DRV_MPU6050_ERROR_PARAMETER,
  DRV_MPU6050_DEVICE_INVALID
} drv_mpu6050_err_t;

/**
 * @defgroup MPU6050 Typedefs
 * @brief    Library Typedefs
 * @{
 */

/**
 * @brief  MPU6050 can have 2 different slave addresses, depends on it's input AD0 pin
 *         This feature allows you to use 2 different sensors with this library at the same time
 */
typedef enum
{
  DRV_MPU6050_DEVICE_0 = 0,   /*!< AD0 pin is set to low */
  DRV_MPU6050_DEVICE_1 = 0x02 /*!< AD0 pin is set to high */
} drv_mpu6050_device_t;

/**
 * @brief  Parameters for accelerometer range
 */
typedef enum
{
  DRV_MPU6050_ACCELEROMETER_2G  = 0x00, /*!< Range is +- 2G */
  DRV_MPU6050_ACCELEROMETER_4G  = 0x01, /*!< Range is +- 4G */
  DRV_MPU6050_ACCELEROMETER_8G  = 0x02, /*!< Range is +- 8G */
  DRV_MPU6050_ACCELEROMETER_16G = 0x03  /*!< Range is +- 16G */
} drv_mpu6050_accelerometer_t;

/**
 * @brief  Parameters for gyroscope range
 */
typedef enum
{
  DRV_MPU6050_GYROSCOPE_250s  = 0x00, /*!< Range is +- 250 degrees/s */
  DRV_MPU6050_GYROSCOPE_500s  = 0x01, /*!< Range is +- 500 degrees/s */
  DRV_MPU6050_GYROSCOPE_1000s = 0x02, /*!< Range is +- 1000 degrees/s */
  DRV_MPU6050_GYROSCOPE_2000s = 0x03  /*!< Range is +- 2000 degrees/s */
} drv_mpu6050_gyroscope_t;

/**
 * @brief  Main MPU6050 structure
 */
typedef struct
{
  /* I2C device address */
  uint8_t device_address;
  /* Function pointers --------------------------------------------------------- */
  bool (*i2c_is_device_ready)(uint8_t device_address);
  bool (*i2c_read_at)(uint8_t device_address, uint8_t reg_read, uint8_t *data_read, uint16_t size_data);
  bool (*i2c_write_at)(uint8_t device_address, uint8_t reg_write, uint8_t *data_write, uint16_t size_data);
  /* Config MPU6050 */
  drv_mpu6050_accelerometer_t accelerometer_config;
  drv_mpu6050_gyroscope_t     gyroscope_config;

  /* Private */
  uint8_t address;   /*!< I2C address of device. Only for private use */
  float   gyro_mult; /*!< Gyroscope corrector from raw data to "degrees/s". Only for private use */
  float   acce_mult; /*!< Accelerometer corrector from raw data to "g". Only for private use */
  /* Public */
  int16_t accelerometer_X; /*!< Accelerometer value X axis */
  int16_t accelerometer_Y; /*!< Accelerometer value Y axis */
  int16_t accelerometer_Z; /*!< Accelerometer value Z axis */
  int16_t gyroscope_X;     /*!< Gyroscope value X axis */
  int16_t gyroscope_Y;     /*!< Gyroscope value Y axis */
  int16_t gyroscope_Z;     /*!< Gyroscope value Z axis */
  float   temperature;     /*!< Temperature in degrees */
  float   angle_X;
  float   angle_Y;
  float   angle_Z;
} drv_mpu6050_config_t;

/* Public macros ------------------------------------------------------ */

/* Public variables --------------------------------------------------- */

/* Public function prototypes ----------------------------------------- */

/**
 * @defgroup MPU6050 functions
 * @brief    Library Functions
 * @{
 */

/**
 * @brief  Initializes MPU6050 and I2C peripheral
 * @param  *mpu6050_data: data config mpu6050 @ref drv_mpu6050_config_t structure
 * @param   device_number: MPU6050 has one pin, AD0 which can be used to set address of device.
 *          This feature allows you to use 2 different sensors on the same board with same library.
 *          If you set AD0 pin to low, then this parameter should be drv_mpu6050_device_0,
 *          but if AD0 pin is high, then you should use drv_mpu6050_device_1
 *
 *          Parameter can be a value of @ref drv_mpu6050_device_t enumeration
 * @retval Status:
 *            - drv_mpu6050_err_t: Everything OK
 *            - Other member: in other cases
 */
drv_mpu6050_err_t drv_mpu6050_init(drv_mpu6050_config_t *mpu6050_data, uint8_t device_number);

/**
 * @brief  Reads accelerometer data from sensor
 * @param  *mpu6050_data: Pointer to @ref drv_mpu6050_config_t structure to store data to
 * @retval Member of @ref drv_mpu6050_err_t:
 *            - drv_mpu6050_result_Ok: everything is OK
 *            - Other: in other cases
 */
drv_mpu6050_err_t drv_mpu6050_read_accelerometer(drv_mpu6050_config_t *mpu6050_data);

/**
 * @brief  Reads gyroscope data from sensor
 * @param  *mpu6050_data: Pointer to @ref drv_mpu6050_config_t structure to store data to
 * @retval Member of @ref drv_mpu6050_err_t:
 *            - drv_mpu6050_result_Ok: everything is OK
 *            - Other: in other cases
 */
drv_mpu6050_err_t drv_mpu6050_read_gyroscope(drv_mpu6050_config_t *mpu6050_data);

/**
 * @brief  Reads temperature data from sensor
 * @param  *mpu6050_data: Pointer to @ref drv_mpu6050_config_t structure to store data to
 * @retval Member of @ref drv_mpu6050_err_t:
 *            - drv_mpu6050_result_Ok: everything is OK
 *            - Other: in other cases
 */
drv_mpu6050_err_t drv_mpu6050_read_temperature(drv_mpu6050_config_t *mpu6050_data);

/**
 * @brief  Reads accelerometer, gyroscope and temperature data from sensor
 * @param  *mpu6050_data: Pointer to @ref drv_mpu6050_config_t structure to store data to
 * @retval Member of @ref drv_mpu6050_err_t:
 *            - drv_mpu6050_result_Ok: everything is OK
 *            - Other: in other cases
 */
drv_mpu6050_err_t drv_mpu6050_read_all(drv_mpu6050_config_t *mpu6050_data);

/**
 * @brief  Reads accelerometer, gyroscope and temperature data from sensor convert angle
 * @param  *mpu6050_data: Pointer to @ref drv_mpu6050_config_t structure to store data to
 * @retval Member of @ref drv_mpu6050_err_t:
 *            - drv_mpu6050_result_Ok: everything is OK
 *            - Other: in other cases
 */
drv_mpu6050_err_t drv_mpu6050_read_angles(drv_mpu6050_config_t *mpu6050_data);

/**
 * @}
 */

/**
 * @}
 */

/**
 * @}
 */

#endif  // __DRV_MPU6050_H

/* End of file -------------------------------------------------------- */
