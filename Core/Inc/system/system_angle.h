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

/* Define to prevent recursive inclusion ------------------------------ */
#ifndef __SYSTEM_ANGLE_H
#define __SYSTEM_ANGLE_H

/* Includes ----------------------------------------------------------- */
#include "main.h"
#include "system_angle.h"

/* Public defines ----------------------------------------------------- */
typedef enum
{
  SYSTEM_ANGLE_OK,
  SYSTEM_ANGLE_ERROR,
} system_angle_error_t;

/* Public enumerate/structure ----------------------------------------- */

/* Public macros ------------------------------------------------------ */

/* Public variables --------------------------------------------------- */

/* Public function prototypes ----------------------------------------- */

system_angle_error_t system_angle_init(void);

/**
 * @brief init sensor angle
 *
 * @return system_angle_error_t
 *  - SYSTEM_ANGLE_OK : system angle ok
 *  - SYSTEM_ANGLE_ERROR : system angle error
 */

system_angle_error_t system_angle_get_value(float *angle_X, float *angle_Y, float *angle_Z);

#endif /* __SYSTEM_ANGLE_H */
