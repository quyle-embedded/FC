/**
 * @file       kalman_filter.h
 * @copyright  Copyright (C) 2019 QuyLe Co., Ltd. All rights reserved.
 * @license    This project is released under the QuyLe License.
 * @version    v1.0.0
 * @date       2024-05-09
 * @author     QuyLe
 * @author     QuyLe
 *
 * @brief      kalman filter
 *
 * @note       None
 * @example    None
 */

/* Define to prevent recursive inclusion ------------------------------ */
#ifndef __KALMAN_FILTER_H
#define __KALMAN_FILTER_H

/* Includes ----------------------------------------------------------- */
#include "main.h"

/* Public defines ----------------------------------------------------- */

/* Public enumerate/structure ----------------------------------------- */

typedef struct
{
  /* data */
  /* Kalman filter variables */
  float Q_angle;    // Process noise variance for the accelerometer
  float Q_bias;     // Process noise variance for the gyro bias
  float R_measure;  // Measurement noise variance - this is actually the variance of the measurement noise

  float angle;  // The angle calculated by the Kalman filter - part of the 2x1 state vector
  float bias;   // The gyro bias calculated by the Kalman filter - part of the 2x1 state vector
  float rate;   // Unbiased rate calculated from the rate and the calculated bias - you have to call getAngle to update the rate

  float P[2][2];  // Error covariance matrix - This is a 2x2 matrix

} kalman_filter_t;

/* Public macros ------------------------------------------------------ */

/* Public variables --------------------------------------------------- */

/* Public function prototypes ----------------------------------------- */

void kalman_filter_init(kalman_filter_t *kalman);

/* The angle should be in degrees and the rate should be in degrees per second and the delta time in seconds */
float kalman_filter_get_angle(kalman_filter_t* kalman, float new_angle, float new_rate, float dt);

#endif /* __SYSTEM_ANGLE_H */
