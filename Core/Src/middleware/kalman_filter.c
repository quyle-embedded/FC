/**
 * @file       kalman_filter.c
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

/* Includes ----------------------------------------------------------- */
#include "kalman_filter.h"

/* Private defines ---------------------------------------------------- */

/* Private enumerate/structure ---------------------------------------- */

/* Private macros ----------------------------------------------------- */

/* Public variables --------------------------------------------------- */

/* Private variables -------------------------------------------------- */

/* Private function prototypes ---------------------------------------- */

/* Function definitions ----------------------------------------------- */

void kalman_filter_init(kalman_filter_t *kalman)
{
  /* We will set the variables like so, these can also be tuned by the user */
  kalman->Q_angle   = 0.001f;
  kalman->Q_bias    = 0.003f;
  kalman->R_measure = 0.03f;

  kalman->angle = 0.0f;  // Reset the angle
  kalman->bias  = 0.0f;  // Reset bias

  kalman->P[0][0] = 0.0f;  // Since we assume that the bias is 0 and we know the starting angle (use setAngle), the error covariance matrix is set like so -

  kalman->P[0][1] = 0.0f;
  kalman->P[1][0] = 0.0f;
  kalman->P[1][1] = 0.0f;
}

// The angle should be in degrees and the rate should be in degrees per second and the delta time in seconds
float kalman_filter_get_angle(kalman_filter_t* kalman, float new_angle, float new_rate, float dt)
{
  // KasBot V2  -  Kalman filter module - http://www.x-firm.com/?page_id=145
  // Modified by Kristian Lauszus
  // See my blog post for more information: http://blog.tkjelectronics.dk/2012/09/a-practical-approach-to-kalman-filter-and-how-to-implement-it

  // Discrete Kalman filter time update equations - Time Update ("Predict")
  // Update xhat - Project the state ahead
  /* Step 1 */
  kalman->rate = new_rate - kalman->bias;
  kalman->angle += dt * kalman->rate;

  // Update estimation error covariance - Project the error covariance ahead
  /* Step 2 */
  kalman->P[0][0] += dt * (dt * kalman->P[1][1] - kalman->P[0][1] - kalman->P[1][0] + kalman->Q_angle);
  kalman->P[0][1] -= dt * kalman->P[1][1];
  kalman->P[1][0] -= dt * kalman->P[1][1];
  kalman->P[1][1] += kalman->Q_bias * dt;

  // Discrete Kalman filter measurement update equations - Measurement Update ("Correct")
  // Calculate Kalman gain - Compute the Kalman gain
  /* Step 4 */
  float S = kalman->P[0][0] + kalman->R_measure;  // Estimate error
  /* Step 5 */
  float K[2];  // Kalman gain - This is a 2x1 vector
  K[0] = kalman->P[0][0] / S;
  K[1] = kalman->P[1][0] / S;

  // Calculate angle and bias - Update estimate with measurement zk (new_angle)
  /* Step 3 */
  float y = new_angle - kalman->angle;  // Angle difference
  /* Step 6 */
  kalman->angle += K[0] * y;
  kalman->bias += K[1] * y;

  // Calculate estimation error covariance - Update the error covariance
  /* Step 7 */
  float P00_temp = kalman->P[0][0];
  float P01_temp = kalman->P[0][1];

  kalman->P[0][0] -= K[0] * P00_temp;
  kalman->P[0][1] -= K[0] * P01_temp;
  kalman->P[1][0] -= K[1] * P00_temp;
  kalman->P[1][1] -= K[1] * P01_temp;

  return kalman->angle;
}

/* End of file -------------------------------------------------------- */
