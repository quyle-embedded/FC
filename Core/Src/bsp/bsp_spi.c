/**
 * @file       bsp_spi.h
 * @copyright  Copyright (C) 2019 Fiot Co., Ltd. All rights reserved.
 * @license    This project is released under the QuyLe License.
 * @version    v1.0.0
 * @date       2024-05-12
 * @author     Quy Le
 *
 * @brief      handle spi
 *
 * @note
 */

/* Includes ----------------------------------------------------------- */
#include "bsp_spi.h"

/* Private defines ---------------------------------------------------- */

#define BSP_SPI_TIMEOUT 100

#if BSP_SPI1 == 1 || BSP_SPI2 == 1 || BSP_SPI3 == 1
typedef void (*bsp_spi_tx_cplt_callback_t)(SPI_HandleTypeDef *hspi);
typedef void (*bsp_spi_rx_cplt_callback_t)(SPI_HandleTypeDef *hspi);
#endif

#if BSP_SPI1 == 1
extern SPI_HandleTypeDef hspi1;
#endif
#if BSP_SPI2 == 1
extern SPI_HandleTypeDef hspi2;
#endif
#if BSP_SPI3 == 1
extern SPI_HandleTypeDef hspi3;
#endif

/* Private enumerate/structure ---------------------------------------- */

#if BSP_SPI1 == 1
bsp_spi_tx_cplt_callback_t bsp_spi1_tx_cplt_callback = NULL;
bsp_spi_rx_cplt_callback_t bsp_spi1_rx_cplt_callback = NULL;
#endif
#if BSP_SPI2 == 1
bsp_spi_tx_cplt_callback_t bsp_spi2_tx_cplt_callback = NULL;
bsp_spi_rx_cplt_callback_t bsp_spi2_rx_cplt_callback = NULL;
#endif
#if BSP_SPI3 == 1
bsp_spi_tx_cplt_callback_t bsp_spi3_tx_cplt_callback = NULL;
bsp_spi_rx_cplt_callback_t bsp_spi3_rx_cplt_callback = NULL;
#endif

/* Private macros ----------------------------------------------------- */

/* Public variables --------------------------------------------------- */

/* Private variables -------------------------------------------------- */

/* Private function prototypes ---------------------------------------- */

/* Function definitions ----------------------------------------------- */

bool bsp_spi1_write(uint8_t *data_write, uint16_t size_data)
{
  /* Get status bus SPI */
  if (HAL_SPI_GetState(&hspi1) == HAL_SPI_STATE_BUSY)
    return false;

  /* Transmit data */
  if (HAL_SPI_Transmit(&hspi1, data_write, size_data, BSP_SPI_TIMEOUT) != HAL_OK)
    return false;
  return true;
}

bool bsp_spi1_read(uint8_t *data_read, uint16_t size_data)
{
  /* Get status bus SPI */
  if (HAL_SPI_GetState(&hspi1) == HAL_SPI_STATE_BUSY)
    return false;

  /* Receive data */
  if (HAL_SPI_Receive(&hspi1, data_read, size_data, BSP_SPI_TIMEOUT) != HAL_OK)
    return false;
  return true;
}

bool bsp_spi1_send(uint8_t *data_write, uint8_t *data_read, uint16_t size_data)
{
  /* Get status bus SPI */
  if (HAL_SPI_GetState(&hspi1) == HAL_SPI_STATE_BUSY)
    return false;

  /* Transmit Receive data */
  if (HAL_SPI_TransmitReceive(&hspi1, data_write, data_read, size_data, BSP_SPI_TIMEOUT) != HAL_OK)
    return false;
  return true;
}

#if BSP_SPI1 == 1
void bsp_spi1_set_tx_cplt_callback(void *cb)
{
  bsp_spi1_tx_cplt_callback = cb;
}
void bsp_spi1_set_rx_cplt_callback(void *cb)
{
  bsp_spi1_rx_cplt_callback = cb;
}
#endif
#if BSP_SPI2 == 1
void bsp_spi2_set_tx_cplt_callback(void *cb)
{
  bsp_spi2_tx_cplt_callback = cb;
}
void bsp_spi2_set_rx_cplt_callback(void *cb)
{
  bsp_spi2_rx_cplt_callback = cb;
}
#endif
#if BSP_SPI3 == 1
void bsp_spi1_set_tx_cplt_callback(void *cb)
{
  bsp_spi1_tx_cplt_callback = cb;
}
void bsp_spi1_set_rx_cplt_callback(void *cb)
{
  bsp_spi1_rx_cplt_callback = cb;
}
#endif

void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef *hspi)
{
#if BSP_SPI1 == 1
  if (hspi->Instance == SPI1)
  {
    if (bsp_spi1_tx_cplt_callback != NULL)
      bsp_spi1_tx_cplt_callback(hspi);
  }
#endif
#if BSP_SPI2 == 1
  if (hspi->Instance == SPI2)
  {
    if (bsp_spi2_tx_cplt_callback != NULL)
      bsp_spi2_tx_cplt_callback(hspi);
  }
#endif
#if BSP_SPI3 == 1
  if (hspi->Instance == SPI3)
  {
    if (bsp_spi3_tx_cplt_callback != NULL)
      bsp_spi3_tx_cplt_callback(hspi);
  }
#endif
}

void HAL_SPI_RxCpltCallback(SPI_HandleTypeDef *hspi)
{
#if BSP_SPI1 == 1
  if (hspi->Instance == SPI1)
  {
    if (bsp_spi1_rx_cplt_callback != NULL)
      bsp_spi1_rx_cplt_callback(hspi);
  }
#endif
#if BSP_SPI2 == 1
  if (hspi->Instance == SPI2)
  {
    if (bsp_spi2_rx_cplt_callback != NULL)
      bsp_spi2_rx_cplt_callback(hspi);
  }
#endif
#if BSP_SPI3 == 1
  if (hspi->Instance == SPI3)
  {
    if (bsp_spi3_rx_cplt_callback != NULL)
      bsp_spi3_rx_cplt_callback(hspi);
  }
#endif
}

/* End of file -------------------------------------------------------- */
