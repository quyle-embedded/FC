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

/* Define to prevent recursive inclusion ------------------------------ */
#ifndef __BSP_SPI_H
#define __BSP_SPI_H

/* Includes ----------------------------------------------------------- */
#include "main.h"

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
/* Public defines ----------------------------------------------------- */
#define BSP_SPI1 1
#define BSP_SPI2 0
#define BSP_SPI3 0

/* Public enumerate/structure ----------------------------------------- */

/* Public macros ------------------------------------------------------ */

/* Public variables --------------------------------------------------- */

/* Public function prototypes ----------------------------------------- */

bool bsp_spi1_write(uint8_t *data_write, uint16_t size_data);

bool bsp_spi1_read(uint8_t *data_read, uint16_t size_data);

bool bsp_spi1_send(uint8_t *data_write, uint8_t *data_read, uint16_t size_data);

#if BSP_SPI1 == 1
void bsp_spi1_set_tx_cplt_callback(void *cb);
void bsp_spi1_set_rx_cplt_callback(void *cb);
#endif
#if BSP_SPI2 == 1
void bsp_spi2_set_tx_cplt_callback(void *cb);
void bsp_spi2_set_rx_cplt_callback(void *cb);
#endif
#if BSP_SPI3 == 1
void bsp_spi1_set_tx_cplt_callback(void *cb);
void bsp_spi1_set_rx_cplt_callback(void *cb);
#endif

#endif  // __BSP_SPI_H

/* End of file -------------------------------------------------------- */
