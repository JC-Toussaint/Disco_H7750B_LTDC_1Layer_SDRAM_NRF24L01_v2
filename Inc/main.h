/**
  ******************************************************************************
  * @file    LTDC/LTDC_Display_1Layer/Inc/main.h
  * @author  MCD Application Team
  * @brief   Header for main.c module
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2019 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

extern void Error_Handler(void);

/* Includes ------------------------------------------------------------------*/
#include "stm32h7xx_hal.h"
#include "rk043fn48h.h"
#include "stm32h750b_discovery.h"
#include "stm32h750b_discovery_lcd.h"
#include "stm32_lcd.h"

#include <stdio.h>
#include <stdlib.h>

/* Private defines -----------------------------------------------------------*/
#define ARD_D11_MOSI_Pin GPIO_PIN_15
#define ARD_D11_MOSI_GPIO_Port GPIOB
#define ARD_D12_MISO_Pin GPIO_PIN_2
#define ARD_D12_MISO_GPIO_Port GPIOI
#define ARD_D13_SCK_Pin GPIO_PIN_3
#define ARD_D13_SCK_GPIO_Port GPIOD

#define NRF_CSN_Pin GPIO_PIN_4
#define NRF_CSN_GPIO_Port GPIOB
#define NRF_IRQ_Pin GPIO_PIN_8
#define NRF_IRQ_GPIO_Port GPIOF
#define NRF_CE_Pin GPIO_PIN_0
#define NRF_CE_GPIO_Port GPIOC

/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
#define SDRAM_BANK_ADDR                 ((uint32_t)0xD0000000)

/* #define SDRAM_MEMORY_WIDTH            FMC_SDRAM_MEM_BUS_WIDTH_8  */
/* #define SDRAM_MEMORY_WIDTH            FMC_SDRAM_MEM_BUS_WIDTH_16 */
#define SDRAM_MEMORY_WIDTH            FMC_SDRAM_MEM_BUS_WIDTH_16

#define SDCLOCK_PERIOD                   FMC_SDRAM_CLOCK_PERIOD_2
/* #define SDCLOCK_PERIOD                FMC_SDRAM_CLOCK_PERIOD_3 */

#define SDRAM_TIMEOUT                    ((uint32_t)0xFFFF)
#define REFRESH_COUNT                    ((uint32_t)0x0603)   /* SDRAM refresh counter */

#define SDRAM_MODEREG_BURST_LENGTH_1             ((uint16_t)0x0000)
#define SDRAM_MODEREG_BURST_LENGTH_2             ((uint16_t)0x0001)
#define SDRAM_MODEREG_BURST_LENGTH_4             ((uint16_t)0x0002)
#define SDRAM_MODEREG_BURST_LENGTH_8             ((uint16_t)0x0004)
#define SDRAM_MODEREG_BURST_TYPE_SEQUENTIAL      ((uint16_t)0x0000)
#define SDRAM_MODEREG_BURST_TYPE_INTERLEAVED     ((uint16_t)0x0008)
#define SDRAM_MODEREG_CAS_LATENCY_2              ((uint16_t)0x0020)
#define SDRAM_MODEREG_CAS_LATENCY_3              ((uint16_t)0x0030)
#define SDRAM_MODEREG_OPERATING_MODE_STANDARD    ((uint16_t)0x0000)
#define SDRAM_MODEREG_WRITEBURST_MODE_PROGRAMMED ((uint16_t)0x0000)
#define SDRAM_MODEREG_WRITEBURST_MODE_SINGLE     ((uint16_t)0x0200)

/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */

#endif /* __MAIN_H */

