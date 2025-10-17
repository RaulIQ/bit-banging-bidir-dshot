/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "eth.h"
#include "usart.h"
#include "usb_otg.h"
#include "gpio.h"
#include <stm32f7xx_hal_rcc.h>
#include <sys/unistd.h>
#include <sys/stat.h>
#include <setup.h>
#include <bdshot.h>
#include <global_constants.h>

int _write(int fd, char* ptr, int len) {
    if (fd == 1 || fd == 2) { // stdout или stderr
        HAL_UART_Transmit(&huart3, (uint8_t*)ptr, len, HAL_MAX_DELAY);
        return len;
    }
    return -1;
}

void timer1_init(void) {
  RCC->APB2ENR |= RCC_APB2ENR_TIM1EN;

  TIM1->CCR1 = DSHOT_BB_FRAME_LENGTH / DSHOT_BB_FRAME_SECTIONS;

	//	TIM1 is 216 [MHz]:
	TIM1->PSC = 216000 / DSHOT_MODE / DSHOT_BB_FRAME_LENGTH - 1;
	TIM1->ARR = DSHOT_BB_FRAME_LENGTH / DSHOT_BB_FRAME_SECTIONS - 1;

  TIM1->CCER |= TIM_CCER_CC1E;

  TIM1->CR1 |= TIM_CR1_ARPE | TIM_CR1_URS;

  TIM1->DIER |= TIM_DIER_CC1DE;

  TIM1->EGR |= TIM_EGR_UG;   // generate first update event
  TIM1->CR1  |= TIM_CR1_CEN;  // start counter
}

void dma_init(void) {
  RCC->AHB1ENR |= RCC_AHB1ENR_DMA2EN;

  // TIM1_UP -> DMA2 Stream2 Channel5 (RM0410, Table 32)
  DMA2_Stream1->CR &= ~DMA_SxCR_EN;
  while (DMA2_Stream1->CR & DMA_SxCR_EN); // wait until disabled

  // Clear all pending flags
  DMA2->LIFCR = (0x3D << 0); // clear Stream2 flags (bits [5:0])

  DMA2_Stream1->PAR  = (uint32_t)&GPIOC->BSRR;
  DMA2_Stream1->M0AR = (uint32_t)dshot_bb_buffer_1;
  DMA2_Stream1->NDTR = DSHOT_BB_BUFFER_LENGTH * DSHOT_BB_FRAME_SECTIONS;

  DMA2_Stream1->CR =
        (6U << DMA_SxCR_CHSEL_Pos)   // Channel 6 = TIM1_CH1
      | DMA_SxCR_MINC                // increment memory
      | DMA_SxCR_DIR_0               // mem->periph
      // | DMA_SxCR_CIRC     /           // circular
      | DMA_SxCR_PL_1                // high priority
      | DMA_SxCR_MSIZE_1             // mem = 32-bit
      | DMA_SxCR_PSIZE_1;            // periph = 32-bit

  DMA2_Stream1->FCR = 0x00000000u;
  DMA2_Stream1->CR |= DMA_SxCR_EN;   // enable stream
}

int main(void)
{
  HAL_Init();
  setup();
  MX_GPIO_Init();
  MX_ETH_Init();
  MX_USART3_UART_Init();
  MX_USB_OTG_FS_PCD_Init();

  preset_bb_Dshot_buffer_single();

  dma_init();
  timer1_init();
  uint16_t motor_val = 3000;
  // fill_bb_BDshot_buffer(motor_val);

  while (1)
  {
    fill_bb_BDshot_buffer(motor_val);

    DMA2->LIFCR = DMA_LIFCR_CTCIF1   // Очистить Transfer Complete
           | DMA_LIFCR_CHTIF1   // Очистить Half Transfer
           | DMA_LIFCR_CTEIF1   // Очистить Transfer Error
           | DMA_LIFCR_CDMEIF1;

    DMA2_Stream1->CR &= ~DMA_SxCR_EN;
    while (DMA2_Stream1->CR & DMA_SxCR_EN);
    DMA2_Stream1->NDTR = DSHOT_BB_BUFFER_LENGTH * DSHOT_BB_FRAME_SECTIONS;
    DMA2_Stream1->CR |= DMA_SxCR_EN;
    HAL_Delay(5);
    // for (uint8_t i = 0; i < N_SAMPLES; i++) 
    // {
    //   GPIOB->BSRR = pwm_buf[i];
    //   HAL_Delay(50);
    // }
  }
  /* USER CODE END 3 */
}



void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

