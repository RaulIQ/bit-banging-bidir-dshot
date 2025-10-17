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

// flags for reception or transmission:
static bool bdshot_reception_1 = true;

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
    DMA2_Stream1->CR |= DMA_SxCR_TCIE;
    DMA2_Stream1->CR |= DMA_SxCR_EN;   // enable stream
}

void update_dma(void) {
    GPIOC->MODER |= GPIO_MODER_MODER9_0;

    TIM1->CR1 &= ~TIM_CR1_CEN;
    TIM1->CCR1 = DSHOT_BB_FRAME_LENGTH / DSHOT_BB_FRAME_SECTIONS;
	TIM1->ARR = DSHOT_BB_FRAME_LENGTH / DSHOT_BB_FRAME_SECTIONS - 1;


    DMA2_Stream1->CR &= ~DMA_SxCR_EN;
    // while (DMA2_Stream1->CR & DMA_SxCR_EN);
    DMA2_Stream1->CR |= DMA_SxCR_DIR_0;
    DMA2_Stream1->PAR  = (uint32_t)&GPIOC->BSRR;
    DMA2_Stream1->M0AR = (uint32_t)dshot_bb_buffer_1;
    DMA2_Stream1->NDTR = DSHOT_BB_BUFFER_LENGTH * DSHOT_BB_FRAME_SECTIONS;
    DMA2_Stream1->CR |= DMA_SxCR_EN;

    TIM1->EGR |= TIM_EGR_UG;
    TIM1->CR1 |= TIM_CR1_CEN;

    bdshot_reception_1 = true;
}

void DMA2_Stream1_IRQHandler(void)
{
    if (DMA2->LISR & DMA_LISR_TCIF1)
    {
        DMA2->LIFCR |= DMA_LIFCR_CTCIF1;

        if (bdshot_reception_1)
        {
            // set GPIOs as inputs:
            GPIOC->MODER &= ~GPIO_MODER_MODER9;
            // set pull up for those pins:
            GPIOC->PUPDR |= GPIO_PUPDR_PUPDR9_0;

            // set timer:
            TIM1->ARR = DSHOT_BB_FRAME_LENGTH * DSHOT_MODE / BDSHOT_RESPONSE_BITRATE / BDSHOT_RESPONSE_OVERSAMPLING - 1;
            TIM1->CCR1 = DSHOT_BB_FRAME_LENGTH * DSHOT_MODE / BDSHOT_RESPONSE_BITRATE / BDSHOT_RESPONSE_OVERSAMPLING;

            DMA2_Stream1->CR &= ~(DMA_SxCR_DIR);
            DMA2_Stream1->PAR = (uint32_t)(&(GPIOC->IDR));
            DMA2_Stream1->M0AR = (uint32_t)(dshot_bb_buffer_1_r);
            // Main idea:
            // After sending DShot frame to ESC start receiving GPIO values.
            // Capture data (probing longer than ESC response).
            // There is ~33 [us] gap before the response so it is necessary to add more samples:
            DMA2_Stream1->NDTR = ((int)(33 * BDSHOT_RESPONSE_BITRATE / 1000 + BDSHOT_RESPONSE_LENGTH + 1) * BDSHOT_RESPONSE_OVERSAMPLING);

            DMA2_Stream1->CR |= DMA_SxCR_EN;
            bdshot_reception_1 = false;
        }
    }
    
    uint32_t isr = DMA2->LISR;
    if (isr & (DMA_LISR_TCIF1 | DMA_LISR_HTIF1 | DMA_LISR_TEIF1 | DMA_LISR_DMEIF1))
    {
        DMA2->LIFCR = DMA_LIFCR_CTCIF1
                    | DMA_LIFCR_CHTIF1
                    | DMA_LIFCR_CTEIF1
                    | DMA_LIFCR_CDMEIF1;
    }
}

int main(void)
{
    HAL_Init();
    setup();
    setup_NVIC();
    MX_GPIO_Init();
    MX_ETH_Init();
    MX_USART3_UART_Init();
    MX_USB_OTG_FS_PCD_Init();

    preset_bb_Dshot_buffer_single();

    dma_init();
    timer1_init();
    uint16_t motor_val = 1953;

    uint32_t tickstart = HAL_GetTick();
    uint32_t wait = 5000;
    if (wait < HAL_MAX_DELAY)
    {
        wait += (uint32_t)(uwTickFreq);
    }
    while ((HAL_GetTick() - tickstart) < wait)
    {
        fill_bb_BDshot_buffer(motor_val);
        update_dma();
        HAL_Delay(3);
        print_motors_rpm();
    }

    motor_val = 2999;

    while (1)
    {
        fill_bb_BDshot_buffer(motor_val);
        update_dma();
        HAL_Delay(3);
        print_motors_rpm();
    }
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

