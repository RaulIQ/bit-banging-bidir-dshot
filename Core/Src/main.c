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

int main(void)
{
    HAL_Init();
    setup();
    MX_GPIO_Init();
    MX_ETH_Init();
    MX_USART3_UART_Init();
    MX_USB_OTG_FS_PCD_Init();

    preset_bb_Dshot_buffer_single();

    uint16_t motor_val = 0;

    uint32_t tickstart = HAL_GetTick();
    uint32_t wait = 5000;
    if (wait < HAL_MAX_DELAY)
    {
        wait += (uint32_t)(uwTickFreq);
    }
    while ((HAL_GetTick() - tickstart) < wait)
    {
        fill_bb_BDshot_buffer(motor_val, true, false);
        update_dma();
        HAL_Delay(3);
    }

    motor_val = 500;

    uint8_t i = 0;
    int32_t rpm_sum = 0;

    while (1)
    {
        fill_bb_BDshot_buffer(motor_val, true, true);
        update_dma();
        HAL_Delay(3);

        int32_t rpm = get_motors_rpm();

        if (rpm != -1) {
            rpm_sum += rpm;
            i++;
        }

        if (i >= 100) {
            int32_t mean = rpm_sum / i; 
            printf("RPM = %ld\n", (long)mean);

            i = 0;
            rpm_sum = 0;
        }
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

