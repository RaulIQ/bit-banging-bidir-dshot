
#include "stm32f7xx.h"
#include "global_constants.h"
#include "global_variables.h"
#include "bdshot.h"
#include "setup.h"
#include <stm32f767xx.h>
#include "main.h"


static void setup_HSE();
static void setup_PLL();
static void setup_GPIOC();	// GPIOA (pin 2 - motor; pin 3 - motor)
static void setup_GPIOB();	// GPIOB (pin 0 - motor; pin 1 - motor)
static void timer1_init(); // Bidirectional DShot
static void setup_DMA();
static void SystemClock_Config();

void setup()
{
	// basic configuration:
	SystemClock_Config();
	// BDshot specific setup:
  // timer1_init();
  setup_GPIOC();
	// setup_DMA();
  // setup_NVIC();
}

static void SystemClock_Config()
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure LSE Drive Capability
  */
  HAL_PWR_EnableBkUpAccess();

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 216;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 9;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Activate the Over-Drive mode
  */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_7) != HAL_OK)
  {
    Error_Handler();
  }
}

static void setup_GPIOC()
{
	// enable GPIOA clock:
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOCEN;

	//	set mode (00-input; 01-output; 10-alternate):
	// will be set in bdshot routine

	// set speed (max speed):
	GPIOC->OSPEEDR |= (GPIO_OSPEEDER_OSPEEDR9);
  GPIOC->MODER |= GPIO_MODER_MODER9_0;
}

static void timer1_init(void) {
  RCC->APB2ENR |= RCC_APB2ENR_TIM1EN;

  TIM1->CCR1 = DSHOT_BB_FRAME_LENGTH / DSHOT_BB_FRAME_SECTIONS;

	//	TIM1 is 168 [MHz]:
	TIM1->PSC = 216000 / DSHOT_MODE / DSHOT_BB_FRAME_LENGTH - 1;
	TIM1->ARR = DSHOT_BB_FRAME_LENGTH / DSHOT_BB_FRAME_SECTIONS - 1;

  TIM1->CCER |= TIM_CCER_CC1E;

  TIM1->CR1 |= TIM_CR1_ARPE | TIM_CR1_URS;

  TIM1->DIER |= TIM_DIER_CC1DE;

  TIM1->EGR |= TIM_EGR_UG;   // generate first update event
  TIM1->CR1  |= TIM_CR1_CEN;  // start counter
}


static void setup_DMA()
{
	RCC->AHB1ENR |= RCC_AHB1ENR_DMA1EN;

	// TIM1_UP -> DMA2 Stream2 Channel1 (RM0410, Table 32)
  DMA2_Stream1->CR &= ~DMA_SxCR_EN;
  while (DMA2_Stream1->CR & DMA_SxCR_EN); // wait until disabled

  // Clear all pending flags
  DMA2->LIFCR = (0x3D << 0); // clear Stream2 flags (bits [5:0])

	DMA2_Stream1->CR =
        (6U << DMA_SxCR_CHSEL_Pos)   // Channel 5 = TIM1_UP
      | DMA_SxCR_MINC                // increment memory
      // | DMA_SxCR_DIR_0               // mem->periph
      | DMA_SxCR_PL_1                // high priority
      | DMA_SxCR_MSIZE_1             // mem = 32-bit
      | DMA_SxCR_PSIZE_1;            // periph = 32-bit
}

void setup_NVIC()
{
	//	nvic DMA interrupts enable:
	NVIC_EnableIRQ(DMA2_Stream1_IRQn);
	NVIC_SetPriority(DMA2_Stream1_IRQn, 3);
}
