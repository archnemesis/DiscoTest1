/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
  ******************************************************************************
  *
  * Copyright (c) 2017 STMicroelectronics International N.V. 
  * All rights reserved.
  *
  * Redistribution and use in source and binary forms, with or without 
  * modification, are permitted, provided that the following conditions are met:
  *
  * 1. Redistribution of source code must retain the above copyright notice, 
  *    this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  * 3. Neither the name of STMicroelectronics nor the names of other 
  *    contributors to this software may be used to endorse or promote products 
  *    derived from this software without specific written permission.
  * 4. This software, including modifications and/or derivative works of this 
  *    software, must execute solely and exclusively on microcontroller or
  *    microprocessor devices manufactured by or for STMicroelectronics.
  * 5. Redistribution and use of this software other than as permitted under 
  *    this license is void and will automatically terminate your rights under 
  *    this license. 
  *
  * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS" 
  * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT 
  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A 
  * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
  * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT 
  * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, 
  * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF 
  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING 
  * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include <stdio.h>
#include "main.h"
#include "FreeRTOS.h"
#include "timers.h"
#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_cortex.h"
#include "stm32f4xx_hal_ltdc.h"
#include "cmsis_os.h"
#include "usb_device.h"
#include "lib/commandparser.h"
#include "lib/gfx/gfx.h"
#include "mcufont.h"
#include "Trace.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c3;

LTDC_HandleTypeDef hltdc;

SPI_HandleTypeDef hspi5;

UART_HandleTypeDef huart1;

SDRAM_HandleTypeDef hsdram1;

osThreadId defaultTaskHandle;
osThreadId screenUpdateTaskHandle;

SemaphoreHandle_t xScreenUpdateSemaphore;

uint32_t displayLayer1[320*240] __attribute__ ((section (".sram_data")));
uint32_t displayLayer2[320*240] __attribute__ ((section (".sram_data")));
volatile uint32_t *activeLayer;
volatile uint32_t buf_out[8];
volatile int counter = 0;

char commandBuffer[256];

GFX_Framebuffer gfxFramebuffer;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void Error_Handler(void);
static void MX_GPIO_Init(void);
static void MX_FMC_Init(void);
static void MX_I2C3_Init(void);
static void MX_LTDC_Init(void);
static void MX_SPI5_Init(void);
static void MX_USART1_UART_Init(void);
void StartDefaultTask(void const * argument);
void ScreenUpdateTask(void const * argument);
void ScreenUpdateAfterISR(void * pvParameter1, uint32_t pvParameter2);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN | RCC_AHB1ENR_GPIOBEN | RCC_AHB1ENR_GPIOCEN |
                    RCC_AHB1ENR_GPIODEN | RCC_AHB1ENR_GPIOEEN | RCC_AHB1ENR_GPIOFEN |
                    RCC_AHB1ENR_GPIOGEN;

  /* Configure the system clock */
  SystemClock_Config();

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_FMC_Init();
  MX_I2C3_Init();
  MX_SPI5_Init();
  MX_LTDC_Init();
  MX_USART1_UART_Init();

  /* USER CODE BEGIN 2 */

  SYSCFG->EXTICR[2] |= SYSCFG_EXTICR3_EXTI11_PD;
  EXTI->IMR |= EXTI_IMR_MR11;
  EXTI->RTSR |= EXTI_RTSR_TR11;
  //EXTI->FTSR |= EXTI_FTSR_TR11;

  /* USER CODE END 2 */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  xScreenUpdateSemaphore = xSemaphoreCreateBinary();

  /* Create the thread(s) */
  /* definition and creation of defaultTask */
  osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 128);
  osThreadDef(screenUpdateTask, ScreenUpdateTask, osPriorityNormal, 0, 128);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);
  screenUpdateTaskHandle = osThreadCreate(osThread(screenUpdateTask), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */
 

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */

  }
  /* USER CODE END 3 */

}

/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct;

    /**Configure the main internal regulator output voltage 
    */
  __HAL_RCC_PWR_CLK_ENABLE();

  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }

  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_LTDC;
  PeriphClkInitStruct.PLLSAI.PLLSAIN = 192;
  PeriphClkInitStruct.PLLSAI.PLLSAIR = 6;
  PeriphClkInitStruct.PLLSAIDivR = RCC_PLLSAIDIVR_2;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 15, 0);
}

/* I2C3 init function */
static void MX_I2C3_Init(void)
{

  hi2c3.Instance = I2C3;
  hi2c3.Init.ClockSpeed = 100000;
  hi2c3.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c3.Init.OwnAddress1 = 0;
  hi2c3.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c3.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c3.Init.OwnAddress2 = 0;
  hi2c3.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c3.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c3) != HAL_OK)
  {
    Error_Handler();
  }

}

void ILI9341_SetDC(uint8_t state) {
	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13, state);
}

void ILI9341_SetCS(uint8_t state) {
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_2, state);
}

void ILI9341_SetEnable(uint8_t state) {
	HAL_GPIO_WritePin(GPIOF, GPIO_PIN_10, state);
}

void ILI9341_WriteCommand(uint8_t c) {
	ILI9341_SetDC(0);
	ILI9341_SetCS(0);
	HAL_SPI_Transmit(&hspi5, (uint8_t*)&c, 1, HAL_MAX_DELAY);
	ILI9341_SetCS(1);
}

void ILI9341_WriteData(uint8_t c) {
	ILI9341_SetDC(1);
	ILI9341_SetCS(0);
	HAL_SPI_Transmit(&hspi5, (uint8_t*)&c, 1, HAL_MAX_DELAY);
	ILI9341_SetCS(1);
}

void drawPixel(int x, int y, int color) {
	int xpos = (240 * y) + x;
	displayLayer1[xpos] = color;
}

/* LTDC init function */
static void MX_LTDC_Init(void)
{
	unsigned int i = 0;
	unsigned int d = 0;

	memset(displayLayer1, 0, (320*240*4));
//	RCC->AHB1ENR |= (RCC_AHB1ENR_GPIOAEN | RCC_AHB1ENR_GPIOBEN | RCC_AHB1ENR_GPIOCEN |
//	     RCC_AHB1ENR_GPIODEN | RCC_AHB1ENR_GPIOFEN | RCC_AHB1ENR_GPIOGEN);

    GPIO_InitTypeDef GPIO_InitStruct;
    GPIO_InitStruct.Pin = GPIO_PIN_2;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_13;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);
//
//    GPIO_InitStruct.Pin = GPIO_PIN_10;
//    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
//    GPIO_InitStruct.Pull = GPIO_NOPULL;
//    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
//    HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

    ILI9341_SetCS(1);

	static const unsigned short ltdc_init_spi_data[] = {
	     0x0CA, 0x1C3, 0x108, 0x150,
	     0x0CF, 0x100, 0x1C1, 0x130,
	     0x0ED, 0x164, 0x103, 0x112, 0x181,
	     0x0E8, 0x185, 0x100, 0x178,
	     0x0CB, 0x139, 0x12C, 0x100, 0x134, 0x102,
	     0x0F7, 0x120,
	     0x0EA, 0x100, 0x100,
	     0x0B1, 0x100, 0x11B,
	     0x0B6, 0x10A, 0x1A2,
	     0x0C0, 0x110,
	     0x0C1, 0x110,
	     0x0C5, 0x145, 0x115,
	     0x0C7, 0x190,
	     0x036, 0x1C8,
	     0x0F2, 0x100,
	     0x0B0, 0x1C2,
	     0x0B6, 0x10A, 0x1A7, 0x127, 0x104,
	     0x02A, 0x100, 0x100, 0x100, 0x1EF,
	     0x02B, 0x100, 0x100, 0x101, 0x13F,
	     0x0F6, 0x101, 0x100, 0x106,
	     0x02C, 0x800,
	     0x026, 0x101,
		 0x0E0, 0x10F, 0x129, 0x124, 0x10C, 0x10E, 0x109, 0x14E, 0x178,
	         0x13C, 0x109, 0x113, 0x105, 0x117, 0x111, 0x100,
	     0x0E1, 0x100, 0x116, 0x11B, 0x104, 0x111, 0x107, 0x131, 0x133, 0x142, 0x105,
	         0x10C, 0x10A, 0x128, 0x12F, 0x10F,
	     0x011, 0x800,
		0x035, 0x101,
	    0x029,
	    0x02C,
	     0xFFF /* END */ };


	i = 0;
	d = ltdc_init_spi_data[0];
	while (d != 0xFFF) {
		if (d == 0x800) {
			HAL_Delay(1000);
		}
		else if (d & 0x100) {
			ILI9341_WriteData(d & 0xFF);
		}
		else {
			ILI9341_WriteCommand(d);
		}
		d = ltdc_init_spi_data[++i];
	}

  LTDC_LayerCfgTypeDef pLayerCfg;
  LTDC_LayerCfgTypeDef pLayerCfg1;

  hltdc.Instance = LTDC;
  hltdc.Init.HSPolarity = LTDC_HSPOLARITY_AL;
  hltdc.Init.VSPolarity = LTDC_VSPOLARITY_AL;
  hltdc.Init.DEPolarity = LTDC_DEPOLARITY_AL;
  hltdc.Init.PCPolarity = LTDC_PCPOLARITY_IPC;
  hltdc.Init.HorizontalSync = 7;
  hltdc.Init.VerticalSync = 3;
  hltdc.Init.AccumulatedHBP = 14;
  hltdc.Init.AccumulatedVBP = 5;
  hltdc.Init.AccumulatedActiveW = 254;
  hltdc.Init.AccumulatedActiveH = 325;
  hltdc.Init.TotalWidth = 260;
  hltdc.Init.TotalHeigh = 327;
  hltdc.Init.Backcolor.Blue = 0;
  hltdc.Init.Backcolor.Green = 0;
  hltdc.Init.Backcolor.Red = 0;
  if (HAL_LTDC_Init(&hltdc) != HAL_OK)
  {
    Error_Handler();
  }

  pLayerCfg.WindowX0 = 0;
  pLayerCfg.WindowX1 = 240;
  pLayerCfg.WindowY0 = 0;
  pLayerCfg.WindowY1 = 320;
  pLayerCfg.PixelFormat = LTDC_PIXEL_FORMAT_ARGB8888;
  pLayerCfg.Alpha = 255;
  pLayerCfg.Alpha0 = 0;
  pLayerCfg.BlendingFactor1 = LTDC_BLENDING_FACTOR1_PAxCA;
  pLayerCfg.BlendingFactor2 = LTDC_BLENDING_FACTOR2_PAxCA;
  pLayerCfg.FBStartAdress = displayLayer1;
  pLayerCfg.ImageWidth = 240;
  pLayerCfg.ImageHeight = 320;
  pLayerCfg.Backcolor.Blue = 0;
  pLayerCfg.Backcolor.Green = 0;
  pLayerCfg.Backcolor.Red = 0;
  if (HAL_LTDC_ConfigLayer(&hltdc, &pLayerCfg, 0) != HAL_OK)
  {
    Error_Handler();
  }

  HAL_LTDC_Relaod(&hltdc, LTDC_RELOAD_IMMEDIATE);

  activeLayer = displayLayer1;
	gfxFramebuffer.fbptr = displayLayer2;
	gfxFramebuffer.width = 240;
	gfxFramebuffer.height = 320;
}

/* SPI5 init function */
static void MX_SPI5_Init(void)
{

  hspi5.Instance = SPI5;
  hspi5.Init.Mode = SPI_MODE_MASTER;
  hspi5.Init.Direction = SPI_DIRECTION_2LINES;
  hspi5.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi5.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi5.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi5.Init.NSS = SPI_NSS_SOFT;
  hspi5.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
  hspi5.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi5.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi5.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi5.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi5) != HAL_OK)
  {
    Error_Handler();
  }

}

/* USART1 init function */
static void MX_USART1_UART_Init(void)
{

  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }

}

#define TMRD(x) (x << 0)  /* Load Mode Register to Active */
#define TXSR(x) (x << 4)  /* Exit Self-refresh delay */
#define TRAS(x) (x << 8)  /* Self refresh time */
#define TRC(x)  (x << 12) /* Row cycle delay */
#define TWR(x)  (x << 16) /* Recovery delay */
#define TRP(x)  (x << 20) /* Row precharge delay */
#define TRCD(x) (x << 24) /* Row to column delay */

/* FMC initialization function */
static void MX_FMC_Init(void)
{
	  volatile uint32_t tmp;

//  FMC_SDRAM_TimingTypeDef SdramTiming;
//
//  /** Perform the SDRAM1 memory initialization sequence
//  */
//	  hsdram1.Instance = FMC_SDRAM_DEVICE;
//  /* hsdram1.Init */
//  hsdram1.Init.SDBank = FMC_SDRAM_BANK2;
//  hsdram1.Init.ColumnBitsNumber = FMC_SDRAM_COLUMN_BITS_NUM_8;
//  hsdram1.Init.RowBitsNumber = FMC_SDRAM_ROW_BITS_NUM_12;
//  hsdram1.Init.MemoryDataWidth = FMC_SDRAM_MEM_BUS_WIDTH_16;
//  hsdram1.Init.InternalBankNumber = FMC_SDRAM_INTERN_BANKS_NUM_4;
//  hsdram1.Init.CASLatency = FMC_SDRAM_CAS_LATENCY_1;
//  hsdram1.Init.WriteProtection = FMC_SDRAM_WRITE_PROTECTION_DISABLE;
//  hsdram1.Init.SDClockPeriod = FMC_SDRAM_CLOCK_DISABLE;
//  hsdram1.Init.ReadBurst = FMC_SDRAM_RBURST_DISABLE;
//  hsdram1.Init.ReadPipeDelay = FMC_SDRAM_RPIPE_DELAY_0;
//  /* SdramTiming */
//  SdramTiming.LoadToActiveDelay = 16;
//  SdramTiming.ExitSelfRefreshDelay = 16;
//  SdramTiming.SelfRefreshTime = 16;
//  SdramTiming.RowCycleDelay = 16;
//  SdramTiming.WriteRecoveryTime = 16;
//  SdramTiming.RPDelay = 16;
//  SdramTiming.RCDDelay = 16;

  // Enable clock
  RCC->AHB3ENR |= RCC_AHB3ENR_FMCEN;

    GPIO_InitTypeDef GPIO_InitStruct;
    GPIO_InitStruct.Pin = A0_Pin|A1_Pin|A2_Pin|A3_Pin
                            |A4_Pin|A5_Pin|SDNRAS_Pin|A6_Pin
                            |A7_Pin|A8_Pin|A9_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF12_FMC;
    HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = SDNWE_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF12_FMC;
    HAL_GPIO_Init(SDNWE_GPIO_Port, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = A10_Pin|A11_Pin|BA0_Pin|BA1_Pin
                            |SDCLK_Pin|SDNCAS_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF12_FMC;
    HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = D4_Pin|D5_Pin|D6_Pin|D7_Pin
                            |D8_Pin|D9_Pin|D10_Pin|D11_Pin
                            |D12_Pin|NBL0_Pin|NBL1_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF12_FMC;
    HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = D13_Pin|D14_Pin|D15_Pin|D0_Pin
                            |D1_Pin|D2_Pin|D3_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF12_FMC;
    HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = SDCKE1_Pin|SDNE1_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF12_FMC;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  // Init Step 1
  FMC_SDRAM_DEVICE->SDCR[0] = FMC_SDCR1_SDCLK_1 | FMC_SDCR1_RBURST | FMC_SDCR1_RPIPE_1;
  FMC_SDRAM_DEVICE->SDCR[1] = FMC_SDCR2_NR_0 | FMC_SDCR2_MWID_0 | FMC_SDCR2_NB | FMC_SDCR2_CAS;

  // Init Step 2
  FMC_SDRAM_DEVICE->SDTR[0] = TRC(7) | TRP(2);
  FMC_SDRAM_DEVICE->SDTR[1] = TMRD(2) | TXSR(7) | TRAS(4) | TWR(2) | TRCD(2);

  // Init Step 3
  while (FMC_SDRAM_DEVICE->SDSR & FMC_SDSR_BUSY);
  FMC_SDRAM_DEVICE->SDCMR = 1 | FMC_SDCMR_CTB2 | (1 << 5);

  // Init Step 4 (wait)
  for(tmp = 0; tmp < 10000000; tmp++);

  // Init Step 5
  while (FMC_SDRAM_DEVICE->SDSR & FMC_SDSR_BUSY);
  FMC_SDRAM_DEVICE->SDCMR = 2 | FMC_SDCMR_CTB2 | (1 << 5);

  // Init Step 6
  while (FMC_SDRAM_DEVICE->SDSR & FMC_SDSR_BUSY);
  FMC_SDRAM_DEVICE->SDCMR = 3 | FMC_SDCMR_CTB2 | (4 << 5);

  // Init Step 7
  while (FMC_SDRAM_DEVICE->SDSR & FMC_SDSR_BUSY);
  FMC_SDRAM_DEVICE->SDCMR = 4 | FMC_SDCMR_CTB2 | (1 << 5) | (0x231 << 9);

  // Init Step 8
  while(FMC_SDRAM_DEVICE->SDSR & FMC_SDSR_BUSY);
  FMC_SDRAM_DEVICE->SDRTR |= (683 << 1);
  while(FMC_SDRAM_DEVICE->SDSR & FMC_SDSR_BUSY);

  // Initialize memory region to 0
  for(tmp = 0xD0000000; tmp < (0xD0000000 + 0x00800000); tmp += 4)
      *((volatile uint32_t *)tmp) = 0;
}

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
*/
static void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, NCS_MEMS_SPI_Pin|CSX_Pin|OTG_FS_PSO_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOC, CSX_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(ACP_RST_GPIO_Port, ACP_RST_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, RDX_Pin|WRX_DCX_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOG, LD3_Pin|LD4_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : NCS_MEMS_SPI_Pin CSX_Pin OTG_FS_PSO_Pin */
  GPIO_InitStruct.Pin = NCS_MEMS_SPI_Pin|CSX_Pin|OTG_FS_PSO_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : B1_Pin MEMS_INT1_Pin MEMS_INT2_Pin TP_INT1_Pin */
  GPIO_InitStruct.Pin = B1_Pin|MEMS_INT1_Pin|MEMS_INT2_Pin|TP_INT1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_EVT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : ACP_RST_Pin */
  GPIO_InitStruct.Pin = ACP_RST_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(ACP_RST_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : OTG_FS_OC_Pin */
  GPIO_InitStruct.Pin = OTG_FS_OC_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_EVT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(OTG_FS_OC_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : BOOT1_Pin */
  GPIO_InitStruct.Pin = BOOT1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(BOOT1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : TE_Pin */
  GPIO_InitStruct.Pin = TE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(TE_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : RDX_Pin WRX_DCX_Pin */
  GPIO_InitStruct.Pin = RDX_Pin|WRX_DCX_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : LD3_Pin LD4_Pin */
  GPIO_InitStruct.Pin = LD3_Pin|LD4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/* StartDefaultTask function */
void StartDefaultTask(void const * argument)
{
	/* init code for USB_DEVICE */
	MX_USB_DEVICE_Init();

	CommandParser_Init(xTaskGetCurrentTaskHandle());
	char printbuf[256];

	int received = 0;
//	HAL_GPIO_WritePin(GPIOG, LD3_Pin, 1);
//	HAL_GPIO_WritePin(GPIOG, LD4_Pin, 1);

	/* USER CODE BEGIN 5 */
	/* Infinite loop */
	for(;;)
	{
		if (ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(200))) {
			memset(commandBuffer, 0, 256);
			received = CommandParser_GetCommand((char*)commandBuffer);
			if (received > 0) {
				//trace_printf("Got a string: %s\n", commandBuffer);

				memset(printbuf, 0, 256);
				sprintf(printbuf, "%s\r\n", commandBuffer);
				CDC_Transmit_HS((uint8_t*)printbuf, received);
			}
		}
	}

	vTaskDelete(NULL);
}

void ScreenUpdateTask(void const * argument)
{
	char printbuf[32];
	uint32_t xpos;

	  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 15, 0);
	  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

	activeLayer = displayLayer1;

	GFX_Color fg = GFX_COLOR_BLACK;
	GFX_Color bg = GFX_COLOR_WHITE;
	GFX_Font  fn = GFX_FONT("Ubuntu Regular 12");

	for (;;) {

		sprintf(printbuf, "Counter: 0x%08x", counter);

		xpos = counter >> 2;
		xpos = (xpos % 265) - 25;

		GFX_Clear(&gfxFramebuffer, bg);
		GFX_DrawText(&gfxFramebuffer, GFX_POINT(10, 10), "Welcome!", fg, fn);
		GFX_DrawText(&gfxFramebuffer, GFX_POINT(10, 50), printbuf, fg, fn);
		GFX_DrawFilledRect(&gfxFramebuffer, GFX_RECT(xpos,100,25,25), fg);
		counter++;

		//
		// Drawing is done, now switch the active buffer
		// in preparation for the next draw cycle
		//
		gfxFramebuffer.fbptr = activeLayer;

		if (activeLayer == (uint32_t*)displayLayer1) {
			activeLayer = displayLayer2;
		}
		else {
			activeLayer = displayLayer1;
		}


		//
		// Give the semaphore so the refresh ISR knows
		// it's time to point to the active buffer
		//
		xSemaphoreGive(xScreenUpdateSemaphore);
	}

	vTaskDelete(NULL);
}

/*
 * If the new image is ready, we tell the LTDC to switch
 * to the new framebuffer, if not we simply keep the old one
 * where it is.
 */
void EXTI15_10_IRQHandler(void)
{
	static uint8_t tog = 1;
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;

	if (xSemaphoreTakeFromISR(xScreenUpdateSemaphore, &xHigherPriorityTaskWoken)) {
		//
		// Point the LTDC to the new active framebuffer
		//
		HAL_LTDC_SetAddress(&hltdc, (uint32_t)activeLayer, 0);
		HAL_GPIO_WritePin(GPIOG, LD3_Pin, tog);
	}
	HAL_GPIO_WritePin(GPIOG, LD4_Pin, tog);
	tog = !tog;
	EXTI->PR |= EXTI_PR_PR11;
	portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM1 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
/* USER CODE BEGIN Callback 0 */

/* USER CODE END Callback 0 */
  if (htim->Instance == TIM1) {
    HAL_IncTick();
  }
/* USER CODE BEGIN Callback 1 */

/* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler */
  /* User can add his own implementation to report the HAL error return state */
  while(1) 
  {
  }
  /* USER CODE END Error_Handler */ 
}

#ifdef USE_FULL_ASSERT

/**
   * @brief Reports the name of the source file and the source line number
   * where the assert_param error has occurred.
   * @param file: pointer to the source file name
   * @param line: assert_param error line source number
   * @retval None
   */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */

}

#endif

/**
  * @}
  */ 

/**
  * @}
*/ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
