/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    app_threadx.c
  * @author  MCD Application Team
  * @brief   ThreadX applicative file
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2021 STMicroelectronics.
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
#include "app_threadx.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "app_azure_rtos_config.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */
TX_THREAD MainThread;
TX_SEMAPHORE Semaphore;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */
//void ThreadOne_Entry(ULONG thread_input);
void MainThread_Entry(ULONG thread_input);
void SystemClock_Restore(void);
void Enter_LowPower_Mode(void);
void Exit_LowPower_Mode(void);
static VOID App_Delay(ULONG Delay);
/* USER CODE END PFP */

/**
  * @brief  Application ThreadX Initialization.
  * @param memory_ptr: memory pointer
  * @retval int
  */
UINT App_ThreadX_Init(VOID *memory_ptr)
{
  UINT ret = TX_SUCCESS;
  TX_BYTE_POOL *byte_pool = (TX_BYTE_POOL*)memory_ptr;

  /* USER CODE BEGIN App_ThreadX_MEM_POOL */
  (void)byte_pool;
  /* USER CODE END App_ThreadX_MEM_POOL */

  /* USER CODE BEGIN App_ThreadX_Init */

  CHAR *pointer;

  /* Allocate the stack for MainThread.  */
  if (tx_byte_allocate(byte_pool, (VOID **) &pointer,
                       APP_STACK_SIZE, TX_NO_WAIT) != TX_SUCCESS)
  {
    ret = TX_POOL_ERROR;
  }

  /* Create MainThread.  */
  if (tx_thread_create(&MainThread, "Main Thread", MainThread_Entry, 0,
                       pointer, APP_STACK_SIZE,
                       MAIN_THREAD_PRIO, MAIN_THREAD_PREEMPTION_THRESHOLD,
                       TX_NO_TIME_SLICE, TX_AUTO_START) != TX_SUCCESS)
  {
    ret = TX_THREAD_ERROR;
  }

  /* Create the semaphore.  */
  if (tx_semaphore_create(&Semaphore, "Semaphore", 0) != TX_SUCCESS)
  {
    ret = TX_GROUP_ERROR;
  }
  /* USER CODE END App_ThreadX_Init */

  return ret;
}

/**
  * @brief  App_ThreadX_LowPower_Enter
  * @param  None
  * @retval None
  */
void App_ThreadX_LowPower_Enter(void)
{
  /* USER CODE BEGIN  App_ThreadX_LowPower_Enter */
  Enter_LowPower_Mode();
  /* USER CODE END  App_ThreadX_LowPower_Enter */
}

/**
  * @brief  App_ThreadX_LowPower_Exit
  * @param  None
  * @retval None
  */
void App_ThreadX_LowPower_Exit(void)
{
  /* USER CODE BEGIN  App_ThreadX_LowPower_Exit */
  Exit_LowPower_Mode();
  /* USER CODE END  App_ThreadX_LowPower_Exit */
}

/**
  * @brief  MX_ThreadX_Init
  * @param  None
  * @retval None
  */
void MX_ThreadX_Init(void)
{
  /* USER CODE BEGIN  Before_Kernel_Start */

  /* USER CODE END  Before_Kernel_Start */

  tx_kernel_enter();

  /* USER CODE BEGIN  Kernel_Start_Error */

  /* USER CODE END  Kernel_Start_Error */
}

/* USER CODE BEGIN 1 */
/**
  * @brief  Restore system clock after wake-up from STOP
  *         and select PLL as system clock source.
  * @param  None
  * @retval None
  */
void SystemClock_Restore(void)
{
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  uint32_t pFLatency = 0;

  /* Enable Power Control clock */
  __HAL_RCC_PWR_CLK_ENABLE();

  /* Get the Oscillators configuration according to the internal RCC registers */
  HAL_RCC_GetOscConfig(&RCC_OscInitStruct);

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_NONE;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /* Get the Clocks configuration according to the internal RCC registers */
  HAL_RCC_GetClockConfig(&RCC_ClkInitStruct, &pFLatency);

  /* Select PLL as system clock source and configure the HCLK, PCLK1 and PCLK2
     clocks dividers */
  RCC_ClkInitStruct.ClockType     = RCC_CLOCKTYPE_SYSCLK;
  RCC_ClkInitStruct.SYSCLKSource  = RCC_SYSCLKSOURCE_PLLCLK;
  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, pFLatency) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief EXTI line detection callbacks
  * @param GPIO_Pin: Specifies the pins connected EXTI line
  * @retval None
  */
void HAL_GPIO_EXTI_Rising_Callback(uint16_t GPIO_Pin)
{
  ULONG currentValue = 0;
  if (GPIO_Pin == BUTTON_USER_PIN)
  {
    /* Add additional checks to avoid multiple semaphore puts by successively
    clicking on the user button */
    tx_semaphore_info_get(&Semaphore, NULL, &currentValue, NULL, NULL, NULL);
    if (currentValue == 0)
    {
      /* Put the semaphore to release the MainThread */
      tx_semaphore_put(&Semaphore);
    }
  }
}

 /**
  * @brief  Function implementing the MainThread thread.
  * @param  thread_input: Not used
  * @retval None
  */
void MainThread_Entry(ULONG thread_input)
{
  (void) thread_input;
  UINT i = 0;
  /* Infinite loop */
  while (1)
  {
    if (tx_semaphore_get(&Semaphore, TX_WAIT_FOREVER) == TX_SUCCESS)
    {
      for (i=0; i<10; i++)
      {
      /* Toggle LED to indicate status*/
      BSP_LED_Toggle(LED_GREEN);
      App_Delay(50);
      }
    }
  }
}

/**
  * @brief  Enter LowPower Mode configuration
  * @param  None
  * @retval None
  */
void Enter_LowPower_Mode(void)
{
  /* Enable Power Clock */
  __HAL_RCC_PWR_CLK_ENABLE();

  /* Ensure that MSI is wake-up system clock */
  __HAL_RCC_WAKEUPSTOP_CLK_CONFIG(RCC_STOP_WAKEUPCLOCK_MSI);

  /* De-Configure LED_GREEN */
  BSP_LED_DeInit(LED_GREEN);

  /* Disable SRAM2 */
  HAL_PWREx_DisableSRAM2ContentRetention();

  /* Enter to the stop mode */
  HAL_PWREx_EnterSTOP2Mode(PWR_STOPENTRY_WFI);

}

/**
  * @brief  Exit LowPower Mode configuration
  * @param  None
  * @retval None
  */
void Exit_LowPower_Mode(void)
{
  /* Re-configure the system clock to 120 MHz based on MSI, enable and
  select PLL as system clock source (PLL is disabled in STOP mode) */
  SystemClock_Restore();

  /* Initialize LEDs */
  BSP_LED_Init(LED_GREEN);
}

/**
  * @brief  Application Delay function.
  * @param  Delay : number of ticks to wait
  * @retval None
  */
void App_Delay(ULONG Delay)
{
  ULONG initial_time = tx_time_get();
  while ((tx_time_get() - initial_time) < Delay);
}
/* USER CODE END 1 */
