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
#include "cmsis_os.h"
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <math.h>
#include "usbd_cdc_if.h"
#include "ring-buff-bg.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define RING_BUFF_LEN 50
// Координаты якорей
//       X      Y      Z
// [AN1] 0      0      0
// [AN2] AN2_X  0      0
// [AN1] 0      AN3_Y  0

// Проверочные данные:
// #define AN2_X 100
// #define AN3_Y 100
// double dist[] = {100, 90, 80};
// Результат:
// [x: 59]  [y: 68]  [z: 43]
#define AN2_X 150
#define AN3_Y 150
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
UART_HandleTypeDef huart1;

/* Definitions for trilat_task */
osThreadId_t trilat_taskHandle;
const osThreadAttr_t trilat_task_attributes = {
  .name = "trilat_task",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for dist_task */
osThreadId_t dist_taskHandle;
const osThreadAttr_t dist_task_attributes = {
  .name = "dist_task",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* USER CODE BEGIN PV */
uint8_t ring_buff_array [RING_BUFF_LEN];
ring_buff_t uart_rx_buff;
int32_t dist[] = {0, 0, 0}; // Расстояние до якорей, максимальный ID: 3

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
void start_trilat_task(void *argument);
void start_dist_task(void *argument);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of trilat_task */
  trilat_taskHandle = osThreadNew(start_trilat_task, NULL, &trilat_task_attributes);

  /* creation of dist_task */
  dist_taskHandle = osThreadNew(start_dist_task, NULL, &dist_task_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

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

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USB;
  PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_PLL_DIV1_5;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
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
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart == &huart1)
  {
    ring_buff_move_head(&uart_rx_buff, 1);
    HAL_UART_Receive_IT(&huart1, &uart_rx_buff.buff[uart_rx_buff.head], 1);
  }
}
/* USER CODE END 4 */

/* USER CODE BEGIN Header_start_trilat_task */
/**
  * @brief  Function implementing the trilat_task thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_start_trilat_task */
void start_trilat_task(void *argument)
{
  /* init code for USB_DEVICE */
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN 5 */  
  //vTaskDelay(pdMS_TO_TICKS(15000));
  TickType_t loop_period = pdMS_TO_TICKS(1000);
  TickType_t loop_last_time = 0;
  /* Infinite loop */
  for(;;)
  {
    int32_t save_dist[] = {dist[0], dist[1], dist[2]};
    uint8_t usb_tx_buff[RING_BUFF_LEN];
    size_t usb_tx_size = snprintf((char *)usb_tx_buff, RING_BUFF_LEN, "an: %d\n", AN2_X);
    CDC_Transmit_FS(usb_tx_buff, usb_tx_size);
    vTaskDelay(pdMS_TO_TICKS(50));
    // Вывод расстояний
    usb_tx_size = snprintf((char *)usb_tx_buff, RING_BUFF_LEN, "a1:%4d\na2:%4d\na3:%4d\n",
                           save_dist[0], save_dist[1], save_dist[2]);
    CDC_Transmit_FS(usb_tx_buff, usb_tx_size);
    vTaskDelay(pdMS_TO_TICKS(50));
    // Вывод координат
    int32_t coord_x = (int32_t)((double)(save_dist[0] * save_dist[0] - save_dist[1] * save_dist[1] + AN2_X * AN2_X) / (AN2_X * 2));
    int32_t coord_y = (int32_t)((double)(save_dist[0] * save_dist[0] - save_dist[2] * save_dist[2] + AN3_Y * AN3_Y) / (AN3_Y * 2));
    int32_t coord_z = save_dist[0] * save_dist[0] - coord_x * coord_x - coord_y * coord_y;
    if (coord_z < 0) {
      CDC_Transmit_FS((uint8_t *)&"The solution is impossible\n\n", 28);
    } else {
      coord_z = (int32_t)(sqrt((double)coord_z));
      usb_tx_size = snprintf((char *)usb_tx_buff, RING_BUFF_LEN, "[x:%4d] [y:%4d] [z:%4d]\n\n",
                             coord_x, coord_y, coord_z);
      CDC_Transmit_FS(usb_tx_buff, usb_tx_size);
    }
    vTaskDelayUntil(&loop_last_time, loop_period);
  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_start_dist_task */
/**
* @brief Function implementing the dist_task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_start_dist_task */
void start_dist_task(void *argument)
{
  /* USER CODE BEGIN start_dist_task */
  ring_buff_init(&uart_rx_buff, ring_buff_array, RING_BUFF_LEN);
  HAL_UART_Receive_IT(&huart1, uart_rx_buff.buff, 1);

  vTaskDelay(pdMS_TO_TICKS(10500));
  // HAL_UART_Transmit(&huart1, (uint8_t *)&"AT+RST\r\n", 8, 500);
  // vTaskDelay(pdMS_TO_TICKS(2000));
  HAL_UART_Transmit(&huart1, (uint8_t *)&"AT+switchdis=1\r\n", 16, 500);
  
  uint8_t ant_num = 0;
  double ant_dist = 0;

  TickType_t loop_period = pdMS_TO_TICKS(500);
  TickType_t loop_last_time = 0;
  /* Infinite loop */
  for(;;)
  {
    uint8_t one_str_buff[RING_BUFF_LEN];
    // Выделям одну строку
    for (size_t i = 0; i < RING_BUFF_LEN; i++) {
      // Дожидаемся данных
      while (!ring_buff_get(&uart_rx_buff, &one_str_buff[i], 1)) {
        vTaskDelay(pdMS_TO_TICKS(50));
      }
      if (one_str_buff[i] == '\n') {
        break;
      }
    }
    // Проверяем, что это строка с дистанцией
    if (one_str_buff[0] == 'a' && one_str_buff[1] == 'n' && one_str_buff[3] == ':') {
      ant_num = one_str_buff[2] - 48;
      if (ant_num && ant_num <= 3) { // Проверяем что ID якоря в диапазоне [1;3]
        // ПОЛУЧЕНИЕ РАССТОЯНИЯ
        ant_dist = 0;
        uint8_t get_int = 1;
        double dec_mult = 0.1;
        for (uint8_t i = 4; i < 10; i++) {
          if (one_str_buff[i] >= '0' &&  one_str_buff[i] <= '9') {
            if (get_int) {
              ant_dist = ant_dist * 10 + (one_str_buff[i] - 48);
            } else {
              ant_dist += (one_str_buff[i] - 48) * dec_mult;
              dec_mult *= 0.1;
            }
          } else if (one_str_buff[i] == '.') {
            get_int = 0;
          } else {
            break;
          }
        }
        // Не помещается
        // UPD Если собирать не Debug а Release, то помещается,
        // но все равно занимает много места
        // ant_dist = atof(&one_str_buff[4]);
        if (ant_dist) {
          dist[ant_num - 1] = ant_dist * 100;
        }
      }
    }
    vTaskDelayUntil(&loop_last_time, loop_period);
  }
  /* USER CODE END start_dist_task */
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
  if (htim->Instance == TIM1)
  {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
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
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
