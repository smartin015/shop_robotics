/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"
#include "hw.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

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

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

#define DIR_PORT GPIOD
const uint16_t DIR_PINS[] = {GPIO_PIN_0, GPIO_PIN_1, GPIO_PIN_3, GPIO_PIN_4, GPIO_PIN_5, GPIO_PIN_6};
#define LIMIT_PORT GPIOE
const uint16_t LIMIT_PINS[] = {GPIO_PIN_7, GPIO_PIN_8, GPIO_PIN_9, GPIO_PIN_10, GPIO_PIN_11, GPIO_PIN_12};

// See https://microcontrollerslab.com/led-blinking-tutorial-stm32f4-discovery-board-gpio-hal-library/
#define LED_PORT GPIOD
// On error, light LD5 (PD14) = red
#define ERR_PIN GPIO_PIN_14
// On successful init, light LD4 (PD12) = greem
// WARNING: PD12 conflicts with TIM4 CH1
//#define SUC_PIN GPIO_PIN_12
// On data transfer, light LD6 (PD15) - blue
#define XFER_PIN GPIO_PIN_15

#define BUFLEN 128
#define UART5_FRAGMENT_LEN 1
uint8_t uart3_recv_buf[2][BUFLEN];
uint8_t buf_write_idx = 0;

void hw_init() {
  HAL_GPIO_WritePin(LED_PORT, ERR_PIN, GPIO_PIN_SET);

  HAL_GPIO_WritePin(LED_PORT, ERR_PIN, GPIO_PIN_RESET);
  
  // UART5 is command uart. Don't enable logging input for now.
  //HAL_UART_Receive_IT (&huart3, uart3_recv_buf[buf_write_idx], UART5_FRAGMENT_LEN);
  uint8_t buf[] = "hello ";
  //HAL_GPIO_WritePin(LED_PORT, ERR_PIN, GPIO_PIN_RESET);
  // HAL_UART_Receive(&huart3, buf, 1, 1000);  // receive 4 bytes of data
  while(1) {  
    HAL_GPIO_TogglePin(LED_PORT, XFER_PIN);
    HAL_UART_Transmit(&huart3, buf, sizeof(buf), 10);
  }

  // Start our microsecond counter
  HAL_TIM_Base_Start(&htim2); 
}

void hw_set_dir_and_pwm(uint8_t j, int16_t hz) {
  TIM_HandleTypeDef tmr;
  switch(j) {
    case 0:
      tmr = htim3;
      break;
    case 1:
      tmr = htim4;
      break;
    case 2:
      tmr = htim10; // PB8 shows 1.4kHz
      break;
    case 3:
      tmr = htim11; // PB9 shows 1.6kHz
      break;
    case 4:
      tmr = htim13;
      break;
    case 5:
      tmr = htim14;
      break;
    default:
      return;
  } 
  HAL_TIM_PWM_Stop(&tmr, TIM_CHANNEL_1);
  HAL_GPIO_WritePin(DIR_PORT, DIR_PINS[j], (hz > 0) ? GPIO_PIN_SET : GPIO_PIN_RESET);
  if (hz < 0) {
    hz = -hz;
  }
  if (hz != 0) {
    //tmr.Instance->PSC = TIM_PSC; // handled on init
    // (PSC+1)*(ARR+1) = TIMclk/Updatefrequency
    uint16_t arr = (TIM_CLK/hz)/(TIM_PSC+1) - 1; 
    __HAL_TIM_SET_AUTORELOAD(&tmr, arr);
    __HAL_TIM_SET_COMPARE(&tmr, TIM_CHANNEL_1, arr/2); // 50% duty cycle
    HAL_TIM_PWM_Start(&tmr, TIM_CHANNEL_1);
  }
}

uint8_t uart3_out[BUFLEN];
uint8_t uart4_out[BUFLEN];
bool uart3_sending = false;
bool uart4_sending = false;
uint8_t* hw_uart_get_write_buffer(bool logging) {
  return (logging) ? uart4_out : uart3_out;
}
bool hw_uart_send(uint8_t len, bool logging) {
  uint8_t * out = (logging) ? uart4_out : uart3_out;
  bool * sending = (logging) ? &uart4_sending : &uart3_sending;
  UART_HandleTypeDef * huart = (logging) ? &huart4 : &huart3;
  if (BUFLEN < len || *sending) {
    return false;
  }
  *sending = true;
  HAL_UART_Transmit_IT(huart, out, len);
  return true;
}

bool hw_uart_busy(bool logging) {
  return (logging) ? uart4_sending : uart3_sending;
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart) {
  if (huart == &huart3) { 
    uart3_sending = false;
  } else if (huart == &huart4) {
    uart4_sending = false;
  }
}


#define PACKET_START_BYTE 0x02
uint8_t buf_read_idx = 0;
uint8_t uart3_fragment_buf[UART5_FRAGMENT_LEN];
uint8_t hw_idx[2] = {0,0};
uint8_t hw_readlen[2] = {0,0};
bool hw_uart_data_ready() {
  return buf_write_idx != buf_read_idx;
}
void hw_uart_flip_buffer() {
  buf_read_idx = (buf_read_idx + 1) % 2;
}
uint8_t* hw_uart_recv(uint8_t* sz) {
  if (!hw_uart_data_ready()) {
    return NULL;
  }
  *sz = hw_readlen[buf_read_idx];
  return uart3_recv_buf[buf_read_idx];
}

/* UART1 Interrupt Service Routine */
void USART3_IRQHandler(void)
{
  HAL_UART_IRQHandler(&huart3);
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
  HAL_GPIO_TogglePin(LED_PORT, XFER_PIN);
  uint8_t c;

  // echo back
  HAL_UART_Transmit(&huart3, uart3_fragment_buf, 1, 100);

  uint8_t* buf = uart3_recv_buf[buf_write_idx];
  uint8_t* i = &(hw_idx[buf_write_idx]);
  uint8_t* r = &(hw_readlen[buf_write_idx]);
  for (int f = 0; f < UART5_FRAGMENT_LEN; f++) {
    c = uart3_fragment_buf[f];
    if (c == PACKET_START_BYTE) {
      // Magic byte, next byte is length
      *i=0;
      *r=0;
      continue;
    }
    if (*i == -1) {
      // Skip other bytes until we've processed a start byte
      continue;
    }
    if (*r == 0) {
      // NOTE: Max length is 255 characters
      *r = c;
      continue;
    }
    buf[(*i)++] = c;
    if (*i == *r) {
      // Essentially, drop the packet if we're about to write into
      // an occupied buffer
      if (buf_write_idx == buf_read_idx) { 
        buf_write_idx = (buf_write_idx+1) % 2;
      }
      *i = -1;
    } else if (*i >= BUFLEN) {
      *i = -1;
      // TODO overrun warning indicator
    }
  }
  HAL_UART_Receive_IT(&huart3, uart3_recv_buf[buf_write_idx], UART5_FRAGMENT_LEN); 
}

bool hw_limit_read(uint8_t j) {
  return HAL_GPIO_ReadPin(LIMIT_PORT, LIMIT_PINS[j]) != GPIO_PIN_RESET;
}

// Note that the micros value is NOT uint64_t - overflows every ~71 minutes
uint32_t hw_micros() {
  return TIM2->CNT;
}

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void setup();
void loop();

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
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_TIM10_Init();
  MX_TIM11_Init();
  MX_TIM13_Init();
  MX_TIM14_Init();
  MX_UART4_Init();
  MX_TIM2_Init();
  MX_USART3_UART_Init();
  /* USER CODE BEGIN 2 */
  setup();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    loop();

    /*
    uint8_t Test[] = "Hello "; //Data to send
    HAL_UART_Receive (&huart3, Test,sizeof(Test), 1000);  // receive 4 bytes of data
    if(HAL_UART_Transmit(&huart3,Test,sizeof(Test),100) != HAL_OK) {
    Error_Handler();
    }

    uint8_t Test2[] = "Bye  "; //Data to send
    HAL_UART_Receive (&huart4, Test2,sizeof(Test2), 1000);  // receive 4 bytes of data
    if(HAL_UART_Transmit(&huart4,Test2,sizeof(Test2),100) != HAL_OK) {
    Error_Handler();
    }
    */

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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  __disable_irq();
  HAL_GPIO_WritePin(LED_PORT, ERR_PIN, GPIO_PIN_SET);
  while (1) {}
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
