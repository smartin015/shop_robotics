/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include <stdbool.h>
#include <stdint.h>

#define TIM_CLK 84000000
#define TIM_PSC 64
#define MIN_FREQ 20
#define MAX_FREQ 2400
// (PSC+1)*(ARR+1) = TIMclk/Updatefrequency

void hw_init();
void hw_set_dir_and_pwm(uint8_t j, int16_t hz);
bool hw_limit_read(uint8_t j);

bool hw_uart_busy();
uint8_t* hw_uart_get_write_buffer();
bool hw_uart_send(uint8_t len);

bool hw_uart_data_ready();
uint8_t* hw_uart_recv(uint8_t* sz);
void hw_uart_flip_buffer();
uint32_t hw_micros();


/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define ENC_J2A_Pin GPIO_PIN_2
#define ENC_J2A_GPIO_Port GPIOE
#define ENC_J2A_EXTI_IRQn EXTI2_IRQn
#define ENC_J2B_Pin GPIO_PIN_3
#define ENC_J2B_GPIO_Port GPIOE
#define ENC_J2B_EXTI_IRQn EXTI3_IRQn
#define ENC_J3A_Pin GPIO_PIN_4
#define ENC_J3A_GPIO_Port GPIOE
#define ENC_J3A_EXTI_IRQn EXTI4_IRQn
#define ENC_J3B_Pin GPIO_PIN_5
#define ENC_J3B_GPIO_Port GPIOE
#define ENC_J3B_EXTI_IRQn EXTI9_5_IRQn
#define ENC_J4A_Pin GPIO_PIN_6
#define ENC_J4A_GPIO_Port GPIOE
#define ENC_J4A_EXTI_IRQn EXTI9_5_IRQn
#define PWM_J5_Pin GPIO_PIN_6
#define PWM_J5_GPIO_Port GPIOA
#define PWM_J6_Pin GPIO_PIN_7
#define PWM_J6_GPIO_Port GPIOA
#define LIM_J1_Pin GPIO_PIN_0
#define LIM_J1_GPIO_Port GPIOB
#define LIM_J2_Pin GPIO_PIN_1
#define LIM_J2_GPIO_Port GPIOB
#define LIM_J3_Pin GPIO_PIN_2
#define LIM_J3_GPIO_Port GPIOB
#define ENC_J4B_Pin GPIO_PIN_7
#define ENC_J4B_GPIO_Port GPIOE
#define ENC_J4B_EXTI_IRQn EXTI9_5_IRQn
#define ENC_J5A_Pin GPIO_PIN_8
#define ENC_J5A_GPIO_Port GPIOE
#define ENC_J5A_EXTI_IRQn EXTI9_5_IRQn
#define ENC_J5B_Pin GPIO_PIN_9
#define ENC_J5B_GPIO_Port GPIOE
#define ENC_J5B_EXTI_IRQn EXTI9_5_IRQn
#define ENC_J6A_Pin GPIO_PIN_10
#define ENC_J6A_GPIO_Port GPIOE
#define ENC_J6A_EXTI_IRQn EXTI15_10_IRQn
#define ENC_J6B_Pin GPIO_PIN_11
#define ENC_J6B_GPIO_Port GPIOE
#define ENC_J6B_EXTI_IRQn EXTI15_10_IRQn
#define PWM_J2_Pin GPIO_PIN_12
#define PWM_J2_GPIO_Port GPIOD
#define YELLOW_LED_Pin GPIO_PIN_13
#define YELLOW_LED_GPIO_Port GPIOD
#define RED_LED_Pin GPIO_PIN_14
#define RED_LED_GPIO_Port GPIOD
#define BLUE_LED_Pin GPIO_PIN_15
#define BLUE_LED_GPIO_Port GPIOD
#define PWM_J1_Pin GPIO_PIN_6
#define PWM_J1_GPIO_Port GPIOC
#define DIR_J1_Pin GPIO_PIN_0
#define DIR_J1_GPIO_Port GPIOD
#define DIR_J2_Pin GPIO_PIN_1
#define DIR_J2_GPIO_Port GPIOD
#define DIR_J3_Pin GPIO_PIN_3
#define DIR_J3_GPIO_Port GPIOD
#define DIR_J4_Pin GPIO_PIN_4
#define DIR_J4_GPIO_Port GPIOD
#define DIR_J5_Pin GPIO_PIN_5
#define DIR_J5_GPIO_Port GPIOD
#define DIR_J6_Pin GPIO_PIN_6
#define DIR_J6_GPIO_Port GPIOD
#define LIM_J4_Pin GPIO_PIN_3
#define LIM_J4_GPIO_Port GPIOB
#define LIM_J5_Pin GPIO_PIN_4
#define LIM_J5_GPIO_Port GPIOB
#define LIM_J6_Pin GPIO_PIN_5
#define LIM_J6_GPIO_Port GPIOB
#define PWM_J3_Pin GPIO_PIN_8
#define PWM_J3_GPIO_Port GPIOB
#define PWM_J4_Pin GPIO_PIN_9
#define PWM_J4_GPIO_Port GPIOB
#define ENC_J1A_Pin GPIO_PIN_0
#define ENC_J1A_GPIO_Port GPIOE
#define ENC_J1B_Pin GPIO_PIN_1
#define ENC_J1B_GPIO_Port GPIOE
#define ENC_J1B_EXTI_IRQn EXTI1_IRQn
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
