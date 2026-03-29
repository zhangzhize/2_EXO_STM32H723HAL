/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32h7xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */
extern uint8_t g_timer2_5ms_flag;
extern uint8_t g_pcap01_intn_state;
/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define POWER_24V_2_Pin GPIO_PIN_13
#define POWER_24V_2_GPIO_Port GPIOC
#define POWER_24V_1_Pin GPIO_PIN_14
#define POWER_24V_1_GPIO_Port GPIOC
#define POWER_5V_Pin GPIO_PIN_15
#define POWER_5V_GPIO_Port GPIOC
#define IMU_ACC_CS_Pin GPIO_PIN_0
#define IMU_ACC_CS_GPIO_Port GPIOC
#define IMU_GYRO_CS_Pin GPIO_PIN_3
#define IMU_GYRO_CS_GPIO_Port GPIOC
#define RIGHT_PULLFORCE_Pin GPIO_PIN_0
#define RIGHT_PULLFORCE_GPIO_Port GPIOA
#define LEFT_PULLFORCE_Pin GPIO_PIN_2
#define LEFT_PULLFORCE_GPIO_Port GPIOA
#define PCAP01_INT_Pin GPIO_PIN_4
#define PCAP01_INT_GPIO_Port GPIOA
#define PCAP01_INT_EXTI_IRQn EXTI4_IRQn
#define PCAP01_CS_Pin GPIO_PIN_6
#define PCAP01_CS_GPIO_Port GPIOA
#define VBUS_ADC_Pin GPIO_PIN_4
#define VBUS_ADC_GPIO_Port GPIOC
#define MUX_ROW_RST_Pin GPIO_PIN_5
#define MUX_ROW_RST_GPIO_Port GPIOC
#define IMU_ACC_INT_Pin GPIO_PIN_10
#define IMU_ACC_INT_GPIO_Port GPIOE
#define IMU_ACC_INT_EXTI_IRQn EXTI15_10_IRQn
#define IMU_GYRO_INT_Pin GPIO_PIN_12
#define IMU_GYRO_INT_GPIO_Port GPIOE
#define IMU_GYRO_INT_EXTI_IRQn EXTI15_10_IRQn
#define NRF54_RST_Pin GPIO_PIN_14
#define NRF54_RST_GPIO_Port GPIOE
#define BUZZER_Pin GPIO_PIN_15
#define BUZZER_GPIO_Port GPIOB
#define MUX_COL_RST_Pin GPIO_PIN_7
#define MUX_COL_RST_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */
void AltMainTask(void *argument);
extern struct Vofa *gptr_vofa;
void CallVofaSendOneFrame(struct Vofa *ptr_vofa, uint16_t float_size);
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
