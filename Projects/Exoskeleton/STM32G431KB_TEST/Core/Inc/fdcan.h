/**
  ******************************************************************************
  * @file    fdcan.h
  * @brief   This file contains all the function prototypes for
  *          the fdcan.c file
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __FDCAN_H__
#define __FDCAN_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

extern FDCAN_HandleTypeDef hfdcan1;

/* USER CODE BEGIN Private defines */
typedef struct {
	FDCAN_TxHeaderTypeDef header;
	uint8_t data[8];
} CAN_TxMsgTypeDef;

typedef struct {
	FDCAN_RxHeaderTypeDef header;
	uint8_t data[8];
} CAN_RxMsgTypeDef;

typedef struct {
	FDCAN_HandleTypeDef* module;
	CAN_TxMsgTypeDef txmsg;
	CAN_RxMsgTypeDef rxmsg;
	uint32_t txloc;
	uint32_t rxloc;
	uint32_t activeITs;
} CAN_HandleTypeDef;

extern CAN_HandleTypeDef hcan;
/* USER CODE END Private defines */

void MX_FDCAN1_Init(void);

/* USER CODE BEGIN Prototypes */

/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif

#endif /* __FDCAN_H__ */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/