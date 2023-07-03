/* USER CODE BEGIN Header */
/**
******************************************************************************
* @file    fdcan.h
* @brief   This file contains all the function prototypes for
*          the fdcan.c file
******************************************************************************
* @attention
*
* Copyright (c) 2023 STMicroelectronics.
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
#define MOTOR_DEFAULT	{  0, def, 0, 0, 0, 0.0, 0, 0, 0, 0, {0, }, 0.0, (void (*)(uint32_t))Parsing_SDO,  (uint8_t (*)(uint32_t))Parsing_PDO}
typedef enum{
  OP			=	0x01,
  PRE			=	0x80,
  STOP			=	0x02,
  RESET_COMM	=	0x82,
  RESET_NODE	=	0x81
}NMT_state_t;

/*
(FR ->) DV -> SD ->EN
*/
typedef enum{
  FR	=	0x0080,		// Fault Reset
  SD	=	0x0006,		// Shut Down
  DO	=	0x0007,		// Disable Operation
  EN	=	0x000F,		// Enable Operation
  DV	=	0x0000,		// Disable Voltage
  SW	=	0x0004,		// Switch On
  QS	=	0x000B		// Quick Stop
}DS_state_t;

typedef enum{
  def				= 0x0000,
  MOP 				= 0x6060,
  CONTROLWORD		= 0x6040,
  STATUSWORD		= 0x6041,
  CAN_bit_rate 		= 0x2001,
  Error_code		= 0x603F,
  Position_actual	= 0x6064,
  Target_torque		= 0x6071,
  Error_history		= 0x1003,
  POWER_SUPPLY		= 0x2200,
  RATED_TRQ			= 0x6076,
  MOTOR_DATA		= 0x3001,
  MAX_MOTOR_SPEED	= 0x6080,
  TRQ_CONST			= 0x3001,
  Target_position	= 0x607A,
  RXPDO1			= 0x1600,
  PDO_OBJ
}Obj_dict_t;

typedef enum{
  SDO	= 0x580,
  PDO1	= 0x180,
  PDO2	= 0x280,
  PDO3	= 0x380
}Prot_info_t;

typedef enum{
  Overvoltage = 0x3210
}Error_code_t;

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

extern FDCAN_HandleTypeDef hfdcan1;

typedef struct{
  uint8_t		id;
  Obj_dict_t	Object;
  uint16_t		Statusword;
  int16_t		Torque_actual;
  int32_t		Postion_actual;
  double		angle;
  uint16_t		Controlword;
  int16_t		Target_torque;
  int32_t		Target_position;
  uint8_t		error_index;
  uint16_t		Error_code[5];
  int32_t		Position_zero;
  void (*parsing_SDO)();
  uint8_t (*parsing_PDO)();
} Motor_t;
extern Motor_t motor[4];

/* USER CODE END Private defines */

void MX_FDCAN1_Init(void);

/* USER CODE BEGIN Prototypes */

//  void NMT_TRANS(uint8_t id, NMT_state_t state);
void NMT_TRANS(NMT_state_t state);
void DS_TRANS(uint8_t id, DS_state_t state); // device status transistion
void SET_SDO(uint8_t id, uint8_t length, Obj_dict_t addr, uint8_t addr_sub, int32_t data);
void GET_SDO(uint8_t id, Obj_dict_t addr, uint8_t addr_sub);
//void SET_PDO(const Motor_t* motor);
void SET_PDO(uint8_t id);
void SYNC_FRAME(void);
void READ_STATUS(uint8_t id);
void SEND_FRAME(CAN_HandleTypeDef *h);
void GET_Angle(uint8_t id);
void INIT_CAN();
void Clear_Device_Errors(uint8_t id);
void TRQ_Calc();
void POS_Calc();
/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif

#endif /* __FDCAN_H__ */

