/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    fdcan.c
  * @brief   This file provides code for the configuration
  *          of the FDCAN instances.
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
/* Includes ------------------------------------------------------------------*/
#include "fdcan.h"

/* USER CODE BEGIN 0 */
CAN_HandleTypeDef hcan = {
	&hfdcan1, 
	{0,}, 
	{0,}, 
	FDCAN_TX_BUFFER0,				// FDCAN_TX_BUFFER0, 1, 2
	FDCAN_RX_FIFO0,					// FDCAN_RX_FIFO0, 1
	FDCAN_IT_RX_FIFO0_NEW_MESSAGE,	// FDCANIT_RX_FIFO0_MESSAGE_LOST/FULL/NEW_MESSAGE(New message written to Rx FIFO 0)
};
Motor_t motor_[4];
/* USER CODE END 0 */

FDCAN_HandleTypeDef hfdcan1;

/* FDCAN1 init function */
void MX_FDCAN1_Init(void)
{

  /* USER CODE BEGIN FDCAN1_Init 0 */

  /* USER CODE END FDCAN1_Init 0 */

  /* USER CODE BEGIN FDCAN1_Init 1 */

  /* USER CODE END FDCAN1_Init 1 */
  hfdcan1.Instance = FDCAN1;
  hfdcan1.Init.ClockDivider = FDCAN_CLOCK_DIV1;
  hfdcan1.Init.FrameFormat = FDCAN_FRAME_CLASSIC;
  hfdcan1.Init.Mode = FDCAN_MODE_NORMAL;
  hfdcan1.Init.AutoRetransmission = ENABLE;
  hfdcan1.Init.TransmitPause = DISABLE;
  hfdcan1.Init.ProtocolException = DISABLE;
  hfdcan1.Init.NominalPrescaler = 10;
  hfdcan1.Init.NominalSyncJumpWidth = 1;
  hfdcan1.Init.NominalTimeSeg1 = 14;
  hfdcan1.Init.NominalTimeSeg2 = 2;
  hfdcan1.Init.DataPrescaler = 1;
  hfdcan1.Init.DataSyncJumpWidth = 1;
  hfdcan1.Init.DataTimeSeg1 = 1;
  hfdcan1.Init.DataTimeSeg2 = 1;
  hfdcan1.Init.StdFiltersNbr = 28;
  hfdcan1.Init.ExtFiltersNbr = 0;
  hfdcan1.Init.TxFifoQueueMode = FDCAN_TX_FIFO_OPERATION;
  if (HAL_FDCAN_Init(&hfdcan1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN FDCAN1_Init 2 */
  FDCAN_FilterTypeDef sFilterConfig; // handle for filter config
  /* 나중에 exteded ID가 들어오는 경우 예외처리 하기 또는 유입 막기 */
  sFilterConfig.IdType = FDCAN_STANDARD_ID;				// identifier type, FDCAN_STANDARD_ID, FDCAN_EXTENDED_ID
  sFilterConfig.FilterIndex = 0;						// filter which will be initialized, 0 ~ SRAMCAN_FLS_NBR-1
  sFilterConfig.FilterType = FDCAN_FILTER_RANGE;		// FDCAN_FILTER_RANGE: FilterID1 to FilterID2, DUAL: Dual ID filter, MASK: FilterID1=filter, FilterID2 = mask
  sFilterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO0; // 
  sFilterConfig.FilterID1 = 0x000; // from
  sFilterConfig.FilterID2 = 0x7ff; // to
  
  if (HAL_FDCAN_ConfigFilter(&hfdcan1, &sFilterConfig) != HAL_OK)
  {
	Error_Handler();
  }	  
  /* USER CODE END FDCAN1_Init 2 */

}

void HAL_FDCAN_MspInit(FDCAN_HandleTypeDef *fdcanHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};
  if(fdcanHandle->Instance==FDCAN1)
  {
  /* USER CODE BEGIN FDCAN1_MspInit 0 */

  /* USER CODE END FDCAN1_MspInit 0 */

  /** Initializes the peripherals clocks
  */
    PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_FDCAN;
    PeriphClkInit.FdcanClockSelection = RCC_FDCANCLKSOURCE_PCLK1;
    if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
    {
      Error_Handler();
    }

    /* FDCAN1 clock enable */
    __HAL_RCC_FDCAN_CLK_ENABLE();

    __HAL_RCC_GPIOA_CLK_ENABLE();
    /**FDCAN1 GPIO Configuration
    PA11     ------> FDCAN1_RX
    PA12     ------> FDCAN1_TX
    */
    GPIO_InitStruct.Pin = GPIO_PIN_11|GPIO_PIN_12;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF9_FDCAN1;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* FDCAN1 interrupt Init */
    HAL_NVIC_SetPriority(FDCAN1_IT0_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(FDCAN1_IT0_IRQn);
    HAL_NVIC_SetPriority(FDCAN1_IT1_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(FDCAN1_IT1_IRQn);
  /* USER CODE BEGIN FDCAN1_MspInit 1 */

  /* USER CODE END FDCAN1_MspInit 1 */
  }
}

void HAL_FDCAN_MspDeInit(FDCAN_HandleTypeDef* fdcanHandle)
{

  if(fdcanHandle->Instance==FDCAN1)
  {
  /* USER CODE BEGIN FDCAN1_MspDeInit 0 */

  /* USER CODE END FDCAN1_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_FDCAN_CLK_DISABLE();

    /**FDCAN1 GPIO Configuration
    PA11     ------> FDCAN1_RX
    PA12     ------> FDCAN1_TX
    */
    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_11|GPIO_PIN_12);

    /* FDCAN1 interrupt Deinit */
    HAL_NVIC_DisableIRQ(FDCAN1_IT0_IRQn);
    HAL_NVIC_DisableIRQ(FDCAN1_IT1_IRQn);
  /* USER CODE BEGIN FDCAN1_MspDeInit 1 */

  /* USER CODE END FDCAN1_MspDeInit 1 */
  }
}

/* USER CODE BEGIN 1 */
void SET_SDO(uint8_t id, uint8_t length, Obj_dict_t addr, uint8_t addr_sub, int32_t data){
  hcan.txmsg.header.Identifier = 0x600 + id;
  switch (length){
  case 1:
	hcan.txmsg.data[0] = 0x2F;
	hcan.txmsg.data[4] = data;
	break;	
  case 2:
	hcan.txmsg.data[0] = 0x2B;
	hcan.txmsg.data[4] = data & 0xff;
	hcan.txmsg.data[5] = (data >> 8) & 0xff;
	break;
  case 4:
	hcan.txmsg.data[0] = 0x23;
	hcan.txmsg.data[4] = data & 0xff;
	hcan.txmsg.data[5] = (data >> 8) & 0xff;
	hcan.txmsg.data[6] = ((data >> 8)>>8) & 0xff;
	hcan.txmsg.data[7] = (((data >> 8)>>8)>>8) & 0xff;
	break;
  default:
	hcan.txmsg.data[0] = 0x22;
  }
  hcan.txmsg.data[1] = addr & 0xFF;
  hcan.txmsg.data[2] = (addr >> 8) & 0xFF;  
  hcan.txmsg.data[3] = addr_sub;
  
  SEND_FRAME(&hcan);
}
void SET_PDO(const Motor_t* h){ // const: 주소값 안의 내용만 참조 
  hcan.txmsg.header.Identifier = 0x200 + h->id;
  hcan.txmsg.data[0] = h->Controlword& 0xff;
  hcan.txmsg.data[1] = (h->Controlword >> 8) & 0xff;
  hcan.txmsg.data[2] = h->Target_torque & 0xff;
  hcan.txmsg.data[3] = (h->Target_torque >> 8) & 0xff;  
  
  SEND_FRAME(&hcan); 
}
void NMT_TRANS(NMT_state_t state){
  memset(hcan.txmsg.data, 0, 8);
  hcan.txmsg.header.Identifier = 0x00;
  hcan.txmsg.data[0] = state;
  //  hcan.txmsg.data[1] = id;
  hcan.txmsg.data[1] = 0; // all nodes
  
  SEND_FRAME(&hcan);
}
void DS_TRANS(uint8_t id, DS_state_t state){
  SET_SDO(id, sizeof(uint16_t), CONTROLWORD, 0, state);
}
void Parsing_SDO(Motor_t *h, uint8_t id){  
  h->id			= id + 1;
  h->Object	= (Obj_dict_t)(hcan.rxmsg.data[1] | (hcan.rxmsg.data[2] << 8));
  switch(h->Object){
  case STATUSWORD:
	h->Statusword = hcan.rxmsg.data[4] | (hcan.rxmsg.data[5] << 8);
	break;
  }
}
void Parsing_PDO(Motor_t *h, uint8_t id){
  h->id = id + 1;
  h->Object = STATUSWORD;
  h->Statusword = hcan.rxmsg.data[0] | (hcan.rxmsg.data[1] << 8);
  h->Torque_actual = hcan.rxmsg.data[2] | (hcan.rxmsg.data[3] << 8);
  h->Postion_actual = hcan.rxmsg.data[4] | (hcan.rxmsg.data[5] << 8) | (hcan.rxmsg.data[6] << 16) | (hcan.rxmsg.data[7] << 24);
}
void SYNC_FRAME(void){
  hcan.txmsg.header.Identifier = 0x80;
  memset(hcan.txmsg.data,0,8);  
  SEND_FRAME(&hcan);
}
void Calc_torq(){
  
}

void SEND_FRAME(CAN_HandleTypeDef *h){
  
  // 	while(!__HAL_FDCAN_GET_FLAG(&hfdcan1, FDCAN_FLAG_TX_FIFO_EMPTY));
  //	if (HAL_FDCAN_GetTxFifoFreeLevel(hcan.module) > 0)
  while(HAL_FDCAN_GetTxFifoFreeLevel(h->module) <= 0);
  if (HAL_FDCAN_AddMessageToTxFifoQ(h->module, &h->txmsg.header, h->txmsg.data) != HAL_OK){
	Error_Handler();
  }
  //  if(HAL_FDCAN_GetTxFifoFreeLevel(hcan.module) > 0)	{ 
  
  if(__HAL_FDCAN_GET_FLAG(h->module, FDCAN_FLAG_TX_EVT_FIFO_ELT_LOST)){
	while(1);
  } 
}
void GET_SDO(uint8_t id, Obj_dict_t addr, uint8_t addr_sub){
  hcan.txmsg.header.Identifier = 0x600 + id;
  hcan.txmsg.data[0] = 0x40;  
  hcan.txmsg.data[1] = addr & 0xFF;
  hcan.txmsg.data[2] = (addr >> 8) & 0xFF;  
  hcan.txmsg.data[3] = addr_sub;
  if(HAL_FDCAN_GetTxFifoFreeLevel(hcan.module) > 0)	{
	if (HAL_FDCAN_AddMessageToTxFifoQ(hcan.module, &hcan.txmsg.header, hcan.txmsg.data) != HAL_OK){
	  Error_Handler();
	}
  }	    
}
void READ_STATUS(uint8_t id){
  GET_SDO(id, STATUSWORD, 0);
}

Motor_t motor[4] = {
MOTOR_DEFAULT,
MOTOR_DEFAULT,
MOTOR_DEFAULT,
MOTOR_DEFAULT
};
/* USER CODE END 1 */
