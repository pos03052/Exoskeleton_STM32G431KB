/* USER CODE BEGIN Header */
/**
******************************************************************************
* @file    i2c.c
* @brief   This file provides code for the configuration
*          of the I2C instances.
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
#include "i2c.h"

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

I2C_HandleTypeDef hi2c1;

/* I2C1 init function */
void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */
  
  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */
  
  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x10802D9B;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */
  
  /* USER CODE END I2C1_Init 2 */

}

void HAL_I2C_MspInit(I2C_HandleTypeDef* i2cHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};
  if(i2cHandle->Instance==I2C1)
  {
  /* USER CODE BEGIN I2C1_MspInit 0 */
	//	__HAL_RCC_I2C2_FORCE_RESET();
	//	HAL_Delay(200);
	//	__HAL_RCC_I2C2_RELEASE_RESET();
  /* USER CODE END I2C1_MspInit 0 */

  /** Initializes the peripherals clocks
  */
    PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_I2C1;
    PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_PCLK1;
    if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
    {
      Error_Handler();
    }

    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    /**I2C1 GPIO Configuration
    PA15     ------> I2C1_SCL
    PB7     ------> I2C1_SDA
    */
    GPIO_InitStruct.Pin = GPIO_PIN_15;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF4_I2C1;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_7;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF4_I2C1;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /* I2C1 clock enable */
    __HAL_RCC_I2C1_CLK_ENABLE();

    /* I2C1 interrupt Init */
    HAL_NVIC_SetPriority(I2C1_EV_IRQn, 2, 0);
    HAL_NVIC_EnableIRQ(I2C1_EV_IRQn);
    HAL_NVIC_SetPriority(I2C1_ER_IRQn, 2, 0);
    HAL_NVIC_EnableIRQ(I2C1_ER_IRQn);
  /* USER CODE BEGIN I2C1_MspInit 1 */
	
  /* USER CODE END I2C1_MspInit 1 */
  }
}

void HAL_I2C_MspDeInit(I2C_HandleTypeDef* i2cHandle)
{

  if(i2cHandle->Instance==I2C1)
  {
  /* USER CODE BEGIN I2C1_MspDeInit 0 */
	
  /* USER CODE END I2C1_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_I2C1_CLK_DISABLE();

    /**I2C1 GPIO Configuration
    PA15     ------> I2C1_SCL
    PB7     ------> I2C1_SDA
    */
    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_15);

    HAL_GPIO_DeInit(GPIOB, GPIO_PIN_7);

    /* I2C1 interrupt Deinit */
    HAL_NVIC_DisableIRQ(I2C1_EV_IRQn);
    HAL_NVIC_DisableIRQ(I2C1_ER_IRQn);
  /* USER CODE BEGIN I2C1_MspDeInit 1 */
	
  /* USER CODE END I2C1_MspDeInit 1 */
  }
}

/* USER CODE BEGIN 1 */

HAL_StatusTypeDef ret;
uint8_t tx_buf[12];
uint8_t rx_buf[24];
uint16_t angle_addr = 0x3d;
float angle_i2c[12] = {0, };
uint16_t max_timeout = 20;
float temp_c;


//static const uint8_t REG_YAW = 0x3B;

void I2C_COMM(){
  static double coef = 32768 / 180;
//  HAL_I2C_Mem_Write(&hi2c1, (uint16_t)(0x51 << 1), angle_addr, 1, tx_buf, 1, max_timeout);
//  //  while(__HAL_I2C_GET_FLAG(&hi2c1, I2C_FLAG_BUSY) == SET){
//  ////	__HAL_I2C_CLEAR_FLAG(&hi2c1, I2C_FLAG_STOPF);
//  //	__HAL_I2C_CLEAR_FLAG(&hi2c1, I2C_FLAG_BUSY);
//  //	I2C_RESET_CR2(&hi2c1);
//  //	temp_c = temp_c;
//  //  }
//  while(HAL_I2C_GetState(&hi2c1) == HAL_I2C_STATE_BUSY);
//  while(HAL_I2C_GetState(&hi2c1) == HAL_I2C_STATE_BUSY_RX);
//  while(HAL_I2C_GetState(&hi2c1) == HAL_I2C_STATE_BUSY_TX);
//  while(HAL_I2C_GetState(&hi2c1) != HAL_I2C_STATE_READY);
//  HAL_Delay(1);
//  HAL_I2C_Mem_Read(&hi2c1, (uint16_t)(0x51 << 1), angle_addr, 1, rx_buf, 6, max_timeout);
//  //  HAL_Delay(1);
  
  HAL_I2C_Mem_Write(&hi2c1, (uint16_t)(0x50 << 1), angle_addr, 1, tx_buf, 1, max_timeout);  
  HAL_Delay(1);  
  while(HAL_I2C_GetState(&hi2c1) == HAL_I2C_STATE_BUSY);
  while(HAL_I2C_GetState(&hi2c1) == HAL_I2C_STATE_BUSY_RX);
  while(HAL_I2C_GetState(&hi2c1) == HAL_I2C_STATE_BUSY_TX);
  while(HAL_I2C_GetState(&hi2c1) != HAL_I2C_STATE_READY);  
  HAL_I2C_Mem_Read(&hi2c1, (uint16_t)(0x50 << 1), angle_addr, 1, &rx_buf[6], 6, max_timeout);
  while(HAL_I2C_GetState(&hi2c1) == HAL_I2C_STATE_BUSY);
  while(HAL_I2C_GetState(&hi2c1) == HAL_I2C_STATE_BUSY_RX);
  while(HAL_I2C_GetState(&hi2c1) == HAL_I2C_STATE_BUSY_TX);
  while(HAL_I2C_GetState(&hi2c1) != HAL_I2C_STATE_READY);
  
//  HAL_I2C_Mem_Write(&hi2c1, (uint16_t)(0x53 << 1), angle_addr, 1, tx_buf, 1, max_timeout);  
//  HAL_Delay(1);  
//  while(HAL_I2C_GetState(&hi2c1) == HAL_I2C_STATE_BUSY);
//  while(HAL_I2C_GetState(&hi2c1) == HAL_I2C_STATE_BUSY_RX);
//  while(HAL_I2C_GetState(&hi2c1) == HAL_I2C_STATE_BUSY_TX);
//  while(HAL_I2C_GetState(&hi2c1) != HAL_I2C_STATE_READY);
//  HAL_I2C_Mem_Read(&hi2c1, (uint16_t)(0x53 << 1), angle_addr, 1, &rx_buf[12], 6, max_timeout);
//  while(HAL_I2C_GetState(&hi2c1) == HAL_I2C_STATE_BUSY);
//  while(HAL_I2C_GetState(&hi2c1) == HAL_I2C_STATE_BUSY_RX);
//  while(HAL_I2C_GetState(&hi2c1) == HAL_I2C_STATE_BUSY_TX);
//  while(HAL_I2C_GetState(&hi2c1) != HAL_I2C_STATE_READY);
//  HAL_I2C_Mem_Write(&hi2c1, (uint16_t)(0x52 << 1), angle_addr, 1, tx_buf, 1, max_timeout);  
//  HAL_Delay(1);  
//  while(HAL_I2C_GetState(&hi2c1) == HAL_I2C_STATE_BUSY);
//  while(HAL_I2C_GetState(&hi2c1) == HAL_I2C_STATE_BUSY_RX);
//  while(HAL_I2C_GetState(&hi2c1) == HAL_I2C_STATE_BUSY_TX);
//  while(HAL_I2C_GetState(&hi2c1) != HAL_I2C_STATE_READY);
//  HAL_I2C_Mem_Read(&hi2c1, (uint16_t)(0x52 << 1), angle_addr, 1, &rx_buf[18], 6, max_timeout);
//  while(HAL_I2C_GetState(&hi2c1) == HAL_I2C_STATE_BUSY);
//  while(HAL_I2C_GetState(&hi2c1) == HAL_I2C_STATE_BUSY_RX);
//  while(HAL_I2C_GetState(&hi2c1) == HAL_I2C_STATE_BUSY_TX);
//  while(HAL_I2C_GetState(&hi2c1) != HAL_I2C_STATE_READY);
  
//  angle_i2c[0] = ((int16_t)(rx_buf[0] | rx_buf[1] << 8)) / coef;
//  angle_i2c[1] = ((int16_t)(rx_buf[2] | rx_buf[3] << 8)) / coef;
//  angle_i2c[2] = ((int16_t)(rx_buf[4] | rx_buf[5] << 8)) / coef;
  angle_i2c[3] = ((int16_t)(rx_buf[6] | rx_buf[7] << 8)) / coef;
  angle_i2c[4] = ((int16_t)(rx_buf[8] | rx_buf[9] << 8)) / coef;
  angle_i2c[5] = ((int16_t)(rx_buf[10] | rx_buf[11] << 8)) / coef;
//  angle_i2c[6] = ((int16_t)(rx_buf[12] | rx_buf[13] << 8)) / coef;
//  angle_i2c[7] = ((int16_t)(rx_buf[14] | rx_buf[15] << 8)) / coef;
//  angle_i2c[8] = ((int16_t)(rx_buf[16] | rx_buf[17] << 8)) / coef;
//  angle_i2c[9] = ((int16_t)(rx_buf[18] | rx_buf[19] << 8)) / coef;
//  angle_i2c[10] = ((int16_t)(rx_buf[20] | rx_buf[21] << 8)) / coef;
//  angle_i2c[11] = ((int16_t)(rx_buf[22] | rx_buf[23] << 8)) / coef;  
}
/* USER CODE END 1 */
