/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file         stm32g4xx_hal_msp.c
  * @brief        This file provides code for the MSP Initialization
  *               and de-Initialization codes.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */

/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN Define */

/* USER CODE END Define */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN Macro */

/* USER CODE END Macro */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* External functions --------------------------------------------------------*/
/* USER CODE BEGIN ExternalFunctions */

/* USER CODE END ExternalFunctions */

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);
                                        /**
  * Initializes the Global MSP.
  */
void HAL_MspInit(void)
{
  /* USER CODE BEGIN MspInit 0 */

  /* USER CODE END MspInit 0 */

  __HAL_RCC_SYSCFG_CLK_ENABLE();
  __HAL_RCC_PWR_CLK_ENABLE();

  HAL_NVIC_SetPriorityGrouping(NVIC_PRIORITYGROUP_3);

  /* System interrupt init*/

  /** Disable the internal Pull-Up in Dead Battery pins of UCPD peripheral
  */
  HAL_PWREx_DisableUCPDDeadBattery();

  /* USER CODE BEGIN MspInit 1 */

  /* USER CODE END MspInit 1 */
}

static uint32_t HAL_RCC_ADC12_CLK_ENABLED=0;
static uint32_t HAL_RCC_ADC345_CLK_ENABLED=0;

/**
* @brief ADC MSP Initialization
* This function configures the hardware resources used in this example
* @param hadc: ADC handle pointer
* @retval None
*/
extern uint32_t ADC_ConvertedValue[12];
DMA_HandleTypeDef hdma_adc1;
void HAL_ADC_MspInit(ADC_HandleTypeDef* hadc)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(hadc->Instance==ADC1)
  {
  /* USER CODE BEGIN ADC1_MspInit 0 */

  /* USER CODE END ADC1_MspInit 0 */
    /* Peripheral clock enable */
//    HAL_RCC_ADC12_CLK_ENABLED++;
//    if(HAL_RCC_ADC12_CLK_ENABLED==1){
//      __HAL_RCC_ADC12_CLK_ENABLE();
//    }

//    __HAL_RCC_GPIOC_CLK_ENABLE();
//    /**ADC1 GPIO Configuration
//    PC2     ------> ADC1_IN8
//    PC3     ------> ADC1_IN9
//    */
//    GPIO_InitStruct.Pin = M1_CURR_AMPL_U_Pin|M1_CURR_AMPL_V_Pin;
//    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
//    GPIO_InitStruct.Pull = GPIO_NOPULL;
//    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

		
		
		//新添加的
		__HAL_RCC_ADC12_CLK_ENABLE();
    __HAL_RCC_DMA1_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();
		__HAL_RCC_GPIOC_CLK_ENABLE();
    /**ADC1 GPIO Configuration    
    PA1     ------> ADC1_IN2 
    */
    GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
		
//		 GPIO_InitStruct.Pin = GPIO_PIN_3;         /*测试用，adc1通道3*/
//    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
//    GPIO_InitStruct.Pull = GPIO_NOPULL;
//    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
  /* USER CODE BEGIN ADC1_MspInit 1 */
   /* ADC1 DMA Init */
    /* ADC1 Init */


  /* USER CODE END ADC1_MspInit 1 */
  }
  else if(hadc->Instance==ADC2)
  {
  /* USER CODE BEGIN ADC2_MspInit 0 */

  /* USER CODE END ADC2_MspInit 0 */
    /* Peripheral clock enable */
    HAL_RCC_ADC12_CLK_ENABLED++;


    __HAL_RCC_GPIOA_CLK_ENABLE();
		__HAL_RCC_GPIOC_CLK_ENABLE();
    /**ADC2 GPIO Configuration
    PC0     ------> ADC2_IN6
    PC1     ------> ADC2_IN7
    PC3     ------> ADC2_IN9
    PC4     ------> ADC2_IN5
    */
		 GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_4|GPIO_PIN_6;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
		
		GPIO_InitStruct.Pin = GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_3;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
//    GPIO_InitStruct.Pin = M1_CURR_AMPL_W_Pin|M1_BUS_VOLTAGE_Pin|M1_CURR_AMPL_V_Pin|M1_TEMPERATURE_Pin;
//    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
//    GPIO_InitStruct.Pull = GPIO_NOPULL;
//    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /* USER CODE BEGIN ADC2_MspInit 1 */

  /* USER CODE END ADC2_MspInit 1 */
  }
//  else if(hadc->Instance==ADC3)
//  {
//  /* USER CODE BEGIN ADC3_MspInit 0 */

//  /* USER CODE END ADC3_MspInit 0 */
//    /* Peripheral clock enable */
//    HAL_RCC_ADC345_CLK_ENABLED++;
//    if(HAL_RCC_ADC345_CLK_ENABLED==1)
//			{
//      __HAL_RCC_ADC345_CLK_ENABLE();
//      }
//    __HAL_RCC_GPIOE_CLK_ENABLE();
//    __HAL_RCC_GPIOD_CLK_ENABLE();
//    /**ADC3 GPIO Configuration
//    PE7     ------> ADC3_IN4
//    PD10     ------> ADC3_IN7
//    PD12     ------> ADC3_IN9
//    */
////    GPIO_InitStruct.Pin = M2_TEMPERATURE_Pin;
////    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
////    GPIO_InitStruct.Pull = GPIO_NOPULL;
////    HAL_GPIO_Init(M2_TEMPERATURE_GPIO_Port, &GPIO_InitStruct);

////    GPIO_InitStruct.Pin = M2_CURR_AMPL_U_Pin|M2_CURR_AMPL_V_Pin;
////    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
////    GPIO_InitStruct.Pull = GPIO_NOPULL;
////    HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

//  /* USER CODE BEGIN ADC3_MspInit 1 */

//  /* USER CODE END ADC3_MspInit 1 */
//  }
//  else if(hadc->Instance==ADC4)
//  {
//  /* USER CODE BEGIN ADC4_MspInit 0 */

//  /* USER CODE END ADC4_MspInit 0 */
//    /* Peripheral clock enable */
//    HAL_RCC_ADC345_CLK_ENABLED++;
//    if(HAL_RCC_ADC345_CLK_ENABLED==1){
//      __HAL_RCC_ADC345_CLK_ENABLE();
//    }

//    __HAL_RCC_GPIOE_CLK_ENABLE();
//    __HAL_RCC_GPIOD_CLK_ENABLE();
//    /**ADC4 GPIO Configuration
//    PE14     ------> ADC4_IN1
//    PD12     ------> ADC4_IN9
//    PD13     ------> ADC4_IN10
//    */
////    GPIO_InitStruct.Pin = M2_BUS_VOLTAGE_Pin;
////    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
////    GPIO_InitStruct.Pull = GPIO_NOPULL;
////    HAL_GPIO_Init(M2_BUS_VOLTAGE_GPIO_Port, &GPIO_InitStruct);

////    GPIO_InitStruct.Pin = M2_CURR_AMPL_V_Pin|M2_CURR_AMPL_W_Pin;
////    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
////    GPIO_InitStruct.Pull = GPIO_NOPULL;
////    HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

//  /* USER CODE BEGIN ADC4_MspInit 1 */

//  /* USER CODE END ADC4_MspInit 1 */
//  }

}

/**
* @brief ADC MSP De-Initialization
* This function freeze the hardware resources used in this example
* @param hadc: ADC handle pointer
* @retval None
*/
void HAL_ADC_MspDeInit(ADC_HandleTypeDef* hadc)
{
  if(hadc->Instance==ADC1)
  {
  /* USER CODE BEGIN ADC1_MspDeInit 0 */

  /* USER CODE END ADC1_MspDeInit 0 */
    /* Peripheral clock disable */
//    HAL_RCC_ADC12_CLK_ENABLED--;
//    if(HAL_RCC_ADC12_CLK_ENABLED==0){
//      __HAL_RCC_ADC12_CLK_DISABLE();
//    }

//    /**ADC1 GPIO Configuration
//    PC2     ------> ADC1_IN8
//    PC3     ------> ADC1_IN9
//    */
//    HAL_GPIO_DeInit(GPIOC, M1_CURR_AMPL_U_Pin|M1_CURR_AMPL_V_Pin);

		 __HAL_RCC_ADC12_CLK_DISABLE();
  
    /**ADC1 GPIO Configuration    
    PA1     ------> ADC1_IN2 
    */
    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_1);
		
    /* ADC1 interrupt DeInit */
  /* USER CODE BEGIN ADC1:ADC1_2_IRQn disable */
    /**
    * Uncomment the line below to disable the "ADC1_2_IRQn" interrupt
    * Be aware, disabling shared interrupt may affect other IPs
    */
    /* HAL_NVIC_DisableIRQ(ADC1_2_IRQn); */
  /* USER CODE END ADC1:ADC1_2_IRQn disable */

  /* USER CODE BEGIN ADC1_MspDeInit 1 */

  /* USER CODE END ADC1_MspDeInit 1 */
  }
  else if(hadc->Instance==ADC2)
  {
  /* USER CODE BEGIN ADC2_MspDeInit 0 */

  /* USER CODE END ADC2_MspDeInit 0 */
    /* Peripheral clock disable */
    HAL_RCC_ADC12_CLK_ENABLED--;
    if(HAL_RCC_ADC12_CLK_ENABLED==0){
      __HAL_RCC_ADC12_CLK_DISABLE();
    }

    /**ADC2 GPIO Configuration
    PC0     ------> ADC2_IN6
    PC1     ------> ADC2_IN7
    PC3     ------> ADC2_IN9
    PC4     ------> ADC2_IN5
    */
//    HAL_GPIO_DeInit(GPIOC, M1_CURR_AMPL_W_Pin|M1_BUS_VOLTAGE_Pin|M1_CURR_AMPL_V_Pin|M1_TEMPERATURE_Pin);

    /* ADC2 interrupt DeInit */
  /* USER CODE BEGIN ADC2:ADC1_2_IRQn disable */
    /**
    * Uncomment the line below to disable the "ADC1_2_IRQn" interrupt
    * Be aware, disabling shared interrupt may affect other IPs
    */
    /* HAL_NVIC_DisableIRQ(ADC1_2_IRQn); */
  /* USER CODE END ADC2:ADC1_2_IRQn disable */

  /* USER CODE BEGIN ADC2_MspDeInit 1 */

  /* USER CODE END ADC2_MspDeInit 1 */
  }
//  else if(hadc->Instance==ADC3)
//  {
//  /* USER CODE BEGIN ADC3_MspDeInit 0 */

//  /* USER CODE END ADC3_MspDeInit 0 */
//    /* Peripheral clock disable */
//    HAL_RCC_ADC345_CLK_ENABLED--;
//    if(HAL_RCC_ADC345_CLK_ENABLED==0){
//      __HAL_RCC_ADC345_CLK_DISABLE();
//    }

//    /**ADC3 GPIO Configuration
//    PE7     ------> ADC3_IN4
//    PD10     ------> ADC3_IN7
//    PD12     ------> ADC3_IN9
//    */
////    HAL_GPIO_DeInit(M2_TEMPERATURE_GPIO_Port, M2_TEMPERATURE_Pin);

////    HAL_GPIO_DeInit(GPIOD, M2_CURR_AMPL_U_Pin|M2_CURR_AMPL_V_Pin);

//    /* ADC3 interrupt DeInit */
////    HAL_NVIC_DisableIRQ(ADC3_IRQn);
//  /* USER CODE BEGIN ADC3_MspDeInit 1 */

//  /* USER CODE END ADC3_MspDeInit 1 */
//  }
//  else if(hadc->Instance==ADC4)
//  {
//  /* USER CODE BEGIN ADC4_MspDeInit 0 */

//  /* USER CODE END ADC4_MspDeInit 0 */
//    /* Peripheral clock disable */
//    HAL_RCC_ADC345_CLK_ENABLED--;
//    if(HAL_RCC_ADC345_CLK_ENABLED==0){
//      __HAL_RCC_ADC345_CLK_DISABLE();
//    }

//    /**ADC4 GPIO Configuration
//    PE14     ------> ADC4_IN1
//    PD12     ------> ADC4_IN9
//    PD13     ------> ADC4_IN10
//    */
////    HAL_GPIO_DeInit(M2_BUS_VOLTAGE_GPIO_Port, M2_BUS_VOLTAGE_Pin);

////    HAL_GPIO_DeInit(GPIOD, M2_CURR_AMPL_V_Pin|M2_CURR_AMPL_W_Pin);

//  /* USER CODE BEGIN ADC4_MspDeInit 1 */

//  /* USER CODE END ADC4_MspDeInit 1 */
//  }

}

/**
* @brief CORDIC MSP Initialization
* This function configures the hardware resources used in this example
* @param hcordic: CORDIC handle pointer
* @retval None
*/
void HAL_CORDIC_MspInit(CORDIC_HandleTypeDef* hcordic)
{
  if(hcordic->Instance==CORDIC)
  {
  /* USER CODE BEGIN CORDIC_MspInit 0 */

  /* USER CODE END CORDIC_MspInit 0 */
    /* Peripheral clock enable */
    __HAL_RCC_CORDIC_CLK_ENABLE();
  /* USER CODE BEGIN CORDIC_MspInit 1 */

  /* USER CODE END CORDIC_MspInit 1 */
  }

}

/**
* @brief CORDIC MSP De-Initialization
* This function freeze the hardware resources used in this example
* @param hcordic: CORDIC handle pointer
* @retval None
*/
void HAL_CORDIC_MspDeInit(CORDIC_HandleTypeDef* hcordic)
{
  if(hcordic->Instance==CORDIC)
  {
  /* USER CODE BEGIN CORDIC_MspDeInit 0 */

  /* USER CODE END CORDIC_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_CORDIC_CLK_DISABLE();
  /* USER CODE BEGIN CORDIC_MspDeInit 1 */

  /* USER CODE END CORDIC_MspDeInit 1 */
  }

}

/**
* @brief DAC MSP Initialization
* This function configures the hardware resources used in this example
* @param hdac: DAC handle pointer
* @retval None
*/
void HAL_DAC_MspInit(DAC_HandleTypeDef* hdac)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(hdac->Instance==DAC1)
  {
  /* USER CODE BEGIN DAC1_MspInit 0 */

  /* USER CODE END DAC1_MspInit 0 */
    /* Peripheral clock enable */
    __HAL_RCC_DAC1_CLK_ENABLE();

//    __HAL_RCC_GPIOA_CLK_ENABLE();
//    /**DAC1 GPIO Configuration
//    PA4     ------> DAC1_OUT1
//    PA5     ------> DAC1_OUT2
//    */
////    GPIO_InitStruct.Pin = DBG_DAC_CH1_Pin|DBG_DAC_CH2_Pin;
//    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
//    GPIO_InitStruct.Pull = GPIO_NOPULL;
//    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* USER CODE BEGIN DAC1_MspInit 1 */

  /* USER CODE END DAC1_MspInit 1 */
  }
   if(hdac->Instance==DAC3)
  {
	    __HAL_RCC_DAC3_CLK_ENABLE();
  }

}

/**
* @brief DAC MSP De-Initialization
* This function freeze the hardware resources used in this example
* @param hdac: DAC handle pointer
* @retval None
*/
void HAL_DAC_MspDeInit(DAC_HandleTypeDef* hdac)
{
  if(hdac->Instance==DAC1)
  {
  /* USER CODE BEGIN DAC1_MspDeInit 0 */

  /* USER CODE END DAC1_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_DAC1_CLK_DISABLE();

    /**DAC1 GPIO Configuration
    PA4     ------> DAC1_OUT1
    PA5     ------> DAC1_OUT2
    */
//    HAL_GPIO_DeInit(GPIOA, DBG_DAC_CH1_Pin|DBG_DAC_CH2_Pin);

  /* USER CODE BEGIN DAC1_MspDeInit 1 */

  /* USER CODE END DAC1_MspDeInit 1 */
  }

}

/**
* @brief TIM_Base MSP Initialization
* This function configures the hardware resources used in this example
* @param htim_base: TIM_Base handle pointer
* @retval None
*/

//void HAL_TIM_MspPostInit(TIM_HandleTypeDef* timHandle)
//{

//  GPIO_InitTypeDef GPIO_InitStruct = {0};
//  if(timHandle->Instance==TIM1)
//  {
//  /* USER CODE BEGIN TIM1_MspPostInit 0 */

//  /* USER CODE END TIM1_MspPostInit 0 */
//  
//    __HAL_RCC_GPIOA_CLK_ENABLE();
//    /**TIM1 GPIO Configuration    
//    PA10     ------> TIM1_CH3 
//    */
//    GPIO_InitStruct.Pin = GPIO_PIN_10;
//    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
//    GPIO_InitStruct.Pull = GPIO_NOPULL;
//    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
//    GPIO_InitStruct.Alternate = GPIO_AF6_TIM1;
//    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

//  /* USER CODE BEGIN TIM1_MspPostInit 1 */

//  /* USER CODE END TIM1_MspPostInit 1 */
//  }

//}
void HAL_TIM_Base_MspInit(TIM_HandleTypeDef* htim_base)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(htim_base->Instance==TIM1)
  {
  /* USER CODE BEGIN TIM1_MspInit 0 */

  /* USER CODE END TIM1_MspInit 0 */
    /* Peripheral clock enable */
    __HAL_RCC_TIM1_CLK_ENABLE();

    __HAL_RCC_GPIOA_CLK_ENABLE();
    /**TIM1 GPIO Configuration
    PA11     ------> TIM1_BKIN2
    */
//    GPIO_InitStruct.Pin = M1_OCP_Pin;
//    GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
//    GPIO_InitStruct.Pull = GPIO_PULLUP;
//    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
//    GPIO_InitStruct.Alternate = GPIO_AF12_TIM1_COMP1;
//    HAL_GPIO_Init(M1_OCP_GPIO_Port, &GPIO_InitStruct);

  /* USER CODE BEGIN TIM1_MspInit 1 */

  /* USER CODE END TIM1_MspInit 1 */
  }
	
	
	 if(htim_base->Instance==TIM8)
  {
  /* USER CODE BEGIN TIM1_MspInit 0 */

  /* USER CODE END TIM1_MspInit 0 */
    /* Peripheral clock enable */
    __HAL_RCC_TIM8_CLK_ENABLE();

    __HAL_RCC_GPIOB_CLK_ENABLE();

  }
    if(htim_base->Instance==TIM16)
    {
	    __HAL_RCC_TIM16_CLK_ENABLE();
	}
	    if(htim_base->Instance==TIM15)
    {
	    __HAL_RCC_TIM15_CLK_ENABLE();
	}
	if(htim_base->Instance==TIM4)
    {
	    __HAL_RCC_TIM4_CLK_ENABLE();
	}
	
}

/**
* @brief TIM_Encoder MSP Initialization
* This function configures the hardware resources used in this example
* @param htim_encoder: TIM_Encoder handle pointer
* @retval None
*/
void HAL_TIM_Encoder_MspInit(TIM_HandleTypeDef* htim_encoder)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(htim_encoder->Instance==TIM2)
  {
  /* USER CODE BEGIN TIM2_MspInit 0 */

  /* USER CODE END TIM2_MspInit 0 */
    /* Peripheral clock enable */
    __HAL_RCC_TIM2_CLK_ENABLE();

    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOD_CLK_ENABLE();
    /**TIM2 GPIO Configuration
    PA0     ------> TIM2_CH1
    PD4     ------> TIM2_CH2
    */
//    GPIO_InitStruct.Pin = M1_ENCODER_A_Pin;
//    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
//    GPIO_InitStruct.Pull = GPIO_NOPULL;
//    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
//    GPIO_InitStruct.Alternate = GPIO_AF1_TIM2;
//    HAL_GPIO_Init(M1_ENCODER_A_GPIO_Port, &GPIO_InitStruct);

//    GPIO_InitStruct.Pin = M1_ENCODER_B_Pin;
//    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
//    GPIO_InitStruct.Pull = GPIO_NOPULL;
//    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
//    GPIO_InitStruct.Alternate = GPIO_AF2_TIM2;
//    HAL_GPIO_Init(M1_ENCODER_B_GPIO_Port, &GPIO_InitStruct);

  /* USER CODE BEGIN TIM2_MspInit 1 */

  /* USER CODE END TIM2_MspInit 1 */
  }


}

void HAL_TIM_MspPostInit(TIM_HandleTypeDef* htim)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(htim->Instance==TIM1)
  {
  /* USER CODE BEGIN TIM1_MspPostInit 0 */

  /* USER CODE END TIM1_MspPostInit 0 */
		
//    __HAL_RCC_GPIOC_CLK_ENABLE();
			__HAL_RCC_GPIOA_CLK_ENABLE();
//			__HAL_RCC_GPIOB_CLK_ENABLE();
			__HAL_RCC_GPIOE_CLK_ENABLE();
    /**TIM1 GPIO Configuration
    PC13     ------> TIM1_CH1N
    PA8     ------> TIM1_CH1
    PB0     ------> TIM1_CH2N
    PA9     ------> TIM1_CH2
    PF0     ------> TIM1_CH3N
    PA10    ------> TIM1_CH3
    */
//		GPIO_InitStruct.Pin = M1_PWM_UL_Pin;
//    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
//    GPIO_InitStruct.Pull = GPIO_PULLDOWN;
//    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
//    GPIO_InitStruct.Alternate = GPIO_AF2_TIM1;
//    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
		
	
    GPIO_InitStruct.Pin = M1_PWM_WH_Pin|M1_PWM_VH_Pin|M1_PWM_UH_Pin|M1_PWM_UL_Pin|M1_PWM_VL_Pin|M1_PWM_WL_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_PULLDOWN ;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF2_TIM1;
    HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);
//		GPIO_InitStruct.Pin = M1_PWM_VL_Pin;
//    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
//    GPIO_InitStruct.Pull = GPIO_PULLDOWN;
//    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
//    GPIO_InitStruct.Alternate = GPIO_AF2_TIM1;
//    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
//		
//    GPIO_InitStruct.Pin =M1_PWM_WL_Pin;
//    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
//    GPIO_InitStruct.Pull = GPIO_PULLDOWN;
//    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
//    GPIO_InitStruct.Alternate = GPIO_AF2_TIM1;
//    HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);
/*TEST*/
//     __HAL_RCC_GPIOA_CLK_ENABLE();
//    /**TIM1 GPIO Configuration    
//    PA10     ------> TIM1_CH3 
//    */
//    GPIO_InitStruct.Pin = GPIO_PIN_8;                   /*测试PWM输出用，正式版请屏蔽*/
//    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
//    GPIO_InitStruct.Pull = GPIO_NOPULL;
//    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
//    GPIO_InitStruct.Alternate = GPIO_AF6_TIM1;
//    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
  /* USER CODE BEGIN TIM1_MspPostInit 1 */

  /* USER CODE END TIM1_MspPostInit 1 */
  }
  else if(htim->Instance==TIM8)
  {
  /* USER CODE BEGIN TIM8_MspPostInit 0 */

  /* USER CODE END TIM8_MspPostInit 0 */
    __HAL_RCC_TIM8_CLK_ENABLE();
		__HAL_RCC_GPIOB_CLK_ENABLE();
    /**TIM8 GPIO Configuration
    PA15     ------> TIM8_CH1
    PA14     ------> TIM8_CH2
    PB9     ------> TIM8_CH3
    PA7     ------> TIM8_CH1N
    PB0     ------> TIM8_CH2N
    PB1     ------> TIM8_CH3N
    */
    GPIO_InitStruct.Pin = M2_PWM_UH_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed =GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF5_TIM8;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
		
		GPIO_InitStruct.Pin = M2_PWM_UL_Pin|M2_PWM_VL_Pin;     
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed =GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF4_TIM8;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
		
		GPIO_InitStruct.Pin = M2_PWM_VH_Pin|M2_PWM_WH_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed =GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF10_TIM8;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
		
		
		GPIO_InitStruct.Pin = M2_PWM_WL_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed =GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF3_TIM8;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
		
	

  /* USER CODE BEGIN TIM8_MspPostInit 1 */

  /* USER CODE END TIM8_MspPostInit 1 */
  }

}
/**
* @brief TIM_Base MSP De-Initialization
* This function freeze the hardware resources used in this example
* @param htim_base: TIM_Base handle pointer
* @retval None
*/
void HAL_TIM_Base_MspDeInit(TIM_HandleTypeDef* htim_base)
{
  if(htim_base->Instance==TIM1)
  {
  /* USER CODE BEGIN TIM1_MspDeInit 0 */

  /* USER CODE END TIM1_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_TIM1_CLK_DISABLE();

    /**TIM1 GPIO Configuration
    PE8     ------> TIM1_CH1N
    PE9     ------> TIM1_CH1
    PE10     ------> TIM1_CH2N
    PE11     ------> TIM1_CH2
    PE12     ------> TIM1_CH3N
    PE13     ------> TIM1_CH3
    PA11     ------> TIM1_BKIN2
    */
    HAL_GPIO_DeInit(GPIOE, M1_PWM_UL_Pin|M1_PWM_UH_Pin|M1_PWM_VL_Pin|M1_PWM_VH_Pin
                          |M1_PWM_WL_Pin|M1_PWM_WH_Pin);

//    HAL_GPIO_DeInit(M1_OCP_GPIO_Port, M1_OCP_Pin);

    /* TIM1 interrupt DeInit */
    HAL_NVIC_DisableIRQ(TIM1_BRK_TIM15_IRQn);
    HAL_NVIC_DisableIRQ(TIM1_UP_TIM16_IRQn);
  /* USER CODE BEGIN TIM1_MspDeInit 1 */

  /* USER CODE END TIM1_MspDeInit 1 */
  }
  else if(htim_base->Instance==TIM3)
  {
  /* USER CODE BEGIN TIM3_MspDeInit 0 */

  /* USER CODE END TIM3_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_TIM3_CLK_DISABLE();

    /**TIM3 GPIO Configuration
    PE2     ------> TIM3_CH1
    PB0     ------> TIM3_CH3
    PB5     ------> TIM3_CH2
    */
    HAL_GPIO_DeInit(M1_HALL_H1_GPIO_Port, M1_HALL_H1_Pin);

    HAL_GPIO_DeInit(GPIOB, M1_HALL_H3_Pin|M1_HALL_H2_Pin);

    /* TIM3 interrupt DeInit */
    HAL_NVIC_DisableIRQ(TIM3_IRQn);
  /* USER CODE BEGIN TIM3_MspDeInit 1 */

  /* USER CODE END TIM3_MspDeInit 1 */
  }
//  else if(htim_base->Instance==TIM5)
//  {
//  /* USER CODE BEGIN TIM5_MspDeInit 0 */

//  /* USER CODE END TIM5_MspDeInit 0 */
//    /* Peripheral clock disable */
//    __HAL_RCC_TIM5_CLK_DISABLE();

//    /**TIM5 GPIO Configuration
//    PF7     ------> TIM5_CH2
//    PF8     ------> TIM5_CH3
//    PF6     ------> TIM5_CH1
//    */
//    HAL_GPIO_DeInit(GPIOF, M2_HALL_H2_Pin|M2_HALL_H3_Pin|M2_HALL_H1_Pin);

//    /* TIM5 interrupt DeInit */
//    HAL_NVIC_DisableIRQ(TIM5_IRQn);
//  /* USER CODE BEGIN TIM5_MspDeInit 1 */

//  /* USER CODE END TIM5_MspDeInit 1 */
//  }
  else if(htim_base->Instance==TIM8)
  {
  /* USER CODE BEGIN TIM8_MspDeInit 0 */

  /* USER CODE END TIM8_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_TIM8_CLK_DISABLE();

    /**TIM8 GPIO Configuration
    PC6     ------> TIM8_CH1
    PC7     ------> TIM8_CH2
    PC8     ------> TIM8_CH3
    PC9     ------> TIM8_BKIN2
    PC10     ------> TIM8_CH1N
    PC11     ------> TIM8_CH2N
    PC12     ------> TIM8_CH3N
    */
    HAL_GPIO_DeInit(GPIOC, M2_PWM_UH_Pin|M2_PWM_VH_Pin|M2_PWM_WH_Pin
                          |M2_PWM_UL_Pin|M2_PWM_VL_Pin|M2_PWM_WL_Pin);

    /* TIM8 interrupt DeInit */
    HAL_NVIC_DisableIRQ(TIM8_BRK_IRQn);
    HAL_NVIC_DisableIRQ(TIM8_UP_IRQn);
  /* USER CODE BEGIN TIM8_MspDeInit 1 */

  /* USER CODE END TIM8_MspDeInit 1 */
  }

}

/**
* @brief TIM_Encoder MSP De-Initialization
* This function freeze the hardware resources used in this example
* @param htim_encoder: TIM_Encoder handle pointer
* @retval None
*/
void HAL_TIM_Encoder_MspDeInit(TIM_HandleTypeDef* htim_encoder)
{
  if(htim_encoder->Instance==TIM2)
  {
  /* USER CODE BEGIN TIM2_MspDeInit 0 */

  /* USER CODE END TIM2_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_TIM2_CLK_DISABLE();

    /**TIM2 GPIO Configuration
    PA0     ------> TIM2_CH1
    PD4     ------> TIM2_CH2
    */
//    HAL_GPIO_DeInit(M1_ENCODER_A_GPIO_Port, M1_ENCODER_A_Pin);

//    HAL_GPIO_DeInit(M1_ENCODER_B_GPIO_Port, M1_ENCODER_B_Pin);

    /* TIM2 interrupt DeInit */
    HAL_NVIC_DisableIRQ(TIM2_IRQn);
  /* USER CODE BEGIN TIM2_MspDeInit 1 */

  /* USER CODE END TIM2_MspDeInit 1 */
  }
  else if(htim_encoder->Instance==TIM4)
  {
  /* USER CODE BEGIN TIM4_MspDeInit 0 */

  /* USER CODE END TIM4_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_TIM4_CLK_DISABLE();

    /**TIM4 GPIO Configuration
    PB6     ------> TIM4_CH1
    PB7     ------> TIM4_CH2
    */
//    HAL_GPIO_DeInit(GPIOB, M2_ENCODER_A_Pin|M2_ENCODER_B_Pin);

    /* TIM4 interrupt DeInit */
    HAL_NVIC_DisableIRQ(TIM4_IRQn);
  /* USER CODE BEGIN TIM4_MspDeInit 1 */

  /* USER CODE END TIM4_MspDeInit 1 */
  }

}

/**
* @brief UART MSP Initialization
* This function configures the hardware resources used in this example
* @param huart: UART handle pointer
* @retval None
*/
void HAL_UART_MspInit(UART_HandleTypeDef* huart)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(huart->Instance==USART1)
  {
  /* USER CODE BEGIN USART1_MspInit 0 */

  /* USER CODE END USART1_MspInit 0 */
    /* Peripheral clock enable */
    __HAL_RCC_USART1_CLK_ENABLE();

    __HAL_RCC_GPIOB_CLK_ENABLE();
		 __HAL_RCC_GPIOA_CLK_ENABLE();
    /**USART1 GPIO Configuration
    PA9     ------> USART1_TX
    PA10     ------> USART1_RX
    */
    GPIO_InitStruct.Pin =  UART_TX_Pin|UART_RX_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF7_USART1;
    HAL_GPIO_Init(UART_TX_GPIO_Port, &GPIO_InitStruct);

  /* USER CODE BEGIN USART1_MspInit 1 */

  /* USER CODE END USART1_MspInit 1 */
  }

}

/**
* @brief UART MSP De-Initialization
* This function freeze the hardware resources used in this example
* @param huart: UART handle pointer
* @retval None
*/
void HAL_UART_MspDeInit(UART_HandleTypeDef* huart)
{
  if(huart->Instance==USART1)
  {
  /* USER CODE BEGIN USART1_MspDeInit 0 */

  /* USER CODE END USART1_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_USART1_CLK_DISABLE();

    /**USART1 GPIO Configuration
    PA9     ------> USART1_TX
    PA10     ------> USART1_RX
    */
    HAL_GPIO_DeInit(GPIOB, GPIO_PIN_6|GPIO_PIN_7);

    /* USART1 interrupt DeInit */
    HAL_NVIC_DisableIRQ(USART1_IRQn);
  /* USER CODE BEGIN USART1_MspDeInit 1 */

  /* USER CODE END USART1_MspDeInit 1 */
  }

}
void HAL_OPAMP_MspInit(OPAMP_HandleTypeDef* hopamp)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(hopamp->Instance==OPAMP1)
  {
  /* USER CODE BEGIN OPAMP1_MspInit 0 */

  /* USER CODE END OPAMP1_MspInit 0 */

    __HAL_RCC_GPIOA_CLK_ENABLE();
    /**OPAMP1 GPIO Configuration
    PA1     ------> OPAMP1_VINP
    PA2     ------> OPAMP1_VOUT
    PA3     ------> OPAMP1_VINM0
    */
    GPIO_InitStruct.Pin = GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* USER CODE BEGIN OPAMP1_MspInit 1 */

  /* USER CODE END OPAMP1_MspInit 1 */
  }
  else if(hopamp->Instance==OPAMP2)
  {
  /* USER CODE BEGIN OPAMP2_MspInit 0 */

  /* USER CODE END OPAMP2_MspInit 0 */

    __HAL_RCC_GPIOA_CLK_ENABLE();
    /**OPAMP2 GPIO Configuration
    PA5     ------> OPAMP2_VINM0
    PA6     ------> OPAMP2_VOUT
    PA7     ------> OPAMP2_VINP
    */
    GPIO_InitStruct.Pin = GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* USER CODE BEGIN OPAMP2_MspInit 1 */

  /* USER CODE END OPAMP2_MspInit 1 */
  }

}

/**
* @brief OPAMP MSP De-Initialization
* This function freeze the hardware resources used in this example
* @param hopamp: OPAMP handle pointer
* @retval None
*/
void HAL_OPAMP_MspDeInit(OPAMP_HandleTypeDef* hopamp)
{
  if(hopamp->Instance==OPAMP1)
  {
  /* USER CODE BEGIN OPAMP1_MspDeInit 0 */

  /* USER CODE END OPAMP1_MspDeInit 0 */

    /**OPAMP1 GPIO Configuration
    PA1     ------> OPAMP1_VINP
    PA2     ------> OPAMP1_VOUT
    PA3     ------> OPAMP1_VINM0
    */
    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3);

  /* USER CODE BEGIN OPAMP1_MspDeInit 1 */

  /* USER CODE END OPAMP1_MspDeInit 1 */
  }
  else if(hopamp->Instance==OPAMP2)
  {
  /* USER CODE BEGIN OPAMP2_MspDeInit 0 */

  /* USER CODE END OPAMP2_MspDeInit 0 */

    /**OPAMP2 GPIO Configuration
    PA5     ------> OPAMP2_VINM0
    PA6     ------> OPAMP2_VOUT
    PA7     ------> OPAMP2_VINP
    */
    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7);

  /* USER CODE BEGIN OPAMP2_MspDeInit 1 */

  /* USER CODE END OPAMP2_MspDeInit 1 */
  }

}

void HAL_COMP_MspInit(COMP_HandleTypeDef* hcomp)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(hcomp->Instance==COMP1)
  {
  /* USER CODE BEGIN COMP1_MspInit 0 */

  /* USER CODE END COMP1_MspInit 0 */

    __HAL_RCC_GPIOA_CLK_ENABLE();
    /**COMP1 GPIO Configuration
    PA1     ------> COMP1_INP
    PA6     ------> COMP1_OUT
    */
    GPIO_InitStruct.Pin = GPIO_PIN_1;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

//    GPIO_InitStruct.Pin = GPIO_PIN_0;
//    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
//    GPIO_InitStruct.Pull = GPIO_NOPULL;
//    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
//    GPIO_InitStruct.Alternate = GPIO_AF8_COMP1;
//    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* USER CODE BEGIN COMP1_MspInit 1 */

  /* USER CODE END COMP1_MspInit 1 */
  }
	if(hcomp->Instance==COMP2)
  {
	   __HAL_RCC_GPIOA_CLK_ENABLE();
		GPIO_InitStruct.Pin = GPIO_PIN_7;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
	}

}

/**
* @brief COMP MSP De-Initialization
* This function freeze the hardware resources used in this example
* @param hcomp: COMP handle pointer
* @retval None
*/
void HAL_COMP_MspDeInit(COMP_HandleTypeDef* hcomp)
{
  if(hcomp->Instance==COMP1)
  {
  /* USER CODE BEGIN COMP1_MspDeInit 0 */

  /* USER CODE END COMP1_MspDeInit 0 */

    /**COMP1 GPIO Configuration
    PA1     ------> COMP1_INP
    PA6     ------> COMP1_OUT
    */
    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_1|GPIO_PIN_6);

  /* USER CODE BEGIN COMP1_MspDeInit 1 */

  /* USER CODE END COMP1_MspDeInit 1 */
  }
    if(hcomp->Instance==COMP2)
  {
  /* USER CODE BEGIN COMP1_MspDeInit 0 */

  /* USER CODE END COMP1_MspDeInit 0 */

    /**COMP1 GPIO Configuration
    PA1     ------> COMP1_INP
    PA6     ------> COMP1_OUT
    */
    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_7);

  /* USER CODE BEGIN COMP1_MspDeInit 1 */

  /* USER CODE END COMP1_MspDeInit 1 */
  }

}
/* USER CODE BEGIN 1 */
void HAL_FDCAN_MspInit(FDCAN_HandleTypeDef* fdcanHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(fdcanHandle->Instance==FDCAN1)
  {
  /* USER CODE BEGIN FDCAN1_MspInit 0 */

  /* USER CODE END FDCAN1_MspInit 0 */
    /* FDCAN1 clock enable */
    __HAL_RCC_FDCAN_CLK_ENABLE();
  
    __HAL_RCC_GPIOA_CLK_ENABLE();
    /**FDCAN1 GPIO Configuration    
    PA11     ------> FDCAN1_RX
    PA12     ------> FDCAN1_TX 
    */
    GPIO_InitStruct.Pin = GPIO_PIN_11|GPIO_PIN_12;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF9_FDCAN1;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
	  
	GPIO_InitStruct.Pin = GPIO_PIN_8;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP ;
    GPIO_InitStruct.Pull =  GPIO_NOPULL ;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
//    GPIO_InitStruct.Alternate = GPIO_AF9_FDCAN1;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
	HAL_GPIO_WritePin(GPIOA,GPIO_PIN_8,0) ;
		HAL_GPIO_WritePin(GPIOA,GPIO_PIN_8,0) ;
//	HAL_GPIO_WritePin(GPIOA,GPIO_PIN_11,0) ;
	
	

    /* FDCAN1 interrupt Init */
    HAL_NVIC_SetPriority(FDCAN1_IT0_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(FDCAN1_IT0_IRQn);
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
  /* USER CODE BEGIN FDCAN1_MspDeInit 1 */

  /* USER CODE END FDCAN1_MspDeInit 1 */
  }
} 
void HAL_I2C_MspInit(I2C_HandleTypeDef* hi2c)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(hi2c->Instance==I2C3)
  {
  /* USER CODE BEGIN I2C3_MspInit 0 */

  /* USER CODE END I2C3_MspInit 0 */

    __HAL_RCC_GPIOC_CLK_ENABLE();
    /**I2C3 GPIO Configuration
    PC8     ------> I2C3_SCL
    PC9     ------> I2C3_SDA
    */
    GPIO_InitStruct.Pin = GD_SCL_Pin|GD_SDA_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF8_I2C3;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    /* Peripheral clock enable */
    __HAL_RCC_I2C3_CLK_ENABLE();
  /* USER CODE BEGIN I2C3_MspInit 1 */

  /* USER CODE END I2C3_MspInit 1 */
  }

}
/* USER CODE END 1 */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
