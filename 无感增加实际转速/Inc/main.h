/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32g4xx_hal.h"

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

/* USER CODE END EM */

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */
typedef int32_t  s32;
typedef int16_t s16;
typedef int8_t  s8;

typedef uint32_t  u32;
typedef uint16_t u16;
typedef uint8_t  u8;
/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define M1_HALL_H1_Pin GPIO_PIN_0    //HaLLU
#define M1_HALL_H1_GPIO_Port GPIOB
#define M1_HALL_H2_Pin GPIO_PIN_1
#define M1_HALL_H2_GPIO_Port GPIOB  //Hall V
#define M1_HALL_H3_Pin GPIO_PIN_2
#define M1_HALL_H3_GPIO_Port GPIOB  //Hall W

#define M1_PWM_UL_Pin GPIO_PIN_8
#define M1_PWM_UL_GPIO_Port GPIOE  //LAST PE8
#define M1_PWM_UH_Pin GPIO_PIN_9
#define M1_PWM_UH_GPIO_Port GPIOE  //LAST PE9
#define M1_PWM_VL_Pin GPIO_PIN_10
#define M1_PWM_VL_GPIO_Port GPIOE  //LAST PE10
#define M1_PWM_VH_Pin GPIO_PIN_11
#define M1_PWM_VH_GPIO_Port GPIOE  //LAST PE11
#define M1_PWM_WL_Pin GPIO_PIN_12
#define M1_PWM_WL_GPIO_Port GPIOE  //LAST PE12
#define M1_PWM_WH_Pin GPIO_PIN_13
#define M1_PWM_WH_GPIO_Port GPIOE  //LAST PE13

#define M2_HALL_H1_Pin GPIO_PIN_13
#define M2_HALL_H1_GPIO_Port GPIOC
#define M2_HALL_H2_Pin GPIO_PIN_14
#define M2_HALL_H2_GPIO_Port GPIOC
#define M2_HALL_H3_Pin GPIO_PIN_15
#define M2_HALL_H3_GPIO_Port GPIOC

#define M2_PWM_UL_Pin GPIO_PIN_3
#define M2_PWM_UL_GPIO_Port GPIOB   //LAST PC10
#define M2_PWM_UH_Pin GPIO_PIN_6
#define M2_PWM_UH_GPIO_Port GPIOB   //LAST PE13
#define M2_PWM_VL_Pin GPIO_PIN_4
#define M2_PWM_VL_GPIO_Port GPIOB   //LAST PC11
#define M2_PWM_VH_Pin GPIO_PIN_8
#define M2_PWM_VH_GPIO_Port GPIOB  //LAST PC7
#define M2_PWM_WL_Pin GPIO_PIN_5
#define M2_PWM_WL_GPIO_Port GPIOB   //LAST PC12
#define M2_PWM_WH_Pin GPIO_PIN_9
#define M2_PWM_WH_GPIO_Port GPIOB   //LAST PC8

#define UART_TX_Pin GPIO_PIN_9
#define UART_TX_GPIO_Port GPIOA
#define UART_RX_Pin GPIO_PIN_10
#define UART_RX_GPIO_Port GPIOA

#define CAN_TX_Pin GPIO_PIN_12
#define CAN_TX_GPIO_Port GPIOA
#define CAN_RX_Pin GPIO_PIN_11
#define CAN_RX_GPIO_Port GPIOA

#define Fault_1_Pin GPIO_PIN_2   //RED_LED1
#define Fault_1_GPIO_Port GPIOD

#define Version_1_Pin GPIO_PIN_2
#define Version_1_GPIO_Port GPIOE
#define GD_SCL_Pin GPIO_PIN_8
#define GD_SCL_GPIO_Port GPIOC
#define GD_SDA_Pin GPIO_PIN_9
#define GD_SDA_GPIO_Port GPIOC



#define  M1_BREAK		      0X20000000
#define  M2_BREAK             0X40000000
//#define Start_Stop_Pin GPIO_PIN_13
//#define Start_Stop_GPIO_Port GPIOC
//#define Start_Stop_EXTI_IRQn EXTI15_10_IRQn

//#define M1_CURR_AMPL_W_Pin GPIO_PIN_0
//#define M1_CURR_AMPL_W_GPIO_Port GPIOC
//#define M1_BUS_VOLTAGE_Pin GPIO_PIN_1
//#define M1_BUS_VOLTAGE_GPIO_Port GPIOC
//#define M1_CURR_AMPL_U_Pin GPIO_PIN_2
//#define M1_CURR_AMPL_U_GPIO_Port GPIOC
//#define M1_CURR_AMPL_V_Pin GPIO_PIN_3
//#define M1_CURR_AMPL_V_GPIO_Port GPIOC
//#define M1_ENCODER_A_Pin GPIO_PIN_0
//#define M1_ENCODER_A_GPIO_Port GPIOA
//#define DBG_DAC_CH1_Pin GPIO_PIN_4
//#define DBG_DAC_CH1_GPIO_Port GPIOA
//#define DBG_DAC_CH2_Pin GPIO_PIN_5
//#define DBG_DAC_CH2_GPIO_Port GPIOA
//#define M1_TEMPERATURE_Pin GPIO_PIN_4
//#define M1_TEMPERATURE_GPIO_Port GPIOC
//#define M2_TEMPERATURE_Pin GPIO_PIN_7
//#define M2_TEMPERATURE_GPIO_Port GPIOE

//#define M2_BUS_VOLTAGE_Pin GPIO_PIN_14
//#define M2_BUS_VOLTAGE_GPIO_Port GPIOE
//#define M2_CURR_AMPL_U_Pin GPIO_PIN_10
//#define M2_CURR_AMPL_U_GPIO_Port GPIOD
//#define M2_CURR_AMPL_V_Pin GPIO_PIN_12
//#define M2_CURR_AMPL_V_GPIO_Port GPIOD
//#define M2_CURR_AMPL_W_Pin GPIO_PIN_13
//#define M2_CURR_AMPL_W_GPIO_Port GPIOD

//#define M2_OCP_Pin GPIO_PIN_9
//#define M2_OCP_GPIO_Port GPIOC

//#define M1_OCP_Pin GPIO_PIN_11
//#define M1_OCP_GPIO_Port GPIOA
//#define TMS_Pin GPIO_PIN_13
//#define TMS_GPIO_Port GPIOA

//#define TCK_Pin GPIO_PIN_14
//#define TCK_GPIO_Port GPIOA



//#define M1_ENCODER_B_Pin GPIO_PIN_4
//#define M1_ENCODER_B_GPIO_Port GPIOD

//#define M2_ENCODER_A_Pin GPIO_PIN_6
//#define M2_ENCODER_A_GPIO_Port GPIOB
//#define M2_ENCODER_B_Pin GPIO_PIN_7
//#define M2_ENCODER_B_GPIO_Port GPIOB



//#define Normal_Pin GPIO_PIN_15    //GREEN_LED
//#define Normal_GPIO_Port GPIOE
//#define Fault_2_Pin GPIO_PIN_7    //RED_LED2
//#define Fault_2_GPIO_Port GPIOE

/* USER CODE BEGIN Private defines */
#define ParaNum 300
/* 扩展变量 ------------------------------------------------------------------*/

//enum bReadEnum
//{
//	WR = 0,
//	OR = 1,
//};
//enum bSaveToFlashEnum
//{
//	YSFLASH,
//	NOFLASH,
//};
//typedef struct
//{
//  enum			bReadEnum  bReadOnly; 		//读写属性
//	enum			bSaveToFlashEnum bSaveToFlash; 	//是否允许保存Flsash
//}PARAMETER_ATTRIBUTES;
////enum bOnly{RW,RO};
////enum bOnly bReadOnly;
////enum ToFlash{YSFLASH,NOFLASH};
////enum ToFlash bSaveToFlash;
//typedef struct 
//{
//	short								sParID;                                                                                  
//	short								sAddress; 		//通讯参数地址	
//	PARAMETER_ATTRIBUTES		        stAttributes;   //相关属性
////	short								sMaxArrayIndex; //数组索引最大值
////	long								lMinValue; 	    //最小取值 
////	long 								lMaxValue; 		//最大取值范围
////	long								lDefaultValue;  //默认值
//    long*								lpParam; 		//对应变量映射 
//}PARAMETER_TABLE;

//extern const PARAMETER_TABLE PARAMETER[];
typedef enum 
{
	IDLE_t, INIT_t, START_t, RUN_t, STOP_t, BRAKE_t, WAIT_t, FAULT_t,HALL_STUDY_MOTOR5,HALL_STUDY_MOTOR6,Senless_start
} SystStatus_t;
extern SystStatus_t MotorState_t;

#define	BitSet(Var,Mask)	(Var |= Mask)		//??????
#define	BitClr(Var,Mask)	(Var &= (~Mask))	//????3y
#define	BitTst(Var,Mask)	((Var & Mask) != 0 ? 1 : 0)	//??2aê?
#define	BitInv(Var,Mask)	((Var & Mask) != 0 ? (Var &= (~Mask)) : (Var |= Mask))	//??·′×a

#define A1mSec		  0x01
#define A2mSec		  0x02
#define A10mSec		0x04
#define A20mSec		0x08
#define A250mSec	  0x10
#define A500mSec	  0x20
#define A1Sec		  0x40

typedef struct
{
    u8 SysTimFlag;	//system flag
    u8 MicroSec; /* 微秒计数 */
    u8 MilliSec; /* 毫秒计数 */
    u8 SecondCntr;	/*	1秒计数 */
    u8 test;
} SysCLC_TypeDef;
extern SysCLC_TypeDef SysCLC;//system clock variable
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
