/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32g4xx_it.c
  * @brief   Interrupt Service Routines.
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
#include "stm32g4xx_it.h"
#include "bsp_BDCMotor.h"
#include "mc_type.h"
#include "mc_config.h"
#include "mc_tasks.h"
#include "ui_task.h"
#include "parameters_conversion.h"
#include "motorcontrol.h"
#include "stm32g4xx_ll_exti.h"
#include "stm32g4xx_hal.h"
#include "stm32g4xx.h"
#include "state_machine.h"
#include "stdint.h"
#include "function.h"
#include "pid.h"
#include "Sensorless bldc.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
/* USER CODE END Includes */
extern FDCAN_HandleTypeDef hfdcan1;
void FDCAN1_IT0_IRQHandler(void)
{
  /* USER CODE BEGIN FDCAN1_IT0_IRQn 0 */
  /* USER CODE END FDCAN1_IT0_IRQn 0 */
  HAL_FDCAN_IRQHandler(&hfdcan1);
  /* USER CODE BEGIN FDCAN1_IT0_IRQn 1 */

  /* USER CODE END FDCAN1_IT0_IRQn 1 */
}
/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */
void EXTI0_IRQHandler (void)
{
   HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_0);
}
void EXTI1_IRQHandler (void)
{
   HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_1);
}
void EXTI2_IRQHandler (void)
{
   HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_2);
}
void EXTI15_10_IRQHandler(void)
{
    /* USER CODE BEGIN EXTI15_10_IRQn 0 */

    /* USER CODE END EXTI15_10_IRQn 0 */
    HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_13);
    HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_14);
    HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_15);
    /* USER CODE BEGIN EXTI15_10_IRQn 1 */

    /* USER CODE END EXTI15_10_IRQn 1 */
}
void EXTI9_5_IRQHandler(void)
{
    /* USER CODE BEGIN EXTI15_10_IRQn 0 */

    /* USER CODE END EXTI15_10_IRQn 0 */
    HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_5);
    HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_6);
    HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_7);

    /* USER CODE BEGIN EXTI15_10_IRQn 1 */

    /* USER CODE END EXTI15_10_IRQn 1 */
}
/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
//void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
//{
//   if(GPIO_Pin == M1_HALL_H1_Pin)//
//    {
//			TIM8->CCR2 = 800;
//		}
//	 if(GPIO_Pin == M2_HALL_H1_Pin) //
//    {
//			TIM1->CCR3 = 500;
//		}
//}
extern int32_t ADC_ConvertedValue[12];
extern DMA_HandleTypeDef hdma_adc1;
extern ADC_HandleTypeDef hadc1;
extern ADC_HandleTypeDef hadc2;
uint8_t PrevHallState1,PrevHallState2,HallState1,HallState2;
uint8_t HallState1_CW,HallState1_CCW,HallState1_CW,HallState2_CCW;
uint16_t  HALL_CaptureValue,HALL_PreCaptureValue,Speed_Hz;
s16 BLDC1_Speed_RPM = 0;
s8 HallState1_Temp,HallState2_Temp;
uint16_t  HALL_CaptureValue2,HALL_PreCaptureValue2;
s16 BLDC2_Speed_RPM = 0;
u16 Hall1_Slow_CNT,Hall2_Slow_CNT;
s32 HALL_CaptureValueDelta,HALL_CaptureValueDelta2;
u32 HALL_OVF_Counter = 0 ,PreHALL_OVF_Counter=0;
uint8_t Hall_ArrayTab[20],Hall_Tab_CNT=0;


s8 Hall_Dir[2]= {1,-1};        //电机测试架A电机-1，B电机1
void BLDC1_PhaseChange(u8 bHallState,s16 PWM_Duty);
void BLDC2_PhaseChange(u8 bHallState,s16 PWM_Duty);
extern STM_Handle_t STM[NBR_OF_MOTORS];
const s8 Dir_tab[64]=
{
//0
    ERROR,//000|000无变化
    ERROR,//000|001ERR
    ERROR,//000|010ERR
    ERROR,//000|011ERR
    ERROR,//000|100ERR
    ERROR,//000|101ERR  
    ERROR,//000|110ERR
    ERROR,//000|111ERR
//1
    ERROR,//001|000ERR
    MISSTEP,//001|001无变化  
    ERROR,//001|010ERR
    POSITIVE,//001|011正转dir_real=1   
    ERROR,//001|100ERR
    NEGATIVE,//001|101反转dir_real=2   
    ERROR,//001|110ERR
    ERROR,//001|111ERR
//2
    ERROR,//010|000ERR
    ERROR,//010|001ERR
    MISSTEP,//010|010无变化 
    NEGATIVE,//010|011反转dir_real=2   
    ERROR,//010|100ERR
    ERROR,//010|101ERR
    POSITIVE,//010|110正转dir_real=1  
    ERROR,//010|111ERR
//3
    ERROR,//011|000ERR
    NEGATIVE,//011|001反转dir_real=2
    POSITIVE,//011|010正转dir_real=1
    MISSTEP,//011|011无变化
    ERROR,//011|100ERR
    ERROR,//011|101ERR
    ERROR,//011|110ERR
    ERROR,//011|111ERR
//4
    ERROR,//100|000ERR
    ERROR,//100|001ERR
    ERROR,//100|010ERR
    ERROR,//100|011ERR
    MISSTEP,//100|100无变化
    POSITIVE,//100|101正转dir_real=1
    NEGATIVE,//100|110反转dir_real=2
    ERROR,//100|111ERR
//5
    ERROR,//101|000ERR
    POSITIVE,//101|001正转dir_real=1
    ERROR,//101|010ERR
    ERROR,//101|011ERR
    NEGATIVE,//101|100反转dir_real=2
    MISSTEP,//101|101无变化
    ERROR,//101|110ERR
    ERROR,//101|111ERR
//6
    ERROR,//110|000ERR
    ERROR,//110|001ERR
    NEGATIVE,//110|010反转dir_real=2
    ERROR,//110|011ERR
    POSITIVE,//110|100正转dir_real=1
    ERROR,//110|101ERR
    MISSTEP,//110|110无变化
    ERROR,//110|111ERR
//7
    ERROR,//111|000ERR
    ERROR,//111|001ERR
    ERROR,//111|010ERR
    ERROR,//111|011ERR
    ERROR,//111|100ERR
    ERROR,//111|101ERR
    ERROR,//111|110ERR
    ERROR //111|111无变化
};
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
//	HALL1_CC_First++;
    if((GPIO_Pin ==M1_HALL_H1_Pin )||(GPIO_Pin == M1_HALL_H2_Pin )||(GPIO_Pin == M1_HALL_H3_Pin ))
    {
		if(MotorState_t == RUN_t)
		{
		 
        MotorControl[M1].Hall.PrevHallState = MotorControl[M1].Hall.HallState;
        MotorControl[M1].Hall.HallState = HALL_GetPhase1();
        MotorControl[M1].Hall.HallState_Temp = (MotorControl[M1].Hall.PrevHallState<<3) | MotorControl[M1].Hall.HallState ;
        MotorControl[M1].Direction = Hall_Dir[0]*Dir_tab[MotorControl[M1].Hall.HallState_Temp];
        if(MotorState_t == HALL_STUDY_MOTOR5)/*霍尔换向*/
        {
            if((MotorControl[M1].Hall.HallState>0)&&(MotorControl[M1].Hall.HallState<7))
            {
                if(HALL_Study[M1].HallSector == 1)      HALL_Study[M1].HallTab[0] = MotorControl[M1].Hall.HallState;
                else if(HALL_Study[M1].HallSector == 2) HALL_Study[M1].HallTab[1] = MotorControl[M1].Hall.HallState;
                else if(HALL_Study[M1].HallSector == 3) HALL_Study[M1].HallTab[2] = MotorControl[M1].Hall.HallState;
                else if(HALL_Study[M1].HallSector == 4) HALL_Study[M1].HallTab[3] = MotorControl[M1].Hall.HallState;
                else if(HALL_Study[M1].HallSector == 5) HALL_Study[M1].HallTab[4] = MotorControl[M1].Hall.HallState;
                else if(HALL_Study[M1].HallSector == 6) HALL_Study[M1].HallTab[5] = MotorControl[M1].Hall.HallState;
            }
            else
            {
                /*HALL错误*/
                MC_SetFault(M1_HALL_ERR);
            }

            if(MotorControl[M1].Direction < 0)      //上面部分只是拖着正转，但是如果方向是<0，需要反向
            {
                Hall_Dir[0] = -Hall_Dir[0];
            }
        }
	    }
		
		 if(MotorState_t == RUN_t)
		 {
			  if(MotorControl[0].Direction == 1 || MotorControl[0].Direction == -1)
			{
	//			BLDCA_Phase_Check();
				if(MotorControl[0].Hall.HallState == 1)   //一个霍尔周期进行一次测速
				{
					MotorControl[0].Hall.HALL_CaptureValue = TIM16->CNT ;
					MotorControl[0].Hall.HALL_CaptureValueDelta = MotorControl[0].Hall.HALL_CaptureValue - MotorControl[0].Hall.HALL_PreCaptureValue + \
								65536*(HALL_OVF_Counter - MotorControl[0].Hall.PreHALL_OVF_Counter);
					MotorControl[0].Hall.HALL_PreCaptureValue = MotorControl[0].Hall.HALL_CaptureValue;
					MotorControl[0].Hall.PreHALL_OVF_Counter = HALL_OVF_Counter ;
					if(MotorControl[0].Speed_Real > 100 && MotorControl[0].Motor_Start_Stop == 1 && MotorControl[0].Speed_Set != 0)
					{

						MotorA_Default_Phase_Cheek_IT(); //缺相检测

					}
				}
	//			
				if(MotorControl[0].Fault_Flag == 0) // 8-28修改待测试，禁止报错之后继续拧电机轴，进入外部中断，外部中断会换向
				{
					if(MotorControl[0].PWM_Duty < 0)
					{
						MotorControl[0].Hall.HallState_CCW = 0x07 ^ MotorControl[0].Hall.HallState;
						BLDC1_PhaseChange( MotorControl[0].Hall.HallState_CCW,MotorControl[0].PWM_Duty);
					}
					else
					{
						MotorControl[0].Hall.HallState_CW = MotorControl[0].Hall.HallState;
						BLDC1_PhaseChange( MotorControl[0].Hall.HallState_CW,MotorControl[0].PWM_Duty);
					}
				}

			}
	    
				else if(MotorControl[0].Hall.HallState == 0 || MotorControl[0].Hall.HallState == 7)
				{
				  MC_SetFault(M1_HALL_ERR);
				}
		
        else
        {
					 if(MotorControl[M1].Hall.HallState == 4)
            {
//							if(MotorControl[M1].Direction ==1 )
				      {				
                MotorControl[M1].Hall.HALL_CaptureValue = TIM16->CNT ;

                MotorControl[M1].Hall.HALL_CaptureValueDelta = MotorControl[M1].Hall.HALL_CaptureValue - MotorControl[M1].Hall.HALL_PreCaptureValue + 65536*(HALL_OVF_Counter - MotorControl[M1].Hall.PreHALL_OVF_Counter);

                MotorControl[M1].Hall.HALL_PreCaptureValue = MotorControl[M1].Hall.HALL_CaptureValue;
                MotorControl[M1].Hall.PreHALL_OVF_Counter = HALL_OVF_Counter ;
            	}

            }
            if(MotorControl[M1].Speed_Ref < 0)
            {
                MotorControl[M1].Hall.HallState_CCW = 0x07 ^ MotorControl[M1].Hall.HallState;
							  MotorControl[M1].Hall.HallStateValue =MotorControl[M1].Hall.HallState_CCW;
                BLDC1_PhaseChange( MotorControl[M1].Hall.HallStateValue,MotorControl[M1].PWM_Duty);
            }
            else
            {
                MotorControl[M1].Hall.HallState_CW = MotorControl[M1].Hall.HallState;
							  MotorControl[M1].Hall.HallStateValue =MotorControl[M1].Hall.HallState_CW;
                BLDC1_PhaseChange( MotorControl[M1].Hall.HallStateValue,MotorControl[M1].PWM_Duty);
            }
						
        }

    }
		 	 if(MotorState_t == Senless_start)
		 {
			 if(Sensorless[M1].HallState==1)
			 {
				Sensorless[M1].Hall.HALL_CaptureValue = TIM16->CNT ;
                Sensorless[M1].Hall.HALL_CaptureValueDelta = Sensorless[M1].Hall.HALL_CaptureValue - Sensorless[M1].Hall.HALL_PreCaptureValue + 65536*(HALL_OVF_Counter - Sensorless[M1].Hall.PreHALL_OVF_Counter);   //一个周期总共的计数值
                Sensorless[M1].Hall.HALL_PreCaptureValue = Sensorless[M1].Hall.HALL_CaptureValue;
                Sensorless[M1].Hall.PreHALL_OVF_Counter = HALL_OVF_Counter ;	 
			 } 
		 }
		 
}

    if((GPIO_Pin ==M2_HALL_H1_Pin )||(GPIO_Pin == M2_HALL_H2_Pin )||(GPIO_Pin == M2_HALL_H3_Pin ))
    {
        MotorControl[M2].Hall.PrevHallState = MotorControl[M2].Hall.HallState;
        MotorControl[M2].Hall.HallState = HALL_GetPhase2();
			
        Hall_ArrayTab[Hall_Tab_CNT++] = MotorControl[M2].Hall.HallState ;
			  if(Hall_Tab_CNT >17 )Hall_Tab_CNT=0;
			
        MotorControl[M2].Hall.HallState_Temp = (MotorControl[M2].Hall.PrevHallState<<3) | MotorControl[M2].Hall.HallState ;
        MotorControl[M2].Direction = Hall_Dir[1]*Dir_tab[MotorControl[M2].Hall.HallState_Temp];
        /*换向状态换向成功State会自动切换至IDLE*/
        if(MotorState_t == HALL_STUDY_MOTOR6)
        {
            if((MotorControl[M2].Hall.HallState>0)&&(MotorControl[M2].Hall.HallState<7))
            {
                if(HALL_Study[M2].HallSector == 1)      HALL_Study[M2].HallTab[0] = MotorControl[M2].Hall.HallState;
              else if(HALL_Study[M2].HallSector == 2) HALL_Study[M2].HallTab[1] = MotorControl[M2].Hall.HallState;
                else if(HALL_Study[M2].HallSector == 3) HALL_Study[M2].HallTab[2] = MotorControl[M2].Hall.HallState;
                else if(HALL_Study[M2].HallSector == 4) HALL_Study[M2].HallTab[3] = MotorControl[M2].Hall.HallState;
                else if(HALL_Study[M2].HallSector == 5) HALL_Study[M2].HallTab[4] = MotorControl[M2].Hall.HallState;
                else if(HALL_Study[M2].HallSector == 6) HALL_Study[M2].HallTab[5] = MotorControl[M2].Hall.HallState;
            }
            else
            {
                /*HALL错误*/
                MC_SetFault(M2_HALL_ERR);
            }

            if(MotorControl[M2].Direction < 0)
            {
                Hall_Dir[1] = -Hall_Dir[1];
            }
        }
		 else if(MotorControl[1].Direction == 1 || MotorControl[1].Direction == -1)
        {
						BLDCB_Phase_Check();
            if(MotorControl[1].Hall.HallState == 1)
            {
                MotorControl[1].Hall.HALL_CaptureValue = TIM16->CNT ;

                MotorControl[1].Hall.HALL_CaptureValueDelta = MotorControl[1].Hall.HALL_CaptureValue - MotorControl[1].Hall.HALL_PreCaptureValue +65536*(HALL_OVF_Counter - MotorControl[1].Hall.PreHALL_OVF_Counter);

                MotorControl[1].Hall.HALL_PreCaptureValue = MotorControl[1].Hall.HALL_CaptureValue;
                MotorControl[1].Hall.PreHALL_OVF_Counter = HALL_OVF_Counter ;
			if(MotorControl[1].Speed_Real  > 100 && MotorControl[1].Motor_Start_Stop == 1 && MotorControl[1].Speed_Set != 0  )
                {
                    MotorB_Default_Phase_Cheek_IT();
                }
            }
            if(MotorControl[1].Fault_Flag == 0) // 8-28修改待测试，禁止报错之后继续拧电机轴，进入外部中断，外部中断会换向
            {
                if(MotorControl[1].PWM_Duty < 0)
                {
                    MotorControl[1].Hall.HallState_CCW = 0x07 ^ MotorControl[1].Hall.HallState;
                    BLDC2_PhaseChange( MotorControl[1].Hall.HallState_CCW,MotorControl[1].PWM_Duty);
                }
                else
                {
                    MotorControl[1].Hall.HallState_CW = MotorControl[1].Hall.HallState;
                    BLDC2_PhaseChange( MotorControl[1].Hall.HallState_CW,MotorControl[1].PWM_Duty);
                }
            }

        }
        else if(MotorControl[1].Hall.HallState == 0 || MotorControl[1].Hall.HallState == 7)
        {
          MC_SetFault(M2_HALL_ERR);
        }
//        else
//        {

//            if(MotorControl[M2].Hall.HallState == 1)
//            {
//                MotorControl[M2].Hall.HALL_CaptureValue = TIM16->CNT ;

//                MotorControl[M2].Hall.HALL_CaptureValueDelta = MotorControl[M2].Hall.HALL_CaptureValue - MotorControl[M2].Hall.HALL_PreCaptureValue + 65536*(HALL_OVF_Counter - MotorControl[M2].Hall.PreHALL_OVF_Counter);

//                MotorControl[M2].Hall.HALL_PreCaptureValue = MotorControl[M2].Hall.HALL_CaptureValue;
//                MotorControl[M2].Hall.PreHALL_OVF_Counter = HALL_OVF_Counter ;

////                Hall2_Slow_CNT = 0;
//            }

//            if(MotorControl[M2].Speed_Ref < 0 )  //if(MotorControl[M2].PWM_Duty < 0)
//            {  

//							MotorControl[M2].Hall.HallState_CCW = 0x07 ^ MotorControl[M2].Hall.HallState;
//							MotorControl[M2].Hall.HallStateValue =MotorControl[M2].Hall.HallState_CCW;
//              BLDC2_PhaseChange( MotorControl[M2].Hall.HallStateValue,MotorControl[M2].PWM_Duty);
//            }
//            else
//            {
//                MotorControl[M2].Hall.HallState_CW = MotorControl[M2].Hall.HallState;
//							  MotorControl[M2].Hall.HallStateValue =MotorControl[M2].Hall.HallState_CW;
//                BLDC2_PhaseChange( MotorControl[M2].Hall.HallStateValue,MotorControl[M2].PWM_Duty);
//            }

//        }
    }
}
//void DMA1_Channel1_IRQHandler(void)
//{
//    /* USER CODE BEGIN DMA2_Stream0_IRQn 0 */

//    /* USER CODE END DMA2_Stream0_IRQn 0 */
//    HAL_DMA_IRQHandler(&hdma_adc1);
//    /* USER CODE BEGIN DMA2_Stream0_IRQn 1 */

//    /* USER CODE END DMA2_Stream0_IRQn 1 */
//}

void ADC1_2_IRQHandler(void)
{
  /* USER CODE BEGIN ADC1_2_IRQn 0 */

  /* USER CODE END ADC1_2_IRQn 0 */
//	HAL_ADC_Start_DMA(&hadc1, (uint32_t*)&ADC_ConvertedValue,4);
    HAL_ADC_IRQHandler(&hadc1);
	HAL_ADC_IRQHandler(&hadc2);

  /* USER CODE BEGIN ADC1_2_IRQn 1 */

  /* USER CODE END ADC1_2_IRQn 1 */
}

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
int32_t ADC_value1=0;
int32_t ADC_value2=0;
int32_t ADC_value3=0;
uint32_t DMA_Transfer_Complete_Count1=0;
uint32_t DMA_Transfer_Complete_Count2=0;
uint32_t DMA_Transfer_Complete_Count3=0;
__IO uint8_t finish_flag1=0;
__IO uint8_t finish_flag2=0;
__IO uint8_t finish_flag3=0;
extern double Vol_Value1,Vol_Value2,Vol_Value3;
extern PID_Struct_t   PID_Current_InitStructure[2];
int16_t CPWM_Test=4000;
#define COVER_BUFFER_SIZE   ((uint32_t)  1000)  
#define	FILTER_COEFFICIENT		0.005f	//一阶滤波的系数
float Sensorless_COEFFICIENT=1;
s32 UDeepTempValue,VDeepTempValue,WDeepTempValue;
s32 OverZerotime=0.0;
s32 DriveTime=0.0;
extern void Mul_change(u8 Motor_NUM);
void HAL_ADCEx_InjectedConvCpltCallback(ADC_HandleTypeDef* hadc)
{
	s32 TempValue;
	
//	 if(LL_ADC_IsActiveFlag_JEOS(ADC1))
//  {
//    // Clear Flags
//    ADC1->ISR &= ~(uint32_t)(LL_ADC_FLAG_JEOS | LL_ADC_FLAG_JSTRT);
//    
//    TSK_HighFrequencyTask();          /*GUI, this section is present only if DAC is disabled*/
//		GetPhaseCurrentsM1();
//	  ADC_value1= ADC2->JDR2;\\B02A11_H1\../BLDC/bsp_BDCMotor.c\MotorControl[0].Speed_Set
//	  ADC_value2= ADC2->JDR3;
//	  ADC_value3= ADC2->JDR4;
//	  TempValue =((s32)(ADC_value1- Vol_Value1)>>4);
//	  Vol_Value1=TempValue+Vol_Value1;
//	   TempValue =((s32)(ADC_value2- Vol_Value2)>>4);
//	  Vol_Value2=TempValue+Vol_Value2;
//	 TempValue =((s32)(ADC_value3- Vol_Value3)>>4);
//	  Vol_Value3=TempValue+Vol_Value3;
	 if(hadc->Instance==ADC2)
	 {
		 
				 //不滤波
//		 if(MotorControl[0].PWM_Duty<2000)
//		 { 
//		   Sensorless[0].PhaseUCurrent=ADC2->JDR4;
//		   Sensorless[0].PhaseVCurrent=ADC2->JDR2;
//		   Sensorless[0].PhaseWCurrent=ADC2->JDR3;
//		 }
//		 else
//		{
			 //正常滤波
		   Sensorless[0].PhaseUAD=ADC2->JDR4;
		   TempValue = (s32)((Sensorless[0].PhaseUAD - Sensorless[0].PhaseUCurrent )>>3);
		   Sensorless[0].PhaseUCurrent =(u16)(TempValue + Sensorless[0].PhaseUCurrent);
			 
		   Sensorless[0].PhaseVAD=ADC2->JDR2;
		   TempValue = (s32)((Sensorless[0].PhaseVAD - Sensorless[0].PhaseVCurrent )>>3);
		   Sensorless[0].PhaseVCurrent =(u16)(TempValue + Sensorless[0].PhaseVCurrent);
			 
		   Sensorless[0].PhaseWAD=ADC2->JDR3;
		   TempValue = (s32)((Sensorless[0].PhaseWAD - Sensorless[0].PhaseWCurrent )>>3);
		   Sensorless[0].PhaseWCurrent =(u16)(TempValue + Sensorless[0].PhaseWCurrent);
//		   RED_LED1_TOGGLE;
//		   OverZerotime++;
//            RED_LED1_TOGGLE;
//		    BEMF();
//			if(Sensorless[0].State==2)
//			{
//				DriveTime++;
//			}
//		}
		 
//	   HAL_GPIO_TogglePin(GPIOB,GPIO_PIN_10);
		 
		 //深度滤波
//		Sensorless[0].PhaseUAD=ADC2->JDR4;
//		if(Sensorless[0].PhaseUAD< 0 )    Sensorless[0].PhaseUAD = 0;
//	    TempValue = (s32)((Sensorless[0].PhaseUAD - Sensorless[0].PhaseU )>>4);
//	    Sensorless[0].PhaseU =(u16)(TempValue + Sensorless[0].PhaseU);
//		Sensorless[0].PhaseUCurrent = (Sensorless[0].PhaseU*Sensorless_COEFFICIENT+(1-Sensorless_COEFFICIENT)*UDeepTempValue);	
//		UDeepTempValue = Sensorless[0].PhaseUCurrent;
//		
//		 Sensorless[0].PhaseVAD=ADC2->JDR2;
//		 if(Sensorless[0].PhaseVAD< 0 )    Sensorless[0].PhaseVAD = 0;
//		 TempValue = (s32)((Sensorless[0].PhaseVAD - Sensorless[0].PhaseV )>>4);
//	    Sensorless[0].PhaseV =(u16)(TempValue + Sensorless[0].PhaseV);
//		Sensorless[0].PhaseVCurrent = (Sensorless[0].PhaseV*Sensorless_COEFFICIENT+(1-Sensorless_COEFFICIENT)*VDeepTempValue);	
//		VDeepTempValue = Sensorless[0].PhaseVCurrent;
//		 
//		 Sensorless[0].PhaseWAD=ADC2->JDR3;
//		  if(Sensorless[0].PhaseWAD< 0 )    Sensorless[0].PhaseWAD = 0;
//		 TempValue = (s32)((Sensorless[0].PhaseWAD - Sensorless[0].PhaseW )>>4);
//	    Sensorless[0].PhaseW =(u16)(TempValue + Sensorless[0].PhaseW);
//		Sensorless[0].PhaseWCurrent = (Sensorless[0].PhaseW*Sensorless_COEFFICIENT+(1-Sensorless_COEFFICIENT)*WDeepTempValue);	
//		WDeepTempValue = Sensorless[0].PhaseWCurrent;



		 if(MotorState_t == RUN_t)
		 {

		   MotorControl[0].Current.GetADCValue = ADC2->JDR1;
			if(MotorControl[0].Current.GetADCValue< 0 ) MotorControl[0].Current.GetADCValue = 0;
			TempValue = (s32)((MotorControl[0].Current.GetADCValue - MotorControl[0].Current.FilterValue )>>4);
			MotorControl[0].Current.FilterValue = (u16)(TempValue + MotorControl[0].Current.FilterValue);
			 MotorControl[0].Current.DeepFilterVAL = (MotorControl[0].Current.FilterValue*FILTER_COEFFICIENT+(1-FILTER_COEFFICIENT)*MotorControl[0].Current.PreFilterVal);	
			 MotorControl[0].Current.PreFilterVal = MotorControl[0].Current.DeepFilterVAL;
	//		  RED_LED1_TOGGLE;
		   
			
		 }
	 }

	
	
//	    MotorControl[0].Current.GetADCValue = ADC2->JDR1;
//    if(MotorControl[0].Current.GetADCValue< 0 ) MotorControl[0].Current.GetADCValue = 0;
//    TempValue = (s32)((MotorControl[0].Current.GetADCValue - MotorControl[0].Current.FilterValue )>>4);
//    MotorControl[0].Current.FilterValue = (u16)(TempValue + MotorControl[0].Current.FilterValue);
//	
//	    MotorControl[1].Current.GetADCValue = ADC1->JDR1;
//    if(MotorControl[1].Current.GetADCValue< 0 ) MotorControl[1].Current.GetADCValue = 0;
//    TempValue = (s32)((MotorControl[1].Current.GetADCValue - MotorControl[1].Current.FilterValue )>>4);
//    MotorControl[1].Current.FilterValue = (u16)(TempValue + MotorControl[1].Current.FilterValue);
//	 	/*深度滤波*/
//	MotorControl[0].Current.DeepFilterVAL = (MotorControl[0].Current.FilterValue*FILTER_COEFFICIENT+(1-FILTER_COEFFICIENT)*MotorControl[0].Current.PreFilterVal);		
//	MotorControl[1].Current.DeepFilterVAL = (MotorControl[1].Current.FilterValue*FILTER_COEFFICIENT+(1-FILTER_COEFFICIENT)*MotorControl[1].Current.PreFilterVal);
//	MotorControl[0].Current.PreFilterVal = MotorControl[0].Current.DeepFilterVAL;
//	MotorControl[1].Current.PreFilterVal = MotorControl[1].Current.DeepFilterVAL;
	 if(hadc->Instance==ADC1)
  {
	
//	//母线电压
	VoltVar.AdBuf = ADC1->JDR2;
    TempValue = (s32)((VoltVar.AdBuf - VoltVar.BUS )>>4);
    VoltVar.BUS = (u16)(TempValue +VoltVar.BUS);
  }
		if(MotorState_t == RUN_t)
	{
//			MotorControl[5].PWM_Duty = PID_Regulator(MotorControl[5].Current.ADCValue_Ref,MotorControl[5].Current.FilterValue,&PID_Speed_InitStruct[0]);
/***********************5号无刷下发速度*****************************/
		if((MotorControl[0].Speed_Ref < 0 && MotorControl[0].PWM_Duty > 0) || \
			(MotorControl[0].Speed_Ref >0 && MotorControl[0].PWM_Duty < 0) || MotorControl[0].Speed_Ref == 0||MotorControl[0].Fault_Flag == 1 )
		{
			PID_Speed_InitStruct[0].wIntegral=0;
			PID_Current_InitStructure[0].wIntegral=0;
				MotorControl[0].PWM_Duty = 0;
		}
		else 
		{
//			MotorControl[0].PWM_Duty = PID_Regulator(MotorControl[0].Current.ADCValue_Ref,MotorControl[0].Current.DeepFilterVAL,&PID_Current_InitStructure[0]);
//            TIM1->CCR4=MotorControl[0].PWM_Duty-2000;
			MotorControl[0].PWM_Duty=CPWM_Test;
//			  Mul_change(0);
		}
		if(MotorControl[0].Motor_Start_Stop==ENABLE)
		{
			if(MotorControl[0].Fault_Flag == 0)
//				if(MotorControl[1].Fault_Flag == 0&&*PARAMETER[207].lpParam<BREAK_CNT_MAX)
			{
			    SetMotorSpeed(0, MotorControl[0].PWM_Duty);
			}
//				if(MotorControl[0].Fault_Flag == 0&&*PARAMETER[206].lpParam<BREAK_CNT_MAX)
//				else if(*PARAMETER[206].lpParam>=BREAK_CNT_MAX)
//				{
//						MotorControl[5].Motor_Start_Stop = DISABLE;
//						MC_SetFault(MOTOR5_BREAK);
//				}
//				else MotorControl[5].Motor_Start_Stop = DISABLE;
		}
		else if(MotorControl[0].Motor_Start_Stop==DISABLE)
			SetMotorStop(0);	
   }
///***********************5号无刷下发速度end*****************************/

///***********************6号无刷下发速度*****************************/
//		if((MotorControl[1].Speed_Ref < 0 && MotorControl[1].PWM_Duty > 0) || \
//			(MotorControl[1].Speed_Ref >0 && MotorControl[1].PWM_Duty < 0) || MotorControl[1].Speed_Ref == 0||MotorControl[1].Fault_Flag == 1 )
//		{
//			    PID_Speed_InitStruct[1].wIntegral=0;
//			    PID_Current_InitStructure[1].wIntegral=0;
//				MotorControl[1].PWM_Duty = 0;
//			    
//		}
//		else 
//		{
////			if(MotorControl[6].Speed_Real<0) MotorControl[6].Current.DeepFilterVAL = -MotorControl[6].Current.DeepFilterVAL;
//			MotorControl[1].PWM_Duty = PID_Regulator(MotorControl[1].Current.ADCValue_Ref,MotorControl[1].Current.DeepFilterVAL,&PID_Current_InitStructure[1]);
//		}
//		if(MotorControl[1].Motor_Start_Stop==ENABLE)
//		{
//			if(MotorControl[1].Fault_Flag == 0)
////				if(MotorControl[1].Fault_Flag == 0&&*PARAMETER[207].lpParam<BREAK_CNT_MAX)
//			{
//			    SetMotorSpeed(1, MotorControl[1].PWM_Duty);
//			}
////				else if(*PARAMETER[207].lpParam>=BREAK_CNT_MAX)
////				{
////						MotorControl[6].Motor_Start_Stop = DISABLE;
////						MC_SetFault(MOTOR5_BREAK);
////				}
////				else MotorControl[6].Motor_Start_Stop = DISABLE;
//		}
//		else if(MotorControl[1].Motor_Start_Stop==DISABLE)
//			SetMotorStop(1);							
//	}
}
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/

/* USER CODE BEGIN EV */

/* USER CODE END EV */

/******************************************************************************/
/*           Cortex-M4 Processor Interruption and Exception Handlers          */
/******************************************************************************/

/******************************************************************************/
/* STM32G4xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32g4xx.s).                    */
/******************************************************************************/

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
