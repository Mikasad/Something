/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
#include "motorcontrol.h"
#include "stdio.h"
#include "string.h"
#include "bsp_BDCMotor.h"
#include "stspin32g4.h"
#include "Agreement.h"
#include "flash.h"
#include "pid.h"
#include "function.h"
#include "canopen_od.h"
#include "Sensorless bldc.h"
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
ADC_HandleTypeDef hadc1;
ADC_HandleTypeDef hadc2;
ADC_HandleTypeDef hadc3;
ADC_HandleTypeDef hadc4;

CORDIC_HandleTypeDef hcordic;

DAC_HandleTypeDef hdac1;
DAC_HandleTypeDef hdac3;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim5;
TIM_HandleTypeDef htim6;
TIM_HandleTypeDef htim8;
TIM_HandleTypeDef htim15;
TIM_HandleTypeDef htim16;
extern UART_HandleTypeDef huart1;
OPAMP_HandleTypeDef hopamp1;
OPAMP_HandleTypeDef hopamp2;
COMP_HandleTypeDef hcomp1;
COMP_HandleTypeDef hcomp2;
FDCAN_HandleTypeDef hfdcan1;
IWDG_HandleTypeDef hiwdg;
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_ADC2_Init(void);
static void MX_ADC3_Init(void);
static void MX_ADC4_Init(void);
static void MX_CORDIC_Init(void);
static void MX_DAC1_Init(void);
static void MX_DAC3_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM4_Init(void);
static void MX_TIM5_Init(void);
static void MX_TIM8_Init(void);
static void MX_TIM15_Init(void);
extern void MX_USART1_UART_Init(void);
static void MX_NVIC_Init(void);
static void MX_OPAMP1_Init(void);
static void MX_OPAMP2_Init(void);
static void MX_COMP1_Init(void);
static void MX_COMP2_Init(void);
static void MX_TIM16_Init(void);
static void MX_TIM6_Init(void);
static uint32_t GetPage(uint32_t Addr);
static void MX_IWDG_Init(uint8_t prv ,uint16_t rlv);
static void MX_FDCAN1_Init(void);
void MX_DMA_Init(void) ;
static void FDCAN_Config(void);
u8 First_Enter_STUDY_MOTOR5 = 1;
u8 First_Enter_STUDY_MOTOR6 = 1;
void MX_I2C3_Init();
void Sys_Time(void);
/* USER CODE BEGIN PFP */
int32_t test[500]= {0};
uint8_t ubKeyNumber = 0x0;
CanRxMsgTypeDef RxHeader;
CanTxMsgTypeDef TxHeader;
uint8_t RxData[8];
uint8_t TxData[8];
u8 Sensorlessflag;
STSPIN32G4_HandleTypeDef HdlSTSPING4;
extern int32_t transbuf[ParaNum];
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
extern int32_t ADC_value1;
extern int32_t ADC_value2;
extern uint32_t DMA_Transfer_Complete_Count1;
extern uint32_t DMA_Transfer_Complete_Count2;
extern  uint8_t finish_flag1;
extern  uint8_t finish_flag2;
extern u32 HALL_OVF_Counter;
extern STM_Handle_t STM[NBR_OF_MOTORS];
extern s32 DriveTime;
uint16_t Stuck_TimeCnt[MOTOR_NUM] = {0,0};

#define COVER_BUFFER_SIZE   ((uint32_t)  1000)  
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
uint32_t ADC_ConvertedValue[12];
double xxyy;
double AD_Value,Vol_Value1,Vol_Value2,Vol_Value3,Vol_Value4;

__IO uint32_t data32 = 0;
SystStatus_t MotorState_t;
SysCLC_TypeDef SysCLC;
u32 WhileTest;
s32 Sensorlesstime=400;


 void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
 {
	   if (HAL_FDCAN_GetRxFifoFillLevel(&hfdcan1, FDCAN_RX_FIFO0) != 0)
	      {			
            HAL_FDCAN_GetRxMessage(&hfdcan1, FDCAN_RX_FIFO0, &RxHeader, RxHeader.Data);
					
			 if(CANopen_Drive.CanRx_Buffer->IDE == BIT_INVALID)
        {
            CanLoadRate.FrameSize = CanLoadRate.FrameSize + (47 + CANopen_Drive.CanRx_Buffer->DLC * 8);
            MessageType_Check(&CANopen_Drive,CANopen_Drive.CanRx_Buffer->StdId,CANopen_Drive.CanRx_Buffer->RTR);
        }  
		  }
 }
uint8_t CAN2_Send_Msg(uint32_t stdId,uint32_t Ide,uint32_t Rtr,uint8_t* msg,uint8_t len)
{
    uint16_t i=0;
    TxHeader.StdId=stdId;        //?????0001_0010 ??????????????????
    TxHeader.ExtId=0xFFF;        //?????(29?)
    TxHeader.IDE=Ide;    //?????(????)
    TxHeader.RTR=Rtr;  //???(????)
    TxHeader.DLC=len;           //????
//    TxHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;      /* 设置错误状态指示 */
//    TxHeader.BitRateSwitch = FDCAN_BRS_OFF;                  /* 开启可变波特率 */
//    TxHeader.FDFormat = FDCAN_CLASSIC_CAN;                       /* FDCAN格式 */
//    TxHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS;        /* 用于发送事件FIFO控制, 不存储 */
//    TxHeader.MessageMarker = 0;           
	
    for(i=0; i<len; i++)
    {
        TxHeader.Data[i]=msg[i];
    }
	HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &TxHeader, TxHeader.Data);  
//    for(i=0; HAL_CAN_Transmit(&hcan2,10)!=HAL_OK && i<3; i++) {}
    if(i>=3)
    {
    }
    CanLoadRate.FrameSize = CanLoadRate.FrameSize + (47 + TxHeader.DLC * 8);
    return i != 3? 0:1;
}
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
   
   MX_I2C3_Init();
   STSPIN32G4_init( &HdlSTSPING4 );
   STSPIN32G4_reset( &HdlSTSPING4 );
   STSPIN32G4_setVCC( &HdlSTSPING4, (STSPIN32G4_confVCC){ .voltage = _12V,
                                                          .useNFAULT = true,
                                                          .useREADY = false } );
    STSPIN32G4_clearFaults( &HdlSTSPING4 );
    MX_ADC1_Init();
    MX_ADC2_Init();
//    MX_DAC1_Init();
//	MX_DAC3_Init();													  
    MX_TIM1_Init();  
    MX_TIM4_Init();  														  
	MX_TIM15_Init();
	MX_TIM16_Init();
	MX_TIM6_Init();
    MX_TIM8_Init();
    MX_USART1_UART_Init();
	__HAL_USART_ENABLE_IT(&huart1,USART_IT_RXNE);    //开启串口中断处理函数
	Init_Drive_Para();
    HAL_TIM_Base_Start_IT(&htim16);
	HAL_TIM_Base_Start_IT(&htim1);
	HAL_TIM_Base_Start_IT(&htim8);
    HAL_TIM_Base_Start_IT(&htim6);

//    HAL_TIM_Base_Start_IT(&htim15);
//	MX_OPAMP1_Init();
//    MX_OPAMP2_Init();
//	MX_COMP1_Init();
//	MX_COMP2_Init();
//	HAL_OPAMP_Start(&hopamp1);
//  	HAL_OPAMP_Start(&hopamp2);
    HAL_ADCEx_InjectedStart_IT(&hadc1);  
	HAL_ADCEx_InjectedStart_IT(&hadc2);  
//    HAL_DAC_SetValue(&hdac1,DAC_CHANNEL_1,DAC_ALIGN_12B_R,80);//B电机 1A-----4.2  3A----20    5.5A---30   8A-----40    50---10A     60--12.8A     70-----16A       80-----19A       75---17.3A
//    HAL_DAC_SetValue(&hdac1,DAC_CHANNEL_2,DAC_ALIGN_12B_R,80);//A电机 1A-----5    6A-----30   4A----20    8A-----40    80---18A     50----10A     60-----12.5      70-----15.5
//	HAL_DAC_Start(&hdac1, DAC_CHANNEL_1);
//	HAL_DAC_Start(&hdac1, DAC_CHANNEL_2);
//	HAL_COMP_Start(&hcomp2);
//	HAL_COMP_Start(&hcomp1);
    MX_IWDG_Init(IWDG_PRESCALER_64,1250);  //5S
	 __HAL_IWDG_START(&hiwdg);	/* 启动独立看门狗 */
	MX_FDCAN1_Init();
	FDCAN_Config();
  /* USER CODE BEGIN 2 */
   
    /* PWM generation Error */
	/*已经屏蔽的代码*/
    HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_2);   /*测试PWM输出用，正式版请屏蔽*/
    HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_3);  /*测试PWM输出用，正式版请屏蔽*/
	HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);   /*测试PWM输出用，正式版请屏蔽*/
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);  /*测试PWM输出用，正式版请屏蔽*/
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);
	HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_4);
//    HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_3);
//  MX_MotorControl_Init();
    Flash_Read_Init(transbuf,ParaNum);	//read flash data when Power-on

  /* Initialize interrupts */
   
	MX_NVIC_Init();
	HAL_TIM_Base_Start_IT(&htim4);
  /* USER CODE BEGIN 2 */
    CANopen_Drive.CanRx_Buffer =  &RxHeader;
    CANopen_Drive.CanTx_Buffer = &TxHeader;
    CANopen_Parameter_Init(&CANopen_Drive);
	
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
//	  TIM1->CCR4=8000;
	  WhileTest++;
       if(BitTst(SysCLC.SysTimFlag,A1mSec))		/*1MS	*/
        {
            BitClr(SysCLC.SysTimFlag,A1mSec);
            BitInv(SysCLC.SysTimFlag,A2mSec);
            Sys_Time();
		    QuickRead();
//			 BEMF();
	
		
					

            if(BitTst(SysCLC.SysTimFlag,A2mSec))	/*2MS	*/
            {
//                Labview_uart();
		
			 
            }
        }
        if(BitTst(SysCLC.SysTimFlag,A10mSec))		/*10MS	*/
        {
            BitClr(SysCLC.SysTimFlag,A10mSec);
            BitInv(SysCLC.SysTimFlag,A20mSec);
//            DisplayErrLed();
					  ReturnError1();
					  Startagreement();
			    
		      	    	
            if(BitTst(SysCLC.SysTimFlag,A20mSec))	/*20MS	*/
            {
                
            }
        }
		 if(BitTst(SysCLC.SysTimFlag,A250mSec))	/*250MS	*/
        {
            BitClr(SysCLC.SysTimFlag,A250mSec);
            BitInv(SysCLC.SysTimFlag,A500mSec);
            if(MotorControl[0].Hall.HallState == 0||MotorControl[0].Hall.HallState == 7 )
            {
//                MC_SetFault(M1_HALL_ERR);
//				MotorControl[0].Motor_Start_Stop = DISABLE;
            }
            if(MotorControl[1].Hall.HallState == 0||MotorControl[1].Hall.HallState == 7 )
            {
                MC_SetFault(M2_HALL_ERR);
				MotorControl[1].Motor_Start_Stop = DISABLE;
            }
						Flash_WriteCheck();	
        }
		
    /* USER CODE END WHILE */
    /* USER CODE BEGIN 3 */
	 if(HAL_GetTick()%20 == 0)  //10ms
	 {
////////			DisplayErrLed();          
//			/*CAN TEST Program*/
//			TxData[0] = ubKeyNumber;
//            TxData[1] = 0xAD;
////		    
//            HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &TxHeader, TxData);  
//            HAL_Delay(100);	
//           
//		   /*CAN TEST Program*/
	  }
  }

}


void HAL_SYSTICK_Callback(void)
{ 
    if(HAL_GetTick()%1 == 0)  //1MS
    {
		BLDC_Stuck_Chk();	//堵转保护500ms报警
	    GetMotorSpeed(M1);
        GetMotorSpeed(M2);
        BitSet(SysCLC.SysTimFlag,A1mSec);
    }
    if(HAL_GetTick()%1== 0)  //2MS
    {
		
//		HAL_GPIO_TogglePin(GPIOD,GPIO_PIN_2);
//		RED_LED1_ON;
        switch (MotorState_t)
        {
        case IDLE_t:
            MotorState_t= INIT_t;		//偏置电压修正后进入INIT
            break;

        case INIT_t:
                MotorState_t = START_t; //标定成功再进入下一个状态，再使能CAN通信
            HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);
              HAL_NVIC_EnableIRQ(EXTI0_IRQn);
		     HAL_NVIC_EnableIRQ(EXTI1_IRQn);
		    HAL_NVIC_EnableIRQ(EXTI2_IRQn);
            break;

        case START_t:
            MotorState_t = RUN_t;
            break;

        case RUN_t:   // motor running
            for(u8 i=0; i < MOTOR_NUM; i++)
            {
                if(MotorControl[i].Motor_Start_Stop == ENABLE)			/* 硬件过流次数不能超限，否则不能使能 */
                {
						switch (i)
									{
										case 0:
											Ramp_Speed(0);
											MotorControl[0].Speed_Real = GetMotorSpeed(0);
											BLDC1_OverSpdChk();
											MotorControl[0].Current.ADCValue_Ref = PID_Regulator(MotorControl[0].Speed_Ref,MotorControl[0].Speed_Real,&PID_Speed_InitStruct[0]);
											break;
										case 1:
											Ramp_Speed(1);
											MotorControl[1].Speed_Real = GetMotorSpeed(1);
											BLDC2_OverSpdChk();
											MotorControl[1].Current.ADCValue_Ref = PID_Regulator(MotorControl[1].Speed_Ref,MotorControl[1].Speed_Real,&PID_Speed_InitStruct[1]);
											break;
										default:
											if(MotorControl[i].Fault_Flag == 0)
											{
													SetMotorSpeed(i, MotorControl[i].PWM_Duty);
											}
											else
											{
													MotorControl[i].Motor_Start_Stop = DISABLE;
											}
											break;
//					 if(MotorControl[i].Fault_Flag == 0)
//                    {
//						BLDC1_OverSpdChk();
//		                BLDC2_OverSpdChk();
//                        SetMotorSpeed(i, MotorControl[i].PWM_Duty);
//                    }
//                    else
//                    {
//                        MotorControl[i].Motor_Start_Stop = DISABLE;
//                    }
				}
			}									
                else if(MotorControl[i].Motor_Start_Stop == DISABLE)
                {
                    SetMotorStop(i);
                }
            }
		
		
            break;

        case STOP_t:    // motor stopped

            MotorState_t = WAIT_t;

            break;

        case WAIT_t:    // wait MotorState

            break;

        case FAULT_t:

            if(wGlobal_Flags == 0)
            {
                MotorState_t = IDLE_t;
            }

            break;

        case HALL_STUDY_MOTOR5:

            if(First_Enter_STUDY_MOTOR5 == 1)
            {
                TIM1->CCR1 = 0;
                TIM1->CCR2 = 0;
                TIM1->CCR3 = 0;
                for(u16 i = 0; i < 10000; i++) {}
                First_Enter_STUDY_MOTOR5 = 0;
            }
            HallStudyHandle0();

            if(HALL_Study[0].CommuntionState == 3 )
            {
                HALL_Study[0].CommuntionState = 0;
                HALL_Study[0].StudySectorCnt = 0;
                TIM1->CCR1 = 0;
                TIM1->CCR2 = 0;
                TIM1->CCR3 = 0;
                if ( (wGlobal_Flags & M1_HALL_ERR) == 0 )
                {
                    MotorState_t = IDLE_t;
                }
                else
                {
                    MotorState_t = FAULT_t;
                }
                First_Enter_STUDY_MOTOR5 = 1;
            }
            break;
        case HALL_STUDY_MOTOR6:
            if(First_Enter_STUDY_MOTOR6 == 1)
            {
                TIM8->CCR1 = 0;
                TIM8->CCR2 = 0;
                TIM8->CCR3 = 0;
                for(u16 i = 0; i < 10000; i++) {}
                First_Enter_STUDY_MOTOR6 = 0;
            }
            HallStudyHandle1();
            if(HALL_Study[1].CommuntionState == 3)
            {
                HALL_Study[1].CommuntionState = 0;
                HALL_Study[1].StudySectorCnt = 0;
                TIM8->CCR1 = 0;
                TIM8->CCR2 = 0;
                TIM8->CCR3 = 0;
                if ((wGlobal_Flags & M2_HALL_ERR) == 0 )
                {
                    MotorState_t = IDLE_t;
                }
                else
                {
                    MotorState_t = FAULT_t;
                }
                First_Enter_STUDY_MOTOR6 = 1;
            }
            break;
		case Senless_start:
//			TIM1->CCER = 0x1444;
//			TIM1->CCR1 = 8000;
//            TIM1->CCR2 = 8000;
//            TIM1->CCR3 = 8000;
//            for(u16 i = 0; i < 10000; i++) {}
//			Sensorless_Start();
		break;

        default:
            break;
        }
    }
 
    if(HAL_GetTick()%10 == 0) //10ms
    {
        Over_VoltageCheck(); //过压保护
		
				
    }
    if(HAL_GetTick()%500 == 0) //500ms喂狗
    {
        HAL_IWDG_Refresh(&hiwdg);
	
    }   
}

void Sys_Time(void)
{
    SysCLC.MilliSec++;
    if((SysCLC.MilliSec%10)==0)//10MS
    {
        BitSet(SysCLC.SysTimFlag,A10mSec);
    }

    if(SysCLC.MilliSec >= 250)		//250MS
    {
        SysCLC.MilliSec = 0;
        BitSet(SysCLC.SysTimFlag,A250mSec);
        SysCLC.SecondCntr++;		//1????êy?÷
    }

    if(SysCLC.SecondCntr >= 4)		//1000MS = 1S
    {
        SysCLC.SecondCntr = 0;
    }
}
  /* USER CODE END 3 */


/**
  * @brief System Clock Configuration
  * @retval None
  */
/*这是正式版的系统时钟配置*/
void SystemClock_Config(void)          //串口的使用
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Configure the main internal regulator output voltage 
  */
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST);
 
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSI|RCC_OSCILLATORTYPE_HSE; //RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;//RCC_OscInitStruct.HSIState =RCC_HSI_ON;                   // RCC_HSI_ON;
   RCC_OscInitStruct.LSEState = RCC_LSI_ON;
//  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	 RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV5;
  RCC_OscInitStruct.PLL.PLLN = 68;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
//  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
//  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV4;
//  RCC_OscInitStruct.PLL.PLLN = 85;
//  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
//  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
//  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;                         //170Mhz
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;                           //170Mhz
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;                           //170Mhz

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_8) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the peripherals clocks 
  */
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1| RCC_PERIPHCLK_ADC12|RCC_PERIPHCLK_FDCAN;
  PeriphClkInit.Usart1ClockSelection =RCC_USART1CLKSOURCE_HSI ;
  PeriphClkInit.Adc12ClockSelection = RCC_ADC12CLKSOURCE_SYSCLK;
  PeriphClkInit.FdcanClockSelection = RCC_FDCANCLKSOURCE_PCLK1;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
  /*新增的代码*/
       HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

    /* ?μí3μ?′e?¨ê±?÷?D??ó??è?????? */
      HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/20000);                 // 2000对应1ms中断一次，20000对应0.1ms.200000对应0.01ms
    /* ?μí3μ?′e?¨ê±?÷ê±?ó?′ */
    HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

    /* ?μí3μ?′e?¨ê±?÷?D??ó??è?????? */
    HAL_NVIC_SetPriority(SysTick_IRQn, 2, 0);
}


/**
  * @brief NVIC Configuration.
  * @retval None
  */
static void MX_NVIC_Init(void)
{
//  /* TIM1_BRK_TIM15_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(TIM1_BRK_TIM15_IRQn, 0, 1);
  HAL_NVIC_EnableIRQ(TIM1_BRK_TIM15_IRQn);
  /* TIM1_UP_TIM16_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(TIM8_BRK_IRQn, 0, 2);
  HAL_NVIC_EnableIRQ(TIM8_BRK_IRQn);
  HAL_NVIC_SetPriority(TIM1_UP_TIM16_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(TIM1_UP_TIM16_IRQn);
  HAL_NVIC_SetPriority(TIM6_DAC_IRQn, 3, 0);
  HAL_NVIC_EnableIRQ(TIM6_DAC_IRQn);
  HAL_NVIC_SetPriority(TIM4_IRQn,1, 1);
  HAL_NVIC_EnableIRQ(TIM4_IRQn);
//  /* ADC1_2_IRQn interrupt configuration */
//  HAL_NVIC_SetPriority(ADC1_2_IRQn, 2, 0);
//  HAL_NVIC_EnableIRQ(ADC1_2_IRQn);
//  /* TIM8_BRK_IRQn interrupt configuration */
//  HAL_NVIC_SetPriority(TIM8_BRK_IRQn, 4, 1);
//  HAL_NVIC_EnableIRQ(TIM8_BRK_IRQn);
//  /* TIM8_UP_IRQn interrupt configuration */
//  HAL_NVIC_SetPriority(TIM8_UP_IRQn, 0, 0);
//  HAL_NVIC_EnableIRQ(TIM8_UP_IRQn);
//  /* ADC3_IRQn interrupt configuration */
//  HAL_NVIC_SetPriority(ADC3_IRQn, 2, 0);
//  HAL_NVIC_EnableIRQ(ADC3_IRQn);
//  /* TIM2_IRQn interrupt configuration */
//  HAL_NVIC_SetPriority(TIM2_IRQn, 3, 0);
//  HAL_NVIC_EnableIRQ(TIM2_IRQn);
//  /* TIM3_IRQn interrupt configuration */
//  HAL_NVIC_SetPriority(TIM3_IRQn, 3, 0);
//  HAL_NVIC_EnableIRQ(TIM3_IRQn);
//  /* TIM4_IRQn interrupt configuration */
//  HAL_NVIC_SetPriority(TIM4_IRQn, 3, 0);
//  HAL_NVIC_EnableIRQ(TIM4_IRQn);
//  /* TIM5_IRQn interrupt configuration */
//  HAL_NVIC_SetPriority(TIM17_IRQn, 3, 0);
//  HAL_NVIC_EnableIRQ(TIM17_IRQn);
  /* USART1_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(EXTI0_IRQn, 3, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);
      HAL_NVIC_EnableIRQ(FDCAN1_IT0_IRQn);
  HAL_NVIC_SetPriority(FDCAN1_IT0_IRQn, 2, 1);
	HAL_NVIC_SetPriority(EXTI1_IRQn, 3, 0);
  HAL_NVIC_EnableIRQ(EXTI1_IRQn);
	HAL_NVIC_SetPriority(EXTI2_IRQn, 3, 0);
  HAL_NVIC_EnableIRQ(EXTI2_IRQn);
  HAL_NVIC_SetPriority(USART1_IRQn, 2, 1);
  HAL_NVIC_EnableIRQ(USART1_IRQn);
  /* EXTI15_10_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 3, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);
	HAL_NVIC_SetPriority(EXTI9_5_IRQn, 3, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);
}

u32 Cnt_100us=0;                   //100us计数
extern s32 OverZerotime;
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if(htim==(&htim1))         //100us
    {
//		RED_LED1_TOGGLE;
//		BEMF();
//		   OverZerotime++;
//		if(Sensorless[0].State==2)
//		{
//			DriveTime++;
//		}
//		Sensorcnt++;
//		Sensorlesstime++;
//			if(Sensorless[M1].CountSectorCnt >= 3000)
//		         {
//					if(++Sensorcnt >=(Sensorless[M1].period) )
//				    {
//						
//						Sensorcnt=0;
//						change_temp++;
//						 if(change_temp>6)
//						 {
//							 change_temp=1;
//						 }		
//						SensorlessBLDC1_PhaseChange(change_temp,6500);
//						  
//					}
//				}
//       		for(u8 i=0; i< MOTOR_NUM ; i++)
//		{
//				CurrentLimit(i);
//			    if(MotorControl[i].Current.FilterValue > (float)(MotorControl[i].Current.MaxValue1*0.8)  && l_abs(MotorControl[i].Speed_Real) < 50)
//            {
//                Stuck_TimeCnt[i]++;
//                if(Stuck_TimeCnt[i] > 100000)
//                {
//                    SetMotorStop(i);
//                    MotorControl[i].Fault_Flag = 1;
//                    Stuck_TimeCnt[i] = 0;
//                }
//            }
//            else
//            {
//                if(Stuck_TimeCnt[i]>0)
//                {
//                    Stuck_TimeCnt[i]--;
//                }
//            }
//			
////			while(Sensorlessflag==1)
////			{
////				Sensorless_Start();
////				
////			}
//		}
//		MotorA_Default_Phase_Cheek();//缺相检测
        MotorB_Default_Phase_Cheek();//缺相检测
        Cnt_100us++;
        if(Cnt_100us>4000000000)Cnt_100us=0;

    }
	 if(htim==(&htim16))        //66ms
    {
        HALL_OVF_Counter++;
		
//		OverZerotime++;
    }
//	 if(htim==(&htim15))   //定时器15没开中断
//    {
//		RED_LED1_TOGGLE;
//    }
		 if(htim==(&htim4))              //无感代码相关驱动  10us
    {
		
		if(MotorState_t==Senless_start)
		{
			  Sensorless_Start();
		}
//		BEMF();
       	OverZerotime++;
		if(Sensorless[0].State==2)
		{
//			DriveTime++;
		}
		
    }
	
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
extern DMA_HandleTypeDef hdma_adc1;
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */
  /** Common config
  */

  /* USER CODE BEGIN ADC1_Init 2 */

//	ADC_MultiModeTypeDef multimode = {0};
//  ADC_ChannelConfTypeDef sConfig = {0};

//  /** Common config 
//  */		
	ADC_MultiModeTypeDef multimode = {0};
  ADC_InjectionConfTypeDef sConfigInjected = {0};
	ADC_ChannelConfTypeDef sConfig = {0};
  /** Common config 
  */
  __HAL_RCC_ADC12_CLK_ENABLE();
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;   //ADC分辨率
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.GainCompensation = 0;
  hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE; //扫描模式，多通道采集需要
  hadc1.Init.EOCSelection =DISABLE;        //转换完成标记
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE; //连续转换
  hadc1.Init.NbrOfConversion = 1;     //转换的通道
  hadc1.Init.DiscontinuousConvMode = DISABLE;  //非连续转换个数
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc1.Init.OversamplingMode = DISABLE;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure the ADC multi-mode 
  */
  multimode.Mode = ADC_MODE_INDEPENDENT ;
//  if (HAL_ADCEx_MultiModeConfigChannel(&hadc1, &multimode) != HAL_OK)
//  {
//    Error_Handler();
//  }
  /** Configure Regular Channel 
  */
	sConfigInjected.InjectedChannel = ADC_CHANNEL_VOPAMP1;             //B电机电流检测
  sConfigInjected.InjectedRank = ADC_INJECTED_RANK_1;
  sConfigInjected.InjectedSamplingTime = ADC_SAMPLETIME_6CYCLES_5;
  sConfigInjected.InjectedSingleDiff = ADC_SINGLE_ENDED;
  sConfigInjected.InjectedOffsetNumber = ADC_OFFSET_NONE;
  sConfigInjected.InjectedOffset = 0;
  sConfigInjected.InjectedNbrOfConversion = 3;
  sConfigInjected.InjectedDiscontinuousConvMode =ENABLE;           //DISABLE;
  sConfigInjected.AutoInjectedConv = DISABLE;
  sConfigInjected.QueueInjectedContext = DISABLE;
  sConfigInjected.ExternalTrigInjecConv = ADC_EXTERNALTRIGINJEC_T8_CC4 ;//ADC_EXTERNALTRIGINJEC_T1_TRGO;
  sConfigInjected.ExternalTrigInjecConvEdge = ADC_EXTERNALTRIGINJECCONV_EDGE_RISING;
  sConfigInjected.InjecOversamplingMode = DISABLE;
  if (HAL_ADCEx_InjectedConfigChannel(&hadc1, &sConfigInjected) != HAL_OK)
  {
    Error_Handler();
  }
	 sConfigInjected.InjectedChannel = ADC_CHANNEL_9;               //母线电流检测
  sConfigInjected.InjectedRank = ADC_INJECTED_RANK_2;
  if (HAL_ADCEx_InjectedConfigChannel(&hadc1, &sConfigInjected) != HAL_OK)
  {
    Error_Handler();
  }
		 sConfigInjected.InjectedChannel = ADC_CHANNEL_2; 
  sConfigInjected.InjectedRank = ADC_INJECTED_RANK_3;
  if (HAL_ADCEx_InjectedConfigChannel(&hadc1, &sConfigInjected) != HAL_OK)
  {
    Error_Handler();
  }

	  sConfig.Channel = ADC_CHANNEL_4;
    sConfig.Rank = 4;
    sConfig.SamplingTime = ADC_SAMPLETIME_6CYCLES_5  ;
    sConfig.Offset = 0;
    if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
    {
        Error_Handler();
    }
//		HAL_NVIC_SetPriority(ADC1_2_IRQn, 1, 0);
//		HAL_NVIC_EnableIRQ(ADC1_2_IRQn);
//		 ADC1->CR |= 0x06000 ;
//    ADC1->JSQR = PHASE_A_MSK + HALL_MSK1 + SEQUENCE_LENGHT;//
//		HAL_NVIC_SetPriority(ADC1_2_IRQn, 3, 0);
//		HAL_NVIC_EnableIRQ(ADC1_2_IRQn);
}

/**
  * @brief ADC2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC2_Init(void)
{

  /* USER CODE BEGIN ADC2_Init 0 */

  /* USER CODE END ADC2_Init 0 */

  ADC_InjectionConfTypeDef sConfigInjected = {0};
  ADC_ChannelConfTypeDef sConfig = {0};
		ADC_MultiModeTypeDef multimode = {0};

  /* USER CODE BEGIN ADC2_Init 1 */

  /* USER CODE END ADC2_Init 1 */
  /** Common config
  */
//  hadc2.Instance = ADC2;
//  hadc2.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV4;
//  hadc2.Init.Resolution = ADC_RESOLUTION_12B;
//  hadc2.Init.DataAlign = ADC_DATAALIGN_RIGHT;
//  hadc2.Init.GainCompensation = 0;
//  hadc2.Init.ScanConvMode = ADC_SCAN_ENABLE;
//  hadc2.Init.EOCSelection = DISABLE;//ENABLE;
//  hadc2.Init.LowPowerAutoWait = DISABLE;
//  hadc2.Init.ContinuousConvMode = DISABLE;
//  hadc2.Init.NbrOfConversion = 1;
//  hadc2.Init.DMAContinuousRequests = DISABLE;
//  hadc2.Init.Overrun = ADC_OVR_DATA_PRESERVED;
//  hadc2.Init.OversamplingMode = DISABLE;
  hadc2.Instance = ADC2;
  hadc2.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV4;    //原来是4
  hadc2.Init.Resolution = ADC_RESOLUTION_12B;
  hadc2.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc2.Init.GainCompensation = 0;
  hadc2.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc2.Init.EOCSelection = DISABLE;
  hadc2.Init.LowPowerAutoWait = DISABLE;
  hadc2.Init.ContinuousConvMode = DISABLE;
  hadc2.Init.NbrOfConversion = 1;
  hadc2.Init.DMAContinuousRequests = DISABLE;
  hadc2.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc2.Init.OversamplingMode = DISABLE;
  if (HAL_ADC_Init(&hadc2) != HAL_OK)
  {
    Error_Handler();
  }  multimode.Mode =ADC_MODE_INDEPENDENT ;
//  if (HAL_ADCEx_MultiModeConfigChannel(&hadc2, &multimode) != HAL_OK)
//  {
//    Error_Handler();
//  }
  /** Configure Injected Channel
  */
  sConfigInjected.InjectedChannel = ADC_CHANNEL_VOPAMP2;//A电机电流监测
  sConfigInjected.InjectedRank = ADC_INJECTED_RANK_1;
  sConfigInjected.InjectedSamplingTime = ADC_SAMPLETIME_12CYCLES_5;
  sConfigInjected.InjectedSingleDiff = ADC_SINGLE_ENDED;
  sConfigInjected.InjectedOffsetNumber = ADC_OFFSET_NONE;
  sConfigInjected.InjectedOffset = 0;
  sConfigInjected.InjectedNbrOfConversion = 4;
  sConfigInjected.InjectedDiscontinuousConvMode = ENABLE ;
  sConfigInjected.AutoInjectedConv = DISABLE ;
  sConfigInjected.QueueInjectedContext = DISABLE;
  sConfigInjected.ExternalTrigInjecConv =ADC_EXTERNALTRIGINJEC_T1_CC4;
  sConfigInjected.ExternalTrigInjecConvEdge =ADC_EXTERNALTRIGINJECCONV_EDGE_RISING;//ADC_EXTERNALTRIGINJECCONV_EDGE_RISING;
  sConfigInjected.InjecOversamplingMode = DISABLE;
  if (HAL_ADCEx_InjectedConfigChannel(&hadc2, &sConfigInjected) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Injected Channel
  */
  sConfigInjected.InjectedChannel = ADC_CHANNEL_17;
  sConfigInjected.InjectedRank = ADC_INJECTED_RANK_2;
  if (HAL_ADCEx_InjectedConfigChannel(&hadc2, &sConfigInjected) != HAL_OK)
  {
    Error_Handler();
  }
	
  sConfigInjected.InjectedChannel = ADC_CHANNEL_3;
  sConfigInjected.InjectedRank = ADC_INJECTED_RANK_3;
  if (HAL_ADCEx_InjectedConfigChannel(&hadc2, &sConfigInjected) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigInjected.InjectedChannel = ADC_CHANNEL_1;        
  sConfigInjected.InjectedRank = ADC_INJECTED_RANK_4;
  if (HAL_ADCEx_InjectedConfigChannel(&hadc2, &sConfigInjected) != HAL_OK)
  {
    Error_Handler();
  }
        HAL_ADC_MspInit(&hadc2);
		HAL_NVIC_SetPriority(ADC1_2_IRQn, 0, 0);
		HAL_NVIC_EnableIRQ(ADC1_2_IRQn);
  
}
//	  ADC2->CR |= 0x06000 ;
//    ADC2->JSQR = PHASE_B_MSK + HALL_MSK2 + SEQUENCE_LENGHT;//
	
  /** Configure Regular Channel
  */
//  sConfig.Channel = ADC_CHANNEL_7;
//  sConfig.Rank = ADC_REGULAR_RANK_1;
//  sConfig.SamplingTime = ADC_SAMPLETIME_24CYCLES_5;
//  sConfig.SingleDiff = ADC_SINGLE_ENDED;
//  sConfig.OffsetNumber = ADC_OFFSET_NONE;
//  sConfig.Offset = 0;
//  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
//  {
//    Error_Handler();
//  }
//  /** Configure Regular Channel
//  */
//  sConfig.Channel = ADC_CHANNEL_5;
//  sConfig.Rank = ADC_REGULAR_RANK_2;
//  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
//  {
//    Error_Handler();
//  }
  /* USER CODE BEGIN ADC2_Init 2 */

  /* USER CODE END ADC2_Init 2 */



/**
  * @brief ADC3 Initialization Function
  * @param None
  * @retval None
  */

/**
  * @brief CORDIC Initialization Function
  * @param None
  * @retval None
  */
static void MX_CORDIC_Init(void)
{

  /* USER CODE BEGIN CORDIC_Init 0 */

  /* USER CODE END CORDIC_Init 0 */

  /* USER CODE BEGIN CORDIC_Init 1 */

  /* USER CODE END CORDIC_Init 1 */
  hcordic.Instance = CORDIC;
  if (HAL_CORDIC_Init(&hcordic) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CORDIC_Init 2 */

  /* USER CODE END CORDIC_Init 2 */

}

/**
  * @brief DAC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_DAC1_Init(void)     
{

  /* USER CODE BEGIN DAC1_Init 0 */

  /* USER CODE END DAC1_Init 0 */

  DAC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN DAC1_Init 1 */

  /* USER CODE END DAC1_Init 1 */
  /** DAC Initialization
  */
  hdac1.Instance = DAC1;
  if (HAL_DAC_Init(&hdac1) != HAL_OK)
  {
    Error_Handler();
  }
  /** DAC channel OUT1 config
  */
  sConfig.DAC_HighFrequency = DAC_HIGH_FREQUENCY_INTERFACE_MODE_AUTOMATIC;
  sConfig.DAC_DMADoubleDataMode = DISABLE;
  sConfig.DAC_SignedFormat = DISABLE;
  sConfig.DAC_SampleAndHold = DAC_SAMPLEANDHOLD_DISABLE;
  sConfig.DAC_Trigger = DAC_TRIGGER_NONE;
  sConfig.DAC_Trigger2 = DAC_TRIGGER_NONE;
  sConfig.DAC_OutputBuffer = DAC_OUTPUTBUFFER_DISABLE;
  sConfig.DAC_ConnectOnChipPeripheral = DAC_CHIPCONNECT_INTERNAL;
  sConfig.DAC_UserTrimming = DAC_TRIMMING_FACTORY;
  if (HAL_DAC_ConfigChannel(&hdac1, &sConfig, DAC_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
	 if (HAL_DAC_ConfigChannel(&hdac1, &sConfig, DAC_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN DAC1_Init 2 */

  /* USER CODE END DAC1_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)            //周期1ms((9+1)*2*(8499+1)/170)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */
  TIMEx_BreakInputConfigTypeDef sBreakInputConfig = {0};
//  TIM_SlaveConfigTypeDef sSlaveConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
//  TIMEx_BreakInputConfigTypeDef sBreakInputConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};
	 TIM_ClockConfigTypeDef sClockSourceConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;//169/2;
  htim1.Init.CounterMode = TIM_COUNTERMODE_CENTERALIGNED1;
  htim1.Init.Period = 8499;//999;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV2;  //DIV2
  htim1.Init.RepetitionCounter = (REP_COUNTER);
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
	sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
//  sSlaveConfig.SlaveMode = TIM_SLAVEMODE_TRIGGER;
//  sSlaveConfig.InputTrigger = TIM_TS_ITR1;
//  if (HAL_TIM_SlaveConfigSynchro(&htim1, &sSlaveConfig) != HAL_OK)
//  {
//    Error_Handler();
//  }
	  if (HAL_TIM_OC_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;//TIM_TRGO_OC4REF;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
//  sBreakInputConfig.Source = TIM_BREAKINPUTSOURCE_BKIN;
//  sBreakInputConfig.Enable = TIM_BREAKINPUTSOURCE_ENABLE;
//  sBreakInputConfig.Polarity = TIM_BREAKINPUTSOURCE_POLARITY_LOW;
//  if (HAL_TIMEx_ConfigBreakInput(&htim1, TIM_BREAKINPUT_BRK2, &sBreakInputConfig) != HAL_OK)
//  {
//    Error_Handler();
//  }
//  LL_TIM_SetOCRefClearInputSource(TIM1,LL_TIM_OCREF_CLR_INT_COMP1);   //COMP触发TIM_Ocref_clr
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_LOW;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
	
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.Pulse = 0;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.Pulse = 0;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM2;
  sConfigOC.Pulse = 50;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
    if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_5) != HAL_OK)
  {
    Error_Handler();
  }
  
	sBreakInputConfig.Source = TIM_BREAKINPUTSOURCE_COMP2;
  sBreakInputConfig.Enable = TIM_BREAKINPUTSOURCE_ENABLE ;
  sBreakInputConfig.Polarity = TIM_BREAKINPUTSOURCE_POLARITY_LOW;    //low
  if (HAL_TIMEx_ConfigBreakInput(&htim1, TIM_BREAKINPUT_BRK, &sBreakInputConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_ENABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_ENABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_1;
  sBreakDeadTimeConfig.DeadTime = ((DEAD_TIME_COUNTS) / 2);;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_ENABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_LOW;
  sBreakDeadTimeConfig.BreakFilter = 0xF;
  sBreakDeadTimeConfig.BreakAFMode = TIM_BREAK_AFMODE_INPUT;
  sBreakDeadTimeConfig.Break2State = TIM_BREAK2_ENABLE;
  sBreakDeadTimeConfig.Break2Polarity = TIM_BREAK2POLARITY_HIGH;
  sBreakDeadTimeConfig.Break2Filter = 0;
  sBreakDeadTimeConfig.Break2AFMode = TIM_BREAK_AFMODE_INPUT;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
	

  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);
   __HAL_TIM_ENABLE_IT(&htim1, TIM_IT_BREAK);

}



/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = M1_PULSE_NBR;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = M1_ENC_IC_FILTER;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = M1_ENC_IC_FILTER;
  if (HAL_TIM_Encoder_Init(&htim2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_HallSensor_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = M1_HALL_TIM_PERIOD;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = M1_HALL_IC_FILTER;
  sConfig.Commutation_Delay = 0;
  if (HAL_TIMEx_HallSensor_Init(&htim3, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_OC2REF;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
//static void MX_TIM4_Init(void)
//{

//  /* USER CODE BEGIN TIM4_Init 0 */

//  /* USER CODE END TIM4_Init 0 */

//  TIM_Encoder_InitTypeDef sConfig = {0};
//  TIM_MasterConfigTypeDef sMasterConfig = {0};

//  /* USER CODE BEGIN TIM4_Init 1 */

//  /* USER CODE END TIM4_Init 1 */
//  htim4.Instance = TIM4;
//  htim4.Init.Prescaler = 0;
//  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
//  htim4.Init.Period = M2_PULSE_NBR;
//  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
//  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
//  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
//  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
//  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
//  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
//  sConfig.IC1Filter = M2_ENC_IC_FILTER;
//  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
//  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
//  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
//  sConfig.IC2Filter = M2_ENC_IC_FILTER;
//  if (HAL_TIM_Encoder_Init(&htim4, &sConfig) != HAL_OK)
//  {
//    Error_Handler();
//  }
//  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
//  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
//  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
//  {
//    Error_Handler();
//  }
//  /* USER CODE BEGIN TIM4_Init 2 */

//  /* USER CODE END TIM4_Init 2 */

//}

/**
  * @brief TIM5 Initialization Function
  * @param None
  * @retval None
  */
//static void MX_TIM5_Init(void)
//{

//  /* USER CODE BEGIN TIM5_Init 0 */

//  /* USER CODE END TIM5_Init 0 */

//  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
//  TIM_HallSensor_InitTypeDef sConfig = {0};
//  TIM_MasterConfigTypeDef sMasterConfig = {0};

//  /* USER CODE BEGIN TIM5_Init 1 */

//  /* USER CODE END TIM5_Init 1 */
//  htim5.Instance = TIM5;
//  htim5.Init.Prescaler = 0;
//  htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
//  htim5.Init.Period = M2_HALL_TIM_PERIOD;
//  htim5.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
//  htim5.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
//  if (HAL_TIM_Base_Init(&htim5) != HAL_OK)
//  {
//    Error_Handler();
//  }
//  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
//  if (HAL_TIM_ConfigClockSource(&htim5, &sClockSourceConfig) != HAL_OK)
//  {
//    Error_Handler();
//  }
//  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
//  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
//  sConfig.IC1Filter = M2_HALL_IC_FILTER;
//  sConfig.Commutation_Delay = 0;
//  if (HAL_TIMEx_HallSensor_Init(&htim5, &sConfig) != HAL_OK)
//  {
//    Error_Handler();
//  }
//  sMasterConfig.MasterOutputTrigger = TIM_TRGO_OC2REF;
//  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
//  if (HAL_TIMEx_MasterConfigSynchronization(&htim5, &sMasterConfig) != HAL_OK)
//  {
//    Error_Handler();
//  }
//  /* USER CODE BEGIN TIM5_Init 2 */

//  /* USER CODE END TIM5_Init 2 */

//}

/**
  * @brief TIM8 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM8_Init(void)               //周期1ms     
{

  /* USER CODE BEGIN TIM8_Init 0 */

  /* USER CODE END TIM8_Init 0 */

//  TIM_SlaveConfigTypeDef sSlaveConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
//  TIMEx_BreakInputConfigTypeDef sBreakInputConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};
	TIMEx_BreakInputConfigTypeDef sBreakInputConfig = {0};
	 TIM_ClockConfigTypeDef sClockSourceConfig = {0};
//   TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  /* USER CODE BEGIN TIM8_Init 1 */

  /* USER CODE END TIM8_Init 1 */
  htim8.Instance = TIM8;
  htim8.Init.Prescaler = 0;//1532/168;//169/2;
  htim8.Init.CounterMode = TIM_COUNTERMODE_CENTERALIGNED1;
  htim8.Init.Period = 8499;//8399;
  htim8.Init.ClockDivision = TIM_CLOCKDIVISION_DIV2;
  htim8.Init.RepetitionCounter = (REP_COUNTER2);
  htim8.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim8) != HAL_OK)
  {
    Error_Handler();
  }
//  sSlaveConfig.SlaveMode = TIM_SLAVEMODE_TRIGGER;
//  sSlaveConfig.InputTrigger = TIM_TS_ITR1;
//  if (HAL_TIM_SlaveConfigSynchro(&htim8, &sSlaveConfig) != HAL_OK)
//  {
//    Error_Handler();
//  }
  	  if (HAL_TIM_OC_Init(&htim8) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim8, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
//  sBreakInputConfig.Source = TIM_BREAKINPUTSOURCE_BKIN;
//  sBreakInputConfig.Enable = TIM_BREAKINPUTSOURCE_ENABLE;
//  sBreakInputConfig.Polarity = TIM_BREAKINPUTSOURCE_POLARITY_LOW;
//  if (HAL_TIMEx_ConfigBreakInput(&htim8, TIM_BREAKINPUT_BRK2, &sBreakInputConfig) != HAL_OK)
//  {
//    Error_Handler();
//  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_LOW;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim8, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.Pulse = 0;
  if (HAL_TIM_PWM_ConfigChannel(&htim8, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.Pulse = 0;
  if (HAL_TIM_PWM_ConfigChannel(&htim8, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM2;
  sConfigOC.Pulse = 1000;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  if (HAL_TIM_PWM_ConfigChannel(&htim8, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 2000;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  if (HAL_TIM_PWM_ConfigChannel(&htim8, &sConfigOC, TIM_CHANNEL_5) != HAL_OK)
  {
    Error_Handler();
  }
	sBreakInputConfig.Source = TIM_BREAKINPUTSOURCE_COMP1;
  sBreakInputConfig.Enable = TIM_BREAKINPUTSOURCE_ENABLE ;
  sBreakInputConfig.Polarity = TIM_BREAKINPUTSOURCE_POLARITY_LOW;
  if (HAL_TIMEx_ConfigBreakInput(&htim8, TIM_BREAKINPUT_BRK, &sBreakInputConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_ENABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_ENABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_1;
  sBreakDeadTimeConfig.DeadTime =  ((DEAD_TIME_COUNTS) / 2);;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_ENABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_LOW;
  sBreakDeadTimeConfig.BreakFilter = 0xF;
  sBreakDeadTimeConfig.BreakAFMode = TIM_BREAK_AFMODE_INPUT;
  sBreakDeadTimeConfig.Break2State = TIM_BREAK2_DISABLE;
  sBreakDeadTimeConfig.Break2Polarity = TIM_BREAK2POLARITY_HIGH;
  sBreakDeadTimeConfig.Break2Filter = 0;
  sBreakDeadTimeConfig.Break2AFMode = TIM_BREAK_AFMODE_INPUT;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim8, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM8_Init 2 */

  /* USER CODE END TIM8_Init 2 */
  HAL_TIM_MspPostInit(&htim8);
   __HAL_TIM_ENABLE_IT(&htim8, TIM_IT_BREAK);

}
static void MX_TIM6_Init(void)
{

    /* USER CODE BEGIN TIM9_Init 0 */

    /* USER CODE END TIM9_Init 0 */

    TIM_IC_InitTypeDef sConfigIC = {0};
     TIM_ClockConfigTypeDef sClockSourceConfig;
    /* USER CODE BEGIN TIM6_Init 1 */
    __HAL_RCC_TIM6_CLK_ENABLE();
    /* USER CODE END TIM6_Init 1 */
    htim6.Instance = TIM6;
    htim6.Init.Prescaler = 169;
    htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim6.Init.Period = 999;
    htim6.Init.ClockDivision = TIM_CLOCKDIVISION_DIV2;
    htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
    {
        Error_Handler();
    }

    /* USER CODE BEGIN TIM9_Init 2 */

    /* USER CODE END TIM9_Init 2 */

}
static void MX_TIM4_Init(void)
{

    /* USER CODE BEGIN TIM9_Init 0 */

    /* USER CODE END TIM9_Init 0 */

    TIM_IC_InitTypeDef sConfigIC = {0};
     TIM_ClockConfigTypeDef sClockSourceConfig;
    /* USER CODE BEGIN TIM6_Init 1 */
    __HAL_RCC_TIM4_CLK_ENABLE();
    /* USER CODE END TIM6_Init 1 */
    htim4.Instance = TIM4;
    htim4.Init.Prescaler = 0;
    htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim4.Init.Period = 1699;
    htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV2;
    htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
    {
        Error_Handler();
    }

    /* USER CODE BEGIN TIM9_Init 2 */

    /* USER CODE END TIM9_Init 2 */

}
static void MX_TIM15_Init(void)
{

    /* USER CODE BEGIN TIM9_Init 0 */

    /* USER CODE END TIM9_Init 0 */

    TIM_IC_InitTypeDef sConfigIC = {0};

    /* USER CODE BEGIN TIM9_Init 1 */

    /* USER CODE END TIM9_Init 1 */
    htim15.Instance = TIM15;
    htim15.Init.Prescaler = 0;//169; 99
    htim15.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim15.Init.Period = 169;//999;63000
    htim15.Init.ClockDivision = TIM_CLOCKDIVISION_DIV2;
    htim15.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    if (HAL_TIM_Base_Init(&htim15) != HAL_OK)
    {
        Error_Handler();
    }

    /* USER CODE BEGIN TIM9_Init 2 */

    /* USER CODE END TIM9_Init 2 */

}

static void MX_TIM16_Init(void)
{

    /* USER CODE BEGIN TIM9_Init 0 */

    /* USER CODE END TIM9_Init 0 */

    TIM_IC_InitTypeDef sConfigIC = {0};

    /* USER CODE BEGIN TIM9_Init 1 */

    /* USER CODE END TIM9_Init 1 */
    htim16.Instance = TIM16;
    htim16.Init.Prescaler = 169;//169
    htim16.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim16.Init.Period = 65535; //65535
    htim16.Init.ClockDivision = TIM_CLOCKDIVISION_DIV2;
    htim16.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    if (HAL_TIM_Base_Init(&htim16) != HAL_OK)
    {
        Error_Handler();
    }

    /* USER CODE BEGIN TIM9_Init 2 */

    /* USER CODE END TIM9_Init 2 */

}
void HAL_TIMEx_BreakCallback(TIM_HandleTypeDef *htim)
{
    if(htim == (&htim1))
    {
//        if (HAL_COMP_GetOutputLevel(&hcomp2)==1)
//        {
            TIM1->CCR1 = 0;
            TIM1->CCR2 = 0;
            TIM1->CCR3 = 0;
			TIM1->EGR = 1;
		    MC_SetFault(M1_BREAK);
//        }
//        else
//        {
//        }
    }
	if(htim == (&htim8))
    {
//        if (HAL_COMP_GetOutputLevel(&hcomp1)==1 )
//        {
            TIM8->CCR1 = 0;
            TIM8->CCR2 = 0;
            TIM8->CCR3 = 0;
			TIM8->EGR = 1;
		    MC_SetFault(M2_BREAK);
//        }
//        else
//        {
//        }
    }
}
/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */



static void MX_OPAMP1_Init(void)             //OPAMP初始化  B电机
{

  /* USER CODE BEGIN OPAMP1_Init 0 */

  /* USER CODE END OPAMP1_Init 0 */

  /* USER CODE BEGIN OPAMP1_Init 1 */

  /* USER CODE END OPAMP1_Init 1 */
  hopamp1.Instance = OPAMP1;
  hopamp1.Init.PowerMode = OPAMP_POWERMODE_HIGHSPEED;
  hopamp1.Init.Mode = OPAMP_PGA_MODE;          //放大
  hopamp1.Init.NonInvertingInput =OPAMP_NONINVERTINGINPUT_IO0;   //PA1  for OPAMP1
  hopamp1.Init.InternalOutput = ENABLE;       //内部输出，ADC_CHANNEL_VOPAMP1
  hopamp1.Init.TimerControlledMuxmode = OPAMP_TIMERCONTROLLEDMUXMODE_DISABLE;
  hopamp1.Init.PgaConnect = OPAMP_PGA_CONNECT_INVERTINGINPUT_IO0_BIAS ;// 正反馈
  hopamp1.Init.PgaGain = OPAMP_PGA_GAIN_64_OR_MINUS_63      ;     //放大比例
  hopamp1.Init.UserTrimming = OPAMP_TRIMMING_FACTORY;
  if (HAL_OPAMP_Init(&hopamp1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN OPAMP1_Init 2 */

  /* USER CODE END OPAMP1_Init 2 */
}                                                                                                                                                                                                                                                                                                            

/**
  * @brief OPAMP2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_OPAMP2_Init(void)          //A电机
{

  /* USER CODE BEGIN OPAMP2_Init 0 */

  /* USER CODE END OPAMP2_Init 0 */

  /* USER CODE BEGIN OPAMP2_Init 1 */

  /* USER CODE END OPAMP2_Init 1 */
  hopamp2.Instance = OPAMP2;
  hopamp2.Init.PowerMode = OPAMP_POWERMODE_HIGHSPEED;  
  hopamp2.Init.Mode = OPAMP_PGA_MODE;                //放大模式
//  hopamp2.Init.InvertingInput = OPAMP_INVERTINGINPUT_IO0;     //PA5  for OPAMP2
  hopamp2.Init.NonInvertingInput = OPAMP_NONINVERTINGINPUT_IO0;     //PA7  for OPAMP2
  hopamp2.Init.InternalOutput = ENABLE;           //内部输出，ADC_CHANNEL_VOPAMP2
  hopamp2.Init.TimerControlledMuxmode = OPAMP_TIMERCONTROLLEDMUXMODE_DISABLE;
  hopamp2.Init.PgaConnect = OPAMP_PGA_CONNECT_INVERTINGINPUT_IO0_BIAS   ;//OPAMP_PGA_CONNECT_INVERTINGINPUT_IO0_BIAS;// 正反馈
  hopamp2.Init.PgaGain = OPAMP_PGA_GAIN_64_OR_MINUS_63     ;    //放大比例
  hopamp2.Init.UserTrimming = OPAMP_TRIMMING_FACTORY;
  if (HAL_OPAMP_Init(&hopamp2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN OPAMP2_Init 2 */

  /* USER CODE END OPAMP2_Init 2 */

}


static void MX_COMP1_Init(void)
{

  /* USER CODE BEGIN COMP1_Init 0 */

  /* USER CODE END COMP1_Init 0 */

  /* USER CODE BEGIN COMP1_Init 1 */

  /* USER CODE END COMP1_Init 1 */
  hcomp1.Instance = COMP1;
  hcomp1.Init.InputPlus = COMP_INPUT_PLUS_IO1;          //PA1 for COMP1     B电机
  hcomp1.Init.InputMinus = COMP_INPUT_MINUS_DAC1_CH1;   //DAC输出，通过设置CH1，可以得到保护电压，从而保护电路
  hcomp1.Init.OutputPol = COMP_OUTPUTPOL_NONINVERTED ;//comparator output is high when the input plus is at a higher voltage than the input minus，从而触发TIM1 BRAKE
  hcomp1.Init.Hysteresis = COMP_HYSTERESIS_NONE;
  hcomp1.Init.BlankingSrce =COMP_BLANKINGSRC_TIM8_OC5_COMP1; //COMP_BLANKINGSRC_TIM1_OC5_COMP1;
  hcomp1.Init.TriggerMode = COMP_TRIGGERMODE_NONE;
  if (HAL_COMP_Init(&hcomp1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN COMP1_Init 2 */

  /* USER CODE END COMP1_Init 2 */

}

static void MX_COMP2_Init(void)
{

  /* USER CODE BEGIN COMP1_Init 0 */

  /* USER CODE END COMP1_Init 0 */

  /* USER CODE BEGIN COMP1_Init 1 */

  /* USER CODE END COMP1_Init 1 */
  hcomp2.Instance = COMP2;
  hcomp2.Init.InputPlus = COMP_INPUT_PLUS_IO1;       //PA7 for COMP2        A电机
  hcomp2.Init.InputMinus = COMP_INPUT_MINUS_DAC1_CH2;      //DAC输出，通过设置CH2，可以得到保护电压，从而保护电路
  hcomp2.Init.OutputPol = COMP_OUTPUTPOL_NONINVERTED ;//comparator output is high when the input plus is at a higher voltage than the input minus，从而触发TIM1 BRAKE
  hcomp2.Init.Hysteresis = COMP_HYSTERESIS_NONE;
  hcomp2.Init.BlankingSrce =COMP_BLANKINGSRC_TIM1_OC5_COMP2; //COMP_BLANKINGSRC_TIM1_OC5_COMP1;
  hcomp2.Init.TriggerMode = COMP_TRIGGERMODE_NONE;
  if (HAL_COMP_Init(&hcomp2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN COMP1_Init 2 */

  /* USER CODE END COMP1_Init 2 */

}

static void MX_DAC3_Init(void)     
{

  /* USER CODE BEGIN DAC1_Init 0 */

  /* USER CODE END DAC1_Init 0 */

  DAC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN DAC1_Init 1 */

  /* USER CODE END DAC1_Init 1 */
  /** DAC Initialization
  */
  hdac3.Instance = DAC3;
  if (HAL_DAC_Init(&hdac3) != HAL_OK)
  {
    Error_Handler();
  }
  /** DAC channel OUT1 config
  */
  sConfig.DAC_HighFrequency = DAC_HIGH_FREQUENCY_INTERFACE_MODE_AUTOMATIC;
  sConfig.DAC_DMADoubleDataMode = DISABLE;
  sConfig.DAC_SignedFormat = DISABLE;
  sConfig.DAC_SampleAndHold = DAC_SAMPLEANDHOLD_DISABLE;
  sConfig.DAC_Trigger = DAC_TRIGGER_NONE;
  sConfig.DAC_Trigger2 = DAC_TRIGGER_NONE;
  sConfig.DAC_OutputBuffer = DAC_OUTPUTBUFFER_ENABLE;
  sConfig.DAC_ConnectOnChipPeripheral = DAC_CHIPCONNECT_INTERNAL;
  sConfig.DAC_UserTrimming = DAC_TRIMMING_FACTORY;
  if (HAL_DAC_ConfigChannel(&hdac3, &sConfig, DAC_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
	 if (HAL_DAC_ConfigChannel(&hdac3, &sConfig, DAC_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN DAC1_Init 2 */

  /* USER CODE END DAC1_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */

  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();
   __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin : Start_Stop_Pin */
//	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_RESET);

//  /*Configure GPIO pin : PB4 */
//  GPIO_InitStruct.Pin = GPIO_PIN_4;
//  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
//  GPIO_InitStruct.Pull = GPIO_NOPULL;
//  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
//  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
//  GPIO_InitStruct.Pin = GPIO_PIN_4;
//  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
//  GPIO_InitStruct.Pull = GPIO_NOPULL;
//  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
//  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
	
//	GPIO_InitStruct.Pin = GPIO_PIN_0;               //按键检测
//  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
//  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
//  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
//	HAL_NVIC_SetPriority(EXTI0_IRQn, 0, 0);
//  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

  GPIO_InitStruct.Pin = M2_HALL_H1_Pin|M2_HALL_H2_Pin|M2_HALL_H3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(M2_HALL_H2_GPIO_Port, &GPIO_InitStruct);
	
	GPIO_InitStruct.Pin = M1_HALL_H1_Pin|M1_HALL_H2_Pin|M1_HALL_H3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(M1_HALL_H1_GPIO_Port, &GPIO_InitStruct);
	
	GPIO_InitStruct.Pin = Version_1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(Version_1_GPIO_Port, &GPIO_InitStruct);
  
    GPIO_InitStruct.Pin = GPIO_PIN_4;             //暂时的
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
  HAL_GPIO_WritePin(GPIOA,GPIO_PIN_12,0) ;  //CAN_TX
  
    GPIO_InitStruct.Pin = Fault_1_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);
//	RED_LED1_OFF;
	
	GPIO_InitStruct.Pin = GPIO_PIN_10;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
    HAL_GPIO_WritePin(GPIOB,GPIO_PIN_10,1);
}






void MX_FDCAN1_Init(void)
{

  hfdcan1.Instance = FDCAN1;
  hfdcan1.Init.ClockDivider = FDCAN_CLOCK_DIV4;    //4分频     Fclk=170/4
  hfdcan1.Init.FrameFormat = FDCAN_FRAME_CLASSIC;   //帧格式
  hfdcan1.Init.Mode = FDCAN_MODE_NORMAL;           //正常模式
  hfdcan1.Init.AutoRetransmission = DISABLE;         /*使能自动重发 */ 
  hfdcan1.Init.TransmitPause = DISABLE;          /* 配置禁止传输暂停特性 */  
  hfdcan1.Init.ProtocolException = DISABLE;      /* 协议异常处理使能 */
  hfdcan1.Init.NominalPrescaler = 1;
  hfdcan1.Init.NominalSyncJumpWidth = 8;
  hfdcan1.Init.NominalTimeSeg1 = 68;
  hfdcan1.Init.NominalTimeSeg2 = 16;//仲裁段Baud=fclk/(1+68+16),此程序中Fclk为170/4，仲裁段500kps
  hfdcan1.Init.DataPrescaler = 1;
  hfdcan1.Init.DataSyncJumpWidth = 8;
  hfdcan1.Init.DataTimeSeg1 = 5;
  hfdcan1.Init.DataTimeSeg2 = 4;    //数据段Baud=fclk/(1+5+4)
  hfdcan1.Init.StdFiltersNbr = 1;
  hfdcan1.Init.ExtFiltersNbr = 0;
  hfdcan1.Init.TxFifoQueueMode = FDCAN_TX_FIFO_OPERATION;
  if (HAL_FDCAN_Init(&hfdcan1) != HAL_OK)
  {
    Error_Handler();
  }

}

  void FDCAN_Config(void)
{
  FDCAN_FilterTypeDef sFilterConfig;

  /* Configure Rx filter */
  sFilterConfig.IDE = FDCAN_STANDARD_ID;
  sFilterConfig.FilterIndex = 0;
  sFilterConfig.FilterType = FDCAN_FILTER_MASK;     //位屏蔽过滤
  sFilterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
  sFilterConfig.FilterID1 = 0x0000;                 //屏蔽位模式下，FilterID1是消息ID
  sFilterConfig.FilterID2 = 0x0000;                 //屏蔽位模式下，FilterID2是消息屏蔽位       0：表示FilterID1相应bit不关心。1表示FilterID1相应bit必须匹配
	    TxHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;      /* 设置错误状态指示 */
    TxHeader.BitRateSwitch = FDCAN_BRS_OFF;                  /* 开启可变波特率 */
    TxHeader.FDFormat = FDCAN_CLASSIC_CAN;                       /* FDCAN格式 */
    TxHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS;        /* 用于发送事件FIFO控制, 不存储 */
    TxHeader.MessageMarker = 0;   
  if (HAL_FDCAN_ConfigFilter(&hfdcan1, &sFilterConfig) != HAL_OK)          //配置过滤器
  {
    Error_Handler();
  }

  /* Configure global filter:
     Filter all remote frames with STD and EXT ID
     Reject non matching frames with STD ID and EXT ID */
  if (HAL_FDCAN_ConfigGlobalFilter(&hfdcan1, FDCAN_ACCEPT_IN_RX_FIFO0, FDCAN_ACCEPT_IN_RX_FIFO0, FDCAN_FILTER_REMOTE, FDCAN_FILTER_REMOTE) != HAL_OK)
  {
    Error_Handler();
  }

  /* Start the FDCAN module */
  if (HAL_FDCAN_Start(&hfdcan1) != HAL_OK)
  {
    Error_Handler();
  }

  if (HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0) != HAL_OK)
  {
    Error_Handler();
  }
}
/* USER CODE BEGIN 4 */
void MX_IWDG_Init(uint8_t prv ,uint16_t rlv)
{
  
  hiwdg.Instance = IWDG;
  hiwdg.Init.Prescaler = prv;
  hiwdg.Init.Reload = rlv;
  hiwdg.Init.Window=IWDG_WINDOW_DISABLE;
  HAL_IWDG_Init(&hiwdg);

}



 extern   I2C_HandleTypeDef hi2c3;
static void MX_I2C3_Init(void)
{

  /* USER CODE BEGIN I2C3_Init 0 */

  /* USER CODE END I2C3_Init 0 */

  /* USER CODE BEGIN I2C3_Init 1 */

  /* USER CODE END I2C3_Init 1 */
  hi2c3.Instance = I2C3;
  hi2c3.Init.Timing = 0x00802172;
  hi2c3.Init.OwnAddress1 = 0;
  hi2c3.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c3.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c3.Init.OwnAddress2 = 0;
  hi2c3.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c3.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c3.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c3) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c3, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c3, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /** I2C Fast mode Plus enable
  */
  __HAL_SYSCFG_FASTMODEPLUS_ENABLE(I2C_FASTMODEPLUS_I2C3);
  /* USER CODE BEGIN I2C3_Init 2 */
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
int fputc(int ch, FILE *f)
{
  HAL_UART_Transmit(&huart1, (uint8_t *)&ch, 1, 0xffff);
  return ch;
}

/**
  * 函数功能: 重定向c库函数getchar,scanf到DEBUG_USARTx
  * 输入参数: 无
  * 返 回 值: 无
  * 说    明：无
  */
int fgetc(FILE * f)
{
  uint8_t ch = 0;
  HAL_UART_Receive(&huart1,&ch, 1, 0xffff);
  return ch;
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
