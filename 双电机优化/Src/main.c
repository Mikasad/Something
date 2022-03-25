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
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include <stdio.h>
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "bsp_BDCMotor.h"
#include "canopen_od.h"
#include "canopen_timer.h"
#include "ds402.h"
#include "canopen_pdo.h"
#include "bsp_iwdg.h"
#include "Agreement.h"
#include "flash.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
CAN_HandleTypeDef hcan1;
CAN_HandleTypeDef hcan2;
CanTxMsgTypeDef TxMessage;
CanRxMsgTypeDef RxMessage;
MCU_Version ProgramVersion;
uint8_t Rx_flag = 0;
STRUCT_CAPTURE strCapture = { 0, 0, 0 };
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

CAN_HandleTypeDef hcan2;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim5;
TIM_HandleTypeDef htim6;
TIM_HandleTypeDef htim7;
TIM_HandleTypeDef htim8;
TIM_HandleTypeDef htim9;
TIM_HandleTypeDef htim10;
TIM_HandleTypeDef htim11;
TIM_HandleTypeDef htim12;

UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */
void Sys_Time(void);
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_CAN2_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
//static void MX_TIM6_Init(void);
static void MX_TIM5_Init(void);
static void MX_TIM7_Init(void);
static void MX_TIM8_Init(void);
static void MX_TIM9_Init(void);
static void MX_TIM10_Init(void);
static void MX_TIM11_Init(void);
static void MX_TIM12_Init(void);
//static void MX_USART3_UART_Init(void);
static void MX_NVIC_Init(void);
void Labview_uart(void);
int PWM_Change(int time_cnt,int Motor_PWM);
/* USER CODE BEGIN PFP */
uint32_t ADC_ConvertedValue[12];
uint32_t DMA_Transfer_Complete_Count=0;
uint8_t aRxBuffer[50];
SysCLC_TypeDef SysCLC;
SystStatus_t MotorState;
int CH1TEMP = 0, CH2TEMP = 0;
u8 Push_motor_calibrationFLAG = 0;
u8 First_PowerOn = 1;
u8 First_Enter_STUDY_MOTOR5 = 1;
u8 First_Enter_STUDY_MOTOR6 = 1;
u16 Uart_TxBuffer_CNT = 0,Uart_TxBuffer_CNT1 = 0;
int16_t LV_Uart_TxBuffer[4][2000] = {0};
u8 motorset=0;
uint32_t Err_codeHis[10],Err_codeTick[10];
int8_t OverFlow_Cnt[5];
int avglecount=0;
int avglecountB=0;
extern s32 cntime;
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/**
  * @brief CAN????,???????can1?can2
  * @retval None
  */
void HAL_CAN_RxCpltCallback(CAN_HandleTypeDef *hcan)
{
    {
        if(CAN2->RF0R & (1<<4))
        {
            CAN2->RF0R &= (0<<4);
        }
        if(CANopen_Drive.CanRx_Buffer->IDE == BIT_INVALID)
        {
            CanLoadRate.FrameSize = CanLoadRate.FrameSize + (47 + CANopen_Drive.CanRx_Buffer->DLC * 8);
            MessageType_Check(&CANopen_Drive,CANopen_Drive.CanRx_Buffer->StdId,CANopen_Drive.CanRx_Buffer->RTR);
        }
        else
        {}
        HAL_CAN_Receive_IT(hcan, CAN_FIFO0);
    }
}
/**
  * @brief CAN1 ????
  * @param None
  * @retval None
  */
uint8_t CAN1_Send_Msg(uint32_t stdId,uint32_t Ide,uint32_t Rtr,uint8_t* msg,uint8_t len)
{
    uint16_t i=0;
    hcan2.pTxMsg->StdId=stdId;        //?????0001_0010 ??????????????????
    hcan2.pTxMsg->ExtId=0xFFF;        //?????(29?)
    hcan2.pTxMsg->IDE=Ide;    //?????(????)
    hcan2.pTxMsg->RTR=Rtr;  //???(????)
    hcan2.pTxMsg->DLC=len;           //????
    for(i=0; i<len; i++)
    {
        hcan2.pTxMsg->Data[i]=msg[i];
    }
    if(HAL_CAN_Transmit(&hcan2,20)!=HAL_OK)
//		{
//			pHeartBeatPar.Timeout_server = 0;
//		}


        CanLoadRate.FrameSize = CanLoadRate.FrameSize + (47 + hcan2.pTxMsg->DLC * 8);
    return i != 3? 0:1;
}
//uint8_t canbuf[8]={1,1,1,1,1,1,1,1};  //2a��?
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
u8 enableallmotor = 0;
u8 errormotor5 = 0;
u8 errormotor6 = 0;
int Programcount;
int main(void)
{
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
    MX_DMA_Init();
    MX_ADC1_Init();
//    MX_CAN2_Init();
    MX_TIM1_Init();			/* ��ˢ5 ������ˢ13*/
    MX_TIM2_Init();			/* 0_1���Ƹ� */
    MX_TIM3_Init();			/* ��ˢ7 ���8 ������ˢ10_11*/
    MX_TIM4_Init();			/* 2_9���Ƹ� */
	MX_TIM5_Init();			/* �Ż���ǰ�Ǽ�hall���� */
    TIM6_Init();				/* canͨ�� */
    // MX_TIM6_Init();
    MX_TIM7_Init();			/* ��ʱ�������ٶ� */
    MX_TIM8_Init();			/* ��ˢ6 ������ˢ12*/
    MX_TIM9_Init();			/* ���벶�� */
    MX_TIM10_Init();		/* ������ˢ��ŷ�3 */
    MX_TIM11_Init();		/* ������ˢ��ŷ�4 */
	MX_TIM12_Init();
    MX_USART3_UART_Init();
    __HAL_USART_ENABLE_IT(&huart3,USART_IT_RXNE );    //���������жϴ�����
    /* USER CODE BEGIN 2 */
    MX_IWDG_Init(IWDG_PRESCALER_64,3125);  //5S
//    __HAL_IWDG_START(&hiwdg);	/* �����������Ź� */

    MX_NVIC_Init();

    HAL_ADC_Start_DMA(&hadc1,ADC_ConvertedValue,12);
    Init_Drive_Para();
    Program_Version_Init(&ProgramVersion,'5',1,0,0);
    HardVersion_Init(&ProgramVersion,'H','A',30,2);
    Flash_Read(ParaNum);
    Hardware_flowCheck();		//���������������Ƿ���	2021.1125
    HAL_TIM_Base_Start_IT(&htim1);
    HAL_TIM_Base_Start_IT(&htim5);
    HAL_TIM_Base_Start_IT(&htim7);
    HAL_TIM_Base_Start_IT(&htim10);
    HAL_TIM_Base_Start_IT(&htim11);
	HAL_TIM_Base_Start_IT(&htim12);


    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_4);

    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);
    HAL_Delay(500);														//����delay��Ϊ�˸�������ݳ�磬��ֹ������Ӳ������
    HAL_NVIC_SetPriority(EXTI9_5_IRQn, 1, 0);
    HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);					//��ͨ�磬�ٿ���Ӳ���ж�
    HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_4);
    HAL_TIM_OC_Start_IT(&htim5,TIM_CHANNEL_1);
	HAL_TIM_OC_Start_IT(&htim12,TIM_CHANNEL_1);


    /* USER CODE END 2 */

    CANopen_Parameter_Init(&CANopen_Drive);
    HAL_CAN_Receive_IT(&hcan2, CAN_FIFO0);
    HAL_UART_Receive_IT(&huart3,aRxBuffer,1);

    __HAL_TIM_CLEAR_IT(&htim9, TIM_IT_CC1);
    __HAL_TIM_CLEAR_IT(&htim9, TIM_IT_CC2);
    HAL_TIM_IC_Start_IT(&htim9,TIM_CHANNEL_1);
    HAL_TIM_IC_Start_IT(&htim9,TIM_CHANNEL_2);

    HAL_TIM_PWM_Start(&htim10, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim11, TIM_CHANNEL_1);

    /* Infinite loop */
    /* USER CODE BEGIN WHILE */
    while (1)
    {
//		 __HAL_TIM_SET_COMPARE(&htim5,TIM_CHANNEL_1,cntime*5/36);
		Programcount++;
        if(CanLoadRate.timer >= 10000) //���߸����ʼ���
        {
            CanLoadRate.timer = 0;
            CanLoadRate.Rate = (CanLoadRate.FrameSize * (1000))/(obj2099_CanBandrate_Default*1000);
            CanLoadRate.FrameSize = 0;
        }
        if(First_PowerOn == 1)
        {
            if(HAL_GetTick()%1000 ==0)
            {
                Push_motor_calibrationFLAG = 1;
                First_PowerOn = 0;
            }
        }
        if(enableallmotor == 1)			//ʹ�����е��
        {
            enableallmotor = 0;
            for(u8 i = 0 ; i <= 13; i++)
            {
                MotorControl[i].Fault_Flag = 0;
                MotorControl[i].Motor_Start_Stop = 1;
                if(errormotor5 > 10)
                {
                    MotorControl[5].Motor_Start_Stop = 0;
                }
                if(errormotor6 > 10)
                {
                    MotorControl[6].Motor_Start_Stop = 0;
                }
            }
        }
        if(wGlobal_Flags != prewGlobal_Flags)
        {
            static u32 filterwGlobal_Flags;
            if(wGlobal_Flags!=0&&wGlobal_Flags!=filterwGlobal_Flags)
            {
                for(u8 i=9; i>0; i--)                        //add by diamond 2021.1125
                {
                    Err_codeHis[i] = Err_codeHis[i-1];    //�����¼���˳��������飬�����õı���
                }
                for(u8 i=9; i>0; i--)
                {
                    Err_codeTick[i] = Err_codeTick[i-1];    //�¼������
                }
                Err_codeHis[0] = wGlobal_Flags;
                Err_codeTick[0] = uwTick;
                Flash_Writesign =1;    //FLASHд���־λ
                filterwGlobal_Flags = wGlobal_Flags;
            }
            prewGlobal_Flags = wGlobal_Flags;
            sendPDOevent(&CANopen_Drive);                  //������仯�Զ��ϱ�
        }
        if(BitTst(SysCLC.SysTimFlag,A1mSec))		/*1MS	*/
        {
            BitClr(SysCLC.SysTimFlag,A1mSec);
            BitInv(SysCLC.SysTimFlag,A2mSec);
            Sys_Time();

            if(BitTst(SysCLC.SysTimFlag,A2mSec))	/*2MS	*/
            {

                Labview_uart();
//								Startagreement();						/* ��λ��ͨ�� */
//		      	    QuickRead();								/* �Զ��ϱ����� */
                if(strCapture.ucFinishFlag == 1)		/* ������ϼ�� */
                {
                    Suction_motor_errhandle(strCapture.uDuty_ratio);
                    strCapture.ucFinishFlag = 0;
                }
                Motor_Fault_Clear();//�������
//								else if(motorset==1)
//								{
//									FAN_F_R_SET;
//								}
//								else if(motorset==2)
//									FAN_F_R_RESET;
            }
        }


        if(BitTst(SysCLC.SysTimFlag,A10mSec))		/*10MS	*/
        {
            BitClr(SysCLC.SysTimFlag,A10mSec);
            BitInv(SysCLC.SysTimFlag,A20mSec);
//            DisplayErrLed();					/* ״̬�� */
//            ReturnError1();						/* �Զ��ϱ����� */

            if(BitTst(SysCLC.SysTimFlag,A20mSec))	/*20MS	*/
            {

            }
        }
        if(BitTst(SysCLC.SysTimFlag,A250mSec))	/*250MS	*/
        {
            BitClr(SysCLC.SysTimFlag,A250mSec);
            BitInv(SysCLC.SysTimFlag,A500mSec);

            if(MotorControl[5].Hall.HallState == 0||MotorControl[5].Hall.HallState == 7 )
            {
                MC_SetFault(HALL5_SENSOR_ERR);
                MotorControl[5].Motor_Start_Stop = DISABLE;
            }
            if(MotorControl[6].Hall.HallState == 0||MotorControl[6].Hall.HallState == 7 )
            {
                MC_SetFault(HALL6_SENSOR_ERR);
                MotorControl[6].Motor_Start_Stop = DISABLE;
            }
            Flash_WriteCheck();

        }
        /* USER CODE BEGIN 3 */
    }
    /* USER CODE END 3 */
}

/*------------------------------------------------
Function:ʹ���ⲿʱ��25M
Input   :NoOutput  :No
Explain :No
------------------------------------------------*/
void SystemClock_Config(void)
{
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

    /** Configure the main internal regulator output voltage
    */
    __HAL_RCC_PWR_CLK_ENABLE();
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
    /** Initializes the RCC Oscillators according to the specified parameters
    * in the RCC_OscInitTypeDef structure.
    */
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
    RCC_OscInitStruct.HSEState = RCC_HSE_ON;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
    RCC_OscInitStruct.PLL.PLLM = 25;
    RCC_OscInitStruct.PLL.PLLN = 336;
    RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
    RCC_OscInitStruct.PLL.PLLQ = 7;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
    {
        Error_Handler();
    }
    /** Initializes the CPU, AHB and APB buses clocks
    */
    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                                  |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
    {
        Error_Handler();
    }

    // HAL_RCC_GetHCLKFreq()/1000    1ms?D??��?��?
    // HAL_RCC_GetHCLKFreq()/100000	 10us?D??��?��?
    // HAL_RCC_GetHCLKFreq()/1000000 1us?D??��?��?
    HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/2000);                 // ????2��???��?�̨�3��?��e?������?��
    /* ?�̨�3��?��e?������?�¨���?��?�� */
    HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

    /* ?�̨�3��?��e?������?��?D??��??��?????? */
    HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);

}
/*------------------------------------------------
Function:ʹ���ڲ�ʱ��16M
Input   :No
Output  :No
Explain :No
------------------------------------------------*/
//void SystemClock_Config(void)
//{
//    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
//    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

//    /** Configure the main internal regulator output voltage
//    */
//    __HAL_RCC_PWR_CLK_ENABLE();
//    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
//    /** Initializes the RCC Oscillators according to the specified parameters
//    * in the RCC_OscInitTypeDef structure.
//    */
//    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
//    RCC_OscInitStruct.HSIState = RCC_HSI_ON;
//    RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
//    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
//    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
//    RCC_OscInitStruct.PLL.PLLM = 16;
//    RCC_OscInitStruct.PLL.PLLN = 336;
//    RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
//    RCC_OscInitStruct.PLL.PLLQ = 4;
//    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
//    {
//        Error_Handler();
//    }
//    /** Initializes the CPU, AHB and APB buses clocks
//    */
//    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
//                                  |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
//    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
//    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
//    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
//    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

//    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
//    {
//        Error_Handler();
//    }

//    // HAL_RCC_GetHCLKFreq()/1000    1ms?D??��?��?
//    // HAL_RCC_GetHCLKFreq()/100000	 10us?D??��?��?
//    // HAL_RCC_GetHCLKFreq()/1000000 1us?D??��?��?
//    HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/2000);                 // ????2��???��?�̨�3��?��e?������?��
//    /* ?�̨�3��?��e?������?�¨���?��?�� */
//    HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

//    /* ?�̨�3��?��e?������?��?D??��??��?????? */
//    HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);

//}
/*------------------------------------------------
Function:�ж����ȼ�����
Input   :No
Output  :No
Explain :�ر�ע�⣺��Ҫ����શ�ʱ�����ȼ��ߣ������������ӻ�ͬʱ����������߹���ʧЧ���Ῠ�� if(Timeout != HAL_MAX_DELAY)
------------------------------------------------*/
static void MX_NVIC_Init(void)
{
    HAL_NVIC_SetPriority(SysTick_IRQn, 2, 0);
    /* EXTI interrupt init*/
    HAL_NVIC_SetPriority(EXTI0_IRQn, 3, 0);  //�Ƹ�hall
    HAL_NVIC_EnableIRQ(EXTI0_IRQn);

//    HAL_NVIC_SetPriority(ADC_IRQn, 1, 0); //ģ�⿴�Ź�
//    HAL_NVIC_EnableIRQ(ADC_IRQn);
    HAL_NVIC_SetPriority(EXTI1_IRQn, 3, 0);  //�Ƹ�hall
    HAL_NVIC_EnableIRQ(EXTI1_IRQn);

    /* EXTI2_IRQn interrupt configuration */
    HAL_NVIC_SetPriority(EXTI2_IRQn, 3, 0);  //�Ƹ�hall
    HAL_NVIC_EnableIRQ(EXTI2_IRQn);
    /* EXTI3_IRQn interrupt configuration */
    HAL_NVIC_SetPriority(EXTI3_IRQn, 3, 0);  //�Ƹ�hall
    HAL_NVIC_EnableIRQ(EXTI3_IRQn);
    /* EXTI4_IRQn interrupt configuration */
    HAL_NVIC_SetPriority(EXTI4_IRQn, 0, 0);  //���6ɲ��
    HAL_NVIC_EnableIRQ(EXTI4_IRQn);
    /* EXTI9_5_IRQn interrupt configuration */
    HAL_NVIC_SetPriority(EXTI9_5_IRQn, 1, 0);
    HAL_NVIC_DisableIRQ(EXTI9_5_IRQn);

    /* EXTI15_10_IRQn interrupt configuration */
    HAL_NVIC_SetPriority(EXTI15_10_IRQn, 1, 0);
    HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);


    /* TIM1_BRK_TIM9_IRQn interrupt configuration TIM1_UP_TIM10_IRQn */  //���5ɲ��
    HAL_NVIC_SetPriority(TIM1_BRK_TIM9_IRQn, 0, 0);    //���5ɲ��
    HAL_NVIC_EnableIRQ(TIM1_BRK_TIM9_IRQn);

    HAL_NVIC_SetPriority(TIM1_UP_TIM10_IRQn, 1, 0);
    HAL_NVIC_EnableIRQ(TIM1_UP_TIM10_IRQn);

    HAL_NVIC_SetPriority(TIM7_IRQn, 0, 0); //��ˢ���� �͵��7��8����
    HAL_NVIC_EnableIRQ(TIM7_IRQn);

    /* USART3_IRQn interrupt configuration */
    HAL_NVIC_SetPriority(USART3_IRQn, 4, 0);
    HAL_NVIC_EnableIRQ(USART3_IRQn);

    /* DMA2_Stream0_IRQn interrupt configuration */
    HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 3, 0);
    HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);
    /* CAN2_RX0_IRQn interrupt configuration */
    HAL_NVIC_SetPriority(CAN2_RX0_IRQn, 3, 0);
    HAL_NVIC_EnableIRQ(CAN2_RX0_IRQn);

    HAL_NVIC_SetPriority(TIM6_DAC_IRQn,3, 0);
    HAL_NVIC_EnableIRQ(TIM6_DAC_IRQn);
	
    HAL_NVIC_SetPriority(TIM5_IRQn, 0, 1); //��ռ���ȼ�
    HAL_NVIC_EnableIRQ(TIM5_IRQn);
	
	HAL_NVIC_SetPriority(TIM8_BRK_TIM12_IRQn,0,1);
	 HAL_NVIC_EnableIRQ(TIM8_BRK_TIM12_IRQn);
}
/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

    /* USER CODE BEGIN ADC1_Init 0 */

    /* USER CODE END ADC1_Init 0 */

    ADC_ChannelConfTypeDef sConfig = {0};
    ADC_AnalogWDGConfTypeDef AnalogWDGConfig;
    /* USER CODE BEGIN ADC1_Init 1 */

    /* USER CODE END ADC1_Init 1 */
    /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
    */
    hadc1.Instance = ADC1;
    hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
    hadc1.Init.Resolution = ADC_RESOLUTION_12B;
    hadc1.Init.ScanConvMode = ENABLE;
    hadc1.Init.ContinuousConvMode = ENABLE;
    hadc1.Init.DiscontinuousConvMode = DISABLE;
    hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
    hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
    hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
    hadc1.Init.NbrOfConversion = 12;
    hadc1.Init.DMAContinuousRequests = ENABLE;
    hadc1.Init.EOCSelection = ADC_EOC_SEQ_CONV;
    if (HAL_ADC_Init(&hadc1) != HAL_OK)
    {
        Error_Handler();
    }
//  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
//  */
//  sConfig.Channel = ADC_CHANNEL_10;
//  sConfig.Rank = 1;
//  sConfig.SamplingTime = ADC_SAMPLETIME_28CYCLES;
//  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
//  {
//    Error_Handler();
//  }
//  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
//  */
//  sConfig.Channel = ADC_CHANNEL_0;
//  sConfig.Rank = 2;
//  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
//  {
//    Error_Handler();
//  }
//  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
//  */
//  sConfig.Rank = 3;
//  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
//  {
//    Error_Handler();
//  }
//  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
//  */
//  sConfig.Rank = 4;
//  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
//  {
//    Error_Handler();
//  }
//  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
//  */
//  sConfig.Rank = 5;
//  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
//  {
//    Error_Handler();
//  }
//  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
//  */
//  sConfig.Rank = 6;
//  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
//  {
//    Error_Handler();
//  }
//  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
//  */
//  sConfig.Rank = 7;
//  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
//  {
//    Error_Handler();
//  }
//  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
//  */
//  sConfig.Rank = 8;
//  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
//  {
//    Error_Handler();
//  }
//  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
//  */
//  sConfig.Rank = 9;
//  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
//  {
//    Error_Handler();
//  }
//  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
//  */
//  sConfig.Rank = 10;
//  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
//  {
//    Error_Handler();
//  }
//  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
//  */
//  sConfig.Rank = 11;
//  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
//  {
//    Error_Handler();
//  }
//  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
//  */
//  sConfig.Rank = 12;
//  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
//  {
//    Error_Handler();
//  }
    /* USER CODE BEGIN ADC1_Init 2 */

    sConfig.Channel = ADC_CHANNEL_0;  //���3��������
    sConfig.Rank = 1;
    sConfig.SamplingTime = ADC_SAMPLETIME_28CYCLES;
    if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
    {
        Error_Handler();
    }

    sConfig.Channel = ADC_CHANNEL_1;   //���4��������
    sConfig.Rank = 2;
    if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
    {
        Error_Handler();
    }

    sConfig.Channel = ADC_CHANNEL_2;  //���5��������
    sConfig.Rank = 3;
    if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
    {
        Error_Handler();
    }

    /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
    */
    sConfig.Channel = ADC_CHANNEL_2;    //ADC_CHANNEL_3 ���5�¶�
    sConfig.Rank = 4;
    if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
    {
        Error_Handler();
    }
    /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
    */
    sConfig.Channel = ADC_CHANNEL_4;   //���6��������
    sConfig.Rank = 5;
    if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
    {
        Error_Handler();
    }
    /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
    */
//    sConfig.Channel = ADC_CHANNEL_5;
//    sConfig.Rank = 5;
//    if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
//    {
//        Error_Handler();
//    }
    /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
    */
    sConfig.Channel = ADC_CHANNEL_4; //ADC_CHANNEL_6  ���6�¶�
    sConfig.Rank = 6;
    if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
    {
        Error_Handler();
    }
    /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
    */

//    sConfig.Channel = ADC_CHANNEL_8;
//    sConfig.Rank = 7;
//    if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
//    {
//        Error_Handler();
//    }

//    sConfig.Channel = ADC_CHANNEL_9;
//    sConfig.Rank = 8;
//    sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
//    if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
//    {
//        Error_Handler();
//    }

    sConfig.Channel = ADC_CHANNEL_10;  //�Ƹ�1��������
    sConfig.Rank = 7;
    if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
    {
        Error_Handler();
    }

    /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
    */
    sConfig.Channel = ADC_CHANNEL_11;//�Ƹ�2��������
    sConfig.Rank = 8;
    if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
    {
        Error_Handler();
    }
    /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
    */
    sConfig.Channel = ADC_CHANNEL_12; //�Ƹ�3��������
    sConfig.Rank = 9;
    if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
    {
        Error_Handler();
    }
    /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
    */
    sConfig.Channel = ADC_CHANNEL_13;  //�Ƹ�4��������
    sConfig.Rank = 10;
    if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
    {
        Error_Handler();
    }
    /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
    */
    sConfig.Channel = ADC_CHANNEL_14;     //ĸ�ߵ�ѹ�ɼ�
    sConfig.Rank = 11;
    if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
    {
        Error_Handler();
    }
    /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
    */

    sConfig.Channel = ADC_CHANNEL_14; //ADC_CHANNEL_15 Ϊ����¶�
    sConfig.Rank = 12;
    if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
    {
        Error_Handler();
    }
    ADC1->CR1 |= 0x06000 ;
    AnalogWDGConfig.WatchdogMode = ADC_ANALOGWATCHDOG_ALL_REG;
    AnalogWDGConfig.HighThreshold =ADC1AnalogWDGHighThreshold; /* ������ˢADC����ֵ������Կ��Ź���Ӧ������ˢ�����������������126��Ӧ1A  ����Ϊ3500��27A���� */
    AnalogWDGConfig.LowThreshold = 0;  /* ��������ֵΪ0.41V   4095*1/8=511  3.3/4095*511=0.41V*/
    AnalogWDGConfig.ITMode = ENABLE;
    if(HAL_ADC_AnalogWDGConfig(&hadc1, &AnalogWDGConfig)!=HAL_OK)
    {
        Error_Handler();
    }

    /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief CAN2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_CAN2_Init(void)
{

    /* USER CODE BEGIN CAN2_Init 0 */

    /* USER CODE END CAN2_Init 0 */

    /* USER CODE BEGIN CAN2_Init 1 */

    /* USER CODE END CAN2_Init 1 */
    /* USER CODE BEGIN CAN2_Init 0 */
    CAN_FilterConfTypeDef  sFilterConfig;
    /* USER CODE END CAN2_Init 0 */

    /* USER CODE BEGIN CAN2_Init 1 */

    /* USER CODE END CAN2_Init 1 */
    hcan2.Instance = CAN2;
    hcan2.pTxMsg = &TxMessage;
    hcan2.pRxMsg = &RxMessage;

    hcan2.Init.Prescaler = 6;          // BTR-BRP ??????  ???????????? 42/(1+6+7)/3=1Mbps
    hcan2.Init.Mode = CAN_MODE_NORMAL;
    hcan2.Init.SJW = CAN_SJW_1TQ;      // BTR-SJW ???????? 1?????
    hcan2.Init.BS1 = CAN_BS1_6TQ;      // BTR-TS1 ???1 ???6?????
    hcan2.Init.BS2 = CAN_BS2_7TQ;      // BTR-TS1 ???2 ???7?????
    hcan2.Init.TTCM = DISABLE;         // MCR-TTCM  ????????????
    hcan2.Init.ABOM = ENABLE;          // MCR-ABOM  ??????
    hcan2.Init.AWUM = ENABLE;          // MCR-AWUM  ????????
    hcan2.Init.NART = DISABLE;         // MCR-NART  ????????	  DISABLE-????
    hcan2.Init.RFLM = DISABLE;         // MCR-RFLM  ??FIFO ????  DISABLE-?????????????
    hcan2.Init.TXFP = DISABLE;         // MCR-TXFP  ??FIFO??? DISABLE-???????????
    if (HAL_CAN_Init(&hcan2) != HAL_OK)
    {
        Error_Handler();
    }
    /* USER CODE BEGIN CAN2_Init 2 */
    /*CAN??????*/
    sFilterConfig.FilterNumber = 14;                    //????14
    sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;  //???????????
    sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT; //????????32??
    /* ????????????????????????,??ID?????????,???,???FIFO0? */

    sFilterConfig.FilterIdHigh         = 0X0000;//(((uint32_t)0x1314<<3)&0xFFFF0000)>>16;				//????ID??
    sFilterConfig.FilterIdLow          = 0X0000;//(((uint32_t)0x1314<<3)|CAN_ID_EXT|CAN_RTR_DATA)&0xFFFF; //????ID??
    sFilterConfig.FilterMaskIdHigh     = 0X0000;//0xFFFF;			//????16???????
    sFilterConfig.FilterMaskIdLow      = 0X0000;//0xFFFF;			//????16???????
    sFilterConfig.FilterFIFOAssignment = CAN_FILTER_FIFO0;           //???????FIFO 0
    sFilterConfig.FilterActivation = ENABLE;          //?????
    sFilterConfig.BankNumber = 14;
    HAL_CAN_ConfigFilter(&hcan2, &sFilterConfig);
    /* USER CODE BEGIN CAN2_Init 2 */

    /* USER CODE END CAN2_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

    /* USER CODE BEGIN TIM1_Init 0 */

    /* USER CODE END TIM1_Init 0 */

    TIM_ClockConfigTypeDef sClockSourceConfig = {0};
    TIM_MasterConfigTypeDef sMasterConfig = {0};
    TIM_OC_InitTypeDef sConfigOC = {0};
    TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

    /* USER CODE BEGIN TIM1_Init 1 */

    /* USER CODE END TIM1_Init 1 */
    htim1.Instance = TIM1;
    htim1.Init.Prescaler = 0;
    htim1.Init.CounterMode = TIM_COUNTERMODE_CENTERALIGNED1;
    htim1.Init.Period = 8400;
    htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim1.Init.RepetitionCounter = 1;
    htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
    {
        Error_Handler();
    }
    sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
    if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
    {
        Error_Handler();
    }
    if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
    {
        Error_Handler();
    }
    sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
    sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
    if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
    {
        Error_Handler();
    }
    sConfigOC.OCMode = TIM_OCMODE_PWM1;
    sConfigOC.Pulse = 0;
    sConfigOC.OCPolarity = TIM_OCPOLARITY_LOW;
    sConfigOC.OCNPolarity = TIM_OCPOLARITY_LOW;
    sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
    sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
    sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
    if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
    {
        Error_Handler();
    }
    if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
    {
        Error_Handler();
    }
    if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
    {
        Error_Handler();
    }
    sConfigOC.OCPolarity = TIM_OCPOLARITY_LOW;
    if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
    {
        Error_Handler();
    }
    sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_ENABLE;
    sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_ENABLE;
    sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
    sBreakDeadTimeConfig.DeadTime = 200;
    sBreakDeadTimeConfig.BreakState = TIM_BREAK_ENABLE;
    sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_LOW;
    sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_ENABLE;
    if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
    {
        Error_Handler();
    }
    /* USER CODE BEGIN TIM1_Init 2 */

    /* USER CODE END TIM1_Init 2 */
    HAL_TIM_MspPostInit(&htim1);

    HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_1);
    HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_2);
    HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_3);
    __HAL_TIM_ENABLE_IT(&htim1, TIM_IT_BREAK);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
    TIM1->CCER = 0X3ddd;
    /* USER CODE END TIM1_Init 2 */
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

    TIM_ClockConfigTypeDef sClockSourceConfig = {0};
    TIM_MasterConfigTypeDef sMasterConfig = {0};
    TIM_OC_InitTypeDef sConfigOC = {0};

    /* USER CODE BEGIN TIM2_Init 1 */

    /* USER CODE END TIM2_Init 1 */
    htim2.Instance = TIM2;
    htim2.Init.Prescaler = 0;
    htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim2.Init.Period = 8400;
    htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
    {
        Error_Handler();
    }
    sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
    if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
    {
        Error_Handler();
    }
    if (HAL_TIM_OC_Init(&htim2) != HAL_OK)
    {
        Error_Handler();
    }
    sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
    sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
    if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
    {
        Error_Handler();
    }
    sConfigOC.OCMode = TIM_OCMODE_PWM1;
    sConfigOC.Pulse = 0;
    sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
    sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
    if (HAL_TIM_OC_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
    {
        Error_Handler();
    }
    sConfigOC.Pulse = 0;
    if (HAL_TIM_OC_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
    {
        Error_Handler();
    }
    sConfigOC.Pulse = 0;
    if (HAL_TIM_OC_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
    {
        Error_Handler();
    }
    sConfigOC.Pulse = 0;
    if (HAL_TIM_OC_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
    {
        Error_Handler();
    }
    /* USER CODE BEGIN TIM2_Init 2 */

    /* USER CODE END TIM2_Init 2 */
    HAL_TIM_MspPostInit(&htim2);

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
    TIM_MasterConfigTypeDef sMasterConfig = {0};
    TIM_OC_InitTypeDef sConfigOC = {0};

    /* USER CODE BEGIN TIM3_Init 1 */

    /* USER CODE END TIM3_Init 1 */
    htim3.Instance = TIM3;
    htim3.Init.Prescaler = 0;
    htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim3.Init.Period = 8400;
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
    if (HAL_TIM_OC_Init(&htim3) != HAL_OK)
    {
        Error_Handler();
    }
    sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
    sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
    if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
    {
        Error_Handler();
    }
    sConfigOC.OCMode = TIM_OCMODE_PWM1;
    sConfigOC.Pulse = 0;
    sConfigOC.Pulse = 8400;
    sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
    sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
    if (HAL_TIM_OC_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
    {
        Error_Handler();
    }
    if (HAL_TIM_OC_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
    {
        Error_Handler();
    }
    sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH; //��ˢ
    if (HAL_TIM_OC_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
    {
        Error_Handler();
    }
    sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH; //���
    if (HAL_TIM_OC_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
    {
        Error_Handler();
    }
    /* USER CODE BEGIN TIM3_Init 2 */

    /* USER CODE END TIM3_Init 2 */
    HAL_TIM_MspPostInit(&htim3);

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

    /* USER CODE BEGIN TIM4_Init 0 */

    /* USER CODE END TIM4_Init 0 */

    TIM_ClockConfigTypeDef sClockSourceConfig = {0};
    TIM_MasterConfigTypeDef sMasterConfig = {0};
    TIM_OC_InitTypeDef sConfigOC = {0};

    /* USER CODE BEGIN TIM4_Init 1 */

    /* USER CODE END TIM4_Init 1 */
    htim4.Instance = TIM4;
    htim4.Init.Prescaler = 0;
    htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim4.Init.Period = 8400;
    htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
    {
        Error_Handler();
    }
    sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
    if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
    {
        Error_Handler();
    }
    if (HAL_TIM_OC_Init(&htim4) != HAL_OK)
    {
        Error_Handler();
    }
    sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
    sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
    if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
    {
        Error_Handler();
    }
    sConfigOC.OCMode = TIM_OCMODE_PWM1;
    sConfigOC.Pulse = 0;
    sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
    sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
    if (HAL_TIM_OC_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
    {
        Error_Handler();
    }
    sConfigOC.Pulse = 0;
    if (HAL_TIM_OC_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
    {
        Error_Handler();
    }
    sConfigOC.Pulse =0;
    if (HAL_TIM_OC_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
    {
        Error_Handler();
    }
    sConfigOC.Pulse = 0;
    if (HAL_TIM_OC_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
    {
        Error_Handler();
    }
    /* USER CODE BEGIN TIM4_Init 2 */

    /* USER CODE END TIM4_Init 2 */
    HAL_TIM_MspPostInit(&htim4);

}

static void MX_TIM5_Init(void)        //����0.1ms
{

    /* USER CODE BEGIN TIM5_Init 0 */

    /* USER CODE END TIM5_Init 0 */

    TIM_ClockConfigTypeDef sClockSourceConfig;
    TIM_MasterConfigTypeDef sMasterConfig;
	TIM_OC_InitTypeDef sConfigOC = {0};

    /* USER CODE BEGIN TIM5_Init 1 */

    /* USER CODE END TIM5_Init 1 */
    htim5.Instance = TIM5;
    htim5.Init.Prescaler = 99;
    htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
		htim5.Init.Period = 63000;
    htim5.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim5.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    if (HAL_TIM_Base_Init(&htim5) != HAL_OK)
    {
        Error_Handler();
    }
	

    /* USER CODE BEGIN TIM5_Init 2 */
	sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
    HAL_TIM_ConfigClockSource(&htim5, &sClockSourceConfig);

    sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
    sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
    HAL_TIMEx_MasterConfigSynchronization(&htim5, &sMasterConfig);
	
	sConfigOC.OCMode = TIM_OCMODE_TIMING;//TIM_OCMODE_TIMING;
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;               
	sConfigOC.Pulse = 0;                         
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;  
	 /* ����CH1Ϊ�Ƚ����ģʽ */
    HAL_TIM_OC_ConfigChannel(&htim5, &sConfigOC, TIM_CHANNEL_1);
    /* USER CODE END TIM5_Init 2 */
}



/**
  * @brief TIM7 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM7_Init(void)
{

    /* USER CODE BEGIN TIM7_Init 0 */

    /* USER CODE END TIM7_Init 0 */

    TIM_MasterConfigTypeDef sMasterConfig = {0};

    /* USER CODE BEGIN TIM7_Init 1 */

    /* USER CODE END TIM7_Init 1 */
    htim7.Instance = TIM7;
    htim7.Init.Prescaler = 83;
    htim7.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim7.Init.Period = 65535;
    htim7.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    if (HAL_TIM_Base_Init(&htim7) != HAL_OK)
    {
        Error_Handler();
    }
    sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
    sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
    if (HAL_TIMEx_MasterConfigSynchronization(&htim7, &sMasterConfig) != HAL_OK)
    {
        Error_Handler();
    }
    /* USER CODE BEGIN TIM7_Init 2 */

    /* USER CODE END TIM7_Init 2 */

}

/**
  * @brief TIM8 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM8_Init(void)
{

    /* USER CODE BEGIN TIM8_Init 0 */

    /* USER CODE END TIM8_Init 0 */

    TIM_ClockConfigTypeDef sClockSourceConfig = {0};
    TIM_MasterConfigTypeDef sMasterConfig = {0};
    TIM_OC_InitTypeDef sConfigOC = {0};
    TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

    /* USER CODE BEGIN TIM8_Init 1 */

    /* USER CODE END TIM8_Init 1 */
    htim8.Instance = TIM8;
    htim8.Init.Prescaler = 0;
    htim8.Init.CounterMode = TIM_COUNTERMODE_CENTERALIGNED1;
    htim8.Init.Period = 8400;
    htim8.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim8.Init.RepetitionCounter = 1;
    htim8.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    if (HAL_TIM_Base_Init(&htim8) != HAL_OK)
    {
        Error_Handler();
    }
    sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
    if (HAL_TIM_ConfigClockSource(&htim8, &sClockSourceConfig) != HAL_OK)
    {
        Error_Handler();
    }
    if (HAL_TIM_PWM_Init(&htim8) != HAL_OK)
    {
        Error_Handler();
    }
    sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
    sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
    if (HAL_TIMEx_MasterConfigSynchronization(&htim8, &sMasterConfig) != HAL_OK)
    {
        Error_Handler();
    }
    sConfigOC.OCMode = TIM_OCMODE_PWM1;
    sConfigOC.Pulse = 0;
    sConfigOC.OCPolarity = TIM_OCPOLARITY_LOW;
    sConfigOC.OCNPolarity = TIM_OCPOLARITY_LOW;
    sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
    sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
    sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
    if (HAL_TIM_PWM_ConfigChannel(&htim8, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
    {
        Error_Handler();
    }
    if (HAL_TIM_PWM_ConfigChannel(&htim8, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
    {
        Error_Handler();
    }
    if (HAL_TIM_PWM_ConfigChannel(&htim8, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
    {
        Error_Handler();
    }
    sConfigOC.OCPolarity = TIM_OCPOLARITY_LOW;
    if (HAL_TIM_PWM_ConfigChannel(&htim8, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
    {
        Error_Handler();
    }
    sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_ENABLE;
    sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_ENABLE;
    sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_1;
    sBreakDeadTimeConfig.DeadTime = 200;//
    sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
    sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
    sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
    if (HAL_TIMEx_ConfigBreakDeadTime(&htim8, &sBreakDeadTimeConfig) != HAL_OK)
    {
        Error_Handler();
    }
    /* USER CODE BEGIN TIM8_Init 2 */

    /* USER CODE END TIM8_Init 2 */
    HAL_TIM_MspPostInit(&htim8);

    HAL_TIMEx_PWMN_Start(&htim8, TIM_CHANNEL_1);
    HAL_TIMEx_PWMN_Start(&htim8, TIM_CHANNEL_2);
    HAL_TIMEx_PWMN_Start(&htim8, TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_4);
    TIM8->CCER = 0X3ddd;;

}
/**
  * @brief TIM9 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM9_Init(void)
{

    /* USER CODE BEGIN TIM9_Init 0 */

    /* USER CODE END TIM9_Init 0 */
    TIM_ClockConfigTypeDef sClockSourceConfig;
    TIM_MasterConfigTypeDef sMasterConfig;
    TIM_IC_InitTypeDef sConfigIC = {0};

    /* USER CODE BEGIN TIM9_Init 1 */

    /* USER CODE END TIM9_Init 1 */
    htim9.Instance = TIM9;
    htim9.Init.Prescaler = TIM9_PRESCALER;
    htim9.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim9.Init.Period = 0xFFFF;
    htim9.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim9.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    if (HAL_TIM_IC_Init(&htim9) != HAL_OK)
    {
        Error_Handler();
    }
    sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;//ѡ���ڲ�ʱ��
    HAL_TIM_ConfigClockSource(&htim9, &sClockSourceConfig);

    sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
    sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
    HAL_TIMEx_MasterConfigSynchronization(&htim9, &sMasterConfig);

    sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
    sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
    sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
    sConfigIC.ICFilter = 0;
    if (HAL_TIM_IC_ConfigChannel(&htim9, &sConfigIC, TIM_CHANNEL_1) != HAL_OK)
    {
        Error_Handler();
    }
    if (HAL_TIM_IC_ConfigChannel(&htim9, &sConfigIC, TIM_CHANNEL_2) != HAL_OK)
    {
        Error_Handler();
    }
    /* USER CODE BEGIN TIM9_Init 2 */

    /* USER CODE END TIM9_Init 2 */

}

/**
  * @brief TIM10 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM10_Init(void)
{


    /* USER CODE BEGIN TIM10_Init 0 */

    /* USER CODE END TIM10_Init 0 */

    TIM_OC_InitTypeDef sConfigOC = {0};

    /* USER CODE BEGIN TIM10_Init 1 */

    /* USER CODE END TIM10_Init 1 */
    htim10.Instance = TIM10;
    htim10.Init.Prescaler = 0;
    htim10.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim10.Init.Period = 16800;
    htim10.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim10.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    if (HAL_TIM_Base_Init(&htim10) != HAL_OK)
    {
        Error_Handler();
    }
    if (HAL_TIM_PWM_Init(&htim10) != HAL_OK)
    {
        Error_Handler();
    }
    sConfigOC.OCMode = TIM_OCMODE_PWM1;
    sConfigOC.Pulse = 16800;
    sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
    sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
    if (HAL_TIM_PWM_ConfigChannel(&htim10, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
    {
        Error_Handler();
    }
    /* USER CODE BEGIN TIM10_Init 2 */

    /* USER CODE END TIM10_Init 2 */
    HAL_TIM_MspPostInit(&htim10);

}

/**
  * @brief TIM11 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM11_Init(void)
{


    /* USER CODE BEGIN TIM10_Init 0 */

    /* USER CODE END TIM10_Init 0 */

    TIM_OC_InitTypeDef sConfigOC = {0};

    /* USER CODE BEGIN TIM10_Init 1 */

    /* USER CODE END TIM10_Init 1 */
    htim11.Instance = TIM11;
    htim11.Init.Prescaler = 0;
    htim11.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim11.Init.Period = 16800;
    htim11.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim11.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    if (HAL_TIM_Base_Init(&htim11) != HAL_OK)
    {
        Error_Handler();
    }
    if (HAL_TIM_PWM_Init(&htim11) != HAL_OK)
    {
        Error_Handler();
    }
    sConfigOC.OCMode = TIM_OCMODE_PWM1;
    sConfigOC.Pulse = 16800;
    sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
    sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
    if (HAL_TIM_PWM_ConfigChannel(&htim11, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
    {
        Error_Handler();
    }
    /* USER CODE BEGIN TIM10_Init 2 */

    /* USER CODE END TIM10_Init 2 */
    HAL_TIM_MspPostInit(&htim11);

}

static void MX_TIM12_Init(void)        //����0.1ms
{

    /* USER CODE BEGIN TIM5_Init 0 */

    /* USER CODE END TIM5_Init 0 */

    TIM_ClockConfigTypeDef sClockSourceConfig;
    TIM_MasterConfigTypeDef sMasterConfig;
	TIM_OC_InitTypeDef sConfigOC = {0};

    /* USER CODE BEGIN TIM5_Init 1 */

    /* USER CODE END TIM5_Init 1 */
    htim12.Instance = TIM12;
    htim12.Init.Prescaler = 99;
    htim12.Init.CounterMode = TIM_COUNTERMODE_UP;
		htim12.Init.Period = 63000;
    htim12.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim12.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    if (HAL_TIM_Base_Init(&htim12) != HAL_OK)
    {
        Error_Handler();
    }
	

    /* USER CODE BEGIN TIM5_Init 2 */
	sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
    HAL_TIM_ConfigClockSource(&htim12, &sClockSourceConfig);

    sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
    sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
    HAL_TIMEx_MasterConfigSynchronization(&htim12, &sMasterConfig);
	
	sConfigOC.OCMode = TIM_OCMODE_TIMING;//TIM_OCMODE_TIMING;
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;               
	sConfigOC.Pulse = 0;                         
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;  
	 /* ����CH1Ϊ�Ƚ����ģʽ */
    HAL_TIM_OC_ConfigChannel(&htim12, &sConfigOC, TIM_CHANNEL_1);
    /* USER CODE END TIM5_Init 2 */
}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
//static void MX_USART3_UART_Init(void)
//{

//    /* USER CODE BEGIN USART3_Init 0 */

//    /* USER CODE END USART3_Init 0 */

//    /* USER CODE BEGIN USART3_Init 1 */

//    /* USER CODE END USART3_Init 1 */
//    huart3.Instance = USART3;
//    huart3.Init.BaudRate = 115200;
//    huart3.Init.WordLength = UART_WORDLENGTH_8B;
//    huart3.Init.StopBits = UART_STOPBITS_1;
//    huart3.Init.Parity = UART_PARITY_NONE;
//    huart3.Init.Mode = UART_MODE_TX_RX;
//    huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
//    huart3.Init.OverSampling = UART_OVERSAMPLING_16;
//    if (HAL_UART_Init(&huart3) != HAL_OK)
//    {
//        Error_Handler();
//    }
//    /* USER CODE BEGIN USART3_Init 2 */

//    /* USER CODE END USART3_Init 2 */

//}

//void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
//{
//    HAL_UART_Receive_IT(&huart3,aRxBuffer,1);

//}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

    /* DMA controller clock enable */
    __HAL_RCC_DMA2_CLK_ENABLE();

}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{

//  OneFilter(MotorControl[0].Current.FilterValue,ADC_ConvertedValue[4],3);
//  OneFilter(MotorControl[1].Current.FilterValue,ADC_ConvertedValue[5],3);
//  OneFilter(MotorControl[2].Current.FilterValue,ADC_ConvertedValue[12],3);
//  OneFilter(MotorControl[3].Current.FilterValue,ADC_ConvertedValue[13],3);
//  OneFilter(MotorControl[4].Current.FilterValue,ADC_ConvertedValue[6],3);
//  if(DMA_Transfer_Complete_Count%2==0)
//  {
//    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_7, GPIO_PIN_SET);//HAL_GPIO_TogglePin(GPIOE, GPIO_PIN_7);ADC_ConvertedValue
//  }
//  else
//  {
//    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_7, GPIO_PIN_RESET);
//  }
}
u32 Cnt_100us=0;                   //100us����
float DeepFilterValPer_A[2] = {1.08,1.14};
s16 MotorControl5_Current_DeepFilterVAL,MotorControl6_Current_DeepFilterVAL;

int32_t int32test = 0;
extern s32 BLDC_Time,Utime;
extern uint16_t Pre_Bldctime,UPeriod;
u32 avglecont;
u8 hallstatetemp;
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	
    s32 TempValue;
    DMA_Transfer_Complete_Count++;
    CanLoadRate.timer++;
    if(htim==(&htim1))
    {
        if(RPDO_SYNC_SIGN & RPDO_SYNC_READY)
        {
            RPDO_SYNC_SIGN &= ~RPDO_SYNC_READY;
            _RPOD_SyncEvent(&CANopen_Drive,pRpdoDirectPar,1);
        }
        Push_Motor_Cheek();	//�Ƹˣ���ˢ3��4���
        MotorStuckCheck();
        Motor5_Default_Phase_Cheek();//ȱ����
        Motor6_Default_Phase_Cheek();//ȱ����
        Cnt_100us++;
        if(Cnt_100us>4000000000)Cnt_100us=0;

        /*CANopen Sync Data Handler Begin*/
        //Normal sync - 4us  RPDO sync - 13us
        //Think Can FIFO Rec time and here
        MotorControl[0].Current.GetADCValue = ADC_ConvertedValue[6]-MotorControl[0].Current.offset;
        if(MotorControl[0].Current.GetADCValue< 0 ) MotorControl[0].Current.GetADCValue = 0;
        TempValue = (s32)((MotorControl[0].Current.GetADCValue - MotorControl[0].Current.FilterValue )>>4);
        MotorControl[0].Current.FilterValue = (u16)(TempValue + MotorControl[0].Current.FilterValue);

        //��??��1��?��?����ADC??2��?��
        MotorControl[1].Current.GetADCValue = ADC_ConvertedValue[7]-MotorControl[1].Current.offset;
        if(MotorControl[1].Current.GetADCValue < 0) MotorControl[1].Current.GetADCValue= 0;
        TempValue = (s32)((MotorControl[1].Current.GetADCValue - MotorControl[1].Current.FilterValue )>>4);
        MotorControl[1].Current.FilterValue = (u16)(TempValue + MotorControl[1].Current.FilterValue);


        //��??��2��?��?����ADC??2��?��
        MotorControl[2].Current.GetADCValue = ADC_ConvertedValue[8]-MotorControl[2].Current.offset;
        if(MotorControl[2].Current.GetADCValue < 0) MotorControl[2].Current.GetADCValue= 0;
        TempValue = (s32)((MotorControl[2].Current.GetADCValue - MotorControl[2].Current.FilterValue )>>4);
        MotorControl[2].Current.FilterValue = (u16)(TempValue + MotorControl[2].Current.FilterValue);


        //��??��3��?��?����ADC??2��?��
        MotorControl[3].Current.GetADCValue = ADC_ConvertedValue[0]-MotorControl[3].Current.offset;
        if(MotorControl[3].Current.GetADCValue < 0) MotorControl[3].Current.GetADCValue = 0;
        TempValue = (s32)((MotorControl[3].Current.GetADCValue - MotorControl[3].Current.FilterValue )>>4);
        MotorControl[3].Current.FilterValue = (u16)(TempValue + MotorControl[3].Current.FilterValue);
        //    MotorControl[3].Current.FilterValue -= offset[3];
        //    if(MotorControl[3].Current.FilterValue<0) MotorControl[3].Current.FilterValue= 0;

        //��??��4��?��?����ADC??2��?��
        MotorControl[4].Current.GetADCValue = ADC_ConvertedValue[1]-MotorControl[4].Current.offset;
        if(MotorControl[4].Current.GetADCValue <0)  MotorControl[4].Current.GetADCValue= 0;
        TempValue = (s32)((MotorControl[4].Current.GetADCValue - MotorControl[4].Current.FilterValue )>>4);
        MotorControl[4].Current.FilterValue = (u16)(TempValue + MotorControl[4].Current.FilterValue);


        //��??��5��?��?����ADC??2��?��
        MotorControl[5].Current.GetADCValue = ADC_ConvertedValue[2] - MotorControl[5].Current.offset;
        if(MotorControl[5].Current.GetADCValue <0 ) MotorControl[5].Current.GetADCValue = 0;
        TempValue = (s32)((MotorControl[5].Current.GetADCValue - MotorControl[5].Current.FilterValue )>>4);
        MotorControl[5].Current.FilterValue = (u16)(TempValue + MotorControl[5].Current.FilterValue);


        //��??��6��?��?����ADC??2��?��
        MotorControl[6].Current.GetADCValue = ADC_ConvertedValue[4] - MotorControl[6].Current.offset;
        if(MotorControl[6].Current.GetADCValue < 0) MotorControl[6].Current.GetADCValue = 0;
        TempValue = (s32)((MotorControl[6].Current.GetADCValue - MotorControl[6].Current.FilterValue )>>4);
        MotorControl[6].Current.FilterValue = (u16)(TempValue + MotorControl[6].Current.FilterValue);


        MotorControl[9].Current.GetADCValue = ADC_ConvertedValue[9]-MotorControl[9].Current.offset;
        if(MotorControl[9].Current.GetADCValue <0)  MotorControl[9].Current.GetADCValue = 0 ;
        TempValue = (s32)((MotorControl[9].Current.GetADCValue - MotorControl[9].Current.FilterValue )>>4);
        MotorControl[9].Current.FilterValue = (u16)(TempValue + MotorControl[9].Current.FilterValue);
        /*����˲�*/
        MotorControl[5].Current.DeepFilterVAL = (MotorControl[5].Current.FilterValue*FILTER_COEFFICIENT+(1-FILTER_COEFFICIENT)*MotorControl[5].Current.PreFilterVal);		//100	����1A
        MotorControl[6].Current.DeepFilterVAL = (MotorControl[6].Current.FilterValue*FILTER_COEFFICIENT+(1-FILTER_COEFFICIENT)*MotorControl[6].Current.PreFilterVal);//1.37f
        MotorControl[5].Current.PreFilterVal = MotorControl[5].Current.DeepFilterVAL;
        MotorControl[6].Current.PreFilterVal = MotorControl[6].Current.DeepFilterVAL;
//				MotorControl5_Current_DeepFilterVAL = MotorControl[5].Current.DeepFilterVAL/DeepFilterValPer_A[0];
//				MotorControl6_Current_DeepFilterVAL = MotorControl[6].Current.DeepFilterVAL/DeepFilterValPer_A[1];
        /*end*/
        VoltVar.AdBuf = ADC_ConvertedValue[10];
        TempValue = (s32)((VoltVar.AdBuf - VoltVar.BUS )>>4);
        VoltVar.BUS = (u16)(TempValue +VoltVar.BUS);
        if(DMA_Transfer_Complete_Count > 30000) //ƫ�õ�ѹУ׼����ʼ���ݲ��ȶ�����
        {
            if( MotorState == IDLE)
            {
                Voltage_offset_cali();
            }
        }

        if(MotorState == RUN)
        {
            //			MotorControl[5].PWM_Duty = PID_Regulator(MotorControl[5].Current.ADCValue_Ref,MotorControl[5].Current.FilterValue,&PID_Speed_InitStruct[0]);
            /***********************5����ˢ�·��ٶ�*****************************/
            if((MotorControl[5].Speed_Ref < 0 && MotorControl[5].PWM_Duty > 0) || \
                    (MotorControl[5].Speed_Ref >0 && MotorControl[5].PWM_Duty < 0) || MotorControl[5].Speed_Ref == 0 )
            {
                PID_Speed_InitStruct[0].wIntegral = 0;
                PID_Current_InitStructure[0].wIntegral = 0;
                MotorControl[5].PWM_Duty = 0;
            }
            else
            {
				
                if(MotorControl[5].Speed_Real<0) MotorControl[5].Current.DeepFilterVAL = -MotorControl[5].Current.DeepFilterVAL;
				if(MotorControl[5].PWM_Duty>1000&&MotorControl[5].Hall.Hall_Changecnt !=0)
				  {
					MotorControl[5].PWM_Duty = PWM_Change(MotorControl[5].Hall.Hall_Changecnt,MotorControl[5].PWM_Duty);
					MotorControl[5].Hall.Hall_Changecnt--;
				  }
                else 
		          {
                      MotorControl[5].PWM_Duty = PID_Regulator(MotorControl[5].Current.ADCValue_Ref,MotorControl[5].Current.DeepFilterVAL,&PID_Current_InitStructure[0]);
                  }
			  }
				  
            if(MotorControl[5].Motor_Start_Stop==ENABLE)
            {
                if(MotorControl[5].Fault_Flag == 0&&OverFlow_Cnt[0]<BREAK_CNT_MAX)
				{
                        SetMotorSpeed(5, MotorControl[5].PWM_Duty);
				}
                else if(OverFlow_Cnt[0]>=BREAK_CNT_MAX)
                {
                    MotorControl[5].Motor_Start_Stop = DISABLE;
                    MC_SetFault(MOTOR5_BREAK);
                }
                else MotorControl[5].Motor_Start_Stop = DISABLE;
            }
            else if(MotorControl[5].Motor_Start_Stop==DISABLE)
                SetMotorStop(5);
            /***********************5����ˢ�·��ٶ�end*****************************/

            /***********************6����ˢ�·��ٶ�*****************************/
            if((MotorControl[6].Speed_Ref < 0 && MotorControl[6].PWM_Duty > 0) || \
                    (MotorControl[6].Speed_Ref >0 && MotorControl[6].PWM_Duty < 0) || MotorControl[6].Speed_Ref == 0 )
            {

                PID_Speed_InitStruct[1].wIntegral = 0;
                PID_Current_InitStructure[1].wIntegral = 0;
                MotorControl[6].PWM_Duty = 0;
            }
            else
            {
                if(MotorControl[6].Speed_Real<0) MotorControl[6].Current.DeepFilterVAL = -MotorControl[6].Current.DeepFilterVAL;
                MotorControl[6].PWM_Duty = PID_Regulator(MotorControl[6].Current.ADCValue_Ref,MotorControl[6].Current.DeepFilterVAL,&PID_Current_InitStructure[1]);
            }
            if(MotorControl[6].Motor_Start_Stop==ENABLE)
            {
                if(MotorControl[6].Fault_Flag == 0&&OverFlow_Cnt[1]<BREAK_CNT_MAX)
                    SetMotorSpeed(6, MotorControl[6].PWM_Duty);
                else if(OverFlow_Cnt[1]>=BREAK_CNT_MAX)
                {
                    MotorControl[6].Motor_Start_Stop = DISABLE;
                    MC_SetFault(MOTOR6_BREAK);
                }
                else MotorControl[6].Motor_Start_Stop = DISABLE;
            }
            else if(MotorControl[6].Motor_Start_Stop==DISABLE)
                SetMotorStop(6);
        }
        /***********************6����ˢ�·��ٶ�end*****************************/
    }

    if(htim==(&htim7))
    {
        HALL_OVF_Counter++;
        if(HALL_OVF_Counter%4 == 0)
        {
            if(MotorControl[8].Motor_Start_Stop == ENABLE)
            {
                if(CH1TEMP>5||CH1TEMP==0)		//exclude error state
                {
                    MotorControl[8].Speed_Real  = CH1TEMP*228.88;	//�����ٶ�
                }
                else
                {
                    MotorControl[8].Speed_Real  =0;
                }
                CH1TEMP = 0;
            }

            if(MotorControl[7].Motor_Start_Stop == ENABLE)
            {
                MotorControl[7].Speed_Real  = CH2TEMP*228.88;		//�����ٶ�
                CH2TEMP = 0;
            }
        }
    }
	if(htim==&htim5)
	{
//		    BLDC_Time++;
//		    Utime++;
//		    avglecont++;
			HALL_PHASE_Counter++;
//		if(TIM5->CNT==cntime/6 && MotorControl[5].Speed_Set>500)
//		{
//			  MotorControl[5].Hall.HallState_CW = MotorControl[5].Hall.HallState;
//			  MotorControl[5].Hall.HallStateValue= MotorControl[5].Hall.HallState_CW;
//			  BLDC1_PhaseChange( MotorControl[5].Hall.HallStateValue,MotorControl[5].PWM_Duty);
//			
//		}
//		   if(HALL_PHASE_Counter == MotorControl[5].Hall.PreHALL_Timercnt + MotorControl[5].Hall.chgperiod && MotorState != HALL_STUDY_MOTOR5&&MotorControl[5].Fault_Flag == 0)//&&Utime%(UPeriod/6)==0)
//			{
//				if(MotorControl[5].Speed_Ref < 0)
//			   {
//				    hallstatetemp =   0x07 ^MotorControl[5].Hall.HallState;
//				   if(hallstatetemp ==1)	MotorControl[5].Hall.HallState_CCW = 5;
//					else if(hallstatetemp ==3)	MotorControl[5].Hall.HallState_CCW = 1;
//					else if(hallstatetemp ==2)	MotorControl[5].Hall.HallState_CCW = 3;
//					else if(hallstatetemp ==6)	MotorControl[5].Hall.HallState_CCW = 2;
//					else if(hallstatetemp ==4)	MotorControl[5].Hall.HallState_CCW = 6;
//					else if(hallstatetemp ==5)	MotorControl[5].Hall.HallState_CCW = 4;
//					MotorControl[5].Hall.HallStateValue =MotorControl[5].Hall.HallState_CCW;
//					BLDC1_PhaseChange( MotorControl[5].Hall.HallStateValue,MotorControl[5].PWM_Duty);
//				  
//			   }
//				 else
//			   {
////				   hallstatetemp =  MotorControl[5].Hall.HallState;
////				   if(hallstatetemp ==1)	MotorControl[5].Hall.HallState_CW = 3;
////					else if(hallstatetemp ==3)	MotorControl[5].Hall.HallState_CW = 2;
////					else if(hallstatetemp ==2)	MotorControl[5].Hall.HallState_CW = 6;
////					else if(hallstatetemp ==6)	MotorControl[5].Hall.HallState_CW = 4;
////					else if(hallstatetemp ==4)	MotorControl[5].Hall.HallState_CW = 5;
////					else if(hallstatetemp ==5)	MotorControl[5].Hall.HallState_CW = 1;
////					MotorControl[5].Hall.HallStateValue =MotorControl[5].Hall.HallState_CW;
////					BLDC1_PhaseChange( MotorControl[5].Hall.HallStateValue,MotorControl[5].PWM_Duty);
//				   MotorControl[5].Hall.HallState_CW = MotorControl[5].Hall.HallState;
//				   MotorControl[5].Hall.HallStateValue= MotorControl[5].Hall.HallState_CW;
//				    BLDC1_PhaseChange( MotorControl[5].Hall.HallStateValue,MotorControl[5].PWM_Duty);
//				  
//			   }
//			   avglecont=0;
//			   
//			}
//			if(HALL_PHASE_Counter == MotorControl[6].Hall.PreHALL_Timercnt+MotorControl[6].Hall.chgperiod&&MotorState!= HALL_STUDY_MOTOR6&&MotorControl[6].Fault_Flag == 0)
//			{
//				if(MotorControl[6].PWM_Duty < 0)
//			   {
//				MotorControl[6].Hall.HallState_CCW = 0x07 ^ MotorControl[6].Hall.HallState;
//				BLDC2_PhaseChange( MotorControl[6].Hall.HallState_CCW,MotorControl[6].PWM_Duty);
//			   }
//				 else
//			   {
//				MotorControl[6].Hall.HallState_CW = MotorControl[6].Hall.HallState;
//				BLDC2_PhaseChange( MotorControl[6].Hall.HallState_CW,MotorControl[6].PWM_Duty);
//			   }	
//			}
	}
    if(htim ==&htim9)
    {
        strCapture .usPeriod ++;
    }
}
/*------------------------------------------------
Function:ɲ��
Input   :No
Output  :No
Explain :No
------------------------------------------------*/
void HAL_TIMEx_BreakCallback(TIM_HandleTypeDef *htim)
{
    if(htim == (&htim1))
    {
        CanLoadRate.timer++;
        if( ((GPIOE->IDR>>15)&(0x1)) ==0 )		/* Ӳ������ */
        {
            TIM1->CCR1 = 0;
            TIM1->CCR2 = 0;
            TIM1->CCR3 = 0;
            TIM1->EGR = 1;
            MC_SetFault(MOTOR5_BREAK);
            MotorControl[5].Motor_Start_Stop = 0;
            errormotor5++;
            MotorControl[5].Fault_Flag = 1;
            OverFlow_Cnt[0] +=1;
        }
        else
        {
        }
    }
}
/**
  * @brief HAL_TIM_IC_CaptureCallback��??��2a?��
  * @param None
  * @retval None
  */
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
    TIM_IC_InitTypeDef sConfigIC;
    if(htim==(&htim9))
    {
        if(htim->Channel == HAL_TIM_ACTIVE_CHANNEL_2)  //7��ˢ
        {
            HAL_TIM_ReadCapturedValue(htim,TIM_CHANNEL_2);
            CH2TEMP = CH2TEMP+1;
        }
        if(htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1)  //8���
        {
            HAL_TIM_ReadCapturedValue(htim,TIM_CHANNEL_1);
            CH1TEMP = CH1TEMP+1;
        }
    }
}
/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */static void MX_GPIO_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    /* GPIO Ports Clock Enable */
    __HAL_RCC_GPIOE_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOH_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOD_CLK_ENABLE();

    /*Configure GPIO pin Output Level */


    /*Configure GPIO pin Output Level */
    HAL_GPIO_WritePin(GPIOD, PWM_AL3_Pin|PWM_BL3_Pin|PWM_AL4_Pin|PWM_BL4_Pin
                      |PWM_AL1_Pin|PWM_BL1_Pin|PWM_AL2_Pin|PWM_BL2_Pin, GPIO_PIN_RESET);

    /*Configure GPIO pin Output Level */
//    HAL_GPIO_WritePin(GPIOA, SideBrush_BRAKE_Pin|SideBrush_F_R_Pin|SideBrush_EN_Pin, GPIO_PIN_RESET);
//    HAL_GPIO_WritePin(GPIOA,BRAKE3|FAULT2_Pin,GPIO_PIN_SET);

    HAL_GPIO_WritePin(GPIOB,RUN_Pin,GPIO_PIN_SET);

    /*Configure GPIO pins : HALL_3_Pin HALL_4_Pin HALL_1_Pin HALL_2_Pin */
    GPIO_InitStruct.Pin = HALL_3_Pin|HALL_4_Pin|HALL_1_Pin|HALL_2_Pin;   //4���Ƹ� �ⲿ�ж�HALL
    GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

    /*Configure GPIO pins : HALL_U2_Pin HALL_V2_Pin */
    GPIO_InitStruct.Pin = HALL_U2_Pin|HALL_V2_Pin|HALL_W2_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    /*Configure GPIO pin : PC15 */
//  GPIO_InitStruct.Pin = GPIO_PIN_15;
//  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
//  GPIO_InitStruct.Pull = GPIO_NOPULL;
//  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    /*Configure GPIO pin : FAN_EN_Pin */
    GPIO_InitStruct.Pin = FAN_F_R;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(FAN_F_R_GPIO_Port, &GPIO_InitStruct);
    HAL_GPIO_WritePin(FAN_F_R_GPIO_Port, FAN_F_R, GPIO_PIN_RESET);



    /*Configure GPIO pin : RUN_Pin */
    GPIO_InitStruct.Pin = RUN_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(RUN_GPIO_Port, &GPIO_InitStruct);

    /*Configure GPIO pins : PWM_AL3_Pin PWM_BL3_Pin PWM_AL4_Pin PWM_BL4_Pin
                             PWM_AL1_Pin PWM_BL1_Pin PWM_AL2_Pin PWM_BL2_Pin */
    GPIO_InitStruct.Pin = PWM_AL3_Pin|PWM_BL3_Pin|PWM_AL4_Pin|PWM_BL4_Pin
                          |PWM_AL1_Pin|PWM_BL1_Pin|PWM_AL2_Pin|PWM_BL2_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_WritePin(GPIOD,PWM_AL3_Pin|PWM_BL3_Pin|PWM_AL4_Pin|PWM_BL4_Pin
                      |PWM_AL1_Pin|PWM_BL1_Pin|PWM_AL2_Pin|PWM_BL2_Pin,GPIO_PIN_SET); //��ΪFD2103S��Lͨ��Ϊ�������������ʼ��Ϊ�ߵ�ƽ
    HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

    /*Configure GPIO pins : SideBrush_BRAKE_Pin SideBrush_F_R_Pin SideBrush_EN_Pin BRAKE3
                             FAULT2_Pin */
    GPIO_InitStruct.Pin = FAULT2_Pin|SideBrush_BRAKE_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
    HAL_GPIO_WritePin(GPIOA,FAULT2_Pin|SideBrush_BRAKE_Pin,GPIO_PIN_SET);

    GPIO_InitStruct.Pin = SideBrush_F_R_Pin|FAULT1_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
    HAL_GPIO_WritePin(GPIOB,FAULT1_Pin|SideBrush_F_R_Pin,GPIO_PIN_SET);



    /*Configure GPIO pin : BRAKE3 4 5 */

    GPIO_InitStruct.Pin = BRAKE5|BRAKE3|BRAKE4; //BRAKE4|SideBrush_BRAKE_Pin
    GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /*Configure GPIO pin : BRAKE2_Pin */
    GPIO_InitStruct.Pin = BRAKE2_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(BRAKE2_GPIO_Port, &GPIO_InitStruct);

    /*Configure GPIO pins : HALL_U1_Pin HALL_V1_Pin HALL_W1_Pin */
    GPIO_InitStruct.Pin = HALL_U1_Pin|HALL_V1_Pin|HALL_W1_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
void HAL_SYSTICK_Callback(void)
{
    if(HAL_GetTick()%2 == 0)  //1MS
    {
        BLDC_Stuck_Chk();	//��ת����500ms����
        BitSet(SysCLC.SysTimFlag,A1mSec);
    }
    if(HAL_GetTick()%4 == 0)  //2MS
    {
        switch (MotorState)
        {
        case IDLE:

//            MotorState = INIT;		ƫ�õ�ѹ���������INIT

            break;

        case INIT:

//            TIM1->CCER |= 0Xddd;
//            TIM8->CCER |= 0Xddd;
//            if(MotorControl[1].Push_Location_model == 1)
//            {
            MotorState = START; //�궨�ɹ��ٽ�����һ��״̬����ʹ��CANͨ��
//            }
            HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

            break;

        case START:

            MotorState = RUN;

            break;

        case RUN:   // motor running


            for(u8 i=0; i < MOTOR_NUM; i++)
            {
                if(MotorControl[i].Push_motor_calibrationFLAG == 1)
                {
                    MotorControl[i].Motor_Start_Stop = ENABLE;
                }
                if(MotorControl[i].Motor_Start_Stop == ENABLE)			/* Ӳ�������������ܳ��ޣ�������ʹ�� */
                {
                    switch (i)
                    {
                    case 5:
                        Ramp_Speed(5);
                        MotorControl[5].Speed_Real = GetMotorSpeed(5);
                        BLDC1_OverSpdChk();
                        MotorControl[5].Current.ADCValue_Ref = PID_Regulator(MotorControl[5].Speed_Ref,MotorControl[5].Speed_Real,&PID_Speed_InitStruct[0]);
                        break;
                    case 6:
                        Ramp_Speed(6);
                        MotorControl[6].Speed_Real = GetMotorSpeed(6);
                        BLDC2_OverSpdChk();
                        MotorControl[6].Current.ADCValue_Ref = PID_Regulator(MotorControl[6].Speed_Ref,MotorControl[6].Speed_Real,&PID_Speed_InitStruct[1]);
                        break;
                    case 3:
                    case 4:
                        if(MotorControl[i].Fault_Flag == 0&&OverFlow_Cnt[2]<BREAK_CNT_MAX)
                            SetMotorSpeed(i, MotorControl[i].PWM_Duty);
                        else if(OverFlow_Cnt[2]>=BREAK_CNT_MAX)
                        {
                            MC_SetFault(BRAKE_3_4);
                            MotorControl[i].Motor_Start_Stop = DISABLE;
                        }
                        else MotorControl[i].Motor_Start_Stop = DISABLE;
                        break;
                    case 0:
                    case 1:
                    case 2:
                    case 9:
                        if(MotorControl[i].Push_motor_calibrationFLAG == 1)    //ÿ�α궨��ϻὫ�ñ�־λ��0
                        {
                            Push_Motor_Calibrate(i);
                        }
                        else if(MotorControl[i].Fault_Flag == 0&&OverFlow_Cnt[3]<BREAK_CNT_MAX)
                            SetMotorSpeed(i, MotorControl[i].PWM_Duty);
                        else if(OverFlow_Cnt[3]>=BREAK_CNT_MAX)
                        {
                            MC_SetFault(BRAKE_0_1_2_9);
                            MotorControl[i].Motor_Start_Stop = DISABLE;
                        }
                        else MotorControl[i].Motor_Start_Stop = DISABLE;
                        if(HAL_GetTick()%20 == 0) //10msλ�û�
                        {
                            Push_Motor_Location_Control(i);		/* �Ƹ�λ�û� */
                        }
                        break;
                    case 10:
                    case 11:
                    case 12:
                    case 13:
                        if(MotorControl[i].Fault_Flag == 0&&OverFlow_Cnt[4]<BREAK_CNT_MAX)
                            SetMotorSpeed(i, MotorControl[i].PWM_Duty);
                        else if(OverFlow_Cnt[4]>=BREAK_CNT_MAX)
                        {
                            MC_SetFault(BRAKE10_11_12_13);
                            MotorControl[i].Motor_Start_Stop = DISABLE;
                        }
                        else MotorControl[i].Motor_Start_Stop = DISABLE;
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
                    }

                }
                else if(MotorControl[i].Motor_Start_Stop == DISABLE)
                {
                    SetMotorStop(i);
                }
            }
            if(HAL_GetTick()%20 == 0)  //10ms
            {
                Push_Motor_CurSelfAdapt(5,1,580,5);    //�Ƹ˸߶�����Ӧ
            }
            if(HAL_GetTick()%200 == 0)
            {
//                Push_Motor_CurSelfAdapt(6,1,540,5);    //�Ƹ˸߶�����Ӧ
                Motor7_Err_Chk();		//��ˢ�������
            }

            break;

        case STOP1:    // motor stopped

            MotorState = WAIT;

            break;

        case WAIT:    // wait MotorState

            break;

        case FAULT:

            if(wGlobal_Flags == 0)
            {
                MotorState = IDLE;
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
                if ( (wGlobal_Flags & HALL5_SENSOR_ERR) == 0 )
                {
                    MotorState = IDLE;
                }
                else
                {
                    MotorState = FAULT;
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
                if ((wGlobal_Flags & HALL6_SENSOR_ERR) == 0 )
                {
                    MotorState = IDLE;
                }
                else
                {
                    MotorState = FAULT;
                }
                First_Enter_STUDY_MOTOR6 = 1;
            }
            break;

        default:
            break;
        }
    }
    if(HAL_GetTick()%20 == 0) //10ms
    {
        Over_VoltageCheck(); //��ѹ����
    }
    if(HAL_GetTick()%1000 == 0) //500msι��
    {
        HAL_IWDG_Refresh(&hiwdg);
			  Push_OverRunChk(1,20,1,20,0,30,0,30);    //�Ƹ˶���ʱ������
    }

}

int fputc(int ch, FILE *f)
{
    HAL_UART_Transmit(&huart3, (uint8_t *)&ch, 1, 0xffff);
    return ch;
}

/**
  * o����y1|?��: ???��?��c?ao����ygetchar,scanf��?DEBUG_USARTx
  * ��?��?2?��y: ?T
  * ���� ?? ?��: ?T
  * ?��    ?�¡�o?T
  */
int fgetc(FILE * f)
{
    uint8_t ch = 0;
    HAL_UART_Receive(&huart3,&ch, 1, 0xffff);
    return ch;
}

void Sys_Time(void)
{
    SysCLC.MilliSec++;
    if((SysCLC.MilliSec%10)==0)//20MS
    {
        BitSet(SysCLC.SysTimFlag,A10mSec);
    }

    if(SysCLC.MilliSec >= 250)		//250MS
    {
        SysCLC.MilliSec = 0;
        BitSet(SysCLC.SysTimFlag,A250mSec);
        SysCLC.SecondCntr++;		//1????��y?��
    }

    if(SysCLC.SecondCntr >= 4)		//1000MS = 1S
    {
        SysCLC.SecondCntr = 0;
    }
}

int16_t LV_TxBuffer[10],TxBufferSize=10;
u8 uart_counter=2;
#define ac 0
#define ab 1
#define bc 2
#define ac_ab_bc_index 3
extern uint32_t M6_100us_ac_ab_bc_cur[ac_ab_bc_index];
extern uint32_t testM6_100us_ac_ab_bc_cur;
void Labview_uart(void)
{
    s16 temp_tab[5];
//  if(uart_tmr == 1)//20
    {
        switch(uart_counter)
        {
        case 0:
            temp_tab[0] = 0x5A0A;;//
            temp_tab[1] = 23055;//
            temp_tab[2] = 24555;//
            temp_tab[3] = 30000;//
            temp_tab[4] = 20000 ;//

            break;
        case 1:
            temp_tab[0] = 0x5A0A;;//
            temp_tab[1] = MotorControl[0].Current.FilterValue;//
            temp_tab[2] = MotorControl[1].Current.FilterValue;//
            temp_tab[3] = MotorControl[2].Current.FilterValue;//
            temp_tab[4] = MotorControl[3].Current.FilterValue ;//

            break;
        case 2:
            temp_tab[0] = 0x5A0A;;//
            temp_tab[1] = MotorControl[5].Current.GetADCValue;//
            temp_tab[2] = MotorControl[5].Current.FilterValue;//
            temp_tab[3] = MotorControl[6].Current.GetADCValue;//
            temp_tab[4] = MotorControl[6].Current.FilterValue ;//

            break;
        case 3:
            temp_tab[0] = 0x5A0A;;//
            temp_tab[1] = MotorControl[4].Current.FilterValue;//
            temp_tab[2] = MotorControl[9].Current.FilterValue;//
            temp_tab[3] = MotorControl[2].Current.FilterValue;//
            temp_tab[4] = MotorControl[3].Current.FilterValue ;//

            break;

        case 4:
            temp_tab[0] = 0x5A0A;;//
            temp_tab[1] = MotorControl[5].Speed_Ref;//
            temp_tab[2] = MotorControl[5].Speed_Real;//
            temp_tab[3] = MotorControl[6].Speed_Ref;//
            temp_tab[4] = MotorControl[6].Speed_Real ;//

            break;
        case 5:
            temp_tab[0] = 0x5A0A;;//
            temp_tab[1] = MotorControl[5].Speed_Ref;//
            temp_tab[2] = MotorControl[5].Speed_Real;//
            temp_tab[3] = MotorControl[5].PWM_Duty;//
            temp_tab[4] = MotorControl[5].Speed_Set ;

            break;

        case 6:
            temp_tab[0] = 0x5A0A;;//
            temp_tab[1] = M6_100us_ac_ab_bc_cur[ab]/10;//
            temp_tab[2] = M6_100us_ac_ab_bc_cur[ac]/10;//
            temp_tab[3] = M6_100us_ac_ab_bc_cur[bc]/10;//
            temp_tab[4] = testM6_100us_ac_ab_bc_cur/10;

            break;

        default:
            break;
        }

        LV_TxBuffer[0] = temp_tab[0]/256;
        LV_TxBuffer[1] = temp_tab[0]%256;

        LV_TxBuffer[2] = temp_tab[1]/256;
        LV_TxBuffer[3] = temp_tab[1]%256;

        LV_TxBuffer[4] = temp_tab[2]/256;
        LV_TxBuffer[5] = temp_tab[2]%256;

        LV_TxBuffer[6] = temp_tab[3]/256;
        LV_TxBuffer[7] = temp_tab[3]%256;

        LV_TxBuffer[8] = temp_tab[4]/256;
        LV_TxBuffer[9] = temp_tab[4]%256;

//	  HAL_UART_Transmit(&UART6_Handler, (uint8_t *)LV_TxBuffer, TxBufferSize,0x20);

//	  HAL_UART_Transmit_IT(&UART6_Handler, (uint8_t *)LV_TxBuffer, TxBufferSize);

        printf("%c%c%c%c%c%c%c%c%c%c",LV_TxBuffer[0],LV_TxBuffer[1],LV_TxBuffer[2],LV_TxBuffer[3],LV_TxBuffer[4],LV_TxBuffer[5],LV_TxBuffer[6],LV_TxBuffer[7],LV_TxBuffer[8],LV_TxBuffer[9]);

    }
}
int phasecount[5];
int CCRValue,acounta,testcount,CCRValueB;
int SetCCRValue,SetCCRValueB;
int avgle[5];
int pwmchangeflag;
void HAL_TIM_OC_DelayElapsedCallback(TIM_HandleTypeDef *htim)
{
		if(htim==&htim5)
	{ 
		if( MotorControl[5].Speed_Real>500)
		{
			HAL_GPIO_TogglePin(GPIOB,GPIO_PIN_12);
			if(pwmchangeflag==1)
			{
			   MotorControl[5].Hall.Hall_Changecnt = 6;
			}
//			PWM_Change(avglecount,MotorControl[5].PWM_Duty);
			if(avglecount==0)
			{
				MotorControl[5].Hall.HallState_CW = 3;
				avglecount=1;
				
			}
			else if(avglecount==1)
			{
				MotorControl[5].Hall.HallState_CW = 2;
				avglecount=2;
			}
				else if(avglecount==2)
			{
				MotorControl[5].Hall.HallState_CW = 6;
				avglecount=3;
			}
				else if(avglecount==3)
			{
				MotorControl[5].Hall.HallState_CW = 4;
				avglecount=4;
			}
				else if(avglecount==4)
			{
				MotorControl[5].Hall.HallState_CW = 5;
				avglecount=5;
			}
				else if(avglecount==5)
			{
				MotorControl[5].Hall.HallState_CW = 1;
				avglecount=0;
			}
			CCRValue = __HAL_TIM_GetCounter(htim);
            SetCCRValue=CCRValue + MotorControl[5].period;
		    __HAL_TIM_SET_COMPARE(&htim5,TIM_CHANNEL_1,SetCCRValue);
		    BLDC1_PhaseChange(MotorControl[5].Hall.HallState_CW,MotorControl[5].PWM_Duty);
		}
		
		   if( MotorControl[5].Speed_Real<-500)
		{
			
			if(avglecount==0)
			{
				MotorControl[5].Hall.HallState_CCW = 2;
				avglecount=1;
			}
			else if(avglecount==1)
			{
				MotorControl[5].Hall.HallState_CCW = 3;
				avglecount=2;
			}
				else if(avglecount==2)
			{
				MotorControl[5].Hall.HallState_CCW = 1;
				avglecount=3;
			}
				else if(avglecount==3)
			{
				MotorControl[5].Hall.HallState_CCW = 5;
				avglecount=4;
			}
				else if(avglecount==4)
			{
				MotorControl[5].Hall.HallState_CCW = 4;
				avglecount=5;
			}
				else if(avglecount==5)
			{
				MotorControl[5].Hall.HallState_CCW = 6;
				avglecount=0;
			}
			CCRValue = __HAL_TIM_GetCounter(htim);
            SetCCRValue=CCRValue + MotorControl[5].period;
		    __HAL_TIM_SET_COMPARE(&htim5,TIM_CHANNEL_1,SetCCRValue);
		    BLDC1_PhaseChange(MotorControl[5].Hall.HallState_CCW,MotorControl[5].PWM_Duty);
		}	
	}
		if(htim==&htim12)
	{ 
		if( MotorControl[6].Speed_Real>500)
		{
			    if(avglecountB==0)
			{
				MotorControl[6].Hall.HallState_CW = 3;
				avglecountB=1;
				
			}
			else if(avglecountB==1)
			{
				MotorControl[6].Hall.HallState_CW = 2;
				avglecountB=2;
			}
				else if(avglecountB==2)
			{
				MotorControl[6].Hall.HallState_CW = 6;
				avglecountB=3;
			}
				else if(avglecountB==3)
			{
				MotorControl[6].Hall.HallState_CW = 4;
				avglecountB=4;
			}
				else if(avglecountB==4)
			{
				MotorControl[6].Hall.HallState_CW = 5;
				avglecountB=5;
			}
				else if(avglecountB==5)
			{
				MotorControl[6].Hall.HallState_CW = 1;
				avglecountB=0;
			}
			CCRValueB = __HAL_TIM_GetCounter(htim);
            SetCCRValueB=CCRValueB + MotorControl[6].period;
		    __HAL_TIM_SET_COMPARE(&htim12,TIM_CHANNEL_1,SetCCRValueB);
		    BLDC2_PhaseChange(MotorControl[6].Hall.HallState_CW,MotorControl[6].PWM_Duty);
		}
		  if( MotorControl[6].Speed_Real<-500)
		{
			
			if(avglecountB==0)
			{
				MotorControl[6].Hall.HallState_CCW = 2;
				avglecountB=1;
			}
			else if(avglecountB==1)
			{
				MotorControl[6].Hall.HallState_CCW = 3;
				avglecountB=2;
			}
				else if(avglecountB==2)
			{
				MotorControl[6].Hall.HallState_CCW = 1;
				avglecountB=3;
			}
				else if(avglecountB==3)
			{
				MotorControl[6].Hall.HallState_CCW = 5;
				avglecountB=4;
			}
				else if(avglecountB==4)
			{
				MotorControl[6].Hall.HallState_CCW = 4;
				avglecountB=5;
			}
				else if(avglecountB==5)
			{
				MotorControl[6].Hall.HallState_CCW = 6;
				avglecountB=0;
			}
			CCRValueB = __HAL_TIM_GetCounter(htim);
            SetCCRValueB=CCRValueB + MotorControl[6].period;
		    __HAL_TIM_SET_COMPARE(&htim12,TIM_CHANNEL_1,SetCCRValueB);
		    BLDC2_PhaseChange(MotorControl[6].Hall.HallState_CCW,MotorControl[6].PWM_Duty);
		}	
	}
}


int  PWM_Change(int time_cnt,int Motor_PWM)
{
	if(time_cnt==6)
	{
		Motor_PWM = Motor_PWM * 1.2f;
	}
	else 
	{
		Motor_PWM = Motor_PWM * 0.97f;
	}
	return Motor_PWM;
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
