#ifndef __BDCMOTOR_TIM_H__
#define __BDCMOTOR_TIM_H__

/* 包含头文件 ----------------------------------------------------------------*/
#include "stm32g4xx_hal.h"
//#include "MC_PID_regulators.h"
#include "pid_regulator.h"
#include "mc_config.h"

typedef int32_t  s32;
typedef int16_t s16;
typedef int8_t  s8;

typedef uint32_t  u32;
typedef uint16_t u16;
typedef uint8_t  u8;

#define POLE_PAIRES  2
#define POLE_PAIRES2  2
#define PWM_PERIOD 8500       //4.2 change
#define MOTOR_NUM 2

#define NEGATIVE          (s8)-1
#define POSITIVE          (s8)1
#define NEGATIVE_SWAP     (s8)-2
#define POSITIVE_SWAP     (s8)2
#define ERROR             (s8)0
#define MISSTEP           (s8)0

#define U8_MAX     (255)
#define S8_MAX     (127)
#define S8_MIN     (-128)
#define U16_MAX    (65535u)
#define S16_MAX    (32767)
#define S16_MIN    (-32768)
#define U32_MAX    (4294967295uL)
#define S32_MAX    (2147483647)
#define S32_MIN    (-2147483648uL)


#define A1mSec		0x01
#define A2mSec		0x02
#define A10mSec		0x04
#define A20mSec		0x08
#define A250mSec	0x10
#define A500mSec	0x20
#define A1Sec		0x40

//#define HALL_STUDY  15




#define PWMMAX	499
#define PWMMin        0




//#define CurrentValue(A)   (A*14.74)//Current value=(14.74*A)A  3.6mR
//#define CurrentValue(A)   (A*15.48)//Current value=(15.48*A)A  3.6mR






#define MOTOR_5_6_ENABLE  HAL_GPIO_WritePin(GPIOE, BLDC_ENABLE_Pin, GPIO_PIN_SET);

#define MOTOR_0_1_ENABLE  HAL_GPIO_WritePin(GPIOA, RESET_1_Pin, GPIO_PIN_SET);

#define MOTOR_2_3_4_ENABLE  HAL_GPIO_WritePin(RESET_2_GPIO_Port, RESET_2_Pin, GPIO_PIN_SET);


#if 1
#define BIT0 0x01
#define BIT1 0x02
#define BIT2 0x04
#define BIT3 0x08
#define BIT4 0x10
#define BIT5 0x20
#define BIT6 0x40
#define BIT7 0x80



typedef struct
{
    int16_t GetADCValue;//获取的ADC值
    int16_t FilterValue;//滤波后的ADC值
		int16_t ADCValue_Ref;
	
    uint16_t MaxValue1;   //最大值第一阶段
    uint16_t MaxValue2;   //最大值第二阶段
	  uint16_t MaxValue3;   //最大值第一阶段
    uint16_t MaxValue4;   //最大值第二阶段
    uint16_t MinValue;   //最小值
		float PreFilterVal;	//上个周期的滤波ADC值
	float	DeepFilterVAL;	//深度滤波的adc
	
	  int16_t    PhaseACurrent;
	  int16_t    PhaseBCurrent;
	  int16_t    PhaseCCurrent;
	  uint16_t    PhaseAOffset;
	  uint16_t    PhaseBOffset;
	  uint16_t    PhaseCOffset;

    u16 OFCnt1;
    u16 OFCnt2;
	  u16 OFCnt3;
    u16 OFCnt4;
		   
		u16 OFCnt1_T;
    u16 OFCnt2_T;
	  u16 OFCnt3_T;
    u16 OFCnt4_T;
    float ConversionFactor; //转化系数
    float   ActualUnitValue;  //转换为实际单位值
    int offset;
} ADC_ValueParameters_t;

typedef struct
{
    uint8_t ChangeFlag;     //第一次启动标志
    uint8_t HallState;               //当前Hall状态
    uint8_t PrevHallState;           //上一次Hall状态
    uint8_t HallState_Temp;          //两次Hall状态组合，用来判断方向
    uint8_t HallState_CCW;
    uint8_t HallState_CW;
	  uint8_t HallStateValue; 
    uint16_t HALL_CaptureValue;      //Hall捕获值
    uint16_t HALL_PreCaptureValue;   //上一次Hall捕获值
    u32 PreHALL_OVF_Counter;         //上一次计数器溢出值
    s32 HALL_CaptureValueDelta;
} HALL_Parameters_t;

typedef struct
{
    uint8_t  StudySectorCnt;
    uint16_t StudySectorCnt2;
    uint16_t StudySectorCnt3;
    uint8_t HallTab[6];
    uint8_t HallSector;
    int16_t HallCommPWM;
    uint8_t CommuntionState;
}
HALL_Study_t;

extern HALL_Study_t HALL_Study[2];
typedef struct
{
    int16_t Speed_Set;
    int16_t Speed_Ref;
    int16_t Speed_Real;
    int16_t Acceleration;
    int16_t Deceleration;
    int16_t SpeedLimit;

    int16_t PWM_DutySet;
    int16_t PWM_Duty;
    uint8_t Pole_Paires;
    uint8_t Fault_Flag;

    int8_t  Direction;
    FunctionalState  Motor_Start_Stop;

    ADC_ValueParameters_t	Current;
    HALL_Parameters_t   Hall;
    int16_t Fault_Cnt;   //新增Fault_Flag
} MotorControlParameters_t;

typedef struct
{
    u16 AdBuf;
    u16 SVADBuf;
    u16 SVADAverage;
    u16 BUS;
	  u16 BUS_V;
    u16 BusLow;
    u16 BusLowCnt;
	  u16 BusLowCnt_T;
    u16 BusHigh;
    u16 BusHighCnt;
	  u16 BusHighCnt_T;
    u8 SvModelCnt;
    u16 servo;
    u8 Flag;
	u16 HighGuardTime;
	u16 LowGuardTime;
} VoltVar_TypeDef;
extern VoltVar_TypeDef VoltVar;

#define MOTOR_NUM 2
#define NOMINAL_CURRENT  PWM_PERIOD  //PWM Max
#define IQMAX NOMINAL_CURRENT


typedef int32_t  s32;
typedef int16_t s16;
typedef int8_t  s8;

typedef uint32_t  u32;
typedef uint16_t u16;
typedef uint8_t  u8;

/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */


extern MotorControlParameters_t MotorControl[MOTOR_NUM];

extern PID_Handle_t PIDIq_BLDC_M1, PIDIq_BLDC_M2;
extern uint8_t PrevHallState1,PrevHallState2,HallState1,HallState2;
extern uint16_t  HALL_CaptureValue,HALL_PreCaptureValue,Speed_Hz;
extern s16 BLDC1_Speed_RPM ;

extern uint16_t  HALL_CaptureValue2,HALL_PreCaptureValue2;
extern s16 BLDC2_Speed_RPM ;

extern u16 Hall1_Slow_CNT,Hall2_Slow_CNT;
extern s32 HALL_CaptureValueDelta,HALL_CaptureValueDelta2;
extern u32 HALL_OVF_Counter  ;
extern TIM_HandleTypeDef htim7;

extern u16 Uart_TxBuffer_CNT ;
extern u16 Uart_TxBuffer_CNT1 ;
extern int16_t LV_Uart_TxBuffer[5][2000];


extern void SetMotorSpeed(uint8_t Motor_NUM,int16_t PWM_Duty);
extern void SetMotorStop(uint8_t Motor_NUM);
extern u8 HALL_GetPhase2(void);
extern u8 HALL_GetPhase1(void);
extern void BLDC1_PhaseChange(u8 bHallState, s16 PWM_Duty);
extern void BLDC2_PhaseChange(u8 bHallState, s16 PWM_Duty);
extern void HALLSTUDY_PhaseChange0(u8 bHallState);
extern void HALLSTUDY_PhaseChange1(u8 bHallState);
extern s16 Ramp_Speed(u8 Mortor_NUM);
extern s16 Ramp_PPWM(u8 Mortor_NUM);
extern s16 GetMotorSpeed(u8 Mortor_NUM);
extern void HallStudyHandle0(void);
extern void HallStudyHandle1(void);

extern void SysTime(void);

extern void GetPhaseCurrentsM1(void);
#endif
#endif	/* __BDCMOTOR_TIM_H__ */
/******************* (C) COPYRIGHT 2015-2020 硬石嵌入式开发团队 *****END OF FILE****/