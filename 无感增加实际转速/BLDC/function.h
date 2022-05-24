
#include "bsp_BDCMotor.h"

#ifndef	_FUNCTION_H
#define	_FUNCTION_H


#define BusVoltValue(B)   (B*57)// BusVoltValue/ V
#define MOTOR0_CurrentValue(A)   (A*170)  //A  电流检测范围为 25.78A 2mA  PGA64
#define MOTOR1_CurrentValue(A)   (A*170) 	//A  电流检测范围为 25.78A 2mA PGA64


#define M1_CurrentValue(A)   (A*170)
#define M2_CurrentValue(A)   (A*170)

#define ABS(i)  ((i)>=0?(i):~(i))

#define OVER_CURRENT(NUM) 1<<NUM

#define  NO_ERROR                 0X00000000


#define  M1_BREAK		          0X20000000
#define  M2_BREAK                 0X40000000
#define  M1_OVER_CURRENT          0X00000001
#define  M2_OVER_CURRENT          0X00000002
#define  OVER_VOLTAGE             0X00000004
#define  UNDER_VOLTAGE            0X00000008

#define  M1_OVER_HEAT             0X00000010
#define  M1_NTC_ERR               0X00000020
#define  M1_LOCKEDUP              0X00000040
#define  M1_HALL_ERR              0X00000080

#define  M2_OVER_HEAT             0X00000100
#define  M2_NTC_ERR               0X00000200
#define  M2_LOCKEDUP              0X00000400
#define  M2_HALL_ERR              0X00000800

#define  MOTORA_MISSING_PHASE            0X00200000 //电机A启动时就缺相，无法检测是哪一相
#define  MOTORB_MISSING_PHASE            0X00400000 //电机B启动时就缺相，无法检测是哪一相

#define  MOTORA_PHASE_ERROR		  0X00002000//A电机相序错误
#define  MOTORB_PHASE_ERROR       0X00004000//B电机相序错误

#define  MOTORA_OVER_SPEED 		           0X00040000//A号电机失速
#define  MOTORB_OVER_SPEED               0X00080000//B号电机失速




#define  MOTORA_MISSING_PHASE_A      0X00000100 //电机5缺A
#define  MOTORA_MISSING_PHASE_B      0X00000200 //电机5缺B
#define  MOTORA_MISSING_PHASE_C      0X00000400 //电机5缺C
#define  MOTORB_MISSING_PHASE_A      0X00000800 //电机6缺A   
#define  MOTORB_MISSING_PHASE_B       0X00001000 //电机6缺B
#define  MOTORB_MISSING_PHASE_C       0X00002000 //电机6缺C

void BLDCA_Phase_Check(void);
void BLDCB_Phase_Check(void);
#define  CAN_COMMUNICATION_ERR    0X00100000

#define PHASE_A_MSK       (u32)((u32)(ADC_CHANNEL_9<<15 ))
#define PHASE_B_MSK       (u32)((u32)(ADC_CHANNEL_VOPAMP1<<15))
#define HALL_MSK1       (u32)((u32)((u32)(ADC_CHANNEL_9) << 10) + (u32)((u32)(ADC_CHANNEL_8 ) << 5) + (u32)((ADC_CHANNEL_VOPAMP2 )<<0))
#define HALL_MSK2       (u32)((u32)((u32)(ADC_CHANNEL_3) << 10) + (u32)((u32)(ADC_CHANNEL_17) << 5) + (u32)((ADC_CHANNEL_1)<<0))
#define SEQUENCE_LENGHT    0x00300000


#define VOLT_160V  BusVoltValue(16)  
#define VOLT_200V  BusVoltValue(20)  
#define VOLT_240V  BusVoltValue(24)  
#define VOLT_300V  BusVoltValue(30)  
#define VOLT_330V  BusVoltValue(33)  


//#define GREEN_LED_ON     		RED_LED2_ON //HAL_GPIO_WritePin(Normal_GPIO_Port,Normal_Pin,GPIO_PIN_RESET)
//#define GREEN_LED_OFF 			RED_LED2_OFF //HAL_GPIO_WritePin(Normal_GPIO_Port,Normal_Pin,GPIO_PIN_SET)
#define	RED_LED1_ON         HAL_GPIO_WritePin(Fault_1_GPIO_Port,Fault_1_Pin,GPIO_PIN_SET)
#define	RED_LED1_OFF				HAL_GPIO_WritePin(Fault_1_GPIO_Port,Fault_1_Pin,GPIO_PIN_RESET)
#define RED_LED1_TOGGLE    HAL_GPIO_TogglePin(Fault_1_GPIO_Port,Fault_1_Pin)
#define LED_LOW_SPEED 		40
#define LED_HIGH_SPEED		20

#define PB10_Toggle        HAL_GPIO_TogglePin(GPIOB,GPIO_PIN_10)

//#define	RED_LED2_ON 				RED_LED1_ON//HAL_GPIO_WritePin(Fault_2_GPIO_Port,Fault_2_Pin,GPIO_PIN_RESET)
//#define	RED_LED2_OFF				RED_LED1_OFF//HAL_GPIO_WritePin(Fault_2_GPIO_Port,Fault_2_Pin,GPIO_PIN_SET)

extern void CurrentLimit(u8 Motor_NUM);
extern void DisplayErrLed(void);
extern void MC_SetFault(u32 hFault_type);
extern void MC_ClearFault(u32 hFault_type);
extern void Init_Drive_Para(void);

extern u32 wGlobal_Flags;
extern u32 FaultOccurred;

typedef struct
{
    u32 ab;
    u32 ac;
    u32 bc;
    u32 ba;
    u32 ca;
    u32 cb;
    u16  index_ab;
    u16  index_ac;
    u16  index_bc;
    u16  index_ba;
    u16  index_ca;
    u16  index_cb;
    
    u32 ab_t;
    u32 ac_t;
    u32 bc_t;
    u32 ba_t;
    u32 ca_t;
    u32 cb_t;
    u16  index_ab_t;
    u16  index_ac_t;
    u16  index_bc_t;
    u16  index_ba_t;
    u16  index_ca_t;
    u16  index_cb_t;
    u16  skr;
  u16 abaver;
  u16 acaver;
  u16 bcaver;
  u16 baaver;
  u16 caaver;
  u16 cbaver;
} DefaultPhaseCheekCurTemp;
extern DefaultPhaseCheekCurTemp MotorA_Current_temp;
extern DefaultPhaseCheekCurTemp MotorB_Current_temp;
#endif