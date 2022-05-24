/******************** (C) COPYRIGHT TOPBAND ********************
* File Name          : MC_PID_regulators.h
* Author             : Guoxl
* Date First Issued  : 
* Description        : 
********************************************************************************
* COPYRIGHT TOPBAND
* THIS SOURCE CODE IS PROTECTED BY A LICENSE.
*******************************************************************************/

/* Define to prevent recursive inclusion -------------------------------------*/
 
#ifndef __PI_REGULATORS__H
#define __PI_REGULATORS__H

/* Includes ------------------------------------------------------------------*/
//#include "function.h"
#include "bsp_BDCMotor.h"

#define MOTOR_NUM 2
#define NOMINAL_CURRENT  8800  //PWM Max
#define IQMAX 8800

#define KP_GAIN_SPEED        100//200                             //速度PID 比例系数
#define KI_GAIN_SPEED         2//40                             //速度PID 积分系数
/* default values for Speed control loop */
#define PID_SPEED_REFERENCE_RPM   (s16)300
#define PID_SPEED_KP_DEFAULT      (s16)1000              //速度环Kp
#define PID_SPEED_KI_DEFAULT      (s16)5                //速度环KI
#define PID_SPEED_KD_DEFAULT      (s16)0

/* Speed PID parameter dividers          */
#define SP_KPDIV ((u16)(256))
#define SP_KIDIV ((u16)(1024))
#define SP_KDDIV ((u16)(16))


#define KP_GAIN_CURRENT_LIMIT_HIGH (s16)(512)
#define KP_GAIN_CURRENT_LIMIT_LOW  (s16)(128)
//#define KP_GAIN_CURRENT_MIN  (s16)(16)
#define KP_GAIN_CURRENT_MIN  (s16)(80)         //电流环Kp
#define KI_GAIN_CURRENT      (s16)(1)          //电流环Ki
#define KD_GAIN_CURRENT      (s16)(0)

/* current PID parameter dividers          */
//#define CURRENT_KPDIV ((u16)(4096))
//#define CURRENT_KIDIV ((u16)(4096))
//#define CURRENT_KDDIV ((u16)(4096))
#define CURRENT_KPDIV ((u16)(256))
#define CURRENT_KIDIV ((u16)(1024))
#define CURRENT_KDDIV ((u16)(16))
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
typedef struct 
{  
  s16 hKp_Gain;
  u16 hKp_Divisor;
  s16 hKi_Gain;
  u16 hKi_Divisor;  
  s16 hLower_Limit_Output;     //Lower Limit for Output limitation
  s16 hUpper_Limit_Output;     //Lower Limit for Output limitation
  s32 wLower_Limit_Integral;   //Lower Limit for Integral term limitation
  s32 wUpper_Limit_Integral;   //Lower Limit for Integral term limitation
  s32 wIntegral;
	
	s16 hReference;
	s16 hPresentFeedback;
	s16 hErr;
	s32 hOutput ;
  s32 OutPreSat; 
  u16 Ksat;
  s32 SatErr;
	
  // Actually used only if DIFFERENTIAL_TERM_ENABLED is enabled in
  //stm32f10x_MCconf.h
  s16 hKd_Gain;
  u16 hKd_Divisor;
  s32 wPreviousError;
} PID_Struct_t;


extern PID_Struct_t       PID_Speed_InitStruct[MOTOR_NUM];
extern PID_Struct_t		  PID_Current_InitStructure[MOTOR_NUM];

void PID_Init (PID_Struct_t *,PID_Struct_t *);
void PID_Speed_Coefficients_update(s16, PID_Struct_t *);
s16 PID_Regulator(s16, s16, PID_Struct_t *);

/* Exported variables ------------------------------------------------------- */

#endif 