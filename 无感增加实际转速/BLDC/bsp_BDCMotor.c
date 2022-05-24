#include "bsp_BDCMotor.h"
#include "main.h"
#include "mc_type.h"
#include "mc_config.h"
#include "pid.h"
#include "flash.h"
#include "function.h"
#include "Sensorless bldc.h"

/* ?????¨¤???¡§?? --------------------------------------------------------------*/
/* ?????¨º?¡§?? ----------------------------------------------------------------*/
/* ????¡À??? ------------------------------------------------------------------*/
TIM_HandleTypeDef htimx_BDCMOTOR;
MotorControlParameters_t MotorControl[];
u16 Motor_Prescaler = 83;
HALL_Study_t HALL_Study[2];
extern STM_Handle_t STM[NBR_OF_MOTORS];
extern PID_Handle_t PIDSpeed_BLDC_M1;
extern PID_Handle_t PIDSpeed_BLDC_M2;



void GetPhaseCurrentsM1(void);
int test6 = 0;
typedef signed long long s64;


void SetMotorSpeed(uint8_t Motor_NUM,int16_t PWM_Duty)
{
	
    if(PWM_Duty > PWM_PERIOD)
    {
        PWM_Duty = PWM_PERIOD ;
    }
    else if(PWM_Duty <- PWM_PERIOD)
    {
        PWM_Duty = -PWM_PERIOD ;
    }

    switch ( Motor_NUM )
    {
    case 0:	
	    Ramp_Speed(0);		
//        MotorControl[0].PWM_Duty = PID_Regulator(MotorControl[0].Speed_Ref,MotorControl[0].Speed_Real,&PID_Speed_InitStruct[0]);
        if((MotorControl[0].Speed_Ref < 0 && MotorControl[0].PWM_Duty > 0) || \
                (MotorControl[0].Speed_Ref >0 && MotorControl[0].PWM_Duty < 0) || MotorControl[0].Speed_Ref == 0 )
        {
            test6++;
            MotorControl[0].PWM_Duty = 0;
        }
         HAL_NVIC_DisableIRQ(EXTI0_IRQn);
		 HAL_NVIC_DisableIRQ(EXTI1_IRQn);
		 HAL_NVIC_DisableIRQ(EXTI2_IRQn);
        if(MotorControl[0].Speed_Ref < 0)
        {
            MotorControl[0].Hall.HallState_CCW = 0x07 ^MotorControl[0].Hall.HallState;
            BLDC1_PhaseChange( MotorControl[0].Hall.HallState_CCW,PWM_Duty);
        }
        else if(MotorControl[0].Speed_Ref > 0)
        {
            MotorControl[0].Hall.HallState_CW = MotorControl[0].Hall.HallState;
            BLDC1_PhaseChange( MotorControl[0].Hall.HallState_CW,PWM_Duty);
        }
        else
        {
            PWM_Duty =0;
            MotorControl[0].Hall.HallState_CW = MotorControl[0].Hall.HallState;
            BLDC1_PhaseChange( MotorControl[0].Hall.HallState_CW,PWM_Duty);
        }
        HAL_NVIC_EnableIRQ(EXTI0_IRQn);
		HAL_NVIC_EnableIRQ(EXTI1_IRQn);
		HAL_NVIC_EnableIRQ(EXTI2_IRQn);
        break;

    case 1:
        Ramp_Speed(1);
//        MotorControl[1].PWM_Duty = PID_Regulator(MotorControl[1].Speed_Ref,MotorControl[1].Speed_Real,&PID_Speed_InitStruct[1]);
        if((MotorControl[1].Speed_Ref < 0 && MotorControl[1].PWM_Duty > 0) || \
                (MotorControl[1].Speed_Ref >0 && MotorControl[1].PWM_Duty < 0) || MotorControl[1].Speed_Ref == 0 )
        {
            test6++;
            MotorControl[1].PWM_Duty = 0;
        }
        HAL_NVIC_DisableIRQ(EXTI15_10_IRQn);
        if(MotorControl[1].Speed_Ref < 0)
        {
            MotorControl[1].Hall.HallState_CCW = 0x07 ^ MotorControl[1].Hall.HallState;
            BLDC2_PhaseChange( MotorControl[1].Hall.HallState_CCW,PWM_Duty);
        }
        else if(MotorControl[1].Speed_Ref > 0)
        {
            MotorControl[1].Hall.HallState_CW = MotorControl[1].Hall.HallState;
            BLDC2_PhaseChange( MotorControl[1].Hall.HallState_CW,PWM_Duty);
        }
        else
        {
            PWM_Duty =0;
            MotorControl[1].Hall.HallState_CW = MotorControl[1].Hall.HallState;
            BLDC2_PhaseChange( MotorControl[1].Hall.HallState_CW,PWM_Duty);
        }
        HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);
        break;

    default:
        break;
    }

}



void SetMotorStop(uint8_t Motor_NUM)
{
    switch ( Motor_NUM )
    {
    case 0://???????¨²5
        TIM1->CCR1 = 0;
        TIM1->CCR2 = 0;
        TIM1->CCR3 = 0;
        MotorControl[M1].Hall.ChangeFlag = 1;
        MotorControl[M1].PWM_Duty = 0;
        PID_Speed_InitStruct[0].wIntegral  = 0;
		PID_Current_InitStructure[0].wIntegral = 0;
	    MotorControl[M1].Speed_Ref = 0;
		if(MotorControl[M1].Speed_Real ==0 )
        {
            TIM1->CCER = 0x1555;
        }
//		    PIDIq_BLDC_M1.wIntegralTerm = 0;
        break;

    case 1://???????¨²6
		 MotorControl[M2].Speed_Ref = 0;
		PID_Speed_InitStruct[1].wIntegral = 0;
		PID_Current_InitStructure[1].wIntegral = 0;
	    MotorControl[M2].PWM_Duty = 0;
        TIM8->CCR1 = 0;
        TIM8->CCR2 = 0;
        TIM8->CCR3 = 0;
        MotorControl[M2].Hall.ChangeFlag = 1;    
	    MotorControl[M2].Speed_Ref = 0;
	       if(MotorControl[M2].Speed_Real ==0 )
        {
            TIM8->CCER = 0x1555;
        }
//		    PIDIq_BLDC_M2.wIntegralTerm = 0;
        break;

    default:
        break;
    }
}

u8 Hall_Switch = 0;
u8 Hall_Switch1 = 0;
u8 Hall_Tab[] = {5,4,6,2,3,1};
u16 PWM_Complementary_Switch_M1 = 4000,PWM_Complementary_Switch_M2 = 4000;
void BLDC1_PhaseChange(u8 bHallState, s16 PWM_Duty)
{
    if(PWM_Duty < 0) PWM_Duty = -PWM_Duty;

    if(bHallState == 0)
    {
        MC_SetFault(M1_HALL_ERR);
    }
   else if(bHallState == HALL_Study[0].HallTab[4])
    {
		if( PWM_Duty< PWM_Complementary_Switch_M1 )  //BC
		{
			TIM1->CCER = 0x1410;
		}
		else
		{
			TIM1->CCER = 0x1450;
		}
		TIM1->CCR2 = PWM_PERIOD;
        TIM1->CCR1 = PWM_PERIOD;
        TIM1->CCR3 = PWM_PERIOD;
        TIM1->CCR2 = PWM_Duty;
    }

    else if(bHallState == HALL_Study[0].HallTab[5])   //BA
    {
		if( PWM_Duty< PWM_Complementary_Switch_M1 )
		{
			TIM1->CCER = 0x1014;
		}
		else
		{
			TIM1->CCER = 0x1054;
		}
		TIM1->CCR2 = PWM_PERIOD;
        TIM1->CCR1 = PWM_PERIOD;
        TIM1->CCR3 = PWM_PERIOD;
        TIM1->CCR2 = PWM_Duty;
    }
    else if(bHallState == HALL_Study[0].HallTab[0])
    {
		if( PWM_Duty< PWM_Complementary_Switch_M1 )   //CA
		{
			TIM1->CCER = 0x1104;
		}
		else
		{
			TIM1->CCER = 0x1504;
		}
		TIM1->CCR2 = PWM_PERIOD;
        TIM1->CCR1 = PWM_PERIOD;
        TIM1->CCR3 = PWM_PERIOD;
        TIM1->CCR3 = PWM_Duty;

    }
    else if(bHallState == HALL_Study[0].HallTab[1])
    {
		if( PWM_Duty< PWM_Complementary_Switch_M1 )  //CB
		{
			TIM1->CCER = 0x1140;
		}
		else
		{
			TIM1->CCER = 0x1540;
		}
		TIM1->CCR2 = PWM_PERIOD;
        TIM1->CCR1 = PWM_PERIOD;
        TIM1->CCR3 = PWM_PERIOD;
        TIM1->CCR3 = PWM_Duty;
    }
    else if(bHallState == HALL_Study[0].HallTab[2])
    {
		if( PWM_Duty< PWM_Complementary_Switch_M1 )   //AB
		{
			TIM1->CCER = 0x1041;
		}
		else
		{
			TIM1->CCER = 0x1045;
		}
		TIM1->CCR2 = PWM_PERIOD;
        TIM1->CCR1 = PWM_PERIOD;
        TIM1->CCR3 = PWM_PERIOD;
        TIM1->CCR1 = PWM_Duty;
    }

    else if(bHallState == HALL_Study[0].HallTab[3])   //AC
    {
		if( PWM_Duty< PWM_Complementary_Switch_M1 )
		{
			TIM1->CCER = 0x1401;
		}
		else
		{
			TIM1->CCER = 0x1405;
		}
		TIM1->CCR2 = PWM_PERIOD;
        TIM1->CCR1 = PWM_PERIOD;
        TIM1->CCR3 = PWM_PERIOD;
        TIM1->CCR1 = PWM_Duty;
    }
    else if(bHallState == 7)
    {
        MC_SetFault(M1_HALL_ERR);
    }
}

void BLDC2_PhaseChange(u8 bHallState, s16 PWM_Duty)
{
    if(PWM_Duty < 0) PWM_Duty = -PWM_Duty;

    if(bHallState == 0)
    {
        MC_SetFault(M2_HALL_ERR);
    }
   else if(bHallState == HALL_Study[1].HallTab[4])
    {
		if( PWM_Duty< PWM_Complementary_Switch_M1 )
		{
			TIM8->CCER = 0x1410;
		}
		else
		{
			TIM8->CCER = 0x1450;
		}
		TIM8->CCR2 = PWM_PERIOD;
        TIM8->CCR1 = PWM_PERIOD;
        TIM8->CCR3 = PWM_PERIOD;
        TIM8->CCR2 = PWM_Duty;
    }
    else if(bHallState == HALL_Study[1].HallTab[5])
    {
		if( PWM_Duty< PWM_Complementary_Switch_M1 )
		{
			TIM8->CCER = 0x1014;
		}
		else
		{
			TIM8->CCER = 0x1054;
		}
		TIM8->CCR2 = PWM_PERIOD;
        TIM8->CCR1 = PWM_PERIOD;
        TIM8->CCR3 = PWM_PERIOD;
        TIM8->CCR2 = PWM_Duty;
    }
    else if(bHallState == HALL_Study[1].HallTab[0])
    {
		if( PWM_Duty< PWM_Complementary_Switch_M1 )
		{
			TIM8->CCER = 0x1104;
		}
		else
		{
			TIM8->CCER = 0x1504;
		}
		TIM8->CCR2 = PWM_PERIOD;
        TIM8->CCR1 = PWM_PERIOD;
        TIM8->CCR3 = PWM_PERIOD;
        TIM8->CCR3 = PWM_Duty;

    }
    else if(bHallState == HALL_Study[1].HallTab[1])
    {
		if( PWM_Duty< PWM_Complementary_Switch_M1 )
		{
			TIM8->CCER = 0x1140;
		}
		else
		{
			TIM8->CCER = 0x1540;
		}
		TIM8->CCR2 = PWM_PERIOD;
        TIM8->CCR1 = PWM_PERIOD;
        TIM8->CCR3 = PWM_PERIOD;
        TIM8->CCR3 = PWM_Duty;
    }
    else if(bHallState == HALL_Study[1].HallTab[2])
    {
		if( PWM_Duty< PWM_Complementary_Switch_M1 )
		{
			TIM8->CCER = 0x1041;
		}
		else
		{
			TIM8->CCER = 0x1045;
		}
		TIM8->CCR2 = PWM_PERIOD;
        TIM8->CCR1 = PWM_PERIOD;
        TIM8->CCR3 = PWM_PERIOD;
        TIM8->CCR1 = PWM_Duty;
    }

    else if(bHallState == HALL_Study[1].HallTab[3])
    {
		if( PWM_Duty< PWM_Complementary_Switch_M1 )
		{
			TIM8->CCER = 0x1401;
		}
		else
		{
			TIM8->CCER = 0x1405;
		}
		TIM8->CCR2 = PWM_PERIOD;
        TIM8->CCR1 = PWM_PERIOD;
        TIM8->CCR3 = PWM_PERIOD;
        TIM8->CCR1 = PWM_Duty;
    }
    else if(bHallState == 7)
    {
        MC_SetFault(M2_HALL_ERR);
    }
}

void HALLSTUDY_PhaseChange0(u8 bHallState)
{
	static int test7;
    switch (bHallState)//HPWMLpwm
    {
    case 0:

        MC_SetFault(M1_HALL_ERR);

        break;

    case 6:
        TIM1->CCER = 0x1414;//  TIM8->CCER = 0x1bae;//CA    0x1c9c
	    TIM1->CCR1 = HALL_Study[M1].HallCommPWM;
	    TIM1->CCR3 = HALL_Study[M1].HallCommPWM;      
	    test7++;
	    if(test7==50)
        TIM1->CCR2 = HALL_Study[M1].HallCommPWM;

        break;

    case 4:
        TIM1->CCER = 0x1114;//TIM8->CCER = 0x1bea;//CB
	    TIM1->CCR1 = HALL_Study[M1].HallCommPWM;
	    test7++;
	    if(test7==100)
		{
	    TIM1->CCR2 = HALL_Study[M1].HallCommPWM;      
        TIM1->CCR3 = HALL_Study[M1].HallCommPWM;
		}
        break;

    case 5:
        TIM1->CCER = 0x1144;//TIM8->CCER = 0x1aeb;//AB
	    TIM1->CCR1 = HALL_Study[M1].HallCommPWM;
	    TIM1->CCR2 = HALL_Study[M1].HallCommPWM;    
         test7++;
	    if(test7==50)	
        TIM1->CCR3 = HALL_Study[M1].HallCommPWM;

        break;

    case 1:
        TIM1->CCER = 0x1141;//TIM8->CCER = 0x1eab;//AC
	    TIM1->CCR2 = HALL_Study[M1].HallCommPWM;
	   test7++;
	    if(test7==50)
		{
	    TIM1->CCR3 = HALL_Study[M1].HallCommPWM;      
        TIM1->CCR1 = HALL_Study[M1].HallCommPWM;
		}
        break;

    case 3:
        TIM1->CCER = 0x1441;//TIM8->CCER = 0x1eba;//BC
	    TIM1->CCR2 = HALL_Study[M1].HallCommPWM;
	    TIM1->CCR3 = HALL_Study[M1].HallCommPWM;    
        test7++;
	    if(test7==50)	
        TIM1->CCR1 = HALL_Study[M1].HallCommPWM;

        break;

    case 2:
        TIM1->CCER = 0x1411;//TIM8->CCER = 0x1abe;//BA
	    TIM1->CCR3 = HALL_Study[M1].HallCommPWM;
	    test7++;
	    if(test7==50)
		{
	    TIM1->CCR2 = HALL_Study[M1].HallCommPWM;      
        TIM1->CCR1 = HALL_Study[M1].HallCommPWM;
		}
        break;

    case 7:
        MC_SetFault(M1_HALL_ERR);

        break;
    default:
        break;
    }

}
void HALLSTUDY_PhaseChange1(u8 bHallState)
{
    switch (bHallState)//HPWMLpwm
    {
    case 0:

        MC_SetFault(M2_HALL_ERR);

        break;

    case 6:
        TIM8->CCER = 0x1414;//  TIM8->CCER = 0x1bae;//CA    0x1c9c

        break;

    case 4:
        TIM8->CCER = 0x1114;//TIM8->CCER = 0x1bea;//CB

        break;

    case 5:
        TIM8->CCER = 0x1144;//TIM8->CCER = 0x1aeb;//AB

        break;

    case 1:
        TIM8->CCER = 0x1141;//TIM8->CCER = 0x1eab;//AC

        break;

    case 3:
        TIM8->CCER = 0x1441;//TIM8->CCER = 0x1eba;//BC

        break;

    case 2:
        TIM8->CCER = 0x1411;//TIM8->CCER = 0x1abe;//BA

        break;

    case 7:
        MC_SetFault(M2_HALL_ERR);

        break;
    default:
        break;
    }

}

s16 GetMotorSpeed(u8 Mortor_NUM)
{
    MotorControl[Mortor_NUM].Speed_Real = 1000000 *60/(MotorControl[Mortor_NUM].Hall.HALL_CaptureValueDelta * MotorControl[Mortor_NUM].Pole_Paires)*MotorControl[Mortor_NUM].Direction;
    Sensorless[Mortor_NUM].Speed_Real = 1000000 *60/(Sensorless[Mortor_NUM].Hall.HALL_CaptureValueDelta * 4)*1;
    if(HALL_OVF_Counter -MotorControl[Mortor_NUM].Hall.PreHALL_OVF_Counter > 3)
    {
        MotorControl[Mortor_NUM].Speed_Real = 0 ;
//		Sensorless[Mortor_NUM].Speed_Real = 0 ;
    }

    return MotorControl[Mortor_NUM].Speed_Real;
	return Sensorless[Mortor_NUM].Speed_Real;
}


s16 Ramp_Speed(u8 Mortor_NUM)
{

    if(MotorControl[Mortor_NUM].Speed_Set > MotorControl[Mortor_NUM].Speed_Ref)
    {
        MotorControl[Mortor_NUM].Speed_Ref += MotorControl[Mortor_NUM].Acceleration;
        if(MotorControl[Mortor_NUM].Speed_Set < MotorControl[Mortor_NUM].Speed_Ref)
        {
            MotorControl[Mortor_NUM].Speed_Ref = MotorControl[Mortor_NUM].Speed_Set;
        }

    }
    else if(MotorControl[Mortor_NUM].Speed_Set< MotorControl[Mortor_NUM].Speed_Ref)
    {
        MotorControl[Mortor_NUM].Speed_Ref -= MotorControl[Mortor_NUM].Deceleration;
        if(MotorControl[Mortor_NUM].Speed_Set > MotorControl[Mortor_NUM].Speed_Ref)
        {
            MotorControl[Mortor_NUM].Speed_Ref = MotorControl[Mortor_NUM].Speed_Set;
        }

    }


}


s16 Ramp_PPWM(u8 Mortor_NUM)
{
    if(MotorControl[Mortor_NUM].PWM_DutySet > MotorControl[Mortor_NUM].PWM_Duty)
    {
        MotorControl[Mortor_NUM].PWM_Duty += MotorControl[Mortor_NUM].Acceleration;
        if(MotorControl[Mortor_NUM].PWM_DutySet < MotorControl[Mortor_NUM].PWM_Duty)
        {
            MotorControl[Mortor_NUM].PWM_Duty = MotorControl[Mortor_NUM].PWM_DutySet;
        }

    }
    else if(MotorControl[Mortor_NUM].PWM_DutySet< MotorControl[Mortor_NUM].PWM_Duty)
    {
        MotorControl[Mortor_NUM].PWM_Duty -= MotorControl[Mortor_NUM].Deceleration;
        if(MotorControl[Mortor_NUM].PWM_DutySet > MotorControl[Mortor_NUM].PWM_Duty)
        {
            MotorControl[Mortor_NUM].PWM_Duty = MotorControl[Mortor_NUM].PWM_DutySet;
        }

    }

}

u8 HALL_GetPhase2(void)
{
    int32_t tmp = 0;
    tmp |= HAL_GPIO_ReadPin(M2_HALL_H1_GPIO_Port, M2_HALL_H1_Pin);//U(A)
    tmp <<= 1;
    tmp |= HAL_GPIO_ReadPin(M2_HALL_H2_GPIO_Port, M2_HALL_H2_Pin);//V(B)
    tmp <<= 1;
    tmp |= HAL_GPIO_ReadPin(M2_HALL_H3_GPIO_Port, M2_HALL_H3_Pin);//W(C)
    return (u8)(tmp & 0x0007); // ????????
}

u8 HALL_GetPhase1(void)
{
    int32_t tmp = 0;

    tmp |= HAL_GPIO_ReadPin(M1_HALL_H1_GPIO_Port, M1_HALL_H1_Pin);//U(A)
    tmp <<= 1;
    tmp |= HAL_GPIO_ReadPin(M1_HALL_H2_GPIO_Port,M1_HALL_H2_Pin);//V(B)
    tmp <<= 1;
    tmp |= HAL_GPIO_ReadPin(M1_HALL_H3_GPIO_Port,M1_HALL_H3_Pin);//W(C)
    return (u8)(tmp & 0x0007); // ????????
}


void GetPhaseCurrentsM1(void)
{
	  s32 TempValue,hError;
//	  MotorControl[M1].Current.GetADCValue =MotorControl[M1].Current.PhaseACurrent = ADC2->JDR1 - MotorControl[M1].Current.PhaseAOffset;
//	  MotorControl[M2].Current.GetADCValue =MotorControl[M2].Current.PhaseACurrent = ADC1->JDR1- MotorControl[M2].Current.PhaseAOffset ;
	
	
	
//    MotorControl[M1].Current.PhaseACurrent = ADC1->JDR1 - MotorControl[M1].Current.PhaseAOffset ;
//		MotorControl[M1].Current.PhaseBCurrent = ADC1->JDR2 - MotorControl[M1].Current.PhaseBOffset ;
//		MotorControl[M1].Current.PhaseCCurrent = ADC1->JDR3 - MotorControl[M1].Current.PhaseCOffset ;
//	
//		MotorControl[M2].Current.PhaseACurrent = ADC2->JDR1 - MotorControl[M2].Current.PhaseAOffset ;
//		MotorControl[M2].Current.PhaseBCurrent = ADC2->JDR2 - MotorControl[M2].Current.PhaseBOffset ;
//		MotorControl[M2].Current.PhaseCCurrent = ADC2->JDR3 - MotorControl[M2].Current.PhaseCOffset ;
//	
//   
//    if(MotorControl[M1].Hall.HallStateValue == HALL_Study[M1].HallTab[4]) //BC
//    {
//       MotorControl[M1].Current.GetADCValue =  MotorControl[M1].Current.PhaseCCurrent;
//    }
//    else if(MotorControl[M1].Hall.HallStateValue == HALL_Study[M1].HallTab[5]) //BA
//    {
//      MotorControl[M1].Current.GetADCValue =  MotorControl[M1].Current.PhaseACurrent;
//    }
//    else if(MotorControl[M1].Hall.HallStateValue == HALL_Study[M1].HallTab[0]) //CA
//    {
//      MotorControl[M1].Current.GetADCValue =  MotorControl[M1].Current.PhaseACurrent;
//    }
//    else if(MotorControl[M1].Hall.HallStateValue == HALL_Study[M1].HallTab[1]) //CB
//    {
//       MotorControl[M1].Current.GetADCValue =  MotorControl[M1].Current.PhaseBCurrent;
//    }
//    else if(MotorControl[M1].Hall.HallStateValue == HALL_Study[M1].HallTab[2]) //AB
//    {
//        MotorControl[M1].Current.GetADCValue =  MotorControl[M1].Current.PhaseBCurrent;
//    }
//    else if(MotorControl[M1].Hall.HallStateValue == HALL_Study[M1].HallTab[3]) //AC
//    {
//        MotorControl[M1].Current.GetADCValue =  MotorControl[M1].Current.PhaseCCurrent;
//    }

//		
//		
//    if(MotorControl[M2].Hall.HallStateValue == HALL_Study[M2].HallTab[4]) //BC
//    {
//       MotorControl[M2].Current.GetADCValue =  MotorControl[M2].Current.PhaseCCurrent;
//    }
//    else if(MotorControl[M2].Hall.HallStateValue == HALL_Study[M2].HallTab[5]) //BA
//    {
//      MotorControl[M2].Current.GetADCValue =  MotorControl[M2].Current.PhaseACurrent;
//    }
//    else if(MotorControl[M2].Hall.HallStateValue == HALL_Study[M2].HallTab[0]) //CA
//    {
//      MotorControl[M2].Current.GetADCValue =  MotorControl[M2].Current.PhaseACurrent;
//    }
//    else if(MotorControl[M2].Hall.HallStateValue == HALL_Study[M2].HallTab[1]) //CB
//    {
//       MotorControl[M2].Current.GetADCValue =  MotorControl[M2].Current.PhaseBCurrent;
//    }
//    else if(MotorControl[M2].Hall.HallStateValue == HALL_Study[M2].HallTab[2]) //AB
//    {
//        MotorControl[M2].Current.GetADCValue =  MotorControl[M2].Current.PhaseBCurrent;
//    }
//    else if(MotorControl[M2].Hall.HallStateValue == HALL_Study[M2].HallTab[3]) //AC
//    {
//        MotorControl[M2].Current.GetADCValue =  MotorControl[M2].Current.PhaseCCurrent;
//    }

//		if(Uart_TxBuffer_CNT1 < 2000)
//		{
//			LV_Uart_TxBuffer[0][Uart_TxBuffer_CNT1] = MotorControl[M2].Current.PhaseACurrent  ;
//			LV_Uart_TxBuffer[1][Uart_TxBuffer_CNT1] = MotorControl[M2].Current.PhaseBCurrent  ;
//			LV_Uart_TxBuffer[2][Uart_TxBuffer_CNT1] = MotorControl[M2].Current.PhaseCCurrent  ;
//			LV_Uart_TxBuffer[3][Uart_TxBuffer_CNT1] = MotorControl[M2].Current.GetADCValue  ;
//		}
//		Uart_TxBuffer_CNT1++;
//		if(Uart_TxBuffer_CNT1 >1999)
//		{
//			Uart_TxBuffer_CNT1 = 2000 ;
//		}
						
  
		
//		if( STM[M1].bState == RUN) 
//		{
//			hError = MotorControl[M1].Current.ADCValue_Ref -  MotorControl[M1].Current.GetADCValue ;
//			MotorControl[M1].PWM_Duty = PI_Controller( &PIDIq_BLDC_M1, ( int32_t )hError );
//			
//			
////			if(MotorControl[M1].PWM_Duty<6000)MotorControl[M1].PWM_Duty++;
//			
//			
//			if((MotorControl[M1].Speed_Ref < 0 && MotorControl[M1].PWM_Duty > 0) || \
//						(MotorControl[M1].Speed_Ref >0 && MotorControl[M1].PWM_Duty < 0) || MotorControl[M1].Speed_Ref == 0 )
//			{
//				MotorControl[M1].PWM_Duty = 0;
//			}
//			else
//			{
//			  MotorControl[M1].PWM_Duty = PI_Controller( &PIDIq_BLDC_M1, ( int32_t )hError );
//			}

//			if(MotorControl[M1].Motor_Start_Stop == ENABLE)
//			{
//				if(MotorControl[M1].Fault_Flag == 0)
//				{
//						SetMotorSpeed(M1, MotorControl[M1].PWM_Duty);
//				}
//				else
//				{
//						MotorControl[M1].Motor_Start_Stop = DISABLE;
////						MotorControl[M1].Fault_Flag =0;
//				}
//			}
//			else if(MotorControl[M1].Motor_Start_Stop == DISABLE)
//			{
//				SetMotorStop(M1);
//			}
//    }
		
//		if( STM[M2].bState == RUN) 
//		{
//			hError = MotorControl[M2].Current.ADCValue_Ref -  MotorControl[M2].Current.GetADCValue ;
//			
////			MotorControl[M2].PWM_DutySet = PI_Controller( &PIDIq_BLDC_M2, ( int32_t )hError );
//			
//			if(MotorControl[M2].PWM_DutySet - MotorControl[M2].PWM_Duty > 50)
//			{
//			  MotorControl[M2].PWM_Duty += 50 ;
//			}
//			else if(MotorControl[M2].PWM_DutySet - MotorControl[M2].PWM_Duty < -50)
//			{
//			  MotorControl[M2].PWM_Duty -= 50 ;
//			}
//			else
//			{
//			   MotorControl[M2].PWM_Duty = MotorControl[M2].PWM_DutySet ;
//			}
//			
////			if(MotorControl[M2].PWM_Duty<6000)MotorControl[M2].PWM_Duty++;

//			if((MotorControl[M2].Speed_Ref < 0 && MotorControl[M2].PWM_Duty > 0) )
//			{
//				  MotorControl[M2].Speed_Ref = 0;
//					MotorControl[M2].PWM_Duty -= 5;
//				  if(MotorControl[M2].PWM_Duty < 0)MotorControl[M2].PWM_Duty = 0;
//					PIDSpeed_BLDC_M2.wIntegralTerm = 0;
//					PIDIq_BLDC_M2.wIntegralTerm = 0;
//			}
//			else if((MotorControl[M2].Speed_Ref >0 && MotorControl[M2].PWM_Duty < 0))
//			{
//				 MotorControl[M2].Speed_Ref = 0;
//					MotorControl[M2].PWM_Duty += 5;
//				  if(MotorControl[M2].PWM_Duty > 0)MotorControl[M2].PWM_Duty = 0;
//					PIDSpeed_BLDC_M2.wIntegralTerm = 0;
//					PIDIq_BLDC_M2.wIntegralTerm = 0;
//			}
//			else
//			{
//					MotorControl[M2].PWM_Duty = PI_Controller( &PIDIq_BLDC_M2, ( int32_t )hError );
//			}


//			if(MotorControl[M2].Motor_Start_Stop == ENABLE)
//			{
//				if(MotorControl[M2].Fault_Flag == 0)
//				{
//						SetMotorSpeed(M2, MotorControl[M2].PWM_Duty);
//				}
//				else
//				{
//						MotorControl[M2].Motor_Start_Stop = DISABLE;
////						MotorControl[M2].Fault_Flag =0;
//				}
//			}
//			else if(MotorControl[M2].Motor_Start_Stop == DISABLE)
//			{
//				SetMotorStop(M2);
//			}
//    }
						
		
		for(u8 i=0; i< MOTOR_NUM ; i++)
		{
				CurrentLimit(i);
		}

//    ADC1->JSQR = PHASE_A_MSK + HALL_MSK1 + SEQUENCE_LENGHT;//Ia
//    ADC2->JSQR = PHASE_B_MSK + HALL_MSK2 + SEQUENCE_LENGHT;//


}

void HallStudyHandle0(void)
{
    switch(HALL_Study[M1].CommuntionState)
    {
    case 0:
        if(HALL_Study[M1].StudySectorCnt < 200)
        {
            u8 HALL_Studytemp = 0;
            if(++HALL_Study[M1].StudySectorCnt2 > HALL_Study[M1].StudySectorCnt3 )
            {
                HALL_Study[M1].StudySectorCnt3-=1;
                HALL_Study[M1].StudySectorCnt2 =0;
				if(HALL_Study[M1].StudySectorCnt3<20)
				{
					HALL_Study[M1].StudySectorCnt3=20;
				}

                HALL_Study[M1].HallSector ++;
                if(HALL_Study[M1].HallSector>6)
                {
                    HALL_Study[M1].HallSector = 1;
                }
                if(HALL_Study[M1].HallSector == 1) HALL_Studytemp = 6;
                if(HALL_Study[M1].HallSector == 2) HALL_Studytemp = 4;
                if(HALL_Study[M1].HallSector == 3) HALL_Studytemp = 5;
                if(HALL_Study[M1].HallSector == 4) HALL_Studytemp = 1;
                if(HALL_Study[M1].HallSector == 5) HALL_Studytemp = 3;
                if(HALL_Study[M1].HallSector == 6) HALL_Studytemp = 2;
                HALL_Study[M1].StudySectorCnt++;
//				 TIM1->CCR1 = HALL_Study[M1].HallCommPWM;
//				TIM1->CCR3 = HALL_Study[M1].HallCommPWM;
//                TIM1->CCR2 = HALL_Study[M1].HallCommPWM;
//                HALLSTUDY_PhaseChange0(HALL_Studytemp);
				BLDC1_PhaseChange(HALL_Studytemp,3000);
            }

        }
        else
        {
            HALL_Study[M1].CommuntionState = 1;
        }
        break;

    case 1:
        if(HALL_Study[M1].HallTab[0]!=HALL_Study[M1].HallTab[1]&&
                HALL_Study[M1].HallTab[1]!=HALL_Study[M1].HallTab[2]&&
                HALL_Study[M1].HallTab[2]!=HALL_Study[M1].HallTab[3]&&
                HALL_Study[M1].HallTab[3]!=HALL_Study[M1].HallTab[4]&&
                HALL_Study[M1].HallTab[4]!=HALL_Study[M1].HallTab[5])
        {
            MC_ClearFault(M1_HALL_ERR);
            HALL_Study[M1].CommuntionState = 3;
        }
        else
        {
            HALL_Study[M1].CommuntionState = 2;
        }
        break;

    case 2:
        /*HALL?¨ª?¨®*/
        MC_SetFault(M1_HALL_ERR);
        HALL_Study[M1].CommuntionState = 3;

        break;
    case 3:
        break;
    }

}

void HallStudyHandle1(void)
{
    switch(HALL_Study[M2].CommuntionState)
    {
    case 0:
        if(HALL_Study[M2].StudySectorCnt < 12)
        {
            u8 HALL_Studytemp = 0;
            if(++HALL_Study[M2].StudySectorCnt2 > HALL_Study[M2].StudySectorCnt3 )
            {

                HALL_Study[M2].StudySectorCnt2 =0;

                HALL_Study[M2].HallSector ++;
                if(HALL_Study[M2].HallSector>6)
                {
                    HALL_Study[M2].HallSector = 1;
                }
                if(HALL_Study[M2].HallSector == 1) HALL_Studytemp = 6;
                if(HALL_Study[M2].HallSector == 2) HALL_Studytemp = 4;
                if(HALL_Study[M2].HallSector == 3) HALL_Studytemp = 5;
                if(HALL_Study[M2].HallSector == 4) HALL_Studytemp = 1;
                if(HALL_Study[M2].HallSector == 5) HALL_Studytemp = 3;
                if(HALL_Study[M2].HallSector == 6) HALL_Studytemp = 2;
                HALL_Study[M2].StudySectorCnt++;

                TIM8->CCR1 = HALL_Study[M2].HallCommPWM;
                TIM8->CCR2 = HALL_Study[M2].HallCommPWM;
                TIM8->CCR3 = HALL_Study[M2].HallCommPWM;
                HALLSTUDY_PhaseChange1(HALL_Studytemp);
            }

        }
        else
        {
            HALL_Study[M2].CommuntionState = 1;
        }
        break;

    case 1:
        if(HALL_Study[M2].HallTab[0]!=HALL_Study[M2].HallTab[1]&&
                HALL_Study[M2].HallTab[1]!=HALL_Study[M2].HallTab[2]&&
                HALL_Study[M2].HallTab[2]!=HALL_Study[M2].HallTab[3]&&
                HALL_Study[M2].HallTab[3]!=HALL_Study[M2].HallTab[4]&&
                HALL_Study[M2].HallTab[4]!=HALL_Study[M2].HallTab[5])
        {
            MC_ClearFault(M2_HALL_ERR);
            HALL_Study[M2].CommuntionState = 3;
        }
        else
        {
            HALL_Study[M2].CommuntionState = 2;
        }
        break;

    case 2:
        /*HALL?¨ª?¨®*/
        MC_SetFault(M2_HALL_ERR);
        HALL_Study[M2].CommuntionState = 3;

        break;
    case 3:

        break;

    }

}



