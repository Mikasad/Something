#include "function.h"
#include "stm32g4xx_hal.h"
#include "bsp_BDCMotor.h"
#include "main.h"
//#include "canopen_pdo.h"
#include "Agreement.h"
#include "flash.h"
#include "pid.h"
#include "Sensorless bldc.h"
extern PID_Struct_t   PID_Current_InitStructure[2];
uint32_t DisplayCounter = 0;
VoltVar_TypeDef VoltVar;
u32 wGlobal_Flags = 0;
u32 FaultOccurred = 0;
u32 wGlobal_Flags1;
u32 FaultOccurred1 = 0;
u32 PrewGlobal_Flags;
long l_abs(long a);
void LEDSet(u8 led1,u8 led_time,u8 led_mode,u8 led_speed);
void MotorA_Default_Phase_Cheek(void);
void MotorB_Default_Phase_Cheek(void);
void MotorA_Default_Phase_Cheek_IT(void);
void MotorB_Default_Phase_Cheek_IT(void);
void BLDC_Stuck_Chk(void);
void BLDC1_OverSpdChk(void);
void BLDC2_OverSpdChk(void);
#define LED_ON		1
#define	LED_OFF		0
#define	PHASE_ERR_MAX 500
DefaultPhaseCheekCurTemp MotorA_Current_temp;
DefaultPhaseCheekCurTemp MotorB_Current_temp;
#define PB10_Toggle        HAL_GPIO_TogglePin(GPIOB,GPIO_PIN_10)
//void MC_SetFault(u32 hFault_type)
//{
//    wGlobal_Flags |= hFault_type;
//    if(FaultOccurred !=0)
//    {
//        FaultOccurred |=hFault_type;
//        FaultOccurred &=hFault_type;
//    }
//    else
//        FaultOccurred  |= hFault_type; //只报警当前错误类型
//}
//void MC_ClearFault(u32 hFault_type)
//{
//    wGlobal_Flags &= ~hFault_type;
//    FaultOccurred &= ~hFault_type; //清除当前错误类型
//}
long l_abs(long a)
{
    long temp;
    if(a>0) temp = a;
    else if(a<0) temp = -a;

    return temp;
}
void MC_SetFault(u32 hFault_type)
{
	u8 i;
    wGlobal_Flags |= hFault_type;
		if(wGlobal_Flags!=PrewGlobal_Flags&&(wGlobal_Flags == PrewGlobal_Flags+(PrewGlobal_Flags^wGlobal_Flags)))//有新错误且不是已经有的报警
		{
			for(i=9;i>0;i--)													//add by diamond 2021.1125
			{
				*PARAMETER[211+i].lpParam = *PARAMETER[211+i-1].lpParam;		//所有事件按顺序存入数组，清除最久的报警
			}
			for(i=9;i>0;i--)
			{
				*PARAMETER[221+i].lpParam = *PARAMETER[221+i-1].lpParam;		//事件的嘀嗒
			}		
			*PARAMETER[211].lpParam = wGlobal_Flags;
			*PARAMETER[221].lpParam = uwTick;
			Flash_Writesign =1;		//FLASH写入标志位
		}
		PrewGlobal_Flags = wGlobal_Flags;
//    sendPDOevent(&CANopen_Drive);
    if(FaultOccurred !=0)
    {
        FaultOccurred |=hFault_type;
        FaultOccurred &=hFault_type;
    }
    else
        FaultOccurred  |= hFault_type; //只报警当前错误类型
}
void MC_SetFault1(u32 hFault_type)
{
    wGlobal_Flags1 |= hFault_type;
    if(FaultOccurred1 !=0)
    {
        FaultOccurred1 |=hFault_type;
        FaultOccurred1 &=hFault_type;
    }
    else
        FaultOccurred1  |= hFault_type; //只报警当前错误类型
}

void MC_ClearFault(u32 hFault_type)
{
    wGlobal_Flags &= ~hFault_type;
//    sendPDOevent(&CANopen_Drive);
    FaultOccurred &= ~hFault_type; //清除当前错误类型

}

/*
Function:led灯设置
Input   :led1: 1/红灯亮  0/红灯不亮
				 led_time:闪烁的次数
				 led_mode:1/慢闪一次 0/快闪
				 led_speed:闪烁的速度
Output  :No
Explain :No
------------------------------------------------*/
void LEDSet(u8 led1,u8 led_time,u8 led_mode,u8 led_speed)
{			
	    if(led1==0)
		RED_LED1_OFF;
		else if(led_mode)	//慢闪
		{
			if(DisplayCounter<2*led_speed)
				RED_LED1_ON;
			else if(DisplayCounter%(2*led_speed)==0&&DisplayCounter<4*led_time*led_speed)
				RED_LED1_TOGGLE;
		}
		else if(DisplayCounter<=(2*led_time)*led_speed)	//快闪
		{
				if((DisplayCounter-1)%led_speed==0)
				{
						if((DisplayCounter/led_speed)%2==0)
						{
								RED_LED1_ON;
						}
						else
								RED_LED1_OFF;
				}
		}
		else if(DisplayCounter<2*(led_time+1)*led_speed)
		{
				RED_LED1_OFF;
		}
		
		//time clear
		if(led_mode)
		{
			if(DisplayCounter>(4*led_time+2)*led_speed)
				DisplayCounter=0;
		}
		else if(DisplayCounter>(2*led_time+2)*led_speed)
				DisplayCounter=0;
}


void Over_VoltageCheck(void)  //10ms
{
    if(VoltVar.BUS>VoltVar.BusHigh)
    {
        VoltVar.BusHighCnt++;
        if(VoltVar.BusHighCnt>VoltVar.HighGuardTime)
        {
            VoltVar.BusHighCnt   = VoltVar.HighGuardTime+50;
            for(u8 i=0; i < MOTOR_NUM; i++)
            {
                MotorControl[i].Motor_Start_Stop = DISABLE;
            }
            MC_SetFault(OVER_VOLTAGE);
        }
    }
    else if(VoltVar.BUS < VOLT_300V )   //OVR
    {
        if(VoltVar.BusHighCnt>0)
        {
            VoltVar.BusHighCnt--;
            if(VoltVar.BusHighCnt < VoltVar.HighGuardTime)
            {
                MC_ClearFault(OVER_VOLTAGE);
            }
        }
    }

    if(VoltVar.BUS<VoltVar.BusLow)
    {
        if(++VoltVar.BusLowCnt>VoltVar.LowGuardTime)
        {
            VoltVar.BusLowCnt = VoltVar.LowGuardTime+50;
            for(u8 i=0; i < MOTOR_NUM; i++)
            {
                MotorControl[i].Motor_Start_Stop = DISABLE;
            }
            MC_SetFault(UNDER_VOLTAGE);
        }
    }
    else if(VoltVar.BUS > VOLT_200V)
    {
        if(VoltVar.BusLowCnt>0) //UVR
        {
            VoltVar.BusLowCnt--;
            if(VoltVar.BusLowCnt < VoltVar.LowGuardTime)
            {
                MC_ClearFault(UNDER_VOLTAGE);
            }
        }
    }
}

extern void SetMotorStop();
void CurrentLimit(u8 Motor_NUM)
{
    if(MotorControl[Motor_NUM].Current.FilterValue > MotorControl[Motor_NUM].Current.MaxValue1)
    {
        MotorControl[Motor_NUM].Current.OFCnt1++;
        if(MotorControl[Motor_NUM].Current.OFCnt1 > MotorControl[Motor_NUM].Current.OFCnt1_T)
        {
//                SetMotorStop(Motor_NUM);
//                MC_SetFault(OVER_CURRENT(Motor_NUM));
//                MotorControl[Motor_NUM].Fault_Flag = 1;
//                MotorControl[Motor_NUM].Current.OFCnt1 = 0;
        }
        if(MotorControl[Motor_NUM].Current.FilterValue > MotorControl[Motor_NUM].Current.MaxValue2)
        {
            MotorControl[Motor_NUM].Current.OFCnt2++;
            if(MotorControl[Motor_NUM].Current.OFCnt2 > MotorControl[Motor_NUM].Current.OFCnt2_T)
            {
                SetMotorStop(Motor_NUM);
                MC_SetFault(OVER_CURRENT(Motor_NUM));
                MotorControl[Motor_NUM].Fault_Flag = 1;
                MotorControl[Motor_NUM].Current.OFCnt2 = 0;
            }
            if(MotorControl[Motor_NUM].Current.FilterValue > MotorControl[Motor_NUM].Current.MaxValue3)
            {
                MotorControl[Motor_NUM].Current.OFCnt3++;
                if(MotorControl[Motor_NUM].Current.OFCnt3 > MotorControl[Motor_NUM].Current.OFCnt3_T)
                {
                    SetMotorStop(Motor_NUM);
                    MC_SetFault(OVER_CURRENT(Motor_NUM));
                    MotorControl[Motor_NUM].Fault_Flag = 1;
                    MotorControl[Motor_NUM].Current.OFCnt3 = 0;
                }
                if(MotorControl[Motor_NUM].Current.FilterValue > MotorControl[Motor_NUM].Current.MaxValue4)
                {
                    MotorControl[Motor_NUM].Current.OFCnt4++;
                    if(MotorControl[Motor_NUM].Current.OFCnt4 > MotorControl[Motor_NUM].Current.OFCnt4_T)
                    {
                        SetMotorStop(Motor_NUM);
                        MC_SetFault(OVER_CURRENT(Motor_NUM));
                        MotorControl[Motor_NUM].Fault_Flag = 1;
                        MotorControl[Motor_NUM].Current.OFCnt4 = 0;
                    }
                }
                else if(MotorControl[Motor_NUM].Current.OFCnt4 > 0)
                {
                    MotorControl[Motor_NUM].Current.OFCnt4--;
                }
            }
            else if(MotorControl[Motor_NUM].Current.OFCnt3 > 0)
            {
                MotorControl[Motor_NUM].Current.OFCnt3--;
            }
        }
        else if(MotorControl[Motor_NUM].Current.OFCnt2 > 0)
        {
            MotorControl[Motor_NUM].Current.OFCnt2--;
        }
    }
    else if(MotorControl[Motor_NUM].Current.OFCnt1 > 0)
    {
        MotorControl[Motor_NUM].Current.OFCnt1--;
        if(MotorControl[Motor_NUM].Current.OFCnt2 > 0)
        {
            MotorControl[Motor_NUM].Current.OFCnt2--;
        }
        if(MotorControl[Motor_NUM].Current.OFCnt3 > 0)
        {
            MotorControl[Motor_NUM].Current.OFCnt3--;
        }
        if(MotorControl[Motor_NUM].Current.OFCnt4 > 0)
        {
            MotorControl[Motor_NUM].Current.OFCnt4--;
        }
    }

}

void DisplayErrLed(void)
{
    u16 DispCounterTemp;
    DisplayCounter++;
    if(wGlobal_Flags==NO_ERROR)
    {
		wGlobal_Flags1 =0;
		FaultOccurred1 = 0;
        FaultOccurred = wGlobal_Flags;
        DispCounterTemp=LED_LOW_SPEED;
		RED_LED1_OFF;
	}
	else
	{
		 switch (FaultOccurred)
        {
		     case M1_OVER_CURRENT: //A电机过流快闪5次
            LEDSet(LED_ON,5,0,LED_HIGH_SPEED);
            break;
			 
            case M2_OVER_CURRENT: //B电机过流快闪6次
            LEDSet(LED_ON,6,0,LED_HIGH_SPEED);
			
			case M1_HALL_ERR:    //电机A霍尔错误
			LEDSet(LED_ON,2,1,LED_LOW_SPEED);		//慢闪2次
            break;
				
            case M2_HALL_ERR:		// 电机B霍尔错误 
		    LEDSet(LED_ON,3,1,LED_LOW_SPEED);     //慢闪3次		
            break;
			
			case OVER_VOLTAGE:  //过压
            RED_LED1_ON;
            DisplayCounter=0;
            break;
			
            case UNDER_VOLTAGE:   //欠压
            LEDSet(LED_ON,1,1,LED_LOW_SPEED);     //慢闪1次		
            break;
            
			case MOTORA_PHASE_ERROR:			//快闪7次		
			LEDSet(LED_ON,7,0,LED_HIGH_SPEED);
			break;
						
		    case MOTORB_PHASE_ERROR:			//快闪8次	
			LEDSet(LED_ON,8,0,LED_HIGH_SPEED);
		    break;
				
		    case CAN_COMMUNICATION_ERR:	//快闪1次
            LEDSet(LED_ON,1,0,LED_HIGH_SPEED);
            break;
			
			case MOTORA_MISSING_PHASE: //快速闪烁2次
            LEDSet(LED_ON,2,0,LED_HIGH_SPEED);
            break;

            case MOTORB_MISSING_PHASE: //快速闪烁3次
            LEDSet(LED_ON,3,0,LED_HIGH_SPEED);
            break;
			
			default:
            RED_LED1_OFF;
            break;
	    }
	}
}



void BLDCA_Phase_Check(void)
{
	static u8 motor_err_cnt5 = 0;
	if(MotorControl[0].PWM_Duty!=0)
	{
		if(MotorControl[0].Direction!=-1&&MotorControl[0].Direction!=1)
		{
			motor_err_cnt5++;
			if(motor_err_cnt5>10)		//累计10次报警
			{
				motor_err_cnt5=10;
				MotorControl[0].Motor_Start_Stop = 0;
				MotorControl[0].Fault_Flag =1;
				MC_SetFault(MOTORA_PHASE_ERROR);
			}
		}
		else if(ABS(MotorControl[0].Speed_Set-MotorControl[0].Speed_Real)<20)
		{
			if(motor_err_cnt5>0)
			motor_err_cnt5--;
		}
	}
}


void BLDCB_Phase_Check(void)
{
	static u8 motor_err_cnt6 = 0;
	if(MotorControl[1].PWM_Duty!=0)
	{
		if(MotorControl[1].Direction!=-1&&MotorControl[1].Direction!=1)
		{
			motor_err_cnt6++;
			if(motor_err_cnt6>10)//10次报警
			{
				motor_err_cnt6=10;
				MotorControl[1].Motor_Start_Stop = 0;
				MotorControl[1].Fault_Flag =1;
				MC_SetFault(MOTORB_PHASE_ERROR);
			}
		}
		else if(ABS(MotorControl[1].Speed_Set-MotorControl[1].Speed_Real)<20)
		{
			if(motor_err_cnt6>0)
			motor_err_cnt6--;
		}
	}
}


/*************************
*Function Name 		:BLDC_Stuck_Chk
*Description   		:BLDC堵转保护  500ms无动作报警
* Input           : None
* Output          : None
* Return          : None		2021.12.16	by diamond
*************************/
void BLDC_Stuck_Chk(void)			/* 1ms循环里 */
{
		static u16 M1phase_check_cnt=0,M2phase_check_cnt=0;
		if(GetMotorSpeed(M1)==0&&MotorControl[0].PWM_Duty!=0)	//BLDC1堵转检测
		{
			M1phase_check_cnt++;
			if(M1phase_check_cnt>PHASE_ERR_MAX)
			{
				M1phase_check_cnt=PHASE_ERR_MAX;
//				MotorControl[0].Fault_Flag =1;
//				MotorControl[0].Motor_Start_Stop = 0;
//				MC_SetFault(MOTORA_PHASE_ERROR);
			}
		}
		else 
		{
			M1phase_check_cnt=0;
		}
		if(GetMotorSpeed(M2)==0&&MotorControl[1].PWM_Duty!=0)	//BLDC2堵转检测
		{
			M2phase_check_cnt++;
			if(M2phase_check_cnt>PHASE_ERR_MAX)
			{
				M2phase_check_cnt=PHASE_ERR_MAX;
				MotorControl[1].Fault_Flag =1;
				MotorControl[1].Motor_Start_Stop = 0;
				MC_SetFault(MOTORB_PHASE_ERROR);
			}
		}
		else 
		{
			M2phase_check_cnt=0;
		}
}

/*------------------------------------------------
Function:电机5缺相检测
Input   :No
Output  :No
Explain :No
------------------------------------------------*/
int FilterValueRange  = 100; //该值可能不同的电机需要实际调整，如果采样点准确，缺相检测容易做，并且检测准确
int skrRange   =  1500;
#define Current_Range_Of_Missing_Phase 100
void MotorA_Default_Phase_Cheek(void)
{
    if(MotorControl[0].Current.FilterValue > FilterValueRange)
    {
        MotorA_Current_temp.skr++;
        if(MotorA_Current_temp.skr > 2000)
        {
            MotorA_Current_temp.skr = 2000;
        }
    }
    else
    {
        if(MotorA_Current_temp.skr>0)
        {
            MotorA_Current_temp.skr--;
        }
    }

    if(MotorControl[0].Speed_Real == 0 && MotorControl[0].Motor_Start_Stop == 1 && MotorControl[0].Speed_Set != 0 \
			&& MotorControl[0].PWM_Duty == 8400 && MotorControl[0].Current.FilterValue<10) //这种用于启动时已经缺相的检测
    {
        MC_SetFault(MOTORA_MISSING_PHASE);
        MotorControl[0].Fault_Flag = 1;
    }
    if(MotorControl[0].Speed_Real != 0 && MotorControl[0].Motor_Start_Stop == 1 && MotorControl[0].Speed_Set != 0  )
    {

        if(MotorControl[0].Hall.HallState == HALL_Study[0].HallTab[4])      //BC
        {
            MotorA_Current_temp.index_bc++;
            MotorA_Current_temp.bc += MotorControl[0].Current.FilterValue;
        }
        else if(MotorControl[0].Hall.HallState == HALL_Study[0].HallTab[5]) //BA
        {
            MotorA_Current_temp.index_ba++;
            MotorA_Current_temp.ba += MotorControl[0].Current.FilterValue;
        }
        else if(MotorControl[0].Hall.HallState == HALL_Study[0].HallTab[0]) //CA
        {
            MotorA_Current_temp.index_ca++;
            MotorA_Current_temp.ca += MotorControl[0].Current.FilterValue;
        }
        else if(MotorControl[0].Hall.HallState == HALL_Study[0].HallTab[1]) //CB
        {
            MotorA_Current_temp.index_cb++;
            MotorA_Current_temp.cb += MotorControl[0].Current.FilterValue;
        }
        else if(MotorControl[0].Hall.HallState == HALL_Study[0].HallTab[2]) //AB
        {
            MotorA_Current_temp.index_ab++;
            MotorA_Current_temp.ab += MotorControl[0].Current.FilterValue;
        }
        else if(MotorControl[0].Hall.HallState == HALL_Study[0].HallTab[3]) //AC
        {
            MotorA_Current_temp.index_ac++;
            MotorA_Current_temp.ac += MotorControl[0].Current.FilterValue;
        }
    }
}
/*------------------------------------------------
Function:电机5缺相检测for中断
Input   :No
Output  :No
Explain :No
------------------------------------------------*/
void MotorA_Default_Phase_Cheek_IT(void)
{
    MotorA_Current_temp.ab_t = MotorA_Current_temp.ab;//可以改为临时变量
    MotorA_Current_temp.ac_t = MotorA_Current_temp.ac;
    MotorA_Current_temp.bc_t = MotorA_Current_temp.bc;
    MotorA_Current_temp.ba_t = MotorA_Current_temp.ba;
    MotorA_Current_temp.ca_t = MotorA_Current_temp.ca;
    MotorA_Current_temp.cb_t = MotorA_Current_temp.cb;

//    MotorA_Current_temp.index_ab_t = MotorA_Current_temp.index_ab;//可以改为临时变量
//    MotorA_Current_temp.index_ac_t = MotorA_Current_temp.index_ac;
//    MotorA_Current_temp.index_bc_t = MotorA_Current_temp.index_bc;
//    MotorA_Current_temp.index_ba_t = MotorA_Current_temp.index_ba;
//    MotorA_Current_temp.index_ca_t = MotorA_Current_temp.index_ca;
//    MotorA_Current_temp.index_cb_t = MotorA_Current_temp.index_cb;

//    MotorA_Current_temp.abaver = MotorA_Current_temp.ab_t/MotorA_Current_temp.index_ab;//可以改为临时变量
//    MotorA_Current_temp.acaver = MotorA_Current_temp.ac_t/MotorA_Current_temp.index_ac;
//    MotorA_Current_temp.bcaver = MotorA_Current_temp.bc_t/MotorA_Current_temp.index_bc;
//    MotorA_Current_temp.baaver = MotorA_Current_temp.ba_t/MotorA_Current_temp.index_ba;
//    MotorA_Current_temp.caaver = MotorA_Current_temp.ca_t/MotorA_Current_temp.index_ca;
//    MotorA_Current_temp.cbaver = MotorA_Current_temp.cb_t/MotorA_Current_temp.index_cb;
    if(MotorA_Current_temp.skr > skrRange)
    {
        if(MotorA_Current_temp.ac_t < Current_Range_Of_Missing_Phase && MotorA_Current_temp.ca_t <Current_Range_Of_Missing_Phase)   //缺A
        {
            MotorControl[0].Fault_Flag = 1;
            MC_SetFault1(MOTORA_MISSING_PHASE_A);//具体哪一个电机错误查看 第二个32位全局错误 wGlobal_Flags1
            MC_SetFault(MOTORA_MISSING_PHASE); //单纯为了指示灯
        }
        if(MotorA_Current_temp.ab_t < Current_Range_Of_Missing_Phase && MotorA_Current_temp.ba_t < Current_Range_Of_Missing_Phase)  //缺B
        {
            MotorControl[0].Fault_Flag = 1;
            MC_SetFault1(MOTORA_MISSING_PHASE_B);
            MC_SetFault(MOTORA_MISSING_PHASE); //单纯为了指示灯
        }

        if(MotorA_Current_temp.bc_t < Current_Range_Of_Missing_Phase && MotorA_Current_temp.cb_t < Current_Range_Of_Missing_Phase)  //缺C
        {
            MotorControl[0].Fault_Flag = 1;
            MC_SetFault1(MOTORA_MISSING_PHASE_C);
            MC_SetFault(MOTORA_MISSING_PHASE); //单纯为了指示灯
        }
    }
    MotorA_Current_temp.ab = 0;
    MotorA_Current_temp.index_ab = 0;
    MotorA_Current_temp.ac = 0;
    MotorA_Current_temp.index_ac = 0;
    MotorA_Current_temp.bc = 0;
    MotorA_Current_temp.index_bc = 0;
    MotorA_Current_temp.ba = 0;
    MotorA_Current_temp.index_ba = 0;
    MotorA_Current_temp.ca = 0;
    MotorA_Current_temp.index_ca = 0;
    MotorA_Current_temp.cb = 0;
    MotorA_Current_temp.index_cb = 0;
}
/*------------------------------------------------
Function:电机6缺相检测
Input   :No
Output  :No
Explain :用于启动时已经缺相的检测
------------------------------------------------*/
void MotorB_Default_Phase_Cheek(void)
{
    if(MotorControl[1].Current.FilterValue > FilterValueRange)
    {
        MotorB_Current_temp.skr++;
        if(MotorB_Current_temp.skr > 2000)
        {
            MotorB_Current_temp.skr = 2000;
        }
    }
    else
    {
        if(MotorB_Current_temp.skr>0)
        {
            MotorB_Current_temp.skr--;
        }
    }
    if(MotorControl[1].Speed_Real == 0 && MotorControl[1].Motor_Start_Stop == 1 && MotorControl[1].Speed_Set != 0 && MotorControl[1].PWM_Duty == 8400 && MotorControl[1].Current.FilterValue<10) //这种用于启动时已经缺相的检测
    {
        MotorControl[1].Fault_Flag = 1;
        MC_SetFault(MOTORB_MISSING_PHASE);
    }
    if(MotorControl[1].Speed_Real != 0 && MotorControl[1].Motor_Start_Stop == 1 && MotorControl[1].Speed_Set != 0  )
    {

        if(MotorControl[1].Hall.HallState == HALL_Study[1].HallTab[4])      //BC
        {
            MotorB_Current_temp.index_bc++;
            MotorB_Current_temp.bc += MotorControl[1].Current.FilterValue;
        }
        else if(MotorControl[1].Hall.HallState == HALL_Study[1].HallTab[5]) //BA
        {
            MotorB_Current_temp.index_ba++;
            MotorB_Current_temp.ba += MotorControl[1].Current.FilterValue;
        }
        else if(MotorControl[1].Hall.HallState == HALL_Study[1].HallTab[0]) //CA
        {
            MotorB_Current_temp.index_ca++;
            MotorB_Current_temp.ca += MotorControl[1].Current.FilterValue;
        }
        else if(MotorControl[1].Hall.HallState == HALL_Study[1].HallTab[1]) //CB
        {
            MotorB_Current_temp.index_cb++;
            MotorB_Current_temp.cb += MotorControl[1].Current.FilterValue;
        }
        else if(MotorControl[1].Hall.HallState == HALL_Study[1].HallTab[2]) //AB
        {
            MotorB_Current_temp.index_ab++;
            MotorB_Current_temp.ab += MotorControl[1].Current.FilterValue;
        }
        else if(MotorControl[1].Hall.HallState == HALL_Study[1].HallTab[3]) //AC
        {
            MotorB_Current_temp.index_ac++;
            MotorB_Current_temp.ac += MotorControl[1].Current.FilterValue;
        }
    }
}
/*------------------------------------------------
Function:电机6缺相检测for中断
Input   :No
Output  :No
Explain :用于电机运动时的缺相检测
------------------------------------------------*/
void MotorB_Default_Phase_Cheek_IT(void)
{
    MotorB_Current_temp.ab_t = MotorB_Current_temp.ab; //可以改为临时变量
    MotorB_Current_temp.ac_t = MotorB_Current_temp.ac;
    MotorB_Current_temp.bc_t = MotorB_Current_temp.bc;
    MotorB_Current_temp.ba_t = MotorB_Current_temp.ba;
    MotorB_Current_temp.ca_t = MotorB_Current_temp.ca;
    MotorB_Current_temp.cb_t = MotorB_Current_temp.cb;

    MotorB_Current_temp.index_ab_t = MotorB_Current_temp.index_ab; //可以改为临时变量
    MotorB_Current_temp.index_ac_t = MotorB_Current_temp.index_ac;
    MotorB_Current_temp.index_bc_t = MotorB_Current_temp.index_bc;
    MotorB_Current_temp.index_ba_t = MotorB_Current_temp.index_ba;
    MotorB_Current_temp.index_ca_t = MotorB_Current_temp.index_ca;
    MotorB_Current_temp.index_cb_t = MotorB_Current_temp.index_cb;

    MotorB_Current_temp.abaver = MotorB_Current_temp.ab_t/MotorB_Current_temp.index_ab;//可以改为临时变量
    MotorB_Current_temp.acaver = MotorB_Current_temp.ac_t/MotorB_Current_temp.index_ac;
    MotorB_Current_temp.bcaver = MotorB_Current_temp.bc_t/MotorB_Current_temp.index_bc;
    MotorB_Current_temp.baaver = MotorB_Current_temp.ba_t/MotorB_Current_temp.index_ba;
    MotorB_Current_temp.caaver = MotorB_Current_temp.ca_t/MotorB_Current_temp.index_ca;
    MotorB_Current_temp.cbaver = MotorB_Current_temp.cb_t/MotorB_Current_temp.index_cb;
    if(MotorB_Current_temp.skr > skrRange)  //skr为保证电流值大于FilterValueRange的计数值，电流大于FilterValueRange一段时间才进行缺相检测判断，空载容易误判，所以此处过滤空载缺相检测
    {
        if(MotorB_Current_temp.ac_t < Current_Range_Of_Missing_Phase && MotorB_Current_temp.ca_t <Current_Range_Of_Missing_Phase)   //缺A  该值为一段时间内的电流值总和，100只涉及实际应用值
        {
            MotorControl[1].Fault_Flag = 1;
            MC_SetFault1(MOTORB_MISSING_PHASE_A);//具体哪一个电机错误查看 第二个32位全局错误 wGlobal_Flags1
            MC_SetFault(MOTORB_MISSING_PHASE); //单纯为了指示灯
        }
        if(MotorB_Current_temp.ab_t < Current_Range_Of_Missing_Phase && MotorB_Current_temp.ba_t < Current_Range_Of_Missing_Phase)  //缺B
        {
            MotorControl[1].Fault_Flag = 1;
            MC_SetFault1(MOTORB_MISSING_PHASE_B);
            MC_SetFault(MOTORB_MISSING_PHASE); //单纯为了指示灯
        }

        if(MotorB_Current_temp.bc_t < Current_Range_Of_Missing_Phase && MotorB_Current_temp.cb_t < Current_Range_Of_Missing_Phase)  //缺C
        {
            MotorControl[1].Fault_Flag = 1;
            MC_SetFault1(MOTORB_MISSING_PHASE_C);
            MC_SetFault(MOTORB_MISSING_PHASE); //单纯为了指示灯
        }
    }
    MotorB_Current_temp.ab = 0;
    MotorB_Current_temp.index_ab = 0;
    MotorB_Current_temp.ac = 0;
    MotorB_Current_temp.index_ac = 0;
    MotorB_Current_temp.bc = 0;
    MotorB_Current_temp.index_bc = 0;
    MotorB_Current_temp.ba = 0;
    MotorB_Current_temp.index_ba = 0;
    MotorB_Current_temp.ca = 0;
    MotorB_Current_temp.index_ca = 0;
    MotorB_Current_temp.cb = 0;
    MotorB_Current_temp.index_cb = 0;
}

void BLDC1_OverSpdChk(void)
{
	static u16 speed_err_cnt = 0;
	if((MotorControl[0].Speed_Ref>0&&MotorControl[0].Speed_Real<0)||(MotorControl[0].Speed_Ref<0&&MotorControl[0].Speed_Real>0))		/* 反向转 */
		speed_err_cnt++;
	else if(MotorControl[0].Speed_Ref==0&&(ABS(MotorControl[0].Speed_Ref)-ABS(MotorControl[0].Speed_Set))<500)
		speed_err_cnt=0;
	else if(ABS(MotorControl[0].Speed_Ref)-ABS(MotorControl[0].Speed_Set)>500)																								/* 实际>设定 */
		speed_err_cnt++;
	else if(speed_err_cnt>0)
		speed_err_cnt--;
	if(speed_err_cnt>1000)
	{
		speed_err_cnt = 0;
		MotorControl[0].Fault_Flag = 1;
		MotorControl[0].Motor_Start_Stop = DISABLE;
		MC_SetFault(MOTORA_OVER_SPEED);
	}
}
void BLDC2_OverSpdChk(void)
{
	static u16 speed_err_cnt = 0;
	if((MotorControl[1].Speed_Ref>0&&MotorControl[1].Speed_Real<0)||(MotorControl[1].Speed_Ref<0&&MotorControl[1].Speed_Real>0))		/* 反向转 */
		speed_err_cnt++;	
	else if(MotorControl[1].Speed_Ref==0&&(ABS(MotorControl[1].Speed_Ref)-ABS(MotorControl[1].Speed_Set))<500)
		speed_err_cnt=0;	
	else if(ABS(MotorControl[1].Speed_Ref)-ABS(MotorControl[1].Speed_Set)>500)																											/* 实际>设定 */
		speed_err_cnt++;
	else if(speed_err_cnt>0)
		speed_err_cnt--;
	if(speed_err_cnt>1000)
	{
		speed_err_cnt = 0;
		MotorControl[1].Fault_Flag = 1;
		MotorControl[1].Motor_Start_Stop = DISABLE;
		MC_SetFault(MOTORB_OVER_SPEED);
	}
}
void Init_Drive_Para(void)
{
//	 PID_Init (&PID_Speed_InitStruct[0],&PID_Speed_InitStruct[1]);
	
    VoltVar.BusHigh = VOLT_330V ;
    VoltVar.BusLow =  VOLT_160V ;
    VoltVar.BusHighCnt_T = 200;
    VoltVar.BusLowCnt_T = 200;
	VoltVar.HighGuardTime  = 40;
    VoltVar.LowGuardTime  = 200;
    MotorControl[M1].Speed_Set =0 ;
//	    MotorControl[M1].Current.MaxValue1 = M1_CurrentValue(3) ;
//    MotorControl[M1].Current.MaxValue2 = M1_CurrentValue(4) ;
//	  MotorControl[M1].Current.MaxValue3 = M1_CurrentValue(5) ;
//    MotorControl[M1].Current.MaxValue4 = M1_CurrentValue(5) ;
//	
//    MotorControl[M2].Current.MaxValue1 = M1_CurrentValue(2) ;
//    MotorControl[M2].Current.MaxValue2 = M1_CurrentValue(4) ;
//	MotorControl[M2].Current.MaxValue3 = M1_CurrentValue(5) ;
//    MotorControl[M2].Current.MaxValue4 = M1_CurrentValue(6) ;
	
    MotorControl[M1].Current.MaxValue1 = M1_CurrentValue(11) ;
    MotorControl[M1].Current.MaxValue2 = M1_CurrentValue(12) ;
	MotorControl[M1].Current.MaxValue3 = M1_CurrentValue(13) ;
    MotorControl[M1].Current.MaxValue4 = M1_CurrentValue(14) ;
//	

    MotorControl[M1].Pole_Paires = 4;
    MotorControl[M1].Acceleration = 5;
    MotorControl[M1].Deceleration = 5;
	Sensorless[M1].Acceleration = 5;
	Sensorless[M1].Deceleration = 5;
	  MotorControl[M1].Current.OFCnt1_T = 5000 ;
		MotorControl[M1].Current.OFCnt2_T = 500 ;
		MotorControl[M1].Current.OFCnt3_T = 50 ;
		MotorControl[M1].Current.OFCnt4_T = 5 ;
    MotorControl[M1].Motor_Start_Stop = 0; 
	

    MotorControl[M2].Speed_Set =0 ;
    MotorControl[M2].Current.MaxValue1 = M2_CurrentValue(11) ;
    MotorControl[M2].Current.MaxValue2 = M2_CurrentValue(12) ;
	MotorControl[M2].Current.MaxValue3 = M2_CurrentValue(13) ;
    MotorControl[M2].Current.MaxValue4 = M2_CurrentValue(14) ;
    MotorControl[M2].Pole_Paires = 4;
    MotorControl[M2].Acceleration = 5;
    MotorControl[M2].Deceleration = 5;
		MotorControl[M2].Current.OFCnt1_T = 5000 ;
		MotorControl[M2].Current.OFCnt2_T = 500 ;
		MotorControl[M2].Current.OFCnt3_T = 50 ;
		MotorControl[M2].Current.OFCnt4_T = 5 ;
    MotorControl[M2].Motor_Start_Stop = 0;
	
	PID_Init (&PID_Speed_InitStruct[0],&PID_Current_InitStructure[0]);		//电机5的PID
	PID_Init (&PID_Speed_InitStruct[1],&PID_Current_InitStructure[1]);		//电机6的PID

    MotorControl[M1].Hall.HallState = HALL_GetPhase1();
//	  MotorControl[M1].Hall.HallStateValue = MotorControl[M1].Hall.HallState ;
    MotorControl[M1].Hall.ChangeFlag = 1; //电机停止之后需要换向启动
    MotorControl[M2].Hall.HallState = HALL_GetPhase2();
    MotorControl[M2].Hall.ChangeFlag = 1;
//    MotorControl[M2].Hall.HallStateValue = MotorControl[M2].Hall.HallState ;

    HALL_Study[M2].StudySectorCnt3 = 150;/*换向一次时间1代表2ms*/
    HALL_Study[M2].HallSector = 1;
    HALL_Study[M2].StudySectorCnt = 0;
    HALL_Study[M2].HallCommPWM = 4500;/*霍尔换向PWM值*/

    HALL_Study[M1].StudySectorCnt3 = 100;/*换向一次时间1代表2ms*/
    HALL_Study[M1].HallSector = 1;
    HALL_Study[M1].StudySectorCnt = 0;
    HALL_Study[M1].HallCommPWM = 4000;/*霍尔换向PWM值*/
	
	Sensorless[M1].SenlessHallSector=1;
	Sensorless[M1].CountSectorCnt3= 400;//20;
	Sensorless[0].PWM_DutySet =4000;
	Sensorless[0].PWM_Duty =4000;
//	HALL_Study[M1].HallTab[0] = 5; //电机测试架上的电机
//    HALL_Study[M1].HallTab[1] = 1;
//    HALL_Study[M1].HallTab[2] = 3;
//    HALL_Study[M1].HallTab[3] = 2;
//    HALL_Study[M1].HallTab[4] = 6;
//    HALL_Study[M1].HallTab[5] = 4;
//	
	HALL_Study[M1].HallTab[0] = 3; //自己座位上的电机
    HALL_Study[M1].HallTab[1] = 2;
    HALL_Study[M1].HallTab[2] = 6;
    HALL_Study[M1].HallTab[3] = 4;
    HALL_Study[M1].HallTab[4] = 5;
    HALL_Study[M1].HallTab[5] = 1;
	
//	HALL_Study[M1].HallTab[0] = 6; //自己座位上的电机,反转
//    HALL_Study[M1].HallTab[1] = 2;
//    HALL_Study[M1].HallTab[2] = 3;
//    HALL_Study[M1].HallTab[3] = 1;
//    HALL_Study[M1].HallTab[4] = 5;
//    HALL_Study[M1].HallTab[5] = 4;
	
//	HALL_Study[M2].HallTab[0] = 3;
//    HALL_Study[M2].HallTab[1] = 2;
//    HALL_Study[M2].HallTab[2] = 6;
//    HALL_Study[M2].HallTab[3] = 4;
//    HALL_Study[M2].HallTab[4] = 5;
//    HALL_Study[M2].HallTab[5] = 1;

    HALL_Study[M2].HallTab[0] = 5; //测试工装上的电机
    HALL_Study[M2].HallTab[1] = 1;
    HALL_Study[M2].HallTab[2] = 3;
    HALL_Study[M2].HallTab[3] = 2;
    HALL_Study[M2].HallTab[4] = 6;
    HALL_Study[M2].HallTab[5] = 4;
}