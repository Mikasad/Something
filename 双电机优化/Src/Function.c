/************************************************************

************************************************************/

/*
***********************************************
* Include headfile
***********************************************
*/
#include "function.h"
#include "stm32f4xx_hal.h"
#include "bsp_BDCMotor.h"
#include "main.h"
#include "canopen_pdo.h"
#include "Agreement.h"
#include "flash.h"
/*
***********************************************
* Private Const Table
***********************************************
*/

VoltVar_TypeDef VoltVar;
u32 wGlobal_Flags;
u32 prewGlobal_Flags;
u32 wGlobal_Flags1;
u32 FaultOccurred = 0;
u32 FaultOccurred1 = 0;
uint32_t DisplayCounter = 0;
DefaultPhaseCheekCurTemp Motor5_Current_temp;
DefaultPhaseCheekCurTemp Motor6_Current_temp;


/*------------------------------------------------
Function:MCU版本号
Input   :&MCU_Version ,robotype,main_ver, fun_ver,small_ver
Output  :No
Explain :No
------------------------------------------------*/
void Program_Version_Init(MCU_Version *pVersion,u8 robotype,u8 main_ver,u8 fun_ver,u8 small_ver)
{
    pVersion->uVersionPartOfRobotType = robotype;				//项目类型	S线
    pVersion->uVersionPartOfMainVersion = main_ver;			//大版本号
    pVersion->uVersionFullVersion = fun_ver;						//功能版本号
    pVersion->uVersionPartOfSmallVersion = small_ver;		//小版本号，代码bug以及参数微调
    pVersion->uVersionFullVersion = (robotype<<24)+(main_ver<<16)+(fun_ver<<8)+small_ver;
}
/*------------------------------------------------
Function:硬件版本号
Input   :&MCU_Version ,funtype，vol，cur_max，update_ver
Output  :No
Explain :No
------------------------------------------------*/
void HardVersion_Init(MCU_Version *pVersion,u8 funtype,u8 vol,u8 cur_max,u8 update_ver)
{
  pVersion->HardwarePartOfMotorType = funtype;        //电机类型：混合H,伺服S,步进B
  pVersion->HardwarePartOfVotage = vol;                //工作电压  A：24V
  pVersion->HardwarePartOfCurrent = cur_max;          //最大电流
  pVersion->HardwarePartOfVersion = update_ver;        //更新的版本号  
  pVersion->HardwareFullVersion = (funtype<<24)+(vol<<16)+(cur_max<<8)+update_ver;
}
/*------------------------------------------------
Function:电流保护限制
Input   :No
Output  :No
Explain :No
------------------------------------------------*/
void CurrentLimit(u8 Motor_NUM)
{
    if(MotorControl[Motor_NUM].Current.FilterValue > MotorControl[Motor_NUM].Current.MaxValue1)		/* 1阶段对于无刷暂时没用，会被PID调节回来 */
    {
        MotorControl[Motor_NUM].Current.OFCnt1++;
        if(MotorControl[Motor_NUM].Current.OFCnt1 > MotorControl[Motor_NUM].Current.OFCnt1_T)
        {
//            if(MotorControl[Motor_NUM].PWM_Duty>0)
//            {
//                MotorControl[Motor_NUM].PWM_Duty -= 1;
//            }
//            else if(MotorControl[Motor_NUM].PWM_Duty<0)
//            {
//                MotorControl[Motor_NUM].PWM_Duty += 1;
//            }
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

/*------------------------------------------------
Function:设置错误标志
Input   :No
Output  :No
Explain :No
------------------------------------------------*/
void MC_SetFault(u32 hFault_type)
{
    u8 i;
    wGlobal_Flags |= hFault_type;
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
/*------------------------------------------------
Function:清除错误标志
Input   :No
Output  :No
Explain :No
------------------------------------------------*/
void MC_ClearFault(u32 hFault_type)
{
    wGlobal_Flags &= ~hFault_type;
    FaultOccurred &= ~hFault_type; //清除当前错误类型

}
/*------------------------------------------------
Function:错误清除
Input   :No
Output  :No
Explain :No
------------------------------------------------*/
void Motor_Fault_Clear(void)
{
    if(OverFlow_Cnt[3]<BREAK_CNT_MAX) //如果brake报警5次之后就不能清除了
    {
        if(((wGlobal_Flags&(PUSH_MOTOR1_OVER_CUR|PUSHMOTOR_INT_ERROR|PUSH_BRAKE_LINE|PUSH_LOST_HALL))==0)&&MotorControl[0].Fault_Flag == 1)//推杆0错误清除
        {
            MotorControl[0].Fault_Flag = 0;
        }
        if(((wGlobal_Flags&(PUSH_MOTOR2_OVER_CUR|PUSHMOTOR_INT_ERROR|PUSH_BRAKE_LINE|PUSH_LOST_HALL))==0)&&MotorControl[1].Fault_Flag == 1)//推杆1错误清除
        {
            MotorControl[1].Fault_Flag = 0;
        }
        if(((wGlobal_Flags&(PUSH_MOTOR3_OVER_CUR|PUSHMOTOR_INT_ERROR|PUSH_BRAKE_LINE|PUSH_LOST_HALL))==0)&&MotorControl[2].Fault_Flag == 1)//推杆2错误清除
        {
            MotorControl[2].Fault_Flag = 0;
        }
        if(((wGlobal_Flags&(PUSH_MOTOR4_OVER_CUR|PUSHMOTOR_INT_ERROR|PUSH_BRAKE_LINE|PUSH_LOST_HALL))==0)&&MotorControl[9].Fault_Flag == 1)//推杆9错误清除
        {
            MotorControl[9].Fault_Flag = 0;
        }
    }
    if(OverFlow_Cnt[2]<BREAK_CNT_MAX)
    {
        if(((wGlobal_Flags&(ONEWYA_MOTOR1_OVER_CUR|MOTOR3_BRAKE_LINE))==0)&&MotorControl[3].Fault_Flag == 1)//电机3错误清除
        {
            MotorControl[3].Fault_Flag = 0;
        }
        if(((wGlobal_Flags&(ONEWYA_MOTOR2_OVER_CUR|MOTOR4_BRAKE_LINE))==0)&&MotorControl[4].Fault_Flag == 1)//电机4错误清除
        {
            MotorControl[4].Fault_Flag = 0;
        }
    }
    if(OverFlow_Cnt[0]<BREAK_CNT_MAX)
    {
        if(((wGlobal_Flags&(BLDC1_OVER_CUR|HALL5_SENSOR_ERR|MOTOR5_PHASE_ERROR|MOTOR5_OVER_SPEED|MOTOR5_MISSING_PHASE))==0)&&MotorControl[5].Fault_Flag == 1)//电机5错误清除 无刷电机
        {
            MotorControl[5].Fault_Flag = 0;
        }
    }

    if(OverFlow_Cnt[1]<BREAK_CNT_MAX)
    {
        if(((wGlobal_Flags&(BLDC2_OVER_CUR|HALL6_SENSOR_ERR|MOTOR6_PHASE_ERROR|MOTOR6_OVER_SPEED|MOTOR6_MISSING_PHASE))==0)&&MotorControl[6].Fault_Flag == 1)//电机6错误清除 无刷电机
        {
            MotorControl[6].Fault_Flag = 0;
        }
    }
    if(((wGlobal_Flags&SIDE_BRUSH_ERROR)==0)&&MotorControl[7].Fault_Flag == 1)//电机7错误清除 边刷
    {
        MotorControl[7].Fault_Flag = 0;
    }
    if(((wGlobal_Flags&FAN_ERROR)==0)&&MotorControl[8].Fault_Flag == 1)//电机8错误清除 风机
    {
        MotorControl[8].Fault_Flag = 0;
    }
    if(OverFlow_Cnt[4]<BREAK_CNT_MAX)
    {
        if((wGlobal_Flags&BRAKE10_11_12_13)==0&&MotorControl[10].Fault_Flag == 1)
        {
            MotorControl[10].Fault_Flag = 0;
            MotorControl[11].Fault_Flag = 0;
            MotorControl[12].Fault_Flag = 0;
            MotorControl[13].Fault_Flag = 0;
        }
    }
}
/*----------------------------

*Fuction：绝对值函数
*Explain：类型：float
----------------------------*/
float f_abs(float a)
{
    float temp;
    if(a>0) temp = a;
    else if(a<0) temp = -a;

    return temp;
}
long l_abs(long a)
{
    long temp;
    if(a>0) temp = a;
    else if(a<0) temp = -a;

    return temp;
}

//堵转结构体
typedef struct OLCKED_ROTOR_Ttag
{
    long pStuckCurr;
    long pStuckTime;
    long pStuckVel;
} OLCKED_ROTOR_T;

u8 Need_To_Stuck_Function[MOTOR_NUM] = {0,0,0,0,0,1,1,0,0,0,0,0,0,0};  //哪个电机需要堵转保护就置1
//uint16_t Stuck_Current = 0;
int32_t  Stuck_Speed = 50;
uint32_t Stuck_Time = 100000;
uint16_t Stuck_TimeCnt[MOTOR_NUM] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0};
//uint16_t Unit_Of_Current[MOTOR_NUM] = {637,637,637,316,316,253,253,0,0,637,0,0,0,0};
float Percentage_Of_Current[MOTOR_NUM] = {0.5,0.5,0.5,0.5,0.5,0.8,0.8,0.5,0.5,0.5,0.5,0.5,0.5,0.5};
void MotorStuckCheck(void)
{
    for(u8 i=0; i<MOTOR_NUM; i++)
    {
        CurrentLimit(i);
//        if(Need_To_Stuck_Function[i]) //如果该电机需要堵转保护
//        {
//            if(MotorControl[i].Current.FilterValue > (float)(MotorControl[i].Current.MaxValue1*Percentage_Of_Current[i])  && l_abs(MotorControl[i].Speed_Real) < Stuck_Speed)
//            {
//                Stuck_TimeCnt[i]++;
//                if(Stuck_TimeCnt[i] > Stuck_Time)
//                {
//                    SetMotorStop(i);
////                    MC_SetFault(OVER_CURRENT(i));
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
//        }
    }

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
/*------------------------------------------------
Function:参数初始化
Input   :No
Output  :No
Explain :No
------------------------------------------------*/
void Init_Drive_Para(void)
{
    VoltVar.HighGuardTime  = 40;
    VoltVar.LowGuardTime  = 50;
    VoltVar.BusHigh = VOLT_360V ;
    VoltVar.BusLow =  VOLT_160V ;
//    TIM3->CCR1 = 0;
//    TIM8->CCR4 = 8400;
//	   MotorControl[10].PWM_DutySet = 8400;		/* S线单向有刷，供电24V */   50不需要
//    MotorControl[12].PWM_DutySet = 8400;		/* S线单向有刷，供电24V */
//    MotorControl[10].Motor_Start_Stop = 1;
//    MotorControl[12].Motor_Start_Stop = 1;
    /************************推杆初始化***************************/
    MotorControl[0].Current.MaxValue1 = MOTOR0_CurrentValue(2) ;
    MotorControl[0].Current.MaxValue2 = MOTOR0_CurrentValue(3) ;
    MotorControl[0].Current.MaxValue3 = MOTOR0_CurrentValue(4) ;
    MotorControl[0].Current.MaxValue4 = MOTOR0_CurrentValue(5) ;
    MotorControl[0].Current.OFCnt1_T = 200;
    MotorControl[0].Current.OFCnt2_T = 30000;
    MotorControl[0].Current.OFCnt3_T = 10000;
    MotorControl[0].Current.OFCnt4_T = 2000;

    MotorControl[1].Current.MaxValue1 = MOTOR1_CurrentValue(2) ;
    MotorControl[1].Current.MaxValue2 = MOTOR1_CurrentValue(3) ;
    MotorControl[1].Current.MaxValue3 = MOTOR1_CurrentValue(4) ;
    MotorControl[1].Current.MaxValue4 = MOTOR1_CurrentValue(5) ;
    MotorControl[1].Current.OFCnt1_T = 200;
    MotorControl[1].Current.OFCnt2_T = 30000;
    MotorControl[1].Current.OFCnt3_T = 10000;
    MotorControl[1].Current.OFCnt4_T = 2000;

    MotorControl[2].Current.MaxValue1 = MOTOR2_CurrentValue(2) ;
    MotorControl[2].Current.MaxValue2 = MOTOR2_CurrentValue(3) ;
    MotorControl[2].Current.MaxValue3 = MOTOR2_CurrentValue(4) ;
    MotorControl[2].Current.MaxValue4 = MOTOR2_CurrentValue(5) ;
    MotorControl[2].Current.OFCnt1_T = 200;
    MotorControl[2].Current.OFCnt2_T = 30000;
    MotorControl[2].Current.OFCnt3_T = 10000;
    MotorControl[2].Current.OFCnt4_T = 2000;

    MotorControl[9].Current.MaxValue1 = MOTOR9_CurrentValue(2) ;
    MotorControl[9].Current.MaxValue2 = MOTOR9_CurrentValue(3) ;
    MotorControl[9].Current.MaxValue3 = MOTOR9_CurrentValue(4) ;
    MotorControl[9].Current.MaxValue4 = MOTOR9_CurrentValue(5) ;
    MotorControl[9].Current.OFCnt1_T = 200;
    MotorControl[9].Current.OFCnt2_T = 30000;
    MotorControl[9].Current.OFCnt3_T = 10000;
    MotorControl[9].Current.OFCnt4_T = 2000;
    /*********************推杆初始化end***************************/

    /*************************单向有刷3_4***************************/
    MotorControl[3].Current.MaxValue1 = MOTOR3_CurrentValue(3) ;  //过滤电机
    MotorControl[3].Current.MaxValue2 = MOTOR3_CurrentValue(4) ;
    MotorControl[3].Current.MaxValue3 = MOTOR3_CurrentValue(5) ;  //过滤电机
    MotorControl[3].Current.MaxValue4 = MOTOR3_CurrentValue(6) ;
    MotorControl[3].Current.OFCnt1_T = 2000;
    MotorControl[3].Current.OFCnt2_T = 30000;
    MotorControl[3].Current.OFCnt3_T = 10000;
    MotorControl[3].Current.OFCnt4_T = 2000;

    MotorControl[4].Current.MaxValue1 = MOTOR4_CurrentValue(3) ;  //喷水电机
    MotorControl[4].Current.MaxValue2 = MOTOR4_CurrentValue(3) ;
    MotorControl[4].Current.MaxValue3 = MOTOR4_CurrentValue(4) ;  //喷水电机
    MotorControl[4].Current.MaxValue4 = MOTOR4_CurrentValue(5) ;
    MotorControl[4].Current.OFCnt1_T = 2000;
    MotorControl[4].Current.OFCnt2_T = 50000;
    MotorControl[4].Current.OFCnt3_T = 10000;
    MotorControl[4].Current.OFCnt4_T = 2000;

    /*************************单向有刷3_4 end***************************/

    /*************************无刷5，6***************************/
    MotorControl[5].Current.MaxValue1 = MOTOR5_CurrentValue(11) ;
    MotorControl[5].Current.MaxValue2 = MOTOR5_CurrentValue(12) ;
    MotorControl[5].Current.MaxValue3 = MOTOR5_CurrentValue(13) ;
    MotorControl[5].Current.MaxValue4 = MOTOR5_CurrentValue(14) ;
    MotorControl[5].Current.CurOffset = 0.3*MOTOR5CURCOEFFICIENT*100;		//默认0.3A
    MotorControl[5].Current.OFCnt1_T = 30000;
    MotorControl[5].Current.OFCnt2_T = 30000;
    MotorControl[5].Current.OFCnt3_T = 10000;
    MotorControl[5].Current.OFCnt4_T = 2000;
    MotorControl[5].Pole_Paires = 4;
    MotorControl[5].Acceleration = 5;
    MotorControl[5].Deceleration = 5;
	MotorControl[5].Hall.Advance_Angle = 50;	//默认不使用超前角

    MotorControl[6].Current.MaxValue1 = MOTOR6_CurrentValue(11) ;
    MotorControl[6].Current.MaxValue2 = MOTOR6_CurrentValue(12) ;
    MotorControl[6].Current.MaxValue3 = MOTOR6_CurrentValue(13) ;
    MotorControl[6].Current.MaxValue4 = MOTOR6_CurrentValue(14) ;
    MotorControl[6].Current.CurOffset = 0.3*MOTOR6CURCOEFFICIENT*100;		//默认0.3A
    MotorControl[6].Current.OFCnt1_T = 30000;
    MotorControl[6].Current.OFCnt2_T = 30000;
    MotorControl[6].Current.OFCnt3_T = 10000;
    MotorControl[6].Current.OFCnt4_T = 2000;
    MotorControl[6].Pole_Paires = 4;
    MotorControl[6].Acceleration = 5;
    MotorControl[6].Deceleration = 5;
	MotorControl[6].Hall.Advance_Angle = 50;	//默认不使用超前角
    /*************************无刷5，6end***************************/
    PID_Init (&PID_Speed_InitStruct[0],&PID_Current_InitStructure[0]);		//电机5的PID
    PID_Init (&PID_Speed_InitStruct[1],&PID_Current_InitStructure[1]);		//电机6的PID
    PID_PWM_Init(&PID_PWM);																								//推杆0的PID
    /*	加减速度初始化	*/
    MotorControl[0].Acceleration = 10;
    MotorControl[0].Deceleration = 10;
    MotorControl[0].LastMotorDirection = 2;
    MotorControl[0].Hall.HALL_CaptureValue = 0; //用于上电标定
    MotorControl[1].Acceleration = 100;
    MotorControl[1].Deceleration = 100;
    MotorControl[1].LastMotorDirection = 2;
    MotorControl[1].Hall.HALL_CaptureValue = 10000; //由于推杆1需要位置控制，不标定不会打开Push_Location_model开关，所以在直接给定标定命令的时候当前值为0
    MotorControl[2].Acceleration = 10;
    MotorControl[2].Deceleration = 10;
    MotorControl[2].LastMotorDirection = 2;
    MotorControl[2].Hall.HALL_CaptureValue = 0; //用于上电标定

    MotorControl[3].Acceleration = 10;
    MotorControl[3].Deceleration = 10;

    MotorControl[4].Acceleration = 10;
    MotorControl[4].Deceleration = 10;

    MotorControl[7].Acceleration = 10;
    MotorControl[7].Deceleration = 10;

    MotorControl[8].Acceleration = 10;
    MotorControl[8].Deceleration = 10;

    MotorControl[9].Acceleration = 10;
    MotorControl[9].Deceleration = 10;
    MotorControl[9].LastMotorDirection = 2;
    MotorControl[9].Hall.HALL_CaptureValue = 0; //用于上电标定

    MotorControl[10].Current.OFCnt1_T = 5000;
    MotorControl[10].Current.OFCnt2_T = 200;
    MotorControl[10].Current.OFCnt3_T = 20;
    MotorControl[10].Current.OFCnt4_T = 5;
    MotorControl[10].Acceleration = 10;
    MotorControl[10].Deceleration = 10;

    MotorControl[11].Current.OFCnt1_T = 5000;
    MotorControl[11].Current.OFCnt2_T = 200;
    MotorControl[11].Current.OFCnt3_T = 20;
    MotorControl[11].Current.OFCnt4_T = 5;
    MotorControl[11].Acceleration = 10;
    MotorControl[11].Deceleration = 10;

    MotorControl[12].Current.OFCnt1_T = 5000;
    MotorControl[12].Current.OFCnt2_T = 200;
    MotorControl[12].Current.OFCnt3_T = 20;
    MotorControl[12].Current.OFCnt4_T = 5;
    MotorControl[12].Acceleration = 10;
    MotorControl[12].Deceleration = 10;

    MotorControl[13].Current.OFCnt1_T = 5000;
    MotorControl[13].Current.OFCnt2_T = 200;
    MotorControl[13].Current.OFCnt3_T = 20;
    MotorControl[13].Current.OFCnt4_T = 5;
    MotorControl[13].Acceleration = 10;
    MotorControl[13].Deceleration = 10;

    MotorControl[5].Hall.HallState = (GPIOD->IDR>>5)&(0x7);
    MotorControl[6].Hall.HallState = HALL_GetPhase2();

    /*           霍尔学习参数                */
    HALL_Study[1].StudySectorCnt3 = 200;/*换向一次时间1代表2ms*/
    HALL_Study[1].HallSector = 1;
    HALL_Study[1].StudySectorCnt = 0;
    HALL_Study[1].HallCommPWM = 4500;/*霍尔换向PWM值*/

    HALL_Study[0].StudySectorCnt3 = 200;/*换向一次时间1代表2ms*/
    HALL_Study[0].HallSector = 1;
    HALL_Study[0].StudySectorCnt = 0;
    HALL_Study[0].HallCommPWM = 4500;/*霍尔换向PWM值*/

//    HALL_Study[0].HallTab[0] = 5;
//    HALL_Study[0].HallTab[1] = 1;
//    HALL_Study[0].HallTab[2] = 3;
//    HALL_Study[0].HallTab[3] = 2;
//    HALL_Study[0].HallTab[4] = 6;
//    HALL_Study[0].HallTab[5] = 4;
	
	HALL_Study[0].HallTab[0] = 6;    //测功机霍尔学习
    HALL_Study[0].HallTab[1] = 4;
    HALL_Study[0].HallTab[2] = 5;
    HALL_Study[0].HallTab[3] = 1;
    HALL_Study[0].HallTab[4] = 3;
    HALL_Study[0].HallTab[5] = 2;


    HALL_Study[1].HallTab[0] = 5;
    HALL_Study[1].HallTab[1] = 1;
    HALL_Study[1].HallTab[2] = 3;
    HALL_Study[1].HallTab[3] = 2;
    HALL_Study[1].HallTab[4] = 6;
    HALL_Study[1].HallTab[5] = 4;

//    HALL_Study[1].HallTab[0] = 6;// 546231
//    HALL_Study[1].HallTab[1] = 4;
//    HALL_Study[1].HallTab[2] = 5;
//    HALL_Study[1].HallTab[3] = 1;
//    HALL_Study[1].HallTab[4] = 3;
//    HALL_Study[1].HallTab[5] = 2;

    MotorControl[0].Push_Location_model=1;
    MotorControl[1].Push_Location_model=0; //该推杆电机需要标定，标定成功才可以位置控制，其余电机不需要标定默认开启位置控制，只是伸出和收缩的控制没有位置控制
    MotorControl[2].Push_Location_model=1;
    MotorControl[9].Push_Location_model=1;
}
/*------------------------------------------------
Function:led灯设置
Input   :led1: 1/下面红灯亮  0/下面红灯不亮
				 led2: 1/上面红灯亮  0/上面红灯不亮
				 led_time:闪烁的次数
				 led_mode:1/慢闪一次 0/快闪
				 led_speed:闪烁的速度
Output  :No
Explain :No
------------------------------------------------*/
void LEDSet(u8 led1,u8 led2,u8 led_time,u8 led_mode,u8 led_speed)
{
    //led1
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

    //led2
    if(led2==0)
        RED_LED2_OFF;
    else if(led_mode)
    {
        if(DisplayCounter<2*led_speed)
            RED_LED2_ON;
        else if(DisplayCounter%(2*led_speed)==0&&DisplayCounter<4*led_time*led_speed)
            RED_LED2_TOGGLE;
    }
    else if(DisplayCounter<=(2*led_time)*led_speed)
    {
        if((DisplayCounter-1)%led_speed==0)
        {
            if((DisplayCounter/led_speed)%2==0)
            {
                RED_LED2_ON;
            }
            else
                RED_LED2_OFF;
        }
    }
    else if(DisplayCounter<2*(led_time+1)*led_speed)
    {
        RED_LED2_OFF;
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
/*------------------------------------------------
Function:驱动器错误指示
Input   :No
Output  :No
Explain :10ms运行一次
------------------------------------------------*/

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
        if(DisplayCounter<=3*DispCounterTemp)
        {
            GREEN_LED_ON;
        }
        else if(DisplayCounter<=4*DispCounterTemp)
        {
            GREEN_LED_OFF;
        }
        else
        {
            DisplayCounter = 0;
        }
        RED_LED1_OFF;
        RED_LED2_OFF;
    }
    else
    {
        GREEN_LED_OFF;
        switch (FaultOccurred)
        {
        case PUSH_MOTOR1_OVER_CUR: //推杆1闪烁1次
            LEDSet(LED_OFF,LED_ON,1,0,LED_HIGH_SPEED);

            break;

        case PUSH_MOTOR2_OVER_CUR: //推杆2闪烁2次
            LEDSet(LED_OFF,LED_ON,2,0,LED_HIGH_SPEED);

            break;

        case PUSH_MOTOR3_OVER_CUR: //推杆3闪烁3次
            LEDSet(LED_OFF,LED_ON,3,0,LED_HIGH_SPEED);

            break;

        case PUSH_MOTOR4_OVER_CUR: //推杆4闪烁4次
            LEDSet(LED_OFF,LED_ON,4,0,LED_HIGH_SPEED);


            break;
        case BLDC1_OVER_CUR: //BLDC5次
            LEDSet(LED_OFF,LED_ON,5,0,LED_HIGH_SPEED);


            break;
        case BLDC2_OVER_CUR: //上面6次
            LEDSet(LED_OFF,LED_ON,6,0,LED_HIGH_SPEED);


            break;
        case ONEWYA_MOTOR1_OVER_CUR:	//上面7次
            LEDSet(LED_OFF,LED_ON,7,0,LED_HIGH_SPEED);


            break;
        case ONEWYA_MOTOR2_OVER_CUR:		//上面8次
            LEDSet(LED_OFF,LED_ON,8,0,LED_HIGH_SPEED);
            break;

        case HALL5_SENSOR_ERR:    //下面红灯
            LEDSet(LED_ON,LED_OFF,1,1,LED_LOW_SPEED);		//慢闪

            break;

        case HALL6_SENSOR_ERR:		//上面红灯 慢闪1次
            LEDSet(LED_OFF,LED_ON,1,1,LED_LOW_SPEED);

            break;

        case OVER_VOLTAGE:  //两个灯常量
            RED_LED1_ON;
            RED_LED2_ON;
            DisplayCounter=0;
            break;
        case UNDER_VOLTAGE:   //欠压下面红灯亮
            RED_LED2_OFF;
            RED_LED1_ON;
            DisplayCounter=0;
            break;
        case MOTOR5_BREAK:		//上面快闪一次
            LEDSet(LED_ON,LED_OFF,1,0,LED_HIGH_SPEED);
            break;

        case MOTOR6_BREAK:		//上面快闪2次
            LEDSet(LED_ON,LED_OFF,2,0,LED_HIGH_SPEED);
            break;

        case BRAKE_3_4:				//上面快闪3次
            LEDSet(LED_ON,LED_OFF,3,0,LED_HIGH_SPEED);
            break;

        case BRAKE_0_1_2_9:		//上面快闪4次
            LEDSet(LED_ON,LED_OFF,4,0,LED_HIGH_SPEED);
            break;
        case BRAKE10_11_12_13:	//上面快闪5次
            LEDSet(LED_ON,LED_OFF,5,0,LED_HIGH_SPEED);
            break;

            //6次

        case MOTOR5_PHASE_ERROR:			//下面7次
            LEDSet(LED_ON,LED_OFF,7,0,LED_HIGH_SPEED);
            break;

        case MOTOR6_PHASE_ERROR:					//下面8次
            LEDSet(LED_ON,LED_OFF,8,0,LED_HIGH_SPEED);
            break;

        case CAN_COMMUNICATION_ERR:	//两个灯快闪1次
            LEDSet(LED_ON,LED_ON,1,0,LED_HIGH_SPEED);
            break;

        case MOTOR5_MISSING_PHASE: //两个红灯快速闪烁2次
            LEDSet(LED_ON,LED_ON,2,0,LED_HIGH_SPEED);
            break;

        case MOTOR6_MISSING_PHASE: //两个红灯快速闪烁3次
            LEDSet(LED_ON,LED_ON,3,0,LED_HIGH_SPEED);

            break;
        case MOTOR3_BRAKE_LINE:  //两个红灯快速闪烁4次
            LEDSet(LED_ON,LED_ON,4,0,LED_HIGH_SPEED);

            break;
        case MOTOR4_BRAKE_LINE: //两个红灯快速闪烁5次
            LEDSet(LED_ON,LED_ON,5,0,LED_HIGH_SPEED);

            break;
        case SIDE_BRUSH_ERROR:  //两个红灯快速闪烁6次
            LEDSet(LED_ON,LED_ON,6,0,LED_HIGH_SPEED);

            break;
        case MOTOR5_OVER_SPEED:  //两个红灯快速闪烁7次
            LEDSet(LED_ON,LED_ON,7,0,LED_HIGH_SPEED);

            break;
        case MOTOR6_OVER_SPEED:  //两个红灯快速闪烁8次
            LEDSet(LED_ON,LED_ON,8,0,LED_HIGH_SPEED);

            break;
        case PUSH_BRAKE_LINE:  //推杆断线		下面常亮，上面快速闪烁1-4次
            switch (FaultOccurred1)
            {
            case PUSH0_BRAKE_LINE:
                LEDSet(LED_ON,LED_ON,1,0,LED_HIGH_SPEED);
                break;
            case PUSH1_BRAKE_LINE:
                LEDSet(LED_ON,LED_ON,2,0,LED_HIGH_SPEED);
                break;
            case PUSH2_BRAKE_LINE:
                LEDSet(LED_ON,LED_ON,3,0,LED_HIGH_SPEED);
                break;
            case PUSH9_BRAKE_LINE:
                LEDSet(LED_ON,LED_ON,4,0,LED_HIGH_SPEED);
                break;
            }
            RED_LED1_ON;

            break;
        case PUSH_LOST_HALL:  //霍尔断线  	下面常亮，上面快速闪烁5-8次
            switch (FaultOccurred1)
            {
            case PUSH0_LOST_HALL:
                LEDSet(LED_ON,LED_ON,5,0,LED_HIGH_SPEED);
                break;
            case PUSH1_LOST_HALL:
                LEDSet(LED_ON,LED_ON,6,0,LED_HIGH_SPEED);
                break;
            case PUSH2_LOST_HALL:
                LEDSet(LED_ON,LED_ON,7,0,LED_HIGH_SPEED);
                break;
            case PUSH9_LOST_HALL:
                LEDSet(LED_ON,LED_ON,8,0,LED_HIGH_SPEED);
                break;
            }
            RED_LED1_ON;

            break;
        case FAN_ERROR:			//风机错误上面常亮，下面1-5次
            switch(FaultOccurred1)
            {
            case MOTOR8_VOLTAGE_ERROR:
                LEDSet(LED_ON,LED_ON,1,0,LED_HIGH_SPEED);
                break;
            case MOTOR8_OVER_CURRENT:
                LEDSet(LED_ON,LED_ON,2,0,LED_HIGH_SPEED);
                break;
            case MOTOR8_OVER_SPEED:
                LEDSet(LED_ON,LED_ON,3,0,LED_HIGH_SPEED);
                break;
            case MOTOR8_OVER_TEMP:
                LEDSet(LED_ON,LED_ON,4,0,LED_HIGH_SPEED);
                break;
            case MOTOR8_STUCK:
                LEDSet(LED_ON,LED_ON,5,0,LED_HIGH_SPEED);
                break;
            }
            RED_LED2_ON;
            break;
        default:
            RED_LED1_OFF;
            RED_LED2_OFF;
            break;
        }
    }

}
/*------------------------------------------------
Function:推杆电机位置环   上电抬起工作对象
Input   :No
Output  :No
Explain :No
------------------------------------------------*/
void Push_Motor_Location_Control(u8 motornum)
{
    if(MotorControl[motornum].Push_Location_model==1)//标定正确，开始执行位置控制
    {
        if(motornum == 1) //50机器滚刷推杆，后期预留滚刷自适应
        {
            if(MotorControl[5].Current.SetValue ==0)	//没有做电流自适应的情况，进行位置控制
            {
                if(MotorControl[motornum].Hall.HALL_CaptureValue > MotorControl[motornum].Location_Set)// && fabs(MotorControl[1].Hall.HALL_CaptureValue- MotorControl[1].Location_Set) > 5)
                {
                    MotorControl[motornum].PWM_DutySet = -PERCENT_95_OF_PWM_PERIOD;
                }
                else if(MotorControl[motornum].Hall.HALL_CaptureValue < MotorControl[motornum].Location_Set)// && fabs(MotorControl[1].Hall.HALL_CaptureValue- MotorControl[1].Location_Set) > 5)
                {
                    MotorControl[motornum].PWM_DutySet = PERCENT_95_OF_PWM_PERIOD;
                }
                else if(MotorControl[motornum].Hall.HALL_CaptureValue == MotorControl[motornum].Location_Set)//||fabs(MotorControl[1].Hall.HALL_CaptureValue- MotorControl[1].Location_Set) < 5)
                {
                    MotorControl[motornum].PWM_DutySet = 0;
                }
            }
        }
        else
        {
            if(MotorControl[motornum].Hall.HALL_CaptureValue > MotorControl[motornum].Location_Set)// && fabs(MotorControl[1].Hall.HALL_CaptureValue- MotorControl[1].Location_Set) > 5)
            {
                MotorControl[motornum].PWM_DutySet = PERCENT_95_OF_PWM_PERIOD;
            }
            else if(MotorControl[motornum].Hall.HALL_CaptureValue < MotorControl[motornum].Location_Set)// && fabs(MotorControl[1].Hall.HALL_CaptureValue- MotorControl[1].Location_Set) > 5)
            {
                MotorControl[motornum].PWM_DutySet = -PERCENT_95_OF_PWM_PERIOD;
            }
            else if(MotorControl[motornum].Hall.HALL_CaptureValue == MotorControl[motornum].Location_Set)//||fabs(MotorControl[1].Hall.HALL_CaptureValue- MotorControl[1].Location_Set) < 5)
            {
                MotorControl[motornum].PWM_DutySet = 0;
            }
        }
    }
}
/*------------------------------------------------

Function:4路推杆电机霍尔检测和断联检测

Input   :No

Output  :No

Explain :No

------------------------------------------------*/
u8 NeedToCheekPush[MOTOR_NUM] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0}; //如果该推杆电机需要霍尔错误或者推杆断联检测 0 1 2 9号电机默认都打开检测
u8 NeedToMissingHallPush[MOTOR_NUM] = {1,0,0,0,0,0,0,0,0,0,0,0,0,0}; //该数组来确保推杆是否需要霍尔保护
uint16_t PushMaxPulse[MOTOR_NUM] = {780,4000,4000,0,0,0,0,0,0,4000,0,0,0,0}; //这个数组需要确定整机推杆的最大值
u32 Push_Error_Cheek_Cnt[MOTOR_NUM] = {0};
int Push_curr_temp[MOTOR_NUM] = {0};
#define PUSH_PROTECT_MIN_VAL 5999
#define PUSH_PROTECT_MIN_CUR  100
#define MOTOR34_PROTECT_MIN_CUR 50
#define MOTOR34_PROTECT_MIN_VAL 399
void Push_Motor_Cheek(void)
{
    for(u8 i=0; i<MOTOR_NUM; i++)
    {
        if(NeedToCheekPush[i]==1) //如果该推杆电机需要霍尔错误或者推杆断联检测 0 1 2 9号电机默
        {
            Push_Error_Cheek_Cnt[i]++;
            if(Push_Error_Cheek_Cnt[i]%2002000==0) //51和100的最小公倍数时，会同时执行Push_Error_Cheek_Cnt[i]%51 和 Push_Error_Cheek_Cnt[i]%100==0这里剔除这种情况
            {
                Push_Error_Cheek_Cnt[i]++;
            }
            Push_curr_temp[i] += MotorControl[i].Current.FilterValue;
            /*占空比大于5999才保护*/
            if(MotorControl[i].Motor_Start_Stop == 1 && (l_abs(MotorControl[i].Location_Set-MotorControl[i].Hall.HALL_CaptureValue))>20 && \
                    l_abs(MotorControl[i].PWM_Duty) > PUSH_PROTECT_MIN_VAL && MotorControl[i].Hall.HALL_CaptureValue < PushMaxPulse[i])
            {
                if(Push_Error_Cheek_Cnt[i]%1001==0) //100.1ms将捕获值更新一次，并进行一次断线检测
                {
                    MotorControl[i].Hall.HALL_CaptureValueDelta = MotorControl[i].Hall.HALL_CaptureValue;
                    if(Push_curr_temp[i] < PUSH_PROTECT_MIN_CUR)  //断线检测判断
                    {
                        MotorControl[i].Motor_Start_Stop = 0;   //断线错误
//											  MotorControl[i].Fault_Flag = 1;  //断线错误可以不置错误标志位，可以继续使能
                        if(i == 0)
                        {
                            MC_SetFault1(PUSH0_BRAKE_LINE);//具体哪一个电机错误查看 第二个32位全局错误 wGlobal_Flags1
                            MC_SetFault(PUSH_BRAKE_LINE);//单纯为了指示灯
                        }
                        if(i == 1)
                        {
                            MC_SetFault1(PUSH1_BRAKE_LINE);
                            MC_SetFault(PUSH_BRAKE_LINE);//单纯为了指示灯
                        }
                        if(i == 2)
                        {
                            MC_SetFault1(PUSH2_BRAKE_LINE);
                            MC_SetFault(PUSH_BRAKE_LINE);//单纯为了指示灯
                        }
                        if(i == 9)
                        {
                            MC_SetFault1(PUSH9_BRAKE_LINE);
                            MC_SetFault(PUSH_BRAKE_LINE);//单纯为了指示灯
                        }
                    }
                }
                if(NeedToMissingHallPush[i]) //需要霍尔保护的推杆，默认都需要
                {
                    if((Push_Error_Cheek_Cnt[i]%2000==0)&&(MotorControl[i].Hall.HALL_CaptureValueDelta == \
                                                           MotorControl[i].Hall.HALL_CaptureValue)&&Push_curr_temp[i] >= PUSH_PROTECT_MIN_CUR) //如果200ms内推杆的霍尔值没有改变，那就说明推杆霍尔异常
                    {
                        MotorControl[i].Motor_Start_Stop = 0;   //霍尔错误

                        if(i == 0)
                        {
                            MC_SetFault1(PUSH0_LOST_HALL);
                            MC_SetFault(PUSH_LOST_HALL);//单纯为了指示灯
                        }
                        if(i == 1)
                        {
                            MC_SetFault1(PUSH1_LOST_HALL);
                            MC_SetFault(PUSH_LOST_HALL);//单纯为了指示灯
                        }
                        if(i == 2)
                        {
                            MC_SetFault1(PUSH2_LOST_HALL);
                            MC_SetFault(PUSH_LOST_HALL);//单纯为了指示灯
                        }
                        if(i == 9)
                        {
                            MC_SetFault1(PUSH9_LOST_HALL);
                            MC_SetFault(PUSH_LOST_HALL);//单纯为了指示灯
                        }
                        MotorControl[i].Fault_Flag = 1; //之后需要重新进行推杆标定，所以需要手动清除
                    }
                }
            }
            else
            {
                Push_Error_Cheek_Cnt[i] = 0;
            }
            if(Push_Error_Cheek_Cnt[i]%1001==0)  //100.1ms清一次
            {
                Push_curr_temp[i] = 0;
            }
        }
        if(NeedToCheekPush[i]==2)  //电机3电机4断线检测
        {
            Push_Error_Cheek_Cnt[i]++;
            Push_curr_temp[i] += MotorControl[i].Current.FilterValue;
            if(MotorControl[i].Motor_Start_Stop == 1 && l_abs(MotorControl[i].PWM_Duty) > MOTOR34_PROTECT_MIN_VAL) //占空比大于2999才保护
            {
                if(Push_Error_Cheek_Cnt[i]%(100*(TIM10->PSC+1)/10)==0) //10个水泵周期进行一次断线检测
                {
                    if(Push_curr_temp[i] < MOTOR34_PROTECT_MIN_CUR)
                    {
                        MotorControl[i].Motor_Start_Stop = 0;
                        if(i == 3)
                        {
                            MC_SetFault(MOTOR3_BRAKE_LINE);
                        }
                        if(i == 4)
                        {
                            MC_SetFault(MOTOR4_BRAKE_LINE);
                        }
                    }
                    Push_curr_temp[i] = 0;
                }
            }
            else
            {
                Push_Error_Cheek_Cnt[i] = 0;
            }
            if(Push_Error_Cheek_Cnt[i]%(100*(TIM10->PSC+1)/10)==0)  //1000ms清一次
            {
                Push_curr_temp[i] = 0;
            }
        }
    }
}
/*------------------------------------------------
Function:推杆电机标定
Input   :No
Output  :No
Explain :No
------------------------------------------------*/
uint32_t Sys_Ticktemp[10] = {0};
extern u8 First_PowerOn;
void Push_Motor_Calibrate(u8 motornum)
{
    u8 caputernumsetfinsh = 0;

    if(MotorControl[motornum].Push_Location_model ==1)      //每次标定完毕会将该标志位置1
    {
        MotorControl[motornum].Fault_Flag =0;//标定前把错误清零在标定中堵转或者全部缩回都判断为标定成功
        MotorControl[motornum].PWM_Duty = 0;
        MotorControl[motornum].PWM_DutySet = 0;
        MotorControl[motornum].Push_Location_model = 0;
        MotorControl[motornum].Hall.HALL_CaptureValue = 10000;  //用于标定，在不同的位置标定开始之后都需要自减
    }
    if(MotorControl[motornum].Fault_Flag == 0)
    {
        MotorControl[motornum].PWM_DutySet=-PERCENT_95_OF_PWM_PERIOD;//不能删除否则不能进捕获中断
        MotorControl[motornum].PWM_Duty-=10;
    }
    else
    {
        MotorControl[motornum].PWM_Duty=0;
    }
    if(MotorControl[motornum].PWM_Duty > PERCENT_95_OF_PWM_PERIOD)
    {
        MotorControl[motornum].PWM_Duty = PERCENT_95_OF_PWM_PERIOD;
    }
    if(MotorControl[motornum].PWM_Duty < -PERCENT_95_OF_PWM_PERIOD)
    {
        MotorControl[motornum].PWM_Duty = -PERCENT_95_OF_PWM_PERIOD;
    }
    if(motornum == 0)
    {
        MOTOR0CW();
    }
    else if(motornum == 1)
    {
        MOTOR1CCW();
    }
    else if(motornum == 2)
    {
        MOTOR2CW();
    }
    else if(motornum == 9)
    {
        MOTOR9CW();
    }
    Sys_Ticktemp[motornum]++; //计时

    if(Sys_Ticktemp[motornum]%501 == 0) // 501赋值一次 1000检查一次  1002次赋值一次 1503赋值一次  2000检查一次 2004赋值一次 2505赋值一次 3000检查一次 3006赋值一次 3057赋值一次 4000检查一次
    {
        MotorControl[motornum].Hall.HALL_PreCaptureValue = MotorControl[motornum].Hall.HALL_CaptureValue ;
    }
    else if(Sys_Ticktemp[motornum]%1000 == 0) //1000个值检查一次
    {
        caputernumsetfinsh = 1;//局部变量来保证霍尔值更新之后进行到位判断
    }
    else
    {
        caputernumsetfinsh = 0;
    }

    if((caputernumsetfinsh == 1 )&&
            MotorControl[motornum].Hall.HALL_PreCaptureValue == MotorControl[motornum].Hall.HALL_CaptureValue
//            MotorControl[motornum].Fault_Flag ==0
      ) //如果时间到，计数值没有改变，电机没有过流，
    {
        MotorControl[motornum].Fault_Flag = 0; //标定中如果过流则清除错误
        MotorControl[motornum].Motor_Start_Stop = 0;
        MotorControl[motornum].Location_Set = 0;
        MotorControl[motornum].PWM_Duty = 0;
        MotorControl[motornum].PWM_DutySet = 0; //必须清除，因为没有位置环还可以按照PWM控制方式来控制，这种情况不允许
        MotorControl[motornum].Hall.HALL_PreCaptureValue = 0;
        MotorControl[motornum].Hall.HALL_CaptureValue = 0;
        MotorControl[motornum].Push_motor_calibrationFLAG = 2;//标定正确退出标定
        MotorControl[motornum].Push_Location_model=1; //开启推杆位置环，该参数用来启用位置控制，需要赋值默认值来启用那些不需要位置控制的推杆电机
        Sys_Ticktemp[motornum] = 0;//计数清零
        if(motornum == 0)
        {
            MOTOR0STOP();
            MC_ClearFault(PUSH_MOTOR1_OVER_CUR);
        }
        else if(motornum == 1)
        {
            MOTOR1STOP();
            MC_ClearFault(PUSH_MOTOR2_OVER_CUR);
        }
        else if(motornum == 2)
        {
            MOTOR2STOP();
            MC_ClearFault(PUSH_MOTOR3_OVER_CUR);
        }
        else if(motornum == 9)
        {
            MOTOR9STOP();
            MC_ClearFault(PUSH_MOTOR4_OVER_CUR);
        }

    }
    else if(Sys_Ticktemp[motornum] > 5000) //10秒 标定失败
    {
        if(motornum == 0)
        {
            MOTOR0STOP();
            MC_SetFault1(PUSH0_CALIBRATE);
        }
        else if(motornum == 1)
        {
            MOTOR1STOP();
            MC_SetFault1(PUSH1_CALIBRATE);
        }
        else if(motornum == 2)
        {
            MOTOR2STOP();
            MC_SetFault1(PUSH2_CALIBRATE);
        }
        else if(motornum == 9)
        {
            MOTOR9STOP();
            MC_SetFault1(PUSH9_CALIBRATE);
        }
        MotorControl[motornum].Motor_Start_Stop = 0;
        MotorControl[motornum].PWM_Duty = 0;
        MotorControl[motornum].PWM_DutySet = 0;
        MotorControl[motornum].Hall.HALL_PreCaptureValue = 0;
        MotorControl[motornum].Hall.HALL_CaptureValue = 0;
        MotorControl[motornum].Push_motor_calibrationFLAG = 2;   //标定错误退出标定
        MotorControl[motornum].Push_Location_model=0; //关闭推杆1位置环
        Sys_Ticktemp[motornum] = 0;
        MC_SetFault(PUSHMOTOR_INT_ERROR);

        MotorState = START;
    }

}

/*------------------------------------------------
Function:交换排序
Input   :No
Output  :No
Explain :No
------------------------------------------------*/
void swap(float *x,float *y)
{
    uint32_t temp= 0;
    temp=*x;
    *x=*y;
    *y=temp;
}
void BubbleSort(float a[], float n)        // 本算法将a[]中的元素从小到到大进行排序
{
    uint32_t j= 0;
    for(uint32_t i = 0; i < n - 1; i++)
    {
        for(j = n - 1; j > i; j--)
        {
            if(a[j - 1]>a[j])
            {
                swap(&a[j - 1],&a[j]);      // 为交换函数，将a[j] 与 a[j - 1] 进行交换
            }
            else
            {}
        }
    }
}
/*------------------------------------------------
Function:电机5缺相检测
Input   :No
Output  :No
Explain :No
------------------------------------------------*/
#define ac 0
#define ab 1
#define bc 2
#define ac_ab_bc_index 3
float M5_100us_ac_ab_bc_cur[ac_ab_bc_index] = {0};
float M6_100us_ac_ab_bc_cur[ac_ab_bc_index] = {0};
float testM5_100us_ac_ab_bc_cur = 0;
float testM6_100us_ac_ab_bc_cur = 0;
void Motor5_Default_Phase_Cheek(void)
{
    if(MotorControl[5].Speed_Real != 0 && MotorControl[5].Motor_Start_Stop == 1 && \
            MotorControl[5].Speed_Set != 0  )
    {
        Motor5_Current_temp.cnt++;
        if(Motor5_Current_temp.cnt>10000)
        {
            Motor5_Current_temp.cnt = 0;
            BubbleSort(M5_100us_ac_ab_bc_cur,ac_ab_bc_index);
            testM5_100us_ac_ab_bc_cur =(M5_100us_ac_ab_bc_cur[2]-M5_100us_ac_ab_bc_cur[0])/M5_100us_ac_ab_bc_cur[2];//测试观察值
            if(testM5_100us_ac_ab_bc_cur>0.9) //从小到大排序之后最大值-最小值，0.6为测试估计值
            {
//                if(MotorControl[5].Current.DeepFilterVAL>50)
                {
                    MC_SetFault(MOTOR5_MISSING_PHASE); //单纯为了指示灯
                    MotorControl[5].Fault_Flag =1;
                    MotorControl[5].Motor_Start_Stop = 0;

                }
            }
            M5_100us_ac_ab_bc_cur[ab] = 0;
            M5_100us_ac_ab_bc_cur[ac] = 0;
            M5_100us_ac_ab_bc_cur[bc] = 0;
        }
        if(MotorControl[5].Hall.HallState == HALL_Study[0].HallTab[4]||\
                MotorControl[5].Hall.HallState == HALL_Study[0].HallTab[1])      //BC  CB
        {
            M5_100us_ac_ab_bc_cur[bc] += MotorControl[5].Current.FilterValue;
        }
        else if(MotorControl[5].Hall.HallState == HALL_Study[0].HallTab[5]||\
                MotorControl[5].Hall.HallState == HALL_Study[0].HallTab[2]) //BA  AB
        {
            M5_100us_ac_ab_bc_cur[ab] += MotorControl[5].Current.FilterValue;
        }
        else if(MotorControl[5].Hall.HallState == HALL_Study[0].HallTab[0]||\
                MotorControl[5].Hall.HallState == HALL_Study[0].HallTab[3]) //CA  AC
        {
            M5_100us_ac_ab_bc_cur[ac] += MotorControl[5].Current.FilterValue;
        }
    }
    else
    {
        M5_100us_ac_ab_bc_cur[ab] = 0;
        M5_100us_ac_ab_bc_cur[ac] = 0;
        M5_100us_ac_ab_bc_cur[bc] = 0;
    }
}
/*------------------------------------------------
Function:电机6缺相检测
Input   :No
Output  :No
Explain :
------------------------------------------------*/
void Motor6_Default_Phase_Cheek(void)
{
    if(MotorControl[6].Speed_Real != 0 && MotorControl[6].Motor_Start_Stop == 1 && \
            MotorControl[6].Speed_Set != 0  )
    {
        Motor6_Current_temp.cnt++;
        if(Motor6_Current_temp.cnt>20000)
        {
            Motor6_Current_temp.cnt = 0;
            BubbleSort(M6_100us_ac_ab_bc_cur,ac_ab_bc_index);
            testM6_100us_ac_ab_bc_cur =	(M6_100us_ac_ab_bc_cur[2]-M6_100us_ac_ab_bc_cur[0])/M6_100us_ac_ab_bc_cur[2];//测试观察值
            if(testM6_100us_ac_ab_bc_cur>0.9) //从小到大排序之后最大值-最小值，0.5为测试估计值
            {
//                if(MotorControl[6].Current.DeepFilterVAL>50)
                {   
									  MC_SetFault(MOTOR6_MISSING_PHASE); //单纯为了指示灯
                    MotorControl[6].Fault_Flag =1;
                    MotorControl[6].Motor_Start_Stop = 0;

                }
            }
            M6_100us_ac_ab_bc_cur[ab] = 0;
            M6_100us_ac_ab_bc_cur[ac] = 0;
            M6_100us_ac_ab_bc_cur[bc] = 0;
        }
        if(MotorControl[6].Hall.HallState == HALL_Study[1].HallTab[4]||\
                MotorControl[6].Hall.HallState == HALL_Study[1].HallTab[1])      //BC  CB
        {
            M6_100us_ac_ab_bc_cur[bc] += MotorControl[6].Current.FilterValue;
        }
        else if(MotorControl[6].Hall.HallState == HALL_Study[1].HallTab[5]||\
                MotorControl[6].Hall.HallState == HALL_Study[1].HallTab[2]) //BA  AB
        {
            M6_100us_ac_ab_bc_cur[ab] += MotorControl[6].Current.FilterValue;
        }
        else if(MotorControl[6].Hall.HallState == HALL_Study[1].HallTab[0]||\
                MotorControl[6].Hall.HallState == HALL_Study[1].HallTab[3]) //CA  AC
        {
            M6_100us_ac_ab_bc_cur[ac] += MotorControl[6].Current.FilterValue;
        }
    }
    else
    {
        M6_100us_ac_ab_bc_cur[ab] = 0;
        M6_100us_ac_ab_bc_cur[ac] = 0;
        M6_100us_ac_ab_bc_cur[bc] = 0;
    }
}
/*************************
*Function Name 		:Hardware_flowCheck
*Description   		:Check Hardware flow cnt if >=BREAK_CNT_MAX setfault the motor
* Input           : None
* Output          : None
* Return          : None		2021.11.25	by diamond
*************************/
void Hardware_flowCheck(void)
{
    if(OverFlow_Cnt[0]>=BREAK_CNT_MAX)	/* 无刷5 */
    {
        MC_SetFault(MOTOR5_BREAK);
        MotorControl[5].Fault_Flag = 1;
        MotorControl[5].Motor_Start_Stop = DISABLE;
    }
    if(OverFlow_Cnt[1]>=BREAK_CNT_MAX)	/* 无刷6 */
    {
        MC_SetFault(MOTOR6_BREAK);
        MotorControl[6].Fault_Flag = 1;
        MotorControl[6].Motor_Start_Stop = DISABLE;
    }
    if(OverFlow_Cnt[2]>=BREAK_CNT_MAX)	/* 单向有刷3——4 */
    {
        MC_SetFault(BRAKE_3_4);
        MotorControl[3].Motor_Start_Stop = DISABLE;
        MotorControl[3].Fault_Flag = 1;
        MotorControl[4].Motor_Start_Stop = DISABLE;
        MotorControl[4].Fault_Flag = 1;
    }
    if(OverFlow_Cnt[3]>=BREAK_CNT_MAX)	/* 推杆0_1_2_9 */
    {
        MC_SetFault(BRAKE_0_1_2_9);
        MotorControl[0].Motor_Start_Stop = DISABLE;
        MotorControl[1].Motor_Start_Stop = DISABLE;
        MotorControl[2].Motor_Start_Stop = DISABLE;
        MotorControl[9].Motor_Start_Stop = DISABLE;
        MotorControl[0].Fault_Flag = 1;
        MotorControl[1].Fault_Flag = 1;
        MotorControl[2].Fault_Flag = 1;
        MotorControl[9].Fault_Flag = 1;
    }
    if(OverFlow_Cnt[4]>=BREAK_CNT_MAX)	/* 有刷10_11_12_13 */
    {
        MC_SetFault(BRAKE10_11_12_13);
        MotorControl[10].Motor_Start_Stop = DISABLE;
        MotorControl[11].Motor_Start_Stop = DISABLE;
        MotorControl[12].Motor_Start_Stop = DISABLE;
        MotorControl[13].Motor_Start_Stop = DISABLE;
        MotorControl[10].Fault_Flag = 1;
        MotorControl[11].Fault_Flag = 1;
        MotorControl[12].Fault_Flag = 1;
        MotorControl[13].Fault_Flag = 1;
    }
}
/*------------------------------------------------
Function:偏置电压校准
Input   :No
Output  :No
Explain :No
------------------------------------------------*/
u8 Voltage_offset_calibration[8] = {0,1,2,3,4,5,6,9};//上下对起来看
u8 ADC_ConvertedValueindex[8] =    {6,7,8,0,1,2,4,9};//上下对起来看
#define OFFSET_NUM 100
u8 offset_cnt = 0;
void Voltage_offset_cali(void)
{
    offset_cnt++;
    if(offset_cnt<=OFFSET_NUM)
    {
        for(int i = 0; i < 8; i++)
        {
            MotorControl[Voltage_offset_calibration[i]].Current.offset += ADC_ConvertedValue[ADC_ConvertedValueindex[i]];
        }
        if(offset_cnt == OFFSET_NUM)		/* 取100次的均值 */
        {
            for(int k = 0; k < 8; k++)
            {
                MotorControl[Voltage_offset_calibration[k]].Current.offset = (MotorControl[Voltage_offset_calibration[k]].Current.offset/OFFSET_NUM);
            }
            MotorState = INIT;
        }
    }
    else if(offset_cnt>OFFSET_NUM)
    {
        offset_cnt = OFFSET_NUM+1;
        MotorState = INIT;
    }
}
/*************************
*Function Name 		:BLDC5_Phase_Check &&BLDC6_Phase_Check
*Description   		:Check PAHSE
* Input           : None
* Output          : None
* Return          : None		2021.11.30	by diamond
*************************/
void BLDC5_Phase_Check(void)
{
    static u8 motor_err_cnt5 = 0;
    if(MotorControl[5].PWM_Duty!=0)
    {
        if(MotorControl[5].Direction!=-1&&MotorControl[5].Direction!=1)
        {
            motor_err_cnt5++;
            if(motor_err_cnt5>10)		//累计10次报警
            {
                motor_err_cnt5=10;
                MotorControl[5].Motor_Start_Stop = 0;
                MC_SetFault(MOTOR5_PHASE_ERROR);
                MotorControl[5].Fault_Flag =1;

            }
        }
        else if(ABS(MotorControl[5].Speed_Set-MotorControl[5].Speed_Real)<20)
        {
            if(motor_err_cnt5>0)
                motor_err_cnt5--;
        }
    }
}
void BLDC6_Phase_Check(void)
{
    static u8 motor_err_cnt6 = 0;
    if(MotorControl[6].PWM_Duty!=0)
    {
        if(MotorControl[6].Direction!=-1&&MotorControl[6].Direction!=1)
        {
            motor_err_cnt6++;
            if(motor_err_cnt6>10)//10次报警
            {
                MC_SetFault(MOTOR6_PHASE_ERROR);
                motor_err_cnt6=10;
                MotorControl[6].Motor_Start_Stop = 0;
                MotorControl[6].Fault_Flag =1;

            }
        }
        else if(ABS(MotorControl[6].Speed_Set-MotorControl[6].Speed_Real)<20)
        {
            if(motor_err_cnt6>0)
                motor_err_cnt6--;
        }
    }
}
/*************************
*Function Name 		:Push_Motor_CurSelfAdapt
*Description   		:推杆电流根据无刷电机实现高度自适应
* Input           : None
* Output          : None
* Return          : None		2021.12.16	by diamond
*************************/
float cur_ratio[7] = {0,0,0,0,0,MOTOR5CURCOEFFICIENT,MOTOR6CURCOEFFICIENT};
int16_t limitpwm = 1000;
void Push_Motor_CurSelfAdapt(u8 bldc_num,u8 push_num,u16 hall_max,u16 hall_min)
{
    if(MotorControl[bldc_num].Current.SetValue!=0)
    {
        if(MotorControl[push_num].Motor_Start_Stop ==DISABLE)
        {
            MotorControl[push_num].Motor_Start_Stop = ENABLE;
        }
        if(f_abs(MotorControl[bldc_num].Current.DeepFilterVAL-MotorControl[bldc_num].Current.SetValue*cur_ratio[bldc_num])<MotorControl[bldc_num].Current.CurOffset*cur_ratio[bldc_num])
        {
            MotorControl[push_num].PWM_DutySet = 0;
        }
        else
        {
            MotorControl[push_num].PWM_DutySet = PID_Regulator(MotorControl[bldc_num].Current.SetValue,MotorControl[bldc_num].Current.DeepFilterVAL,&PID_PWM);
            if(MotorControl[push_num].PWM_DutySet<limitpwm&&MotorControl[push_num].PWM_DutySet>-limitpwm)
            {
                MotorControl[push_num].PWM_DutySet = 0;
            }
        }
        if(MotorControl[push_num].Hall.HALL_CaptureValue>hall_max&&MotorControl[push_num].PWM_DutySet>0)
        {
            MotorControl[push_num].PWM_DutySet = 0;
        }
        if(MotorControl[push_num].Hall.HALL_CaptureValue<hall_min&&MotorControl[push_num].PWM_DutySet<0)
        {
            MotorControl[push_num].PWM_DutySet = 0;
        }
    }
}
/*************************
*Function Name 		:Motor7_Err_Chk
*Description   		:边刷电机错误检测  2S没速度报警
* Input           : None
* Output          : None
* Return          : None		2021.12.16	by diamond
*************************/
void Motor7_Err_Chk(void)		/* 200ms循环里 */
{
    static u8 err_cur_cnt = 0;
    if(MotorControl[7].PWM_Duty>500&&MotorControl[7].Speed_Real==0)
    {
        err_cur_cnt++;
        if(err_cur_cnt==20)
        {
            MotorControl[7].Fault_Flag =1;
            MotorControl[7].Motor_Start_Stop = DISABLE;
            MC_SetFault(SIDE_BRUSH_ERROR);
        }
    }
    else
        err_cur_cnt=0;
}
/*************************
*Function Name 		:BLDC_Stuck_Chk
*Description   		:BLDC堵转保护  500ms无动作报警
* Input           : None
* Output          : None
* Return          : None		2021.12.16	by diamond
*************************/
u16 M5phase_check_cnt=0,M6phase_check_cnt=0,M5stuck_check_cnt=0,M6stuck_check_cnt=0;
void BLDC_Stuck_Chk(void)			/* 1ms循环里 */
{
//    static u16 M5phase_check_cnt=0,M6phase_check_cnt=0,M5stuck_check_cnt=0,M6stuck_check_cnt=0;
    if(GetMotorSpeed(Motor5)==0&&MotorControl[5].PWM_Duty!=0)	//BLDC1带载时缺相，电流为0
    {
        if(MotorControl[5].Current.DeepFilterVAL < 20) //基本可以判断为带载时缺相，因为那一相是断开的没有电流
        {
            M5phase_check_cnt++;//带载缺相计数
        }
        else
        {
            M5stuck_check_cnt++;//堵转计数
        }

        if(M5phase_check_cnt>PHASE_ERR_MAX)
        {
            MC_SetFault(MOTOR5_MISSING_PHASE);//M5缺相
            M5phase_check_cnt=PHASE_ERR_MAX;
            MotorControl[5].Fault_Flag =1;
            MotorControl[5].Motor_Start_Stop = 0;

        }
        if(M5stuck_check_cnt>SUTCK_ERR_MAX)
        {
            M5stuck_check_cnt = SUTCK_ERR_MAX;
            MC_SetFault(MOTOR5_PHASE_ERROR); //M5相序错误
            MotorControl[5].Fault_Flag =1;
            MotorControl[5].Motor_Start_Stop = 0;

        }
    }
    else
    {
        if(M5phase_check_cnt>0)
        {
            M5phase_check_cnt--;
        }
        if(M5stuck_check_cnt>0)
        {
            M5stuck_check_cnt--;
        }
    }

    if(GetMotorSpeed(Motor6)==0&&MotorControl[6].PWM_Duty!=0)	//BLDC1带载时缺相，电流为0
    {
        if(MotorControl[6].Current.DeepFilterVAL < 20) //基本可以判断为带载时缺相，因为那一相是断开的没有电流
        {
            M6phase_check_cnt++;//带载缺相计数
        }
        else
        {
            M6stuck_check_cnt++;//堵转计数
        }

        if(M6phase_check_cnt>PHASE_ERR_MAX)
        {
            MC_SetFault(MOTOR6_MISSING_PHASE);//M6缺相
            M6phase_check_cnt=PHASE_ERR_MAX;
            MotorControl[6].Fault_Flag =1;
            MotorControl[6].Motor_Start_Stop = 0;

        }
        if(M6stuck_check_cnt>SUTCK_ERR_MAX)
        {
            MC_SetFault(MOTOR6_PHASE_ERROR); //M6相序错误
            M6stuck_check_cnt = SUTCK_ERR_MAX;
            MotorControl[6].Fault_Flag =1;
            MotorControl[6].Motor_Start_Stop = 0;

        }
    }
    else
    {
        if(M6phase_check_cnt>0)
        {
            M6phase_check_cnt--;
        }
        if(M6stuck_check_cnt>0)
        {
            M6stuck_check_cnt--;
        }
    }

}
/*************************
*Function Name 		:BLDC1_OverSpdChk
*Description   		:BLDC1失速报警  4s连续速度异常报警
* Input           : None
* Output          : None
* Return          : None		2021.12.17	by diamond
*************************/
u16 speed_err_cnt1 = 0;
void BLDC1_OverSpdChk(void)
{
    if((MotorControl[5].Speed_Ref>0&&MotorControl[5].Speed_Real<0)||(MotorControl[5].Speed_Ref<0&&MotorControl[5].Speed_Real>0))		/* 反向转 */
        speed_err_cnt1++;
    else if(MotorControl[5].Speed_Ref==0&&(ABS(MotorControl[5].Speed_Ref-MotorControl[5].Speed_Real))<500)
        speed_err_cnt1=0;
    else if((ABS(MotorControl[5].Speed_Ref-MotorControl[5].Speed_Set))>500)																								/* 实际>设定 */
        speed_err_cnt1++;
//	else if(ABS(MotorControl[5].Speed_Ref)-ABS(MotorControl[5].Speed_Set)<-500&&MotorControl[5].PWM_Duty<NOMINAL_SPEED)		/* 实际<设定且PWM没满 */
//		speed_err_cnt1++;
    else if(speed_err_cnt1>0)
        speed_err_cnt1--;
    if(speed_err_cnt1>2000)
    {
        MC_SetFault(MOTOR5_OVER_SPEED);
        speed_err_cnt1 = 0;
        MotorControl[5].Fault_Flag = 1;
        MotorControl[5].Motor_Start_Stop = DISABLE;

    }
}
u16 speed_err_cnt2 = 0;
void BLDC2_OverSpdChk(void)
{

    if((MotorControl[6].Speed_Ref>0&&MotorControl[6].Speed_Real<0)||(MotorControl[6].Speed_Ref<0&&MotorControl[6].Speed_Real>0))		/* 反向转 */
        speed_err_cnt2++;
    else if(MotorControl[6].Speed_Ref==0&&(ABS(MotorControl[6].Speed_Ref)-ABS(MotorControl[6].Speed_Real))<500)
        speed_err_cnt2=0;
    else if(ABS(MotorControl[6].Speed_Real-MotorControl[6].Speed_Ref)>500)																											/* 实际>设定 */
        speed_err_cnt2++;
//	else if(ABS(MotorControl[6].Speed_Ref)-ABS(MotorControl[6].Speed_Set)<-500&&MotorControl[6].PWM_Duty<NOMINAL_SPEED)						/* 实际<设定且PWM没满 */
//		speed_err_cnt2++;
    else if(speed_err_cnt2>0)
        speed_err_cnt2--;
    if(speed_err_cnt2>2000)
    {
        MC_SetFault(MOTOR6_OVER_SPEED);
        speed_err_cnt2 = 0;
        MotorControl[6].Fault_Flag = 1;
        MotorControl[6].Motor_Start_Stop = DISABLE;

    }
}
/*************************
*Function Name 		:Push_OverRunChk
*Description   		:推杆动作时间限制
* Input           : u8 push1,u8 timer0,u8 push2,u8 timer1,u8 push3,u8 timer2,u8 push4,u8 timer3
										push 1-4:	0关闭 1开启	timer:0-256 多少秒后停止
* Output          : None
* Return          : None		2022.1.27	by diamond
*************************/
void Push_OverRunChk(u8 push1,u8 timer0,u8 push2,u8 timer1,u8 push3,u8 timer2,u8 push4,u8 timer3)
{
	static u8 timer_cnt[4];
	if(push1)
	{
		if(MotorControl[0].Motor_Start_Stop==ENABLE&&ABS(MotorControl[0].PWM_Duty)>500)
		{
			timer_cnt[0]++;
			if(timer_cnt[0]>timer0*2)
			{
				MotorControl[0].Motor_Start_Stop=DISABLE;
				MC_SetFault(PUSH_OVER_TIME);
				MC_SetFault1(PUSH0_OVERTIME);
				timer_cnt[0]=0;
			}
		}
		else timer_cnt[0]=0;
	}
	if(push2)
	{
		if(MotorControl[1].Motor_Start_Stop==ENABLE&&ABS(MotorControl[1].PWM_Duty)>500)
		{
			timer_cnt[1]++;
			if(timer_cnt[1]>timer1*2)
			{
				MotorControl[1].Motor_Start_Stop=DISABLE;
				MC_SetFault(PUSH_OVER_TIME);
				MC_SetFault1(PUSH1_OVERTIME);
				timer_cnt[1]=0;
			}
		}
		else timer_cnt[1]=0;
	}
	if(push3)
	{
		if(MotorControl[2].Motor_Start_Stop==ENABLE&&ABS(MotorControl[2].PWM_Duty)>500)
		{
			timer_cnt[2]++;
			if(timer_cnt[2]>timer2*2)
			{
				MotorControl[2].Motor_Start_Stop=DISABLE;
				MC_SetFault(PUSH_OVER_TIME);
				MC_SetFault1(PUSH2_OVERTIME);
				timer_cnt[2]=0;
			}
		}
		else timer_cnt[2]=0;
	}
	if(push4)
	{
		if(MotorControl[9].Motor_Start_Stop==ENABLE&&ABS(MotorControl[9].PWM_Duty)>500)
		{
			timer_cnt[3]++;
			if(timer_cnt[3]>timer3*2)
			{
				MotorControl[9].Motor_Start_Stop=DISABLE;
				MC_SetFault(PUSH_OVER_TIME);
				MC_SetFault1(PUSH9_OVERTIME);
				timer_cnt[3]=0;
			}
		}
		else timer_cnt[3]=0;
	}
	
}
