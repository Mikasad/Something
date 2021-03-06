#include "ds402.h"
#include "mc_config.h"
/* VAR DECLAR BEGIN */
Drive_Data Driver_Data[MAX_AXES];

Drive_Data Driver_Data_M1=
{
    .device_control_data =
    {
        . bAxisIsActive = 2,      //!0为使能M1电机的402状态机
        . Modes_of_operation = 3, //默认为操作模式为：速度模式
    },
};

Drive_Data Driver_Data_M2=
{
    .device_control_data =
    {
        . bAxisIsActive = 2,      //!0为使能M2电机的402状态机
        . Modes_of_operation = 3, //默认为操作模式为：速度模式
    },
};

/* VAR DECLAR END */

/* ---SEGMENT--- */

/* FUNCTION DECLAR BEGIN */
/*------------------------------------------------
Function:402协议参数初始化
Input   :No
Output  :No
Explain :定义bAxisIsActive该值不为0
------------------------------------------------*/
void Par_402_Init(void)
{
    Driver_Data[M1] = *(&Driver_Data_M1);
    Driver_Data[M2] = *(&Driver_Data_M2);
    pCtrlPar[M1]  = *(&pCtrlPar_M1);
    pCtrlPar[M2]  = *(&pCtrlPar_M2);
}
/*------------------------------------------------
Function:Set StateWord
Input   :NewState
Output  :No
Explain :No
------------------------------------------------*/
u8 SetStateWord(Drive_Data* dd,u16 setState)
{
    dd->drivesytemState |= setState;
    return SUCCESS;
}
/*------------------------------------------------
Function:Clear StateWord
Input   :ClearState
Output  :No
Explain :No
------------------------------------------------*/
u8 ClrStateWord(Drive_Data* dd,u16 clrState)
{
    dd->drivesytemState &= ~clrState;
    return SUCCESS;
}
/*------------------------------------------------
Function:Procced Driver State Change
Input   :d dd
Output  :No
Explain :No
------------------------------------------------*/
#define MOTOR_ON 1
#define MOTOR_OFF 0
#define A_AXIS 0
struct  State402test
{
    int motorEn[1];
    int Node_ID;
    int gsEncWaitRedyCnt;
    int statRegs[1];
    int glVBus;
    int DI_SERVO_ON;//如果xx DinPort配置DI SERVO ON功能\r\n该IO引脚为非通用输入输出口，mode功能被分配了，检测到该引脚信号无效
    int mergStopRequest[1];//如果Emcy Stop REQ是En Internal(DI CMD或Communication CMD)
    int disableStopOp[1]; //使能停止方式选择 0:free stop   1:immediatate stop
    int motionStat[1];
    int faultStopOp[1]; //故障停机方式
    int Sci_ErrClr_En;
    int conFltPre[1];
    int conFlt[1];
    int sAlarmClcRequest[1];
    int motionReason[1];
    int controlMode[1];
    int vel[1];
    int sMakeServoOnFlag[1];
} pAxisPar;
extern u8 HallStudyFlag1,HallStudyFlag2;
/*------------------------------------------------
Function:402状态机
Input   :No
Output  :No
Explain :No
------------------------------------------------*/
extern u16 Sync_Async_Control;
extern u8 relative_location_flag[MAX_AXES];
void ProceedDriveStateChange(void)
{
    Drive_Data *pCiA402Axis;
    u16 StatusWord = 0;
    u16 ControlWord6040 = 0;
    u8 status = 0;
    u16 counter = 0;
    static short cntTemp = 0;
    if(Sync_Async_Control == 1)//0：M1和M2异步控制  1：M1和M2同步控制
    {
        Driver_Data[M2].device_control_data.Conrtolword = Driver_Data[M1].device_control_data.Conrtolword;
    }
    for(counter = 0; counter < MAX_AXES; counter++)
    {
        if(!Driver_Data[counter].device_control_data.bAxisIsActive) //注意赋初值1
        {
            continue;
        }
			
        pCiA402Axis = &Driver_Data[counter];
        StatusWord = pCiA402Axis->device_control_data.Statusword;
        ControlWord6040 = pCiA402Axis->device_control_data.Conrtolword; 
				
				/* Status Bit13  EN/DISEN */
        if(STM[counter].bState == RUN)   //判断电机是否使能
        {
            StatusWord |= Bit_Motor_En; //清除电机使能状态字
        }
        else StatusWord &= ~Bit_Motor_En;
				
        switch(pCiA402Axis->drivesytemState)
        {
        case State_Start: //开始
        {
            cntTemp++;
            if(!pAxisPar.Node_ID)
            {
                pCiA402Axis->drivesytemState = State_Start;
            }
            if(cntTemp <= 10)
                break;
            cntTemp = 0;
            if(cntTemp == 0)
            {
                pCiA402Axis->drivesytemState = State_Not_ready_to_switch_on;
            }
        }
        break;
        case State_Not_ready_to_switch_on: //初始化
        {
            StatusWord |= (STATUSWORD_STATE_NOTREADYTOSWITCHON);/* Update StatuWord */
            if(Angle_Switch == 3) //换向完成
            {
                pCiA402Axis->drivesytemState = State_Switch_on_disabled;/* Update Driver State */
            }
            else
            {
                pCiA402Axis->drivesytemState = State_Not_ready_to_switch_on; //继续在当前状态
            }
        }
        break;
        case State_Switch_on_disabled://伺服无故障
        {
            StatusWord |= (STATUSWORD_STATE_SWITCHEDONDISABLED);
//      pCiA402Axis->device_control_data.bLowLevelPowerApplied = TRUE;  //这里有BUG在错误状态下发送控制字0x0080会出现状态改变，但是不会进入这里
            if((ControlWord6040 & CONTROLWORD_COMMAND_SHUTDOWN_MASK ) == CONTROLWORD_COMMAND_SHUTDOWN)
            {
//      pAxisPar.statRegs[A_AXIS] |= STAT_REG_AMPLIFIER_MAIN_RELAY_SET; // 输入6进入准备好
                pCiA402Axis->drivesytemState = State_Ready_to_switch_on; // Transition 2
            }
        }
        break;
        case State_Ready_to_switch_on://伺服准备好
        {
            StatusWord |= (STATUSWORD_STATE_READYTOSWITCHON);
            /* When Control Word Command is set QUICKSTOP||DISABLEVOLTAGE */
            if (((ControlWord6040 & CONTROLWORD_COMMAND_QUICKSTOP_MASK) == CONTROLWORD_COMMAND_QUICKSTOP)\
                    || ((ControlWord6040 & CONTROLWORD_COMMAND_DISABLEVOLTAGE_MASK) == CONTROLWORD_COMMAND_DISABLEVOLTAGE))
            {
                pCiA402Axis->drivesytemState = State_Switch_on_disabled; // Transition 7
            }
            /* When Control Word Command is set SWITCHON||SWITCHON_ENABLEOPERATION */
            else if(((ControlWord6040 & CONTROLWORD_COMMAND_SWITCHON_MASK) == CONTROLWORD_COMMAND_SWITCHON)\
                    || ((ControlWord6040 & CONTROLWORD_COMMAND_SWITCHON_ENABLEOPERATION_MASK) == CONTROLWORD_COMMAND_SWITCHON_ENABLEOPERATION))
            {
                /* High level Voltage check */
                if ( RealBusVoltageSensorParamsM1._Super.FaultState != MC_NO_ERROR ) //电压检测异常
                {
                    break;
                }
                else
                {
                    pCiA402Axis->drivesytemState = State_Switched_on;      // Transition 3
                }
            }
        }
        break;
        case State_Switched_on://等待伺服打开使能
        {
            StatusWord |= (STATUSWORD_STATE_SWITCHEDON);
            /* When Control Word Command is set CONTROLWORD_COMMAND_SHUTDOWN */
            if ((ControlWord6040& CONTROLWORD_COMMAND_SHUTDOWN_MASK) == CONTROLWORD_COMMAND_SHUTDOWN)
            {
                pCiA402Axis->drivesytemState = State_Ready_to_switch_on;// Transition 6
            }
            /* When Control Word Command is set QUICKSTOP||DISABLEVOLTAGE */
            else if(((ControlWord6040& CONTROLWORD_COMMAND_QUICKSTOP_MASK) == CONTROLWORD_COMMAND_QUICKSTOP
                     || (ControlWord6040& CONTROLWORD_COMMAND_DISABLEVOLTAGE_MASK) == CONTROLWORD_COMMAND_DISABLEVOLTAGE))
            {
                pCiA402Axis->drivesytemState = State_Switch_on_disabled;// Transition 10
            }
            /* When Control Word Command is set SWITCHON||ENABLEOPERATION */
            else if((ControlWord6040& CONTROLWORD_COMMAND_ENABLEOPERATION_MASK) == CONTROLWORD_COMMAND_ENABLEOPERATION)
            {
                if(counter == M1&&STM[counter].bState == IDLE)
//                if(counter == M1&&STM[counter].bState != ANY_STOP&&STM[counter].bState != STOP&&STM[counter].bState != STOP_IDLE)
                {
                    MC_StartMotor1();//使能电机M1
                }
                else if(counter == M2&&STM[counter].bState == IDLE)
                {
                    MC_StartMotor2();//使能电机M2
                }
                if(STM[counter].bState == RUN)
                {
                    pCiA402Axis->drivesytemState = State_Operation_enable;
                }
            }
        }
        break;
        case State_Operation_enable: //伺服运行
        {
            StatusWord |= (STATUSWORD_STATE_OPERATIONENABLED);
            if(STM[counter].bState == FAULT_NOW||STM[counter].bState ==FAULT_OVER) //如果出错自动进入State_Fault_reaction_active
            {
                pCiA402Axis->drivesytemState = State_Fault_reaction_active;
            }
            /* If xx DinPort config DI_SERVO_ON Function */
//      if (pAxisPar.DI_SERVO_ON== 3)
//			{
//				printf("如果xx DinPort配置DI SERVO ON功能\r\n该IO引脚为非通用输入输出口，mode功能被分配了，检测到该引脚信号无效\r\n");
//				if (pAxisPar.motorEn[A_AXIS] == MOTOR_ON)
//				{
//					ControlWord6040 &= ~Bit_Enable_Operation;		//伺服运行位  0无效 1有效
//				}
//			}
            /* If Emcy Stop REQ is En Internal(DI CMD or Communication CMD) */
//      if (pAxisPar.mergStopRequest[A_AXIS])
//			{
//				printf("如果Emcy Stop REQ是En Internal(DI CMD或Communication CMD)");
//				ControlWord6040 &= Bit_Enable_Voltage;  //接通主回路电位  0无效 1有效
//			}
            /* DI CMD or Communication CMD */
//      if(STM[counter].bState != RUN) //这里如果电机未使能会强制跳转到
//      {
//        ControlWord6040 &= CONTROLWORD_COMMAND_DISABLEVOLTAGE;
//      }

            /* When Control Word Command is set DISABLEOPERATION */
            if ((ControlWord6040 & CONTROLWORD_COMMAND_DISABLEOPERATION_MASK) == CONTROLWORD_COMMAND_DISABLEOPERATION)
            {
                if(pCiA402Axis->device_control_data.Shutdown_option_code != DISABLE_DRIVE) //使能停止方式选择 0:free stop   1:immediatate stop  注意这个值应该写到605B对象字典当中,在控制当中修改这个值
                {
                    /*disable operation pending*/
                    pCiA402Axis->device_control_data.Pending_Option_code= DISABLE_OPERATION_CODE;
                    return;
                }
                pCiA402Axis->drivesytemState = State_Switched_on;	// Transition   5
            }
            /* When Control Word Command is set QUICKSTOP */
            else if((ControlWord6040 & CONTROLWORD_COMMAND_QUICKSTOP_MASK) == CONTROLWORD_COMMAND_QUICKSTOP)  //快速停机
            {
                StatusWord &= ~Bit_Quick_stop;
                pCiA402Axis->drivesytemState = State_Quick_stop_active;	// Transition 11
                return;
            }
            /* When Control Word Command is set SHUTDOWN */
            else if((ControlWord6040 & CONTROLWORD_COMMAND_SHUTDOWN_MASK) == CONTROLWORD_COMMAND_SHUTDOWN)
            {
                if(pCiA402Axis->device_control_data.Shutdown_option_code != DISABLE_DRIVE)  //关机选项代码  参数shutdown选项代码决定在发生转换时应该采取什么操作 注意这个值应该写到605B对象字典当中,在控制当中修改这个值
                {
                    /*shutdown operation required*/
                    pCiA402Axis->device_control_data.Pending_Option_code= SHUTDOWN_CODE;
                    return;
                }
                else
                {
                    pCiA402Axis->drivesytemState = State_Ready_to_switch_on;	// Transition 8
                }
            }
            /* When Control Word Command is set DISABLEVOLTAGE */
            else if ((ControlWord6040 & CONTROLWORD_COMMAND_DISABLEVOLTAGE_MASK) == CONTROLWORD_COMMAND_DISABLEVOLTAGE)
            {
                pCiA402Axis->drivesytemState = State_Switch_on_disabled;	// Transition 9
            }

            else if((ControlWord6040 & CONTROLWORD_COMMAND_RELATIVE_MASK ) == CONTROLWORD_COMMAND_RELATIVE)//绝对位置控制模式
            {
                relative_location_flag[counter] = 1;
            }
            /* Halt */
        }
        break;
        case State_Quick_stop_active: //快速停机状态
        {
            StatusWord |= STATUSWORD_STATE_QUICKSTOPACTIVE;
            /* DI CMD or Communication CMD */
            if (STM[counter].bState != RUN)
            {
                ControlWord6040 &= CONTROLWORD_COMMAND_DISABLEVOLTAGE;
            }
            /* When Control Word Command is set DISABLEVOLTAGE */
            if ((ControlWord6040 & CONTROLWORD_COMMAND_DISABLEVOLTAGE_MASK) == CONTROLWORD_COMMAND_DISABLEVOLTAGE)
            {
                pCiA402Axis->drivesytemState = State_Switch_on_disabled;// Transition 12
                pCiA402Axis->device_control_data.Pending_Option_code= 0;
//            pAxisPar.mergStopRequest[A_AXIS] = 0;
                return;
            }
//        /* Halt */
//        if ((ControlWord6040& CONTROLWORD_COMMAND_HALT_MASK) == CONTROLWORD_COMMAND_HALT) //厂商自定义错误代码
//        {;}
            /* When QuickStop Code is Exist */
            if((pCiA402Axis->device_control_data.Quick_stop_option_code!= DISABLE_DRIVE)   //obj605A_QuickStop_optcode  注意这个值应该写到605B对象字典当中,在控制当中修改这个值
                    &&((StatusWord & STATUSWORD_STATE_MASK)!= STATUSWORD_STATE_QUICKSTOPACTIVE))
            {
                /*Only execute quick stop action in state transition 11*/
//                if(pCiA402Axis->device_control_data.Pending_Option_code == 0)
                pCiA402Axis->device_control_data.Pending_Option_code= QUICK_STOP_CODE;
                return;
            }
            /* OpCode-0,Direct Switch State */
            else //未使能停机方式直接切换状态机
            {
                pCiA402Axis->drivesytemState = State_Switch_on_disabled;    //continue state transition 12 快速停机方式605A选择为1-8，停机完成后，自然过度，无需指令控制
            }
            /*NOTE: it is not recommend to support transition 16 */
        }
        break;
        case State_Fault_reaction_active: //
        {
            StatusWord |= (STATUSWORD_STATE_FAULTREACTIONACTIVE);
            /* Halt */
//        if ((ControlWord6040& CONTROLWORD_COMMAND_HALT_MASK) == CONTROLWORD_COMMAND_HALT)
//        {;}
            /* This Place need Wait Drive Enabled */
//      if(pAxisPar.motorEn[A_AXIS] != MOTOR_ON)
//        return;
            if(pCiA402Axis->device_control_data.Fault_reaction_option_code != DISABLE_DRIVE) //参数故障反应选项代码决定如果驱动器中发生故障，应该采取什么行动。
            {
                /*fault reaction pending*/
                pCiA402Axis->device_control_data.Pending_Option_code = FAULT_REACT_CODE;//即将执行的选项代码
                return;
            }
            pCiA402Axis->drivesytemState = State_Fault;// Transition 14 故障停机完成后，自然过渡，无需控制指令
        }
            break;  /*NOTE: it is not recommend to support transition 16 */
        case State_Fault:
        {
            StatusWord |= (STATUSWORD_STATE_FAULT);

            if (STM[counter].hFaultOccurred == 0 && STM[counter].hFaultNow == 0) //该故障需要手动清除，如果当前没有故障：自动过度故障→伺服无故障
            {
                if(counter==M1)
                {
									STM[counter].bState = STOP_IDLE; //失能电机
									ControlWord6040 = CONTROLWORD_COMMAND_FAULTRESET;
                }
                else  if(counter==M2)
                {
									STM[counter].bState = STOP_IDLE; //失能电机
									ControlWord6040 = CONTROLWORD_COMMAND_FAULTRESET;
                }
            }
            /* When Control Word Command is set FAULTRESET ，考虑这里是0x80指令切换还是清楚错误之后自动切换*/
            if ((ControlWord6040 & CONTROLWORD_COMMAND_FAULTRESET_MASK) == CONTROLWORD_COMMAND_FAULTRESET)
            {
                pAxisPar.sAlarmClcRequest[A_AXIS] = TRUE;
                pCiA402Axis->drivesytemState  = State_Switch_on_disabled; // Transition 15 故障→伺服无故障
            }
        }
        break;
        default:
        {
            StatusWord = STATUSWORD_STATE_NOTREADYTOSWITCHON;
            pCiA402Axis->drivesytemState  = State_Not_ready_to_switch_on;
        }
        break;
        }
        /* Update Operational Functions */
        switch(pCiA402Axis->drivesytemState)
        {
        case State_Not_ready_to_switch_on:
        case State_Switch_on_disabled:
        case State_Ready_to_switch_on:
            pCiA402Axis->device_control_data.bBrakeApplied = TRUE;
            pCiA402Axis->device_control_data.bHighLevelPowerApplied =  FALSE;
            pCiA402Axis->device_control_data.bAxisFunctionEnabled = FALSE;
            pCiA402Axis->device_control_data.bConfigurationAllowed = TRUE;
            break;

        case State_Switched_on:
            pCiA402Axis->device_control_data.bBrakeApplied = TRUE;
            pCiA402Axis->device_control_data.bHighLevelPowerApplied =  TRUE;
            pCiA402Axis->device_control_data.bAxisFunctionEnabled = FALSE;
            pCiA402Axis->device_control_data.bConfigurationAllowed = TRUE;
            break;

        case State_Operation_enable:
        case State_Quick_stop_active:
        case State_Fault_reaction_active:
            pCiA402Axis->device_control_data.bBrakeApplied = FALSE;
            pCiA402Axis->device_control_data.bHighLevelPowerApplied =  TRUE;
            pCiA402Axis->device_control_data.bAxisFunctionEnabled = TRUE;
            pCiA402Axis->device_control_data.bConfigurationAllowed = FALSE;
            break;

        case State_Fault:
            pCiA402Axis->device_control_data.bBrakeApplied = TRUE;
            pCiA402Axis->device_control_data.bHighLevelPowerApplied =  FALSE;
            pCiA402Axis->device_control_data.bAxisFunctionEnabled = FALSE;
            pCiA402Axis->device_control_data.bConfigurationAllowed = TRUE;
            break;

        default:
            pCiA402Axis->device_control_data.bBrakeApplied = TRUE;
            pCiA402Axis->device_control_data.bHighLevelPowerApplied =  FALSE;
            pCiA402Axis->device_control_data.bAxisFunctionEnabled = FALSE;
            pCiA402Axis->device_control_data.bConfigurationAllowed = TRUE;
            break;
        }

        if( pCiA402Axis->device_control_data.bHighLevelPowerApplied == TRUE)
        {
            StatusWord |= STATUSWORD_VOLTAGE_ENABLED;
        }
        else
        {
            StatusWord &= ~STATUSWORD_VOLTAGE_ENABLED;
        }
        /*state transition finished set controlword complete bit and update status object 0x6041*/
        pCiA402Axis->device_control_data.Statusword =(StatusWord | STATUSWORD_REMOTE);
    }
}

/*------------------------------------------------
Function:Procced Driver State Change
Input   :dd
Output  :No
Explain :No
------------------------------------------------*/
void ProceedDriverHandler(void)
{
    Drive_Data *pCiA402Axis;
    u16 counter = 0;
    u16 StatusWord = 0;
    for(counter = 0; counter < MAX_AXES; counter++)
    {
        if(!Driver_Data[counter].device_control_data.bAxisIsActive) //注意赋初值1
        {
            continue;
        }
        pCiA402Axis = &Driver_Data[counter];
        StatusWord = pCiA402Axis->device_control_data.Statusword;
        switch(pCiA402Axis->drivesytemState)
        {
        case State_Start :
        case State_Not_ready_to_switch_on:
        case State_Switch_on_disabled:
        case State_Ready_to_switch_on:
        case State_Switched_on:
        case State_Fault:
        {
            if(Angle_Switch == 3) //换向完成，再将电机关掉,否在在第一次上电的时候正在换向，这里进行使能电机，会导致换向失败，以上状态都不要使能电机，这里需要判断哪个电机换向完成
            {
                if(counter==M1&&STM[counter].bState == RUN)
                {
                    MC_StopMotor1();
                }
                else  if(counter==M2&&STM[counter].bState == RUN)
                {
                    MC_StopMotor2();
                }
            }
        }
        break;
        case State_Operation_enable:
        case State_Quick_stop_active:
        {
            if(pCiA402Axis->device_control_data.bAxisFunctionEnabled &&\
                    pCiA402Axis->device_control_data.bLowLevelPowerApplied &&\
                    pCiA402Axis->device_control_data.bHighLevelPowerApplied &&\
                    !pCiA402Axis->device_control_data.bBrakeApplied)
            {
                if(pAxisPar.motionReason[A_AXIS] == 7 ||\
                        pAxisPar.motionReason[A_AXIS] == 6\
                  )
                    StatusWord |= STATUSWORD_INTERNAL_LIMIT;
                else
                    StatusWord &= ~STATUSWORD_INTERNAL_LIMIT;
            }
        }
        break;
        case State_Fault_reaction_active:
        {
        } break;

        pCiA402Axis->device_control_data.Statusword = StatusWord;
        }
    }
}
/*------------------------------------------------
Function:CiA402 Trans Action
Input   :Characteristic
Output  :No
Explain :现在该函数在停机时不做任何处理
------------------------------------------------*/
u8 CiA402_TransitionAction(u16 Characteristic)
{
    switch(Characteristic)
    {
    case SLOW_DOWN_RAMP:
        return TRUE;
    case QUICKSTOP_RAMP:
        return TRUE;
    case STOP_ON_CURRENT_LIMIT:
        return TRUE;
    case STOP_ON_VOLTAGE_LIMIT:
        return TRUE;
    case SLOW_DOWN_RAMP_STAY:
        return FALSE;
    case QUICKSTOP_RAMP_STAY:
        return TRUE;
    case STOP_ON_CURRENT_LIMIT_STAY:
        return FALSE;
    case STOP_ON_VOLTAGE_LIMIT_STAY:
        return FALSE;
    default:
        break;
    }
    return FALSE;
}
/*------------------------------------------------
Function:Pending Option Code
Input   :dd
Output  :No
Explain :该函数修改完成，入口参数可以分别选择两个电机的驱动参数结构体
------------------------------------------------*/
void PendingOptionCode()
{
    Drive_Data *pCiA402Axis;
    u16 obj6060_Control_mode =0 ;
    u16 obj6061_Control_mode_status = 0;
    u16 StatusWord = 0;
    u16 ControlWord6040 = 0;
    u8 counter = 0;
    for(counter = 0; counter < MAX_AXES; counter++)
    {
        if(!Driver_Data[counter].device_control_data.bAxisIsActive) //注意赋初值1
        {
            continue;
        }
        pCiA402Axis = &Driver_Data[counter];
        obj6060_Control_mode = pCiA402Axis->device_control_data.Modes_of_operation;                //当前操作模式（读写）
        obj6061_Control_mode_status = pCiA402Axis->device_control_data.Modes_of_operation_display; //显示的当前模式（只读）
        StatusWord = pCiA402Axis->device_control_data.Statusword;                                  //状态字
        ControlWord6040 = pCiA402Axis->device_control_data.Conrtolword;                            //控制字
        obj6061_Control_mode_status = obj6060_Control_mode;
        /*clear "Drive follows the command value" flag if the target values from the master overwritten by the local application*/
        if(pCiA402Axis->device_control_data.Pending_Option_code!= 0 &&\
                (obj6061_Control_mode_status == Profile_Position_Mode ||\
                 obj6061_Control_mode_status == Velocity_Mode ))
        {
            StatusWord &= ~STATUSWORD_DRIVE_FOLLOWS_COMMAND;//判断当前操作模式，来写对应状态字
        }
        else
        {
            StatusWord |= STATUSWORD_DRIVE_FOLLOWS_COMMAND;
        }
        switch(pCiA402Axis->device_control_data.Pending_Option_code)
        {
        /*state transition 11 is pending analyse shutdown option code (0x605A)*/
        case QUICK_STOP_CODE:
        {
            u16 ramp = pCiA402Axis->device_control_data.Quick_stop_option_code;

            /*masked and execute specified quick stop ramp characteristic */
            if(pCiA402Axis->device_control_data.Quick_stop_option_code > 4 && pCiA402Axis->device_control_data.Quick_stop_option_code <9)
            {
                if (pCiA402Axis->device_control_data.Quick_stop_option_code == 5)  ramp = 1;
                if (pCiA402Axis->device_control_data.Quick_stop_option_code == 6)  ramp = 2;
                if (pCiA402Axis->device_control_data.Quick_stop_option_code == 7)  ramp = 3;
                if (pCiA402Axis->device_control_data.Quick_stop_option_code == 8)  ramp = 4;
            }
            if(CiA402_TransitionAction(ramp)) //需要如何停机在CiA402_TransitionAction中实现
            {
                /* Wait Motor Stop */
//                if(Speed_PLL_Motor1 == 0) //当电机停止之后
//					{}
                /*quick stop ramp is finished complete state transition*/
                pCiA402Axis->device_control_data.Pending_Option_code = 0x0;
                if(pCiA402Axis->device_control_data.Quick_stop_option_code > 0 && pCiA402Axis->device_control_data.Quick_stop_option_code < 5)
                {
                    pCiA402Axis->drivesytemState = State_Switch_on_disabled;    //continue state transition 12
                }
                else if (pCiA402Axis->device_control_data.Quick_stop_option_code > 4 && pCiA402Axis->device_control_data.Quick_stop_option_code < 9)
                {
                    StatusWord |= STATUSWORD_TARGET_REACHED;
                }
            }
        }
        break;
        case SHUTDOWN_CODE:
        {
            /*state transition 8 is pending analyse shutdown option code (0x605B)*/
            {
                if(CiA402_TransitionAction(pCiA402Axis->device_control_data.Shutdown_option_code))
                {
                    pCiA402Axis->device_control_data.Shutdown_option_code = 0x0;
                    pCiA402Axis->drivesytemState = State_Ready_to_switch_on;    //continue state transition 8
                }
            }
        }
        break;
        case DISABLE_OPERATION_CODE:
        {
            /*state transition 5 is pending analyse Disable operation option code (0x605C)*/
            if(CiA402_TransitionAction(pCiA402Axis->device_control_data.Disable_operation_option_code))
            {
                pCiA402Axis->device_control_data.Disable_operation_option_code = 0;
                pCiA402Axis->drivesytemState = State_Switched_on;    //continue state transition 5
            }
        }
        break;
        case FAULT_REACT_CODE:
        {
            /*state transition 14 is pending analyse Fault reaction option code (0x605E)*/
            if(CiA402_TransitionAction(pCiA402Axis->device_control_data.Fault_reaction_option_code))
            {
                pCiA402Axis->device_control_data.Pending_Option_code = 0x0;
                pCiA402Axis->drivesytemState = State_Fault;    //continue state transition 14
            }
        }
        break;
        default:
        {
            /* pending transition code is invalid => values from the master are used */
            StatusWord |= STATUSWORD_DRIVE_FOLLOWS_COMMAND;
        }
        break;
        }
        pCiA402Axis->device_control_data.Statusword = StatusWord ; //修改状态字
    }
}
/* FUNCTION DECLAR END */
