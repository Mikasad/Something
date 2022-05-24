#include "ds402.h"
#include "mc_config.h"
/* VAR DECLAR BEGIN */
Drive_Data Driver_Data[MAX_AXES];

Drive_Data Driver_Data_M1=
{
    .device_control_data =
    {
        . bAxisIsActive = 2,      //!0Ϊʹ��M1�����402״̬��
        . Modes_of_operation = 3, //Ĭ��Ϊ����ģʽΪ���ٶ�ģʽ
    },
};

Drive_Data Driver_Data_M2=
{
    .device_control_data =
    {
        . bAxisIsActive = 2,      //!0Ϊʹ��M2�����402״̬��
        . Modes_of_operation = 3, //Ĭ��Ϊ����ģʽΪ���ٶ�ģʽ
    },
};

/* VAR DECLAR END */

/* ---SEGMENT--- */

/* FUNCTION DECLAR BEGIN */
/*------------------------------------------------
Function:402Э�������ʼ��
Input   :No
Output  :No
Explain :����bAxisIsActive��ֵ��Ϊ0
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
    int DI_SERVO_ON;//���xx DinPort����DI SERVO ON����\r\n��IO����Ϊ��ͨ����������ڣ�mode���ܱ������ˣ���⵽�������ź���Ч
    int mergStopRequest[1];//���Emcy Stop REQ��En Internal(DI CMD��Communication CMD)
    int disableStopOp[1]; //ʹ��ֹͣ��ʽѡ�� 0:free stop   1:immediatate stop
    int motionStat[1];
    int faultStopOp[1]; //����ͣ����ʽ
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
Function:402״̬��
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
    if(Sync_Async_Control == 1)//0��M1��M2�첽����  1��M1��M2ͬ������
    {
        Driver_Data[M2].device_control_data.Conrtolword = Driver_Data[M1].device_control_data.Conrtolword;
    }
    for(counter = 0; counter < MAX_AXES; counter++)
    {
        if(!Driver_Data[counter].device_control_data.bAxisIsActive) //ע�⸳��ֵ1
        {
            continue;
        }
			
        pCiA402Axis = &Driver_Data[counter];
        StatusWord = pCiA402Axis->device_control_data.Statusword;
        ControlWord6040 = pCiA402Axis->device_control_data.Conrtolword; 
				
				/* Status Bit13  EN/DISEN */
        if(STM[counter].bState == RUN)   //�жϵ���Ƿ�ʹ��
        {
            StatusWord |= Bit_Motor_En; //������ʹ��״̬��
        }
        else StatusWord &= ~Bit_Motor_En;
				
        switch(pCiA402Axis->drivesytemState)
        {
        case State_Start: //��ʼ
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
        case State_Not_ready_to_switch_on: //��ʼ��
        {
            StatusWord |= (STATUSWORD_STATE_NOTREADYTOSWITCHON);/* Update StatuWord */
            if(Angle_Switch == 3) //�������
            {
                pCiA402Axis->drivesytemState = State_Switch_on_disabled;/* Update Driver State */
            }
            else
            {
                pCiA402Axis->drivesytemState = State_Not_ready_to_switch_on; //�����ڵ�ǰ״̬
            }
        }
        break;
        case State_Switch_on_disabled://�ŷ��޹���
        {
            StatusWord |= (STATUSWORD_STATE_SWITCHEDONDISABLED);
//      pCiA402Axis->device_control_data.bLowLevelPowerApplied = TRUE;  //������BUG�ڴ���״̬�·��Ϳ�����0x0080�����״̬�ı䣬���ǲ����������
            if((ControlWord6040 & CONTROLWORD_COMMAND_SHUTDOWN_MASK ) == CONTROLWORD_COMMAND_SHUTDOWN)
            {
//      pAxisPar.statRegs[A_AXIS] |= STAT_REG_AMPLIFIER_MAIN_RELAY_SET; // ����6����׼����
                pCiA402Axis->drivesytemState = State_Ready_to_switch_on; // Transition 2
            }
        }
        break;
        case State_Ready_to_switch_on://�ŷ�׼����
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
                if ( RealBusVoltageSensorParamsM1._Super.FaultState != MC_NO_ERROR ) //��ѹ����쳣
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
        case State_Switched_on://�ȴ��ŷ���ʹ��
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
                    MC_StartMotor1();//ʹ�ܵ��M1
                }
                else if(counter == M2&&STM[counter].bState == IDLE)
                {
                    MC_StartMotor2();//ʹ�ܵ��M2
                }
                if(STM[counter].bState == RUN)
                {
                    pCiA402Axis->drivesytemState = State_Operation_enable;
                }
            }
        }
        break;
        case State_Operation_enable: //�ŷ�����
        {
            StatusWord |= (STATUSWORD_STATE_OPERATIONENABLED);
            if(STM[counter].bState == FAULT_NOW||STM[counter].bState ==FAULT_OVER) //��������Զ�����State_Fault_reaction_active
            {
                pCiA402Axis->drivesytemState = State_Fault_reaction_active;
            }
            /* If xx DinPort config DI_SERVO_ON Function */
//      if (pAxisPar.DI_SERVO_ON== 3)
//			{
//				printf("���xx DinPort����DI SERVO ON����\r\n��IO����Ϊ��ͨ����������ڣ�mode���ܱ������ˣ���⵽�������ź���Ч\r\n");
//				if (pAxisPar.motorEn[A_AXIS] == MOTOR_ON)
//				{
//					ControlWord6040 &= ~Bit_Enable_Operation;		//�ŷ�����λ  0��Ч 1��Ч
//				}
//			}
            /* If Emcy Stop REQ is En Internal(DI CMD or Communication CMD) */
//      if (pAxisPar.mergStopRequest[A_AXIS])
//			{
//				printf("���Emcy Stop REQ��En Internal(DI CMD��Communication CMD)");
//				ControlWord6040 &= Bit_Enable_Voltage;  //��ͨ����·��λ  0��Ч 1��Ч
//			}
            /* DI CMD or Communication CMD */
//      if(STM[counter].bState != RUN) //����������δʹ�ܻ�ǿ����ת��
//      {
//        ControlWord6040 &= CONTROLWORD_COMMAND_DISABLEVOLTAGE;
//      }

            /* When Control Word Command is set DISABLEOPERATION */
            if ((ControlWord6040 & CONTROLWORD_COMMAND_DISABLEOPERATION_MASK) == CONTROLWORD_COMMAND_DISABLEOPERATION)
            {
                if(pCiA402Axis->device_control_data.Shutdown_option_code != DISABLE_DRIVE) //ʹ��ֹͣ��ʽѡ�� 0:free stop   1:immediatate stop  ע�����ֵӦ��д��605B�����ֵ䵱��,�ڿ��Ƶ����޸����ֵ
                {
                    /*disable operation pending*/
                    pCiA402Axis->device_control_data.Pending_Option_code= DISABLE_OPERATION_CODE;
                    return;
                }
                pCiA402Axis->drivesytemState = State_Switched_on;	// Transition   5
            }
            /* When Control Word Command is set QUICKSTOP */
            else if((ControlWord6040 & CONTROLWORD_COMMAND_QUICKSTOP_MASK) == CONTROLWORD_COMMAND_QUICKSTOP)  //����ͣ��
            {
                StatusWord &= ~Bit_Quick_stop;
                pCiA402Axis->drivesytemState = State_Quick_stop_active;	// Transition 11
                return;
            }
            /* When Control Word Command is set SHUTDOWN */
            else if((ControlWord6040 & CONTROLWORD_COMMAND_SHUTDOWN_MASK) == CONTROLWORD_COMMAND_SHUTDOWN)
            {
                if(pCiA402Axis->device_control_data.Shutdown_option_code != DISABLE_DRIVE)  //�ػ�ѡ�����  ����shutdownѡ���������ڷ���ת��ʱӦ�ò�ȡʲô���� ע�����ֵӦ��д��605B�����ֵ䵱��,�ڿ��Ƶ����޸����ֵ
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

            else if((ControlWord6040 & CONTROLWORD_COMMAND_RELATIVE_MASK ) == CONTROLWORD_COMMAND_RELATIVE)//����λ�ÿ���ģʽ
            {
                relative_location_flag[counter] = 1;
            }
            /* Halt */
        }
        break;
        case State_Quick_stop_active: //����ͣ��״̬
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
//        if ((ControlWord6040& CONTROLWORD_COMMAND_HALT_MASK) == CONTROLWORD_COMMAND_HALT) //�����Զ���������
//        {;}
            /* When QuickStop Code is Exist */
            if((pCiA402Axis->device_control_data.Quick_stop_option_code!= DISABLE_DRIVE)   //obj605A_QuickStop_optcode  ע�����ֵӦ��д��605B�����ֵ䵱��,�ڿ��Ƶ����޸����ֵ
                    &&((StatusWord & STATUSWORD_STATE_MASK)!= STATUSWORD_STATE_QUICKSTOPACTIVE))
            {
                /*Only execute quick stop action in state transition 11*/
//                if(pCiA402Axis->device_control_data.Pending_Option_code == 0)
                pCiA402Axis->device_control_data.Pending_Option_code= QUICK_STOP_CODE;
                return;
            }
            /* OpCode-0,Direct Switch State */
            else //δʹ��ͣ����ʽֱ���л�״̬��
            {
                pCiA402Axis->drivesytemState = State_Switch_on_disabled;    //continue state transition 12 ����ͣ����ʽ605Aѡ��Ϊ1-8��ͣ����ɺ���Ȼ���ȣ�����ָ�����
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
            if(pCiA402Axis->device_control_data.Fault_reaction_option_code != DISABLE_DRIVE) //�������Ϸ�Ӧѡ������������������з������ϣ�Ӧ�ò�ȡʲô�ж���
            {
                /*fault reaction pending*/
                pCiA402Axis->device_control_data.Pending_Option_code = FAULT_REACT_CODE;//����ִ�е�ѡ�����
                return;
            }
            pCiA402Axis->drivesytemState = State_Fault;// Transition 14 ����ͣ����ɺ���Ȼ���ɣ��������ָ��
        }
            break;  /*NOTE: it is not recommend to support transition 16 */
        case State_Fault:
        {
            StatusWord |= (STATUSWORD_STATE_FAULT);

            if (STM[counter].hFaultOccurred == 0 && STM[counter].hFaultNow == 0) //�ù�����Ҫ�ֶ�����������ǰû�й��ϣ��Զ����ȹ��ϡ��ŷ��޹���
            {
                if(counter==M1)
                {
									STM[counter].bState = STOP_IDLE; //ʧ�ܵ��
									ControlWord6040 = CONTROLWORD_COMMAND_FAULTRESET;
                }
                else  if(counter==M2)
                {
									STM[counter].bState = STOP_IDLE; //ʧ�ܵ��
									ControlWord6040 = CONTROLWORD_COMMAND_FAULTRESET;
                }
            }
            /* When Control Word Command is set FAULTRESET ������������0x80ָ���л������������֮���Զ��л�*/
            if ((ControlWord6040 & CONTROLWORD_COMMAND_FAULTRESET_MASK) == CONTROLWORD_COMMAND_FAULTRESET)
            {
                pAxisPar.sAlarmClcRequest[A_AXIS] = TRUE;
                pCiA402Axis->drivesytemState  = State_Switch_on_disabled; // Transition 15 ���ϡ��ŷ��޹���
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
        if(!Driver_Data[counter].device_control_data.bAxisIsActive) //ע�⸳��ֵ1
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
            if(Angle_Switch == 3) //������ɣ��ٽ�����ص�,�����ڵ�һ���ϵ��ʱ�����ڻ����������ʹ�ܵ�����ᵼ�»���ʧ�ܣ�����״̬����Ҫʹ�ܵ����������Ҫ�ж��ĸ�����������
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
Explain :���ڸú�����ͣ��ʱ�����κδ���
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
Explain :�ú����޸���ɣ���ڲ������Էֱ�ѡ��������������������ṹ��
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
        if(!Driver_Data[counter].device_control_data.bAxisIsActive) //ע�⸳��ֵ1
        {
            continue;
        }
        pCiA402Axis = &Driver_Data[counter];
        obj6060_Control_mode = pCiA402Axis->device_control_data.Modes_of_operation;                //��ǰ����ģʽ����д��
        obj6061_Control_mode_status = pCiA402Axis->device_control_data.Modes_of_operation_display; //��ʾ�ĵ�ǰģʽ��ֻ����
        StatusWord = pCiA402Axis->device_control_data.Statusword;                                  //״̬��
        ControlWord6040 = pCiA402Axis->device_control_data.Conrtolword;                            //������
        obj6061_Control_mode_status = obj6060_Control_mode;
        /*clear "Drive follows the command value" flag if the target values from the master overwritten by the local application*/
        if(pCiA402Axis->device_control_data.Pending_Option_code!= 0 &&\
                (obj6061_Control_mode_status == Profile_Position_Mode ||\
                 obj6061_Control_mode_status == Velocity_Mode ))
        {
            StatusWord &= ~STATUSWORD_DRIVE_FOLLOWS_COMMAND;//�жϵ�ǰ����ģʽ����д��Ӧ״̬��
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
            if(CiA402_TransitionAction(ramp)) //��Ҫ���ͣ����CiA402_TransitionAction��ʵ��
            {
                /* Wait Motor Stop */
//                if(Speed_PLL_Motor1 == 0) //�����ֹ֮ͣ��
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
        pCiA402Axis->device_control_data.Statusword = StatusWord ; //�޸�״̬��
    }
}
/* FUNCTION DECLAR END */
