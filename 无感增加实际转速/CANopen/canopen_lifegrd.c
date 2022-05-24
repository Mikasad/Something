#include "canopen_lifegrd.h"
#include "main.h"
#include "stdbool.h"
/*------------------------------------------------
Function:������������ڶ�ʱ�¼��ص�����
Input   :No
Output  :No
Explain :No
------------------------------------------------*/
void ConsumerHeartbeatAlarm(CO_Data* d, unsigned int id)
{

    unsigned char nodeId = (unsigned char)(((d->ConsumerHeartbeatEntries[id]) & (unsigned int)0x00FF0000) >> (unsigned char)16);

    d->ConsumerHeartBeatTimers[id] = TIMER_NONE;

    d->NMTable[nodeId] = DIS_CONNECTED;

    (*d->heartbeatError)(d, nodeId);
}
/*------------------------------------------------
Function:����ڵ��ػ�����
Input   :No
Output  :No
Explain :No
------------------------------------------------*/
void HeartBeat_Message_Handle(CO_Data *d, CanRxMsgTypeDef *m)
{
    unsigned char nodeId = (unsigned char)GET_NODE_ID(m->StdId);

    if(m->RTR == RTR_VALID)
    {
        if(nodeId == d->Node_ID)
        {
            CanTxMsgTypeDef msg;

            /* MNT����(����֡��ʾ��Ӧ��Զ��֡��ʾ����) */
            unsigned short tmp = d->Node_ID + 0x700;
            /* cob-id */
            msg.StdId = UNS16_LE(tmp);
            /* ���� */
            msg.DLC = (unsigned char)0x01;
            /* ����֡����Ӧ */
            msg.RTR = RTR_INVALID;
            /* �ڵ�״̬ */
            msg.Data[0] = d->Node_state;
            /* ͬ����־ */
            if(pHeartBeatPar.toggle)
            {
                msg.Data[0] |= 0x80;
                pHeartBeatPar.toggle = 0;
            }
            else
                pHeartBeatPar.toggle = 1;

            /* ���ͱ��� */
            CAN2_Send_Msg(msg.StdId,CAN_ID_STD,msg.RTR,msg.Data,msg.DLC);
        }
    }
    else
    {
        unsigned char New_NodeState = (unsigned char)((*m).Data[0] & 0x7F);

        if(d->NMTable[nodeId] != New_NodeState)
        {
            (*d->post_SlaveStateChange)(d, nodeId, New_NodeState);
            d->NMTable[nodeId] = New_NodeState;
        }
        /* ����ýڵ��ϱ������Ĳ���δ֪״̬����Ҫ�Ըýڵ�������µ���ʱ */
        if(d->NMTable[nodeId] != UNKNOWN_STATE)
        {
            unsigned char i,ConsumerHeartBeat_nodeId;

            for(i = (unsigned char)0x00; i < *d->ConsumerHeartbeatCount; i++)
            {
                ConsumerHeartBeat_nodeId = (unsigned char)(((d->ConsumerHeartbeatEntries[i]) & (unsigned int)0x00FF0000) >> (unsigned char)16);
                if(nodeId == ConsumerHeartBeat_nodeId)
                {
                    TIMEVAL time = ((d->ConsumerHeartbeatEntries[i]) & (unsigned int)0x0000FFFF);
                    DelAlarm(d->ConsumerHeartBeatTimers[i]);
                    /* ��������������(��վ)���������(��վ)��ʱ�¼� */
                    d->ConsumerHeartBeatTimers[i] = SetAlarm(d, i, &ConsumerHeartbeatAlarm, MS_TO_TIMEVAL(time), 0);
                }
            }
        }
    }
}

/*------------------------------------------------
Function:������(��վ)��������ʱ�¼��ص�����
Input   :No
Output  :No
Explain :No
------------------------------------------------*/
void ProducerHeartbeatAlarm(CO_Data *d, unsigned int id)
{
    if(pHeartBeatPar.Timeout_server)
    {
        CanTxMsgTypeDef msg;

        /* MNT����(����֡��ʾ��Ӧ��Զ��֡��ʾ����) */
        unsigned short tmp = d->Node_ID + 0x700;

        /* cob-id */
        msg.StdId = UNS16_LE(tmp);
        /* ���� */
        msg.DLC = (unsigned char)0x01;
        /* ����֡����Ӧ */
        msg.RTR = RTR_INVALID;
        /* �ڵ�����״̬ */
        msg.Data[0] = d->Node_state;

        /* ���͸�֡ */
        CAN2_Send_Msg(msg.StdId,CAN_ID_STD,msg.RTR,msg.Data,msg.DLC);
    }
    else
    {
        pHeartBeatPar.Timeout_server_timer = DelAlarm(pHeartBeatPar.Timeout_server_timer);
    }
}

/*------------------------------------------------
Function:��ʼ����������
Input   :No
Output  :No
Explain :No
------------------------------------------------*/
void heartbeatInit(CO_Data *d)
{
    unsigned char i = 0;

    RegisterSetOnDentry_Callback(d, 0x1017, 0x00, &OnHeartbeatProducerUpdate);

    /* ͬ����־λ��0 */
    pHeartBeatPar.toggle = 0;

#ifdef _INITCONSUME
    for(i = 0x00; i < *d->ConsumerHeartbeatCount; i++)
    {
        TIMEVAL ConsumeTime = (unsigned short int)((d->ConsumerHeartbeatEntries[i]) & (unsigned int)0x0000FFFF);

        if(ConsumeTime)
        {
            d->ConsumerHeartBeatTimers[i] = SetAlarm(d, i, &ConsumerHeartbeatAlarm, MS_TO_TIMEVAL(ConsumeTime), 0);
        }
    }
#endif
    /* ������(��վ)ע��һ����ʱ�¼����ڹ涨ʱ�����ϱ������� */
    if(pHeartBeatPar.Timeout_server)
    {
        TIMEVAL time = pHeartBeatPar.Timeout_server;
        pHeartBeatPar.Timeout_server_timer = SetAlarm(d, 0, &ProducerHeartbeatAlarm, MS_TO_TIMEVAL(time), MS_TO_TIMEVAL(time));
    }
}

/*------------------------------------------------
Function:ֹͣ������
Input   :No
Output  :No
Explain :No
------------------------------------------------*/
void heartbeatStop(CO_Data *d)
{

    /* ɾ��������(��վ)��������ʱ�� */
    pHeartBeatPar.Timeout_server_timer = DelAlarm( pHeartBeatPar.Timeout_server_timer);
}

/*------------------------------------------------
Function:����ֵ�������Ϊ0x1017����Ŀ�����仯�����øú���
Input   :No
Output  :No
Explain :No
------------------------------------------------*/
unsigned int OnHeartbeatProducerUpdate(CO_Data *d, const indextable *unused_indextable, unsigned char unused_bSubindex)
{
    heartbeatStop(d);
    heartbeatInit(d);

    return 0;
}
/*------------------------------------------------
Function:�����ػ����Ƴ�ʼ��
Input   :No
Output  :No
Explain :No
------------------------------------------------*/
void lifeGuardInit(CO_Data *d)
{
    heartbeatInit(d);
}

/*------------------------------------------------
Function:�����ػ�����ֹͣ
Input   :No
Output  :No
Explain :No
------------------------------------------------*/
void lifeGuardStop(CO_Data *d)
{
    heartbeatStop(d);
}


/*------------------------------------------------
Function:��������ص�����
Input   :No
Output  :No
Explain :No
------------------------------------------------*/
u8 _heartbeatErrorFlag;
extern  bool MC_StopMotor1(), MC_StopMotor2();
void _heartbeatError(CO_Data *d, unsigned char heartbeatID)
{
    MC_StopMotor1();
//    for(u8 i=0; i< MOTOR_NUM ; i++)
//    {
//        MotorControl[i].Motor_Start_Stop = DISABLE;
    MC_StopMotor2();
    _heartbeatErrorFlag = 2;
//	MotorState = RUN;
}
/*------------------------------------------------
Function:��صĽڵ�״̬�ı�ص�����
Input   :No
Output  :No
Explain :No
------------------------------------------------*/
void _post_SlaveStateChange(CO_Data* d, unsigned char nodeId, unsigned char newNodeState)
{
//  ATLAS_PRINT("Node:%d  State Change to:%d\r\n",nodeId,newNodeState);
}
/*------------------------------------------------
Function:������������ʱ�������ı�ص�����
Input   :No
Output  :No
Explain :No
------------------------------------------------*/
unsigned int  ODCallback_t_Index1016_Subindex(CO_Data *d, const indextable *index, unsigned char bSubindex)
{
    unsigned int IndexOffset = 19;
    unsigned int newConsumerNodeID = 0;
    unsigned int pastConsumerNodeID = 0;
    unsigned char i = 0;

    /* ������������ʱ������0��0-5 */
    if(bSubindex == 0)
    {
        if((*(unsigned char *)index->pSubindex[bSubindex].lpParam < 0)||(*(unsigned char *)index->pSubindex[bSubindex].lpParam > 5))
        {
            memcpy(index->pSubindex[bSubindex].lpParam, d->LastObj.Data, d->LastObj.size);
            return SDOABT_GENERAL_ERROR;
        }
    }
    else
    {
        newConsumerNodeID = (unsigned char)(((*(unsigned int *)index->pSubindex[bSubindex].lpParam) & (unsigned int)0x00FF0000) >> (unsigned char)16);
        for(i=0; i<5; i++)
        {
            pastConsumerNodeID = (bSubindex != (i+1))?(unsigned char)(((*(unsigned int *)d->objdict[IndexOffset].pSubindex[i+1].lpParam) & (unsigned int)0x00FF0000) >> (unsigned char)16):0;
            if(newConsumerNodeID == pastConsumerNodeID)
            {
                memcpy(index->pSubindex[bSubindex].lpParam, d->LastObj.Data, d->LastObj.size);
                return GEERAL_PARA_INCOMPATIBILITY_REASON;
            }
        }
    }

    return 0;
}
