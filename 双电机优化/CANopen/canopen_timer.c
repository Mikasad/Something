#include "canopen_timer.h"

TIM_HandleTypeDef TIM6_Handler;
/* VARIABLE DECLARE BEGIN */

s_timer_entry timers[MAX_NB_TIMER] = {{TIMER_FREE, NULL, NULL, 0, 0, 0},};

TIMEVAL total_sleep_time = TIMEVAL_MAX;
TIMER_HANDLE last_timer_raw = -1;
TIMEVAL last_time_set = 0;

unsigned int TimeCNT=0;             //ʱ�����
unsigned int NextTime=0;            //��һ�δ���ʱ�����

/* VARIABLE DECLARE END */

/*
** -------  Use this to declare a new alarm ------
**
** @param d
** @param id
** @param callback
** @param value
** @param period
**
** @return
**/
TIMER_HANDLE SetAlarm(CO_Data* d, unsigned int id, TimerCallback_t callback, TIMEVAL value, TIMEVAL Timecycle)
{
	TIMER_HANDLE row_number;
	s_timer_entry *row;

	for(row_number=0, row=timers; row_number <= last_timer_raw + 1 && row_number < MAX_NB_TIMER; row_number++, row++)
	{
		if (callback && 	           /* if something to store */
		   row->state == TIMER_FREE) /* and empty row */
		{	/* just store */
			TIMEVAL real_timer_value;
			TIMEVAL elapsed_time;
			if (row_number == last_timer_raw + 1) last_timer_raw++;
      
      /* ��ȡ��ʱ����ǰ�Ѿ����ŵ�ʱ�� */
			elapsed_time = getElapsedTime();
			real_timer_value = value;
			real_timer_value = min_val(real_timer_value, TIMEVAL_MAX);
      
			if (total_sleep_time > elapsed_time && total_sleep_time - elapsed_time > real_timer_value)
			{
				total_sleep_time = elapsed_time + real_timer_value;
        /* ����CANopen timer����ֵ */
				setTimer(real_timer_value);
			}
			row->callback = callback;
			row->d = d;
			row->id = id;
			row->val = value + elapsed_time;
			row->interval = Timecycle;
			row->state = TIMER_ARMED;
			return row_number;
		}
	}

	return TIMER_NONE;
}

/*----------------------------
*Fuction��ɾ����ʱ�¼�
*Explain��No
----------------------------*/
TIMER_HANDLE DelAlarm(TIMER_HANDLE handle)
{
	
//  ATLAS_PRINT("DelAlarm. handle = %d\r\n",handle);
	
	/* ���ö�ʱ�¼����״̬��Ϊ���� */
	if(handle != TIMER_NONE)
	{
		if(handle == last_timer_raw)
			last_timer_raw--;
		timers[handle].state = TIMER_FREE;
	}
	
	/* ���ش��� */
	return TIMER_NONE;
}
/*----------------------------
*Fuction��CANoen timer dispatch
*Explain��No
----------------------------*/
void TimeDispatch(void)
{
	TIMER_HANDLE i;
	TIMEVAL next_wakeup = TIMEVAL_MAX;
	
	unsigned int overrun = (unsigned int)getElapsedTime();

	TIMEVAL real_total_sleep_time = total_sleep_time + overrun;

	s_timer_entry *row;
  

	/* ������ʱ�¼���� */
	for(i = 0, row = timers; i <= last_timer_raw; i++, row++)
	{
		if(row->state & TIMER_ARMED)
		{
			if(row->val <= real_total_sleep_time)
			{
				if((row->interval) == NULL)
				{
					row->state = TIMER_TRIG;
				}
				else
				{
					row->val = row->interval - (overrun % (unsigned int)row->interval);
					row->state = TIMER_TRIG_PERIOD;
					
					if(row->val < next_wakeup)
						next_wakeup = row->val;
				}
			}
			/* û�г�ʱ������Ҫ���� */
			else
			{
				row->val -= real_total_sleep_time;

				if(row->val < next_wakeup)
					next_wakeup = row->val;
			}
		}
	}

	total_sleep_time = next_wakeup;

	setTimer(next_wakeup);

	for(i = 0, row = timers; i <= last_timer_raw; i++, row++)
	{
		if(row->state & TIMER_TRIG)
		{
			row->state &= ~TIMER_TRIG;
			
			if(row->callback)
				(*row->callback)(row->d, row->id);
		}
	}
}

/*----------------------------
*Fuction��ͨ�ö�ʱ��5��ʼ������
*Explain��No
----------------------------*/
void TimerForCan(void)
{
    TimeCNT++;
    
    if (TimeCNT >= CANOPEN_TIMER_CNT_MAX)
    {
        TimeCNT=0;
    }
    if (TimeCNT == NextTime)
    {
        last_time_set = TimeCNT;
        TimeDispatch();     //��ʱʱ�䵽��ִ��ʱ����صķַ�����
    }
}
/*----------------------------
*Fuction���趨CANopen timer �¼�����ֵ
*Explain��No
----------------------------*/
void setTimer(TIMEVAL value)
{ 
    NextTime = (TimeCNT+value)%CANOPEN_TIMER_CNT_MAX;
}
/*----------------------------
*Fuction��Get CANopen timer escape time
*Explain��No
----------------------------*/
TIMEVAL getElapsedTime(void)
{
    unsigned int ret = TimeCNT;
    
    return ret > last_time_set ? ret - last_time_set : last_time_set - ret;
}




/* CALLBACK TEST BEGIN */
void Callbacktest(CO_Data* d, unsigned int id)
{
  CanTxMsgTypeDef msg;
		
  unsigned short tmp = d->Node_ID + 0x100;
  
  /* cob-id */
  msg.StdId = UNS16_LE(tmp);
  /* ���� */
  msg.DLC = (unsigned char)0x01;
  /* ����֡����Ӧ */
  msg.RTR = RTR_INVALID;
  /* �ڵ�����״̬ */
  msg.Data[0] = d->Node_state;
  
  /* ���͸�֡ */
  CAN1_Send_Msg(msg.StdId,CAN_ID_STD,msg.RTR,msg.Data,msg.DLC);
}

void Callbacktest1(CO_Data* d, unsigned int id)
{
  CanTxMsgTypeDef msg;
		
  unsigned short tmp = d->Node_ID + 0x200;
  
  /* cob-id */
  msg.StdId = UNS16_LE(tmp);
  /* ���� */
  msg.DLC = (unsigned char)0x01;
  /* ����֡����Ӧ */
  msg.RTR = RTR_INVALID;
  /* �ڵ�����״̬ */
  msg.Data[0] = d->Node_state;
  
  /* ���͸�֡ */
  CAN1_Send_Msg(msg.StdId,CAN_ID_STD,msg.RTR,msg.Data,msg.DLC);
}
void Callbacktest2(CO_Data* d, unsigned int id)
{
  CanTxMsgTypeDef msg;
		
  /* ��ʱ���Ա���(����֡��ʾ��Ӧ��Զ��֡��ʾ����) */
  unsigned short tmp = d->Node_ID + 0x300;
  
  /* cob-id */
  msg.StdId = UNS16_LE(tmp);
  /* ���� */
  msg.DLC = (unsigned char)0x01;
  /* ����֡����Ӧ */
  msg.RTR = RTR_INVALID;
  /* �ڵ�����״̬ */
  msg.Data[0] = d->Node_state;
  
  /* ���͸�֡ */
  CAN1_Send_Msg(msg.StdId,CAN_ID_STD,msg.RTR,msg.Data,msg.DLC);
}
/* CALLBACK TEST END */
/*----------------------------
*Fuction��ͨ�ö�ʱ��5��ʼ������
*Explain��No
----------------------------*/	
void TIM6_Init(void)
{
  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;

  __HAL_RCC_TIM6_CLK_ENABLE();
  TIM6_Handler.Instance = TIM6;
  TIM6_Handler.Init.Prescaler = TIMER6_DIV;										//[��Ƶ84MHz]		[14MHz]
  TIM6_Handler.Init.CounterMode = TIM_COUNTERMODE_UP;
  TIM6_Handler.Init.Period = TIMER6_PERIOD;										//[14MHz/13999+1=1K T=1ms]						[T=1000us]
  TIM6_Handler.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  HAL_TIM_Base_Init(&TIM6_Handler);

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  HAL_TIM_ConfigClockSource(&TIM6_Handler, &sClockSourceConfig);

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  HAL_TIMEx_MasterConfigSynchronization(&TIM6_Handler, &sMasterConfig);  
	
  HAL_TIM_Base_Start_IT(&TIM6_Handler);                               //���ж�ģʽ��������ʱ��

}
/*----------------------------
*Fuction��ͨ�ö�ʱ��6�жϷ�����
*Explain��100us
----------------------------*/	
void TIM6_DAC_IRQHandler(void)
{ 
  uint16_t Itstatus = 0x0,Itenable = 0x0;
  
  Itstatus = TIM6->SR & TIM_IT;
  Itenable = TIM6->DIER & TIM_IT;
  if((Itstatus != RESET)&&(Itenable != RESET))
  {
    TIM6->SR &= ~TIM_IT;
    /* CANopen timer */
    TimerForCan();
  }
}
