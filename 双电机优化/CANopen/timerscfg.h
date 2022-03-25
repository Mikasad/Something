#ifndef __TIMERSCFG_H__
#define __TIMERSCFG_H__

/* MACRO DEFINITION BEGIN */
// Whatever your microcontroller, the timer wont work if
// TIMEVAL is not at least on 32 bits
#define TIMEVAL unsigned int 

// The timer of the STM32 counts from 0000 to 0xFFFF
#define TIMEVAL_MAX 0xFFFFFFFF

// The timer is incrementing every 1 us.
#define MS_TO_TIMEVAL(ms) ((ms) * 1)
#define US_TO_TIMEVAL(us) ((us) * 1)

#define TIMER6_DIV            5
#define TIMER6_PERIOD         13999
#define TIM_IT                0x00000001

#define CANOPEN_TIMER_FREQ    14000000
#define CANOPEN_TIMER_PERIOD  0.0001

#define CANOPEN_TIMER_CNT_MAX 70000
#define MAX_NB_TIMER 8                    //�����趨ʱ������¼���

#define TIMER_HANDLE short
/* ��ʱ�¼����״̬ */
#define TIMER_FREE 0					/* ���У��������û�б���ʱ�¼�ռ�� */
#define TIMER_ARMED 1					/* ��ռ�ã���������Ѿ���ʱ�¼�ռ�� */
#define TIMER_TRIG 2					/* ���δ��� */
#define TIMER_TRIG_PERIOD 3		/* ���ڴ��� */

#define TIMER_NONE -1					/* ���󷵻� */

/* ����Сֵ */
#define min_val(a,b) ((a<b)?a:b)



#endif
