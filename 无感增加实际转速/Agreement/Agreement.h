#ifndef __BSP_DEBUG_USART_H__
#define __BSP_DEBUG_USART_H__

/* ����ͷ�ļ� ----------------------------------------------------------------*/
#include "stm32g4xx_hal.h"
#include <stdio.h>
#include "stm32g4xx_hal_usart.h"
/* ���Ͷ��� ------------------------------------------------------------------*/
/* �궨�� --------------------------------------------------------------------*/

//#define DEBUG_USARTx_IRQn                            USART3_IRQn
//#define DEBUG_USARTx_IRQHandler                      USART3_IRQHandler



//#define FRAME_LENTH               6    // ָ���
#define FRAME_START               0x5A  // Э��֡��ʼ
//#define FRAME_END                 '/'   // Э��֡����
//#define FRAME_CHECK_BEGIN          1    // У���뿪ʼ��λ�� RxBuf[1]
//#define FRAME_CHECKSUM            14    // У�����λ��   RxBuf[14]
//#define FRAME_CHECK_NUM           13    // ��ҪУ����ֽ���
//#define FILL_VALUE                0x55  // ���ֵ
//#define CODE_SETPID               0x07  // ����PID����
//#define CODE_SETTGT               0x08  // ����Ŀ��ֵ
//#define CODE_RESET                0x09   // ��λ����
//#define CODE_STARTMOTOR           0x0A   // �������

#define ParaNum 250
/* ��չ���� ------------------------------------------------------------------*/

enum bReadEnum
{
	WR = 0,
	OR = 1,
};
enum bSaveToFlashEnum
{
	YSFLASH,
	NOFLASH,
};
extern UART_HandleTypeDef husart_debug;

extern uint8_t Flash_Writesign;

  

extern UART_HandleTypeDef huart3;

/* �������� ------------------------------------------------------------------*/
 void MX_USART1_UART_Init(void);              //USART��ʼ��

void Flash_WriteCheck(void);
void ReturnError1(void); 
void QuickRead(void);
void Startagreement(void);

extern void (*ptr_Fun_)(void) ;//����ָ��
/* �������� ------------------------------------------------------------------*/
//void MX_USARTx_Init(void);
//void MX_USART3_UART_Init(void);
typedef struct
{
  enum			bReadEnum  bReadOnly; 		//��д����
	enum			bSaveToFlashEnum bSaveToFlash; 	//�Ƿ�������Flsash
}PARAMETER_ATTRIBUTES;
//enum bOnly{RW,RO};
//enum bOnly bReadOnly;
//enum ToFlash{YSFLASH,NOFLASH};
//enum ToFlash bSaveToFlash;
typedef struct 
{
	short								sParID;                                                                                  
	short								sAddress; 		//ͨѶ������ַ	
	PARAMETER_ATTRIBUTES		        stAttributes;   //�������
//	short								sMaxArrayIndex; //�����������ֵ
//	long								lMinValue; 	    //��Сȡֵ 
//	long 								lMaxValue; 		//���ȡֵ��Χ
//	long								lDefaultValue;  //Ĭ��ֵ
    long*								lpParam; 		//��Ӧ����ӳ�� 
}PARAMETER_TABLE;

extern const PARAMETER_TABLE PARAMETER[];
//extern int32_t transbuf[250];
#endif  /* __BSP_DEBUG_USART_H__ */