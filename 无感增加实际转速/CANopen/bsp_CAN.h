#ifndef __BSP_CAN_H__
#define __BSP_CAN_H__

/* ����ͷ�ļ� ----------------------------------------------------------------*/
#include "stm32g4xx_hal.h"
#include <stdio.h>

/* ���Ͷ��� ------------------------------------------------------------------*/
/* �궨�� --------------------------------------------------------------------*/

#define CANx                            CAN2
#define CANx_CLK_ENABLE()               __HAL_RCC_CAN2_CLK_ENABLE()
#define CANx_FORCE_RESET()              __HAL_RCC_CAN2_FORCE_RESET()
#define CANx_RELEASE_RESET()            __HAL_RCC_CAN2_RELEASE_RESET()

#define CAN1_CLK_ENABLE()               __HAL_RCC_CAN1_CLK_ENABLE()
#define CANx_GPIO_CLK_ENABLE()          __HAL_RCC_GPIOB_CLK_ENABLE();
#define CANx_TX_GPIO_PORT               GPIOB
#define CAN_TX_Pin 											GPIO_PIN_13

#define CANx_RX_GPIO_PORT               GPIOB
#define CAN_RX_Pin											GPIO_PIN_5

#define CANx_RX_IRQn                   CAN2_RX0_IRQn
#define CANx_RX_IRQHandler             CAN2_RX0_IRQHandler

#define GPIO_AS_CAN                     GPIO_AF9_CAN2

/* ��չ���� ------------------------------------------------------------------*/


/* �������� ------------------------------------------------------------------*/
void MX_CAN2_Init(void);


uint8_t CAN2_Send_Msg(uint32_t stdId,uint32_t Ide,uint32_t Rtr,uint8_t* msg,uint8_t len);
#endif /* __BSP_CAN_H__ */


/******************* (C) COPYRIGHT 2015-2020 ӲʯǶ��ʽ�����Ŷ� *****END OF FILE****/
