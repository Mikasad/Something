#include "flash.h"
#include "Agreement.h"
#include "bsp_BDCMotor.h"
#include "main.h"
#include "function.h"
/* Ҫ�����ڲ�FLASH����ʼ��ַ */
#define TRUE 1;
#define FALSE 0;
int32_t tempbuf[200];
void Flash_Read_Init(int32_t *buf,u16 bufsize);
void FlashRight();
uint32_t GetPage(uint32_t Addr);
u8 InternalFlash(int32_t *data,u32 data_len);
uint8_t WriteData_toFlash(int32_t *data,uint32_t len,uint32_t address);
uint32_t FirstPage = 0, NbOfPages = 0;
uint32_t Address = 0, PageError = 0;
__IO uint32_t MemoryProgramStatus = 0;
#define FLASH_USER_START_ADDR   ADDR_FLASH_PAGE_40    
static FLASH_EraseInitTypeDef EraseInitStruct;


static uint32_t GetSector(uint32_t Address);

u8 InternalFlash(int32_t *data,u32 data_len)
{
	FLASH_EraseInitTypeDef EraseInitStruct;
    s32 i,j,temp32;
    static u32 tempi = 0;
    /* ����,ɾ����д������Ƚ��� */
    HAL_FLASH_Unlock();
	   __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_OPTVERR);
    /* ��ȡҪ�������׸����� */
    FirstPage = GetPage(FLASH_USER_START_ADDR);
    /* ��ȡҪ������������Ŀ */
    NbOfPages =20;// (data_len*4>ADDR_FLASH_PAGE_16 -FLASH_USER_START_ADDR)?16:15;
    /* ��ʼ�������ṹ�� */
    EraseInitStruct.TypeErase   = FLASH_TYPEERASE_PAGES ;//��������
	   EraseInitStruct.Banks=1;
    EraseInitStruct.Page        = FirstPage;
    EraseInitStruct.NbPages     = NbOfPages;
    
//		__HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_EOP | FLASH_FLAG_OPERR | FLASH_FLAG_WRPERR |FLASH_FLAG_PGAERR | FLASH_FLAG_PGPERR|FLASH_FLAG_PGSERR);
    if(HAL_FLASHEx_Erase(&EraseInitStruct, &PageError) != HAL_OK)
    {
        /* �˴�����ʹ��HAL_FLASH_GetError()����ȡ��������������� */
//		   	HAL_FLASH_GetError();
        tempi =0;
        return FALSE;
    }
	    	Address = FLASH_USER_START_ADDR;
		    for(i=0; i<data_len; i++)
      {
        if(PARAMETER[i].stAttributes.bSaveToFlash==YSFLASH)
        {
            tempbuf[tempi] = data[i];
            tempi++;
        }
	  }
		for(i=0; i<tempi; i++)
		{
			if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, Address,tempbuf[i]) == HAL_OK)
				{
					Address = Address + 8; 
				
				}
				else
				{
					 return FALSE;
				}
			
		}
    /* ����������Ƕ�ȡ�Ļ���������� */
    HAL_FLASH_Lock();
    MemoryProgramStatus = 0;
		Address = FLASH_USER_START_ADDR;
    /* ��ȡ������֤�Ƿ��д���������ͬ�������ͬ����ôMemoryProgramStatus=0 */
   
    for(j=0; j<tempi; j++)
    {
			temp32= *(__IO int32_t*)(Address+j*8);
        if((tempbuf[j])!= temp32 )
        {
            MemoryProgramStatus++;
        }
    }
		tempi=0;
		
		 if (MemoryProgramStatus)
    {
        return FALSE;
    }
    else
    {
        return TRUE;
    }
}
static uint32_t GetPage(uint32_t Addr)
{
  return (Addr - FLASH_BASE) / FLASH_PAGE_SIZE;;
}

void Flash_Read_Init(int32_t *buf,u16 bufsize)
{
    u16 i;
    static u32 address = FLASH_USER_START_ADDR;
    for(i=0; i<bufsize; i++)
    {
        if(PARAMETER[i].stAttributes.bSaveToFlash ==YSFLASH)
        {
            buf[i] = *(__IO uint32_t*)(address);
            address = address+8;
//			      address = address+8;
        }
    }
}