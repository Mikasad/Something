#include "flash.h"
#include "Agreement.h"
#include "bsp_BDCMotor.h"


/* Ҫ�����ڲ�FLASH����ʼ��ַ */
#define FLASH_USER_START_ADDR   ADDR_FLASH_SECTOR_6
#define FLASH_USER_END_ADDR			ADDR_FLASH_SECTOR_7
uint32_t FirstSector = 0, NbOfSectors = 0, Address = 0;
uint32_t SectorError = 0;
__IO uint32_t MemoryProgramStatus = 0;
int32_t tempbuf[ParaNum];

static FLASH_EraseInitTypeDef EraseInitStruct;


static uint32_t GetSector(uint32_t Address);


u8 InternalFlash(int32_t *data,u32 data_len)
{
    u32 i,temp32;
		
    static u32 tempi = 0;
    /* ����,ɾ����д������Ƚ��� */
    HAL_FLASH_Unlock();

    /* ��ȡҪ�������׸����� */
    FirstSector = GetSector(FLASH_USER_START_ADDR);
    /* ��ȡҪ������������Ŀ */
    NbOfSectors = (data_len*4>FLASH_USER_END_ADDR-FLASH_USER_START_ADDR)?2:1; 
    /* ��ʼ�������ṹ�� */
    EraseInitStruct.TypeErase = FLASH_TYPEERASE_SECTORS;//��������
    EraseInitStruct.VoltageRange = FLASH_VOLTAGE_RANGE_3;
    EraseInitStruct.Sector = FirstSector;
    EraseInitStruct.NbSectors = NbOfSectors;
		__HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_EOP | FLASH_FLAG_OPERR | FLASH_FLAG_WRPERR |FLASH_FLAG_PGAERR | FLASH_FLAG_PGPERR|FLASH_FLAG_PGSERR);
    if(HAL_FLASHEx_Erase(&EraseInitStruct, &SectorError) != HAL_OK)
    {
        /* �˴�����ʹ��HAL_FLASH_GetError()����ȡ��������������� */
        tempi =0;
        return FALSE;
    }
    /* ����wordд�����ݵ��ڲ�flash */
    for(i=0; i<data_len; i++)
    {
        if(PARAMETER[i].stAttributes.bSaveToFlash==YSFLASH)
        {
            tempbuf[tempi] = data[i];
            tempi++;
        }
    }
    if (WriteData_toFlash(tempbuf,tempi,FLASH_USER_START_ADDR)==FALSE)
    {
        tempi =0;
        return FALSE;
    }

    /* ����������Ƕ�ȡ�Ļ���������� */
    HAL_FLASH_Lock();
    Address = FLASH_USER_START_ADDR;
    MemoryProgramStatus = 0;
    /* ��ȡ������֤�Ƿ��д���������ͬ�������ͬ����ôMemoryProgramStatus=0 */

    for(i=0; i<tempi; i++)
    {
        temp32 = *(__IO uint32_t*)(Address+i*4);
        if(temp32 != tempbuf[i])
        {
            MemoryProgramStatus++;
        }
    }
    tempi =0;
    if (MemoryProgramStatus)
    {
        return FALSE;
    }
    else
    {
        return TRUE;
    }

}

/**
  * ��������: ��������ĵ�ַ���������ڵ�sector
  * �������: Address flash��ַ
  * �� �� ֵ: ��
  * ˵    ��: ��
  */
static uint32_t GetSector(uint32_t Address)
{
    uint32_t sector = 0;

    if((Address < ADDR_FLASH_SECTOR_1) && (Address >= ADDR_FLASH_SECTOR_0))
    {
        sector = FLASH_SECTOR_0;
    }
    else if((Address < ADDR_FLASH_SECTOR_2) && (Address >= ADDR_FLASH_SECTOR_1))
    {
        sector = FLASH_SECTOR_1;
    }
    else if((Address < ADDR_FLASH_SECTOR_3) && (Address >= ADDR_FLASH_SECTOR_2))
    {
        sector = FLASH_SECTOR_2;
    }
    else if((Address < ADDR_FLASH_SECTOR_4) && (Address >= ADDR_FLASH_SECTOR_3))
    {
        sector = FLASH_SECTOR_3;
    }
    else if((Address < ADDR_FLASH_SECTOR_5) && (Address >= ADDR_FLASH_SECTOR_4))
    {
        sector = FLASH_SECTOR_4;
    }
    else if((Address < ADDR_FLASH_SECTOR_6) && (Address >= ADDR_FLASH_SECTOR_5))
    {
        sector = FLASH_SECTOR_5;
    }
    else if((Address < ADDR_FLASH_SECTOR_7) && (Address >= ADDR_FLASH_SECTOR_6))
    {
        sector = FLASH_SECTOR_6;
    }
    else if((Address < ADDR_FLASH_SECTOR_8) && (Address >= ADDR_FLASH_SECTOR_7))
    {
        sector = FLASH_SECTOR_7;
    }
    else if((Address < ADDR_FLASH_SECTOR_9) && (Address >= ADDR_FLASH_SECTOR_8))
    {
        sector = FLASH_SECTOR_8;
    }
    else if((Address < ADDR_FLASH_SECTOR_10) && (Address >= ADDR_FLASH_SECTOR_9))
    {
        sector = FLASH_SECTOR_9;
    }
    else if((Address < ADDR_FLASH_SECTOR_11) && (Address >= ADDR_FLASH_SECTOR_10))
    {
        sector = FLASH_SECTOR_10;
    }
    else /* (Address < FLASH_END_ADDR) && (Address >= ADDR_FLASH_SECTOR_11) */
    {
        sector = FLASH_SECTOR_11;
    }

    return sector;
}


uint8_t Erase_Flash(uint32_t sector_start,uint32_t sector_end)
{

    HAL_FLASH_Unlock();
    EraseInitStruct.TypeErase = FLASH_TYPEERASE_SECTORS;
    EraseInitStruct.VoltageRange = FLASH_VOLTAGE_RANGE_3;
    EraseInitStruct.Sector = sector_start;
    EraseInitStruct.NbSectors = sector_end;
    if(HAL_FLASHEx_Erase(&EraseInitStruct, &SectorError) != HAL_OK)
    {
        /* �˴�����ʹ��HAL_FLASH_GetError()����ȡ��������������� */
        while (1)
        {
        }
    }
    HAL_FLASH_Lock();
    return 0;
}
uint8_t WriteData_toFlash(int32_t *data,uint32_t len,uint32_t address)
{
    uint32_t i=0;
    for(i=0; i<len; i++)
    {
        if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, (uint32_t)address, *data) == HAL_OK)
        {
            address = address+4;
            data = data+1;
        }
        else
        {
            return FALSE;
        }
    }
    return TRUE;
}

void Flash_Read(u16 bufsize)			//��ȡflash����
{
    u16 i;
    u32 dataddress = FLASH_USER_START_ADDR;
    for(i=0; i<bufsize; i++)
    {
        if(PARAMETER[i].stAttributes.bSaveToFlash ==YSFLASH)
				{
					*PARAMETER[i].lpParam = *(long*)(dataddress);
					dataddress +=4;
				}
		}
}