#include "canopen_objacces.h"


/*------------------------------------------------              
Function:OBJ access err record  
Input   :Index SubIndex DataSize DataSizeGiven errcode                    
Output  :Result                     
Explain :No                     
------------------------------------------------*/
unsigned char OBJ_Access_Err_Record(unsigned short int Index,unsigned char SubIndex,unsigned int DataSize,unsigned int DataSizeGiven,unsigned int errcode)
{
  ATLAS_ERR("Index : %d\r\n",Index);
  ATLAS_ERR("SubIndex : %d\r\n",SubIndex);
  
  switch(errcode)
  {
    case  OD_NO_SUCH_OBJECT:
      ATLAS_ERR("Index not found ��%d\r\n", Index);
      break;
    case OD_NO_SUCH_SUBINDEX :
      ATLAS_ERR("SubIndex not found ��%d\r\n", SubIndex);
      break;
    case OD_WRITE_NOT_ALLOWED :
      ATLAS_ERR("Write not allowed, data is read only ��%d\r\n", Index);
      break;
    case OD_LENGTH_DATA_INVALID :
      ATLAS_ERR("Conflict size data. Should be (bytes)  : %d\r\n", DataSize);
      ATLAS_ERR("But you have given the size  : %d\r\n", DataSizeGiven);
      break;
    case OD_NOT_MAPPABLE :
      ATLAS_ERR("Not mappable data in a PDO at index  : %d\r\n", Index);
      break;
    case OD_VALUE_TOO_LOW :
      ATLAS_ERR("Value range error : value too low. SDOabort ��%d\r\n", errcode);
      break;
    case OD_VALUE_TOO_HIGH :
      ATLAS_ERR("Value range error : value too high. SDOabort ��%d\r\n", errcode);
      break;
    default :
      ATLAS_ERR("Unknown error code  ��%d\r\n", errcode);
      break;
  }
  
  return 0;
}
/*------------------------------------------------
Function:ͨ���ֵ������������������ݿ�������                                             
Input   :d Index Subindex  Aim_data_chache	 Aim_data_chache_len	dataType	if_check_	
Output  :Result                                                                             
Explain :�����ݿ�����������  �������ݳ���  �������ͷ��ظ�ָ��ָ���ʵ��                 
------------------------------------------------*/
unsigned int _getODentry(CO_Data *d, 
                         unsigned short int wIndex,\
                         unsigned char bSubindex,\
                         void *pDestData,\
                         unsigned int *pExpectedSize,\
                         unsigned char *pDataType,\
                         unsigned char checkAccess,\
                         unsigned char endianize)
{
  unsigned int errorCode;
  unsigned int szData;
  const indextable *ptrTable;
  ODCallback_t *Callback;
  
  ptrTable = (*d->OD_ScanIndex)(wIndex, &errorCode, &Callback);
  if(errorCode != OD_SUCCESSFUL)
    return errorCode;

  if(ptrTable->bSubcount <= bSubindex) 
	{
//    OBJ_Access_Err_Record(wIndex, bSubindex, 0, 0, OD_NO_SUCH_SUBINDEX);
    return OD_NO_SUCH_SUBINDEX;
  }
  
  if(checkAccess && (ptrTable->pSubindex[bSubindex].accessType & WO))
  {
//    OBJ_Access_Err_Record(wIndex, bSubindex, 0, 0, OD_READ_NOT_ALLOWED);
    return OD_READ_NOT_ALLOWED;
  }
  if(pDestData == 0) 
	{
    return SDOABT_GENERAL_ERROR;
  }

  if(RS232_OD_SIGN == 1)
  {
    *pExpectedSize = ptrTable->pSubindex[bSubindex].size;
  }

  if(ptrTable->pSubindex[bSubindex].size > *pExpectedSize) 
	{
    *pExpectedSize = ptrTable->pSubindex[bSubindex].size;
    return SDOABT_OUT_OF_MEMORY;
  }
  /* �������������� ���ݸ� Ŀ�Ļ������������� */
  *pDataType = ptrTable->pSubindex[bSubindex].dataType;
  /* ���ݴ�С */
  szData = ptrTable->pSubindex[bSubindex].size;
  
  /* ������ǿ����ַ��� */
  if(*pDataType != UNSIGNED8_STRING) 
	{
    memcpy(pDestData, ptrTable->pSubindex[bSubindex].lpParam, szData);
		/* ���ݴ�С */
    *pExpectedSize = szData;
  }
  /* �����ַ��� */
  else
  {
    /* �ַ���ָ�� */
    unsigned char *ptr = (unsigned char*)ptrTable->pSubindex[bSubindex].lpParam;
    /* ��ʼָ�� */
		unsigned char *ptr_start = ptr;
		/* ����ָ�� */
		unsigned char *ptr_end = ptr + (*pExpectedSize ? *pExpectedSize : szData);
		/* Ŀ��ָ�� */
		unsigned char *ptr_dest = (unsigned char*)pDestData;
		/* �������� */
		while(*ptr && ptr < ptr_end)
		{
			*(ptr_dest++) = *(ptr++);
		}
		/* �����ֽ��� */
    *pExpectedSize = (unsigned int)(ptr - ptr_start);
		/* ���û�и����꣬���һ���ֽ���Ϊ'/0' */
    if(*pExpectedSize < szData)
			*(ptr) = 0;
  }
  return OD_SUCCESSFUL;
}
/*------------------------------------------------
Function:ͨ���ֵ������������������ݿ�����ȥ                                             
Input   :CO_Data *d  ��ַ ������  Ŀ�Ļ�����	 Ŀ�Ļ���������		��������	�Ƿ�������	
Output  :No                                                                             
Explain :�����ݿ�����������  �������ݳ���  �������ͷ��ظ�ָ��ָ���ʵ��                 
------------------------------------------------*/
unsigned int _setODentry(CO_Data *d,\
                         unsigned short int wIndex,\
                         unsigned char bSubindex,\
                         void *pSourceData,\
                         unsigned int *pExpectedSize,\
                         unsigned char checkAccess,\
                         unsigned char endianize)
{
  unsigned int errorCode;
  unsigned char dataType;
  unsigned int szData;
  const indextable *ptrTable;
  ODCallback_t *Callback;
  
  ptrTable =(*d->OD_ScanIndex)(wIndex, &errorCode, &Callback);
  if(errorCode != OD_SUCCESSFUL)
    return errorCode;

  if(ptrTable->bSubcount <= bSubindex) 
	{
    return OD_NO_SUCH_SUBINDEX;
  }
  /* �����Ҫ������ԣ����Ҹ���������Ŀ������Ϊֻд���򱨴� */
  if(checkAccess && (ptrTable->pSubindex[bSubindex].accessType & (RO|CONST)))
  {
    return OD_WRITE_NOT_ALLOWED;
  }
  
  /* �������� */
  dataType = ptrTable->pSubindex[bSubindex].dataType;
	/* ���ݴ�С */
  szData = ptrTable->pSubindex[bSubindex].size;

  if(RS232_OD_SIGN == 1)
  {
    RS232_OD_SIGN = 0;
    *pExpectedSize = szData;
  }
  if(*pExpectedSize == 0 || *pExpectedSize == szData ||\
     (dataType == UNSIGNED8_STRING && *pExpectedSize < szData))
  {

    errorCode = (*d->valueRangeTest)(dataType, pSourceData);
    if(errorCode) 
		{
			return errorCode;
		}
    d->LastObj.Index = wIndex;
    d->LastObj.SubIndex = bSubindex;
    d->LastObj.size = szData;
    d->LastObj.dataType = dataType;
    memcpy(d->LastObj.Data, ptrTable->pSubindex[bSubindex].lpParam, szData);
		memcpy(ptrTable->pSubindex[bSubindex].lpParam, pSourceData, *pExpectedSize);
		if(dataType == UNSIGNED8_STRING && *pExpectedSize < szData)
			((unsigned char*)ptrTable->pSubindex[bSubindex].lpParam)[*pExpectedSize] = 0;
    
    /* ȡ���ֽ��� */
		*pExpectedSize = szData;
    
		if(Callback && Callback[bSubindex])
		{
			errorCode = (Callback[bSubindex])(d, ptrTable, bSubindex);
			if(errorCode != OD_SUCCESSFUL)
			{
				return errorCode;
			}
		}
    
    /* ��������ΪTO_BE_SAVE�����ûص�����(����֪�����Ǹ����õ�) */
		if(ptrTable->pSubindex[bSubindex].accessType & TO_BE_SAVE)
		{
			(*d->storeODSubIndex)(d, wIndex, bSubindex);
		}
		return OD_SUCCESSFUL;
  }
  else
  {
    *pExpectedSize = szData;

		return OD_LENGTH_DATA_INVALID;
  }
}
/*------------------------------------------------
Function:���ֵ䣬����ȡ��������ΪwIndex����Ŀ�Ļص�����  
Input   :d  Index *errode **callback
Output  :Setting result                  
Explain :No                              
------------------------------------------------*/
const indextable *OD_Index_san(CO_Data *d, unsigned short int wIndex, unsigned int *errorCode, ODCallback_t **Callback)
{
  return (*d->OD_ScanIndex)(wIndex, errorCode, Callback);
}
/*------------------------------------------------
Function:�������ֵ����ע���µĻص�����  
Input   :Index SubIndex ״̬�� �����±�  
Output  :Setting result                  
Explain :No                              
------------------------------------------------*/
unsigned int RegisterSetOnDentry_Callback(CO_Data *d, unsigned short int wIndex, unsigned char bSubindex, ODCallback_t Callback)
{
  unsigned int errorCode;
	ODCallback_t *CallbackList;
	const indextable *odentry;

	/* �����ֵ����� */
  odentry = OD_Index_san(d, wIndex, &errorCode, &CallbackList);
  if(errorCode == OD_SUCCESSFUL && CallbackList && bSubindex < odentry->bSubcount) 
    CallbackList[bSubindex] = Callback;
	
  return errorCode;
}
