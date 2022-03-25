#include "canopen_sdo.h"

#define SDO_SERVICE

#ifdef SDO_SERVICE
/*------------------------------------------------
Function:SDO���䳬ʱ�ص�����
Input   :No
Output  :No
Explain :No
------------------------------------------------*/
void SDOTimeoutAlarm(CO_Data *d, unsigned int id)
{
  unsigned short int offset;
	unsigned char nodeId;
	
	offset = d->firstIndex->SDO_CLT;
	if((offset == 0) || ((offset + d->transfers[id].CliServNbr) > d->lastIndex->SDO_CLT)) 
	{
		return;
	}
	

	nodeId = (unsigned char)*((unsigned int*)d->objdict[offset + d->transfers[id].CliServNbr].pSubindex[3].lpParam);
	
//  ATLAS_ERR("SDO timeout. SDO response not received");
//  ATLAS_ERR("server node id : %d", nodeId);
//  ATLAS_ERR("         index : %d", d->transfers[id].index);
//  ATLAS_ERR("      subIndex : %d", d->transfers[id].subIndex);
	
	d->transfers[id].timer = TIMER_NONE;
	d->transfers[id].state = SDO_ABORTED_INTERNAL;

	SDO_Err_Record(d, d->transfers[id].whoami, d->transfers[id].CliServNbr, d->transfers[id].index, d->transfers[id].subIndex, SDOABT_TIMED_OUT);
	d->transfers[id].abortCode = SDOABT_TIMED_OUT;
	
	if(d->transfers[id].Callback)
		(*d->transfers[id].Callback)(d, nodeId);

	if(d->transfers[id].abortCode == SDOABT_TIMED_OUT) 
  {
    resetSDOline(d, (unsigned char)id);
  }

}

#define StopSDO_TIMER(id) \
d->transfers[id].timer = DelAlarm(d->transfers[id].timer);

#define StartSDO_TIMER(id) \
	d->transfers[id].timer = SetAlarm(d, id, &SDOTimeoutAlarm,US_TO_TIMEVAL(SDO_TIMEOUT_MS),0);

#define RestartSDO_TIMER(id) \
if(d->transfers[id].timer != TIMER_NONE) { StopSDO_TIMER(id) StartSDO_TIMER(id) }

/*------------------------------------------------
Function:��λ����sdo����ͨ��
Input   :No
Output  :No
Explain :No
------------------------------------------------*/
void resetSDO(CO_Data *d)
{
	unsigned char j;

	for(j = 0; j < SDO_MAX_SIMULTANEOUS_TRANSFERS; j++)
	{
		resetSDOline(d, j);
	}
}
/*------------------------------------------------
Function:�����ݴӴ���ͨ���������������ֵ���
Input   :No
Output  :No
Explain :No
------------------------------------------------*/
unsigned int SDOlineToObjdict(CO_Data *d, unsigned char line)
{
	unsigned int size;
	unsigned int errorCode;
	
	//ATLAS_PRINT("Enter in SDOlineToObjdict : %d\r\n",line);
	
	/* ȷ���ô���ͨ�������˶����ֽ� */
	if(d->transfers[line].count == 0)
	{
		d->transfers[line].count = d->transfers[line].offset;
	}
	size = d->transfers[line].count;

	errorCode = setODentry(d, d->transfers[line].index, d->transfers[line].subIndex,
												 (void *)d->transfers[line].data, &size, 1);
	if(errorCode != OD_SUCCESSFUL)
		return errorCode;

	//ATLAS_PRINT("exit of SDOlineToObjdict : %d\r\n",line);

	return 0;

}


/*------------------------------------------------
Function:�����ݴ��ֵ��п���������ͨ��������
Input   :No
Output  :No
Explain :No
------------------------------------------------*/
unsigned int objdictToSDOline(CO_Data *d, unsigned char line)
{
	unsigned int size = SDO_MAX_LENGTH_TRANSFER;
	unsigned char dataType;
	unsigned int errorCode;

  //ATELAS_MAR("obj to trans chache",d->transfers[line].index,d->transfers[line].subIndex);

	errorCode = getODentry(d, d->transfers[line].index, d->transfers[line].subIndex, 
												 (void *)d->transfers[line].data, &size, &dataType, 1);
  
	if(errorCode != OD_SUCCESSFUL)
		return errorCode;

	d->transfers[line].count = size;
	d->transfers[line].offset = 0;

	return 0;
}

/*------------------------------------------------
Function:�����ݴӴ���ͨ����������������
Input   :No
Output  :No
Explain :No
------------------------------------------------*/
unsigned char lineToSDO(CO_Data *d, unsigned char line, unsigned int nbBytes, unsigned char *data) 
{
	unsigned char i;
	unsigned int offset;

	if((d->transfers[line].offset + nbBytes) > d->transfers[line].count) 
	{
//		ATLAS_ERR("SDO Size of data too large. Exceed count : %d\r\n", nbBytes);
		return 0xFF;
	}
	offset = d->transfers[line].offset;

	for(i = 0; i < nbBytes; i++)
		*(data + i) = d->transfers[line].data[offset + i];

	/* ƫ�������� */
	d->transfers[line].offset = d->transfers[line].offset + nbBytes;
	
	return 0;
}
/*------------------------------------------------
Function:�����ݿ���������ͨ��������
Input   :No
Output  :No
Explain :No
------------------------------------------------*/
unsigned char SDOtoLine(CO_Data *d, unsigned char line, unsigned int nbBytes, unsigned char *data)
{
	unsigned char i;
	unsigned int offset;

	/* ������ƫ���� */
	offset = d->transfers[line].offset;
	
	for (i = 0; i < nbBytes; i++)
		d->transfers[line].data[offset + i] = *(data + i);

	d->transfers[line].offset = d->transfers[line].offset + nbBytes;
	
	return 0;
}
/*------------------------------------------------
Function:��λsdoͨ��
Input   :No
Output  :No
Explain :No
------------------------------------------------*/
void resetSDOline(CO_Data *d, unsigned char line)
{
	unsigned int i;
	
	//ATLAS_PRINT("reset SDO line nb : %d\r\n",line);
	
	initSDOline(d, line, 0, 0, 0, SDO_RESET);
	
	for(i = 0; i < SDO_MAX_LENGTH_TRANSFER; i++)
		d->transfers[line].data[i] = 0;
	
	d->transfers[line].whoami = 0;
	
	d->transfers[line].abortCode = 0;
}

/*------------------------------------------------
Function:sdo����ͨ��ʧ��
Input   :No
Output  :No
Explain :No
------------------------------------------------*/
unsigned char failedSDO(CO_Data *d, unsigned char CliServNbr, unsigned char whoami, unsigned short int index, unsigned char subIndex, unsigned int abortCode)
{
	unsigned char err;
	unsigned char line;
	
	err = getSDOlineOnUse(d, CliServNbr, whoami, &line);
	if(!err)
	{
		//ATLAS_PRINT("FailedSDO : line found : %d\r\n", line);
	}
	
	if((!err) && (whoami == SDO_SERVER)) 
	{
		resetSDOline(d, line);
		//ATLAS_PRINT("FailedSDO : line released : %d\r\n", line);
	}
	if((!err) && (whoami == SDO_CLIENT)) 
	{
		StopSDO_TIMER(line);
		d->transfers[line].state = SDO_ABORTED_INTERNAL;
		d->transfers[line].abortCode = abortCode;
	}
	
	//ATLAS_PRINT("Sending SDO abort ");
	
	/* ����sdo��ֹ���� */
	err = SDO_Err_Record(d, whoami, CliServNbr, index, subIndex, abortCode);
	if(err) 
	{
//		ATLAS_ERR("Unable to send the SDO abort\r\n");
		return 0xFF;
	}

	return 0;
}
/*------------------------------------------------
Function:��ʼ��sdo����ͨ��
Input   :No
Output  :No
Explain :No
------------------------------------------------*/
unsigned char initSDOline(CO_Data *d, unsigned char line, unsigned char CliServNbr, unsigned short int index, unsigned char subIndex, unsigned char state)
{
	//ATLAS_PRINT("init SDO line nb : %d\r\n",line);
	
	if(state == SDO_DOWNLOAD_IN_PROGRESS || state == SDO_UPLOAD_IN_PROGRESS || 
		 state == SDO_BLOCK_DOWNLOAD_IN_PROGRESS || state == SDO_BLOCK_UPLOAD_IN_PROGRESS)
	{
		StartSDO_TIMER(line)
	}
	else
	{
		StopSDO_TIMER(line)
	}


	d->transfers[line].CliServNbr = CliServNbr;

	d->transfers[line].index = index;

	d->transfers[line].subIndex = subIndex;

	d->transfers[line].state = state;
	d->transfers[line].toggle = 0;

	d->transfers[line].count = 0;
	d->transfers[line].offset = 0;
	d->transfers[line].peerCRCsupport = 0;
	d->transfers[line].blksize = 0;
	d->transfers[line].ackseq = 0;
	d->transfers[line].objsize = 0;
	d->transfers[line].lastblockoffset = 0;
	d->transfers[line].seqno = 0;
	d->transfers[line].endfield = 0;
	/* ��ʼ���鴫�����״̬ */
	d->transfers[line].rxstep = RXSTEP_INIT;
	d->transfers[line].dataType = 0;

	d->transfers[line].Callback = NULL;

	return 0;
}
/*------------------------------------------------
Function:��ȡ���е�sdo����ͨ��
Input   :No
Output  :No
Explain :No
------------------------------------------------*/
unsigned char getSDOfreeLine(CO_Data *d, unsigned char whoami, unsigned char *line)
{
	unsigned char i;


	for(i = 0; i < SDO_MAX_SIMULTANEOUS_TRANSFERS; i++)
	{
		if(d->transfers[i].state == SDO_RESET) 
		{
			*line = i;
			d->transfers[i].whoami = whoami;
			return 0;
		}
	}
	
	//ATLAS_PRINT("Too many SDO in progress. Aborted. %d\r\n", i);

	return 0xFF;
}
/*------------------------------------------------
Function:ͨ���ͻ���/�������Ż�ȡsdo����ͨ����
Input   :No
Output  :No
Explain :No
------------------------------------------------*/
unsigned char getSDOlineOnUse(CO_Data *d, unsigned char CliServNbr, unsigned char whoami, unsigned char *line)
{
	unsigned char i;

	/* ��������sdo����ͨ�� */
	for(i = 0; i < SDO_MAX_SIMULTANEOUS_TRANSFERS; i++)
	{
		if((d->transfers[i].state != SDO_RESET) &&
			 (d->transfers[i].state != SDO_ABORTED_INTERNAL) &&
			 (d->transfers[i].CliServNbr == CliServNbr) &&
			 (d->transfers[i].whoami == whoami)) 
		{
			if(line)
			{				
				*line = i;
			}
			
			return 0;
		}
	}

	return 0xFF;
}
/*------------------------------------------------
Function:ͨ���ͻ���/�������Ż�ȡsdo����ͨ����
Input   :No
Output  :No
Explain :No
------------------------------------------------*/
unsigned char getSDOlineToClose(CO_Data *d, unsigned char CliServNbr, unsigned char whoami, unsigned char *line)
{
	unsigned char i;

	for(i = 0; i < SDO_MAX_SIMULTANEOUS_TRANSFERS; i++)
	{
		if((d->transfers[i].state != SDO_RESET) &&
			 (d->transfers[i].CliServNbr == CliServNbr) &&
			 (d->transfers[i].whoami == whoami)) 
		{
			if(line) 
				*line = i;
			
			return 0;
		}
	}
	
	return 0xFF;
}
/*------------------------------------------------
Function:�ر�sdo����ͨ��
Input   :No
Output  :No
Explain :No
------------------------------------------------*/
unsigned char closeSDOtransfer(CO_Data *d, unsigned char nodeId, unsigned char whoami)
{
//	unsigned char err;
//	unsigned char line;
//	unsigned char CliNbr;
//	
//	/* ͨ���ڵ�Ų��ҿͻ��˺� */
//	CliNbr = GetSDOClientFromNodeId(d, nodeId);
//	if(CliNbr >= 0xFE)
//		return SDO_ABORTED_INTERNAL;
//	
//	/* ͨ���ͻ���/�������Ż�ȡsdo����ͨ���� */
//	err = getSDOlineToClose(d, CliNbr, whoami, &line);
//	if(err) 
//	{
//		MSG_WAR(0x2A30, "No SDO communication to close", 0);
//		return 0xFF;
//	}

//	/* ��λsdoͨ�� */
//	resetSDOline(d, line);
	
	return 0;
}
/*------------------------------------------------
Function:��ȡSDO����ͨ��ʣ���ֽ���
Input   :No
Output  :No
Explain :No
------------------------------------------------*/
unsigned char getSDOlineRestBytes(CO_Data *d, unsigned char line, unsigned int *nbBytes)
{
	if(d->transfers[line].count == 0)
		*nbBytes = 0;
	else
		*nbBytes = d->transfers[line].count - d->transfers[line].offset;
	return 0;
}
/*------------------------------------------------
Function:����sdo����ͨ��ʣ���ֽ���
Input   :No
Output  :No
Explain :No
------------------------------------------------*/
unsigned char setSDOlineRestBytes(CO_Data *d, unsigned char line, unsigned int nbBytes)
{
	d->transfers[line].count = nbBytes;

	return 0;
}

#endif
/*------------------------------------------------
Function:����SDO����
Input   :No
Output  :No
Explain :No
------------------------------------------------*/
unsigned char SendSDO(CO_Data *d,unsigned char whoami,unsigned char CliServNbr,unsigned char *data)
{
  unsigned short int offset;
  CanTxMsgTypeDef m;
  unsigned char i;
  
  //ATELAS_MAR("SendSDO",0,0);
  
  if(!((d->Node_state == HEART_STAT_PRE_OPERAT)||(d->Node_state == HEART_STAT_OPERAT)))
  {
    //ATELAS_MAR("unable to send the SDO (not in op or pre-op mode",0,0);
    return 0xFF;
  }
  
  /* ����������SDO */
  if(whoami == SDO_SERVER)
  {
    offset = d->firstIndex->SDO_SVR;
    if((offset == 0)||((offset+CliServNbr) > d->lastIndex->SDO_SVR))//sdo not exist
    {
      //ATELAS_MAR("SendSDO : SDO server not found",0,0);
      return 0xFF;
    }
    m.StdId = (unsigned short int)*((unsigned int *)d->objdict[offset + CliServNbr].pSubindex[2].lpParam);
    //ATELAS_MAR("I am server Tx cobId :",m.StdId,0);
  }
  for(i=0;i<8;i++)
  {
    m.Data[i] = data[i];
  }
  m.RTR = RTR_INVALID;
  m.DLC = 8;

  return CAN1_Send_Msg(m.StdId,CAN_ID_STD,m.RTR,data,m.DLC);
}

/*------------------------------------------------
Function:SDO��ֹ�뱨��
Input   :No
Output  :No
Explain :No
------------------------------------------------*/
unsigned char SDO_Err_Record(CO_Data *d,unsigned char whoami,unsigned char CliServNbr,unsigned int Index,unsigned char subIndex,unsigned int EndCode)
{
  unsigned char data[8];
  unsigned char ret;
  
  data[0] = (ABORT_TRANSER<<5)&0x80;
  data[1] = (Index>>0)&0xFF;
	data[2] = (Index>>8)&0xFF;
	data[3] = subIndex;
	data[4] = (unsigned char)((EndCode>>0)&0xFF);
	data[5] = (unsigned char)((EndCode>>8)&0xFF);
	data[6] = (unsigned char)((EndCode>>16)&0xFF);
	data[7] = (unsigned char)((EndCode>>24)&0xFF);
  
  /* Send SDO */
  ret = SendSDO(d,whoami,CliServNbr,data);
  
  return ret;
}


/*------------------------------------------------
Function:SDO Message handler
Input   :*CO_Data  *m
Output  :No
Explain :No
------------------------------------------------*/
unsigned char SDO_Email_Handler(CO_Data *d,CanRxMsgTypeDef *m)
{
  unsigned char err;
	unsigned char cs;
	unsigned char line;
	unsigned int nbBytes;
	unsigned char nodeId = 0;
	unsigned char CliServNbr;
	unsigned char whoami = SDO_UNKNOWN;
	unsigned int errorCode;
	unsigned char data[8];
	unsigned short int index;
	unsigned char subIndex;
	unsigned int i;
	unsigned char	j;
	unsigned int *pCobId = NULL;
	unsigned short int offset;
	unsigned short int lastIndex;
  unsigned int TimeCheck;
  
  //ATLAS_PRINT("SDO Handle Begin\r\n");
  
  whoami = SDO_UNKNOWN;

	offset = d->firstIndex->SDO_SVR;

	lastIndex = d->lastIndex->SDO_SVR;
	
	j = 0;
  
  TimeCheck = TIM6->CNT;

	if(offset)
  {
		while(offset <= lastIndex)
    {

			if(d->objdict[offset].bSubcount <= 1) 
			{
//				ATLAS_ERR("Subindex 1  not found at index : %d\r\n", 0x1200 + j);
				return 0xFF;
			}

			pCobId = (unsigned int*)d->objdict[offset].pSubindex[1].lpParam;

			if(((*pCobId)&CAN_ID_BIT) == ((UNS16_LE(m->StdId))&CAN_ID_BIT)) 
			{

				whoami = SDO_SERVER;
				//ATLAS_PRINT("SDO_Email_Handler. I am server. index : %d\r\n", 0x1200 + j);

				CliServNbr = j;
				break;
			}
			j++;
			offset++;
    }
  }
	if(whoami == SDO_UNKNOWN) 
	{
		offset = d->firstIndex->SDO_CLT;
		lastIndex = d->lastIndex->SDO_CLT;
		
		j = 0;
		
		/* ��������˿ͻ��� */
		if(offset) 
		{
			/* ��������sdo�ͻ��� */
			while(offset <= lastIndex) 
			{
				if(d->objdict[offset].bSubcount <= 3) 
				{
//				ATLAS_ERR("Subindex 3  not found at index : %d\r\n", 0x1280 + j);
					return 0xFF;
				}
				pCobId = (unsigned int*)d->objdict[offset].pSubindex[2].lpParam;
				if(((*pCobId)&CAN_ID_BIT) == ((UNS16_LE(m->StdId))&CAN_ID_BIT))
				{
					whoami = SDO_CLIENT;
					
					//ATLAS_PRINT("proceedSDO. I am client index : %d\r\n", 0x1280 + j);
					
					/* ������/�ͻ��˺� */
					CliServNbr = j;
					
					/* �������˵Ľڵ�� */
					nodeId = *((unsigned char*) d->objdict[offset].pSubindex[3].lpParam);
					break;
				}
				j++;
				offset++;
			}
		}
    
    if(whoami == SDO_UNKNOWN) 
    {
      return 0xFF;
    }
	}
  
	if(whoami == SDO_UNKNOWN) 
	{
		return 0xFF;
	}

	/* SDO����һ����8�ֽ� */
	if((*m).DLC != 8) 
	{
		ATLAS_ERR("Error size SDO\r\n");
		failedSDO(d, CliServNbr, whoami, 0, 0, SDOABT_GENERAL_ERROR);
		return 0xFF;
	}
  
  if(whoami == SDO_CLIENT) 
	{
		//ATLAS_PRINT("I am CLIENT number :%d\r\n", CliServNbr);
	}
	else 
	{
		//ATLAS_PRINT("I am SERVER number :%d\r\n", CliServNbr);
	}
  
  /* ͨ������/�ͻ��˺Ż�ȡͨ���� */
	err = getSDOlineOnUse(d, CliServNbr, whoami, &line);
  
  cs = 0xFF; 
  if(!err) 
	{
		if(((whoami == SDO_SERVER) && (d->transfers[line].state == SDO_BLOCK_DOWNLOAD_IN_PROGRESS)) ||
			 ((whoami == SDO_CLIENT) && (d->transfers[line].state == SDO_BLOCK_UPLOAD_IN_PROGRESS))) 
		{
			if(m->Data[0] == 0x80)
				cs = 4;
			else
				cs = 6;
		}
	}
  if(cs == 0xFF)
	{
		cs = getSDOcs(m->Data[0]);
	}
  
  /* SDO cs handle */
  switch(cs)
  {
    case SEGMENT_DOWNLOAD_REQUEST:
    {
      if(whoami == SDO_SERVER) 
			{
				if(!err)
				{
					err = d->transfers[line].state != SDO_DOWNLOAD_IN_PROGRESS;
				}
				if(err) 
				{
//					ATLAS_ERR("SDO error : Received download segment for unstarted trans. index 0x1200 + %d\r\n", CliServNbr);
					failedSDO(d, CliServNbr, whoami, 0, 0, SDOABT_LOCAL_CTRL_ERROR);
					return 0xFF;
				}
				/* ��������ͨ����ʱ��ʱ�� */
				RestartSDO_TIMER(line)
				
				//ATLAS_PRINT("Received SDO download segment defined at index 0x1200 + %d\r\n", CliServNbr);
				
				index = d->transfers[line].index;
				subIndex = d->transfers[line].subIndex;
				
				if(d->transfers[line].toggle != getSDOt(m->Data[0])) 
				{
//					ATLAS_ERR("SDO error : Toggle error : %d\r\n", getSDOt(m->Data[0]));
					failedSDO(d, CliServNbr, whoami, index, subIndex, SDOABT_TOGGLE_NOT_ALTERNED);
					return 0xFF;
				}
				
				/* ȡ���ֽ��� */
				nbBytes = 7 - getSDOn3(m->Data[0]);
				err = SDOtoLine(d, line, nbBytes, (*m).Data + 1);
				if(err)
				{
					failedSDO(d, CliServNbr, whoami, index, subIndex, SDOABT_GENERAL_ERROR);
					return 0xFF;
				}
				
				/* ������Ӧ�� */
				data[0] = (1 << 5) | (d->transfers[line].toggle << 4);
				for(i = 1 ; i < 8 ; i++)
				{
					data[i] = 0;
				}
				//ATLAS_PRINT("SDO. Send response to download request defined at index 0x1200 + %d\r\n", CliServNbr);
				
				/* ������Ӧ�� */
				SendSDO(d, whoami, CliServNbr, data);
				
				d->transfers[line].toggle = !d->transfers[line].toggle & 1;
				if(getSDOc(m->Data[0])) 
				{
					errorCode = SDOlineToObjdict(d, line);
					if(errorCode) 
					{
//						ATLAS_ERR("SDO error : Unable to copy the data in the object dictionary ");
						failedSDO(d, CliServNbr, whoami, index, subIndex, errorCode);
						return 0xFF;
					}
					resetSDOline(d, line);
					//ATLAS_PRINT("SDO. End of download defined at index 0x1200 + %d\r\n", CliServNbr);
				}
			}
    }break;
    case INIT_DOWNLOAD_REQUEST://183us
    {
      if(whoami == SDO_SERVER) 
			{
				/* ���� */
				index = getSDOindex(m->Data[1], m->Data[2]);
				/* ������ */
				subIndex = getSDOsubIndex(m->Data[3]);
				//ATLAS_PRINT("Received SDO Initiate Download (to store data) defined at index 0x1200 + %d\r\n", CliServNbr);
				//ATLAS_PRINT("Writing at index : %d\r\n", index);
				//ATLAS_PRINT("Writing at subIndex : %d\r\n", subIndex);
				if(!err) 
				{
//					ATLAS_ERR("SDO error : Transmission yet started.");
					failedSDO(d, CliServNbr, whoami, index, subIndex, SDOABT_LOCAL_CTRL_ERROR);
					return 0xFF;
				}
				
				err = getSDOfreeLine(d, whoami, &line);
				if(err) 
				{
//					ATLAS_ERR("SDO error : No line free, too many SDO in progress. Aborted.");
					failedSDO(d, CliServNbr, whoami, index, subIndex, SDOABT_LOCAL_CTRL_ERROR);
					return 0xFF;
				}
				initSDOline(d, line, CliServNbr, index, subIndex, SDO_DOWNLOAD_IN_PROGRESS);

				if(getSDOe(m->Data[0])) 
				{
					nbBytes = 4 - getSDOn2(m->Data[0]);
					d->transfers[line].count = nbBytes;
					err = SDOtoLine(d, line, nbBytes, (*m).Data + 4);
					if(err) 
					{
						failedSDO(d, CliServNbr, whoami, index, subIndex, SDOABT_GENERAL_ERROR);
						return 0xFF;
					}

					//ATLAS_PRINT("SDO Initiate Download is an expedited transfer. Finished. ");
					
					errorCode = SDOlineToObjdict(d, line);
					if(errorCode) 
					{
//						ATLAS_ERR("SDO error : Unable to copy the data in the object dictionary");
						failedSDO(d, CliServNbr, whoami, index, subIndex, errorCode);
						return 0xFF;
					}
					/* ��λsdoͨ�� */
					resetSDOline(d, line);
				}
				else 
				{
					/* �����ֽ�Ϊ�ֽڼ����� */
					if(getSDOs(m->Data[0])) 
					{
						/* ȡ���ֽ��� */
						nbBytes = (m->Data[4]) + ((unsigned int)(m->Data[5])<<8) + ((unsigned int)(m->Data[6])<<16) + ((unsigned int)(m->Data[7])<<24);
						err = setSDOlineRestBytes(d, line, nbBytes);
						if(err)
						{
							failedSDO(d, CliServNbr, whoami, index, subIndex, SDOABT_GENERAL_ERROR);
							return 0xFF;
						}
					}
				}
				/* ����������������Ӧ */
				data[0] = 3 << 5;
				data[1] = index & 0xFF;
				data[2] = (index >> 8) & 0xFF;
				data[3] = subIndex;
				for(i = 4 ; i < 8 ; i++)
					data[i] = 0;
				SendSDO(d, whoami, CliServNbr, data);
			}
    }break;
    case INIT_UPLOAD_REQUEST://251us
    {
      if(whoami == SDO_SERVER) 
			{
				index = getSDOindex(m->Data[1], m->Data[2]);
				subIndex = getSDOsubIndex(m->Data[3]);
				//ATLAS_PRINT("Received SDO Initiate upload (to send data) defined at index 0x1200 + %d\r\n", CliServNbr);
				//ATLAS_PRINT("Reading at index : %d\r\n", index);
				//ATLAS_PRINT("Reading at subIndex : %d\r\n", subIndex);
				if(!err) 
				{
//					ATLAS_ERR("SDO error : Transmission yet started at line : %d\r\n", line);
					//ATLAS_PRINT("Server Nbr = %d\r\ns", CliServNbr);
					failedSDO(d, CliServNbr, whoami, index, subIndex, SDOABT_LOCAL_CTRL_ERROR);
					return 0xFF;
				}
				err = getSDOfreeLine(d, whoami, &line);
				//����ͨ
				if(err) 
				{
//					ATLAS_ERR("SDO error : No line free, too many SDO in progress. Aborted.");
					failedSDO(d, CliServNbr, whoami, index, subIndex, SDOABT_LOCAL_CTRL_ERROR);
					return 0xFF;
				}
				/* ��ʼ������ͨ�� */
				initSDOline(d, line, CliServNbr, index, subIndex, SDO_UPLOAD_IN_PROGRESS);
				errorCode = objdictToSDOline(d, line);
				if(errorCode) 
				{
//					ATLAS_ERR("SDO error : Unable to copy the data from object dictionary. Err code : %d\r\n", errorCode);
					failedSDO(d, CliServNbr, whoami, index, subIndex, errorCode);
					return 0xFF;
				}
				getSDOlineRestBytes(d, line, &nbBytes);
				if(nbBytes > 4) 
				{
					/* ���� */
					data[0] = (2 << 5) | 1;
					/* ���� */
					data[1] = index & 0xFF;
					data[2] = (index >> 8) & 0xFF;
					/* ������ */
					data[3] = subIndex;
					/* �ֽڼ����� */
					data[4] = (unsigned char)nbBytes;
					data[5] = (unsigned char)(nbBytes >> 8);
					data[6] = (unsigned char)(nbBytes >> 16);
					data[7] = (unsigned char)(nbBytes >> 24);
 					//ATLAS_PRINT("SDO. Sending normal upload initiate response defined at index 0x1200 + %d\r\n", nodeId);
					/* ����sdo���� */
					SendSDO(d, whoami, CliServNbr, data);
				}
				/* �ֽ���������4�����ٴ��� */
				else
				{
					/* ���� */
					data[0] = (unsigned char)((2 << 5) | ((4 - nbBytes) << 2) | 3);
					/* ���� */
					data[1] = index & 0xFF;
					data[2] = (index >> 8) & 0xFF;
					/* ������ */
					data[3] = subIndex;
					/* �����ݴӴ���ͨ��������������sdo���� */
					err = lineToSDO(d, line, nbBytes, data + 4);
					if(err) 
					{
						failedSDO(d, CliServNbr, whoami, index, subIndex, SDOABT_GENERAL_ERROR);
						return 0xFF;
					}
					/* ����8�ֽ���0 */
					for(i = 4 + nbBytes; i < 8; i++)
					{
						data[i] = 0;
					}
					
					//ATLAS_PRINT("SDO. Sending expedited upload initiate response defined at index 0x1200 + %d\r\n", CliServNbr);
					
					/* ����sdo���� */
					SendSDO(d, whoami, CliServNbr, data);
					
					/* ��λsdo����ͨ�� */
					resetSDOline(d, line);
				}
			}
    }break;
    case SEGMENT_UPLOAD_REQUEST:
    {
      if(whoami == SDO_SERVER) 
			{
				if(!err)
					err = d->transfers[line].state != SDO_UPLOAD_IN_PROGRESS;
				if(err) 
				{
//					ATLAS_ERR("SDO error : Received upload segment for unstarted trans. index 0x1200 + %d\r\n", CliServNbr);
					failedSDO(d, CliServNbr, whoami, 0, 0, SDOABT_LOCAL_CTRL_ERROR);
					
					return 0xFF;
				}

				RestartSDO_TIMER(line)
					
				//ATLAS_PRINT("Received SDO upload segment defined at index 0x1200 + %d\r\n", CliServNbr);
				
				index = d->transfers[line].index;
				subIndex = d->transfers[line].subIndex;
				
				if(d->transfers[line].toggle != getSDOt(m->Data[0])) 
				{
//					ATLAS_ERR("SDO error : Toggle error : %d\r\n", getSDOt(m->Data[0]));
					failedSDO(d, CliServNbr, whoami, index, subIndex, SDOABT_TOGGLE_NOT_ALTERNED);
					
					return 0xFF;
				}

				/* ��ȡ����ͨ��ʣ���ֽ��� */
				getSDOlineRestBytes(d, line, &nbBytes);
				if(nbBytes > 7) 
				{
					/* �������� */
					data[0] = (d->transfers[line].toggle << 4);
					/* �����ݴ�sdo����ͨ��������������sdo������ */
					err = lineToSDO(d, line, 7, data + 1);
					if(err) 
					{
						failedSDO(d, CliServNbr, whoami, index, subIndex, SDOABT_GENERAL_ERROR);
						return 0xFF;
					}

					/* ������λȡ�� */
					d->transfers[line].toggle = !d->transfers[line].toggle & 1;
					
					//ATLAS_PRINT("SDO. Sending upload segment defined at index 0x1200 + %d\r\n", CliServNbr);
					
					/* ���ͱ��� */
					SendSDO(d, whoami, CliServNbr, data);
				}
				else 
				{
					/* �������� */
					data[0] = (unsigned char)((d->transfers[line].toggle << 4) | ((7 - nbBytes) << 1) | 1);
					/* �����ݴ�sdo����ͨ��������������sdo������ */
					err = lineToSDO(d, line, nbBytes, data + 1);
					if (err) {
						failedSDO(d, CliServNbr, whoami, index, subIndex, SDOABT_GENERAL_ERROR);
						return 0xFF;
					}
					/* ����8�ֽڲ�0 */
					for(i = nbBytes + 1; i < 8; i++)
						data[i] = 0;
					
					//ATLAS_PRINT("SDO. Sending last upload segment defined at index 0x1200 + %d\r\n", CliServNbr);
					
					/* ���ͱ��� */
					SendSDO(d, whoami, CliServNbr, data);
					
					/* ��λ����ͨ�� */
					resetSDOline(d, line);
				}
			}
    }break;
    default:
//      ATLAS_ERR("SDO. Received unknown command specifier : %d\r\n", cs);
    break;
  }
  
 // TimeConsume.SDO = (TIM6->CNT - TimeCheck)>(TIMER6_PERIOD/2)?(TIM6->CNT - TimeCheck + TIMER6_PERIOD):(TIM6->CNT - TimeCheck);
  
  return 0;
}

