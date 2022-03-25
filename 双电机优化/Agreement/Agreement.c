#include "Agreement.h"
#include "stm32f4xx_hal_usart.h"
#include "flash.h"
#include "bsp_BDCMotor.h"
#include "function.h"
static void Agreement_funF4(void);
static void Agreement_funF1(void);
static void Agreement_funF3(void);
static void Agreement_fun3(void);
static void Agreement_funEF(void);
static void Agreement_funF2(void);
void Startagreement(void);
void ReturnError1(void);
void QuickRead(void);
static void ReturnRight(void);
uint8_t CheckSum(uint8_t *Ptr,uint8_t Num );
uint8_t reflag;
uint8_t Flash_Writesign = 0;
uint16_t count=0;
uint8_t aRxBuffer1[200]= {0};
uint8_t SendBuf[200]= {0};
__IO uint8_t SendBuf1[4]= {0};  //��ȷ����ָ�������
__IO uint8_t SendBuf2[5]= {0};
__IO uint8_t SendBuf3[4]= {0};
UART_HandleTypeDef husart_debug;
u8 txflag=0;
void FlashRight();
void changestartstop(void);
int ErrTime1;/* ����ָ����ʱ��*/
/* ��չ���� ------------------------------------------------------------------*/
/* ˽�к���ԭ�� --------------------------------------------------------------*/
/* ������ --------------------------------------------------------------------*/

/**
  * ��������: ����Ӳ����ʼ������
  * �������: huart�����ھ������ָ��
  * �� �� ֵ: ��
  * ˵    ��: �ú�����HAL���ڲ�����
  */


/**
  * ��������: ����Ӳ������ʼ������
  * �������: huart�����ھ������ָ��
  * �� �� ֵ: ��
  * ˵    ��: �ú�����HAL���ڲ�����
  */


/**
  * ��������: ���ڲ�������.
  * �������: ��
  * �� �� ֵ: ��
  * ˵    ������
  */
int32_t transbuf[ParaNum];

u8 SendFlag,ReceiveFlag;

static void changestartstop(void)
{
	if(SendFlag==1)
	{
	transbuf[18]=MotorControl[3].PWM_DutySet;
	transbuf[19]=MotorControl[4].PWM_DutySet;
	transbuf[20]=MotorControl[7].PWM_DutySet;
	transbuf[21]=MotorControl[8].PWM_DutySet;
	transbuf[23]=MotorControl[10].PWM_DutySet;
	transbuf[24]=MotorControl[11].PWM_DutySet;
	transbuf[25]=MotorControl[12].PWM_DutySet;
	transbuf[26]=MotorControl[13].PWM_DutySet;
	transbuf[27]=MotorControl[5].Speed_Set; //��ˢ1�ٶȸ���
	transbuf[28]=MotorControl[6].Speed_Set; //��ˢ2�ٶȸ���
	transbuf[29]=MotorControl[5].Speed_Ref; //��ˢ1�ٶȲο�
	transbuf[30]=MotorControl[6].Speed_Ref; //��ˢ2�ٶȲο�	
  transbuf[33]= MotorControl[5].Speed_Real;  //��ˢ1�ٶ�
	transbuf[34]= MotorControl[6].Speed_Real;  //��ˢ2�ٶ�
	transbuf[35]=MotorControl[8].Speed_Real; 
	transbuf[36]=MotorControl[7].Speed_Real;	
	transbuf[37]=MotorControl[5].SpeedLimit; 
	transbuf[38]=MotorControl[6].SpeedLimit;
	transbuf[44]= MotorControl[0].Current.FilterValue;   //���0����
	transbuf[45]= MotorControl[1].Current.FilterValue;   //���1����
	transbuf[46]= MotorControl[2].Current.FilterValue;   //���2����
	transbuf[47]= MotorControl[5].Current.FilterValue;   //���3����
	transbuf[48]= MotorControl[5].Current.ADCValue_Ref;   //���4����
	transbuf[49]= MotorControl[5].Current.DeepFilterVAL;   //���5����
	transbuf[50]= MotorControl[5].Current.GetADCValue;   //���6����
	transbuf[51]= MotorControl[9].Current.FilterValue;   //���9����
	transbuf[52]=MotorControl[5].Pole_Paires;            //��ˢ1������
	transbuf[53]=MotorControl[6].Pole_Paires ;           //��ˢ2������
	transbuf[54]=MotorControl[5].Direction ;
	transbuf[55]=MotorControl[6].Direction ;
	transbuf[56]=MotorControl[0].Acceleration; //���0���ٶ�
	transbuf[57]=MotorControl[1].Acceleration; //���1���ٶ�	
	transbuf[58]=MotorControl[2].Acceleration; //���2���ٶ�
	transbuf[59]=MotorControl[3].Acceleration; //���3���ٶ�	
	transbuf[60]=MotorControl[4].Acceleration; //���4���ٶ�
	transbuf[61]=MotorControl[5].Acceleration; //���5���ٶ�
	transbuf[62]=MotorControl[6].Acceleration; //���6���ٶ�	
	transbuf[63]=MotorControl[7].Acceleration; //���7���ٶ�
	transbuf[64]=MotorControl[8].Acceleration; //���8���ٶ�	
	transbuf[65]=MotorControl[9].Acceleration; //���9���ٶ�
	transbuf[66]=MotorControl[10].Acceleration; //���10���ٶ�	
	transbuf[67]=MotorControl[11].Acceleration; //���11���ٶ�
	transbuf[68]=MotorControl[12].Acceleration; //���12���ٶ�
	transbuf[69]=MotorControl[13].Acceleration; //���13���ٶ�		
	transbuf[70]=MotorControl[0].Deceleration; //���0���ٶ�
	transbuf[71]=MotorControl[1].Deceleration; //���1���ٶ�
	transbuf[72]=MotorControl[2].Deceleration; //���2���ٶ�
	transbuf[73]=MotorControl[3].Deceleration; //���3���ٶ�
	transbuf[74]=MotorControl[4].Deceleration; //���4���ٶ�
	transbuf[75]=MotorControl[5].Deceleration; //���5���ٶ�
	transbuf[76]=MotorControl[6].Deceleration; //���6���ٶ�
	transbuf[77]=MotorControl[7].Deceleration; //���7���ٶ�
	transbuf[78]=MotorControl[8].Deceleration; //���8���ٶ�
	transbuf[79]=MotorControl[9].Deceleration; //���9���ٶ�
	transbuf[80]=MotorControl[10].Deceleration; //���10���ٶ�
	transbuf[81]=MotorControl[11].Deceleration; //���11���ٶ�
	transbuf[82]=MotorControl[12].Deceleration; //���12���ٶ�
	transbuf[83]=MotorControl[13].Deceleration; //���13���ٶ�
	transbuf[101]=VoltVar.BUS;                  //ĸ�ߵ�ѹ
	transbuf[120]=PID_Speed_InitStruct[0].hKp_Gain;  //��ˢ1kp
	transbuf[121]=PID_Speed_InitStruct[1].hKp_Gain;  //��ˢ2kp
	transbuf[126]=PID_Speed_InitStruct[0].hKi_Gain;  //��ˢ1ki
	transbuf[127]=PID_Speed_InitStruct[1].hKi_Gain;  //��ˢ2ki
	transbuf[134]=MotorControl[0].Current.MaxValue1;
	transbuf[135]=MotorControl[1].Current.MaxValue1;
	transbuf[136]=MotorControl[2].Current.MaxValue1;
	transbuf[137]=MotorControl[9].Current.MaxValue1;
	transbuf[138]=MotorControl[3].Current.MaxValue1;
	transbuf[139]=MotorControl[4].Current.MaxValue1;
	transbuf[140]=MotorControl[5].Current.MaxValue1;
	transbuf[141]=MotorControl[6].Current.MaxValue1;
	transbuf[142]=MotorControl[0].Current.MaxValue2;
	transbuf[143]=MotorControl[1].Current.MaxValue2;
	transbuf[144]=MotorControl[2].Current.MaxValue2;
	transbuf[145]=MotorControl[9].Current.MaxValue2;
	transbuf[146]=MotorControl[3].Current.MaxValue2;
	transbuf[147]=MotorControl[4].Current.MaxValue2;
	transbuf[148]=MotorControl[5].Current.MaxValue2;
	transbuf[149]=MotorControl[6].Current.MaxValue2;
	transbuf[150]=MotorControl[0].Current.OFCnt1;
	transbuf[151]=MotorControl[1].Current.OFCnt1;
	transbuf[152]=MotorControl[2].Current.OFCnt1;
	transbuf[153]=MotorControl[9].Current.OFCnt1;
	transbuf[154]=MotorControl[3].Current.OFCnt1;
	transbuf[155]=MotorControl[4].Current.OFCnt1;
	transbuf[156]=MotorControl[5].Current.OFCnt1;
	transbuf[157]=MotorControl[6].Current.OFCnt1;
	transbuf[158]=MotorControl[0].Current.OFCnt2; 
	transbuf[159]=MotorControl[1].Current.OFCnt2; 
	transbuf[160]=MotorControl[2].Current.OFCnt2; 
	transbuf[161]=MotorControl[9].Current.OFCnt2; 
	transbuf[162]=MotorControl[3].Current.OFCnt2; 
	transbuf[163]=MotorControl[4].Current.OFCnt2;
  transbuf[164]=MotorControl[5].Current.OFCnt2;  
	transbuf[165]=MotorControl[6].Current.OFCnt2;
	transbuf[166]=MotorControl[5].Hall.HallState;
	transbuf[167]=MotorControl[6].Hall.HallState;
  transbuf[170]=HALL_Study[0].StudySectorCnt3;
	transbuf[171]=HALL_Study[1].StudySectorCnt3;
	transbuf[172]=HALL_Study[0].HallSector;
	transbuf[173]=HALL_Study[1].HallSector;
	transbuf[174]=HALL_Study[0].HallCommPWM;
  transbuf[175]=HALL_Study[1].HallCommPWM;	
	transbuf[176]=Push_motor_calibrationFLAG;
	transbuf[177]=wGlobal_Flags;
	transbuf[178]=MotorState;
	transbuf[179]=TIM4->PSC;
	transbuf[180]=TIM10->PSC;

	transbuf[187]=MotorControl[0].Hall.HALL_CaptureValue;
  transbuf[188]=MotorControl[1].Hall.HALL_CaptureValue;
  transbuf[189]=MotorControl[2].Hall.HALL_CaptureValue;
  transbuf[190]=MotorControl[9].Hall.HALL_CaptureValue;
  transbuf[191]=MotorControl[5].Hall.HALL_CaptureValue;
  transbuf[192]=MotorControl[6].Hall.HALL_CaptureValue;		
	transbuf[194]=HALL_Study[0].HallTab[0];
	transbuf[195]=HALL_Study[0].HallTab[1];
	transbuf[196]=HALL_Study[0].HallTab[2];
	transbuf[197]=HALL_Study[0].HallTab[3];
	transbuf[198]=HALL_Study[0].HallTab[4];
	transbuf[199]=HALL_Study[0].HallTab[5];
	transbuf[200]=HALL_Study[1].HallTab[0];
	transbuf[201]=HALL_Study[1].HallTab[1];
	transbuf[202]=HALL_Study[1].HallTab[2];
	transbuf[203]=HALL_Study[1].HallTab[3];
	transbuf[204]=HALL_Study[1].HallTab[4];
	transbuf[205]=HALL_Study[1].HallTab[5];
	transbuf[206]=OverFlow_Cnt[0];
  transbuf[207]=OverFlow_Cnt[1];
  transbuf[208]=OverFlow_Cnt[2];
  transbuf[209]=OverFlow_Cnt[3];
  transbuf[210]=OverFlow_Cnt[4];
  transbuf[211]=Err_codeHis[0];
  transbuf[212]=Err_codeHis[1];
  transbuf[213]=Err_codeHis[2];
  transbuf[214]=Err_codeHis[3];
  transbuf[215]=Err_codeHis[4];
  transbuf[216]=Err_codeHis[5];
  transbuf[217]=Err_codeHis[6];
  transbuf[218]=Err_codeHis[7];
  transbuf[219]=Err_codeHis[8];
  transbuf[220]=Err_codeHis[9];
  transbuf[221]=Err_codeTick[0];
  transbuf[222]=Err_codeTick[1];
  transbuf[223]=Err_codeTick[2];
  transbuf[224]=Err_codeTick[3];
  transbuf[225]=Err_codeTick[4];
  transbuf[226]=Err_codeTick[5];
  transbuf[227]=Err_codeTick[6];
  transbuf[228]=Err_codeTick[7];
  transbuf[229]=Err_codeTick[8];
  transbuf[230]=Err_codeTick[9];
	SendFlag=0;
  }
		
	/*����Ϊ��Ҫд�������*/
	 if(ReceiveFlag==1)
	{
		MotorControl[0].Motor_Start_Stop=transbuf[1];
		MotorControl[1].Motor_Start_Stop=transbuf[2];
		MotorControl[2].Motor_Start_Stop=transbuf[3];
		MotorControl[3].Motor_Start_Stop=transbuf[4];
		MotorControl[4].Motor_Start_Stop=transbuf[5];
		MotorControl[5].Motor_Start_Stop=transbuf[6];
		MotorControl[6].Motor_Start_Stop=transbuf[7];
		MotorControl[7].Motor_Start_Stop=transbuf[8];
		MotorControl[8].Motor_Start_Stop=transbuf[9];
		MotorControl[9].Motor_Start_Stop=transbuf[10];
		MotorControl[10].Motor_Start_Stop=transbuf[11];
		MotorControl[11].Motor_Start_Stop=transbuf[12];
		MotorControl[12].Motor_Start_Stop=transbuf[13];
		MotorControl[13].Motor_Start_Stop=transbuf[14];
		MotorControl[3].PWM_DutySet=transbuf[18];
		MotorControl[4].PWM_DutySet=transbuf[19];
		MotorControl[7].PWM_DutySet=transbuf[20];
		MotorControl[8].PWM_DutySet=transbuf[21];
		MotorControl[10].PWM_DutySet=transbuf[23];
		MotorControl[11].PWM_DutySet=transbuf[24];
		MotorControl[12].PWM_DutySet=transbuf[25];
		MotorControl[13].PWM_DutySet=transbuf[26];
		MotorControl[5].Speed_Set=transbuf[27]; //��ˢ1�ٶȸ���
		MotorControl[6].Speed_Set=transbuf[28]; //��ˢ2�ٶȸ���
	
	 MotorControl[5].Pole_Paires=transbuf[52];            //��ˢ1������
	 MotorControl[6].Pole_Paires=transbuf[53];           //��ˢ2������	
	 MotorControl[5].Direction=transbuf[54];  //��ˢ�������
	 MotorControl[6].Direction=transbuf[55];
   MotorControl[0].Acceleration=transbuf[56]; //���0���ٶ�
	 MotorControl[1].Acceleration=transbuf[57]; //���1���ٶ�	
	 MotorControl[2].Acceleration=transbuf[58]; //���2���ٶ�
	 MotorControl[3].Acceleration=transbuf[59]; //���3���ٶ�	
	 MotorControl[4].Acceleration=transbuf[60]; //���4���ٶ�
	 MotorControl[5].Acceleration=transbuf[61]; //���5���ٶ�
	 MotorControl[6].Acceleration=transbuf[62]; //���6���ٶ�	
	 MotorControl[7].Acceleration=transbuf[63]; //���7���ٶ�
	 MotorControl[8].Acceleration=transbuf[64]; //���8���ٶ�	
	 MotorControl[9].Acceleration=transbuf[65]; //���9���ٶ�
	 MotorControl[10].Acceleration=transbuf[66]; //���10���ٶ�	
	 MotorControl[11].Acceleration=transbuf[67]; //���11���ٶ�
	 MotorControl[12].Acceleration=transbuf[68]; //���12���ٶ�
	 MotorControl[13].Acceleration=transbuf[69]; //���13���ٶ�	
	 MotorControl[0].Deceleration=transbuf[70]; //���0���ٶ�
	 MotorControl[1].Deceleration=transbuf[71]; //���1���ٶ�
	 MotorControl[2].Deceleration=transbuf[72]; //���2���ٶ�
	 MotorControl[3].Deceleration=transbuf[73]; //���3���ٶ�
	 MotorControl[4].Deceleration=transbuf[74]; //���4���ٶ�
	 MotorControl[5].Deceleration=transbuf[75]; //���5���ٶ�
	 MotorControl[6].Deceleration=transbuf[76]; //���6���ٶ�
	 MotorControl[7].Deceleration=transbuf[77]; //���7���ٶ�
	 MotorControl[8].Deceleration=transbuf[78]; //���8���ٶ�
	 MotorControl[9].Deceleration=transbuf[79]; //���9���ٶ�
	 MotorControl[10].Deceleration=transbuf[80]; //���10���ٶ�
	 MotorControl[11].Deceleration=transbuf[81]; //���11���ٶ�
	 MotorControl[12].Deceleration=transbuf[82]; //���12���ٶ�
	 MotorControl[13].Deceleration=transbuf[83]; //���13���ٶ�
	MotorControl[0].Fault_Flag=transbuf[84];   //���0������
	MotorControl[1].Fault_Flag=transbuf[85];   //���1������
	MotorControl[2].Fault_Flag=transbuf[89];   //���2������
	MotorControl[3].Fault_Flag=transbuf[90];   //���3������
	MotorControl[4].Fault_Flag=transbuf[90];   //���4������
 	MotorControl[5].Fault_Flag=transbuf[89];   //���5������
	MotorControl[6].Fault_Flag=transbuf[90];   //���6������
	MotorControl[7].Fault_Flag=transbuf[91];   //���7������
	MotorControl[8].Fault_Flag=transbuf[92];   //���8������
	MotorControl[9].Fault_Flag=transbuf[93];   //���9������
	MotorControl[10].Fault_Flag=transbuf[94];   //���10������
	MotorControl[11].Fault_Flag=transbuf[95];   //���11������
 	MotorControl[12].Fault_Flag=transbuf[96];   //���12������
	MotorControl[13].Fault_Flag=transbuf[97];   //���13������
	PID_Speed_InitStruct[0].hKp_Gain=transbuf[120];  //��ˢ1kp
	PID_Speed_InitStruct[1].hKp_Gain=transbuf[121];  //��ˢ2kp
	PID_Speed_InitStruct[0].hKi_Gain=transbuf[126];  //��ˢ1ki
	PID_Speed_InitStruct[1].hKi_Gain=transbuf[127];  //��ˢ2ki
	HALL_Study[0].StudySectorCnt3=transbuf[170];
	HALL_Study[1].StudySectorCnt3=transbuf[171];
	HALL_Study[0].HallCommPWM=transbuf[174];
  HALL_Study[1].HallCommPWM=transbuf[175];	
	Push_motor_calibrationFLAG=transbuf[176];	
	wGlobal_Flags=transbuf[177];	
	MotorState=transbuf[178];	
	TIM4->PSC=transbuf[179];	
	TIM10->PSC=transbuf[180];	
	
	MotorControl[0].Location_Set=transbuf[181];
	MotorControl[1].Location_Set=transbuf[182];//λ�ø���
	MotorControl[2].Location_Set=transbuf[183];
	MotorControl[9].Location_Set=transbuf[184];
	HALL_Study[0].HallTab[0]=transbuf[194]; /*����ѧϰ*/
	HALL_Study[0].HallTab[1]=transbuf[195];
	HALL_Study[0].HallTab[2]=transbuf[196];
	HALL_Study[0].HallTab[3]=transbuf[197];
	HALL_Study[0].HallTab[4]=transbuf[198];
	HALL_Study[0].HallTab[5]=transbuf[199];
	HALL_Study[1].HallTab[0]=transbuf[200];
	HALL_Study[1].HallTab[1]=transbuf[201];
	HALL_Study[1].HallTab[2]=transbuf[202];
	HALL_Study[1].HallTab[3]=transbuf[203];
	HALL_Study[1].HallTab[4]=transbuf[204];
	HALL_Study[1].HallTab[5]=transbuf[205];
	ReceiveFlag=0;
	 }
}


const PARAMETER_TABLE  PARAMETER[ParaNum] =
{   \
   {0,1000,WR,NOFLASH,(long*)&transbuf[0]},
	{1,1000,WR,NOFLASH,(long*)&transbuf[1]},	
	{2,1001,WR,NOFLASH,(long*)&transbuf[2]},	
	{3,1002,WR,NOFLASH,(long*)&transbuf[3]},	
	{4,1003,WR,NOFLASH,(long*)&transbuf[4]},  
	{5,1004,WR,NOFLASH,(long*)&transbuf[5]},  
	{6,1005,WR,NOFLASH,(long*)&transbuf[6]},  
	{7,1006,WR,NOFLASH,(long*)&transbuf[7]},  //���6ʹ��
	{8,1007,WR,NOFLASH,(long*)&transbuf[8]},  //���7ʹ��
	{9,1008,WR,NOFLASH,(long*)&transbuf[9]},  //���8ʹ��
	{10,1009,WR,NOFLASH,(long*)&transbuf[10]}, //���9ʹ��
	{11,1010,WR,NOFLASH,(long*)&transbuf[11]}, //���10ʹ��
	{12,1011,WR,NOFLASH,(long*)&transbuf[12]}, //���11ʹ��
	{13,1012,WR,NOFLASH,(long*)&transbuf[13]}, //���12ʹ��
	{14,1013,WR,NOFLASH,(long*)&transbuf[14]}, //���13ʹ��
	{15,1014,WR,NOFLASH,(long*)&transbuf[15]}, //���0PWM
	{16,1015,WR,NOFLASH,(long*)&transbuf[16]},	 //1PWM
	{17,1016,WR,NOFLASH,(long*)&transbuf[17]},	 //2PWM������λ�û����޷�ͨ��dutyset����
	{18,1017,WR,NOFLASH,(long*)&transbuf[18]},	 //3PWM
	{19,1018,WR,NOFLASH,(long*)&transbuf[19]},  //4PWM
	{20,1019,WR,NOFLASH,(long*)&transbuf[20]},  //7PWM
	{21,1020,WR,NOFLASH,(long*)&transbuf[21]},  //8PWM
	{22,1021,WR,NOFLASH,(long*)&transbuf[22]},  //9PWM
	{23,1022,WR,NOFLASH,(long*)&transbuf[23]},  //10PWM
	{24,1023,WR,NOFLASH,(long*)&transbuf[24]},  //11PWM
	{25,1024,WR,NOFLASH,(long*)&transbuf[25]},  //12PWM
	{26,1025,WR,NOFLASH,(long*)&transbuf[26]},  //13PWM
	{27,1026,WR,NOFLASH,(long*)&transbuf[27]},  //��ˢ�ٶȸ���1
	{28,1027,WR,NOFLASH,(long*)&transbuf[28]},  //��ˢ�ٶȸ���2
	{29,1028,OR,NOFLASH,(long*)&transbuf[29]},  //��ˢ1�ٶȲο�
	{30,1029,OR,NOFLASH,(long*)&transbuf[30]},  //��ˢ2�ٶȲο�
	{31,1030,OR,NOFLASH,(long*)&transbuf[31]},  //��ˢ1�ٶ�ƫ��
	
	{32,1031,OR,NOFLASH,(long*)&transbuf[32]},	 //��ˢ2�ٶ�ƫ��
	{33,1032,OR,NOFLASH,(long*)&transbuf[33]},	 //��ˢ1ʵ���ٶ�
	{34,1033,OR,NOFLASH,(long*)&transbuf[34]},	 //��ˢ2ʵ���ٶ�
	{35,1034,OR,NOFLASH,(long*)&transbuf[35]},  //���1ʵ���ٶ�
	{36,1035,OR,NOFLASH,(long*)&transbuf[36]},  //��ˢʵ���ٶ�
	{37,1036,WR,YSFLASH,(long*)&transbuf[37]},                    //��ˢ1����ٶ�����
	{38,1037,WR,YSFLASH,(long*)&transbuf[38]},                    //��ˢ2����ٶ�����
	{39,1038,OR,NOFLASH,(long*)&transbuf[39]},  //���0ʵ��λ��
	{40,1039,OR,NOFLASH,(long*)&transbuf[40]},  //���1ʵ��λ��
	{41,1040,OR,NOFLASH,(long*)&transbuf[41]},  //���2ʵ��λ��
	{42,1041,OR,NOFLASH,(long*)&transbuf[42]},  //���9ʵ��λ��
	{43,1042,OR,NOFLASH,(long*)&transbuf[43]},  //���4ʵ��λ��
	{44,1043,OR,NOFLASH,(long*)&transbuf[44]},  //���0����
	{45,1044,OR,NOFLASH,(long*)&transbuf[45]},  //���1����
	{46,1045,OR,NOFLASH,(long*)&transbuf[46]},  //���2����
	{47,1046,OR,NOFLASH,(long*)&transbuf[47]},  //���3����
	  
	{48,1047,OR,NOFLASH,(long*)&transbuf[48]},	 //���4����
	{49,1048,OR,NOFLASH,(long*)&transbuf[49]},	 //���5����
	{50,1049,OR,NOFLASH,(long*)&transbuf[50]},	 //���6����
	{51,1050,OR,NOFLASH,(long*)&transbuf[51]},  //���9����
	{52,1051,WR,YSFLASH,(long*)&transbuf[52]},  //��ˢ1������
	{53,1052,WR,YSFLASH,(long*)&transbuf[53]},  //��ˢ2������
	{54,1053,WR,NOFLASH,(long*)&transbuf[54]},          //��ˢ1���з���
	{55,1054,WR,NOFLASH,(long*)&transbuf[55]},          //��ˢ2���з���
	{56,1055,WR,YSFLASH,(long*)&transbuf[56]},  //���0���ٶ�
	{57,1056,WR,YSFLASH,(long*)&transbuf[57]},  //���1���ٶ�
	{58,1057,WR,YSFLASH,(long*)&transbuf[58]},  //���2���ٶ�
	{59,1058,WR,YSFLASH,(long*)&transbuf[59]},  //���3���ٶ�
	{60,1059,WR,YSFLASH,(long*)&transbuf[60]},  //���4���ٶ�
	{61,1060,WR,YSFLASH,(long*)&transbuf[61]},  //���5���ٶ�
	{62,1061,WR,YSFLASH,(long*)&transbuf[62]},  //6���ٶ�
	{63,1062,WR,YSFLASH,(long*)&transbuf[63]},  //7���ٶ�
	
	{64,1063,WR,YSFLASH,(long*)&transbuf[64]},	 //8���ٶ�
	{65,1064,WR,YSFLASH,(long*)&transbuf[65]},	 //9���ٶ�
	{66,1065,WR,YSFLASH,(long*)&transbuf[66]},	 //10���ٶ�
	{67,1066,WR,YSFLASH,(long*)&transbuf[67]},  //11���ٶ�
	{68,1067,WR,YSFLASH,(long*)&transbuf[68]},  //12���ٶ�
	{69,1068,WR,YSFLASH,(long*)&transbuf[69]},  //13���ٶ�
	{70,1069,WR,YSFLASH,(long*)&transbuf[70]},  //0���ٶ�
	{71,1070,WR,YSFLASH,(long*)&transbuf[71]},  //1���ٶ�
	{72,1071,WR,YSFLASH,(long*)&transbuf[72]},  //2���ٶ�
	{73,1072,WR,YSFLASH,(long*)&transbuf[73]},  //3���ٶ�
	{74,1073,WR,YSFLASH,(long*)&transbuf[74]},  //4���ٶ�
	{75,1074,WR,YSFLASH,(long*)&transbuf[75]},             //5���ٶ�
	{76,1075,WR,YSFLASH,(long*)&transbuf[76]},             //6���ٶ�
	{77,1076,WR,YSFLASH,(long*)&transbuf[77]},  //7���ٶ�
	{78,1077,WR,YSFLASH,(long*)&transbuf[78]},  //8���ٶ�
	{79,1078,WR,YSFLASH,(long*)&transbuf[79]},  //9���ٶ�
	
	{80,1079,WR,YSFLASH,(long*)&transbuf[80]},	 //10���ٶ�
	{81,1080,WR,YSFLASH,(long*)&transbuf[81]},	 //11���ٶ�
	{82,1081,WR,YSFLASH,(long*)&transbuf[82]},	 //12���ٶ�
	{83,1082,WR,YSFLASH,(long*)&transbuf[83]},  //13���ٶ�
	{84,1083,WR,NOFLASH,(long*)&transbuf[84]},  //0������
	{85,1084,WR,NOFLASH,(long*)&transbuf[85]},  //1������
	{86,1085,WR,NOFLASH,(long*)&transbuf[86]},  //2������
	{87,1086,WR,NOFLASH,(long*)&transbuf[87]},  //3������
	{88,1087,WR,NOFLASH,(long*)&transbuf[88]},  //4������
	{89,1088,WR,NOFLASH,(long*)&transbuf[89]},              //5������
	{90,1089,WR,NOFLASH,(long*)&transbuf[90]},              //6������
	{91,1090,WR,NOFLASH,(long*)&transbuf[91]},  //7������
	{92,1091,WR,NOFLASH,(long*)&transbuf[92]},  //8������
	{93,1092,WR,NOFLASH,(long*)&transbuf[93]},  //9������
	{94,1093,WR,NOFLASH,(long*)&transbuf[94]},  //10������
	{95,1094,WR,NOFLASH,(long*)&transbuf[95]},  //11������
	
	{96,1095,WR,NOFLASH,(long*)&transbuf[96]},	 //12������
	{97,1096,WR,NOFLASH,(long*)&transbuf[97]},	 //13������
	{98,1097,OR,NOFLASH,(long*)&transbuf[98]},	 //��ˢ0�¶�
	{99,1098,OR,NOFLASH,(long*)&transbuf[99]},  //��ˢ1�¶�
	{100,1099,OR,NOFLASH,(long*)&transbuf[100]},  //����¶�
	{101,1100,OR,NOFLASH,(long*)&transbuf[101]},  //ĸ�ߵ�ѹ
	{102,1101,WR,NOFLASH,(long*)&transbuf[102]},  //0ɲ���ź�
	{103,1102,WR,NOFLASH,(long*)&transbuf[103]},  //1ɲ���ź�
	{104,1103,WR,NOFLASH,(long*)&transbuf[104]},  //2ɲ���ź�
	{105,1104,WR,NOFLASH,(long*)&transbuf[105]},  //3ɲ���ź�
	{106,1105,WR,NOFLASH,(long*)&transbuf[106]},  //4ɲ���ź�
	{107,1106,WR,NOFLASH,(long*)&transbuf[107]},  //5ɲ���ź�
	{108,1107,WR,NOFLASH,(long*)&transbuf[108]},  //6ɲ���ź�
	{109,1108,WR,NOFLASH,(long*)&transbuf[109]},  //7ɲ���ź�
	{110,1109,WR,NOFLASH,(long*)&transbuf[110]},  //8ɲ���ź�
	{111,1110,WR,NOFLASH,(long*)&transbuf[111]},  //9ɲ���ź�
	
	{112,1111,WR,NOFLASH,(long*)&transbuf[112]},	 //10ɲ���ź�
	{113,1112,WR,NOFLASH,(long*)&transbuf[113]},		//11ɲ���ź�
	{114,1113,WR,NOFLASH,(long*)&transbuf[114]},	  //12ɲ���ź�
	{115,1114,WR,NOFLASH,(long*)&transbuf[115]},    //13ɲ���ź�
	{116,1115,WR,YSFLASH,(long*)&transbuf[116]},    //���0kp
	{117,1116,WR,YSFLASH,(long*)&transbuf[117]},    //���1kp
	{118,1117,WR,YSFLASH,(long*)&transbuf[118]},    //���2kp
	{119,1118,WR,YSFLASH,(long*)&transbuf[119]},     //���9kp
	{120,1119,WR,YSFLASH,(long*)&transbuf[120]},     //���5kp
	{121,1120,WR,YSFLASH,(long*)&transbuf[121]},    //���6kp
	{122,1121,WR,YSFLASH,(long*)&transbuf[122]},    //���0ki
	{123,1122,WR,YSFLASH,(long*)&transbuf[123]},     //���1ki
	{124,1123,WR,YSFLASH,(long*)&transbuf[124]},     //2ki
	{125,1124,WR,YSFLASH,(long*)&transbuf[125]},    //9ki
	{126,1125,WR,YSFLASH,(long*)&transbuf[126]},    //5ki
	{127,1126,WR,YSFLASH,(long*)&transbuf[127]},   //6ki
	
	{128,1127,WR,YSFLASH,(long*)&transbuf[128]},	 //0kd
	{129,1128,WR,YSFLASH,(long*)&transbuf[129]},	 //1kd
	{130,1129,WR,YSFLASH,(long*)&transbuf[130]},	 //2kd
	{131,1130,WR,YSFLASH,(long*)&transbuf[131]},   //9kd
	{132,1131,WR,YSFLASH,(long*)&transbuf[132]},   //5kd
	{133,1132,WR,YSFLASH,(long*)&transbuf[133]},   //6kd
	{134,1133,OR,YSFLASH,(long*)&transbuf[134]},    //0��һ�׶ε�������
	{135,1134,OR,YSFLASH,(long*)&transbuf[135]},    //1��һ�׶ε�������
	{136,1135,OR,YSFLASH,(long*)&transbuf[136]},    //2��һ�׶ε�������
	{137,1136,OR,YSFLASH,(long*)&transbuf[137]},    //9��һ�׶ε�������
	{138,1137,OR,YSFLASH,(long*)&transbuf[138]},    //3��һ�׶ε�������
	{139,1138,OR,YSFLASH,(long*)&transbuf[139]},    //4��һ�׶ε�������
	{140,1139,OR,YSFLASH,(long*)&transbuf[140]},              //5��һ�׶ε�������
	{141,1140,OR,YSFLASH,(long*)&transbuf[141]},              //6��һ�׶ε�������
	{142,1141,OR,YSFLASH,(long*)&transbuf[142]},    //0�ڶ��׶ε�������
	{143,1142,OR,YSFLASH,(long*)&transbuf[143]},    //1�ڶ��׶ε�������
	
	{144,1143,OR,YSFLASH,(long*)&transbuf[144]},	   //2�ڶ��׶ε�������
	{145,1144,OR,YSFLASH,(long*)&transbuf[145]},		//9���׶ε�������
	{146,1145,OR,YSFLASH,(long*)&transbuf[146]},	   //3���׶ε�������
	{147,1146,OR,YSFLASH,(long*)&transbuf[147]},     //4���׶ε�������
	{148,1147,OR,YSFLASH,(long*)&transbuf[148]},             //5���׶ε�������
	{149,1148,OR,YSFLASH,(long*)&transbuf[149]},             //6���׶ε�������
	{150,1149,OR,YSFLASH,(long*)&transbuf[150]},     //0һ�״����ۼ�
	{151,1150,OR,YSFLASH,(long*)&transbuf[151]},     //1һ�״����ۼ�
	{152,1151,OR,YSFLASH,(long*)&transbuf[152]},     //2һ�״����ۼ�
	{153,1152,OR,YSFLASH,(long*)&transbuf[153]},     //3һ�״����ۼ�
	{154,1153,OR,YSFLASH,(long*)&transbuf[154]},     //4һ�״����ۼ�
	{155,1154,OR,YSFLASH,(long*)&transbuf[155]},     //5һ�״����ۼ�
	{156,1155,OR,YSFLASH,(long*)&transbuf[156]},     //6һ�״����ۼ�
	{157,1156,OR,YSFLASH,(long*)&transbuf[157]},     //7һ�״����ۼ�
	{158,1157,OR,YSFLASH,(long*)&transbuf[158]},    //0���״����ۼ�
	{159,1158,OR,YSFLASH,(long*)&transbuf[159]},    //1���״����ۼ�
	
	{160,1159,OR,YSFLASH,(long*)&transbuf[160]},	   //2���׽״����ۼ�
	{161,1160,OR,YSFLASH,(long*)&transbuf[161]},		 //3���׽״����ۼ�
	{162,1161,OR,YSFLASH,(long*)&transbuf[162]},	   //4���׽״����ۼ�
	{163,1162,OR,YSFLASH,(long*)&transbuf[163]},     //5���׽״����ۼ�
	{164,1163,OR,YSFLASH,(long*)&transbuf[164]},      //6���׽״����ۼ�
	{165,1164,OR,YSFLASH,(long*)&transbuf[165]},       //7���׽״����ۼ�
	{166,1165,OR,NOFLASH,(long*)&transbuf[166]},       //��ˢ1������ǰֵ
	{167,1166,OR,NOFLASH,(long*)&transbuf[167]},       //��ˢ2������ǰֵ
	{168,1167,OR,YSFLASH,(long*)&transbuf[168]},       //��ˢ1����ѧϰ����
	{169,1168,OR,YSFLASH,(long*)&transbuf[169]},       //��ˢ2����ѧϰ����
	{170,1169,WR,YSFLASH,(long*)&transbuf[170]},       //��ˢ1���λ���ʱ��
	{171,1170,WR,YSFLASH,(long*)&transbuf[171]},       //��ˢ2���λ���ʱ��
	{172,1171,WR,YSFLASH,(long*)&transbuf[172]},       //��ˢ1ѧϰ�������
	{173,1172,WR,YSFLASH,(long*)&transbuf[173]},       //��ˢ2ѧϰ�������
	{174,1173,WR,YSFLASH,(long*)&transbuf[174]},       //��ˢ1ѧϰpwm
	{175,1174,WR,YSFLASH,(long*)&transbuf[175]},       //��ˢ2ѧϰpwm
	
	{176,1175,WR,NOFLASH,(long*)&transbuf[176]},	     //�Ƹ˱궨��־ 
	{177,1176,WR,NOFLASH,(long*)&transbuf[177]},		   //ȫ�ִ���
	{178,1177,WR,NOFLASH,(long*)&transbuf[178]},	     //��ˢ1����״̬
	{179,1178,WR,YSFLASH,(long*)&transbuf[179]},      //ˮ��1Ƶ�ʣ�����˫��ʱ��ʾ��ˢ2����״̬
	{180,1179,WR,YSFLASH,(long*)&transbuf[180]},       //ˮ��2Ƶ��
  {181,1180,WR,NOFLASH,(long*)&transbuf[181]},        //���0λ�ø���
  {182,1181,WR,NOFLASH,(long*)&transbuf[182]},        //���1λ�ø���
  {183,1182,OR,NOFLASH,(long*)&transbuf[183]},        //���2λ�ø���
  {184,1183,OR,NOFLASH,(long*)&transbuf[184]},        //���9λ�ø���
  {185,1184,OR,NOFLASH,(long*)&transbuf[185]},       
  {186,1185,OR,NOFLASH,(long*)&transbuf[186]},      
  {187,1186,OR,NOFLASH,(long*)&transbuf[187]},        //���0ʵ��λ��
  {188,1187,OR,NOFLASH,(long*)&transbuf[188]},        //���1ʵ��λ��
  {189,1188,OR,NOFLASH,(long*)&transbuf[189]},        //���2ʵ��λ��
  {190,1189,OR,NOFLASH,(long*)&transbuf[190]},        //���9ʵ��λ��
  {191,1190,OR,NOFLASH,(long*)&transbuf[191]},        //���5ʵ��λ��
	{192,1191,OR,NOFLASH,(long*)&transbuf[192]},        //���6ʵ��λ��
  {193,1192,OR,NOFLASH,(long*)&transbuf[193]},            
	{194,1,WR,YSFLASH,(long*)&transbuf[194]},        //��ˢ1����ѧϰ����
	{195,1,WR,YSFLASH,(long*)&transbuf[195]},        //��ˢ1����ѧϰ����
	{196,1,WR,YSFLASH,(long*)&transbuf[196]},        //��ˢ1����ѧϰ����
	{197,1,WR,YSFLASH,(long*)&transbuf[197]},        //��ˢ1����ѧϰ����
	{198,1,WR,YSFLASH,(long*)&transbuf[198]},        //��ˢ1����ѧϰ����
	{199,1,WR,YSFLASH,(long*)&transbuf[199]},        //��ˢ1����ѧϰ����
	{200,1,WR,YSFLASH,(long*)&transbuf[200]},        //��ˢ2����ѧϰ����
	{201,1,WR,YSFLASH,(long*)&transbuf[201]},        //��ˢ2����ѧϰ����
	{202,1,WR,YSFLASH,(long*)&transbuf[202]},        //��ˢ2����ѧϰ����
	{203,1,WR,YSFLASH,(long*)&transbuf[203]},        //��ˢ2����ѧϰ����
	{204,1,WR,YSFLASH,(long*)&transbuf[204]},        //��ˢ2����ѧϰ����
	{205,1,WR,YSFLASH,(long*)&transbuf[205]},        //��ˢ2����ѧϰ����
	{206,1206,WR,YSFLASH,(long*)&transbuf[206]},			//���5����������¼
	{207,1207,WR,YSFLASH,(long*)&transbuf[207]},			//���6����������¼
	{208,1208,WR,YSFLASH,(long*)&transbuf[208]},			//�Ƹ˹���������¼
	{209,1209,WR,YSFLASH,(long*)&transbuf[209]},			//���3��4����������¼
	{210,1210,WR,YSFLASH,(long*)&transbuf[210]},				//���10��11��12��13����������¼
	{211,1211,OR,YSFLASH,(long*)&transbuf[211]},			//������ʷ1
	{212,1212,OR,YSFLASH,(long*)&transbuf[212]},			//������ʷ2
	{213,1213,OR,YSFLASH,(long*)&transbuf[213]},			//������ʷ3
	{214,1214,OR,YSFLASH,(long*)&transbuf[214]},			//������ʷ4
	{215,1215,OR,YSFLASH,(long*)&transbuf[215]},			//������ʷ5
	{216,1216,OR,YSFLASH,(long*)&transbuf[216]},			//������ʷ6
	{217,1217,OR,YSFLASH,(long*)&transbuf[217]},			//������ʷ7
	{218,1218,OR,YSFLASH,(long*)&transbuf[218]},			//������ʷ8
	{219,1219,OR,YSFLASH,(long*)&transbuf[219]},			//������ʷ9
	{220,1220,OR,YSFLASH,(long*)&transbuf[220]},			//������ʷ10
	{221,1221,OR,YSFLASH,(long*)&transbuf[221]},			//����ʱ��1
	{222,1222,OR,YSFLASH,(long*)&transbuf[222]},			//����ʱ��2
	{223,1223,OR,YSFLASH,(long*)&transbuf[223]},			//����ʱ��3
	{224,1224,OR,YSFLASH,(long*)&transbuf[224]},			//����ʱ��4
	{225,1225,OR,YSFLASH,(long*)&transbuf[225]},			//����ʱ��5
	{226,1226,OR,YSFLASH,(long*)&transbuf[226]},			//����ʱ��6
	{227,1227,OR,YSFLASH,(long*)&transbuf[227]},			//����ʱ��7
	{228,1228,OR,YSFLASH,(long*)&transbuf[228]},			//����ʱ��8
	{229,1229,OR,YSFLASH,(long*)&transbuf[229]},			//����ʱ��9
	{230,1230,OR,YSFLASH,(long*)&transbuf[230]},			//����ʱ��10
};
uint32_t  test=0;
extern uint8_t aRxBuffer[50];
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
//    test++;
//    HAL_UART_Receive_IT(&huart3,aRxBuffer,1);

}
void MX_USART3_UART_Init(void)
{

    /* USER CODE BEGIN USART3_Init 0 */

    /* USER CODE END USART3_Init 0 */

    /* USER CODE BEGIN USART3_Init 1 */

    /* USER CODE END USART3_Init 1 */
    huart3.Instance = USART3;
    huart3.Init.BaudRate = 115200;
    huart3.Init.WordLength = UART_WORDLENGTH_8B;
    huart3.Init.StopBits = UART_STOPBITS_1;
    huart3.Init.Parity = UART_PARITY_NONE;
    huart3.Init.Mode = UART_MODE_TX_RX;
    huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart3.Init.OverSampling = UART_OVERSAMPLING_16;
    if (HAL_UART_Init(&huart3) != HAL_OK)
    {
        Error_Handler(); 
    } 
    /* USER CODE BEGIN USART3_Init 2 */

    /* USER CODE END USART3_Init 2 */

}

//void DEBUG_USARTx_IRQHandler()                     //�����жϴ�����
//{
//    if(__HAL_USART_GET_FLAG(&huart3,USART_FLAG_RXNE)!= RESET) // �����жϣ����յ�����
//    {
//        uint8_t data;
//        data=READ_REG(huart3.Instance->DR); // ��ȡ����
//        /*�¼ӵĶ���*/
//        if(count==0) // ��������½��յ�����֡���������ڿ����ж�
//        {
//            __HAL_UART_CLEAR_FLAG(&huart3,USART_FLAG_IDLE); // ��������жϱ�־
//            __HAL_UART_ENABLE_IT(&huart3,USART_IT_IDLE);     // ʹ�ܿ����ж�
//        }

//        aRxBuffer1[count++]=data;//�����ݱ�����aRxBuffer��
//    }
//    else	if(__HAL_USART_GET_FLAG(&huart3,USART_FLAG_IDLE)!= RESET) // ���ڿ����ж� �¼ӵ�
//    {
//        __HAL_UART_CLEAR_FLAG(&huart3,USART_FLAG_IDLE); // ��������жϱ�־
//        __HAL_UART_DISABLE_IT(&huart3,USART_IT_IDLE);    // �رտ����ж�
//        reflag=1;		                                 // ����֡��λ����ʶ���յ�һ����������֡
//        count=0;
//    }
//		
//}

void Flash_WriteCheck(void)
{
	if(Flash_Writesign)
	{
		if(InternalFlash(transbuf,ParaNum))
			Flash_Writesign=0;
	}
}

void Startagreement()
{
    if(reflag==1)             //�յ�������
    {

        if(aRxBuffer1[0]==FRAME_START)                          // �ж���λ�ǲ���5A
        {
            if(aRxBuffer1[1]==0x03)                           //�жϹ�����//��λ�������ݸ���λ��
            {
							SendFlag=1;
							changestartstop();
                if(CheckSum((uint8_t*)&aRxBuffer1[0],aRxBuffer1[2]+3)==aRxBuffer1[aRxBuffer1[2]+3])	  //03������У��
                {
                    Agreement_fun3();
									  ReturnRight();
                }

            }

            if((aRxBuffer1[1]==0xE2) && (aRxBuffer1[2]==0x00)&& (aRxBuffer1[3]==0x3C))
            {
                if(InternalFlash(transbuf,ParaNum)==1)
                    FlashRight();
            }


            if(aRxBuffer1[1]==0x06)                           //��λ��������λ��
            {
                if(CheckSum((uint8_t*)&aRxBuffer1[0],7)==aRxBuffer1[7])	  //06������У��
                {
									  ReceiveFlag=1;
										if((aRxBuffer1[2]==0xB2)|| (aRxBuffer1[2]==0xB3))    //����״̬����������Ϊenum����
									{
										*PARAMETER[aRxBuffer1[2]].lpParam=aRxBuffer1[3];
										changestartstop();
										ReturnRight();
									}
									else
									{
                    *PARAMETER[aRxBuffer1[2]].lpParam=aRxBuffer1[3]|aRxBuffer1[4]<<8|aRxBuffer1[5]<<16|aRxBuffer1[6]<<24;
										changestartstop();
                    ReturnRight();
									}
									
									
                }		
            }
        }
        reflag=0;
    }
}


void QuickRead()
{
    if(aRxBuffer1[0]==FRAME_START)                          // �ж���λ�ǲ���5A
    {

        if(aRxBuffer1[1]==0xEF)                           //�жϹ�����//��λ�������ݸ���λ��
        {
					  SendFlag=1;
					  changestartstop();
            if(CheckSum((uint8_t*)&aRxBuffer1[0],aRxBuffer1[2]+3)==aRxBuffer1[aRxBuffer1[2]+3])	  //У��           
            Agreement_funEF();   
        }
		else if(aRxBuffer1[1]==0xF2)
		  {
				SendFlag=1;
				changestartstop();
        if(CheckSum((uint8_t*)&aRxBuffer1[0],4)==aRxBuffer1[4])	  //У��
				        Agreement_funF2();
		  }
			else if(aRxBuffer1[1]==0xF1)
		  {
				SendFlag=1;
				changestartstop();
        if(CheckSum((uint8_t*)&aRxBuffer1[0],3)==aRxBuffer1[3])	  //У��
					 {
				        Agreement_funF1();
					 }

		  }
			else if(aRxBuffer1[1]==0xF3)
		  {
				SendFlag=1;
				changestartstop();
        if(CheckSum((uint8_t*)&aRxBuffer1[0],5)==aRxBuffer1[5])	  //У��
					 {
				        Agreement_funF3();
					 }
		  }
					else if(aRxBuffer1[1]==0xF4)
		  {
				SendFlag=1;
				changestartstop();
        if(CheckSum((uint8_t*)&aRxBuffer1[0],6)==aRxBuffer1[6])	  //У��
					 {
				        Agreement_funF4();
					 }
		  }
    }
}



void Agreement_fun3()         //���Ĵ���
{
    int j;
    SendBuf[0]=FRAME_START;   //�������ݵ�һ���ֽ�
    SendBuf[1]=0x03;            //�������ݵڶ�һ���ֽ�
    SendBuf[2]=aRxBuffer1[2];           //�������ݵ������ֽ�
    for(j=1; j<=aRxBuffer1[2]; j++)   //ѭ����������
    {
        SendBuf[4*(j-1)+3]= (*PARAMETER[aRxBuffer1[2+j]].lpParam)&0xFF;
        SendBuf[4*(j-1)+4]= (*PARAMETER[aRxBuffer1[2+j]].lpParam)>>8&0xFF;
        SendBuf[4*(j-1)+5]= ((*PARAMETER[aRxBuffer1[2+j]].lpParam)>>16)&0xFF;
        SendBuf[4*(j-1)+6]= ((*PARAMETER[aRxBuffer1[2+j]].lpParam)>>24);
    }
    SendBuf[4*aRxBuffer1[2]+3]=CheckSum((uint8_t*)&SendBuf[0],4*aRxBuffer1[2]+3);
    HAL_UART_Transmit(&huart3,(uint8_t *)&SendBuf,4*aRxBuffer1[2]+4,1000);   //�������ݸ���λ��
}



void Agreement_funEF()         //��������Ĵ�����ʵʱ��
{
    int j;
    SendBuf[0]=FRAME_START;   //�������ݵ�һ���ֽ�
    SendBuf[1]=0xEF;            //�������ݵڶ�һ���ֽ�
    SendBuf[2]=aRxBuffer1[2];           //�������ݵ������ֽ�
    for(j=1; j<=aRxBuffer1[2]; j++)   //ѭ����������
    {
        SendBuf[4*(j-1)+3]=  *PARAMETER[aRxBuffer1[2+j]].lpParam&0xFF;
        SendBuf[4*(j-1)+4]= (*PARAMETER[aRxBuffer1[2+j]].lpParam>>8)&0xFF;
        SendBuf[4*(j-1)+5]= (*PARAMETER[aRxBuffer1[2+j]].lpParam>>16)&0XFF;
        SendBuf[4*(j-1)+6]= (*PARAMETER[aRxBuffer1[2+j]].lpParam>>24)&0XFF;
    } 
    SendBuf[4*aRxBuffer1[2]+3]=CheckSum((uint8_t*)&SendBuf[0],4*aRxBuffer1[2]+3);   //����CRCУ��λ
    HAL_UART_Transmit(&huart3,(uint8_t *)&SendBuf,4*aRxBuffer1[2]+4,500);   //�������ݸ���λ��(����У��λ������Ϊ��4)
}

void Agreement_funF2()             //ʾ��������   //������115200(bps) �� 115200 (λ/��) = 11.25 (KB/��)= 11520 (�ֽ�/��) 						  
{
	int j;
	SendBuf[0]=FRAME_START;   //�������ݵ�һ���ֽ�
	SendBuf[1]=0xF2;            //�������ݵڶ�һ���ֽ�
	for(j=1;j<=2;j++)     //ѭ����������
	{
		SendBuf[4*(j-1)+2]=  *PARAMETER[aRxBuffer1[1+j]].lpParam&0xFF;
		SendBuf[4*(j-1)+3]= (*PARAMETER[aRxBuffer1[1+j]].lpParam>>8)&0xFF;
		SendBuf[4*(j-1)+4]= (*PARAMETER[aRxBuffer1[1+j]].lpParam>>16)&0XFF;
		SendBuf[4*(j-1)+5]= (*PARAMETER[aRxBuffer1[1+j]].lpParam>>24)&0XFF;
	}
	   SendBuf[10]=CheckSum((uint8_t*)&SendBuf[0],10);   //����CRCУ��λ
	   HAL_UART_Transmit(&huart3,(uint8_t *)&SendBuf,11,200);   //�������ݸ���λ��(����У��λ������Ϊ��4)
}

void Agreement_funF1()             //ʾ��������   //������115200(bps) �� 115200 (λ/��) = 11.25 (KB/��)= 11520 (�ֽ�/��) 						  
{
	int j;
	SendBuf[0]=FRAME_START;   //�������ݵ�һ���ֽ�
	SendBuf[1]=0xF1;            //�������ݵڶ�һ���ֽ�
	for(j=1;j<=1;j++)     //ѭ����������
	{
		SendBuf[4*(j-1)+2]=  *PARAMETER[aRxBuffer1[1+j]].lpParam&0xFF;
		SendBuf[4*(j-1)+3]= (*PARAMETER[aRxBuffer1[1+j]].lpParam>>8)&0xFF;
		SendBuf[4*(j-1)+4]= (*PARAMETER[aRxBuffer1[1+j]].lpParam>>16)&0XFF;
		SendBuf[4*(j-1)+5]= (*PARAMETER[aRxBuffer1[1+j]].lpParam>>24)&0XFF;
	}
	   SendBuf[6]=CheckSum((uint8_t*)&SendBuf[0],6);   //����CRCУ��λ
	   HAL_UART_Transmit(&huart3,(uint8_t *)&SendBuf,7,200);   //�������ݸ���λ��(����У��λ������Ϊ��4)
}

void Agreement_funF3()             //ʾ��������   //������115200(bps) �� 115200 (λ/��) = 11.25 (KB/��)= 11520 (�ֽ�/��) 						  
{
	int j;
	SendBuf[0]=FRAME_START;   //�������ݵ�һ���ֽ�
	SendBuf[1]=0xF3;            //�������ݵڶ�һ���ֽ�
	for(j=1;j<=3;j++)     //ѭ����������
	{
		SendBuf[4*(j-1)+2]=  *PARAMETER[aRxBuffer1[1+j]].lpParam&0xFF;
		SendBuf[4*(j-1)+3]= (*PARAMETER[aRxBuffer1[1+j]].lpParam>>8)&0xFF;
		SendBuf[4*(j-1)+4]= (*PARAMETER[aRxBuffer1[1+j]].lpParam>>16)&0XFF;
		SendBuf[4*(j-1)+5]= (*PARAMETER[aRxBuffer1[1+j]].lpParam>>24)&0XFF;
	}
	   SendBuf[14]=CheckSum((uint8_t*)&SendBuf[0],14);   //����CRCУ��λ
	   HAL_UART_Transmit(&huart3,(uint8_t *)&SendBuf,15,200);   //�������ݸ���λ��(����У��λ������Ϊ��4)
}

void Agreement_funF4()             //ʾ��������   //������115200(bps) �� 115200 (λ/��) = 11.25 (KB/��)= 11520 (�ֽ�/��) 						  
{
	int j;
	SendBuf[0]=FRAME_START;   //�������ݵ�һ���ֽ�
	SendBuf[1]=0xF4;            //�������ݵڶ�һ���ֽ�
	for(j=1;j<=4;j++)     //ѭ����������
	{
		SendBuf[4*(j-1)+2]=  *PARAMETER[aRxBuffer1[1+j]].lpParam&0xFF;
		SendBuf[4*(j-1)+3]= (*PARAMETER[aRxBuffer1[1+j]].lpParam>>8)&0xFF;
		SendBuf[4*(j-1)+4]= (*PARAMETER[aRxBuffer1[1+j]].lpParam>>16)&0XFF;
		SendBuf[4*(j-1)+5]= (*PARAMETER[aRxBuffer1[1+j]].lpParam>>24)&0XFF;
	}
	   SendBuf[18]=CheckSum((uint8_t*)&SendBuf[0],18);   //����CRCУ��λ
	   HAL_UART_Transmit(&huart3,(uint8_t *)&SendBuf,19,200);   //�������ݸ���λ��(����У��λ������Ϊ��4)
}


void ReturnRight()         //������ȷָ��
{
    SendBuf1[0]=FRAME_START;   //�������ݵ�һ���ֽ�
    SendBuf1[1]=0x06;            //�������ݵڶ�һ���ֽ�
    SendBuf1[2]=0x00;           //�������ݵ������ֽ�
    SendBuf1[3]=0xF0;
    HAL_UART_Transmit(&huart3,(uint8_t *)&SendBuf1,4,1000);   //�������ݸ���λ��
}


uint8_t CheckSum(uint8_t *Ptr,uint8_t Num )        //У������
{
    uint16_t Sum = 0;
    while(Num--)
    {
        Sum += *Ptr;
        Ptr++;
    }
    Sum=Sum%0x100;               //ȡ��
    return Sum;
}





void ReturnError1()        //��������
{
	  uint8_t code0[8]={0x5A,0xE1,0x54,0x00,0x00,0x00,0x01,0x90};  //���0����
		uint8_t code1[8]={0x5A,0xE1,0x55,0x00,0x00,0x00,0x01,0x91};  //���1����
		uint8_t code2[8]={0x5A,0xE1,0x56,0x00,0x00,0x00,0x01,0x92};  //���2����
		uint8_t code3[8]={0x5A,0xE1,0x57,0x00,0x00,0x00,0x01,0x93};  //���3����
		uint8_t code4[8]={0x5A,0xE1,0x58,0x00,0x00,0x00,0x01,0x94};  //���4����
		uint8_t code5[8]={0x5A,0xE1,0x59,0x00,0x00,0x00,0x01,0x95};  //���5����
		uint8_t code6[8]={0x5A,0xE1,0x5A,0x00,0x00,0x00,0x01,0x96};  //���6����
		uint8_t code7[8]={0x5A,0xE1,0x5B,0x00,0x00,0x00,0x01,0x97};  //���7����
		uint8_t code8[8]={0x5A,0xE1,0x5C,0x00,0x00,0x00,0x01,0x98};  //���8����
		uint8_t code9[8]={0x5A,0xE1,0x5D,0x00,0x00,0x00,0x01,0x99};  //���9����
		uint8_t code10[8]={0x5A,0xE1,0x5E,0x00,0x00,0x00,0x01,0x9A};  //���10����
		uint8_t code11[8]={0x5A,0xE1,0x5F,0x00,0x00,0x00,0x01,0x9B};  //���11����
		uint8_t code12[8]={0x5A,0xE1,0x60,0x00,0x00,0x00,0x01,0x9C};  //���12����
		uint8_t code13[8]={0x5A,0xE1,0x61,0x00,0x00,0x00,0x01,0x9D};  //���13����
		uint8_t err0[8]={0x5A,0xE1,0xB1,0x00,0x00,0x00,0x00,0x00};
		if(wGlobal_Flags!=0)     //ȫ�ִ���
		 {
			 ErrTime1++;
				if(ErrTime1==100)
		  {  
				ErrTime1=0;
				err0[3]=(wGlobal_Flags>>24)&0xFF;
				err0[4]=(wGlobal_Flags>>16)&0xFF;
				err0[5]=(wGlobal_Flags>>8)&0xFF;
				err0[6]= wGlobal_Flags&0xFF;
				err0[7]=CheckSum((uint8_t*)&err0[0],7);
			  HAL_UART_Transmit(&huart3,(uint8_t *)&err0,8,1000);
			}
		}
		

		if(MotorControl[0].Fault_Flag==1)        //���0����
		{
			MotorControl[0].Fault_Cnt++; 
			if(MotorControl[0].Fault_Cnt==100)
			{
				MotorControl[0].Fault_Cnt=0;
			  HAL_UART_Transmit(&huart3,(uint8_t *)&code0,8,1000);
			}
		}
			if(MotorControl[1].Fault_Flag==1)        //���1����
		{
			MotorControl[1].Fault_Cnt++; 
			if(MotorControl[1].Fault_Cnt==100)
			{
				MotorControl[1].Fault_Cnt=0;
			  HAL_UART_Transmit(&huart3,(uint8_t *)&code1,8,1000);
			}
		}
		  if(MotorControl[2].Fault_Flag==1)        //���2����
		{
			MotorControl[2].Fault_Cnt++; 
			if(MotorControl[2].Fault_Cnt==100)
			{
				MotorControl[2].Fault_Cnt=0;
			  HAL_UART_Transmit(&huart3,(uint8_t *)&code2,8,1000);
			}
		}
		    if(MotorControl[3].Fault_Flag==1)        //���3����
		{
			MotorControl[3].Fault_Cnt++; 
			if(MotorControl[3].Fault_Cnt==100)
			{
				MotorControl[3].Fault_Cnt=0;
			  HAL_UART_Transmit(&huart3,(uint8_t *)&code3,8,1000);
			}
		}
		
		  if(MotorControl[4].Fault_Flag==1)        //���4����
		{
			MotorControl[4].Fault_Cnt++; 
			if(MotorControl[4].Fault_Cnt==100)
			{
				MotorControl[4].Fault_Cnt=0;
			  HAL_UART_Transmit(&huart3,(uint8_t *)&code4,8,1000);
			}
		}
		
		  if(MotorControl[5].Fault_Flag==1)        //���5����
		{
			MotorControl[5].Fault_Cnt++; 
			if(MotorControl[5].Fault_Cnt==100)
			{
				MotorControl[5].Fault_Cnt=0;
			  HAL_UART_Transmit(&huart3,(uint8_t *)&code5,8,1000);
			}
		}
		
		  if(MotorControl[6].Fault_Flag==1)        //���6����
		{
			MotorControl[6].Fault_Cnt++; 
			if(MotorControl[6].Fault_Cnt==100)
			{
				MotorControl[6].Fault_Cnt=0;
			  HAL_UART_Transmit(&huart3,(uint8_t *)&code6,8,1000);
			}
		}
		
		    if(MotorControl[7].Fault_Flag==1)        //���7����
		{
			MotorControl[7].Fault_Cnt++; 
			if(MotorControl[7].Fault_Cnt==100)
			{
				MotorControl[7].Fault_Cnt=0;
			  HAL_UART_Transmit(&huart3,(uint8_t *)&code7,8,1000);
			}
		}
		if(MotorControl[8].Fault_Flag==1)        //���8����
		{
			MotorControl[8].Fault_Cnt++; 
			if(MotorControl[8].Fault_Cnt==100)
			{
				MotorControl[8].Fault_Cnt=0;
			  HAL_UART_Transmit(&huart3,(uint8_t *)&code8,8,1000);
			}
		}
			if(MotorControl[9].Fault_Flag==1)        //���9����
		{
			MotorControl[9].Fault_Cnt++; 
			if(MotorControl[9].Fault_Cnt==100)
			{
				MotorControl[9].Fault_Cnt=0;
			  HAL_UART_Transmit(&huart3,(uint8_t *)&code9,8,1000);
			}
		}
		  if(MotorControl[10].Fault_Flag==1)        //���10����
		{
			MotorControl[10].Fault_Cnt++; 
			if(MotorControl[10].Fault_Cnt==100)
			{
				MotorControl[10].Fault_Cnt=0;
			  HAL_UART_Transmit(&huart3,(uint8_t *)&code10,8,1000);
			}
		}
		    if(MotorControl[11].Fault_Flag==1)        //���11����
		{
			MotorControl[11].Fault_Cnt++; 
			if(MotorControl[11].Fault_Cnt==100)
			{
				MotorControl[11].Fault_Cnt=0;
			  HAL_UART_Transmit(&huart3,(uint8_t *)&code11,8,1000);
			}
		}
		
		  if(MotorControl[12].Fault_Flag==1)        //���12����
		{
			MotorControl[12].Fault_Cnt++; 
			if(MotorControl[12].Fault_Cnt==100)
			{
				MotorControl[12].Fault_Cnt=0;
			  HAL_UART_Transmit(&huart3,(uint8_t *)&code12,8,1000);
			}
		}
		
		  if(MotorControl[13].Fault_Flag==1)        //���13����
		{
			MotorControl[13].Fault_Cnt++; 
			if(MotorControl[13].Fault_Cnt==100)
			{
				MotorControl[13].Fault_Cnt=0;
			  HAL_UART_Transmit(&huart3,(uint8_t *)&code13,8,1000);
			}
		}
}
void FlashRight()
{
    SendBuf3[0]=FRAME_START;   //�������ݵ�һ���ֽ�
    SendBuf3[1]=0xE2;            //�������ݵڶ�һ���ֽ�
    SendBuf3[2]=0x00;           //�������ݵ������ֽ�
    SendBuf3[3]=0xF0;
    HAL_UART_Transmit(&huart3,(uint8_t *)&SendBuf3,4,1000);   //�������ݸ���λ��
}