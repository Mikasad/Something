#include "bsp_BDCMotor.h"
#include "main.h"
#include "function.h"
#include "Sensorless bldc.h"
void Sensorless_Start(void);
void SensorlessBLDC1_PhaseChange(u8 bHallState, s16 PWM_Duty);
s16 Sensorless_ChangePwm(u8 Mortor_NUM);
unsigned long BEMF(void);
extern u16 PWM_Complementary_Switch_M1 ,PWM_Complementary_Switch_M2 ;
void Mul_change(u8 Motor_NUM);
s32 f_abs(s32 a);
Sensorless_t Sensorless[2];
u8 bHallStartStep=1;
int16_t VoltBEMF[13]={0};
u8 dir=0;
u8 ucMotorAD=0;
int32_t usOZTimeS=0;
int32_t usOZTimeS1=0;
u8 ClockDir=0;
int16_t BEMF_Cnt;
int16_t hallmin;
int sensorless[100];
s32 Ack_time[9]={0};           //过零点时间
s32 Record_time[9]={0};           //过零点时间
s16 Ack_Flag[9]={0};
float RiseMul=1; //0.8   0.8
float DowmMul=1;  //1.3   1.2

s32 f_abs(s32 a)
{
    s32 temp;
    if(a>0) temp = a;
    else if(a<0) temp = -a;

    return temp;
}



s16 Sensorless_ChangePwm(u8 Mortor_NUM)
{
    if(Sensorless[Mortor_NUM].PWM_DutySet > Sensorless[Mortor_NUM].PWM_Duty)
    {
        Sensorless[Mortor_NUM].PWM_Duty += Sensorless[Mortor_NUM].Acceleration;
        if(Sensorless[Mortor_NUM].PWM_DutySet < Sensorless[Mortor_NUM].PWM_Duty)
        {
            Sensorless[Mortor_NUM].PWM_Duty = Sensorless[Mortor_NUM].PWM_DutySet;
        }

    }
    else if(Sensorless[Mortor_NUM].PWM_DutySet< Sensorless[Mortor_NUM].PWM_Duty)
    {
        Sensorless[Mortor_NUM].PWM_Duty -= Sensorless[Mortor_NUM].Deceleration;
        if(Sensorless[Mortor_NUM].PWM_DutySet > Sensorless[Mortor_NUM].PWM_Duty)
        {
            Sensorless[Mortor_NUM].PWM_Duty = Sensorless[Mortor_NUM].PWM_DutySet;
        }
    }

}
void SensorlessBLDC1_PhaseChange(u8 bHallState, s16 PWM_Duty)
{
	Sensorless[0].PWMTicksPre = Sensorless[0].PWMTicks;
    Sensorless[0].FLAGBEMF=0;
    Sensorless[0].PWMTicks = 0;
    if(PWM_Duty < 0) PWM_Duty = -PWM_Duty;

    if(bHallState == 0)
    {
        MC_SetFault(M1_HALL_ERR);
    }
   else if(bHallState == 5)     //BC
    {
		TIM1->CCER = 0x1410;
		TIM1->CCR2 = PWM_PERIOD;
        TIM1->CCR1 = PWM_PERIOD;
        TIM1->CCR3 = PWM_PERIOD;
        TIM1->CCR2 = PWM_Duty;
    }

    else if(bHallState == 6)  //AB
    {
		TIM1->CCER = 0x1041;
		TIM1->CCR2 = PWM_PERIOD;
        TIM1->CCR1 = PWM_PERIOD;
        TIM1->CCR3 = PWM_PERIOD;
        TIM1->CCR1 = PWM_Duty;
    }
    else if(bHallState == 1) //BA
    {
		TIM1->CCER =  0x1014;
		TIM1->CCR2 = PWM_PERIOD;
        TIM1->CCR1 = PWM_PERIOD;
        TIM1->CCR3 = PWM_PERIOD;
        TIM1->CCR2 = PWM_Duty;
    }
    else if(bHallState == 2) //CB
    {
		TIM1->CCER = 0x1140;
		TIM1->CCR2 = PWM_PERIOD;
        TIM1->CCR1 = PWM_PERIOD;
        TIM1->CCR3 = PWM_PERIOD;
        TIM1->CCR3 = PWM_Duty;
    }
    else if(bHallState == 3) //CA
    {
		TIM1->CCER = 0x1104;
		TIM1->CCR2 = PWM_PERIOD;
        TIM1->CCR1 = PWM_PERIOD;
        TIM1->CCR3 = PWM_PERIOD;
        TIM1->CCR3 = PWM_Duty;
    }

    else if(bHallState == 4) //AC
    {

		TIM1->CCER = 0x1401;
		TIM1->CCR2 = PWM_PERIOD;
        TIM1->CCR1 = PWM_PERIOD;
        TIM1->CCR3 = PWM_PERIOD;
        TIM1->CCR1 = PWM_Duty;
    }
    else if(bHallState == 7)
    {
        MC_SetFault(M1_HALL_ERR);
    }
}

u8 change_temp = 0;
u8 change_flag=0;


int i=0;
int caixiangzhi=20;
s32 Sensorcnt=0;
extern s32 Sensorlesstime;
extern s32 DriveTime;
int MeneFlag=0;
uint8_t HallchangeValue=0;
int8_t Sensorless_Time[7]={0};
int8_t Fileter_cnt[7]={0};
int Delay_ZeroTime=0;
void Sensorless_Start(void)                 //三步无感驱动。程序在main中定时器中断回调函数中运行
{
	 Sensorless[0].PWMTicks++;
	switch (Sensorless[0].State)
	 {
		 case 0:
			  if(++Sensorless[M1].IdleCountCnt1 <1000)
			 {
				 SensorlessBLDC1_PhaseChange(1,4000);
			 }
			 else
			 {
				 Sensorless[0].State++;
			 }
			 break;
		 case 1:			
		    if(Sensorless[M1].CountSectorCnt < 50)
			{
				if(++Sensorless[M1].CountSectorCnt2 >Sensorless[M1].CountSectorCnt3 )
				{
					BEMF();
					Sensorless[M1].CountSectorCnt3-=1;
					Sensorless[M1].CountSectorCnt2 =0;
					if(Sensorless[M1].CountSectorCnt3<Sensorlesstime)           //换向时间
					{
						Sensorless[M1].CountSectorCnt3=Sensorlesstime-1;
					}		
					 Sensorless[M1].SenlessHallSector ++;

					if(Sensorless[M1].SenlessHallSector>6)
					{
						Sensorless[M1].SenlessHallSector = 1;
						Sensorless_Time[1]=0;
						Sensorless_Time[2]=0;
						Sensorless_Time[3]=0;
						Sensorless_Time[4]=0;
						Sensorless_Time[5]=0;
						Sensorless_Time[6]=0;
					}		
					if(Sensorless[M1].SenlessHallSector == 1) change_temp = 3;
					if(Sensorless[M1].SenlessHallSector == 2) change_temp = 2;
					if(Sensorless[M1].SenlessHallSector == 3) change_temp = 6;
					if(Sensorless[M1].SenlessHallSector == 4) change_temp = 4;
					if(Sensorless[M1].SenlessHallSector == 5) change_temp = 5;
					if(Sensorless[M1].SenlessHallSector == 6) change_temp = 1;
     				Sensorless[M1].CountSectorCnt++;
					SensorlessBLDC1_PhaseChange(change_temp,4000);  //5000对应37
					
				}
			}
			 		if(Sensorless[M1].CountSectorCnt>=50)
		        {
				    Sensorless[0].State++;
					Sensorless[0].FirstFlag=1;
		        }
				break;	 	 
		 case 2:	
			  if(Sensorless[0].FLAGBEMF==0)
		    {  
              if(BEMF()==1)                                         //				 if(DriveTime>=Ack_time[HallchangeValue])                  //达到每次的换向时间     
				{
					Sensorless[0].FlagSwitchStep = Sensorless[0].PWMTicksPre >>2;///2;//>>1;     //转动第一个周期延迟换向时间
					Sensorless[0].FLAGBEMF = 1;  //检测到过零事件之后，不再检测
				}	
			}
			else if(Sensorless[0].FLAGBEMF==1)   //检测到过零点
			  {
                  if(Sensorless[0].FlagSwitchStep==0)   //延时时间到
			     { 
				   Sensorless[M1].SenlessHallSector++;      //换向次数累加
				   if(Sensorless[M1].SenlessHallSector>6)
					{
						Sensorless[M1].SenlessHallSector = 1;
						Sensorless_Time[1]=0;                   //每次周期过零点标志位清0
						Sensorless_Time[2]=0;
						Sensorless_Time[3]=0;
						Sensorless_Time[4]=0;
						Sensorless_Time[5]=0;
						Sensorless_Time[6]=0;
					}	
					
					if(Sensorless[M1].SenlessHallSector == 1) change_temp = 3;
					if(Sensorless[M1].SenlessHallSector == 2) change_temp = 2;
					if(Sensorless[M1].SenlessHallSector == 3) change_temp = 6;
					if(Sensorless[M1].SenlessHallSector == 4) change_temp = 4;
					if(Sensorless[M1].SenlessHallSector == 5) change_temp = 5;
					if(Sensorless[M1].SenlessHallSector == 6) change_temp = 1;		
					Sensorless[M1].HallState = change_temp;          //当前Hall状态
                    Sensorless_ChangePwm(0);          //改变PWM的时候用                     
			  	    if(Sensorless[0].PWM_DutySet>8000) Sensorless[0].PWM_DutySet=8000;   //限制PWM最大值
//				    if(Sensorless[0].PWM_DutySet<1000) Sensorless[0].PWM_DutySet=1000;	 //限制PWM最小值				
					SensorlessBLDC1_PhaseChange(change_temp,Sensorless[0].PWM_Duty);
                    Sensorless[0].FLAGBEMF=0;        //过零点标志清0   				
					Sensorless[0].FirstFlag=2;		//调试用			
//					DriveTime=0;                    //换向计数清0
//                    if( Sensorless[0].SenlessCommPWM!=Sensorless[0].PWM_Duty) 	 //速度改变时使用
//					{
//						Ack_Flag[0]=1;
//					}
					 sensorless[i]=change_temp;    //记录每次换向
//				     i++;
//				     if(i>10000){i=0;MotorState_t=3;}		//自启动运行时间
			      }
				  else
			      {
				     Sensorless[0].FlagSwitchStep--;        //延迟
			      }			 
		    }
			  break;
		default:
			break;
	}
}
int16_t VoltBEMF1[3]={0};
int16_t VoltBEMF2[3]={0};
int16_t VoltBEMF3[3]={0};
int16_t VoltBEMF4[3]={0};
int16_t VoltBEMF5[3]={0};
int16_t Ack_Max;
int16_t Ack_Min;
int16_t Ack_Sum;

int16_t Sensorless_Time1=0;
int16_t Change_cnt=0;

//int8_t Ack_Flag[7]={0};
extern s32 OverZerotime;
unsigned long BEMF(void)      //程序在main中定时器中断回调函数中运行
{
	int cnt;
	Mul_change(0);
	switch(change_temp)//正转  645132
	{
		case 1:    //BA
			 VoltBEMF[1]=Sensorless[0].PhaseWCurrent;
		  if(Sensorless[0].PhaseWCurrent >=RiseMul*VoltVar.BUS/2&&Sensorless_Time[1]==0)
		  {
			  Fileter_cnt[1]++;    //计数
		  }
		  else if(Sensorless[0].PhaseWCurrent < RiseMul*VoltVar.BUS/2&&Sensorless_Time[1]==0)
		  {
			  if(Fileter_cnt[1]>0)
			  {
			     Fileter_cnt[1]--;
			  }
		  }
		  if(Fileter_cnt[1]==1)     //先后2次都达到母线电压的一半
		  {
			  Sensorless_Time[1]=1;
			  Fileter_cnt[1]=0;
		      RED_LED1_TOGGLE;
			  Record_time[6]=Ack_time[6];      //记录前一次换向的时间
			  Ack_time[6]=OverZerotime;       //记录此时的换向时间
			  OverZerotime=0;                //计数清0
			   Ack_Flag[1]=1;
			  return 1;
		  }
		  else
		  {
			  return 0;
		  }
		   break;
		case 2:  //CB
		   VoltBEMF[2]=Sensorless[0].PhaseUCurrent;
		  if(Sensorless[0].PhaseUCurrent >=RiseMul*VoltVar.BUS/2&&Sensorless_Time[2]==0)
		  {
			  Fileter_cnt[2]++;      //计数
		  }
		  else if(Sensorless[0].PhaseUCurrent < RiseMul*VoltVar.BUS/2&&Sensorless_Time[2]==0)
		  {
			    if(Fileter_cnt[2]>0)
			  {
			     Fileter_cnt[2]--;
			  }
//			  Fileter_cnt[2]--;
		  }
		  if(Fileter_cnt[2]==1)     //先后2次都达到母线电压的一半
		  {
			  Sensorless_Time[2]=1;
			  Fileter_cnt[2]=0;
		      RED_LED1_TOGGLE;
			  Record_time[2]=Ack_time[2];   //记录前一次换向的时间
			  Ack_time[2]=OverZerotime;        //记录此时的换向时间
			  OverZerotime=0;
			  Ack_Flag[3]=1;
			    return 1;
		  }
		    else
		  {
			  return 0;
		  }
		   break;
		case 3:  //CA
		   VoltBEMF[3]=Sensorless[0].PhaseVCurrent;
		  if(Sensorless[0].PhaseVCurrent <=DowmMul*VoltVar.BUS/2&&Sensorless_Time[3]==0)
		  {
			  Fileter_cnt[3]++;       //计数
		  }
		  else if(Sensorless[0].PhaseVCurrent > DowmMul*VoltVar.BUS/2&&Sensorless_Time[3]==0)
		  {
			    if(Fileter_cnt[3]>0)
			  {
			     Fileter_cnt[3]--;
			  }
//			  Fileter_cnt[3]--;
		  }
		   if(Fileter_cnt[3]==1)       //先后2次都达到母线电压的一半
		  {
			 Sensorless_Time[3]=1;
			    Fileter_cnt[3]=0;
		      RED_LED1_TOGGLE;
			  Record_time[1]=Ack_time[1];         //记录前一次的换向时间
			  Ack_time[1]=OverZerotime;           //记录此时的换向时间
			  OverZerotime=0;
			  Ack_Flag[2]=1;
			    return 1;
		  }
		    else
		  {
			  return 0;
		  }
		
		    dir=1;
		   break;
		case 4:  //AC
		   VoltBEMF[4]=Sensorless[0].PhaseVCurrent;
		  if(Sensorless[0].PhaseVCurrent >=RiseMul*VoltVar.BUS/2&&Sensorless_Time[4]==0)
		  {
			  Fileter_cnt[4]++;
		  }
		  else if(Sensorless[0].PhaseVCurrent < RiseMul*VoltVar.BUS/2&&Sensorless_Time[4]==0)
		  {
			    if(Fileter_cnt[4]>0)
			  {
			     Fileter_cnt[4]--;
			  }
//			  Fileter_cnt[4]--;
		  }
		   if(Fileter_cnt[4]==1)           //先后2次都达到母线电压的一半
		  {
			  Sensorless_Time[4]=1;
			  Fileter_cnt[4]=0;
		      RED_LED1_TOGGLE;
			  Record_time[4]=Ack_time[4];             //记录前一次的换向时间
			  Ack_time[4]=OverZerotime;                 //记录此时的换向时间
			  OverZerotime=0; 
			  Ack_Flag[5]=1;
			    return 1;
		  }
		    else
		  {
			  return 0;
		  }
		   break;
		case 5: //BC
		   VoltBEMF[5]=Sensorless[0].PhaseUCurrent;
		  if(Sensorless[0].PhaseUCurrent <=DowmMul*VoltVar.BUS/2&&Sensorless_Time[5]==0)
		  {
			  Fileter_cnt[5]++;
		  }
		  else if(Sensorless[0].PhaseUCurrent >DowmMul*VoltVar.BUS/2&&Sensorless_Time[5]==0)
		  {
			    if(Fileter_cnt[5]>0)
			  {
			     Fileter_cnt[5]--;
			  }
//			  Fileter_cnt[5]--;
		  }
		   if(Fileter_cnt[5]==1)            //先后2次都达到母线电压的一半
		  {
			  Sensorless_Time[5]=1;
			   Fileter_cnt[5]=0;
		      RED_LED1_TOGGLE;
			  Record_time[5]=Ack_time[5];           //记录前一次的换向时间
			  Ack_time[5]=OverZerotime;             //记录此时的换向时间
			  OverZerotime=0;
			  Ack_Flag[6]=1;
		        return 1;
		  }
		    else
		  {
			  return 0;
		  }
		   break;
		case 6:  //AB
		   VoltBEMF[6]=Sensorless[0].PhaseWCurrent;
		  if(Sensorless[0].PhaseWCurrent <= DowmMul*VoltVar.BUS/2&&Sensorless_Time[6]==0)
		  {
			  Fileter_cnt[6]++;
		  }
		  else if(Sensorless[0].PhaseWCurrent > DowmMul*VoltVar.BUS/2&&Sensorless_Time[6]==0)
		  {
			    if(Fileter_cnt[6]>0)
			  {
			     Fileter_cnt[6]--;
			  }
//			  Fileter_cnt[6]--;
		  }
		   if(Fileter_cnt[6]==1)        //先后2次都达到母线电压的一半
		  {
			  Sensorless_Time[6]=1;
			  Fileter_cnt[6]=0;
		      RED_LED1_TOGGLE;
			  Record_time[3]=Ack_time[3];           //记录前一次的换向时间
			  Ack_time[3]=OverZerotime;               //记录此时的换向时间
			  OverZerotime=0;
			  Ack_Flag[4]=1;
			  return 1;
		  }
		    else
		  {
			  return 0;
		  }
		   break;
		default:
			break;
	}
}
extern int16_t CPWM_Test;



void Mul_change(u8 Motor_NUM)   //没用到
{
	if(Sensorless[Motor_NUM].PWM_Duty >=2000)
    {
		RiseMul=1;
		DowmMul=1;	
	}
	else if(1350<=Sensorless[Motor_NUM].PWM_Duty && Sensorless[Motor_NUM].PWM_Duty< 2000)
	{
		RiseMul=0.98;
		DowmMul=0.8;
	}
	 else if(1300<=Sensorless[Motor_NUM].PWM_Duty && Sensorless[Motor_NUM].PWM_Duty< 1350)
	{
		RiseMul=1;
		DowmMul=0.9;
	}
	 else if(1255<=Sensorless[Motor_NUM].PWM_Duty && Sensorless[Motor_NUM].PWM_Duty< 1300)
	{
		RiseMul=1;
		DowmMul=1;
	}
	 else if(1200<=Sensorless[Motor_NUM].PWM_Duty && Sensorless[Motor_NUM].PWM_Duty< 1255)
	{
		RiseMul=1.05;
		DowmMul=1;
	}
	 else if(1100<=Sensorless[Motor_NUM].PWM_Duty && Sensorless[Motor_NUM].PWM_Duty< 1200)
	{
		RiseMul=1.135;
		DowmMul=1.1;
	}
	 else if(1020<=Sensorless[Motor_NUM].PWM_Duty && Sensorless[Motor_NUM].PWM_Duty< 1100)
	{
		RiseMul=1.2;
		DowmMul=1.15;
	}
	 else if(950<=Sensorless[Motor_NUM].PWM_Duty && Sensorless[Motor_NUM].PWM_Duty< 1020)
	{
		RiseMul=1.25;//1.25
		DowmMul=1.2;
	}
}
