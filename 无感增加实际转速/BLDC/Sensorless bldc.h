

#include "stm32g4xx_hal.h"
extern void SensorlessBLDC1_PhaseChange(u8 bHallState, s16 PWM_Duty);
extern void Sensorless_Start(void);
typedef struct
{
	uint16_t IdleCountCnt1;
    uint16_t  CountSectorCnt;
    uint16_t CountSectorCnt2;
    uint16_t CountSectorCnt3;
    uint8_t  SenlessHallSector;
    int16_t SenlessCommPWM;
	int16_t PWM_DutySet;
	int16_t PWM_Duty;
	int16_t    PhaseU;
    int16_t    PhaseV;
    int16_t    PhaseW;
	int16_t    Speed_Real;
    int16_t    PhaseUCurrent;
    int16_t    PhaseVCurrent;
    int16_t    PhaseWCurrent;
	uint8_t    State;
	int16_t    PWMTicks;
	int16_t    PWMTicksPre;
	int8_t    FLAGBEMF;
	uint8_t    Step;
	int16_t FlagSwitchStep;
	 int16_t Acceleration;
    int16_t Deceleration;
	u32 period;
	int16_t    PhaseUAD;
    int16_t    PhaseVAD;
    int16_t    PhaseWAD;
	int16_t    FirstFlag;
	uint8_t HallState;               //µ±Ç°Hall×´Ì¬
	 HALL_Parameters_t   Hall;
}
Sensorless_t;

extern Sensorless_t Sensorless[2];
extern u8 change_temp;
extern s16 Sensorless_ChangePwm(u8 Mortor_NUM);