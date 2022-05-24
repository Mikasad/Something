
/**
  ******************************************************************************
  * @file    mc_tasks.c
  * @author  Motor Control SDK Team, ST Microelectronics
  * @brief   This file implements tasks definition
  *
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "mc_type.h"
#include "mc_math.h"
#include "mc_config.h"
#include "motorcontrol.h"
#include "regular_conversion_manager.h"
#include "mc_interface.h"
#include "mc_tuning.h"
#include "digital_output.h"
#include "state_machine.h"
#include "pwm_common.h"

#include "mc_tasks.h"
#include "parameters_conversion.h"
#include "bsp_BDCMotor.h"
#include "function.h"
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* USER CODE BEGIN Private define */
/* Private define ------------------------------------------------------------*/
#define BLDC_HALL     //方波控制打开
#define CHARGE_BOOT_CAP_MS  10
#define CHARGE_BOOT_CAP_MS2 10
#define OFFCALIBRWAIT_MS     0
#define OFFCALIBRWAIT_MS2    0
#define STOPPERMANENCY_MS  400
#define STOPPERMANENCY_MS2 400
#define CHARGE_BOOT_CAP_TICKS  (uint16_t)((SYS_TICK_FREQUENCY * CHARGE_BOOT_CAP_MS)/ 1000)
#define CHARGE_BOOT_CAP_TICKS2 (uint16_t)((SYS_TICK_FREQUENCY * CHARGE_BOOT_CAP_MS2)/ 1000)
#define OFFCALIBRWAITTICKS     (uint16_t)((SYS_TICK_FREQUENCY * OFFCALIBRWAIT_MS)/ 1000)
#define OFFCALIBRWAITTICKS2    (uint16_t)((SYS_TICK_FREQUENCY * OFFCALIBRWAIT_MS2)/ 1000)
#define STOPPERMANENCY_TICKS   (uint16_t)((SYS_TICK_FREQUENCY * STOPPERMANENCY_MS)/ 1000)
#define STOPPERMANENCY_TICKS2  (uint16_t)((SYS_TICK_FREQUENCY * STOPPERMANENCY_MS2)/ 1000)

/* Un-Comment this macro define in order to activate the smooth
   braking action on over voltage */
/* #define  MC.SMOOTH_BRAKING_ACTION_ON_OVERVOLTAGE */

/* USER CODE END Private define */
#define VBUS_TEMP_ERR_MASK (MC_OVER_VOLT| MC_UNDER_VOLT| MC_OVER_TEMP)
#define VBUS_TEMP_ERR_MASK2 (MC_OVER_VOLT| MC_UNDER_VOLT| MC_OVER_TEMP)

/* Private variables----------------------------------------------------------*/
FOCVars_t FOCVars[NBR_OF_MOTORS];
MCI_Handle_t Mci[NBR_OF_MOTORS];
MCI_Handle_t * oMCInterface[NBR_OF_MOTORS];
MCT_Handle_t MCT[NBR_OF_MOTORS];
STM_Handle_t STM[NBR_OF_MOTORS];
SpeednTorqCtrl_Handle_t *pSTC[NBR_OF_MOTORS];
PID_Handle_t *pPIDSpeed[NBR_OF_MOTORS];
PID_Handle_t *pPIDIq[NBR_OF_MOTORS];
PID_Handle_t *pPIDId[NBR_OF_MOTORS];
EncAlign_Handle_t *pEAC[NBR_OF_MOTORS];
RDivider_Handle_t *pBusSensorM1;

RDivider_Handle_t *pBusSensorM2;
NTC_Handle_t *pTemperatureSensor[NBR_OF_MOTORS];
PWMC_Handle_t * pwmcHandle[NBR_OF_MOTORS];
DOUT_handle_t *pR_Brake[NBR_OF_MOTORS];
DOUT_handle_t *pOCPDisabling[NBR_OF_MOTORS];
PQD_MotorPowMeas_Handle_t *pMPM[NBR_OF_MOTORS];
CircleLimitation_Handle_t *pCLM[NBR_OF_MOTORS];
FW_Handle_t *pFW[NBR_OF_MOTORS];     /* only if M1 or M2 has FW */
RampExtMngr_Handle_t *pREMNG[NBR_OF_MOTORS];   /*!< Ramp manager used to modify the Iq ref
                                                 during the start-up switch over.*/
u8 First_Enter_STUDY_M1=1,First_Enter_STUDY_M2=1;
extern PID_Handle_t PIDSpeed_BLDC_M1;
extern PID_Handle_t PIDSpeed_BLDC_M2;
static volatile uint16_t hMFTaskCounterM1 = 0;
static volatile uint16_t hBootCapDelayCounterM1 = 0;
static volatile uint16_t hStopPermanencyCounterM1 = 0;
static volatile uint16_t hMFTaskCounterM2 = 0;
static volatile uint16_t hBootCapDelayCounterM2 = 0;
static volatile uint16_t hStopPermanencyCounterM2 = 0;

uint8_t bMCBootCompleted = 0;
extern MotorControlParameters_t MotorControl[MOTOR_NUM];
/* USER CODE BEGIN Private Variables */

/* USER CODE END Private Variables */

/* Private functions ---------------------------------------------------------*/
void TSK_MediumFrequencyTaskM1(void);
void FOC_Clear(uint8_t bMotor);
void FOC_InitAdditionalMethods(uint8_t bMotor);
void FOC_CalcCurrRef(uint8_t bMotor);
static uint16_t FOC_CurrControllerM1(void);
static uint16_t FOC_CurrControllerM2(void);
void TSK_SetChargeBootCapDelayM1(uint16_t hTickCount);
bool TSK_ChargeBootCapDelayHasElapsedM1(void);
void TSK_SetStopPermanencyTimeM1(uint16_t hTickCount);
bool TSK_StopPermanencyTimeHasElapsedM1(void);
void TSK_SafetyTask_PWMOFF(uint8_t motor);
void TSK_MediumFrequencyTaskM2(void);
void TSK_SetChargeBootCapDelayM2(uint16_t hTickCount);
bool TSK_ChargeBootCapDelayHasElapsedM2(void);
void TSK_SetStopPermanencyTimeM2(uint16_t SysTickCount);
bool TSK_StopPermanencyTimeHasElapsedM2(void);

#define FOC_ARRAY_LENGTH 2
static uint8_t FOC_array[FOC_ARRAY_LENGTH]={ 0, 0 };
static uint8_t FOC_array_head = 0; // Next obj to be executed
static uint8_t FOC_array_tail = 0; // Last arrived
void UI_Scheduler(void);

/* USER CODE BEGIN Private Functions */

/* USER CODE END Private Functions */
/**
  * @brief  It initializes the whole MC core according to user defined
  *         parameters.
  * @param  pMCIList pointer to the vector of MCInterface objects that will be
  *         created and initialized. The vector must have length equal to the
  *         number of motor drives.
  * @param  pMCTList pointer to the vector of MCTuning objects that will be
  *         created and initialized. The vector must have length equal to the
  *         number of motor drives.
  * @retval None
  */
__weak void MCboot( MCI_Handle_t* pMCIList[NBR_OF_MOTORS],MCT_Handle_t* pMCTList[NBR_OF_MOTORS] )
{
  /* USER CODE BEGIN MCboot 0 */

  /* USER CODE END MCboot 0 */

  /**************************************/
  /*    State machine initialization    */
  /**************************************/
  STM_Init(&STM[M1]);

  bMCBootCompleted = 0;
  pCLM[M1] = &CircleLimitationM1;
  pFW[M1] = &FW_M1; /* only if M1 has FW */

  pCLM[M2] = &CircleLimitationM2;
  pFW[M2] = &FW_M2; /* only if M2 has FW */

  /**********************************************************/
  /*    PWM and current sensing component initialization    */
  /**********************************************************/
  pwmcHandle[M1] = &PWM_Handle_M1._Super;
  R3_2_Init(&PWM_Handle_M1);
  pwmcHandle[M2] = &PWM_Handle_M2._Super;
  R3_2_Init(&PWM_Handle_M2);
  /* USER CODE BEGIN MCboot 1 */

  /* USER CODE END MCboot 1 */

  /**************************************/
  /*    Start timers synchronously      */
  /**************************************/
  startTimers();

  /******************************************************/
  /*   PID component initialization: speed regulation   */
  /******************************************************/
  PID_HandleInit(&PIDSpeedHandle_M1);
  pPIDSpeed[M1] = &PIDSpeedHandle_M1;

  /******************************************************/
  /*   Main speed sensor component initialization       */
  /******************************************************/
  pSTC[M1] = &SpeednTorqCtrlM1;
  ENC_Init (&ENCODER_M1);

  /******************************************************/
  /*   Main encoder alignment component initialization  */
  /******************************************************/
  EAC_Init(&EncAlignCtrlM1,pSTC[M1],&VirtualSpeedSensorM1,&ENCODER_M1);
  pEAC[M1] = &EncAlignCtrlM1;

  /******************************************************/
  /*   Speed & torque component initialization          */
  /******************************************************/
  STC_Init(pSTC[M1],pPIDSpeed[M1], &ENCODER_M1._Super);

  /******************************************************/
  /*   Auxiliary speed sensor component initialization  */
  /******************************************************/
  HALL_Init (&HALL_M1);

  /****************************************************/
  /*   Virtual speed sensor component initialization  */
  /****************************************************/
  VSS_Init (&VirtualSpeedSensorM1);

  /********************************************************/
  /*   PID component initialization: current regulation   */
  /********************************************************/
  PID_HandleInit(&PIDIqHandle_M1);
  PID_HandleInit(&PIDIdHandle_M1);
  pPIDIq[M1] = &PIDIqHandle_M1;
  pPIDId[M1] = &PIDIdHandle_M1;

  /********************************************************/
  /*   Bus voltage sensor component initialization        */
  /********************************************************/
  pBusSensorM1 = &RealBusVoltageSensorParamsM1;
  RVBS_Init(pBusSensorM1);

  /*************************************************/
  /*   Power measurement component initialization  */
  /*************************************************/
  pMPM[M1] = &PQD_MotorPowMeasM1;
  pMPM[M1]->pVBS = &(pBusSensorM1->_Super);
  pMPM[M1]->pFOCVars = &FOCVars[M1];

  /*******************************************************/
  /*   Temperature measurement component initialization  */
  /*******************************************************/
  NTC_Init(&TempSensorParamsM1);
  pTemperatureSensor[M1] = &TempSensorParamsM1;

  /*******************************************************/
  /*   Flux weakening component initialization           */
  /*******************************************************/
  PID_HandleInit(&PIDFluxWeakeningHandle_M1);
  FW_Init(pFW[M1],pPIDSpeed[M1],&PIDFluxWeakeningHandle_M1);

  pREMNG[M1] = &RampExtMngrHFParamsM1;
  REMNG_Init(pREMNG[M1]);

  FOC_Clear(M1);
  FOCVars[M1].bDriveInput = EXTERNAL;
  FOCVars[M1].Iqdref = STC_GetDefaultIqdref(pSTC[M1]);
  FOCVars[M1].UserIdref = STC_GetDefaultIqdref(pSTC[M1]).d;
  oMCInterface[M1] = & Mci[M1];
  MCI_Init(oMCInterface[M1], &STM[M1], pSTC[M1], &FOCVars[M1] );
  MCI_ExecSpeedRamp(oMCInterface[M1],
  STC_GetMecSpeedRefUnitDefault(pSTC[M1]),0); /*First command to STC*/
  pMCIList[M1] = oMCInterface[M1];
  MCT[M1].pPIDSpeed = pPIDSpeed[M1];
  MCT[M1].pPIDIq = pPIDIq[M1];
  MCT[M1].pPIDId = pPIDId[M1];
  MCT[M1].pPIDFluxWeakening = &PIDFluxWeakeningHandle_M1; /* only if M1 has FW */
  MCT[M1].pPWMnCurrFdbk = pwmcHandle[M1];
  MCT[M1].pRevupCtrl = MC_NULL;              /* only if M1 is not sensorless*/
  MCT[M1].pSpeedSensorMain = (SpeednPosFdbk_Handle_t *) &ENCODER_M1;
  MCT[M1].pSpeedSensorAux = (SpeednPosFdbk_Handle_t *) &HALL_M1;
  MCT[M1].pSpeedSensorVirtual = MC_NULL;
  MCT[M1].pSpeednTorqueCtrl = pSTC[M1];
  MCT[M1].pStateMachine = &STM[M1];
  MCT[M1].pTemperatureSensor = (NTC_Handle_t *) pTemperatureSensor[M1];
  MCT[M1].pBusVoltageSensor = &(pBusSensorM1->_Super);
  MCT[M1].pBrakeDigitalOutput = MC_NULL;   /* brake is defined, oBrakeM1*/
  MCT[M1].pNTCRelay = MC_NULL;             /* relay is defined, oRelayM1*/
  MCT[M1].pMPM =  (MotorPowMeas_Handle_t*)pMPM[M1];
  MCT[M1].pFW = pFW[M1];
  MCT[M1].pFF = MC_NULL;

  MCT[M1].pPosCtrl = MC_NULL;

  MCT[M1].pSCC = MC_NULL;
  MCT[M1].pOTT = MC_NULL;
  pMCTList[M1] = &MCT[M1];

  /******************************************************/
  /*   Motor 2 features initialization                  */
  /******************************************************/

  /**************************************/
  /*    State machine initialization    */
  /**************************************/
  STM_Init(&STM[M2]);

  /******************************************************/
  /*   PID component initialization: speed regulation   */
  /******************************************************/
  PID_HandleInit(&PIDSpeedHandle_M2);
  pPIDSpeed[M2] = &PIDSpeedHandle_M2;

  /***********************************************************/
  /*   Main speed  sensor initialization: speed regulation   */
  /***********************************************************/
  pSTC[M2] = &SpeednTorqCtrlM2;
  ENC_Init (&ENCODER_M2);

  /******************************************************/
  /*   Main encoder alignment component initialization  */
  /******************************************************/
  EAC_Init(&EncAlignCtrlM2,pSTC[M2],&VirtualSpeedSensorM2,&ENCODER_M2);
  pEAC[M2] = &EncAlignCtrlM2;

  /******************************************************/
  /*   Speed & torque component initialization          */
  /******************************************************/
  STC_Init(pSTC[M2],pPIDSpeed[M2], &ENCODER_M2._Super);

  /***********************************************************/
  /*   Auxiliary speed sensor component initialization       */
  /***********************************************************/
  HALL_Init (&HALL_M2);
  /****************************************************/
  /*   Virtual speed sensor component initialization  */
  /****************************************************/
  VSS_Init (&VirtualSpeedSensorM2);

  /********************************************************/
  /*   PID component initialization: current regulation   */
  /********************************************************/
  PID_HandleInit(&PIDIqHandle_M2);
  PID_HandleInit(&PIDIdHandle_M2);
  pPIDIq[M2] = &PIDIqHandle_M2;
  pPIDId[M2] = &PIDIdHandle_M2;

  /**********************************************************/
  /*   Bus voltage sensor component initialization          */
  /**********************************************************/
  pBusSensorM2 = &RealBusVoltageSensorParamsM2; /* powerboard configuration: Rdivider or Virtual*/
  RVBS_Init(pBusSensorM2);

  /*************************************************/
  /*   Power measurement component initialization  */
  /*************************************************/
  pMPM[M2] = &PQD_MotorPowMeasM2;
  pMPM[M2]->pVBS = &(pBusSensorM2->_Super);
  pMPM[M2]->pFOCVars = &FOCVars[M2];

  /*******************************************************/
  /*   Temperature measurement component initialization  */
  /*******************************************************/
  NTC_Init(&TempSensorParamsM2);
  pTemperatureSensor[M2] = &TempSensorParamsM2;

  /*************************************************/
  /*   Flux weakening component initialization     */
  /*************************************************/
  PID_HandleInit(&PIDFluxWeakeningHandle_M2);
  FW_Init(pFW[M2],pPIDSpeed[M2],&PIDFluxWeakeningHandle_M2);    /* only if M2 has FW */
  pREMNG[M2] = &RampExtMngrHFParamsM2;
  REMNG_Init(pREMNG[M2]);
  FOC_Clear(M2);
  FOCVars[M2].bDriveInput = EXTERNAL;
  FOCVars[M2].Iqdref = STC_GetDefaultIqdref(pSTC[M2]);
  FOCVars[M2].UserIdref = STC_GetDefaultIqdref(pSTC[M2]).d;
  oMCInterface[M2] = &Mci[M2];
  MCI_Init(oMCInterface[M2], &STM[M2], pSTC[M2], &FOCVars[M2] );
  MCI_ExecSpeedRamp(oMCInterface[M2],
  STC_GetMecSpeedRefUnitDefault(pSTC[M2]),0); /*First command to STC*/
  pMCIList[M2] = oMCInterface[M2];
  MCT[M2].pPIDSpeed = pPIDSpeed[M2];
  MCT[M2].pPIDIq = pPIDIq[M2];
  MCT[M2].pPIDId = pPIDId[M2];
  MCT[M2].pPIDFluxWeakening = &PIDFluxWeakeningHandle_M2; /* only if M2 has FW */
  MCT[M2].pPWMnCurrFdbk = pwmcHandle[M2];
  MCT[M2].pRevupCtrl = MC_NULL;              /* only if M2 is not sensorless*/
  MCT[M2].pSpeedSensorMain = (SpeednPosFdbk_Handle_t *) &ENCODER_M2;
  MCT[M2].pSpeedSensorAux = (SpeednPosFdbk_Handle_t *) &HALL_M2;
  MCT[M2].pSpeedSensorVirtual = MC_NULL;
  MCT[M2].pSpeednTorqueCtrl = pSTC[M2];
  MCT[M2].pStateMachine = &STM[M2];
  MCT[M2].pTemperatureSensor = (NTC_Handle_t *) pTemperatureSensor[M2];
  MCT[M2].pBusVoltageSensor = &(pBusSensorM2->_Super);
  MCT[M2].pBrakeDigitalOutput = MC_NULL;   /* brake is defined, oBrakeM2*/
  MCT[M2].pNTCRelay = MC_NULL;             /* relay is defined, oRelayM2*/
  MCT[M2].pMPM = (MotorPowMeas_Handle_t*)pMPM[M2];
  MCT[M2].pFW = pFW[M2];
  MCT[M2].pFF = MC_NULL;
  MCT[M2].pPosCtrl = MC_NULL;
  MCT[M2].pSCC = MC_NULL;
  pMCTList[M2] = &MCT[M2];

  /* USER CODE BEGIN MCboot 2 */

  /* USER CODE END MCboot 2 */

  bMCBootCompleted = 1;
}

/**
 * @brief Runs all the Tasks of the Motor Control cockpit
 *
 * This function is to be called periodically at least at the Medium Frequency task
 * rate (It is typically called on the Systick interrupt). Exact invokation rate is
 * the Speed regulator execution rate set in the Motor Contorl Workbench.
 *
 * The following tasks are executed in this order:
 *
 * - Medium Frequency Tasks of each motors
 * - Safety Task
 * - Power Factor Correction Task (if enabled)
 * - User Interface task.
 */
__weak void MC_RunMotorControlTasks(void)
{
  if ( bMCBootCompleted ) {
    /* ** Medium Frequency Tasks ** */
    MC_Scheduler();

    /* Safety task is run after Medium Frequency task so that
     * it can overcome actions they initiated if needed. */
//    TSK_SafetyTask();

    /* ** User Interface Task ** */
    UI_Scheduler();
  }
}

/**
 * @brief  Executes the Medium Frequency Task functions for each drive instance.
 *
 * It is to be clocked at the Systick frequency.
 */
__weak void MC_Scheduler(void)
{
/* USER CODE BEGIN MC_Scheduler 0 */

/* USER CODE END MC_Scheduler 0 */

  if (bMCBootCompleted == 1)
  {
    if(hMFTaskCounterM1 > 0u)
    {
      hMFTaskCounterM1--;
    }
    else
    {
      TSK_MediumFrequencyTaskM1();
      /* USER CODE BEGIN MC_Scheduler 1 */

      /* USER CODE END MC_Scheduler 1 */
      hMFTaskCounterM1 = MF_TASK_OCCURENCE_TICKS;
    }
    if(hMFTaskCounterM2 > 0u)
    {
      hMFTaskCounterM2--;
    }
    else
    {
      TSK_MediumFrequencyTaskM2();
      /* USER CODE BEGIN MC_Scheduler MediumFrequencyTask M2 */

      /* USER CODE END MC_Scheduler MediumFrequencyTask M2 */
      hMFTaskCounterM2 = MF_TASK_OCCURENCE_TICKS2;
    }
    if(hBootCapDelayCounterM1 > 0u)
    {
      hBootCapDelayCounterM1--;
    }
    if(hStopPermanencyCounterM1 > 0u)
    {
      hStopPermanencyCounterM1--;
    }
    if(hBootCapDelayCounterM2 > 0u)
    {
      hBootCapDelayCounterM2--;
    }
    if(hStopPermanencyCounterM2 > 0u)
    {
      hStopPermanencyCounterM2--;
    }
  }
  else
  {
  }
  /* USER CODE BEGIN MC_Scheduler 2 */

  /* USER CODE END MC_Scheduler 2 */
}

/**
  * @brief Executes medium frequency periodic Motor Control tasks
  *
  * This function performs some of the control duties on Motor 1 according to the
  * present state of its state machine. In particular, duties requiring a periodic
  * execution at a medium frequency rate (such as the speed controller for instance)
  * are executed here.
  */
__weak void TSK_MediumFrequencyTaskM1(void)
{
  /* USER CODE BEGIN MediumFrequencyTask M1 0 */

  /* USER CODE END MediumFrequencyTask M1 0 */

  State_t StateM1;
  int16_t wAux = 0;
	int16_t hError;

  (void) HALL_CalcAvrgMecSpeedUnit( &HALL_M1, &wAux );
  (void) ENC_CalcAvrgMecSpeedUnit( &ENCODER_M1, &wAux );
  PQD_CalcElMotorPower( pMPM[M1] );

  StateM1 = STM_GetState( &STM[M1] );

  switch ( StateM1 )
  {

  case IDLE:
//    if ( EAC_GetRestartState( &EncAlignCtrlM1 ) )
//    {
      /* The Encoder Restart State is true: the IDLE state has been entered
       * after Encoder alignment was performed. The motor can now be started. */
//      EAC_SetRestartState( &EncAlignCtrlM1,false );

      /* USER CODE BEGIN MediumFrequencyTask M1 Encoder Restart */

      /* USER CODE END MediumFrequencyTask M1 Encoder Restart */

      STM_NextState( &STM[M1], IDLE_START );
//    }
    break;

  case IDLE_START:

//    if ( EAC_IsAligned( &EncAlignCtrlM1 ) == false )
//    {
//      /* The encoder is not aligned. It needs to be and the alignment procedure will make
//       * the state machine go back to IDLE. Setting the Restart State to true ensures that
//       * the start up procedure will carry on after alignment. */
//      EAC_SetRestartState( &EncAlignCtrlM1, true );

//      STM_NextState( &STM[M1], IDLE_ALIGNMENT );
//      break;
//    }

//    R3_2_TurnOnLowSides( pwmcHandle[M1] );
    TSK_SetChargeBootCapDelayM1( CHARGE_BOOT_CAP_TICKS );
    STM_NextState( &STM[M1], CHARGE_BOOT_CAP );
    break;

  case CHARGE_BOOT_CAP:
    if ( TSK_ChargeBootCapDelayHasElapsedM1() )
    {
//      PWMC_CurrentReadingCalibr( pwmcHandle[M1], CRC_START );

      /* USER CODE BEGIN MediumFrequencyTask M1 Charge BootCap elapsed */

      /* USER CODE END MediumFrequencyTask M1 Charge BootCap elapsed */

      STM_NextState(&STM[M1],OFFSET_CALIB);
    }
    break;

  case OFFSET_CALIB:
    if ( PWMC_CurrentReadingCalibr( pwmcHandle[M1], CRC_EXEC ) )
    {
      STM_NextState( &STM[M1], CLEAR );
    }
    break;

  case CLEAR:
    ENC_Clear( &ENCODER_M1 );
    HALL_Clear( &HALL_M1 );

    if ( STM_NextState( &STM[M1], START ) == true )
    {
      FOC_Clear( M1 );

      R3_2_SwitchOnPWM( pwmcHandle[M1] );
    }
    break;

  case IDLE_ALIGNMENT:
//    R3_2_TurnOnLowSides( pwmcHandle[M1] );
    TSK_SetChargeBootCapDelayM1( CHARGE_BOOT_CAP_TICKS );
    STM_NextState( &STM[M1], ALIGN_CHARGE_BOOT_CAP );
    break;

   case ALIGN_CHARGE_BOOT_CAP:
      if ( TSK_ChargeBootCapDelayHasElapsedM1() )
        {
            PWMC_CurrentReadingCalibr( pwmcHandle[M1], CRC_START );

            /* USER CODE BEGIN MediumFrequencyTask M1 Align Charge BootCap elapsed */
            for(u8 i=0; i<8; i++) 
            {
                MotorControl[M1].Current.PhaseAOffset  += (ADC2->JDR1>>3) ;//修改
                MotorControl[M1].Current.PhaseBOffset  += (ADC1->JDR2>>3) ;
                MotorControl[M1].Current.PhaseCOffset  += (ADC1->JDR3>>3) ;
            }
            /* USER CODE END MediumFrequencyTask M1 Align Charge BootCap elapsed */

            STM_NextState(&STM[M1],ALIGN_OFFSET_CALIB);
        }
        break;

  case ALIGN_OFFSET_CALIB:
    if ( PWMC_CurrentReadingCalibr( pwmcHandle[M1], CRC_EXEC ) )
    {
      STM_NextState( &STM[M1], ALIGN_CLEAR );
    }
    break;

  case ALIGN_CLEAR:
    FOCVars[M1].bDriveInput = EXTERNAL;
    STC_SetSpeedSensor( pSTC[M1], &VirtualSpeedSensorM1._Super );
    EAC_StartAlignment( &EncAlignCtrlM1 );

    if ( STM_NextState( &STM[M1], ALIGNMENT ) == true )
    {
      FOC_Clear( M1 );
      R3_2_SwitchOnPWM( pwmcHandle[M1] );
    }
    break;

  case START:
    {
        STM_NextState( &STM[M1], START_RUN ); /* only for sensored*/
    }
    break;

  case ALIGNMENT:
    if ( !EAC_Exec( &EncAlignCtrlM1 ) )
    {
      qd_t IqdRef;
      IqdRef.q = 0;
      IqdRef.d = STC_CalcTorqueReference( pSTC[M1] );
      FOCVars[M1].Iqdref = IqdRef;
    }
    else
    {
      R3_2_SwitchOffPWM( pwmcHandle[M1] );
      STC_SetControlMode( pSTC[M1], STC_SPEED_MODE );
      STC_SetSpeedSensor( pSTC[M1], &ENCODER_M1._Super );

      /* USER CODE BEGIN MediumFrequencyTask M1 EndOfEncAlignment */

      /* USER CODE END MediumFrequencyTask M1 EndOfEncAlignment */

      STM_NextState( &STM[M1], ANY_STOP );
    }
    break;

  case START_RUN:
    {
        /* USER CODE BEGIN MediumFrequencyTask M1 1 */
        PIDSpeed_BLDC_M1.wIntegralTerm = 0; 
		   	MotorControl[M1].Hall.HallStateValue = MotorControl[M1].Hall.HallState ;
		  	TIM1->CNT = 0;
			  TIM8->CNT = 0;
        /* USER CODE END MediumFrequencyTask M1 1 */
        FOC_InitAdditionalMethods(M1);
        FOC_CalcCurrRef( M1 );
        STM_NextState( &STM[M1], RUN );
    }
        STC_ForceSpeedReferenceToCurrentSpeed( pSTC[M1] ); /* Init the reference speed to current speed */
        MCI_ExecBufferedCommands( oMCInterface[M1] ); /* Exec the speed ramp after changing of the speed sensor */

    break;

  case RUN:
    /* USER CODE BEGIN MediumFrequencyTask M1 2 */

    /* USER CODE END MediumFrequencyTask M1 2 */
#ifndef BLDC_HALL

        MCI_ExecBufferedCommands( oMCInterface[M1] );
        FOC_CalcCurrRef( M1 );

#else
     Ramp_Speed(M1);
     MotorControl[M1].Speed_Real = GetMotorSpeed(M1);
     hError = MotorControl[M1].Speed_Ref - MotorControl[M1].Speed_Real ;
		 MotorControl[M1].Current.ADCValue_Ref = PI_Controller( &PIDSpeed_BLDC_M1, ( int32_t )hError );

    /* USER CODE BEGIN MediumFrequencyTask M1 3 */
#endif
    /* USER CODE END MediumFrequencyTask M1 3 */
    break;

  case ANY_STOP:
    R3_2_SwitchOffPWM( pwmcHandle[M1] );
    FOC_Clear( M1 );
    MPM_Clear( (MotorPowMeas_Handle_t*) pMPM[M1] );
    TSK_SetStopPermanencyTimeM1( STOPPERMANENCY_TICKS );

    /* USER CODE BEGIN MediumFrequencyTask M1 4 */

    /* USER CODE END MediumFrequencyTask M1 4 */

    STM_NextState( &STM[M1], STOP );
    break;

  case STOP:
    if ( TSK_StopPermanencyTimeHasElapsedM1() )
    {
      STM_NextState( &STM[M1], STOP_IDLE );
    }
    break;

  case STOP_IDLE:
    /* USER CODE BEGIN MediumFrequencyTask M1 5 */

    /* USER CODE END MediumFrequencyTask M1 5 */
    STM_NextState( &STM[M1], IDLE );
    break;
   case HALL_STUDY:
			
	    	MotorControl[M1].Motor_Start_Stop = DISABLE;
        if(First_Enter_STUDY_M1 == 1)
				{
						TIM1->CCR1 = 0;
						TIM1->CCR2 = 0;
						TIM1->CCR3 = 0;
						for(u16 i = 0; i < 10000; i++) {}
						First_Enter_STUDY_M1 = 0;
				}
        HallStudyHandle0();

        if(HALL_Study[0].CommuntionState == 3)
        {
            HALL_Study[0].CommuntionState = 0;

            HALL_Study[0].StudySectorCnt = 0;

            TIM1->CCR1 = 0;
            TIM1->CCR2 = 0;
            TIM1->CCR3 = 0;

            if ( ((wGlobal_Flags & M1_HALL_ERR) == 0))
            {
                STM_NextState( &STM[M1], IDLE );
            }
            else
            {
                STM_NextState( &STM[M1], ANY_STOP );
            }
          First_Enter_STUDY_M1 = 1;
        }

        break;
  default:
    break;
  }

  /* USER CODE BEGIN MediumFrequencyTask M1 6 */

  /* USER CODE END MediumFrequencyTask M1 6 */
}

/**
  * @brief  It re-initializes the current and voltage variables. Moreover
  *         it clears qd currents PI controllers, voltage sensor and SpeednTorque
  *         controller. It must be called before each motor restart.
  *         It does not clear speed sensor.
  * @param  bMotor related motor it can be M1 or M2
  * @retval none
  */
__weak void FOC_Clear(uint8_t bMotor)
{
  /* USER CODE BEGIN FOC_Clear 0 */

  /* USER CODE END FOC_Clear 0 */
  ab_t NULL_ab = {(int16_t)0, (int16_t)0};
  qd_t NULL_qd = {(int16_t)0, (int16_t)0};
  alphabeta_t NULL_alphabeta = {(int16_t)0, (int16_t)0};

  FOCVars[bMotor].Iab = NULL_ab;
  FOCVars[bMotor].Ialphabeta = NULL_alphabeta;
  FOCVars[bMotor].Iqd = NULL_qd;
  FOCVars[bMotor].Iqdref = NULL_qd;
  FOCVars[bMotor].hTeref = (int16_t)0;
  FOCVars[bMotor].Vqd = NULL_qd;
  FOCVars[bMotor].Valphabeta = NULL_alphabeta;
  FOCVars[bMotor].hElAngle = (int16_t)0;

  PID_SetIntegralTerm(pPIDIq[bMotor], (int32_t)0);
  PID_SetIntegralTerm(pPIDId[bMotor], (int32_t)0);

  STC_Clear(pSTC[bMotor]);

  PWMC_SwitchOffPWM(pwmcHandle[bMotor]);

  if (pFW[bMotor])
  {
    FW_Clear(pFW[bMotor]);
  }
  /* USER CODE BEGIN FOC_Clear 1 */

  /* USER CODE END FOC_Clear 1 */
}

/**
  * @brief  Use this method to initialize additional methods (if any) in
  *         START_TO_RUN state
  * @param  bMotor related motor it can be M1 or M2
  * @retval none
  */
__weak void FOC_InitAdditionalMethods(uint8_t bMotor)
{
  /* USER CODE BEGIN FOC_InitAdditionalMethods 0 */

  /* USER CODE END FOC_InitAdditionalMethods 0 */
}

/**
  * @brief  It computes the new values of Iqdref (current references on qd
  *         reference frame) based on the required electrical torque information
  *         provided by oTSC object (internally clocked).
  *         If implemented in the derived class it executes flux weakening and/or
  *         MTPA algorithm(s). It must be called with the periodicity specified
  *         in oTSC parameters
  * @param  bMotor related motor it can be M1 or M2
  * @retval none
  */
__weak void FOC_CalcCurrRef(uint8_t bMotor)
{
  qd_t IqdTmp;

  /* USER CODE BEGIN FOC_CalcCurrRef 0 */

  /* USER CODE END FOC_CalcCurrRef 0 */
  if(FOCVars[bMotor].bDriveInput == INTERNAL)
  {
    FOCVars[bMotor].hTeref = STC_CalcTorqueReference(pSTC[bMotor]);
    FOCVars[bMotor].Iqdref.q = FOCVars[bMotor].hTeref;

    if (pFW[bMotor])
    {
      IqdTmp.q = FOCVars[bMotor].Iqdref.q;
      IqdTmp.d = FOCVars[bMotor].UserIdref;
      FOCVars[bMotor].Iqdref = FW_CalcCurrRef(pFW[bMotor],IqdTmp);
    }
  }
  /* USER CODE BEGIN FOC_CalcCurrRef 1 */

  /* USER CODE END FOC_CalcCurrRef 1 */
}

/**
  * @brief  It set a counter intended to be used for counting the delay required
  *         for drivers boot capacitors charging of motor 1
  * @param  hTickCount number of ticks to be counted
  * @retval void
  */
__weak void TSK_SetChargeBootCapDelayM1(uint16_t hTickCount)
{
   hBootCapDelayCounterM1 = hTickCount;
}

/**
  * @brief  Use this function to know whether the time required to charge boot
  *         capacitors of motor 1 has elapsed
  * @param  none
  * @retval bool true if time has elapsed, false otherwise
  */
__weak bool TSK_ChargeBootCapDelayHasElapsedM1(void)
{
  bool retVal = false;
  if (hBootCapDelayCounterM1 == 0)
  {
    retVal = true;
  }
  return (retVal);
}

/**
  * @brief  It set a counter intended to be used for counting the permanency
  *         time in STOP state of motor 1
  * @param  hTickCount number of ticks to be counted
  * @retval void
  */
__weak void TSK_SetStopPermanencyTimeM1(uint16_t hTickCount)
{
  hStopPermanencyCounterM1 = hTickCount;
}

/**
  * @brief  Use this function to know whether the permanency time in STOP state
  *         of motor 1 has elapsed
  * @param  none
  * @retval bool true if time is elapsed, false otherwise
  */
__weak bool TSK_StopPermanencyTimeHasElapsedM1(void)
{
  bool retVal = false;
  if (hStopPermanencyCounterM1 == 0)
  {
    retVal = true;
  }
  return (retVal);
}

#if defined (CCMRAM_ENABLED)
#if defined (__ICCARM__)
#pragma location = ".ccmram"
#elif defined (__CC_ARM)
__attribute__((section (".ccmram")))
#endif
#endif
/**
  * @brief Executes medium frequency periodic Motor Control tasks
  *
  * This function performs some of the control duties on Motor 2 according to the
  * present state of its state machine. In particular, duties requiring a periodic
  * execution at a medium frequency rate (such as the speed controller for instance)
  * are executed here.
  */
__weak void TSK_MediumFrequencyTaskM2(void)
{
  /* USER CODE BEGIN MediumFrequencyTask M2 0 */

  /* USER CODE END MediumFrequencyTask M2 0 */
  State_t StateM2;
  int16_t wAux = 0;
  int16_t    hError;
  (void) HALL_CalcAvrgMecSpeedUnit( &HALL_M2, &wAux );
  (void) ENC_CalcAvrgMecSpeedUnit( &ENCODER_M2, &wAux );
  PQD_CalcElMotorPower( pMPM[M2] );

  StateM2 = STM_GetState( &STM[M2] );

  switch ( StateM2 )
  {

  case IDLE:
//    if ( EAC_GetRestartState( &EncAlignCtrlM2 ) )
//    {
//      /* The Encoder Restart State is true: the IDLE state has been entered
//       * after Encoder alignment was performed. The motor can now be started. */
//      EAC_SetRestartState( &EncAlignCtrlM2,false );

      /* USER CODE BEGIN MediumFrequencyTask M2 Encoder Restart */

      /* USER CODE END MediumFrequencyTask M2 Encoder Restart */

      STM_NextState( &STM[M2], IDLE_START );
//    }
    break;

  case IDLE_START:
 /*  only for encoder*/
    if ( EAC_IsAligned ( &EncAlignCtrlM2 ) == false )
    {
      /* The encoder is not aligned. It needs to be and the alignment procedure will make
       * the state machine go back to IDLE. Setting the Restart State to true ensures that
       * the start up procedure will carry on after alignment. */
      EAC_SetRestartState( &EncAlignCtrlM2, true );

      STM_NextState( &STM[M2], IDLE_ALIGNMENT );
      break;
    }

//    R3_2_TurnOnLowSides( pwmcHandle[M2] );
    TSK_SetChargeBootCapDelayM2( CHARGE_BOOT_CAP_TICKS2 );
    STM_NextState( &STM[M2], CHARGE_BOOT_CAP );
    break;

  case CHARGE_BOOT_CAP:
    if ( TSK_ChargeBootCapDelayHasElapsedM2() )
    {
//      PWMC_CurrentReadingCalibr( pwmcHandle[M2], CRC_START );

      /* USER CODE BEGIN MediumFrequencyTask M2 Charge BootCap elapsed */
      STM_NextState( &STM[M2], START_RUN );
      /* USER CODE END MediumFrequencyTask M2 Charge BootCap elapsed */

//      STM_NextState( &STM[M2], OFFSET_CALIB );
    }
    break;

  case OFFSET_CALIB:
    if ( PWMC_CurrentReadingCalibr( pwmcHandle[M2], CRC_EXEC ) )
    {
      STM_NextState( &STM[M2], CLEAR );
    }
    break;

  case CLEAR:
    ENC_Clear( &ENCODER_M2 );
    HALL_Clear( &HALL_M2 );

    if ( STM_NextState( &STM[M2], START ) == true )
    {
      FOC_Clear(M2);
      R3_2_SwitchOnPWM( pwmcHandle[M2] );
    }
    break;

 /*  only for encoder*/
  case IDLE_ALIGNMENT:
//    R3_2_TurnOnLowSides( pwmcHandle[M2] );
    TSK_SetChargeBootCapDelayM2( CHARGE_BOOT_CAP_TICKS2 );
    STM_NextState( &STM[M2], ALIGN_CHARGE_BOOT_CAP );
    break;

  case ALIGN_CHARGE_BOOT_CAP:
    if ( TSK_ChargeBootCapDelayHasElapsedM2() )
        {
            PWMC_CurrentReadingCalibr( pwmcHandle[M2], CRC_START );

            /* USER CODE BEGIN MediumFrequencyTask M2 Align Charge BootCap elapsed */
            for(u8 i=0; i<8; i++)
            {
                MotorControl[M2].Current.PhaseAOffset  += (ADC1->JDR1>>3) ;
                MotorControl[M2].Current.PhaseBOffset  += (ADC2->JDR2>>3) ;
                MotorControl[M2].Current.PhaseCOffset  += (ADC2->JDR3>>3) ;
            }
            /* USER CODE END MediumFrequencyTask M2 Align Charge BootCap elapsed */

            STM_NextState( &STM[M2], ALIGN_OFFSET_CALIB );
        }
        break;

  case ALIGN_OFFSET_CALIB:
    if ( PWMC_CurrentReadingCalibr( pwmcHandle[M2], CRC_EXEC ) )
    {
      STM_NextState( &STM[M2], ALIGN_CLEAR );
    }
    break;

  case ALIGN_CLEAR:
    FOCVars[M2].bDriveInput = EXTERNAL;
    STC_SetSpeedSensor( pSTC[M2], &VirtualSpeedSensorM2._Super );
    EAC_StartAlignment( &EncAlignCtrlM2 );

    if ( STM_NextState( &STM[M2], ALIGNMENT ) == true )
    {
      FOC_Clear( M2 );
      R3_2_SwitchOnPWM( pwmcHandle[M2] );
    }
    break;

  case START:
    {
      STM_NextState( &STM[M2], START_RUN );
    }
    break;

 /*  only for encoder*/
  case ALIGNMENT:
    if ( !EAC_Exec( &EncAlignCtrlM2 ) )
    {
      qd_t IqdRef;
      IqdRef.q = 0;
      IqdRef.d = STC_CalcTorqueReference( pSTC[M2] );
      FOCVars[M2].Iqdref = IqdRef;
    }
    else
    {
      R3_2_SwitchOffPWM( pwmcHandle[M2] );
      STC_SetControlMode( pSTC[M2], STC_SPEED_MODE );
      STC_SetSpeedSensor( pSTC[M2], &ENCODER_M2._Super );

      /* USER CODE BEGIN MediumFrequencyTask M2 EndOfEncAlignment */

      /* USER CODE END MediumFrequencyTask M2 EndOfEncAlignment */

      STM_NextState( &STM[M2], ANY_STOP );
    }
    break;

  case START_RUN:
    {
      /* USER CODE BEGIN MediumFrequencyTask M2 1 */
     PIDSpeed_BLDC_M2.wIntegralTerm = 0;
		 MotorControl[M2].Hall.HallStateValue = MotorControl[M2].Hall.HallState ;
		 TIM1->CNT = 0;
		 TIM8->CNT = 0;
      /* USER CODE END MediumFrequencyTask M2 1 */

      FOC_InitAdditionalMethods( M2 );
      FOC_CalcCurrRef( M2 );

      STM_NextState(&STM[M2], RUN);
    }
    STC_ForceSpeedReferenceToCurrentSpeed(pSTC[M2]); /* Init the reference speed to current speed */
    MCI_ExecBufferedCommands(oMCInterface[M2]); /* Exec the speed ramp after changing of the speed sensor */
    break;

  case RUN:
    /* USER CODE BEGIN MediumFrequencyTask M2 2 */

    /* USER CODE END MediumFrequencyTask M2 2 */

#ifndef BLDC_HALL 

        MCI_ExecBufferedCommands( oMCInterface[M2] );
        FOC_CalcCurrRef( M2 );

#else

    /* USER CODE BEGIN MediumFrequencyTask M2 3 */
        Ramp_Speed(M2);
        GetMotorSpeed(M2);
        hError = MotorControl[M2].Speed_Ref - MotorControl[M2].Speed_Real ;
        MotorControl[M2].Current.ADCValue_Ref = PI_Controller( &PIDSpeed_BLDC_M2, ( int32_t )hError );
#endif
    /* USER CODE END MediumFrequencyTask M2 3 */
    break;

  case ANY_STOP:
    R3_2_SwitchOffPWM( pwmcHandle[M2] );
    FOC_Clear( M2 );
    MPM_Clear( (MotorPowMeas_Handle_t*) pMPM[M2] );
    TSK_SetStopPermanencyTimeM2( STOPPERMANENCY_TICKS2 );

    /* USER CODE BEGIN MediumFrequencyTask M2 4 */

    /* USER CODE END MediumFrequencyTask M2 4 */

    STM_NextState(&STM[M2], STOP);
    break;

  case STOP:
    if ( TSK_StopPermanencyTimeHasElapsedM2() )
    {
      STM_NextState( &STM[M2], STOP_IDLE );
    }
    break;

  case STOP_IDLE:
    /* USER CODE BEGIN MediumFrequencyTask M2 5 */

    /* USER CODE END MediumFrequencyTask M2 5 */
    STM_NextState( &STM[M2], IDLE );
    break;
   case HALL_STUDY:

//				TIM8->CCR1 = 0;
//				TIM8->CCR2 = 0;
//				TIM8->CCR3 = 0;
		    MotorControl[M2].Motor_Start_Stop = DISABLE;
		
	    	if(First_Enter_STUDY_M2 == 1)
				{
						TIM8->CCR1 = 0;
						TIM8->CCR2 = 0;
						TIM8->CCR3 = 0;
						for(u16 i = 0; i < 10000; i++) {}
						First_Enter_STUDY_M2 = 0;
				}
					
        HallStudyHandle1();

        if(HALL_Study[1].CommuntionState == 3)
        {
            HALL_Study[1].CommuntionState = 0;

            HALL_Study[1].StudySectorCnt = 0;

            TIM8->CCR1 = 0;
            TIM8->CCR2 = 0;
            TIM8->CCR3 = 0;

            if ( ((wGlobal_Flags & M2_HALL_ERR) == 0))
            {
                STM_NextState( &STM[M2], IDLE );
            }
            else
            {
                STM_NextState( &STM[M2], ANY_STOP );
            }
           First_Enter_STUDY_M2 = 1;
        }

        break;
  default:
    break;
  }

  /* USER CODE BEGIN MediumFrequencyTask M2 6 */

  /* USER CODE END MediumFrequencyTask M2 6 */
}

/**
  * @brief  It set a counter intended to be used for counting the delay required
  *         for drivers boot capacitors charging of motor 2
  * @param  hTickCount number of ticks to be counted
  * @retval void
  */
__weak void TSK_SetChargeBootCapDelayM2(uint16_t hTickCount)
{
   hBootCapDelayCounterM2 = hTickCount;
}

/**
  * @brief  Use this function to know whether the time required to charge boot
  *         capacitors of motor 2 has elapsed
  * @param  none
  * @retval bool true if time has elapsed, false otherwise
  */
__weak bool TSK_ChargeBootCapDelayHasElapsedM2(void)
{
  bool retVal = false;
  if (hBootCapDelayCounterM2 == 0)
  {
    retVal = true;
  }
  return (retVal);
}

/**
  * @brief  It set a counter intended to be used for counting the permanency
  *         time in STOP state of motor 2
  * @param  hTickCount number of ticks to be counted
  * @retval void
  */
__weak void TSK_SetStopPermanencyTimeM2(uint16_t hTickCount)
{
  hStopPermanencyCounterM2 = hTickCount;
}

/**
  * @brief  Use this function to know whether the permanency time in STOP state
  *         of motor 2 has elapsed
  * @param  none
  * @retval bool true if time is elapsed, false otherwise
  */
__weak bool TSK_StopPermanencyTimeHasElapsedM2(void)
{
  bool retVal = false;
  if (hStopPermanencyCounterM2 == 0)
  {
    retVal = true;
  }
  return (retVal);
}

#if defined (CCMRAM)
#if defined (__ICCARM__)
#pragma location = ".ccmram"
#elif defined (__CC_ARM)
__attribute__((section (".ccmram")))
#endif
#endif
/**
  * @brief  Executes the Motor Control duties that require a high frequency rate and a precise timing
  *
  *  This is mainly the FOC current control loop. It is executed depending on the state of the Motor Control
  * subsystem (see the state machine(s)).
  *
  * @retval Number of the  motor instance which FOC loop was executed.
  */
__weak uint8_t TSK_HighFrequencyTask(void)
{
  /* USER CODE BEGIN HighFrequencyTask 0 */

  /* USER CODE END HighFrequencyTask 0 */

 uint8_t bMotorNbr = 0;
    uint16_t hFOCreturn;

    bMotorNbr = FOC_array[FOC_array_head];
    if (bMotorNbr == M1)
    {

#ifndef BLDC_HALL
        HAL_NVIC_DisableIRQ(EXTI15_10_IRQn);
        ENC_CalcAngle (&ENCODER_M1);
        /* USER CODE BEGIN HighFrequencyTask DUALDRIVE_1 */

        /* USER CODE END HighFrequencyTask DUALDRIVE_1 */
        hFOCreturn = FOC_CurrControllerM1();
        /* USER CODE BEGIN HighFrequencyTask DUALDRIVE_2 */
#else
        HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

#endif
        /* USER CODE END HighFrequencyTask DUALDRIVE_2 */
    }
    else /* bMotorNbr != M1 */
    {
#ifndef BLDC_HALL
        HAL_NVIC_DisableIRQ(EXTI9_5_IRQn);
        ENC_CalcAngle (&ENCODER_M2);
        /* USER CODE BEGIN HighFrequencyTask DUALDRIVE_3 */

        /* USER CODE END HighFrequencyTask DUALDRIVE_3 */
        hFOCreturn = FOC_CurrControllerM2();
        /* USER CODE BEGIN HighFrequencyTask DUALDRIVE_4 */
#else
        HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);
#endif

        /* USER CODE END HighFrequencyTask DUALDRIVE_4 */
    }
    if(hFOCreturn == MC_FOC_DURATION)
    {
//    STM_FaultProcessing(&STM[bMotorNbr], MC_FOC_DURATION, 0);
    }
    else
    {
        if (bMotorNbr == M1)
        {
        }
        else // bMotorNbr != M1
        {

        }
    }
    FOC_array_head++;
    if (FOC_array_head == FOC_ARRAY_LENGTH)
    {
        FOC_array_head = 0;
    }
    /* USER CODE BEGIN HighFrequencyTask 1 */

    /* USER CODE END HighFrequencyTask 1 */

    return bMotorNbr;
}

#if defined (CCMRAM)
#if defined (__ICCARM__)
#pragma location = ".ccmram"
#elif defined (__CC_ARM) || defined(__GNUC__)
__attribute__((section (".ccmram")))
#endif
#endif
/**
  * @brief It executes the core of FOC drive that is the controllers for Iqd
  *        currents regulation. Reference frame transformations are carried out
  *        accordingly to the active speed sensor. It must be called periodically
  *        when new motor currents have been converted
  * @param this related object of class CFOC.
  * @retval int16_t It returns MC_NO_FAULTS if the FOC has been ended before
  *         next PWM Update event, MC_FOC_DURATION otherwise
  */
inline uint16_t FOC_CurrControllerM1(void)
{
  qd_t Iqd, Vqd;
  ab_t Iab;
  alphabeta_t Ialphabeta, Valphabeta;

  int16_t hElAngle;
  uint16_t hCodeError;
  SpeednPosFdbk_Handle_t *speedHandle;

  speedHandle = STC_GetSpeedSensor(pSTC[M1]);
  hElAngle = SPD_GetElAngle(speedHandle);
  PWMC_GetPhaseCurrents(pwmcHandle[M1], &Iab);
  RCM_ReadOngoingConv();
  RCM_ExecNextConv();
  Ialphabeta = MCM_Clarke(Iab);
  Iqd = MCM_Park(Ialphabeta, hElAngle);
  Vqd.q = PI_Controller(pPIDIq[M1],
            (int32_t)(FOCVars[M1].Iqdref.q) - Iqd.q);

  Vqd.d = PI_Controller(pPIDId[M1],
            (int32_t)(FOCVars[M1].Iqdref.d) - Iqd.d);

  Vqd = Circle_Limitation(pCLM[M1], Vqd);
  hElAngle += SPD_GetInstElSpeedDpp(speedHandle)*REV_PARK_ANGLE_COMPENSATION_FACTOR;
  Valphabeta = MCM_Rev_Park(Vqd, hElAngle);
  hCodeError = PWMC_SetPhaseVoltage(pwmcHandle[M1], Valphabeta);
  FOCVars[M1].Vqd = Vqd;
  FOCVars[M1].Iab = Iab;
  FOCVars[M1].Ialphabeta = Ialphabeta;
  FOCVars[M1].Iqd = Iqd;
  FOCVars[M1].Valphabeta = Valphabeta;
  FOCVars[M1].hElAngle = hElAngle;
  FW_DataProcess(pFW[M1], Vqd);
  return(hCodeError);
}

#if defined (CCMRAM)
#if defined (__ICCARM__)
#pragma location = ".ccmram"
#elif defined (__CC_ARM) || defined(__GNUC__)
__attribute__((section (".ccmram")))
#endif
#endif
/**
  * @brief It executes the core of FOC drive that is the controllers for Iqd
  *        currents regulation of motor 2. Reference frame transformations are carried out
  *        accordingly to the active speed sensor. It must be called periodically
  *        when new motor currents have been converted
  * @param this related object of class CFOC.
  * @retval int16_t It returns MC_NO_FAULTS if the FOC has been ended before
  *         next PWM Update event, MC_FOC_DURATION otherwise
  */
inline uint16_t FOC_CurrControllerM2(void)
{
  ab_t Iab;
  alphabeta_t Ialphabeta, Valphabeta;
  qd_t Iqd, Vqd;

  int16_t hElAngle;
  uint16_t hCodeError;
  SpeednPosFdbk_Handle_t *speedHandle;

  speedHandle = STC_GetSpeedSensor(pSTC[M2]);
  hElAngle = SPD_GetElAngle(speedHandle);
  PWMC_GetPhaseCurrents(pwmcHandle[M2], &Iab);
  RCM_ReadOngoingConv();
  RCM_ExecNextConv();
  Ialphabeta = MCM_Clarke(Iab);
  Iqd = MCM_Park(Ialphabeta, hElAngle);
  Vqd.q = PI_Controller(pPIDIq[M2],
            (int32_t)(FOCVars[M2].Iqdref.q) - Iqd.q);

  Vqd.d = PI_Controller(pPIDId[M2],
            (int32_t)(FOCVars[M2].Iqdref.d) - Iqd.d);

  Vqd = Circle_Limitation(pCLM[M2], Vqd);
  hElAngle += SPD_GetInstElSpeedDpp(speedHandle)*REV_PARK_ANGLE_COMPENSATION_FACTOR2;
  Valphabeta = MCM_Rev_Park(Vqd, hElAngle);
  hCodeError = PWMC_SetPhaseVoltage(pwmcHandle[M2], Valphabeta);
  FOCVars[M2].Vqd = Vqd;
  FOCVars[M2].Iab = Iab;
  FOCVars[M2].Ialphabeta = Ialphabeta;
  FOCVars[M2].Iqd = Iqd;
  FOCVars[M2].Valphabeta = Valphabeta;
  FOCVars[M2].hElAngle = hElAngle;
  FW_DataProcess(pFW[M2], Vqd);
  return(hCodeError);
}

/**
  * @brief  Executes safety checks (e.g. bus voltage and temperature) for all drive instances.
  *
  * Faults flags are updated here.
  */
__weak void TSK_SafetyTask(void)
{
  /* USER CODE BEGIN TSK_SafetyTask 0 */

  /* USER CODE END TSK_SafetyTask 0 */
  if (bMCBootCompleted == 1)
  {
    TSK_SafetyTask_PWMOFF(M1);
    /* Second drive */
    TSK_SafetyTask_PWMOFF(M2);
    /* User conversion execution */
//    RCM_ExecUserConv ();
  /* USER CODE BEGIN TSK_SafetyTask 1 */

  /* USER CODE END TSK_SafetyTask 1 */
  }
}

/**
  * @brief  Safety task implementation if  MC.ON_OVER_VOLTAGE == TURN_OFF_PWM
  * @param  bMotor Motor reference number defined
  *         \link Motors_reference_number here \endlink
  * @retval None
  */
__weak void TSK_SafetyTask_PWMOFF(uint8_t bMotor)
{
  /* USER CODE BEGIN TSK_SafetyTask_PWMOFF 0 */

  /* USER CODE END TSK_SafetyTask_PWMOFF 0 */

  uint16_t CodeReturn = MC_NO_ERROR;
  uint16_t errMask[NBR_OF_MOTORS] = {VBUS_TEMP_ERR_MASK, VBUS_TEMP_ERR_MASK2};

  CodeReturn |= errMask[bMotor] & NTC_CalcAvTemp(pTemperatureSensor[bMotor]); /* check for fault if FW protection is activated. It returns MC_OVER_TEMP or MC_NO_ERROR */
  CodeReturn |= PWMC_CheckOverCurrent(pwmcHandle[bMotor]);                    /* check for fault. It return MC_BREAK_IN or MC_NO_FAULTS
                                                                                 (for STM32F30x can return MC_OVER_VOLT in case of HW Overvoltage) */
  if(bMotor == M1)
  {
    CodeReturn |=  errMask[bMotor] &RVBS_CalcAvVbus(pBusSensorM1);
  }
  if(bMotor == M2)
  {
    CodeReturn |=  errMask[bMotor] & RVBS_CalcAvVbus(pBusSensorM2);
  }

  STM_FaultProcessing(&STM[bMotor], CodeReturn, ~CodeReturn); /* Update the STM according error code */
  switch (STM_GetState(&STM[bMotor])) /* Acts on PWM outputs in case of faults */
  {
  case FAULT_NOW:
    /* reset Encoder state */
    if (pEAC[bMotor] != MC_NULL)
    {
      EAC_SetRestartState( pEAC[bMotor], false );
    }
    PWMC_SwitchOffPWM(pwmcHandle[bMotor]);
    FOC_Clear(bMotor);
    MPM_Clear((MotorPowMeas_Handle_t*)pMPM[bMotor]);
    /* USER CODE BEGIN TSK_SafetyTask_PWMOFF 1 */

    /* USER CODE END TSK_SafetyTask_PWMOFF 1 */
    break;
  case FAULT_OVER:
    PWMC_SwitchOffPWM(pwmcHandle[bMotor]);
	/* USER CODE BEGIN TSK_SafetyTask_PWMOFF 2 */

    /* USER CODE END TSK_SafetyTask_PWMOFF 2 */
    break;
  default:
    break;
  }
  /* USER CODE BEGIN TSK_SafetyTask_PWMOFF 3 */

  /* USER CODE END TSK_SafetyTask_PWMOFF 3 */
}

#if defined (CCMRAM_ENABLED)
#if defined (__ICCARM__)
#pragma location = ".ccmram"
#elif defined (__CC_ARM)
__attribute__((section (".ccmram")))
#endif
#endif
/**
  * @brief Reserves FOC execution on ADC ISR half a PWM period in advance
  *
  *  This function is called by TIMx_UP_IRQHandler in case of dual MC and
  * it allows to reserve half PWM period in advance the FOC execution on
  * ADC ISR
  * @param  pDrive Pointer on the FOC Array
  */
__weak void TSK_DualDriveFIFOUpdate(uint8_t Motor)
{
  FOC_array[FOC_array_tail] = Motor;
  FOC_array_tail++;
  if (FOC_array_tail == FOC_ARRAY_LENGTH)
  {
    FOC_array_tail = 0;
  }
}

/**
  * @brief  This function returns the reference of the MCInterface relative to
  *         the selected drive.
  * @param  bMotor Motor reference number defined
  *         \link Motors_reference_number here \endlink
  * @retval MCI_Handle_t * Reference to MCInterface relative to the selected drive.
  *         Note: it can be MC_NULL if MCInterface of selected drive is not
  *         allocated.
  */
__weak MCI_Handle_t * GetMCI(uint8_t bMotor)
{
  MCI_Handle_t * retVal = MC_NULL;
  if (bMotor < NBR_OF_MOTORS)
  {
    retVal = oMCInterface[bMotor];
  }
  return retVal;
}

/**
  * @brief  This function returns the reference of the MCTuning relative to
  *         the selected drive.
  * @param  bMotor Motor reference number defined
  *         \link Motors_reference_number here \endlink
  * @retval MCT_Handle_t motor control tuning handler for the selected drive.
  *         Note: it can be MC_NULL if MCInterface of selected drive is not
  *         allocated.
  */
__weak MCT_Handle_t* GetMCT(uint8_t bMotor)
{
  MCT_Handle_t* retVal = MC_NULL;
  if (bMotor < NBR_OF_MOTORS)
  {
    retVal = &MCT[bMotor];
  }
  return retVal;
}

/**
  * @brief  Puts the Motor Control subsystem in in safety conditions on a Hard Fault
  *
  *  This function is to be executed when a general hardware failure has been detected
  * by the microcontroller and is used to put the system in safety condition.
  */
__weak void TSK_HardwareFaultTask(void)
{
  /* USER CODE BEGIN TSK_HardwareFaultTask 0 */

  /* USER CODE END TSK_HardwareFaultTask 0 */

  R3_2_SwitchOffPWM(pwmcHandle[M1]);
  STM_FaultProcessing(&STM[M1], MC_SW_ERROR, 0);
  R3_2_SwitchOffPWM(pwmcHandle[M2]);
  STM_FaultProcessing(&STM[M2], MC_SW_ERROR, 0);
  /* USER CODE BEGIN TSK_HardwareFaultTask 1 */

  /* USER CODE END TSK_HardwareFaultTask 1 */
}
 /**
  * @brief  Locks GPIO pins used for Motor Control to prevent accidental reconfiguration
  */
__weak void mc_lock_pins (void)
{
//LL_GPIO_LockPin(M2_OCP_GPIO_Port, M2_OCP_Pin);
LL_GPIO_LockPin(M2_PWM_UL_GPIO_Port, M2_PWM_UL_Pin);
LL_GPIO_LockPin(M2_PWM_VL_GPIO_Port, M2_PWM_VL_Pin);
LL_GPIO_LockPin(M2_PWM_WL_GPIO_Port, M2_PWM_WL_Pin);
LL_GPIO_LockPin(M2_PWM_UH_GPIO_Port, M2_PWM_UH_Pin);
LL_GPIO_LockPin(M2_PWM_VH_GPIO_Port, M2_PWM_VH_Pin);
LL_GPIO_LockPin(M2_PWM_WH_GPIO_Port, M2_PWM_WH_Pin);
//LL_GPIO_LockPin(M2_BUS_VOLTAGE_GPIO_Port, M2_BUS_VOLTAGE_Pin);
//LL_GPIO_LockPin(M2_CURR_AMPL_W_GPIO_Port, M2_CURR_AMPL_W_Pin);
//LL_GPIO_LockPin(M1_BUS_VOLTAGE_GPIO_Port, M1_BUS_VOLTAGE_Pin);
//LL_GPIO_LockPin(M1_CURR_AMPL_W_GPIO_Port, M1_CURR_AMPL_W_Pin);
//LL_GPIO_LockPin(M1_TEMPERATURE_GPIO_Port, M1_TEMPERATURE_Pin);
//LL_GPIO_LockPin(M2_TEMPERATURE_GPIO_Port, M2_TEMPERATURE_Pin);
//LL_GPIO_LockPin(M2_CURR_AMPL_U_GPIO_Port, M2_CURR_AMPL_U_Pin);
//LL_GPIO_LockPin(M2_CURR_AMPL_V_GPIO_Port, M2_CURR_AMPL_V_Pin);
//LL_GPIO_LockPin(M1_CURR_AMPL_U_GPIO_Port, M1_CURR_AMPL_U_Pin);
//LL_GPIO_LockPin(M1_CURR_AMPL_V_GPIO_Port, M1_CURR_AMPL_V_Pin);
LL_GPIO_LockPin(M2_HALL_H3_GPIO_Port, M2_HALL_H3_Pin);
LL_GPIO_LockPin(M2_HALL_H1_GPIO_Port, M2_HALL_H1_Pin);
LL_GPIO_LockPin(M2_HALL_H2_GPIO_Port, M2_HALL_H2_Pin);
////LL_GPIO_LockPin(M2_ENCODER_A_GPIO_Port, M2_ENCODER_A_Pin);
////LL_GPIO_LockPin(M2_ENCODER_B_GPIO_Port, M2_ENCODER_B_Pin);
LL_GPIO_LockPin(M1_HALL_H3_GPIO_Port, M1_HALL_H3_Pin);
LL_GPIO_LockPin(M1_HALL_H1_GPIO_Port, M1_HALL_H1_Pin);
LL_GPIO_LockPin(M1_HALL_H2_GPIO_Port, M1_HALL_H2_Pin);
//LL_GPIO_LockPin(M1_ENCODER_A_GPIO_Port, M1_ENCODER_A_Pin);
//LL_GPIO_LockPin(M1_ENCODER_B_GPIO_Port, M1_ENCODER_B_Pin);
LL_GPIO_LockPin(M1_PWM_UH_GPIO_Port, M1_PWM_UH_Pin);
LL_GPIO_LockPin(M1_PWM_VH_GPIO_Port, M1_PWM_VH_Pin);
//LL_GPIO_LockPin(M1_OCP_GPIO_Port, M1_OCP_Pin);
LL_GPIO_LockPin(M1_PWM_VL_GPIO_Port, M1_PWM_VL_Pin);
LL_GPIO_LockPin(M1_PWM_WH_GPIO_Port, M1_PWM_WH_Pin);
LL_GPIO_LockPin(M1_PWM_WL_GPIO_Port, M1_PWM_WL_Pin);
LL_GPIO_LockPin(M1_PWM_UL_GPIO_Port, M1_PWM_UL_Pin);
}

/* USER CODE BEGIN mc_task 0 */

/* USER CODE END mc_task 0 */

/******************* (C) COPYRIGHT 2019 STMicroelectronics *****END OF FILE****/
