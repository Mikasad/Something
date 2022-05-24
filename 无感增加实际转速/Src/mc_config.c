/**
  ******************************************************************************
  * @file    mc_config.c
  * @author  Motor Control SDK Team, ST Microelectronics
  * @brief   Motor Control Subsystem components configuration and handler structures.
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
#include "main.h"
#include "mc_type.h"
#include "parameters_conversion.h"
#include "mc_parameters.h"
#include "mc_config.h"

/* USER CODE BEGIN Additional include */

/* USER CODE END Additional include */

#define OFFCALIBRWAIT_MS     0
#define OFFCALIBRWAIT_MS2    0
#include "pqd_motor_power_measurement.h"
/* USER CODE BEGIN Additional define */

/* USER CODE END Additional define */

PQD_MotorPowMeas_Handle_t PQD_MotorPowMeasM1 =
{
  .wConvFact = PQD_CONVERSION_FACTOR
};
PQD_MotorPowMeas_Handle_t *pPQD_MotorPowMeasM1 = &PQD_MotorPowMeasM1;

PQD_MotorPowMeas_Handle_t PQD_MotorPowMeasM2=
{
  .wConvFact = PQD_CONVERSION_FACTOR2
};
PQD_MotorPowMeas_Handle_t *pPQD_MotorPowMeasM2 = &PQD_MotorPowMeasM2;

/**
  * @brief  PI / PID Speed loop parameters Motor 1
  */
PID_Handle_t PIDSpeedHandle_M1 =
{
  .hDefKpGain          = (int16_t)PID_SPEED_KP_DEFAULT,
  .hDefKiGain          = (int16_t)PID_SPEED_KI_DEFAULT,
  .wUpperIntegralLimit = (int32_t)IQMAX * (int32_t)SP_KIDIV,
  .wLowerIntegralLimit = -(int32_t)IQMAX * (int32_t)SP_KIDIV,
  .hUpperOutputLimit       = (int16_t)IQMAX,
  .hLowerOutputLimit       = -(int16_t)IQMAX,
  .hKpDivisor          = (uint16_t)SP_KPDIV,
  .hKiDivisor          = (uint16_t)SP_KIDIV,
  .hKpDivisorPOW2      = (uint16_t)SP_KPDIV_LOG,
  .hKiDivisorPOW2      = (uint16_t)SP_KIDIV_LOG,
  .hDefKdGain           = 0x0000U,
  .hKdDivisor           = 0x0000U,
  .hKdDivisorPOW2       = 0x0000U,
};


PID_Handle_t PIDSpeed_BLDC_M1 =
{
  .hDefKpGain          = (int16_t)100,
  .hDefKiGain          = (int16_t)3,
  .wUpperIntegralLimit = (int32_t)IQMAX_M1 * (int32_t)SP_KIDIV,
  .wLowerIntegralLimit = -(int32_t)IQMAX_M1 * (int32_t)SP_KIDIV,
  .hUpperOutputLimit       = (int16_t)IQMAX_M1,
  .hLowerOutputLimit       = -(int16_t)IQMAX_M1,
  .hKpDivisor          = (uint16_t)SP_KPDIV,
  .hKiDivisor          = (uint16_t)SP_KIDIV,
  .hKpDivisorPOW2      = (uint16_t)SP_KPDIV_LOG,
  .hKiDivisorPOW2      = (uint16_t)SP_KIDIV_LOG,
  .hDefKdGain           = 0x0000U,
  .hKdDivisor           = 0x0000U,
  .hKdDivisorPOW2       = 0x0000U,
};

PID_Handle_t PIDSpeed_BLDC_M2 =
{
  .hDefKpGain          = (int16_t)100,
  .hDefKiGain          = (int16_t)8,
  .wUpperIntegralLimit = (int32_t)IQMAX_M2 * (int32_t)SP_KIDIV,
  .wLowerIntegralLimit = -(int32_t)IQMAX_M2 * (int32_t)SP_KIDIV,
  .hUpperOutputLimit       = (int16_t)IQMAX_M2,
  .hLowerOutputLimit       = -(int16_t)IQMAX_M2,
  .hKpDivisor          = (uint16_t)SP_KPDIV,
  .hKiDivisor          = (uint16_t)SP_KIDIV,
  .hKpDivisorPOW2      = (uint16_t)SP_KPDIV_LOG,
  .hKiDivisorPOW2      = (uint16_t)SP_KIDIV_LOG,
  .hDefKdGain           = 0x0000U,
  .hKdDivisor           = 0x0000U,
  .hKdDivisorPOW2       = 0x0000U,
};

PID_Handle_t PIDIq_BLDC_M1 =
{
  .hDefKpGain          = (int16_t)100,
  .hDefKiGain          = (int16_t)2,
  .wUpperIntegralLimit = (int32_t)PWM_M1 * TF_KIDIV,
  .wLowerIntegralLimit = (int32_t)-PWM_M1 * TF_KIDIV,
  .hUpperOutputLimit       = PWM_M1,
  .hLowerOutputLimit       = -PWM_M1,
  .hKpDivisor          = (uint16_t)TF_KPDIV,
  .hKiDivisor          = (uint16_t)TF_KIDIV,
  .hKpDivisorPOW2      = (uint16_t)TF_KPDIV_LOG,
  .hKiDivisorPOW2      = (uint16_t)TF_KIDIV_LOG,
  .hDefKdGain           = 0x0000U,
  .hKdDivisor           = 0x0000U,
  .hKdDivisorPOW2       = 0x0000U,
};

PID_Handle_t PIDIq_BLDC_M2 =
{
  .hDefKpGain          = (int16_t)100,
  .hDefKiGain          = (int16_t)2,
  .wUpperIntegralLimit = (int32_t)PWM_M2 * TF_KIDIV,
  .wLowerIntegralLimit = (int32_t)-PWM_M2 * TF_KIDIV,
  .hUpperOutputLimit       = PWM_M2,
  .hLowerOutputLimit       = -PWM_M2,
  .hKpDivisor          = (uint16_t)TF_KPDIV,
  .hKiDivisor          = (uint16_t)TF_KIDIV,
  .hKpDivisorPOW2      = (uint16_t)TF_KPDIV_LOG,
  .hKiDivisorPOW2      = (uint16_t)TF_KIDIV_LOG,
  .hDefKdGain           = 0x0000U,
  .hKdDivisor           = 0x0000U,
  .hKdDivisorPOW2       = 0x0000U,
};
/**
  * @brief  PI / PID Iq loop parameters Motor 1
  */
PID_Handle_t PIDIqHandle_M1 =
{
  .hDefKpGain          = (int16_t)PID_TORQUE_KP_DEFAULT,
  .hDefKiGain          = (int16_t)PID_TORQUE_KI_DEFAULT,
  .wUpperIntegralLimit = (int32_t)INT16_MAX * TF_KIDIV,
  .wLowerIntegralLimit = (int32_t)-INT16_MAX * TF_KIDIV,
  .hUpperOutputLimit       = INT16_MAX,
  .hLowerOutputLimit       = -INT16_MAX,
  .hKpDivisor          = (uint16_t)TF_KPDIV,
  .hKiDivisor          = (uint16_t)TF_KIDIV,
  .hKpDivisorPOW2      = (uint16_t)TF_KPDIV_LOG,
  .hKiDivisorPOW2      = (uint16_t)TF_KIDIV_LOG,
  .hDefKdGain           = 0x0000U,
  .hKdDivisor           = 0x0000U,
  .hKdDivisorPOW2       = 0x0000U,
};

/**
  * @brief  PI / PID Id loop parameters Motor 1
  */
PID_Handle_t PIDIdHandle_M1 =
{
  .hDefKpGain          = (int16_t)PID_FLUX_KP_DEFAULT,
  .hDefKiGain          = (int16_t)PID_FLUX_KI_DEFAULT,
  .wUpperIntegralLimit = (int32_t)INT16_MAX * TF_KIDIV,
  .wLowerIntegralLimit = (int32_t)-INT16_MAX * TF_KIDIV,
  .hUpperOutputLimit       = INT16_MAX,
  .hLowerOutputLimit       = -INT16_MAX,
  .hKpDivisor          = (uint16_t)TF_KPDIV,
  .hKiDivisor          = (uint16_t)TF_KIDIV,
  .hKpDivisorPOW2      = (uint16_t)TF_KPDIV_LOG,
  .hKiDivisorPOW2      = (uint16_t)TF_KIDIV_LOG,
  .hDefKdGain           = 0x0000U,
  .hKdDivisor           = 0x0000U,
  .hKdDivisorPOW2       = 0x0000U,
};

/**
  * @brief  FluxWeakeningCtrl component parameters Motor 1
  */
FW_Handle_t FW_M1 =
{
  .hMaxModule             = MAX_MODULE,
  .hDefaultFW_V_Ref       = (int16_t)FW_VOLTAGE_REF,
  .hDemagCurrent          = ID_DEMAG,
  .wNominalSqCurr         = ((int32_t)NOMINAL_CURRENT*(int32_t)NOMINAL_CURRENT),
  .hVqdLowPassFilterBW    = M1_VQD_SW_FILTER_BW_FACTOR,
  .hVqdLowPassFilterBWLOG = M1_VQD_SW_FILTER_BW_FACTOR_LOG
};

/**
  * @brief  PI Flux Weakening control parameters Motor 1
  */
PID_Handle_t PIDFluxWeakeningHandle_M1 =
{
  .hDefKpGain          = (int16_t)FW_KP_GAIN,
  .hDefKiGain          = (int16_t)FW_KI_GAIN,
  .wUpperIntegralLimit = 0,
  .wLowerIntegralLimit = (int32_t)(-NOMINAL_CURRENT) * (int32_t)FW_KIDIV,
  .hUpperOutputLimit       = 0,
  .hLowerOutputLimit       = -INT16_MAX,
  .hKpDivisor          = (uint16_t)FW_KPDIV,
  .hKiDivisor          = (uint16_t)FW_KIDIV,
  .hKpDivisorPOW2      = (uint16_t)FW_KPDIV_LOG,
  .hKiDivisorPOW2      = (uint16_t)FW_KIDIV_LOG,
  .hDefKdGain           = 0x0000U,
  .hKdDivisor           = 0x0000U,
  .hKdDivisorPOW2       = 0x0000U,
};

/**
  * @brief  FluxWeakeningCtrl component parameters Motor 2
  */
FW_Handle_t FW_M2 =
{
  .hMaxModule             = MAX_MODULE2,
  .hDefaultFW_V_Ref       = (int16_t)FW_VOLTAGE_REF2,
  .hDemagCurrent          = ID_DEMAG2,
  .wNominalSqCurr         = ((int32_t)NOMINAL_CURRENT2*(int32_t)NOMINAL_CURRENT2),
  .hVqdLowPassFilterBW    = M2_VQD_SW_FILTER_BW_FACTOR,
  .hVqdLowPassFilterBWLOG = M2_VQD_SW_FILTER_BW_FACTOR_LOG
};

/**
  * @brief  PI Flux Weakening control parameters Motor 2
  */
PID_Handle_t PIDFluxWeakeningHandle_M2 =
{
  .hDefKpGain          = (int16_t)FW_KP_GAIN2,
  .hDefKiGain          = (int16_t)FW_KI_GAIN2,
  .wUpperIntegralLimit = 0,
  .wLowerIntegralLimit = (int32_t)(-NOMINAL_CURRENT2) * (int32_t)FW_KIDIV2,
  .hUpperOutputLimit       = 0,
  .hLowerOutputLimit       = -INT16_MAX,
  .hKpDivisor          = (uint16_t)FW_KPDIV2,
  .hKiDivisor          = (uint16_t)FW_KIDIV2,
  .hKpDivisorPOW2      = (uint16_t)FW_KPDIV_LOG2,
  .hKiDivisorPOW2      = (uint16_t)FW_KIDIV_LOG2,
  .hDefKdGain           = 0x0000U,
  .hKdDivisor           = 0x0000U,
  .hKdDivisorPOW2       = 0x0000U,
};

/**
  * @brief  SpeednTorque Controller parameters Motor 1
  */
SpeednTorqCtrl_Handle_t SpeednTorqCtrlM1 =
{
  .STCFrequencyHz =           		MEDIUM_FREQUENCY_TASK_RATE,
  .MaxAppPositiveMecSpeedUnit =	(uint16_t)(MAX_APPLICATION_SPEED_UNIT),
  .MinAppPositiveMecSpeedUnit =	(uint16_t)(MIN_APPLICATION_SPEED_UNIT),
  .MaxAppNegativeMecSpeedUnit =	(int16_t)(-MIN_APPLICATION_SPEED_UNIT),
  .MinAppNegativeMecSpeedUnit =	(int16_t)(-MAX_APPLICATION_SPEED_UNIT),
  .MaxPositiveTorque =				(int16_t)NOMINAL_CURRENT,
  .MinNegativeTorque =				-(int16_t)NOMINAL_CURRENT,
  .ModeDefault =					DEFAULT_CONTROL_MODE,
  .MecSpeedRefUnitDefault =		(int16_t)(DEFAULT_TARGET_SPEED_UNIT),
  .TorqueRefDefault =				(int16_t)DEFAULT_TORQUE_COMPONENT,
  .IdrefDefault =					(int16_t)DEFAULT_FLUX_COMPONENT,
};
PWMC_R3_2_Handle_t PWM_Handle_M1 =
{
  {
    .pFctGetPhaseCurrents              = &R3_2_GetPhaseCurrents,
    .pFctSwitchOffPwm                  = &R3_2_SwitchOffPWM,
    .pFctSwitchOnPwm                   = &R3_2_SwitchOnPWM,
    .pFctCurrReadingCalib              = &R3_2_CurrentReadingPolarization,
    .pFctTurnOnLowSides                = &R3_2_TurnOnLowSides,
    .pFctIsOverCurrentOccurred         = &R3_2_IsOverCurrentOccurred,
    .pFctOCPSetReferenceVoltage        = MC_NULL,
    .pFctRLDetectionModeEnable         = &R3_2_RLDetectionModeEnable,
    .pFctRLDetectionModeDisable        = &R3_2_RLDetectionModeDisable,
    .pFctRLDetectionModeSetDuty        = &R3_2_RLDetectionModeSetDuty,

    .hT_Sqrt3 = (PWM_PERIOD_CYCLES*SQRT3FACTOR)/16384u,
    .Sector = 0,
    .CntPhA = 0,
    .CntPhB = 0,
    .CntPhC = 0,
    .SWerror = 0,
    .TurnOnLowSidesAction = false,
    .OffCalibrWaitTimeCounter = 0,
    .Motor = M1,
    .RLDetectionMode = false,
    .Ia = 0,
    .Ib = 0,
    .Ic = 0,
    .DTTest = 0,
    .PWMperiod          = PWM_PERIOD_CYCLES,
    .OffCalibrWaitTicks = (uint16_t)((SYS_TICK_FREQUENCY * OFFCALIBRWAIT_MS)/ 1000),
    .DTCompCnt          = DTCOMPCNT,
    .Ton                 = TON,
    .Toff                = TOFF
  },
  .PhaseAOffset = 0,
  .PhaseBOffset = 0,
  .PhaseCOffset = 0,
  .Half_PWMPeriod = PWM_PERIOD_CYCLES/2u,
  .OverCurrentFlag = false,
  .OverVoltageFlag = false,
  .BrakeActionLock = false,
  .pParams_str = &R3_2_ParamsM1
};

PWMC_R3_2_Handle_t PWM_Handle_M2 =
{
  {
    .pFctGetPhaseCurrents              = &R3_2_GetPhaseCurrents,
    .pFctSwitchOffPwm                  = &R3_2_SwitchOffPWM,
    .pFctSwitchOnPwm                   = &R3_2_SwitchOnPWM,
    .pFctCurrReadingCalib              = &R3_2_CurrentReadingPolarization,
    .pFctTurnOnLowSides                = &R3_2_TurnOnLowSides,
    .pFctIsOverCurrentOccurred         = &R3_2_IsOverCurrentOccurred,
    .pFctOCPSetReferenceVoltage        = MC_NULL,
    .pFctRLDetectionModeEnable         = &R3_2_RLDetectionModeEnable,
    .pFctRLDetectionModeDisable        = &R3_2_RLDetectionModeDisable,
    .pFctRLDetectionModeSetDuty        = &R3_2_RLDetectionModeSetDuty,

    .hT_Sqrt3 = (PWM_PERIOD_CYCLES2*SQRT3FACTOR)/16384u,
    .Sector = 0,
    .CntPhA = 0,
    .CntPhB = 0,
    .CntPhC = 0,
    .SWerror = 0,
    .TurnOnLowSidesAction = false,
    .OffCalibrWaitTimeCounter = 0,
    .Motor = M2,
    .RLDetectionMode = false,
    .Ia = 0,
    .Ib = 0,
    .Ic = 0,
    .DTTest = 0,
    .PWMperiod          = PWM_PERIOD_CYCLES2,
    .OffCalibrWaitTicks = (uint16_t)((SYS_TICK_FREQUENCY * OFFCALIBRWAIT_MS2)/ 1000),
    .DTCompCnt          = DTCOMPCNT2,
    .Ton                 = TON2,
    .Toff                = TOFF2
  },
  .PhaseAOffset = 0,
  .PhaseBOffset = 0,
  .PhaseCOffset = 0,
  .Half_PWMPeriod = PWM_PERIOD_CYCLES2/2u,
  .OverCurrentFlag = false,
  .OverVoltageFlag = false,
  .BrakeActionLock = false,
  .pParams_str = &R3_2_ParamsM2
};
/**
  * @brief  PI / PID Speed loop parameters Motor 2
  */
PID_Handle_t PIDSpeedHandle_M2 =
{
  .hDefKpGain          = (int16_t)PID_SPEED_KP_DEFAULT2,
  .hDefKiGain          = (int16_t)PID_SPEED_KI_DEFAULT2,
  .wUpperIntegralLimit = (int32_t)IQMAX2 * (int32_t)SP_KIDIV2,
  .wLowerIntegralLimit = -(int32_t)IQMAX2 * (int32_t)SP_KIDIV2,
  .hUpperOutputLimit       = (int16_t)IQMAX2,
  .hLowerOutputLimit       = -(int16_t)IQMAX2,
  .hKpDivisor          = (uint16_t)SP_KPDIV2,
  .hKiDivisor          = (uint16_t)SP_KIDIV2,
  .hKpDivisorPOW2      = (uint16_t)SP_KPDIV_LOG2,
  .hKiDivisorPOW2      = (uint16_t)SP_KIDIV_LOG2,
  .hDefKdGain           = 0x0000U,
  .hKdDivisor           = 0x0000U,
  .hKdDivisorPOW2       = 0x0000U,
};

/**
  * @brief  PI / PID Iq loop parameters Motor 2
  */
PID_Handle_t PIDIqHandle_M2 =
{
  .hDefKpGain          = (int16_t)PID_TORQUE_KP_DEFAULT2,
  .hDefKiGain          = (int16_t)PID_TORQUE_KI_DEFAULT2,
  .wUpperIntegralLimit = (int32_t)INT16_MAX * TF_KIDIV2,
  .wLowerIntegralLimit = (int32_t)-INT16_MAX * TF_KIDIV2,
  .hUpperOutputLimit       = INT16_MAX,
  .hLowerOutputLimit       = -INT16_MAX,
  .hKpDivisor          = (uint16_t)TF_KPDIV2,
  .hKiDivisor          = (uint16_t)TF_KIDIV2,
  .hKpDivisorPOW2      = (uint16_t)TF_KPDIV_LOG2,
  .hKiDivisorPOW2      = (uint16_t)TF_KIDIV_LOG2,
  .hDefKdGain           = 0x0000U,
  .hKdDivisor           = 0x0000U,
  .hKdDivisorPOW2       = 0x0000U,
};

/**
  * @brief  PI / PID Id loop parameters Motor 2
  */
PID_Handle_t PIDIdHandle_M2 =
{
  .hDefKpGain          = (int16_t)PID_FLUX_KP_DEFAULT2,
  .hDefKiGain          = (int16_t)PID_FLUX_KI_DEFAULT2,
  .wUpperIntegralLimit = (int32_t)INT16_MAX * TF_KIDIV2,
  .wLowerIntegralLimit = (int32_t)-INT16_MAX * TF_KIDIV2,
  .hUpperOutputLimit       = INT16_MAX,
  .hLowerOutputLimit       = -INT16_MAX,
  .hKpDivisor          = (uint16_t)TF_KPDIV2,
  .hKiDivisor          = (uint16_t)TF_KIDIV2,
  .hKpDivisorPOW2      = (uint16_t)TF_KPDIV_LOG2,
  .hKiDivisorPOW2      = (uint16_t)TF_KIDIV_LOG2,
  .hDefKdGain           = 0x0000U,
  .hKdDivisor           = 0x0000U,
  .hKdDivisorPOW2       = 0x0000U,
};

/**
  * @brief  SpeedNPosition sensor parameters Motor 2
  */
VirtualSpeedSensor_Handle_t VirtualSpeedSensorM2 =
{
  ._Super = {
    .bElToMecRatio                     =	POLE_PAIR_NUM2,
    .hMaxReliableMecSpeedUnit          =	(uint16_t)(1.15*MAX_APPLICATION_SPEED_UNIT2),
    .hMinReliableMecSpeedUnit          =	(uint16_t)(MIN_APPLICATION_SPEED_UNIT2),
    .bMaximumSpeedErrorsNumber         =	MEAS_ERRORS_BEFORE_FAULTS2,
    .hMaxReliableMecAccelUnitP         =	65535,
    .hMeasurementFrequency             =	TF_REGULATION_RATE_SCALED2,
    .DPPConvFactor                     =  DPP_CONV_FACTOR2,
    },
  .hSpeedSamplingFreqHz =	MEDIUM_FREQUENCY_TASK_RATE2,
  .hTransitionSteps     =	(int16_t)(TF_REGULATION_RATE2 * TRANSITION_DURATION2/ 1000.0),
};

/**
  * @brief  SpeednTorque Controller parameters Motor 2
  */
SpeednTorqCtrl_Handle_t SpeednTorqCtrlM2 =
{
  .STCFrequencyHz =           		MEDIUM_FREQUENCY_TASK_RATE2,
  .MaxAppPositiveMecSpeedUnit =	(uint16_t)(MAX_APPLICATION_SPEED_UNIT2),
  .MinAppPositiveMecSpeedUnit =	(uint16_t)(MIN_APPLICATION_SPEED_UNIT2),
  .MaxAppNegativeMecSpeedUnit =	(int16_t)(-MIN_APPLICATION_SPEED_UNIT2),
  .MinAppNegativeMecSpeedUnit =	(int16_t)(-MAX_APPLICATION_SPEED_UNIT2),
  .MaxPositiveTorque =				(int16_t)NOMINAL_CURRENT2,
  .MinNegativeTorque =				-(int16_t)NOMINAL_CURRENT2,
  .ModeDefault =					DEFAULT_CONTROL_MODE2,
  .MecSpeedRefUnitDefault =		(int16_t)(DEFAULT_TARGET_SPEED_UNIT2),
  .TorqueRefDefault =				(int16_t)DEFAULT_TORQUE_COMPONENT2,
  .IdrefDefault =					(int16_t)DEFAULT_FLUX_COMPONENT2,
};
/**
  * @brief  SpeedNPosition sensor parameters Motor 1 - Base Class
  */
VirtualSpeedSensor_Handle_t VirtualSpeedSensorM1 =
{

  ._Super = {
    .bElToMecRatio                     =	POLE_PAIR_NUM,
    .hMaxReliableMecSpeedUnit          =	(uint16_t)(1.15*MAX_APPLICATION_SPEED_UNIT),
    .hMinReliableMecSpeedUnit          =	(uint16_t)(MIN_APPLICATION_SPEED_UNIT),
    .bMaximumSpeedErrorsNumber         =	MEAS_ERRORS_BEFORE_FAULTS,
    .hMaxReliableMecAccelUnitP         =	65535,
    .hMeasurementFrequency             =	TF_REGULATION_RATE_SCALED,
    .DPPConvFactor                     =  DPP_CONV_FACTOR,
    },
  .hSpeedSamplingFreqHz =	MEDIUM_FREQUENCY_TASK_RATE,
  .hTransitionSteps     =	(int16_t)(TF_REGULATION_RATE * TRANSITION_DURATION/ 1000.0),

};

/**
  * @brief  SpeedNPosition sensor parameters Motor 1 - Encoder
  */
ENCODER_Handle_t ENCODER_M1 =
{
  ._Super = {
    .bElToMecRatio                     =	POLE_PAIR_NUM,
    .hMaxReliableMecSpeedUnit          =	(uint16_t)(1.15*MAX_APPLICATION_SPEED_UNIT),
    .hMinReliableMecSpeedUnit          =	(uint16_t)(MIN_APPLICATION_SPEED_UNIT),
    .bMaximumSpeedErrorsNumber         =	MEAS_ERRORS_BEFORE_FAULTS,
    .hMaxReliableMecAccelUnitP         =	65535,
    .hMeasurementFrequency             =	TF_REGULATION_RATE_SCALED,
    .DPPConvFactor                     =  DPP_CONV_FACTOR,
  },
  .PulseNumber           =	M1_ENCODER_PPR*4,
  .RevertSignal           =	(FunctionalState)ENC_INVERT_SPEED,
  .SpeedSamplingFreqHz   =	MEDIUM_FREQUENCY_TASK_RATE,
  .SpeedBufferSize       =	ENC_AVERAGING_FIFO_DEPTH,
  .TIMx                  =	TIM2,
  .ICx_Filter            =  M1_ENC_IC_FILTER,

};

/**
  * @brief  Encoder Alignment Controller parameters Motor 1
  */
EncAlign_Handle_t EncAlignCtrlM1 =
{
  .hEACFrequencyHz =	MEDIUM_FREQUENCY_TASK_RATE,
  .hFinalTorque    =	FINAL_I_ALIGNMENT,
  .hElAngle        =	ALIGNMENT_ANGLE_S16,
  .hDurationms     =	ALIGNMENT_DURATION,
  .bElToMecRatio   =	POLE_PAIR_NUM,
};

/**
  * @brief  SpeedNPosition sensor parameters Motor 2 - Encoder
  */
ENCODER_Handle_t ENCODER_M2 =
{
  ._Super = {
    .bElToMecRatio                     =	POLE_PAIR_NUM2,
    .hMaxReliableMecSpeedUnit          =	(uint16_t)(1.15*MAX_APPLICATION_SPEED_UNIT2),
    .hMinReliableMecSpeedUnit          =	(uint16_t)(MIN_APPLICATION_SPEED_UNIT2),
    .bMaximumSpeedErrorsNumber         =	MEAS_ERRORS_BEFORE_FAULTS2,
    .hMaxReliableMecAccelUnitP         =	65535,
    .hMeasurementFrequency             =	TF_REGULATION_RATE_SCALED2,
    .DPPConvFactor                     =  DPP_CONV_FACTOR,
  },
  .PulseNumber           =	M2_ENCODER_PPR*4,
  .RevertSignal           =	(FunctionalState)ENC_INVERT_SPEED2,
  .SpeedSamplingFreqHz   =	 MEDIUM_FREQUENCY_TASK_RATE2,
  .SpeedBufferSize       =	ENC_AVERAGING_FIFO_DEPTH2,
  .TIMx                  =	TIM4,
  .ICx_Filter            =  M2_ENC_IC_FILTER,
};

/**
  * @brief  Encoder Alignment Controller parameters Motor 2
  */
EncAlign_Handle_t EncAlignCtrlM2 =
{
  .hEACFrequencyHz =	MEDIUM_FREQUENCY_TASK_RATE2,
  .hFinalTorque    =	FINAL_I_ALIGNMENT2,
  .hElAngle        =	ALIGNMENT_ANGLE_S162,
  .hDurationms     =	ALIGNMENT_DURATION2,
  .bElToMecRatio   =	POLE_PAIR_NUM2,
};

/**
  * @brief  SpeedNPosition sensor parameters Motor 2 - HALL
  */
HALL_Handle_t HALL_M2 =
{
  ._Super = {
    .bElToMecRatio                     =	POLE_PAIR_NUM2,
    .hMaxReliableMecSpeedUnit          =	(uint16_t)(1.15*MAX_APPLICATION_SPEED_UNIT2),
    .hMinReliableMecSpeedUnit          =	(uint16_t)(MIN_APPLICATION_SPEED_UNIT2),
    .bMaximumSpeedErrorsNumber         =	MEAS_ERRORS_BEFORE_FAULTS2,
    .hMaxReliableMecAccelUnitP         =	65535,
    .hMeasurementFrequency             =	TF_REGULATION_RATE_SCALED2,
    .DPPConvFactor                     =  DPP_CONV_FACTOR2,
  },
  .SensorPlacement     = HALL_SENSORS_PLACEMENT2,
  .PhaseShift          = (int16_t)(HALL_PHASE_SHIFT2 * 65536/360),
  .SpeedSamplingFreqHz = MEDIUM_FREQUENCY_TASK_RATE2,
  .SpeedBufferSize     = HALL_AVERAGING_FIFO_DEPTH2,
 .TIMClockFreq       = HALL_TIM_CLK2,
// .TIMx                = TIM5,

 .ICx_Filter          = M2_HALL_IC_FILTER,

 .PWMFreqScaling      = PWM_FREQ_SCALING2,
 .HallMtpa            = HALL_MTPA2,

 .H1Port             =  M2_HALL_H1_GPIO_Port,
 .H1Pin              =  M2_HALL_H1_Pin,
 .H2Port             =  M2_HALL_H2_GPIO_Port,
 .H2Pin              =  M2_HALL_H2_Pin,
 .H3Port             =  M2_HALL_H3_GPIO_Port,
 .H3Pin              =  M2_HALL_H3_Pin,
};

/**
  * @brief  SpeedNPosition sensor parameters Motor 1 - HALL
  */

HALL_Handle_t HALL_M1 =
{
  ._Super = {
    .bElToMecRatio                     =	POLE_PAIR_NUM,
    .hMaxReliableMecSpeedUnit          =	(uint16_t)(1.15*MAX_APPLICATION_SPEED_UNIT),
    .hMinReliableMecSpeedUnit          =	(uint16_t)(MIN_APPLICATION_SPEED_UNIT),
    .bMaximumSpeedErrorsNumber         =	MEAS_ERRORS_BEFORE_FAULTS,
    .hMaxReliableMecAccelUnitP         =	65535,
    .hMeasurementFrequency             =	TF_REGULATION_RATE_SCALED,
    .DPPConvFactor                     =  DPP_CONV_FACTOR,
  },
  .SensorPlacement     = HALL_SENSORS_PLACEMENT,
  .PhaseShift          = (int16_t)(HALL_PHASE_SHIFT * 65536/360),
  .SpeedSamplingFreqHz = MEDIUM_FREQUENCY_TASK_RATE,
  .SpeedBufferSize     = HALL_AVERAGING_FIFO_DEPTH,
 .TIMClockFreq       = HALL_TIM_CLK,
 .TIMx                = TIM3,

 .ICx_Filter          = M1_HALL_IC_FILTER,

 .PWMFreqScaling      = PWM_FREQ_SCALING,
 .HallMtpa            = HALL_MTPA,

 .H1Port             =  M1_HALL_H1_GPIO_Port,
 .H1Pin              =  M1_HALL_H1_Pin,
 .H2Port             =  M1_HALL_H2_GPIO_Port,
 .H2Pin              =  M1_HALL_H2_Pin,
 .H3Port             =  M1_HALL_H3_GPIO_Port,
 .H3Pin              =  M1_HALL_H3_Pin,
};

/**
  * temperature sensor parameters Motor 1
  */
NTC_Handle_t TempSensorParamsM1 =
{
  .bSensorType = REAL_SENSOR,
  .TempRegConv =
  {
    .regADC = ADC2,
    .channel = MC_ADC_CHANNEL_5,
    .samplingTime = M1_TEMP_SAMPLING_TIME,
  },
  .hLowPassFilterBW        = M1_TEMP_SW_FILTER_BW_FACTOR,
  .hOverTempThreshold      = (uint16_t)(OV_TEMPERATURE_THRESHOLD_d),
  .hOverTempDeactThreshold = (uint16_t)(OV_TEMPERATURE_THRESHOLD_d - OV_TEMPERATURE_HYSTERESIS_d),
  .hSensitivity            = (uint16_t)(ADC_REFERENCE_VOLTAGE/dV_dT),
  .wV0                     = (uint16_t)(V0_V *65536/ ADC_REFERENCE_VOLTAGE),
  .hT0                     = T0_C,
};

/**
  * temperature sensor parameters Motor 2
  */
NTC_Handle_t TempSensorParamsM2 =
{
  .bSensorType = REAL_SENSOR,
  .TempRegConv =
  {
//    .regADC = ADC3,
    .channel = MC_ADC_CHANNEL_4,
    .samplingTime = M2_TEMP_SAMPLING_TIME,
  },
  .hLowPassFilterBW        = M2_TEMP_SW_FILTER_BW_FACTOR,
  .hOverTempThreshold      = (uint16_t)(OV_TEMPERATURE_THRESHOLD_d2),
  .hOverTempDeactThreshold = (uint16_t)(OV_TEMPERATURE_THRESHOLD_d2 - OV_TEMPERATURE_HYSTERESIS_d2),
  .hSensitivity            = (uint16_t)(ADC_REFERENCE_VOLTAGE/dV_dT2),
  .wV0                     = (uint16_t)(V0_V2 *65536/ ADC_REFERENCE_VOLTAGE),
  .hT0                     = T0_C2,
};

/* Bus voltage sensor value filter buffer */
uint16_t RealBusVoltageSensorFilterBufferM1[M1_VBUS_SW_FILTER_BW_FACTOR];

/**
  * Bus voltage sensor parameters Motor 1
  */
RDivider_Handle_t RealBusVoltageSensorParamsM1 =
{
  ._Super                =
  {
    .SensorType          = REAL_SENSOR,
    .ConversionFactor    = (uint16_t)(ADC_REFERENCE_VOLTAGE / VBUS_PARTITIONING_FACTOR),
  },

  .VbusRegConv =
  {
    .regADC = ADC2,
    .channel = MC_ADC_CHANNEL_7,
    .samplingTime = M1_VBUS_SAMPLING_TIME,
  },
  .LowPassFilterBW       =  M1_VBUS_SW_FILTER_BW_FACTOR,
  .OverVoltageThreshold  = OVERVOLTAGE_THRESHOLD_d,
  .UnderVoltageThreshold =  UNDERVOLTAGE_THRESHOLD_d,
  .aBuffer = RealBusVoltageSensorFilterBufferM1,
};

/* Bus voltage sensor value filter buffer */
uint16_t RealBusVoltageSensorFilterBufferM2[M2_VBUS_SW_FILTER_BW_FACTOR];

/**
  * Bus voltage sensor parameters Motor 2
  */
RDivider_Handle_t RealBusVoltageSensorParamsM2 =
{
  ._Super                =
  {
    .SensorType          = REAL_SENSOR,
    .ConversionFactor    = (uint16_t)(ADC_REFERENCE_VOLTAGE / VBUS_PARTITIONING_FACTOR2),
  },
  .VbusRegConv =
  {
//    .regADC = ADC4,
    .channel = MC_ADC_CHANNEL_1,
    .samplingTime = M2_VBUS_SAMPLING_TIME,
  },
  .LowPassFilterBW       =  M2_VBUS_SW_FILTER_BW_FACTOR,
  .OverVoltageThreshold  = OVERVOLTAGE_THRESHOLD_d2,
  .UnderVoltageThreshold =  UNDERVOLTAGE_THRESHOLD_d2,
  .aBuffer = RealBusVoltageSensorFilterBufferM2,
   };

UI_Handle_t UI_Params =
{
  .bDriveNum = 0,
  .pFct_DACInit = &DAC_Init,
  .pFct_DACExec = &DAC_Exec,
  .pFctDACSetChannelConfig    = &DAC_SetChannelConfig,
  .pFctDACGetChannelConfig    = &DAC_GetChannelConfig,
  .pFctDACSetUserChannelValue = &DAC_SetUserChannelValue,
  .pFctDACGetUserChannelValue = &DAC_GetUserChannelValue,

};

DAC_UI_Handle_t DAC_UI_Params =
{
  .hDAC_CH1_ENABLED = ENABLE,
  .hDAC_CH2_ENABLED = ENABLE
};

/** RAMP for Motor1.
  *
  */
RampExtMngr_Handle_t RampExtMngrHFParamsM1 =
{
  .FrequencyHz = TF_REGULATION_RATE
};

/**
  * @brief  CircleLimitation Component parameters Motor 1 - Base Component
  */
CircleLimitation_Handle_t CircleLimitationM1 =
{
  .MaxModule          = MAX_MODULE,
  .MaxVd          	  = (uint16_t)(MAX_MODULE * FW_VOLTAGE_REF / 1000),
  .Circle_limit_table = MMITABLE,
  .Start_index        = START_INDEX,
};
/** RAMP for Motor2.
  *
  */
RampExtMngr_Handle_t RampExtMngrHFParamsM2 =
{
  .FrequencyHz = TF_REGULATION_RATE2
};

/**
  * @brief  CircleLimitation Component parameters Motor 2 - Base Component
  */
CircleLimitation_Handle_t CircleLimitationM2 =
{
  .MaxModule          = MAX_MODULE2,
  .MaxVd          	  = (uint16_t)(MAX_MODULE2 * FW_VOLTAGE_REF2 / 1000),
  .Circle_limit_table = MMITABLE2,
  .Start_index        = START_INDEX2,
};

UFCP_Handle_t pUSART =
{
  ._Super.RxTimeout = 0,
  .USARTx = USART1,

};

/* USER CODE BEGIN Additional configuration */

/* USER CODE END Additional configuration */

/******************* (C) COPYRIGHT 2019 STMicroelectronics *****END OF FILE****/

