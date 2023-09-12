/*
 * Sensorless.h
 *
 *  Created on: 2022年1月6日
 *      Author: Fernando.Wang.HHW
 */

#ifndef INC_SENSORLESS_H_
#define INC_SENSORLESS_H_
#include "Constant.h"
#include "Math.h"
#include "AngleObserver.h"
#include "MathFunctionInclude.h"
#include "CoordinateTransfer.h"

#define CORDIC_EEMF_ATAN2_SCALE 31638469.48046352f //-67.87571217141374~67.87571217141374->-1~1 : 1/67.87571217141374*Q31 = 31638469.48046352f [ EEMF peak = 67.87571217141374 @ Id=-300A、Iq=300A、9000RPM ]
#define CORDIC_HFI_ATAN2_SCALE 7158278.826666667f //-300~300->-1~1 : 1/300*Q31 = 7158278.826666667f

#define HFI_SIN_TIMES 10

enum SensorlessState_e
{
	SensorlessState_None,
	SensorlessState_Initial_Angle_Align,
	SensorlessState_Using_HFI_Algorithm,
	SensorlessState_Using_EEMF_Algorithm,
	SensorlessState_Switching_from_HFI_to_EEMF,
	SensorlessState_Switching_from_EEMF_to_HFI
};

enum SENSORLESS_INIT_STATUS_ENUM
{
	SENSORLESS_INIT_OK = 0,
	SENSORLESS_ERROR_POLEPAIR,
};

enum IPM_SENSORLESS_EEMF_ON_STATOR_INIT_STATUS
{
	EEMF_ON_STATOR_INIT_OK = 0,
	EEMF_ON_STATOR_ERROR_PERIOD,
	EEMF_ON_STATOR_ERROR_ANGLE_OBSERVER,
};

enum IPM_SENSORLESS_EEMF_CALC_PROCESS_STATUS_ENUM
{
	EEMF_CALC_PROCESS_OUT = 0,
	EEMF_CALC_PROCESS_IN,
};

enum IPM_SENSORLESS_HFI_SIN_INIT_STATUS_ENUM
{
	HFI_ON_STATOR_INIT_OK = 0,
	HFI_ON_STATOR_ERROR_PERIOD,
	HFI_ON_STATOR_ERROR_ANGLE_OBSERVER,
	HFI_ON_STATOR_ERROR_FILTER_IALPHA_BPF,
	HFI_ON_STATOR_ERROR_FILTER_IBETA_BPF,
	HFI_ON_STATOR_ERROR_FILTER_IALPHA_LPF,
	HFI_ON_STATOR_ERROR_FILTER_IBETA_LPF,
};

enum IPM_SENSORLESS_CALC_PROCESS_STATUS_ENUM
{
	SENSORLESS_CALC_PROCESS_CLEAN = 0,
	SENSORLESS_CALC_PROCESS_EXE,
	SENSORLESS_CALC_PROCESS_ENTERING,
};

typedef uint16_t (*pfunSensorless_Init)( void*, void* );
typedef void (*pfunSensorless_Clean)( void*, float, float );
typedef void (*pfunEEMFOnStator_Clean)( void*, float, float, float, float, float, float );
typedef uint16_t (*pfunEEMFOnStator_Init)( void*, void* );
typedef void (*pfunEEMFOnStator_Calc)( void*, float , float , float , float , float );
typedef void (*pfunHFISin_Clean)( void*, float, float, float, float );
typedef uint16_t (*pfunHFISin_Init)( void*, void* );
typedef void (*pfunHFISin_Calc)( void*, float, float );

typedef struct
{
	float Ld;
	float Lq;
	float Res;
	float J;
	float Period;
	float AngleObserverL1;
	float AngleObserverL2;
	float AngleObserverL3;
} IPMSensorlessEEMFOnStatorInitParam_t;

typedef struct
{
	uint16_t Error : 8;
	uint16_t LowerLevelError : 8;
	uint16_t Start : 1;
	uint16_t EEMFCalcProcess : 2;
	uint16_t Reserved : 13;
	float Ld;
	float Lq;
	float Res;
	float Period;
	float DividePeriod;
	float Valpha;
	float Vbeta;
	float IalphaPre;
	float IbetaPre;
	float EEMFAlpha;
	float EEMFBeta;
	float EleAtan2Angle;
	float EleSpeed;
	float EleAngle;
	float WindingResBase;
	float WindingResTempCoeff;
	float WindingTemp;
	AngleObserver_t AngleObserver;
	pfunEEMFOnStator_Clean Clean;
	pfunEEMFOnStator_Init Init;
	pfunEEMFOnStator_Calc Calc;
} IPMSensorlessEEMFOnStator_t;

typedef struct
{
	float Vamp;
	float J;
	float Period;
	float AngleObserverL1;
	float AngleObserverL2;
	float AngleObserverL3;
	float LPFHz;
	float BPFHz;
	float BPFQ;
} IPMSensorlessHFISinInitParam_t;

typedef struct
{
	uint16_t Error : 8;
	uint16_t LowerLevelError : 8;
	uint16_t Start : 1;
	uint16_t HFISinCalcProcess : 2;
	uint16_t Reserved : 13;
	uint16_t VinjCnt;
	float Vamp;
	float Vinj;
	float IalphaPosBPF;
	float IbetaPosBPF;
	float IalphaPos;
	float IbetaPos;
	float IalphaCtrl;
	float IbetaCtrl;
	float EleAtan2Angle;
	float EleSpeed;
	float EleAngle;
	float SinValue;
	float SinValuePre;
	float CosTable[HFI_SIN_TIMES];
	float SinTable[HFI_SIN_TIMES];
	FilterBilinearBPF_t CalcIalphaPosBPF;
	FilterBilinearBPF_t CalcIbetaPosBPF;
	FILTER_BILINEAR_1ORDER_TYPE CalcIalphaPosLPF;
	FILTER_BILINEAR_1ORDER_TYPE CalcIbetaPosLPF;
	ROTOR_TO_STATOR_TYPE VinjStator;
	AngleObserver_t AngleObserver;
	pfunHFISin_Clean Clean;
	pfunHFISin_Init Init;
	pfunHFISin_Calc Calc;
} IPMSensorlessHFISin_t;

typedef struct
{
	uint16_t Reserved;
	uint16_t Polepair;
	float Ld;
	float Lq;
	float Res;
	float J;
	float Period;
//	float CurrentLimit;
//	float IFAcelTime;
//	float InEEMFEleSpeedAbs;
//	float OutEEMFEleSpeedAbs;
	float EEMFAngleObserverHz1;
	float EEMFAngleObserverHz2;
	float EEMFAngleObserverHz3;
	float HFISinVamp;
	float HFISinAngleObserverHz1;
	float HFISinAngleObserverHz2;
	float HFISinAngleObserverHz3;
	float HFISinLPFHz;
	float HFISinBPFHz;
	float HFISinBPFQ;
	float AngleInitFixedCmdFirstTime;
	float AngleInitFixedCmdSecondTime;
	float AngleInitFixedCmdFirstId;
	float AngleInitFixedCmdFirstIq;
	float AngleInitFixedCmdSecondId;
	float AngleInitFixedCmdSecondIq;
	float AngleInitFixedCmdDelayTimeSec;
} SensorlessSetting_t;

typedef struct
{
	uint16_t Cnt;
	uint16_t Shift;
	uint16_t Interval;
	uint16_t Section;
	uint16_t Mod;
	uint16_t DetectionNumber;
	uint16_t MaxCnt;
	float Vinj;
	float Vamp;
	float EleAngle;
	float EleAngleAdd;
	float EleAngleInit;
	float IdMax;
} IPMSensorlessAngleInitByPulseDetection_t;

typedef struct
{
	uint16_t Cnt;
	uint16_t MaxCnt;
	uint16_t SecondCnt;
	uint16_t DelayCnt;
	float IdFirstCmd;
	float IqFirstCmd;
	float IdSecondCmd;
	float IqSecondCmd;
} IPMSensorlessAngleInitByFixedCmd_t;

typedef struct
{
	uint16_t Start:1;
	uint16_t ReservedBit1:1;
	uint16_t Reserved:14;
	float Vinj;
	float EleAngle;
	float EleAngleInit;
	float IdCmd;
	float IqCmd;
	ROTOR_TO_STATOR_TYPE VinjStator;
	IPMSensorlessAngleInitByPulseDetection_t PulseDetection;
	IPMSensorlessAngleInitByFixedCmd_t FixedCmd;
} IPMSensorlessAngleInit_t;

typedef struct
{
	uint16_t Error : 8;
	uint16_t LowerLevelError : 8;
	uint16_t SensorlessState;
	IPMSensorlessEEMFOnStator_t EEMF;
	IPMSensorlessHFISin_t HFISin;
	IPMSensorlessAngleInit_t AngleInit;
	pfunSensorless_Init Init;
	pfunSensorless_Clean Clean;
} Sensorless_t;

uint16_t Sensorless_Init( Sensorless_t *p, SensorlessSetting_t *pSetting );
void Sensorless_Clean( Sensorless_t *p, float EleAngle, float EleSpeed );
void EEMFOnStator_Clean( IPMSensorlessEEMFOnStator_t *p, float EleAngle, float EleSpeed, float Valpha, float Vbeta, float Ialpha, float Ibeta );
uint16_t EEMFOnStator_Init( IPMSensorlessEEMFOnStator_t *p, IPMSensorlessEEMFOnStatorInitParam_t *pSetting );
void EEMFOnStator_Calc( IPMSensorlessEEMFOnStator_t *p, float Valpha, float Vbeta, float Ialpha, float Ibeta, float EleSpeed );
void HFISin_Clean( IPMSensorlessHFISin_t *p, float EleAngle, float EleSpeed, float Ialpha, float Ibeta );
uint16_t HFISin_Init( IPMSensorlessHFISin_t *p, IPMSensorlessHFISinInitParam_t *pSetting );
void HFISin_Calc( IPMSensorlessHFISin_t *p, float Ialpha, float Ibeta );

#define IPM_SENSORLESS_EEMF_ON_STATOR_INIT_PARAM_DEFAULT \
{			\
	0.0f,	\
	0.0f,	\
	0.0f,	\
	0.0f,	\
	0.0f,	\
	0.0f,	\
	0.0f,	\
	0.0f,	\
}

#define IPM_SENSORLESS_EEMF_ON_STATOR_DEFAULT	\
{			\
	0,		\
	0,		\
	0,		\
	0,		\
	0,		\
	0.0f,	\
	0.0f,	\
	0.0f,	\
	0.0f,	\
	0.0f,	\
	0.0f,	\
	0.0f,	\
	0.0f,	\
	0.0f,	\
	0.0f,	\
	0.0f,	\
	0.0f,	\
	0.0f,	\
	0.0f,	\
	0.0f,	\
	0.0f,	\
	0.0f,	\
	ANGLE_OBSERVER_DEFAULT, \
	(pfunEEMFOnStator_Clean) EEMFOnStator_Clean, 	\
	(pfunEEMFOnStator_Init) EEMFOnStator_Init, 		\
	(pfunEEMFOnStator_Calc) EEMFOnStator_Calc,		\
}

#define SENSORLESS_SETTING_DEFAULT	\
{			\
	0,		\
	0,		\
	0.0f,	/*Ld;    */\
	0.0f,	/*Lq;    */\
	0.0f,	/*Res;   */\
	0.0f,	/*J;     */\
	0.0f,	/*Period;*/\
	0.0f,	/*EEMFAngleObserverHz1;       */\
	0.0f,	/*EEMFAngleObserverHz2;       */\
	0.0f,	/*EEMFAngleObserverHz3;       */\
	0.0f,	/*HFISinVamp;                 */\
	0.0f,	/*HFISinAngleObserverHz1;     */\
	0.0f,	/*HFISinAngleObserverHz2;     */\
	0.0f,	/*HFISinAngleObserverHz3;     */\
	0.0f,	/*HFISinLPFHz;                */\
	0.0f,	/*HFISinBPFHz;                */\
	0.0f,	/*HFISinBPFQ;                 */\
	0.0f,	/*AngleInitFixedCmdFirstTime; */\
	0.0f,	/*AngleInitFixedCmdSecondTime;*/\
	0.0f,	/*AngleInitFixedCmdFirstId; */\
	0.0f,	/*AngleInitFixedCmdFirstIq; */\
	0.0f,	/*AngleInitFixedCmdSecondId;*/\
	0.0f,	/*AngleInitFixedCmdSecondIq;*/\
	0.0f,	/*AngleInitFixedCmdDelayTimeSec;*/\
}

#define IPM_SENSORLESS_HFI_SIN_SETTING_DEFAULT \
{			\
	0.0f,	\
	0.0f,	\
	0.0f,	\
	0.0f,	\
	0.0f,	\
	0.0f,	\
	0.0f,	\
	0.0f,	\
	0.0f,	\
}

#define IPM_SENSORLESS_HFI_SIN_DEFAULT \
{			\
	0,		\
	0,		\
	0,		\
	0,		\
	0,		\
	0,		\
	10.0f,  \
	0.0f,	\
	0.0f,	\
	0.0f,	\
	0.0f,	\
	0.0f,	\
	0.0f,	\
	0.0f,	\
	0.0f,	\
	0.0f,	\
	0.0f,	\
	0.0f,	\
	0.0f,	\
	{1.00000000f,	0.80901699f,	0.30901699f,	-0.30901699f,	-0.80901699f,	-1.00000000f,	-0.80901699f,	-0.30901699f,	0.30901699f,	0.80901699f}, \
	{0.00000000f,	0.58778525f,	0.95105652f,	0.95105652f,	0.58778525f,	0.00000000f,	-0.58778525f,	-0.95105652f,	-0.95105652f,	-0.58778525f}, \
	FILTER_BILINEAR_BPF_DEFAULT, \
	FILTER_BILINEAR_BPF_DEFAULT, \
	FILTER_BILINEAR_1ORDER_DEFAULT, \
	FILTER_BILINEAR_1ORDER_DEFAULT, \
	ROTOR_TO_STATOR_DEFAULT, \
	ANGLE_OBSERVER_DEFAULT, \
	(pfunHFISin_Clean) HFISin_Clean, 	\
	(pfunHFISin_Init) HFISin_Init,	\
	(pfunHFISin_Calc) HFISin_Calc,	\
}

#define IPM_SENSORLESS_ANGLE_INIT_BY_PULSE_DETECTION_DEFAULT \
{			\
	0,		\
	0,		\
	0,		\
	0,		\
	0,		\
	0,		\
	0,		\
	0.0f,	\
	0.0f,	\
	0.0f,	\
	0.0f,	\
	0.0f,	\
	0.0f,	\
}

#define IPM_SENSORLESS_ANGLE_INIT_BY_FIXED_CMD_DEFAULT	\
{			\
	0,		\
	0,		\
	0,		\
	0,		\
	0.0f,	\
	0.0f,	\
}

#define IPM_SENSORLESS_ANGLE_INIT_DEFAULT \
{			\
	0,		\
	0,		\
	0,		\
	0.0f,	\
	0.0f,	\
	0.0f,	\
	0.0f,	\
	0.0f,	\
	ROTOR_TO_STATOR_DEFAULT, 								\
	IPM_SENSORLESS_ANGLE_INIT_BY_PULSE_DETECTION_DEFAULT, 	\
	IPM_SENSORLESS_ANGLE_INIT_BY_FIXED_CMD_DEFAULT,			\
}

#define SENSORLESS_DEFAULT	\
{			\
	0,		\
	0,		\
	0,		\
	IPM_SENSORLESS_EEMF_ON_STATOR_DEFAULT, \
	IPM_SENSORLESS_HFI_SIN_DEFAULT, 	\
	IPM_SENSORLESS_ANGLE_INIT_DEFAULT,	\
	(pfunSensorless_Init) Sensorless_Init,	\
	(pfunSensorless_Clean) Sensorless_Clean, \
}

#define SENSORLESS_ANGLE_INIT_BY_FIXED_CMD(p,pAngleInit)	\
	p->Cnt++;												\
	if( p->Cnt < p->SecondCnt )								\
	{														\
		pAngleInit->IdCmd = p->IdFirstCmd; 						\
		pAngleInit->IqCmd = p->IqFirstCmd;			    		\
	}														\
	else if( p->Cnt < p->MaxCnt )							\
	{														\
		pAngleInit->IdCmd = p->IdSecondCmd;						\
		pAngleInit->IqCmd = p->IqSecondCmd;						\
	}														\
	else if( p->Cnt < ( p->MaxCnt + p->DelayCnt )) 			\
	{														\
		pAngleInit->IdCmd = 0.0f;							\
		pAngleInit->IqCmd = 0.0f;							\
	}														\
	else 													\
	{														\
		pAngleInit->IdCmd = 0.0f;							\
		pAngleInit->IqCmd = 0.0f;							\
		p->Cnt = p->MaxCnt + p->DelayCnt + 1;								\
		pAngleInit->Start = FUNCTION_NO;					\
	}														\
	/*assigne to AngleInit_t*/								\
	pAngleInit->Vinj = 0.0f;								\
	pAngleInit->EleAngle = 0.0f;							\
	pAngleInit->EleAngleInit = 3.66519f;							/*210 deg*/\


#endif /* INC_SENSORLESS_H_ */
