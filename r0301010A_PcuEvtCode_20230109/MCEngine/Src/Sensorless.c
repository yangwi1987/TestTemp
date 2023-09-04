/*
 * Sensorless.c
 *
 *  Created on: 2022年1月6日
 *      Author: Fernando.Wang.HHW
 */
#include "Sensorless.h"


uint16_t Sensorless_Init( Sensorless_t *p, SensorlessSetting_t *pSetting )
{
	if( pSetting->Polepair < 1 )
	{
		p->Error = SENSORLESS_ERROR_POLEPAIR;
		return p->Error;
	}

	//EEMF sensorless on stator
	IPMSensorlessEEMFOnStatorInitParam_t EEMFSetting = IPM_SENSORLESS_EEMF_ON_STATOR_INIT_PARAM_DEFAULT;
	EEMFSetting.J = pSetting->J;
	EEMFSetting.Ld = pSetting->Ld;
	EEMFSetting.Lq = pSetting->Lq;
	EEMFSetting.Res = pSetting->Res;
	EEMFSetting.Period = pSetting->Period;

	float EEMFPole1 = ( pSetting->EEMFAngleObserverHz1 ) * _2PI;
	float EEMFPole2 = ( pSetting->EEMFAngleObserverHz2 ) * _2PI;
	float EEMFPole3 = ( pSetting->EEMFAngleObserverHz3 ) * _2PI;
	EEMFSetting.AngleObserverL1 = EEMFPole1 + EEMFPole2 + EEMFPole3 ;
	EEMFSetting.AngleObserverL2 = EEMFPole1 * EEMFPole2 + EEMFPole2 * EEMFPole3 + EEMFPole3 * EEMFPole1;
	EEMFSetting.AngleObserverL3 = EEMFPole1 * EEMFPole2 * EEMFPole3;
	p->EEMF.Init(&(p->EEMF),&EEMFSetting);

	//HFI Square
	IPMSensorlessHFISinInitParam_t HFISinSetting = IPM_SENSORLESS_HFI_SIN_SETTING_DEFAULT;
	HFISinSetting.Vamp = pSetting->HFISinVamp;
	HFISinSetting.J = pSetting->J;
	HFISinSetting.Period = pSetting->Period;

	float HFISinPole1 = ( pSetting->HFISinAngleObserverHz1 ) * _2PI;
	float HFISinPole2 = ( pSetting->HFISinAngleObserverHz2 ) * _2PI;
	float HFISinPole3 = ( pSetting->HFISinAngleObserverHz3 ) * _2PI;
	HFISinSetting.AngleObserverL1 = HFISinPole1 + HFISinPole2 + HFISinPole3 ;
	HFISinSetting.AngleObserverL2 = HFISinPole1 * HFISinPole2 + HFISinPole2 * HFISinPole3 + HFISinPole3 * HFISinPole1;
	HFISinSetting.AngleObserverL3 = HFISinPole1 * HFISinPole2 * HFISinPole3;
	HFISinSetting.LPFHz = pSetting->HFISinLPFHz;
	HFISinSetting.BPFHz = pSetting->HFISinBPFHz;
	HFISinSetting.BPFQ = pSetting->HFISinBPFQ;
	p->HFISin.Init(&(p->HFISin),&HFISinSetting);

	//Angle Init Setting
	p->AngleInit.Vinj = 0.0f;
	p->AngleInit.EleAngle = 0.0f;
	p->AngleInit.EleAngleInit = 0.0f;
	p->AngleInit.IdCmd = 0.0f;
	p->AngleInit.IqCmd = 0.0f;

	//Angle Init Pulse Detection Setting
	p->AngleInit.PulseDetection.Shift = 7;
	p->AngleInit.PulseDetection.Interval = ( ((uint16_t)1) << p->AngleInit.PulseDetection.Shift );
	p->AngleInit.PulseDetection.DetectionNumber = 6;
	p->AngleInit.PulseDetection.MaxCnt = ( p->AngleInit.PulseDetection.DetectionNumber + 1 ) * p->AngleInit.PulseDetection.Interval;
	p->AngleInit.PulseDetection.Vamp = 10.0f;
	p->AngleInit.PulseDetection.EleAngleAdd = _2PI / ((float)p->AngleInit.PulseDetection.DetectionNumber);
	//Angle Init Pulse Detection  Clean
	p->AngleInit.PulseDetection.Cnt = 0;
	p->AngleInit.PulseDetection.Section = 0;
	p->AngleInit.PulseDetection.Mod = 0;
	p->AngleInit.PulseDetection.Vinj = 0.0f;
	p->AngleInit.PulseDetection.EleAngle = -p->AngleInit.PulseDetection.EleAngleAdd;
	p->AngleInit.PulseDetection.EleAngleInit = 0.0f;
	p->AngleInit.PulseDetection.IdMax = 0.0f;

	//Angle Init Fixed Cmd Setting
	p->AngleInit.FixedCmd.MaxCnt = (uint16_t)((pSetting->AngleInitFixedCmdFirstTime + pSetting->AngleInitFixedCmdSecondTime) / pSetting->Period);
	p->AngleInit.FixedCmd.IdFirstCmd = pSetting->AngleInitFixedCmdFirstId;
	p->AngleInit.FixedCmd.IqFirstCmd = pSetting->AngleInitFixedCmdFirstIq;
	p->AngleInit.FixedCmd.SecondCnt = (uint16_t)(pSetting->AngleInitFixedCmdSecondTime / pSetting->Period);
	p->AngleInit.FixedCmd.IdSecondCmd = pSetting->AngleInitFixedCmdSecondId;
	p->AngleInit.FixedCmd.IqSecondCmd = pSetting->AngleInitFixedCmdSecondIq;
	//Angle Init Fixed Cmd Clean
	p->AngleInit.FixedCmd.Cnt = 0;
	return p->Error;
}

void Sensorless_Clean( Sensorless_t *p, float EleAngle, float EleSpeed )
{
	//EEMF
	p->EEMF.Start = FUNCTION_NO;
	p->EEMF.Clean(&(p->EEMF),EleAngle,EleSpeed,0.0f,0.0f,0.0f,0.0f);

	//HFI
	p->HFISin.Start = FUNCTION_NO;
	p->HFISin.Clean(&(p->HFISin),EleAngle,EleSpeed,0.0f,0.0f);

	//Angle Init Setting
	p->AngleInit.Vinj = 0.0f;
	p->AngleInit.EleAngle = 0.0f;
	p->AngleInit.EleAngleInit = 0.0f;
	p->AngleInit.IdCmd = 0.0f;
	p->AngleInit.IqCmd = 0.0f;

	//AngleInit by Pulse Detection
	p->AngleInit.PulseDetection.Cnt = 0;
	p->AngleInit.PulseDetection.Section = 0;
	p->AngleInit.PulseDetection.Mod = 0;
	p->AngleInit.PulseDetection.Vinj = 0.0f;
	p->AngleInit.PulseDetection.EleAngle = -p->AngleInit.PulseDetection.EleAngleAdd;
	p->AngleInit.PulseDetection.EleAngleInit = 0.0f;
	p->AngleInit.PulseDetection.IdMax = 0.0f;
	p->AngleInit.Start = FUNCTION_YES;

	//Angle Init Fixed Cmd Clean
	p->AngleInit.FixedCmd.Cnt = 0;

	//Sensorless state
	p->SensorlessState = SensorlessState_None;
}

void EEMFOnStator_Clean( IPMSensorlessEEMFOnStator_t *p, float EleAngle, float EleSpeed, float Valpha, float Vbeta, float Ialpha, float Ibeta )
{
	p->Valpha = Valpha;
	p->Vbeta = Vbeta;
	p->IalphaPre = Ialpha;
	p->IbetaPre = Ibeta;
	p->EEMFAlpha=0.0f;
	p->EEMFBeta=0.0f;
	p->EleAtan2Angle=EleAngle;
	p->EleSpeed=EleSpeed;
	p->EleAngle=EleAngle;
	p->EEMFCalcProcess = SENSORLESS_CALC_PROCESS_CLEAN;
	p->AngleObserver.Clean(&(p->AngleObserver),EleAngle,EleSpeed,0.0f);
}

uint16_t EEMFOnStator_Init( IPMSensorlessEEMFOnStator_t *p, IPMSensorlessEEMFOnStatorInitParam_t *pSetting )
{
	p->Error = EEMF_ON_STATOR_INIT_OK;
	EEMFOnStator_Clean( p, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f );
	p->Ld = pSetting->Ld;
	p->Lq = pSetting->Lq;
	p->Res = pSetting->Res;
	p->WindingResBase = pSetting->Res;
	p->WindingResTempCoeff = WINDING_TEMP_COEFFICIENT_OF_RES;
	p->WindingTemp = 0.0f;
	if( pSetting->Period <= 0.0f )
	{
		p->Period = 0.0f;
		p->DividePeriod = 0.0f;
		p->Error = EEMF_ON_STATOR_ERROR_PERIOD;
	}
	else
	{
		p->Period = pSetting->Period;
		p->DividePeriod = 1.0f / p->Period;
	}

	AngleObserverInitParm_t AngleObserverSetting = ANGLE_OBSERVER_INIT_PARAM_DEFAULT;
	AngleObserverSetting.Period = pSetting->Period;
	AngleObserverSetting.J = pSetting->J;
	AngleObserverSetting.L1 = pSetting->AngleObserverL1;
	AngleObserverSetting.L2 = pSetting->AngleObserverL2;
	AngleObserverSetting.L3 = pSetting->AngleObserverL3;
	AngleObserverSetting.SpeedLowerLimit = -3769.9111843f;	//9000RPM/60*4*2*pi = 3769.9111843 rad/s(Electrical)
	AngleObserverSetting.SpeedUpperLimit = 3769.9111843f;	//9000RPM/60*4*2*pi = 3769.9111843 rad/s(Electrical)
	AngleObserverSetting.AcelLowerLimit = -111374.6392f;
	AngleObserverSetting.AcelUpperLimit = 111374.6392f;

	p->LowerLevelError = p->AngleObserver.Init( &(p->AngleObserver) , &AngleObserverSetting );
	if( p->LowerLevelError !=0 )
	{
		p->Error = EEMF_ON_STATOR_ERROR_ANGLE_OBSERVER;
	}
	else
	{
		//do nothing
	}
	return p->Error;
}

static inline float EEMFOnStator_CalcEEMF( IPMSensorlessEEMFOnStator_t *p, float Volt, float Imain, float ImainPre, float Iseondary, float EleSpeedMultiLdiff )
{
	float ResMultiImain = p->Res * Imain;
	float LdMultiImainDiff = p->Ld * ( Imain - ImainPre ) * p->DividePeriod;
	float EleSpeedMultiLdiffMultiIsecondary = EleSpeedMultiLdiff * Iseondary;
	return ( Volt - ResMultiImain - LdMultiImainDiff - EleSpeedMultiLdiffMultiIsecondary );
}

void EEMFOnStator_Calc( IPMSensorlessEEMFOnStator_t *p, float Valpha, float Vbeta, float Ialpha, float Ibeta, float EleSpeed )
{
	float EleSpeedMultiLdiff = EleSpeed * ( p->Ld - p->Lq );
	p->EEMFAlpha = EEMFOnStator_CalcEEMF( p, p->Valpha, Ialpha, p->IalphaPre, Ibeta, EleSpeedMultiLdiff );
	p->EEMFBeta = EEMFOnStator_CalcEEMF( p, p->Vbeta, Ibeta, p->IbetaPre, Ialpha, -EleSpeedMultiLdiff );

	CordicMath_Atan2_Macro(-p->EEMFAlpha,p->EEMFBeta,CORDIC_EEMF_ATAN2_SCALE,p->EleAtan2Angle)

	ANGLE_OBSERVER_CALC_MACRO( (&(p->AngleObserver)), (p->EleAtan2Angle) );
	p->EleAngle = p->AngleObserver.Angle;
	p->EleSpeed = p->AngleObserver.Speed;

	p->Valpha = Valpha;
	p->Vbeta = Vbeta;
	p->IalphaPre = Ialpha;
	p->IbetaPre = Ibeta;
}

void HFISin_Clean( IPMSensorlessHFISin_t *p, float EleAngle, float EleSpeed, float Ialpha, float Ibeta )
{
	p->EleAtan2Angle=EleAngle;
	p->EleSpeed=EleSpeed;
	p->EleAngle=EleAngle;
	p->Vinj = 0.0f;
	p->AngleObserver.Clean( &(p->AngleObserver), EleAngle,EleSpeed,0.0f );
	p->CalcIalphaPosBPF.Clean( &p->CalcIalphaPosBPF );
	p->CalcIbetaPosBPF.Clean( &p->CalcIbetaPosBPF );
	p->CalcIalphaPosLPF.Clean( &p->CalcIalphaPosLPF );
	p->CalcIbetaPosLPF.Clean( &p->CalcIbetaPosLPF );
	p->HFISinCalcProcess = SENSORLESS_CALC_PROCESS_CLEAN;
}

uint16_t HFISin_Init( IPMSensorlessHFISin_t *p, IPMSensorlessHFISinInitParam_t *pSetting )
{
	p->Vamp = pSetting->Vamp;
	p->Error = EEMF_ON_STATOR_INIT_OK;
	HFISin_Clean( p, 0.0f, 0.0f, 0.0f, 0.0f );
	if( pSetting->Period <= 0.0f )
	{
		p->Error = HFI_ON_STATOR_ERROR_PERIOD;
	}
	else
	{
		//do nothing
	}

	AngleObserverInitParm_t AngleObserverSetting = ANGLE_OBSERVER_INIT_PARAM_DEFAULT;
	AngleObserverSetting.Period = pSetting->Period;
	AngleObserverSetting.J = pSetting->J;
	AngleObserverSetting.L1 = pSetting->AngleObserverL1;
	AngleObserverSetting.L2 = pSetting->AngleObserverL2;
	AngleObserverSetting.L3 = pSetting->AngleObserverL3;
	AngleObserverSetting.SpeedLowerLimit = -3769.9111843f;	//9000RPM/60*4*2*pi = 3769.9111843 rad/s(Electrical)
	AngleObserverSetting.SpeedUpperLimit = 3769.9111843f;	//9000RPM/60*4*2*pi = 3769.9111843 rad/s(Electrical)
	AngleObserverSetting.AcelLowerLimit = -111374.6392f;
	AngleObserverSetting.AcelUpperLimit = 111374.6392f;

	p->LowerLevelError = p->AngleObserver.Init( &(p->AngleObserver) , &AngleObserverSetting );
	if( p->LowerLevelError !=0 )
	{
		p->Error = HFI_ON_STATOR_ERROR_ANGLE_OBSERVER;
	}
	else
	{
		//do nothing
	}
	FILTER_INIT_BILINEAR_1ORDER_TYPE FilterSettingTmp = FILTER_INIT_BILINEAR_1ORDER_DEFAULT;
	FilterBilinearBPFSetting_t FilterSettingTmp2 = FILTER_BILINEAR_BPF_SETTING_DEFAULT;
	FilterSettingTmp2.BandwithHz = pSetting->BPFHz;
	FilterSettingTmp2.Period = pSetting->Period;
	FilterSettingTmp2.Q = pSetting->BPFQ;
	p->LowerLevelError = p->CalcIalphaPosBPF.Init( &(p->CalcIalphaPosBPF) , &FilterSettingTmp2 );
	if( p->LowerLevelError !=0 )
	{
		p->Error = HFI_ON_STATOR_ERROR_FILTER_IALPHA_BPF;
	}
	else
	{
		//do nothing
	}
	p->LowerLevelError = p->CalcIbetaPosBPF.Init( &(p->CalcIbetaPosBPF) , &FilterSettingTmp2 );
	if( p->LowerLevelError !=0 )
	{
		p->Error = HFI_ON_STATOR_ERROR_FILTER_IBETA_BPF;
	}
	else
	{
		//do nothing
	}

	FilterSettingTmp.BandwithHz = pSetting->LPFHz;
	FilterSettingTmp.Period = pSetting->Period;
	FilterSettingTmp.Type = FILTER_TYPE_LPF;
	p->LowerLevelError = p->CalcIalphaPosLPF.Init( &(p->CalcIalphaPosLPF) , &FilterSettingTmp );
	if( p->LowerLevelError !=0 )
	{
		p->Error = HFI_ON_STATOR_ERROR_FILTER_IALPHA_LPF;
	}
	else
	{
		//do nothing
	}
	p->LowerLevelError = p->CalcIbetaPosLPF.Init( &(p->CalcIbetaPosLPF) , &FilterSettingTmp );
	if( p->LowerLevelError !=0 )
	{
		p->Error = HFI_ON_STATOR_ERROR_FILTER_IBETA_LPF;
	}
	else
	{
		//do nothing
	}
	return p->Error;
}

void HFISin_Calc( IPMSensorlessHFISin_t *p, float Ialpha, float Ibeta )
{
	//calc ipos
	float Y = 0.0f;
	float X = 0.0f;
	FILTER_BILINEAR_BPF_CALC_MACRO( (&(p->CalcIalphaPosBPF)), Ialpha, (p->IalphaPosBPF) )
	FILTER_BILINEAR_BPF_CALC_MACRO( (&(p->CalcIbetaPosBPF)), Ibeta, (p->IbetaPosBPF) )
	X = p->IalphaPosBPF * p->SinValuePre;
	FILTER_BILINEAR_1ORDER_CALC_MACRO( (&(p->CalcIalphaPosLPF)), X, Y, (p->IalphaPos) )
	X = p->IbetaPosBPF * p->SinValuePre;
	FILTER_BILINEAR_1ORDER_CALC_MACRO( (&(p->CalcIbetaPosLPF)), X, Y, (p->IbetaPos) )

	p->IalphaCtrl = Ialpha - p->IalphaPosBPF;
	p->IbetaCtrl = Ibeta - p->IbetaPosBPF;
	p->SinValuePre = p->SinValue;	//Don't move this line because of timing

	CordicMath_Atan2_Macro(p->IbetaPos,p->IalphaPos,CORDIC_HFI_ATAN2_SCALE,p->EleAtan2Angle);

	ANGLE_OBSERVER_CALC_MACRO( (&(p->AngleObserver)), (p->EleAtan2Angle) );
	p->EleAngle = p->AngleObserver.Angle;
	p->EleSpeed = p->AngleObserver.Speed;

	//calc Vinj
	p->Vinj = p->Vamp * p->CosTable[p->VinjCnt];
	p->SinValue = p->SinTable[p->VinjCnt];
	p->VinjCnt++;
	p->VinjCnt = ( p->VinjCnt >= HFI_SIN_TIMES ) ? 0 : p->VinjCnt;
}
