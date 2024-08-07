/*
 * ADC_DRI.c
 *
 *  Created on: Dec 30, 2019
 *      Author: Hank Chen
 */

/* Includes ------------------------------------------------------------------*/
#include "ADC_DRI.h"
#include "math.h"
#include "ConstantParamAndUseFunction.h"

#define RING(X, XMax)		( (X > XMax) ? 0 : X )

ADC_HandleTypeDef *AdcAddress[ ADC_ADDRESS_SIZE ] =
{
		ADC_NONE,
		&hadc1,
		&hadc2,
		&hadc3,
		&hadc4,
		&hadc5,
};
uint16_t AdcIJRankReMapping[ INJECTED_RANK_SIZE ] =
{
		ADC_NONE,
		ADC_INJECTED_RANK_1,
		ADC_INJECTED_RANK_2,
		ADC_INJECTED_RANK_3,
		ADC_INJECTED_RANK_4
};

void AdcStation_MarkInjectionGroupReadFlag( AdcStation *v, ADC_HandleTypeDef* hadc )
{
	if( hadc->Instance == ADC1 ) {
		v->AdcInjGroup |= 0x01 << 0;
	} else if( hadc->Instance == ADC2 ) {
		v->AdcInjGroup |= 0x01 << 1;
	} else if( hadc->Instance == ADC3 ) {
		v->AdcInjGroup |= 0x01 << 2;
	} else if( hadc->Instance == ADC4 ) {
		v->AdcInjGroup |= 0x01 << 3;
	} else if( hadc->Instance == ADC5 ) {
		v->AdcInjGroup |= 0x01 << 4;
	}
}

void AdcStation_MarkRegularGroupReadFlag( AdcStation *v, ADC_HandleTypeDef* hadc )
{
	if( hadc->Instance == ADC1 ) {
		v->AdcRegGroup |= 0x01 << 0;
	} else if( hadc->Instance == ADC2 ) {
		v->AdcRegGroup |= 0x01 << 1;
	} else if( hadc->Instance == ADC3 ) {
		v->AdcRegGroup |= 0x01 << 2;
	} else if( hadc->Instance == ADC4 ) {
		v->AdcRegGroup |= 0x01 << 3;
	} else if( hadc->Instance == ADC5 ) {
		v->AdcRegGroup |= 0x01 << 4;
	}
}

void AdcStation_Init( AdcStation *v )
{
	int i;

	// Init Injection Group Channels
	for( i = 0; i < CURRENT_LOOP_CHANNEL_SIZE; i++ )
	{
		v->InjCh[i].AdcGroup = AdcAddress[ PCUTable.AdcInjectionGroupTable[i].AdcGroup ];
		v->InjCh[i].AdcInjRank = AdcIJRankReMapping[ PCUTable.AdcInjectionGroupTable[i].AdcInjRank ];
		v->InjCh[i].NameDef = PCUTable.AdcInjectionGroupTable[i].NameDef;
		v->InjCh[i].ChEnable = PCUTable.AdcInjectionGroupTable[i].ChEnable;
		v->InjCh[i].ZeroEnable = PCUTable.AdcInjectionGroupTable[i].ZeroEnable;
		v->InjCh[i].GainValue = PCUTable.AdcInjectionGroupTable[i].GainValue;
		if( v->InjCh[i].ZeroEnable )
		{
			v->ZeroCalibInjChCount++;
		}
	}

	// Init Regular Group Channels (Using DMA)
	for( i = 0; i < PLC_LOOP_CHANNEL_SIZE; i++ )
	{
		v->RegCh[i].AdcGroupIndex = ( PCUTable.AdcRegularGroupTable[i].AdcGroup == GROUP_DISABLE )? PCUTable.AdcRegularGroupTable[i].AdcGroup : ( PCUTable.AdcRegularGroupTable[i].AdcGroup - 1 );
		v->RegCh[i].AdcRankIndex = ( PCUTable.AdcRegularGroupTable[i].AdcRegRank == RANK_DISABLE )? PCUTable.AdcRegularGroupTable[i].AdcRegRank : ( PCUTable.AdcRegularGroupTable[i].AdcRegRank - 1 );
		v->RegCh[i].NameDef = PCUTable.AdcRegularGroupTable[i].NameDef;
		v->RegCh[i].ChEnable = PCUTable.AdcRegularGroupTable[i].ChEnable;
		v->RegCh[i].ZeroEnable = PCUTable.AdcRegularGroupTable[i].ZeroEnable;
		v->RegCh[i].GainValue = PCUTable.AdcRegularGroupTable[i].GainValue;
		if( v->RegCh[i].ZeroEnable )
		{
			v->ZeroCalibRegChCount++;
		}
		if( v->RegCh[i].ChEnable )
		{
			v->AdcDmaChCnt[v->RegCh[i].AdcGroupIndex]++;
		}
	}

	// Init Thermo Channels (Using DMA)
	for( i = 0; i < THERMAL_CHANNEL_SIZE; i++ )
	{
		v->ThermoCh[i].AdcGroupIndex = ( PCUTable.AdcThermoTable[i].AdcGroup == GROUP_DISABLE ) ? PCUTable.AdcThermoTable[i].AdcGroup : PCUTable.AdcThermoTable[i].AdcGroup - 1;
		v->ThermoCh[i].AdcRankIndex = ( PCUTable.AdcThermoTable[i].AdcRank == RANK_DISABLE ) ? PCUTable.AdcThermoTable[i].AdcRank : PCUTable.AdcThermoTable[i].AdcRank - 1;
		v->ThermoCh[i].NameDef = PCUTable.AdcThermoTable[i].NameDef;
		v->ThermoCh[i].ChEnable = PCUTable.AdcThermoTable[i].ChEnable;
		v->ThermoCh[i].ZeroEnable = PCUTable.AdcThermoTable[i].ZeroEnable;
		v->ThermoCh[i].TableIndex = ( PCUTable.AdcThermoTable[i].TableIdx == NO_TABLE )? PCUTable.AdcThermoTable[i].TableIdx : ( PCUTable.AdcThermoTable[i].TableIdx - 1 );

		if( v->ThermoCh[i].ChEnable )
		{
			v->AdcDmaChCnt[v->ThermoCh[i].AdcGroupIndex]++;
		}
	}

	for( i = 0; i < ADC_GROUP_CNT; i++ )
	{
		if( PCUTable.AdcSetupTab[i].GroupEna )
		{
			HAL_ADCEx_Calibration_Start( ( ADC_HandleTypeDef * )AdcAddress[PCUTable.AdcSetupTab[i].Group], ADC_SINGLE_ENDED );
		}

		if( PCUTable.AdcSetupTab[i].GroupEna && PCUTable.AdcSetupTab[i].InjectedMode )
		{
			HAL_ADCEx_InjectedStart_IT( ( ADC_HandleTypeDef * )AdcAddress[PCUTable.AdcSetupTab[i].Group] );
			v->AdcInjGroupFlag |= 0x01 << i;
		}

		if( PCUTable.AdcSetupTab[i].GroupEna && PCUTable.AdcSetupTab[i].DmaEnable )
		{
			HAL_ADC_Start_DMA( ( ADC_HandleTypeDef * )AdcAddress[PCUTable.AdcSetupTab[i].Group], ( uint32_t * )&v->AdcDmaData[ PCUTable.AdcSetupTab[i].Group-1 ][ 0 ], ( uint32_t )v->AdcDmaChCnt[i] );
			v->AdcRegGroupFlag |= 0x01 << i;
		}
	}

	// Init NTC Curve
	for( i = 0; i < NTC_TYPE_NUMBER; i++ )
	{
		v->NTCTable[i].A6 = PCUTable.AdcCurveFittingTable[i].A6;
		v->NTCTable[i].A5 = PCUTable.AdcCurveFittingTable[i].A5;
		v->NTCTable[i].A4 = PCUTable.AdcCurveFittingTable[i].A4;
		v->NTCTable[i].A3 = PCUTable.AdcCurveFittingTable[i].A3;
		v->NTCTable[i].A2 = PCUTable.AdcCurveFittingTable[i].A2;
		v->NTCTable[i].A1 = PCUTable.AdcCurveFittingTable[i].A1;
		v->NTCTable[i].A0 = PCUTable.AdcCurveFittingTable[i].A0;
	}

	uint16_t Index = 0;

	// Link temperature pointer to DMA
	uint16_t AdcGroupIndex = v->ThermoCh[v->ThermoTraIndex].AdcGroupIndex;
	uint16_t AdcRankIndex = v->ThermoCh[v->ThermoTraIndex].AdcRankIndex;
	for ( Index = 0; Index < THERMAL_CHANNEL_SIZE; Index++ )
	{
		if( v->ThermoCh[Index].ChEnable == ADC_ENABLE )
		{
			AdcGroupIndex = v->ThermoCh[Index].AdcGroupIndex;
			AdcRankIndex = v->ThermoCh[Index].AdcRankIndex;
			v->pTempADCValue[Index] = &(v->AdcDmaData[AdcGroupIndex][AdcRankIndex]);
		}
	}

	/*
	 * Throttle ADC filter Initial
	 */
	FILTER_INIT_BILINEAR_1ORDER_TYPE FilterSettingTmp = FILTER_INIT_BILINEAR_1ORDER_DEFAULT;
	FilterSettingTmp.BandwithHz = 10.0f;
	FilterSettingTmp.Period = 0.0001f;
	FilterSettingTmp.Type = FILTER_TYPE_LPF;
	Filter_Bilinear1OrderInit( &(v->ThrotAdcFilter), &FilterSettingTmp );
}

void AdcStation_ZeroCalibInjectionGroup( AdcStation *v )
{
	if(( v->ZeroCalibInjDone == 1 ) || ( v->ZeroCalibInjChCount == 0 ))
		return;

	uint16_t i = 0;

	for( i = 0; i < CURRENT_LOOP_CHANNEL_SIZE; i++ )
	{
		if( v->InjCh[i].ZeroEnable == ADC_ENABLE )
		{
			if( v->AdcRawData.Inj[i].ZeroCnt < ADC_ZEROCALIB_MAX_CNT )
			{
				v->AdcRawData.Inj[i].ZeroCnt++;
				v->AdcRawData.Inj[i].ZeroValue += v->AdcRawData.Inj[i].RawAdcValue;
			}
			else
			{
				v->AdcRawData.Inj[i].ZeroValue = v->AdcRawData.Inj[i].ZeroValue >> ADC_ZERO_CALIB_SHIFT_NUM;
				v->AdcExeZeroP[i] = v->AdcRawData.Inj[i].ZeroValue;
				v->AdcRawData.Inj[i].ZeroFinish = ADC_DONE;
				v->ZeroCalibInjChCount--;

				if( v->ZeroCalibInjChCount == 0 )
					v->ZeroCalibInjDone = 1;
			}
		}
	}
}

void AdcStation_ZeroCalibRegularGroup( AdcStation *v )
{
	if( ( v->ZeroCalibRegDone == 1 ) || ( v->ZeroCalibRegChCount == 0 ) )
		return;

	uint16_t i = 0;

	for( i = 0; i < PLC_LOOP_CHANNEL_SIZE; i++ )
	{
		if( v->RegCh[i].ZeroEnable == ADC_ENABLE )
		{
			if( v->AdcRawData.Reg[i].ZeroCnt < ADC_ZEROCALIB_MAX_CNT )
			{
				v->AdcRawData.Reg[i].ZeroCnt++;
				v->AdcRawData.Reg[i].ZeroValue += v->AdcDmaData[v->RegCh[i].AdcGroupIndex][v->RegCh[i].AdcRankIndex];
			}
			else
			{
				v->AdcRawData.Reg[i].ZeroValue = v->AdcRawData.Reg[i].ZeroValue >> ADC_ZERO_CALIB_SHIFT_NUM;
				v->AdcRawData.Reg[i].ZeroFinish = ADC_DONE;
				v->ZeroCalibRegChCount--;

				if( v->ZeroCalibRegChCount == 0 )
					v->ZeroCalibRegDone = 1;
			}
		}
	}
}

void AdcStation_ReadInjectionGroupValue( AdcStation *v, ADC_HandleTypeDef* hadc )
{
	uint16_t i = 0;

	for( i = 0; i < CURRENT_LOOP_CHANNEL_SIZE; i++ )
	{
		if( v->InjCh[i].ChEnable == ADC_ENABLE && v->InjCh[i].AdcGroup == hadc )
		{
			if( __HAL_ADC_GET_FLAG( hadc, ADC_FLAG_JEOS ) )
			{
				v->AdcState = ADC_READING;
				v->AdcRawData.Inj[i].RawAdcValue = HAL_ADCEx_InjectedGetValue( hadc, v->InjCh[i].AdcInjRank );
				v->AdcRawData.Inj[i].AdcReadDone = ADC_DONE;
			}
		}
	}
}

__attribute__(( section(".ram_function"))) void AdcStation_DoCurrentLoop( AdcStation *v )
{
	v->AdcTraOut.Iu[0] = (float)( v->AdcExeGain[ISE_U_A0].FDta * ( v->AdcRawData.Inj[ISE_U_A0].RawAdcValue - v->AdcExeZeroP[ISE_U_A0] ));
	v->AdcTraOut.Iv[0] = (float)( v->AdcExeGain[ISE_V_A0].FDta * ( v->AdcRawData.Inj[ISE_V_A0].RawAdcValue - v->AdcExeZeroP[ISE_V_A0] ));
	v->AdcTraOut.Iw[0] = (float)( v->AdcExeGain[ISE_W_A0].FDta * ( v->AdcRawData.Inj[ISE_W_A0].RawAdcValue - v->AdcExeZeroP[ISE_W_A0] ));
#if USE_VOLTAGE_CALIBRATION == USE_FUNCTION
	v->AdcTraOut.BatVdc = (float)( v->AdcExeGain[BAT_VDC].FDta  * ( v->AdcRawData.Inj[BAT_VDC].RawAdcValue - v->AdcExeZeroP[BAT_VDC] )) +0.5f;
#else
	v->AdcTraOut.BatVdc = (float)( v->AdcExeGain[BAT_VDC].FDta  * ( v->AdcRawData.Inj[BAT_VDC].RawAdcValue - v->AdcExeZeroP[BAT_VDC] ));
#endif
}

float AdcStation_DoThermoTransition( AdcStation *v )
{
	if( v->ThermoCh[v->ThermoTraIndex].ChEnable != ADC_ENABLE )
		return 0.0;

	float Result = 0.0;
	float Denominator =0.0;

	uint16_t TableIndex = v->ThermoCh[v->ThermoTraIndex].TableIndex;

	float Item3th = 0.0f;
	float Item2nd = 0.0f;
	float Item1st = 0.0f;

	float RawVoltage = v->ThermoADCCatchValue[v->ThermoTraIndex] * ADC_THERMAL_GAIN;

	Item1st = RawVoltage;
	Item2nd = Item1st * Item1st;
	Item3th = Item2nd * Item1st;

	Result = v->NTCTable[TableIndex].A6 + v->NTCTable[TableIndex].A5 * Item1st + v->NTCTable[TableIndex].A4 * Item2nd + v->NTCTable[TableIndex].A3 * Item3th;
	Denominator = 1 + v->NTCTable[TableIndex].A2 * Item1st + v->NTCTable[TableIndex].A1 * Item2nd + v->NTCTable[TableIndex].A0 * Item3th;
	Result =  Result / Denominator;

	return Result;
}

void AdcStation_DoNTCBreakReplaceStrategy( NTC_BREAK_REPLACE_t *v, uint16_t NTCIsBroken,float Current )
{
	if ( NTCIsBroken & (((uint16_t)1) << (v->NTCID)) )
	{
		if ( NTCIsBroken & (((uint16_t)1) << (v->ReplaceID)) )
		{
			//Replace NTC Is Broken, do nothing
		}
		else
		{
			(*v->Temperature) = (*v->ReplaceTemperature) - ( v->p1 * Current + v->p0 );
		}
	}
	else
	{
		//do nothing
	}
}

void AdcStation_DoPLCLoop( AdcStation *v )
{
	AdcStation_ZeroCalibRegularGroup(v);
	v->AdcTraOut.Pedal_V1    = (float)v->RegCh[ACC_FB1].GainValue  * (v->AdcDmaData[v->RegCh[ACC_FB1].AdcGroupIndex][v->RegCh[ACC_FB1].AdcRankIndex]);
	v->AdcTraOut.Pedal_V2    = (float)v->RegCh[ACC_FB2].GainValue  * (v->AdcDmaData[v->RegCh[ACC_FB2].AdcGroupIndex][v->RegCh[ACC_FB2].AdcRankIndex]);
	v->AdcTraOut.S13V8    = (float)v->RegCh[S13V8].GainValue  * (v->AdcDmaData[v->RegCh[S13V8].AdcGroupIndex][v->RegCh[S13V8].AdcRankIndex]);
	v->AdcTraOut.PreC    = (float)v->RegCh[PREC_FB].GainValue  * (v->AdcDmaData[v->RegCh[PREC_FB].AdcGroupIndex][v->RegCh[PREC_FB].AdcRankIndex]);
	v->AdcTraOut.EA5V    = (float)v->RegCh[EA5V_FB].GainValue  * (v->AdcDmaData[v->RegCh[EA5V_FB].AdcGroupIndex][v->RegCh[EA5V_FB].AdcRankIndex]);
	v->AdcTraOut.E5V    = (float)v->RegCh[E5V_FB].GainValue  * (v->AdcDmaData[v->RegCh[E5V_FB].AdcGroupIndex][v->RegCh[E5V_FB].AdcRankIndex]);
	v->AdcTraOut.ES5V    = (float)v->RegCh[ES5V_FB].GainValue  * (v->AdcDmaData[v->RegCh[ES5V_FB].AdcGroupIndex][v->RegCh[ES5V_FB].AdcRankIndex]);
	v->AdcTraOut.HwID1    = (float)v->RegCh[HW_ID1].GainValue  * (v->AdcDmaData[v->RegCh[HW_ID1].AdcGroupIndex][v->RegCh[HW_ID1].AdcRankIndex]);
	v->AdcTraOut.HwID2    = (float)v->RegCh[HW_ID2].GainValue  * (v->AdcDmaData[v->RegCh[HW_ID2].AdcGroupIndex][v->RegCh[HW_ID2].AdcRankIndex]);

	v->ThermoADCCatcher++;
	if( v->ThermoADCCatcher >= 10 ) //10ms
	{
		v->ThermoADCCatchValue[MOS_NTC_1] = (*v->pTempADCValue[MOS_NTC_1]);
		v->ThermoADCCatchValue[MOS_NTC_2] = (*v->pTempADCValue[MOS_NTC_2]);
		v->ThermoADCCatchValue[CAP_NTC] = (*v->pTempADCValue[CAP_NTC]);
		v->ThermoADCCatchValue[MOTOR_NTC_0_A0] = (*v->pTempADCValue[MOTOR_NTC_0_A0]);
		v->ThermoADCCatchValue[MOTOR_NTC_1_A0] = (*v->pTempADCValue[MOTOR_NTC_1_A0]);
		v->ThermoADCCatchValue[MOTOR_NTC_2_A0] = (*v->pTempADCValue[MOTOR_NTC_2_A0]);
		v->ThermoADCCatcher = 0;
	}

	v->ThrotADCRawRatio = ( v->AdcTraOut.EA5V > 1.0f ) ? v->AdcTraOut.Pedal_V1 / v->AdcTraOut.EA5V : v->AdcTraOut.Pedal_V1;

	float tempThrotADCRawRatio = v->ThrotADCRawRatio;
	if ( tempThrotADCRawRatio > v->AdcExeThrotMax )
	{
		tempThrotADCRawRatio = v->AdcExeThrotMax;
	}
	else if ( tempThrotADCRawRatio < v->AdcExeThrotZero )
	{
		tempThrotADCRawRatio = v->AdcExeThrotZero;
	}

	v->AdcTraOut.Throttle = v->AdcExeThrotGain.FDta * ( tempThrotADCRawRatio - v->AdcExeThrotZero );
}

void AdcStation_Do100HzLoop( AdcStation *v )
{
	v->ThermoTraIndex = MOS_NTC_1;
	v->AdcTraOut.PCU_NTC[MOS_NTC_1] = AdcStation_DoThermoTransition(v);
	v->ThermoTraIndex = MOS_NTC_2;
	v->AdcTraOut.PCU_NTC[MOS_NTC_2] = AdcStation_DoThermoTransition(v);
	v->ThermoTraIndex = CAP_NTC;
	v->AdcTraOut.PCU_NTC[CAP_NTC] = AdcStation_DoThermoTransition(v);
	v->ThermoTraIndex = MOTOR_NTC_0_A0;
	v->AdcTraOut.MOTOR_NTC_0 = AdcStation_DoThermoTransition(v);
	v->ThermoTraIndex = MOTOR_NTC_1_A0;
	v->AdcTraOut.MOTOR_NTC_1 = AdcStation_DoThermoTransition(v);
	v->ThermoTraIndex = MOTOR_NTC_2_A0;
	v->AdcTraOut.MOTOR_NTC_2 = AdcStation_DoThermoTransition(v);
}


