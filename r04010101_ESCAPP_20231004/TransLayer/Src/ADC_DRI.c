/*
 * ADC_DRI.c
 *
 *  Created on: Dec 30, 2019
 *      Author: Hank Chen
 */

/* Includes ------------------------------------------------------------------*/
#include "ADC_DRI.h"
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
				v->AdcRawData.Inj[i].RawAdcValue = HAL_ADCEx_InjectedGetValue( hadc, v->InjCh[i].AdcInjRank );
				v->AdcRawData.Inj[i].AdcReadDone = ADC_DONE;
			}
		}
	}
}


