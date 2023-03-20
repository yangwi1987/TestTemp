/*
 * VoltageCalib.c
 *
 *  Created on: May 30, 2022
 *      Author: Hank.Chen.CHC
 */


#include "VoltageCalibration.h"

#if BME
uint16_t ReadVoltageAdcValue( VOLT_CALIB_DEFINE *v, AdcStation *a, uint32_t *Enable )
{
	if( *Enable > 0 )
	{
		v->Calib_StartFlag = *Enable - 1;
		v->AccumulatorADC += a->AdcRawData.Inj[ BAT_VDC ].RawAdcValue;
		v->AccumulatorCnt++;

		if( v->AccumulatorCnt == ACC_NUM )
		{
			v->AvgValue = ( uint16_t )( v->AccumulatorADC >> SHIFT_BIT );
			v->AccumulatorADC = MF_CLEAR;
			v->AccumulatorCnt = MF_CLEAR;
			*Enable = MF_DISABLE;
			return v->AvgValue;
		}
		else
		{
			return v->AvgValue;
		}
	}
	else
	{
		return v->AvgValue;
	}
}
#endif
