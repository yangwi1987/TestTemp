/*
 * MFCodeInc.c
 *
 *  Created on: 2020年8月5日
 *      Author: Hank.Chen.CHC
 */


#include "MFCodeInc.h"


void MFFunc_CalMaxAvgCnt( MFStation *p, uint32_t SpdCmd, uint16_t Authority )
{
	if( ( Authority >= Mfsa_LscMf  ) && ( SpdCmd > 0 ) )
	{
		if( p->MaxAvgCntCalDone == MF_DISABLE )
		{
			p->MaxAvgCnt = ( (float)PWM_PERIOD/ ( (float)SpdCmd/ 60.0f * 4.0f ) )+0.5f;
			p->MaxAvgCntCalDone = MF_ENABLE;
		}else;
	}else;
}

void MFFunc_CalSumRoot(MFStation *p, AdcStation *v, uint16_t Authority )
{
	uint16_t i = 0;
	if( Authority >= Mfsa_LscMf )
	{
		if( p->MaxAvgCnt > 0  )
		{
			if( p->RmsCnt < p->MaxAvgCnt )
			{
				for( i= 0; i< 2; i++ )
				{
					p->CurrRms.IuSum[i] += ( v->AdcTraOut.Iu[i] * v->AdcTraOut.Iu[i] );
					p->CurrRms.IvSum[i] += ( v->AdcTraOut.Iv[i] * v->AdcTraOut.Iv[i] );
					p->CurrRms.IwSum[i] += ( v->AdcTraOut.Iw[i] * v->AdcTraOut.Iw[i] );
				}
				p->RmsCnt++;
			}
			else
			{
				for( i= 0; i< 2; i++ )
				{
					p->CurrRms.IuBuf[i] = p->CurrRms.IuSum[i];
					p->CurrRms.IvBuf[i] = p->CurrRms.IvSum[i];
					p->CurrRms.IwBuf[i] = p->CurrRms.IwSum[i];
					p->CurrRms.IuSum[i] = 0.0f;
					p->CurrRms.IvSum[i] = 0.0f;
					p->CurrRms.IwSum[i] = 0.0f;
				}
				p->RmsCnt = 0;
			}
		}else;
	}else;
}

void MFFunc_RootMeanSquare( MFStation *p, uint16_t Authority )
{
	uint16_t i;
	if( Authority >= Mfsa_LscMf )
	{
		if( p->MaxAvgCntCalDone == MF_ENABLE )
		{
			for( i= 0; i< 2; i++ )
			{
				p->CurrRms.Iu[i] = sqrt( p->CurrRms.IuBuf[i]/ (float)p->MaxAvgCnt );
				p->CurrRms.Iv[i] = sqrt( p->CurrRms.IvBuf[i]/ (float)p->MaxAvgCnt );
				p->CurrRms.Iw[i] = sqrt( p->CurrRms.IwBuf[i]/ (float)p->MaxAvgCnt );
			}
		} else;
	} else;
}


void MFFunc_GpioReadInformation( MFStation *p)
{
//	p->MFGpioInfo.Bits.DI1 = HAL_GPIO_ReadPin( SAFTYSSR_GPIO_Port, SAFTYSSR_Pin );
//	p->MFGpioInfo.Bits.DI2 = HAL_GPIO_ReadPin( FOIL_DI2_GPIO_Port, FOIL_DI2_Pin);
//	p->MFGpioInfo.Bits.DI3 = HAL_GPIO_ReadPin( FOIL_DI3_GPIO_Port, FOIL_DI3_Pin);
//	p->MFGpioInfo.Bits.HWOCP_State 	= HAL_GPIO_ReadPin( HWOCP_GPIO_Port, HWOCP_Pin);
}
