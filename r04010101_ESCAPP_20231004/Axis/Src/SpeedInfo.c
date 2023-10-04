/*
 * SpeedInfo.c
 *
 *  Created on: 2022年2月8日
 *      Author: Fernando.Wang.HHW
 */
#include "SpeedInfo.h"

uint16_t SpeedInfo_Init( SpeedInfo_t* p,float Polepair )
{
	int32_t PolepairTmp = (int16_t)Polepair;
	if( PolepairTmp <= 0 )
	{
		p->Polepair = 0.0f;
		p->DividePolepair = 1.0f;
		return 1;
	}
	else
	{
		p->Polepair = Polepair;
		p->DividePolepair = 1/(p->Polepair);
		return 0;
	}
}

