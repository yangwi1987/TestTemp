/*
 * SixWave.c
 *
 *  Created on: 2020年4月15日
 *      Author: Fernando.Wang.HHW
 */
#include "MotorControl.h"
const int16_t TestHallSectionTable[6] =	{ VHWL, WHUL, VHUL,  UHVL, UHWL,  WHVL};

void SixWaveHall_Degree( SIX_WAVE_HALL_TYPE *p, uint16_t Hall )
{
//	switch (Hall)
//	{
//		case  HALL_UH_VL_WH:
//			p->HallSection = UHWL;
//			break;
//		case  HALL_UH_VL_WL:
//			p->HallSection = VHWL;
//			break;
//		case  HALL_UH_VH_WL:
//			p->HallSection = VHUL;
//			break;
//		case  HALL_UL_VH_WL:
//			p->HallSection = WHUL;
//			break;
//		case  HALL_UL_VH_WH:
//			p->HallSection = WHVL;
//			break;
//		case  HALL_UL_VL_WH:
//			p->HallSection = UHVL;
//			break;
//		default:
//			break;
//	}
	if ( ( Hall > 6 ) || ( Hall < 1 ) )
	{
		// There is a signal bug, do nothing now.
	}
	else
	{
		if ( p->HallType == SIX_WAVE_HALL_TYPE_TEST)
		{
			p->HallSection = TestHallSectionTable[Hall-1];
		}
	}

}

void SixWaveCtrl120_CurrFb( SIX_WAVE_120_CURRENT_CONTROL_TYPE *p, uint16_t Section, float Iu, float Iv, float Iw )
{
	switch (Section)
	{
		case UHWL:
			p->CurrFb = -Iw;
			break;
		case VHWL:
			p->CurrFb = -Iw;
			break;
		case VHUL:
			p->CurrFb = -Iu;
			break;
		case WHUL:
			p->CurrFb = -Iu;
			break;
		case WHVL:
			p->CurrFb = -Iv;
			break;
		case UHVL:
			p->CurrFb = -Iv;
			break;
	}
}

void SixWaveCtrl120_Clean( SIX_WAVE_120_CURRENT_CONTROL_TYPE *p)
{
	p->CurrCmd = 0.0f;
	p->CurrFb = 0.0f;
	p->Regulator.Clean(&(p->Regulator));
}

uint16_t SixWaveCtrl120_Init( SIX_WAVE_120_CURRENT_CONTROL_TYPE *p, float Kp, float Ki, float Period)
{
	uint16_t Error=0;
	SixWaveCtrl120_Clean(p);
	Error = p->Regulator.Init( Period, Kp, Ki, 1.0f, 0.0f, &(p->Regulator) );
	return Error;
}

