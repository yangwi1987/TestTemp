/*
 * Polling.c
 *
 *  Created on: 2021年6月17日
 *      Author: Fernando.Wang.HHW
 */

#include "Polling.h"

#define POLLING_CONDITION_JUDGE_MACRO( Way, Value, Level, JudgmentTmp ) \
	switch(Way)											\
	{													\
	case POLLING_WAY_GREATER :							\
		JudgmentTmp = ( Value > Level ) ? 1 : 0;		\
		break;											\
	case POLLING_WAY_GREATER_OR_EQUAL :					\
		JudgmentTmp = ( Value >= Level ) ? 1 : 0;		\
		break;											\
	case POLLING_WAY_LESS :								\
		JudgmentTmp = ( Value < Level ) ? 1 : 0;		\
		break;											\
	case POLLING_WAY_LESS_OR_EQUAL :					\
		JudgmentTmp = ( Value <= Level ) ? 1 : 0;		\
		break;											\
	default:											\
		JudgmentTmp = 0;								\
		break;											\
	}													\

#define POLLING_CNT_MACRO( p, JudgmentTmp, CntTmp, AddTmp, SubTmp ) 				\
	CntTmp = ( JudgmentTmp == 1 ) ? ( CntTmp + AddTmp ) : ( CntTmp - SubTmp );		\
	CntTmp = ( CntTmp < 0 ) ? 0 : CntTmp;											\
	CntTmp = ( CntTmp > p->MaxCnt ) ? p->MaxCnt : CntTmp;							\
	p->Cnt = CntTmp;


void Polling_Init1Tolereance1Level( Polling1T1L_t *p, uint16_t Way, float Level, uint16_t Add, uint16_t Sub, uint16_t MaxCnt, uint16_t Tolerance )
{
	p->Level = Level;
	p->Way = Way;
	p->Add = Add;
	p->Sub = Sub;
	p->Tolerance = Tolerance;
	p->MaxCnt = MaxCnt;
}

void Polling_Detect1Tolereance1Level( Polling1T1L_t *p, float Value )
{
	uint16_t Judgment = 0;
	POLLING_CONDITION_JUDGE_MACRO( p->Way, Value, p->Level, Judgment )

	int32_t Cnt = p->Cnt;
	int32_t Add = p->Add;
	int32_t Sub = p->Sub;
	POLLING_CNT_MACRO( p, Judgment, Cnt, Add, Sub )

	if ( p->Cnt >= p->Tolerance)
	{
		p->Result = POLLING_RESULT_TOLERANCE1;
	}
	else
	{
		p->Result = POLLING_RESULT_NONE;
	}

}
