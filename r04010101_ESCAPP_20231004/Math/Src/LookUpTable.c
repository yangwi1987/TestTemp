/*
 * LookUpTable.c
 *
 *  Created on: 2019年12月18日
 *      Author: Fernando
 */
#include "MathFunctionInclude.h"


void Lut_FindIndexAndMod( LOOK_UP_TABLE_TYPE *p, float Input)
{
	float PerUnit = 0.0f;
	int16_t Index = 0;
	PerUnit = ( (Input - p->InputMin) * p->DevideInput );
	Index = ( PerUnit >= 0.0f ) ? (int16_t)PerUnit : -1 ;
	if ( Index >= p->MaxIndex)
	{
		p->Index1 = p->MaxIndex;
		p->Index2 = p->MaxIndex;
		p->Mod = 0.0f;
	}
	else if ( Index < 0)
	{
		p->Index1 = 0;
		p->Index2 = 0;
		p->Mod = 0.0f;
	}
	else
	{
		p->Index1 = Index;
		p->Index2 = p->Index1 + 1;
		p->Mod = PerUnit - (float)(p->Index1);
	}
}

uint16_t Lut_Int16TableParaInit( LOOK_UP_TABLE_TYPE *p, int16_t TableLength, float InputInterval, float InputMin)
{
	if ( TableLength < 2 )
	{
		return 	LOOK_UP_TABLE_INIT_ERROR_1DIM_TABLE_LENGTH_IS_LESS_THAN_2;
	}
	else if ( InputInterval <= 0.0f )
	{
		return 	LOOK_UP_TABLE_INIT_ERROR_1DIM_INPUT_INTERVAL_IS_INCORRECT;
	}
	else
	{
		p->MaxIndex = TableLength - 1;
		p->DevideInput = 1.0f / InputInterval;
		p->InputMin = InputMin;
		return LOOK_UP_TABLE_INIT_OK;
	}
}

uint16_t Lut_Int16Table1DimInit( LUT_INT16_1DIM_TYPE *p, const LUT_INIT_PARA_INT16_1DIM_TYPE *pSetting  )
{
	uint16_t Status;
	int16_t XLength;
	float XInterval;
	float XMin;
	float Scale;
	XLength = pSetting->Length;
	XInterval = pSetting->Interval;
	XMin = pSetting->Min;
	Scale = pSetting->Scale;

	Status = Lut_Int16TableParaInit( &(p->X), XLength, XInterval, XMin );
	p->pTableStart = pSetting->pTableStart;
	p->Scale = Scale;
	return Status;
}

float Lut_Int16Table1DimCalc( LUT_INT16_1DIM_TYPE *p, float Input)
{
	float Y1 = 0.0f;
	float Y2 = 0.0f;
	float Y = 0.0f;

	Lut_FindIndexAndMod( &(p->X), Input);

	Y1 = (float)(p->pTableStart[p->X.Index1]);
	Y2 = (float)(p->pTableStart[p->X.Index2]);
	Y = Y1 + ( Y2 - Y1 ) * p->X.Mod;
	Y = Y * p->Scale;
	return Y;
}

uint16_t Lut_Int16Table2DimInit( LUT_INT16_2DIM_TYPE *p,  const LUT_INIT_PARA_INT16_2DIM_TYPE *pSetting)
{
	uint16_t StatusX = 0;
	uint16_t StatusY = 0;
	int16_t XLength = 0;
	float XInterval = 0.0f;
	float XMin = 0.0f;
	uint16_t YLength = 0;
	float YInterval = 0.0f;
	float YMin = 0.0f;
	float Scale = 0.0f;

	XLength = pSetting->XLength;
	XInterval = pSetting->XInterval;
	XMin = pSetting->XMin;
	YLength = pSetting->YLength;
	YInterval = pSetting->YInterval;
	YMin = pSetting->YMin;
	Scale = pSetting->Scale;


	StatusX = Lut_Int16TableParaInit( &(p->X), XLength, XInterval, XMin );
	StatusY = Lut_Int16TableParaInit( &(p->Y), YLength, YInterval, YMin );
	p->pTableStart = pSetting->pTableStart;

	if ( StatusX == LOOK_UP_TABLE_INIT_ERROR_1DIM_TABLE_LENGTH_IS_LESS_THAN_2 )
	{
		return LOOK_UP_TABLE_INIT_ERROR_2DIM_X_LENGTH_IS_LESS_THAN_2;
	}
	else if ( StatusY == LOOK_UP_TABLE_INIT_ERROR_1DIM_TABLE_LENGTH_IS_LESS_THAN_2 )
	{
		return LOOK_UP_TABLE_INIT_ERROR_2DIM_Y_LENGTH_IS_LESS_THAN_2;
	}
	else if ( StatusX == LOOK_UP_TABLE_INIT_ERROR_1DIM_INPUT_INTERVAL_IS_INCORRECT )
	{
		return LOOK_UP_TABLE_INIT_ERROR_2DIM_X_INTERVAL_IS_INCORRECT;
	}
	else if ( StatusY == LOOK_UP_TABLE_INIT_ERROR_1DIM_INPUT_INTERVAL_IS_INCORRECT )
	{
		return LOOK_UP_TABLE_INIT_ERROR_2DIM_Y_INTERVAL_IS_INCORRECT;
	}
	else
	{
		p->Scale = Scale;
		return LOOK_UP_TABLE_INIT_OK;
	}
}

float Lut_Int16Table2DimCalc( LUT_INT16_2DIM_TYPE *p, float XInput, float YInput)
{
	float X1Y1 = 0.0f;
	float X1Y2 = 0.0f;
	float X2Y1 = 0.0f;
	float X2Y2 = 0.0f;
	float Z1 = 0.0f;
	float Z2 = 0.0f;
	float Z = 0.0f;
	uint16_t XLength = 0;
	uint16_t ShiftAddr = 0;

	//2us
	Lut_FindIndexAndMod( &(p->X), XInput);
	Lut_FindIndexAndMod( &(p->Y), YInput);

	//3us
	XLength = p->X.MaxIndex + 1;
	ShiftAddr = p->Y.Index1 * XLength + p->X.Index1;
	X1Y1 = (float)(*(p->pTableStart + ShiftAddr));
	ShiftAddr = p->Y.Index1 * XLength + p->X.Index2;
	X2Y1 = (float)(*(p->pTableStart + ShiftAddr));
	ShiftAddr = p->Y.Index2 * XLength + p->X.Index1;
	X1Y2 = (float)(*(p->pTableStart + ShiftAddr));
	ShiftAddr = p->Y.Index2 * XLength + p->X.Index2;
	X2Y2 = (float)(*(p->pTableStart + ShiftAddr));

	Z1 = X1Y1 + ( X2Y1 - X1Y1 ) * p->X.Mod;
	Z2 = X1Y2 + ( X2Y2 - X1Y2 ) * p->X.Mod;

	Z = Z1 + ( Z2 - Z1 ) * p->Y.Mod;
	Z = Z * p->Scale;
	return Z;
}


