/*
 * CordicMath.c
 *
 *  Created on: Mar 6, 2020
 *      Author: Fernando
 */
#include "MathFunctionInclude.h"

// no use now
void CordicMath_GetSinCosValue(SIN_COS_TYPE *p, float Angle)
{
	SIN_COS_UNION_TYPE DataTmp;
	int16_t AngleQ15;
	AngleQ15=(int16_t)(Angle*CORDIC_RAD_PERUNIT_GAIN+0.5f);
	AngleQ15=(Angle>=0.0f)?AngleQ15:AngleQ15-1;
	WRITE_REG(CORDIC->CSR, CORDIC_CONFIG_COSINE);
	LL_CORDIC_WriteData(CORDIC, 0x7FFF0000 + (uint32_t) AngleQ15);
	DataTmp.CordicData = LL_CORDIC_ReadData(CORDIC);
	p->SinValue=(float)(DataTmp.Q15Data.SinQ15)*RIGHT_SHIFT_Q15;
	p->CosValue=(float)(DataTmp.Q15Data.CosQ15)*RIGHT_SHIFT_Q15;

}
