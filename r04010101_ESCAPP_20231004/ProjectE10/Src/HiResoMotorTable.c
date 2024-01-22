/*
 * HiResoMotorTable.c
 *
 *  Created on: 2023年12月27日
 *      Author: Jeff.Chang
 */

#include "HiResoMotorTable.h"

LUT_INT16_2DIM_TYPE HRIdCmdLut = LUT_INT16_2DIM_DEFAULT;
LUT_INT16_2DIM_TYPE HRIqCmdLut = LUT_INT16_2DIM_DEFAULT;

__attribute__((__section__(".HrMorotTable"),used)) const HR_MOTOR_TABLE_TYPE HRMotorTable =
{
	.HRIdCmdLutTable=
	{
			{0xFF}
	},

    .HRIdCmdLutInfo=
        {
    	.XLength = 220,
    	.XInterval = 0.1,
    	.XMin = 0,
    	.YLength = 135,
    	.YInterval = 1.489162,
    	.YMin = 148.9162,
    	.Scale = 0.015625,
    	.pTableStart = &HRMotorTable.HRIdCmdLutTable[0][0],
    },

	.HRIqCmdLutTable=
	{
		{0xFF}
	},

    .HRIqCmdLutInfo=
    {
       .XLength = 220,
       .XInterval = 0.1,
       .XMin = 0,
       .YLength = 135,
       .YInterval = 1.489162,
       .YMin = 148.9162,
       .Scale = 0.015625,
    	.pTableStart = &HRMotorTable.HRIqCmdLutTable[0][0],
    }
};


