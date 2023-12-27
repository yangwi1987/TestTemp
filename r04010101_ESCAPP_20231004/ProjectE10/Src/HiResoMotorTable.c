/*
 * HiResoMotorTable.c
 *
 *  Created on: 2023年12月27日
 *      Author: Jeff.Chang
 */

#include "HiResoMotorTable.h"

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

//__STATIC_FORCEINLINE float HiResoMotorTable_Get_Id ( float TorqueTargetAbs, float AllowFluxRec)
//{
//    uint16_t TorqueTargetIndex = 0;
//    uint16_t AllowFluxRecIndex = 0;
//
//    TorqueTargetIndex = (uint16_t)((TorqueTargetAbs - HRMotorTable.HRIdCmdLutInfo.XMin) / HRMotorTable.HRIdCmdLutInfo.XInterval);
//    AllowFluxRecIndex = (uint16_t)((AllowFluxRec- HRMotorTable.HRIdCmdLutInfo.YMin) / HRMotorTable.HRIdCmdLutInfo.YInterval) ;
//
//	return 	((float)HRMotorTable.HRIdCmdLutTable[TorqueTargetIndex][AllowFluxRecIndex]);
//}
//
//__STATIC_FORCEINLINE float HiResoMotorTable_Get_Iq ( float TorqueTargetAbs, float AllowFluxRec)
//{
//    uint16_t TorqueTargetIndex = 0;
//    uint16_t AllowFluxRecIndex = 0;
//
//    TorqueTargetIndex = (uint16_t)((TorqueTargetAbs - HRMotorTable.HRIqCmdLutInfo.XMin) / HRMotorTable.HRIqCmdLutInfo.XInterval);
//    AllowFluxRecIndex = (uint16_t)((AllowFluxRec- HRMotorTable.HRIqCmdLutInfo.YMin) / HRMotorTable.HRIqCmdLutInfo.YInterval) ;
//
//	return 	((float)HRMotorTable.HRIqCmdLutTable[TorqueTargetIndex][AllowFluxRecIndex]);
//}


