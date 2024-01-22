/*
 * HiResoMotorTable.h
 *
 *  Created on: 2023年12月27日
 *      Author: Jeff.Chang
 */

#ifndef INC_HIRESOMOTORTABLE_H_
#define INC_HIRESOMOTORTABLE_H_

#include "Table_Type_Define.h"
#include "ParamTable.h"
#include "ConstantParamAndUseFunction.h"

typedef struct
{
	const INFO_LUT_2DIM_int16_t_TYPE HRIdCmdLutInfo;
	const INFO_LUT_2DIM_int16_t_TYPE HRIqCmdLutInfo;

	const int16_t HRIdCmdLutTable[136][205];
	const int16_t HRIqCmdLutTable[136][205];
}HR_MOTOR_TABLE_TYPE;

extern const HR_MOTOR_TABLE_TYPE HRMotorTable;
extern LUT_INT16_2DIM_TYPE HRIdCmdLut;
extern LUT_INT16_2DIM_TYPE HRIqCmdLut;

__STATIC_FORCEINLINE void HiResoMotorTable_Init ( void)
{
	HRIdCmdLut.Init(&HRIdCmdLut, &HRMotorTable.HRIdCmdLutInfo);
	HRIdCmdLut.pTableStart = &HRMotorTable.HRIdCmdLutTable[0][0];
	HRIqCmdLut.Init(&HRIqCmdLut, &HRMotorTable.HRIqCmdLutInfo);
	HRIqCmdLut.pTableStart = &HRMotorTable.HRIqCmdLutTable[0][0];
}

__STATIC_FORCEINLINE float HiResoMotorTable_Get_Id ( float TorqueTargetAbs, float AllowFluxRec)
{
//    uint16_t TorqueTargetIndex = 0;
//    uint16_t AllowFluxRecIndex = 0;
//
//    TorqueTargetIndex = (uint16_t)((TorqueTargetAbs - HRMotorTable.HRIdCmdLutInfo.XMin) / HRMotorTable.HRIdCmdLutInfo.XInterval);
//
//    AllowFluxRecIndex = ( AllowFluxRec < HRMotorTable.HRIdCmdLutInfo.YMin ) ?
//    		            (uint16_t)((AllowFluxRec- HRMotorTable.HRIdCmdLutInfo.YMin) / HRMotorTable.HRIdCmdLutInfo.YInterval) : 0;
//
//	return 	((float)HRMotorTable.HRIdCmdLutTable[AllowFluxRecIndex][TorqueTargetIndex] * HRMotorTable.HRIdCmdLutInfo.Scale);
    return (HRIdCmdLut.Calc(&HRIdCmdLut,TorqueTargetAbs,AllowFluxRec ));
}

__STATIC_FORCEINLINE float HiResoMotorTable_Get_Iq ( float TorqueTargetAbs, float AllowFluxRec)
{
//    uint16_t TorqueTargetIndex = 0;
//    uint16_t AllowFluxRecIndex = 0;
//
//    TorqueTargetIndex = (uint16_t)((TorqueTargetAbs - HRMotorTable.HRIqCmdLutInfo.XMin) / HRMotorTable.HRIqCmdLutInfo.XInterval);
//    AllowFluxRecIndex = (uint16_t)((AllowFluxRec- HRMotorTable.HRIqCmdLutInfo.YMin) / HRMotorTable.HRIqCmdLutInfo.YInterval) ;
//
//	return 	((float)HRMotorTable.HRIqCmdLutTable[AllowFluxRecIndex][TorqueTargetIndex] * HRMotorTable.HRIqCmdLutInfo.Scale);
    return (HRIqCmdLut.Calc(&HRIqCmdLut,TorqueTargetAbs,AllowFluxRec ));
}



#endif /* INC_HIRESOMOTORTABLE_H_ */
