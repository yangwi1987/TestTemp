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

	const int16_t HRIdCmdLutTable[135][220];
	const int16_t HRIqCmdLutTable[135][220];
}HR_MOTOR_TABLE_TYPE;

extern const HR_MOTOR_TABLE_TYPE HRMotorTable;

__STATIC_FORCEINLINE float HiResoMotorTable_Get_Id ( float TorqueTargetAbs, float AllowFluxRec)
{
    uint16_t TorqueTargetIndex = 0;
    uint16_t AllowFluxRecIndex = 0;

    TorqueTargetIndex = (uint16_t)((TorqueTargetAbs - HRMotorTable.HRIdCmdLutInfo.XMin) / HRMotorTable.HRIdCmdLutInfo.XInterval);

    AllowFluxRecIndex = ( AllowFluxRec < HRMotorTable.HRIdCmdLutInfo.YMin ) ?
    		            (uint16_t)((AllowFluxRec- HRMotorTable.HRIdCmdLutInfo.YMin) / HRMotorTable.HRIdCmdLutInfo.YInterval) : 0;

	return 	((float)HRMotorTable.HRIdCmdLutTable[AllowFluxRecIndex][TorqueTargetIndex] * HRMotorTable.HRIdCmdLutInfo.Scale);
}

__STATIC_FORCEINLINE float HiResoMotorTable_Get_Iq ( float TorqueTargetAbs, float AllowFluxRec)
{
    uint16_t TorqueTargetIndex = 0;
    uint16_t AllowFluxRecIndex = 0;

    TorqueTargetIndex = (uint16_t)((TorqueTargetAbs - HRMotorTable.HRIqCmdLutInfo.XMin) / HRMotorTable.HRIqCmdLutInfo.XInterval);
    AllowFluxRecIndex = (uint16_t)((AllowFluxRec- HRMotorTable.HRIqCmdLutInfo.YMin) / HRMotorTable.HRIqCmdLutInfo.YInterval) ;

	return 	((float)HRMotorTable.HRIqCmdLutTable[AllowFluxRecIndex][TorqueTargetIndex] * HRMotorTable.HRIqCmdLutInfo.Scale);
}



#endif /* INC_HIRESOMOTORTABLE_H_ */
