/*
 * MotorTable.h
 *
 *  Created on: Feb 20, 2020
 *      Author: Fernando
 */

#if E10
#ifndef INC_MOTOR_INIT_TABLE_H_
#define INC_MOTOR_INIT_TABLE_H_

#include "Table_Type_Define.h"
#include "ParamTable.h"
#include "ConstantParamAndUseFunction.h"

//USER CODE END INCLUDE

typedef struct
{
	const uint16_t Version[4];
	const uint16_t NumOfHeader;
	const uint16_t CheckSum;

	//Header
	const HEADER_LUT_2DIM_int16_t_TYPE IdCmdHeader;
	const HEADER_LUT_2DIM_int16_t_TYPE IqCmdHeader;
	const HEADER_LUT_2DIM_int16_t_TYPE AcCurrLimitHeader;
	const HEADER_LUT_2DIM_int16_t_TYPE DcCurrLimitHeader;
	const HEADER_LUT_1DIM_int16_t_TYPE MaxTorqueHeader;
	uint16_t Reserved_Header[74];

	// Info
	const INFO_LUT_2DIM_int16_t_TYPE IdCmdLutInfo;
	const INFO_LUT_2DIM_int16_t_TYPE IqCmdLutInfo;
	const INFO_LUT_2DIM_int16_t_TYPE AcCurrLimitLutInfo;
	const INFO_LUT_2DIM_int16_t_TYPE DcCurrLimitLutInfo;
	const INFO_LUT_1DIM_int16_t_TYPE MaxTorqueLutInfo;
	
	// Table
	const int16_t IdCmdLutTable[40][15];
	const int16_t IqCmdLutTable[40][15];
	const int16_t AcCurrLimitLutTable[23][27];
	const int16_t DcCurrLimitLutTable[41][26];
	const int16_t MaxTorqueLutTable[21];
	const int16_t FdlutTable[14][40];
	const int16_t FqlutTable[14][40];
	uint16_t Reserved_Table[3952];


	// Param
	ParamTableInfo_t MotEncParamTableInfoArray[PCU_PARAM_SIZE];				// 4000 Bytes
	uint16_t Reserved_MotParam[47];											// 94 Bytes
	uint16_t CheckWord;														// 2 Bytes
} MOTOR_TABLE_TYPE;

extern const MOTOR_TABLE_TYPE MotorTable;

#endif /* INC_MOTOR_INIT_TABLE_H_ */
#endif /* E10 */
