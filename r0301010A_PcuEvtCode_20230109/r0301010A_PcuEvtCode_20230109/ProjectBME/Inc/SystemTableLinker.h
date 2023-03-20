/*
 * SystemTableLinker.h
 *
 *  Created on: 2020年4月24日
 *      Author: Mike.Wen.SFW
 */
 
#if BME
#ifndef INC_SYSTEMTABLELINKER_H_
#define INC_SYSTEMTABLELINKER_H_

#include "AlarmTable.h"
#include "FourQuadTable.h"
#include "ParamTable.h"
#include "MotorStall.h"
#include "ThermoStrategy.h"
#include "UtilityBase.h"

typedef struct /*__attribute__((__packed__))*/
{
	uint16_t Version[4];
	uint16_t NumOfHeader;
	uint16_t CheckSum;
	uint16_t Reserverd[94];												// 200 Bytes
	AlarmTableInfo_t AlarmTableInfo[MAX_ALARM_NUM];						// 768 Bytes
	uint16_t Reserved_Alarm[228];										// 456 Bytes
	BACK_ROLL_TABLE_TYPE BackRollTable;									// 8 Bytes
	uint16_t Reserved_FourQuad[308];									// 572 Bytes + 44Bytes
	MotorReserveTableInfo_t MotorReserveTableInfo;						// 26 Bytes (after padding: 28 bytes)
	int16_t ReservedTable[MST_ROW][MST_CLN];							// 228 Bytes
	const int16_t WindingDeratingTable[8];								// 16 Bytes
	const int16_t MosDeratingTable[12];									// 24 Bytes
	const int16_t CapDeratingTable[22];									// 44 Bytes
	LUT_INIT_PARA_INT16_1DIM_TYPE WindingDeratingInfo;					// 20 Bytes
	LUT_INIT_PARA_INT16_1DIM_TYPE MosDeratingInfo;						// 20 Bytes
	LUT_INIT_PARA_INT16_1DIM_TYPE CapDeratingInfo;						// 20 Bytes
	uint16_t Reserved_Thermo[230];										// 416 Bytes + 20 Bytes + 24 Bytes
	uint16_t Reserved_CAN[512];											// 1024 Bytes
	ParamTableInfo_t SysParamTableInfoArray[SYS_PARAM_SIZE];			// 4000 Bytes ( ideal: 18 * 2000 = 3600. after padding: 20 * 200 = 4000)
	uint16_t SysFunAllow[3];											// 6 Bytes
	uint16_t Reserved_SysParam[126];									// 272 Bytes - 20Bytes
	uint16_t CheckWord;													// 2 Bytes
} System_Table_t_Linker;

extern const System_Table_t_Linker SystemTable;
extern uint16_t SystemTable_FunEnable(uint16_t FunID);

#endif /* INC_SYSTEMTABLELINKER_H_ */
#endif /* BME */
