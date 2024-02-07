/*
 * ParamMgr.c
 *
 *  Created on: Dec 4, 2019
 *      Author: MikeSFWen
 */

#include "ParamMgr.h"

uint32_t DriveFnRegs[PARAM_FN_REGS_SIZE];
#define Reserved7 7

/*
 * The value of index (ex FN_AUTHORITY) is defined in "ParamData.h"
 *
 */
const DriveFnRegsInfo_t DriveFnRegsInfo[PARAM_FN_REGS_SIZE] =
{
//	 Min,	Max,	Authority
	{0, 	20000, 	7}, 		// 0-FN_AUTHORITY
	{0, 	2, 		1}, 		// 1-FN_ENABLE
	{0, 	10,		1}, 		// 2-FN_MF_FUNC_SEL
	{0, 	5, 		6}, 		// 3-FN_RD_FUNC_SEL
	{0, 	100, 	6}, 		// 4-FN_THRO_COMMAND
	{32768, 33768, 	1}, 		// 5-FN_TORQ_COMMAND
	{0, 	32000, 	6}, 		// 6-FN_AC_CURR_LIMIT
	{0, 	32000, 	6}, 		// 7-FN_DC_CURR_LIMIT
	{0, 	200, 	6}, 		// 8-FN_MAX_TORQUE
	{0, 	10000, 	5}, 		// 9-FN_RPM_SLOPE_CMD
	{0, 	10000, 	5}, 		// 10-FN_RPM_GAIN_CMD
	{0, 	1, 		6}, 		// 11-FN_REINIT
	{0, 	1000, 	1}, 		// 12-FN_PARAM_SAVE_INDEX
	{0, 	2, 		1}, 		// 13-FN_PARAM_SAVE_AXISID
	{0, 	3, 		1},			// 14-FN_PARAM_SAVE_ADDR
	{0, 	32768, 	5}, 		// 15-FN_OPEN_SPD_COMMAND
	{0, 	1, 		1}, 		// 16-FN_PARAM_BACKUP_TO_EFLASH
	{0, 	10000, 	5},			// 17-FN_OPEN_SPD_V_I_LIMIT
	{0, 	511, 	6},			// 18-FN_FOUR_QUAD_STATES
	{0, 	2, 		5},			// 19-FN_PING_CAN1_2_COMMAND
	{0,     0,      5},			// 20-FN_MF_NA0 MF_REQ_PARAM_RESPONSE_0
	{0,     0,      5},  		// 21-FN_MF_NA1 MF_REQ_PARAM_RESPONSE_1
	{0,     0,      5},			// 22-FN_MF_NA2 MF_REQ_PARAM_RESPONSE_2
	{0, 	16, 	5}, 		// 23-FN_MF_CURR_CALIB_SETUP
	{0, 	1, 		5},			// 24-FN_MF_CURR_CALIB_START
	{0, 	1023, 	5},			// 25-FN_MF_CURR_CALC
	{0, 	1, 		5}, 		// 26-FN_FUNCTION_SELECT
	{0, 	1, 		5}, 		// 27-FN_FUNCTION_ACTIVATE
	{0, 	7, 		0}, 		// 28-FN_READ_VERSION_CONTROL
	{0, 	1, 		0},			// 29-FN_ALARM_ID_READ_SRC
	{0, 	0xFFFF, 5},			// 30-FN_ASIC_CTRL_ADDR_LOW
	{0, 	0xFFFF, 5}, 		// 31-FN_ASIC_CTRL_ADDR_HIGH
	{0, 	0xFFFF, 5}, 		// 32-FN_ASIC_CTRL_VALUE_LOW
	{0, 	0xFFFF, 5}, 		// 33-FN_ASIC_CTRL_VALUE_HIGH
	{0, 	0x0F, 	5},			// 34-FN_ASIC_CTRL_OPERATION_CODE
	{0, 	1, 		5},			// 35-FN_ASIC_CTRL_AXIS_NUM
	{0, 	2, 		1}, 		// 36-FN_THROT_CALIB_READ_AD
	{0, 	1, 		1},			// 37-FN_THROT_CALIB_CALC
	{0, 	0, 		Reserved7},	// 38
	{0, 	0, 		Reserved7},	// 39
	{0, 	0x0001, 5},			// 40-FN_PCU_SN_OPERATION
	{0, 	0xFFFF, 0},			// 41
	{0, 	0xFFFF, 0},			// 42
	{0, 	0xFFFF, 0},			// 43
	{0, 	0xFFFF, 0},			// 44
	{0, 	0xFFFF, 0},			// 45
	{0, 	0xFFFF, 0},			// 46
	{0, 	0xFFFF, 0},			// 47
	{0, 	0xFFFF, 0},			// 48
	{0, 	0xFFFF, 0},			// 49
	{0, 	0xFFFF, 0},			// 50
	{0, 	0, 		Reserved7},	// 51
	{0, 	1, 		2},			// 52-FN_PCU_RESET_OPERATION
	{0, 	1, 		5},			// 53-FN_FLASH_ID_SECTION_EREASE
	{0, 	0, 		Reserved7},	// 54
	{0, 	0, 		Reserved7},	// 55
	{0, 	0, 		Reserved7},	// 56
	{0, 	0, 		Reserved7}, // 57
	{0, 	0, 		Reserved7},	// 58
	{0, 	10, 	1},			// 59-FN_EXTFLASH_DATA_RST_SET
	{0, 	10, 	1},			// 60-FN_EXTFLASH_DATA_RST_ENA
	{0, 	2, 		5},			// 61-FN_MF_VOLT_CALIB_START
	{0, 	1, 		5},	        // 62-FN_ORIGIN_PARAM_BACKUP
	{0, 	0, 		Reserved7},	// 63
	{0, 	0, 		Reserved7},	// 64
	{0, 	0, 		Reserved7},	// 65
	{0, 	0, 		Reserved7},	// 66
	{0, 	0, 		Reserved7},	// 67
	{0, 	0, 		Reserved7},	// 68
	{0, 	0, 		Reserved7},	// 69
	{0, 	1, 		5},	        // 70 FN_OPEN_POSITION_CMD_ENABLE
	{0, 	62832, 		5},	    // 71 FN_OPEN_POSITION_CMD
	{27768, 37768, 		5},	// 72 FN_CURRENT_ID_CMD
	{27768, 37768, 		5},	// 73 FN_CURRENT_IQ_CMD
	{0, 	5000, 		5},	// 74 FN_CURRENT_IS_CMD
	{0, 	62832, 		5},	// 75 FN_CURRENT_THETA_CMD
	{0, 	10000, 		5},	// 76 FN_PWM_U_CMD
	{0, 	10000, 		5},	// 77 FN_PWM_V_CMD
	{0, 	10000, 		5},	// 78 FN_PWM_W_CMD
	{0, 	3, 		5},	// 79 FN_PWM_Mode
	{0, 	0, 		Reserved7},	// 80
	{0, 	0, 		Reserved7},	// 81
	{0, 	0, 		Reserved7}, // 82
	{0, 	0, 		Reserved7}, // 83
	{0, 	0, 		Reserved7}, // 84
	{0, 	0, 		Reserved7}, // 85
	{0, 	0, 		Reserved7}, // 86
	{0, 	0, 		Reserved7}, // 87
	{0, 	0, 		Reserved7}, // 88
	{0, 	0, 		Reserved7},	// 89
	{0, 	0, 		Reserved7},	// 90
	{0, 	0, 		Reserved7},	// 91
	{0, 	0, 		Reserved7},	// 92
	{0, 	0, 		Reserved7},	// 93
	{0, 	0, 		Reserved7},	// 94
	{0, 	0, 		Reserved7},	// 95
	{0, 	0, 		Reserved7},	// 96
	{0, 	0, 		Reserved7},	// 97
	{0, 	0, 		Reserved7},	// 98
	{0, 	65535, 	7},			// 99-DEBUG COMMAND
};

uint8_t ParamMgr_ReadEnableIndicator( ParamMgr_t *v, uint16_t PN )
{
	uint8_t Row = 0, Page = 0, Col = 0;
	uint8_t temp = 0;

	// error case
	if( PN < 0 )
	{
		return 0;
	}
	else if( PN < PN_PCU_BASE )
	{
		Page = 0;
		Row = (uint8_t)(PN >> R_SHIFT_TO_ROW);
		Col = (uint8_t)(PN % ROW_SIZE);
	}
	else if( PN < PN_MOT_BASE )
	{
		Page = 1;
		Row = (uint8_t)((PN - SYS_PARAM_SIZE) >> R_SHIFT_TO_ROW);
		Col = (uint8_t)(PN % ROW_SIZE);
	}

	// PN is out of readable region.
	else
	{
		return 0;
	}

	temp = *(v->pFlashParaReadEnableTable + (Page * ROW_NUM) + Row); // Get entire row value.
	temp = (temp >> Col) & EnableBitMask; // Shift to corresponding column and get enable bit.
	return temp;
}

void ParamMgr_TableIndicator( ParamMgr_t *v, uint16_t PN, uint16_t* index )
{
	if( PN < 0 )
	{
		return;
	}
	else if( PN < PN_PCU_BASE )
	{
		v->pParamTable = (ParamTableInfo_t *)SystemTable.SysParamTableInfoArray;
		*index = PN;
	}
	else if( PN < PN_MOT_BASE )
	{
		v->pParamTable = (ParamTableInfo_t *)PCUTable.PcuParamTableInfoArray;
		*index = PN - SYS_PARAM_SIZE;
	}
	else if( PN < PARAM_NUMBER_SIZE )
	{
		v->pParamTable = (ParamTableInfo_t *)MotorTable.MotEncParamTableInfoArray;
		*index = PN - SYS_PARAM_SIZE - PCU_PARAM_SIZE;
	}
	else
	{
		return;
	}
}

uint16_t ParamMgr_CheckParamRange ( ParamMgr_t *v, uint16_t index, uint16_t TempData )
{
	// Todo if size is not 16 bit, further implementation is needed.
	if ( (TempData < v->pParamTable[index].Min) || (TempData > v->pParamTable[index].Max) )
	{
		return v->pParamTable[index].Default;
	}
	else
	{
		return TempData;
	}
}

void ParamMgr_Init( ParamMgr_t *v, ExtFlash_t *pExtFlash )
{
	// Check if each size of different parameter regions in RAM is correct.
	_Static_assert( sizeof(DriveParams.SystemParams) == ( SYS_PARAM_SIZE * sizeof(uint16_t)), "SystemParams size is error" );
	_Static_assert( sizeof(DriveParams.PCUParams) == ( PCU_PARAM_SIZE * sizeof(uint16_t)), "PCUParams size is error" );
	_Static_assert( sizeof(DriveParams.MotorEncoderParams[0]) == ( MOT_PARAM_SIZE * sizeof(uint16_t)), "MotorEncoderParams size is error" );

	// Check if the size of flash read enable table is correct. (Total table size is equal to Pn0-00 ~ Pn3-99)
	_Static_assert( (((ROW_SIZE * ROW_NUM) * PAGE_NUM) == (PCU_PARAM_SIZE + SYS_PARAM_SIZE)), "flash read enable table is error" );

	// Check if PAGE_SIZE == SYS_PARAM_SIZE == PCU_PARAM_SIZE. see comment in ExtFlash.h
	_Static_assert( (SYS_PARAM_SIZE == PCU_PARAM_SIZE), "number of system parameters is not the same as number of PCU parameters" );

	uint16_t i, index;
	uint16_t TempData = 0;
	uint8_t IsParaReadEnable = 0;

	// clear function registers
	for( i = 0; i < PARAM_FN_REGS_SIZE; i++ )
	{
		DriveFnRegs[i] = 0;
	}

	// Load Parameters from ext. flash and check version.
	pExtFlash->LoadParam( pExtFlash );
	v->pFlashParaReadEnableTable = pExtFlash->pParaReadEnableTable;

	// Assign data to RAM from external flash or default value.
	// If there is no parameter backup alarm in external flash (even though warning exists), assign data to RAM.
	if ( pExtFlash->AlarmStatus == FLASHERROR_NONE )
	{
		for( i = 0; i < PARAM_NUMBER_SIZE; i++ )
		{
			IsParaReadEnable = ParamMgr_ReadEnableIndicator( v, i );
			ParamMgr_TableIndicator ( v, i, &index ); // i = PN number; index = array index in each table
			if ( v->pParamTable[index].pAddr != 0 ) // if the parameter index is not reserved.
			{
				if( (v->pParamTable[index].Property & PPD_READ_LOC) == READ_FROM_EXTFLASH &&
						IsParaReadEnable )
				{
					TempData = *((uint16_t*)(&pExtFlash->ParamPack.DriveParamsExtMemory) + i );
					*v->pParamTable[index].pAddr = ParamMgr_CheckParamRange ( v, index, TempData );
				}
				else
				{
					*v->pParamTable[index].pAddr = v->pParamTable[index].Default;
				}
			}
		}
	}
	// Use default value from constant table.
	else
	{
		for ( i = 0; i < PARAM_NUMBER_SIZE; i++ )
		{
			ParamMgr_TableIndicator ( v, i, &index );
			if ( v->pParamTable[index].pAddr != 0 ) // if the parameter index is not reserved.
			{
				*v->pParamTable[index].pAddr = v->pParamTable[index].Default;
			}
		}
	}
}

int32_t ParamMgr_ReadParam( ParamMgr_t *v, uint16_t AxisID, uint16_t PN, uint8_t *pError)
{

	if( AxisID < 0 || AxisID > MAX_AXIS_NUM )
	{
		if( pError != NULL )
		{
			*pError = PARAM_ACCESS_FAIL_WORNG_AXIS_ID;
		}
		return 0;
	}

	uint16_t index;

	ParamMgr_TableIndicator( v, PN, &index );

	int IsParamSizeLong = (v->pParamTable[index].Property & PPD_SIZE_LONG) ? 1 : 0;

	// To avoid empty address
	if( v->pParamTable[index].pAddr != 0 )
	{
		uint16_t *TempAddr = v->pParamTable[index].pAddr;
		if( pError != NULL )
		{
			*pError = PARAM_ACCESS_SUCCESS;
		}
		return IsParamSizeLong ? *(int32_t*)TempAddr : *TempAddr;
	}
	else
	{
		if( pError != NULL )
		{
			*pError = PARAM_ACCESS_FAIL_PARAM_ID_NOT_DEFINED;
		}
		return 0;
	}
}

int32_t ParamMgr_WriteParam( ParamMgr_t *v, uint16_t AxisID, uint16_t PN, int32_t value, uint8_t *pError )
{
	*pError = PARAM_ACCESS_SUCCESS;
	
	if( AxisID < 0 || AxisID > MAX_AXIS_NUM )
	{
		if( pError != NULL )
		{
			*pError = PARAM_ACCESS_FAIL_WORNG_AXIS_ID;
		}
		return 0;
	}

	uint16_t index;
	ParamMgr_TableIndicator( v, PN, &index );

	int IsParamSizeLong = (v->pParamTable[index].Property & PPD_SIZE_LONG) ? 1 : 0;

	// To avoid empty address
	if( v->pParamTable[index].pAddr != 0 ) {
		uint16_t *TempAddr = v->pParamTable[index].pAddr;
		// check authority
		if( v->Security < (v->pParamTable[index].Property & PPD_AUTHORITY_MASK) )
		{
			if( pError != NULL )
			{
				*pError = PARAM_ACCESS_FAIL_NO_AUTH;
			}
			return IsParamSizeLong ? *(int32_t*)TempAddr : *TempAddr;
		}

		// check min & max
		if( value > v->pParamTable[index].Max || value < v->pParamTable[index].Min ) {
			if( pError != NULL )
			{
				*pError = PARAM_ACCESS_FAIL_VALUE_NOT_AVAILABLE;
			}
			return IsParamSizeLong ? *(int32_t*)TempAddr : *TempAddr;
		} else {
			if( IsParamSizeLong ) {
				*(int32_t*)TempAddr = value;
			} else {
				*TempAddr = value;
			}
			v->OnParamValueChanged(AxisID, PN);
		}
		
		if( pError != NULL )
		{
			*pError = PARAM_ACCESS_SUCCESS;
		}
		return IsParamSizeLong ? *(int32_t*)TempAddr : *TempAddr;
	} else {
		if( pError != NULL )
		{
			*pError = PARAM_ACCESS_FAIL_PARAM_ID_NOT_DEFINED;
		}
		return 0;
	}
}

int32_t ParamMgr_ReadFnRegs( ParamMgr_t *v, uint16_t FN, uint8_t *pError )
{
	if( pError != NULL )
	{
		*pError = PARAM_ACCESS_SUCCESS;
	}
	return DriveFnRegs[FN - FN_BASE];
}

int32_t ParamMgr_WriteFnRegs( ParamMgr_t *v, uint16_t FN, uint32_t value, uint8_t *pError )
{
	uint16_t index = FN - FN_BASE;
	ParamAccessResult_e Result;

	if( v->Security < DriveFnRegsInfo[index].Security ){
		Result = PARAM_ACCESS_FAIL_NO_AUTH;
	}else if( (value >DriveFnRegsInfo[index].Max) || (value < DriveFnRegsInfo[index].Min) ){
		Result = PARAM_ACCESS_FAIL_VALUE_NOT_AVAILABLE;
	}else{
		DriveFnRegs[index] = value;
		Result = PARAM_ACCESS_SUCCESS;
	}

	if(pError!=NULL)
	{
		*pError =Result;
	}
	return DriveFnRegs[index];
}

uint16_t ParamMgr_ParaGainHandler( DriveParams_t *v, uint16_t *Var, float *Out )
{
	uint32_t Offset = Var - (&v->SystemParams.Reserved000);

	float Para = ((float)(*Var));
	uint16_t Property = 0;
	if( Offset < PN_PCU_BASE )
	{
		Property = SystemTable.SysParamTableInfoArray[Offset].Property;
	}
	else if( ( Offset >= PN_PCU_BASE ) &&( Offset < PN_MOT_BASE ) )
	{
		Property = PCUTable.PcuParamTableInfoArray[Offset-PN_PCU_BASE].Property;
	}
	else if( ( Offset >= PN_MOT_BASE ) &&( Offset < PN_UI_BASE ) )
	{
		Property = MotorTable.MotEncParamTableInfoArray[Offset-PN_MOT_BASE].Property;
	}
	else
	{
		return 1;
	}

	if ( ( Property & PPD_SHIFT_MASK ) == PPD_SHIFT_MASK )
	{
		Para -= PARA_VALUE_SHIFT;
	}
	uint16_t Decimal = ( Property & PPD_DECIMAL_MASK ) >> PPD_DECIMAL_SHIFT;
	if ( Decimal > 0 )
	{
		uint16_t i = 0;
		for( i = 0 ; i < Decimal ; i++ )
		{
			Para *= 0.1f;
		}
	}
	else
	{
		uint16_t Power10 = ( Property & PPD_POWER10_MASK ) >> PPD_POWER10_SHIFT;
		if( Power10 == 0 )
		{
			//do nothing
		}
		else
		{
			uint16_t i = 0;
			for( i = 0 ; i < Power10 ; i++ )
			{
				Para *= 10.0f;
			}
		}
	}
	*Out = Para;
	return 0;
}
