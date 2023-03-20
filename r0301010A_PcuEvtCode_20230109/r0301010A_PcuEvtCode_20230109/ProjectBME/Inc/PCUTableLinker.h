/*
 * PCUTableLinker.h
 *
 *  Created on: 2020年4月24日
 *      Author: Mike.Wen.SFW
 */
 
#if BME
#ifndef INC_PCUTABLELINKER_H_
#define INC_PCUTABLELINKER_H_

#include "ParamTable.h"
#include "ADC_Init_Table.h"
#include "PWM_Init_Table.h"

typedef struct
{
	uint16_t Version[4];
	uint16_t NumOfHeader;
	uint16_t CheckSum;
	uint16_t Reserverd[94];													// 200 Bytes
	ADC_GROUP_INFO AdcSetupTab[ ADC_GROUP_CNT ];							// 12 Bytes  <---- Alignment+2Bytes
	ADC_INJ_GROUP_TABLE AdcInjectionGroupTable[ CURRENT_LOOP_CHANNEL_SIZE ];// 72 Bytes
	ADC_DMA_GROUP_TABLE AdcRegularGroupTable[ PLC_LOOP_CHANNEL_SIZE ];		// 104 Bytes
	ADC_THERMO_TRA_TABLE AdcThermoTable[ THERMAL_CHANNEL_SIZE ];			// 24 Bytes  <---- Alignment+2Bytes
	NTC_CURVE_GAIN AdcCurveFittingTable[ NTC_TYPE_NUMBER ];					// 84 Bytes
	uint16_t Reserved_ADC[112];												// 226 Bytes <---- Compensate-4Bytes
	uint16_t Reserved_Enc[212];												// 424 Bytes
	PWM_START_TABLE_INFO PwmStartUpTable_Axis1[PWM_CH_SIZE];				// 12 Bytes
	PWM_TIM_INIT_PARA Pwm_Tim_Initializer_Axis1;							// 88 Bytes
	uint16_t Reserved_Pwm[128];												// 256 Bytes
	uint16_t Reserved_PcuTable[1298];										// 2596 Bytes + 48 Bytes
	ParamTableInfo_t PcuParamTableInfoArray[PCU_PARAM_SIZE];				// 4000 Bytes
	uint16_t Reserved_PcuParam[47];											// 94 Bytes
	uint16_t CheckWord;														// 2 Bytes
} PCU_Table_t_Linker;

extern const PCU_Table_t_Linker PCUTable;

#endif /* INC_PCUTABLELINKER_H_ */
#endif /* BME */
