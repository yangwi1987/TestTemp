/*
 * VoltageCalibration.h
 *
 *  Created on: 2022年5月30日
 *      Author: Hank.Chen.CHC
 */

#ifndef INC_VOLTAGECALIBRATION_H_
#define INC_VOLTAGECALIBRATION_H_

#include "ADC_DRI.h"
#include "MF_CONST_AND_STRUCT_DEF.h"

typedef uint16_t ( *functypeVoltCalib_ReadVoltageAdc )( void *, void *, uint32_t * );

typedef struct{
	uint16_t Calib_StartFlag;
	uint16_t TempVal;
	uint16_t CtrlMode;
	uint16_t AvgValue;
	uint32_t AccumulatorADC;
	uint16_t AccumulatorCnt;
	functypeVoltCalib_ReadVoltageAdc RVoltAdc;
}VOLT_CALIB_DEFINE;

uint16_t ReadVoltageAdcValue( VOLT_CALIB_DEFINE *v, AdcStation *a, uint32_t *Enable );

#define VOLT_CALIB_DEFINE_DEFAULT {	\
	0, \
	0, \
	0, \
	0, \
	0, \
	0, \
	(functypeVoltCalib_ReadVoltageAdc)ReadVoltageAdcValue, \
}	\

#endif /* INC_VOLTAGECALIBRATION_H_ */
