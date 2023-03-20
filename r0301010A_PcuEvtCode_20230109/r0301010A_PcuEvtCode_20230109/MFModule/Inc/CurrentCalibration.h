/*
 * CurrentCalibration.h
 *
 *  Created on: 2020年9月4日
 *      Author: Hank.Chen.CHC
 */

#ifndef INC_CURRENTCALIBRATION_H_
#define INC_CURRENTCALIBRATION_H_

#include "ADC_DRI.h"
#include "MF_CONST_AND_STRUCT_DEF.h"

typedef void ( *functypeCurrCalib_RDModeAndAuth )( void *, uint16_t, uint16_t );
typedef void ( *functypeCurrCalib_ReadCmdInfo )( void *, uint32_t, uint32_t * );
typedef void ( *functypeCurrCalib_ReadCurrentAdc )( void *, void *, uint32_t * );
typedef void ( *functypeCurrCalib_CalbGainAndZeroPoint )( void *, uint32_t * );

typedef struct{
	uint16_t SelectPhase;
	uint16_t SelecetSide;
	uint16_t SelectAxis;
	uint16_t R_StartFlg;
	uint16_t Calib_StartFlag;
	uint16_t Authority;
	uint16_t CtrlMode;
	uint32_t AccumulatorADC;
	uint16_t AccumulatorCnt;
	ADC_TYPE_DEFINE AdcVal[2];
	ZERO_VAL_DEFINE Zero[2];
	GAIN_DEFINE CalibrationGain[2];
	functypeCurrCalib_RDModeAndAuth RDModeAndAuth;
	functypeCurrCalib_ReadCmdInfo ReadCmdInfo;
	functypeCurrCalib_ReadCurrentAdc RCurrentAdc;
	functypeCurrCalib_CalbGainAndZeroPoint CalibGainandZPoint;
}CURR_CALIB_DEFINE;

void ReadCtrlModeAndAuthority(CURR_CALIB_DEFINE *v, uint16_t Mode, uint16_t Auth);
void ReadCmdInfo( CURR_CALIB_DEFINE *v, uint32_t Setup, uint32_t *Enable );
void ReadCurrentAdcValue( CURR_CALIB_DEFINE *v, AdcStation *a, uint32_t *Enable );
void CalbGainAndZeroPoint( CURR_CALIB_DEFINE *v, uint32_t *CmdValue );

#define GURR_CALIB_DEFINE_DEFAULT {	\
	0, \
	0, \
	0, \
	0, \
	0, \
	0, \
	0, \
	0, \
	0, \
	{CURR_DEFINE_DEFAULT, CURR_DEFINE_DEFAULT},	\
	{ZERO_VAL_DEFINE_DEFAULT, ZERO_VAL_DEFINE_DEFAULT},	\
	{GAIN_DEFINE_DEFAULT, GAIN_DEFINE_DEFAULT},	\
	(functypeCurrCalib_RDModeAndAuth)ReadCtrlModeAndAuthority, \
	(functypeCurrCalib_ReadCmdInfo)ReadCmdInfo, \
	(functypeCurrCalib_ReadCurrentAdc)ReadCurrentAdcValue, \
	(functypeCurrCalib_CalbGainAndZeroPoint)CalbGainAndZeroPoint \
}	\

#endif /* INC_CURRENTCALIBRATION_H_ */
