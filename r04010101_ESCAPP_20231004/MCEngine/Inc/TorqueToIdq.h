/*
 * TorqueToIdq.h
 *
 *  Created on: 2020年4月9日
 *      Author: Fernando.Wang.HHW
 */

#ifndef INC_TORQUETOIDQ_H_
#define INC_TORQUETOIDQ_H_

#include "Filter.h"
#include "HiResoMotorTable.h"

enum GET_IDQ_ENUM
{
	GET_IDQ_BY_LUT = 0x0000,
};

typedef void (*pfunTorqueToIdq_GetIdqCmd)( void*, float, float );
typedef void (*pfunTorqueToIdq_GetAllowFluxRec)( void*, float, float, float, float*, float*);

typedef struct
{
	uint16_t Way;
	float IdCmd;
	float IqCmd;
	float VbusGainForIcmd;
	float VbusGainForFW;
	float VbusGainForVsaturation;
	float VbusUsed;
	float VbusReal;
	float OutputTqRatio;
	LUT_INT16_2DIM_TYPE IdCmdLut;
	LUT_INT16_2DIM_TYPE IqCmdLut;
	LUT_INT16_1DIM_TYPE MaxTorqueLut;
	FILTER_BILINEAR_1ORDER_TYPE EleSpeedFilter;
	pfunTorqueToIdq_GetIdqCmd	GetIdqCmd;
	pfunTorqueToIdq_GetAllowFluxRec GetAllowFluxRec;
} TORQUE_TO_IDQ_TYPE;

#define TORQUE_TO_IDQ_DEFAULT				\
{											\
	GET_IDQ_BY_LUT,							\
	0.0f,/*IdCmd;                 */		\
	0.0f,/*IqCmd;                 */		\
	0.87f,/*VbusGainForIcmd;       */		\
	0.87f,/*VbusGainForFW;         */		\
	0.935f,/*VbusGainForVsaturation;*/		\
	0.0f,/*VbusUsed;              */		\
	0.0f,/*VbusReal;              */		\
	0.0f,/*OutputTqRatio;         */		\
	LUT_INT16_2DIM_DEFAULT,					\
	LUT_INT16_2DIM_DEFAULT,					\
	LUT_INT16_1DIM_DEFAULT,					\
	FILTER_BILINEAR_1ORDER_DEFAULT,			\
	(pfunTorqueToIdq_GetIdqCmd)TorqueToIdq_GetIdqCmd,					\
	(pfunTorqueToIdq_GetAllowFluxRec)TorqueToIdq_GetAllowFluxRec,		\
}

void TorqueToIdq_GetIdqCmd( TORQUE_TO_IDQ_TYPE *p, float TorqueTarget, float AllowFluxRec );
void TorqueToIdq_GetAllowFluxRec( TORQUE_TO_IDQ_TYPE *p, float EleSpeedInput, float Vbus, float MaxDuty, float *AllowFluxRec1, float *AllowFluxRec2 );

#endif /* INC_TORQUETOIDQ_H_ */
