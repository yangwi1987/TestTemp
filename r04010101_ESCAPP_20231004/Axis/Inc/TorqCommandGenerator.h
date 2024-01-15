/*
 * TorqueCommandGenerator.h
 *
 *  Created on: Dec 4, 2019
 *      Author: MikeSFWen
 */

#ifndef INC_TORQCOMMANDGENERATOR_H_
#define INC_TORQCOMMANDGENERATOR_H_

#include "MathFunctionInclude.h"
#include "Constant.h"
#include "FourQuadControl.h"

typedef void (*functypeTorqCommandGenerator_Calc)(void *, void * , float );

typedef struct {
	float DCLimitedTorqueCommand;
	float ACLimitedTorqueCommand;
	float Out;
	LUT_INT16_2DIM_TYPE AcCurrLimitLut;
	LUT_INT16_2DIM_TYPE DcCurrLimitLut;
	float AcCurrLimit;
	float DcCurrLimit;
	float AllowFluxRec;
	float MotorSpeed;
	float VbusUsed;
	float VbusReal;
	float TorqueCommandMax;
	float TorqueCommandmin;
	float TNTorqueAbs;
	float ACTorqueAbs;
	float DCTorqueAbs;
	functypeTorqCommandGenerator_Calc Calc;
} TorqCommandGenerator_t;

#define TORQ_COMMAND_GENERTATOR_DEFAULT {\
	0.0, \
	0.0, \
	0.0, \
	LUT_INT16_2DIM_DEFAULT,	\
	LUT_INT16_2DIM_DEFAULT,	\
	10000.0f,				\
	10000.0f,				\
	0.0f,					\
	0.0f, 					\
	0.0f,					\
	0.0f,					\
	50.0f, 					\
	0.0f, 					\
	0.0f, 					\
	0.0f, 					\
	0.0f, 					\
	(functypeTorqCommandGenerator_Calc)TorqCommandGenerator_Calc }

void TorqCommandGenerator_Calc( TorqCommandGenerator_t *v, FourQuadControl *p, float ThrottleCmd );
float TorqCommandGenerator_DCLimitation( TorqCommandGenerator_t *v );
float TorqCommandGenerator_ACLimitation( TorqCommandGenerator_t *v );

#endif /* INC_TORQCOMMANDGENERATOR_H_ */
