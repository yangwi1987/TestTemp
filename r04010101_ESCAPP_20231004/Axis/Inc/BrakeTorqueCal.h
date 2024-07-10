/*
 * BrakeTorqueCal.h
 *
 *  Created on: 2024年7月9日
 *      Author: Jeff C
 */

#ifndef INC_BRAKETORQUECAL_H_
#define INC_BRAKETORQUECAL_H_

#include "stdio.h"
#include "SystemTableLinker.h"

#define BREAK_PEDAL_PRESSED 1
#define BREAK_PEDAL_RELEASED 0
#define DEFAULT_OUTPUT_WHEN_FUNCITON_DISABLED BREAK_PEDAL_RELEASED

typedef void (*functypeBrakeTorqueCalc_Init)( void *, void * );
typedef int8_t (*functypeBrakeTorqueCalc_PickSignal)( void *, float , int8_t );
typedef void (*functypeBrakeTorqueCalc_ThrottleCalc)( void *, float *, int8_t );

typedef struct {
	uint8_t Enable;
	uint8_t IsUseAnalogInput;   //0 = use digital input; 1 = use analog input
	float AnalogThreshold;
	float OutputFactor;
	functypeBrakeTorqueCalc_Init Init;
	functypeBrakeTorqueCalc_PickSignal PickSignal;
	functypeBrakeTorqueCalc_ThrottleCalc ThrottleCalc;
}BrakeTorqueCalc_t;

#define BREAK_TORQUE_CALC_DEFAULT {\
	0,    \
	0,    \
	0.0f, \
	0.0f, \
	(functypeBrakeTorqueCalc_Init)BrakeTorqueCalc_Init,\
	(functypeBrakeTorqueCalc_PickSignal)BrakeTorqueCalc_PickSignal,\
	(functypeBrakeTorqueCalc_ThrottleCalc)BrakeTorqueCalc_ThrottleCalc,\
}


void BrakeTorqueCalc_Init( BrakeTorqueCalc_t *v, DriveParams_t *a );
int8_t BrakeTorqueCalc_PickSignal ( BrakeTorqueCalc_t *v, float AnalotIn, int8_t DigitalIn );
void BrakeTorqueCalc_ThrottleCalc( BrakeTorqueCalc_t *v, float *ThrottlePercentageTarget, int8_t BrakePedal );


#endif /* INC_BRAKETORQUECAL_H_ */
