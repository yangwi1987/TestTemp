/*
 * MotorStall.h
 *
 *  Created on: 2020年8月17日
 *      Author: Mike.Wen.SFW
 */

#ifndef INC_MOTORSTALL_H_
#define INC_MOTORSTALL_H_

#include "stdio.h"

#define MOTORSTALL_BEGIN_ELEC_FREQ		13.33
#define MOTORSTALL_BEGIN_RPM			200

#define MST_ROW				19
#define MST_CLN				6

typedef void ( *functypeMotorStall_Init )( void * );
typedef void ( *functypeMotorStall_Calc )( void *, float, float );
typedef void ( *functypeMotorStall_Reset )( void * );

typedef struct /*__attribute__((__packed__))*/
{
	float reserve1;
	float reserve2;
	float reserve3;
	float reserve4;
	float reserve5;
	float reserve6;
	const int16_t *p2DTable;
} MotorReserveTableInfo_t;

typedef struct {
	uint16_t IsMotorStall;
	float LastAccHeatEnergy;
	float AccHeatEnergy;
	functypeMotorStall_Init Init;
	functypeMotorStall_Calc Calc;
	functypeMotorStall_Reset Reset;
} MotorStall_t;

void MotorStall_Init( MotorStall_t *v );
void MotorStall_Calc( MotorStall_t *v, float ACCurrent, float MotorRPM );
void MotorStall_Reset( MotorStall_t *v );

#define MOTORSTALL_DEFAULT \
{	0, \
	0.0f, \
	0.0f, \
	(functypeMotorStall_Init)MotorStall_Init, \
	(functypeMotorStall_Calc)MotorStall_Calc, \
	(functypeMotorStall_Reset)MotorStall_Reset }


#endif /* INC_MOTORSTALL_H_ */
