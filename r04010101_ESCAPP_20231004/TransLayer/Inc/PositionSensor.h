/*
 * PositionSensor.h
 *
 *  Created on: 2023年12月6日
 *      Author: User
 */

#ifndef INC_POSITIONSENSOR_H_
#define INC_POSITIONSENSOR_H_

#include "main.h"
#include "Constant.h"
#include "stm32g4xx_hal.h"
#include "MathFunctionInclude.h"

#define DEFAULT_ABZ_RESOLUTION_PER_MEC_REVOLUTION 4096 // resolution_pairs set as 0b0100
#define DEFAULT_POLE_PAIRS 5 //read from param int the future
#define DEFAULT_ABZ_RESOLUTION_PER_ELE_REVOLUSTION DEFAULT_ABZ_RESOLUTION_PER_MEC_REVOLUTION / DEFAULT_POLE_PAIRS //819.2 steps per elec revolution, which means 0.439 degrees

typedef void (*functypePositionSensor_Init)(void*);
typedef void (*functypePositionSesnor_DoPLCLoop)(void*);
typedef void (*functypePositionSensor_DoCurrentLoop)(void*);

typedef enum
{
	PS_SM_INIT_POSITION_SENSOR = 0,
	PS_SM_PROCESSING_READ_INITI_POSITION,
	PS_SM_PROCESSING_READ_OPERATING_POSITION,
	PS_SM_ERROR
}PS_SM_e;

typedef enum
{
	PS_DIRECTION_UPCOUNTER = 0,
	PS_DIRECTION_DOWNCOUNTER
}PS_DIR_e;

typedef struct
{
	PS_SM_e PositionSensor_StateMachine;
	uint32_t CntFromABZ;
	PS_DIR_e Direction;
	float MechSpeedRaw;     // rad/s
	float ElecSpeed;     // rad/s
	float MechSpeed;     // rad/s
	float ElecPosition;     // rad
	float MechPosition;     // rad
	float DutyFromPwm;
	float FreqFromPwm;
	float PreMechPosition;
	float InitMechPosition;
	float MechPosCompCoefBySpeed;
	FILTER_BILINEAR_1ORDER_TYPE CalcMechSpeedLPF;
	functypePositionSensor_Init Init;
	functypePositionSesnor_DoPLCLoop DoPLCLoop;
	functypePositionSensor_DoCurrentLoop DoCurrentLoop;
}PS_t;

#define PS_DEFAULT {\
	0,\
	0,\
	PS_DIRECTION_UPCOUNTER,\
	0.0f,\
	0.0f,\
	0.0f,\
	0.0f,\
	0.0f,\
	0.0f,\
	0.0f,\
	0.0f,\
	0.0f,\
	0.0f,\
	FILTER_BILINEAR_1ORDER_DEFAULT,\
	(functypePositionSensor_Init)PositionSensor_Init,\
	(functypePositionSesnor_DoPLCLoop)PositionSesnor_DoPLCLoop,\
	(functypePositionSensor_DoCurrentLoop)PositionSensor_ReadPosIdle,\
}

void PositionSensor_Init(PS_t* v);
void PositionSesnor_DoPLCLoop(PS_t* v);
void PositionSensor_DummyCurrentLoop(PS_t* v);
void PositionSensor_ReadPosViaABZ(PS_t* v);
void PositionSensor_ReadPosIdle(PS_t* v);
extern TIM_HandleTypeDef htim4;
extern __IO float uwDutyCycle;

#endif /* INC_POSITIONSENSOR_H_ */
