/*
 * PositionCalibration.h
 *
 *  Created on: 2024年1月17日
 *      Author: User
 */

#ifndef INC_POSITIONCALIBRATION_H_
#define INC_POSITIONCALIBRATION_H_

#include "ParamMgr.h"
#include "PositionSensor.h"

#define DEFAULT_CURRENT_CMD_FOR_POS_CALI 500  //unit 0.1A
#define DEFAULT_DELAY_TIME_FOR_POSITIONING 1000 // uint: ms
#define ROTATE_STEPS_FOR_ELE_POS 500
#define ROTATE_ELE_POS_PER_MS ( _2PI * 10000.0f / (float)ROTATE_STEPS_FOR_ELE_POS )
#define MECH_ZERO_UP_BONDARY ( _2PI / DEFAULT_POLE_PAIRS / 2.0f )
#define MECH_ZERO_LOW_BONDARY ( _2PI - MECH_ZERO_UP_BONDARY)

typedef enum
{
	UP = 0,
	DOWN
}PS_CALI_ROTATE_DIR_e;

typedef enum
{
	PS_CALI_ZERO_SM_NONE = 0,
	PS_CALI_ZERO_SM_FIND_MECH_ZERO,
	PS_CALI_ZERO_SM_CAL_OFFSET,
	PS_CALI_ZERO_SM_SAVING_PARAM,
	PS_CALI_ZERO_SM_FINISHED,
	PS_CALI_ZERO_SM_ERROR
}PS_CALI_ZERO_OFFSET_SM_e;

typedef enum
{
	PS_CALI_LINEAR_SM_NONE = 0,
	PS_CALI_LINEAR_SM_FIND_MECH_ZERO,
	PS_CALI_LINEAR_SM_FIND_POINTS,
	PS_CALI_LINEAR_SM_SEND_RESULT,
	PS_CALI_LINEAR_SM_FINISHED,
	PS_CALI_LINEAR_SM_ERROR
}PS_CALI_LINEAR_SM_e;

typedef enum
{
	PS_CALI_FIND0_NONE = 0,
	PS_CALI_FIND0_ROTATING,
	PS_CALI_FIND0_POSITIONING,
	PS_CALI_FIND0_END,
	PS_CALI_FIND0_ERROR
}PS_CALI_FIND0_SM_e;

typedef enum
{
	PS_CALI_SEL_NONE = 0,
	PS_CALI_SEL_AUTO_ZERO_OFFSET,
	PS_CALI_SEL_LINEARIZATION,
}PS_CALI_SEL_e;

typedef struct
{
	PS_CALI_SEL_e Calibration_Metho_Select;
	PS_CALI_ZERO_OFFSET_SM_e Zero_Offset_State;
	PS_CALI_LINEAR_SM_e Linear_State;
	PS_CALI_FIND0_SM_e Find_Mech_Zero_State;
	PS_CALI_ROTATE_DIR_e Rotate_Direction;
}PS_CALI_t;

#define PS_CALI_DEFAULT {\
		PS_CALI_SEL_NONE,\
		PS_CALI_ZERO_SM_NONE,\
		PS_CALI_LINEAR_SM_NONE,\
		PS_CALI_FIND0_NONE,\
		UP,\
}

extern 	float LinearPointsMechPosRad[32];
extern void PositionCalibration_Routine(uint32_t *PosCaliSel, PS_t *u );
#endif /* INC_POSITIONCALIBRATION_H_ */
