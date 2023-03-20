/*
 * UiApp.h
 *
 *  Created on: 2020年10月21日
 *      Author: Fernando.Wang.HHW
 */

#ifndef INC_UIAPP_H_
#define INC_UIAPP_H_

#include "stdint.h"

typedef struct {
	uint16_t MfFunMode;
	uint16_t MotorServoOn;
	int16_t MotorCtrlMode;
} CTRL_UI_TYPE;

extern CTRL_UI_TYPE CtrlUi;



#endif /* INC_UIAPP_H_ */
