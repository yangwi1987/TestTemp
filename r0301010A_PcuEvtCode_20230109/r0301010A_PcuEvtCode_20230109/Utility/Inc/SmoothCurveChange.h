/*
 * SmoothCurveChange.h
 *
 *  Created on: 2021年4月18日
 *      Author: Fernando.Wang.HHW
 */

#ifndef INC_SMOOTHCURVECHANGE_H_
#define INC_SMOOTHCURVECHANGE_H_

#include "stdint.h"
#include "MathFunctionInclude.h"

extern float SmoothCurveChange( float TargetNow, float ResultPrevious, uint8_t *CurveNow, uint8_t *CurvePrevious, float Rising, float Falling );

#endif /* INC_SMOOTHCURVECHANGE_H_ */
