/*
 * MathFunction.h
 *
 *  Created on: 2019年12月4日
 *      Author: Fernando
 */

#ifndef INC_MATHFUNCTIONINCLUDE_H_
#define INC_MATHFUNCTIONINCLUDE_H_

#include "stdint.h"
#include "math.h"

#define RIGHT_SHIFT_Q15 0.00003051757813f //>>15 = *0.00003051757813f
#define INVERSE(x) 1.0f/(x)

#include "CordicMath.h"
#include "Pid.h"
#include "Ramp.h"
#include "LookUpTable.h"
#include "Filter.h"


#endif /* INC_MATHFUNCTIONINCLUDE_H_ */
