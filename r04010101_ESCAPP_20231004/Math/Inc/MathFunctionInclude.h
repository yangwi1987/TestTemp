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
#define LIM_MIN(X, MIN)     (((X)>(MIN)) ? (X) : (MIN))
#define LIM_MAX(X, MAX)     (((X)<(MAX)) ? (X) : (MAX))
#define LIMIT(X, MIN, MAX)  (((X)>(MIN)) ? (((X)<(MAX)) ? (X) : (MAX)) : (MIN))
#define MIN2(A, B)           (((A)<(B)) ? (A) : (B))
#define MAX2(A, B)           (((A)>(B)) ? (A) : (B))
#define ABS(x) 	( (x) > 0 ? (x) : -(x) )
#define MAX3(x,y,z)   (( (x > y) ? x : y ) > z ? ( x > y ? x : y ) : z)
#define MIN3(a,b,c)  (((( a > b ) ? b : a) > c ) ? c : ( a > b ? b : a ))


#include "CordicMath.h"
#include "Pid.h"
#include "Ramp.h"
#include "LookUpTable.h"
#include "Filter.h"
#include "Sin_LT.h"

#endif /* INC_MATHFUNCTIONINCLUDE_H_ */
