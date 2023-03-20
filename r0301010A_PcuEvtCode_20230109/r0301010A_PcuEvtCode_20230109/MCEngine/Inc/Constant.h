/*
 * Constant.h
 *
 *  Created on: 2019年12月9日
 *      Author: Fernando
 */

#ifndef INC_CONSTANT_H_
#define INC_CONSTANT_H_
#include "stdint.h"

#define DIVIDE_2PI      0.15915493431f
#define _2PI 			6.283185307f
#define _PI				3.1415926536f
#define RAD_2_RPM       9.5492965855f
#define	INV_2PI			0.15915494309f
#define DEGREE_TO_RAD 	0.017453292f
#define RAD_TO_DEGREE	57.29577951f
#define RPM_TO_SPEED	0.104719755f
#define SPEED_TO_RPM	9.549296586f
#define SQRT3			1.7320508f
#define DIVIDE_SQRT3	0.57735026f
#define _2PI_OVER_6 	1.04719755f
#define DIVIDE__2PI_OVER_6	0.95492966f
#define WINDING_TEMP_COEFFICIENT_OF_RES	0.0039f

#define FUNCTION_DISABLE 	0
#define FUNCTION_ENABLE		1
#define FUNCTION_NO 		0
#define FUNCTION_YES		1

typedef union {
	float FloatType;
	int16_t Int16Type[2];
	uint16_t Uint16Type[2];
	int8_t Int8Type[4];
	uint8_t Uint8Type[4];
} DATA_TYPE_TRANSFER;

#endif /* INC_CONSTANT_H_ */
