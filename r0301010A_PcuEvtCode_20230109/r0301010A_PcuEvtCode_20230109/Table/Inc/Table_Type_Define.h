/*
 * Table_Type_Define.h
 *
 *  Created on: Feb 20, 2020
 *      Author: Fernando
 */

#ifndef INC_TABLE_TYPE_DEFINE_H_
#define INC_TABLE_TYPE_DEFINE_H_

#include "stdint.h"
#include "Constant.h"

#include "MathFunctionInclude.h"

typedef LUT_INIT_PARA_INT16_1DIM_TYPE INFO_LUT_1DIM_int16_t_TYPE;
typedef LUT_INIT_PARA_INT16_2DIM_TYPE INFO_LUT_2DIM_int16_t_TYPE;

enum TABLE_TYPE_ENUM
{
		TABLE_TYPE_STRUCT_SINGLE=0,
		TABLE_TYPE_STRUCT_ARRAY,
		TABLE_TYPE_LUT_INT16_1DIM,
		TABLE_TYPE_LUT_INT16_2DIM,
};

typedef struct
{
	const uint16_t Type;
	const INFO_LUT_1DIM_int16_t_TYPE *Para;
}HEADER_LUT_1DIM_int16_t_TYPE;

typedef struct
{
	const uint16_t Type;
	const INFO_LUT_2DIM_int16_t_TYPE *Para;
}HEADER_LUT_2DIM_int16_t_TYPE;



#endif /* INC_TABLE_TYPE_DEFINE_H_ */
