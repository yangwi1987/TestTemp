/*
 * LookUpTable.h
 *
 *  Created on: 2019年12月18日
 *      Author: Fernando
 */

#ifndef INC_LOOKUPTABLE_H_
#define INC_LOOKUPTABLE_H_

enum LOOK_UP_TABLE_INIT_STATUS_ENUM
{
	LOOK_UP_TABLE_INIT_OK = 0x0000,
	LOOK_UP_TABLE_INIT_ERROR_1DIM_TABLE_LENGTH_IS_LESS_THAN_2 = 0x8001,
	LOOK_UP_TABLE_INIT_ERROR_1DIM_INPUT_INTERVAL_IS_INCORRECT,
	LOOK_UP_TABLE_INIT_ERROR_2DIM_X_LENGTH_IS_LESS_THAN_2,
	LOOK_UP_TABLE_INIT_ERROR_2DIM_X_INTERVAL_IS_INCORRECT,
	LOOK_UP_TABLE_INIT_ERROR_2DIM_Y_LENGTH_IS_LESS_THAN_2,
	LOOK_UP_TABLE_INIT_ERROR_2DIM_Y_INTERVAL_IS_INCORRECT,
};


typedef uint16_t (*pInt16Table1DimInit)( void *, const void * );
typedef float (*pInt16Table1DimCalc)( void *, float );
typedef uint16_t (*pInt16Table2DimInit)( void *, const void * );
typedef float (*pInt16Table2DimCalc)( void *, float, float );

// Information of X axis in the look up table
typedef struct
{
	float InputMin; // min X value
	uint16_t Index1;
	uint16_t Index2;
	float Mod;
	uint16_t MaxIndex;
	float DevideInput; // reciprocal of the interval of X points
} LOOK_UP_TABLE_TYPE;

typedef struct
{
	LOOK_UP_TABLE_TYPE X;
	const int16_t *pTableStart;
	float Scale; // the multiplier of Y value after look up table
	pInt16Table1DimInit Init;
	pInt16Table1DimCalc Calc;
} LUT_INT16_1DIM_TYPE;

typedef struct
{
	LOOK_UP_TABLE_TYPE X;
	LOOK_UP_TABLE_TYPE Y;
	float Scale;
	const int16_t *pTableStart;
	pInt16Table2DimInit Init;
	pInt16Table2DimCalc Calc;
} LUT_INT16_2DIM_TYPE;

typedef struct
{
	int16_t Length; // number of X
	float Interval; // interval of X points
	float Min; // min X value
	float Scale;  // the multiplier of Y value after look up table
	const int16_t *pTableStart;
} LUT_INIT_PARA_INT16_1DIM_TYPE;

typedef struct
{
	int16_t XLength;
	float XInterval;
	float XMin;
	int16_t YLength;
	float YInterval;
	float YMin;
	float Scale;
	const int16_t *pTableStart;
} LUT_INIT_PARA_INT16_2DIM_TYPE;


uint16_t Lut_Int16Table1DimInit( LUT_INT16_1DIM_TYPE *p, const LUT_INIT_PARA_INT16_1DIM_TYPE *pSetting );
float Lut_Int16Table1DimCalc( LUT_INT16_1DIM_TYPE *p, float Input);
uint16_t Lut_Int16Table2DimInit( LUT_INT16_2DIM_TYPE *p,  const LUT_INIT_PARA_INT16_2DIM_TYPE *pSetting );
float Lut_Int16Table2DimCalc( LUT_INT16_2DIM_TYPE *p, float XInput, float YInput);

#define LOOK_UP_TABLE_DEFAULT	\
{								\
	0.0f,						\
	0U,							\
	0U,							\
	0.0f,						\
	1U,							\
	0.0f						\
}

#define LUT_INT16_1DIM_DEFAULT	\
{								\
	LOOK_UP_TABLE_DEFAULT,		\
	0U,							\
	0.0f,						\
	(pInt16Table1DimInit)Lut_Int16Table1DimInit,	\
	(pInt16Table1DimCalc)Lut_Int16Table1DimCalc		\
}

#define LUT_INT16_2DIM_DEFAULT 	\
{								\
	LOOK_UP_TABLE_DEFAULT,		\
	LOOK_UP_TABLE_DEFAULT,		\
	0.0f,						\
	0U,							\
	(pInt16Table2DimInit)Lut_Int16Table2DimInit,		\
	(pInt16Table2DimCalc)Lut_Int16Table2DimCalc		\
}


#endif /* INC_LOOKUPTABLE_H_ */
