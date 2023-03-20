/*
 * MfConstAndStruct.h
 *
 *  Created on: 2020年12月16日
 *      Author: Hank.Chen.CHC
 */

#ifndef INC_MF_CONST_AND_STRUCT_DEF_H_
#define INC_MF_CONST_AND_STRUCT_DEF_H_

#define MF_TORQUE_MODE			1
#define MF_CALIB_CURR_CTRL		6
#define MF_CALIB_THROT_MODE		8
#define MF_CALIB_VOLT_CTRL		10
#define PWM_PERIOD				10000
#define SHIFT_BIT				4
#define ACC_NUM					(0x01 << SHIFT_BIT)

enum MFStateEnum{
	MF_DISABLE = 0,
	MF_ENABLE
};

enum MFServoOnEnum{
	MF_SERVO_OFF = 0,
	MF_SERVO_ON
};

enum CalibCurrentEnum{
	MF_LOW = 0,
	MF_HIGH,
	MF_CLEAR = 0,
};

enum MfCurrentEnum{
	Ise_U = 0,
	Ise_V,
	Ise_W
};

enum MfSecurityAccess{
	Mfsa_EndUser = 0,
	Mfsa_VehicleDealer = 1,
	Mfsa_VehicleMf = 2,
	Mfsa_LscMf = 5,
	Mfsa_LscFAE = 6,
	Mfsa_LscRd = 7
};

enum MfAxisEnum{
	Axis_0_Mf = 0,
	Axis_1_Mf,
};

typedef struct{
	uint16_t	AxisNum: 		2;
	uint16_t    Phase:			6;
	uint16_t	POSorNEG: 		8;
}MF_CURR_CALIB_SETUP;

typedef struct{
	uint16_t Value[2];
}DTA_DEFINE;

#define DTA_DEFINE_DEFAULT { \
	{0, 0},	\
}\

typedef struct{
	DTA_DEFINE Ise[3];
}ADC_TYPE_DEFINE;

#define CURR_DEFINE_DEFAULT {	\
	{DTA_DEFINE_DEFAULT, DTA_DEFINE_DEFAULT, DTA_DEFINE_DEFAULT},	\
}	\

typedef struct{
	uint16_t Iu;
	uint16_t Iv;
	uint16_t Iw;
}ZERO_VAL_DEFINE;

#define ZERO_VAL_DEFINE_DEFAULT {	\
	0,	\
	0,	\
	0,	\
}	\

typedef union{
	float FDta;
	int16_t	Int16Type[2];
	uint16_t Uint16Type[2];
	int8_t Int8Type[4];
	uint8_t Uint8Type[4];
	int32_t Int32Type;
	uint32_t Uint32Type;
}GAIN_TYPE_DEFINE;

#define UNION_TYPE_DEFINE_DEFAULT {	\
	0.0f	\
}	\

typedef struct{
	GAIN_TYPE_DEFINE Ise[3];
}GAIN_DEFINE;

#define GAIN_DEFINE_DEFAULT {	\
	{UNION_TYPE_DEFINE_DEFAULT,	UNION_TYPE_DEFINE_DEFAULT, UNION_TYPE_DEFINE_DEFAULT}\
}	\

typedef struct{
	uint16_t DI1: 			1;
	uint16_t DI2: 			1;
	uint16_t DI3: 			1;
	uint16_t DI4:			1;
	uint16_t DI5:			1;
	uint16_t DI6:			1;
	uint16_t DI7:			1;
	uint16_t DI8:			1;
	uint16_t HWOCP_State:	1;
	uint16_t Hall_A:		1;
	uint16_t Hall_B:		1;
	uint16_t Hall_C:		1;
	uint16_t DO:			1;
	uint16_t Reserved:		3;
}BITS_DEF;

typedef union{
	BITS_DEF Bits;
	uint16_t AllBit;
}GPIO_UNION_DEF;

#define BITS_DEF_DEFAULT { \
		0,	\
		0,	\
		0,	\
		0,	\
		0,	\
		0,	\
		0,	\
		0,	\
		0,	\
		0,	\
		0,	\
		0,	\
		0,	\
		0,	\
}\

typedef struct{
	float  Iu[2];
	float  Iv[2];
	float  Iw[2];
	float  IuSum[2];
	float  IvSum[2];
	float  IwSum[2];
	float  IuBuf[2];
	float  IvBuf[2];
	float  IwBuf[2];
}RMS_DEF;

#define RMS_DEF_DEFAULT { \
	{0.0f, 0.0f}, \
	{0.0f, 0.0f}, \
	{0.0f, 0.0f}, \
	{0.0f, 0.0f}, \
	{0.0f, 0.0f}, \
	{0.0f, 0.0f}, \
	{0.0f, 0.0f}, \
	{0.0f, 0.0f}, \
	{0.0f, 0.0f}, \
}\

#endif /* INC_MF_CONST_AND_STRUCT_DEF_H_ */
