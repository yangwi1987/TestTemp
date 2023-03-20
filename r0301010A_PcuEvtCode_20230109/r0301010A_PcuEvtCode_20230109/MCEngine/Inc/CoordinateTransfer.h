/*
 * CoordinateTransfer.h
 *
 *  Created on: 2019年12月2日
 *      Author: Fernando
 */

#ifndef INC_COORDINATETRANSFER_H_
#define INC_COORDINATETRANSFER_H_

typedef void (*pPhaseToStator)( float, float, float, void*);
typedef void (*pStatorToRotor)( float, float, float, void*);
typedef void (*pRotorToStator)( float, float, float, void*);
typedef void (*pStatorToPhase)( float, float, void*);
typedef void (*pPhaseToStatorClean)( void*);
typedef void (*pStatorToRotorClean)( void*);
typedef void (*pRotorToStatorClean)( void*);
typedef void (*pStatorToPhaseClean)( void*);


typedef struct
{
	float Alpha;
	float Beta;
	pPhaseToStator Calc;
	pPhaseToStatorClean Clean;
} PHASE_TO_STATOR_TYPE;

typedef struct
{
	float D;
	float Q;
	pStatorToRotor Calc;
	pStatorToRotorClean Clean;
} STATOR_TO_ROTOR_TYPE;

typedef struct
{
	float Alpha;
	float Beta;
	pRotorToStator Calc;
	pRotorToStatorClean Clean;
} ROTOR_TO_STATOR_TYPE;

typedef struct
{
	float U;
	float V;
	float W;
	pStatorToPhase Calc;
	pStatorToPhaseClean Clean;
} STATOR_TO_PHASE_TYPE;

void CoordinateTransfer_PhaseToStator( float U, float V, float W, PHASE_TO_STATOR_TYPE* p);
void CoordinateTransfer_StatorToRotor( float Alpha, float Beta, float Angle, STATOR_TO_ROTOR_TYPE* p);
void CoordinateTransfer_RotorToStator( float D, float Q, float Angle, ROTOR_TO_STATOR_TYPE* p);
void CoordinateTransfer_StatorToPhase( float Alpha, float Beta, STATOR_TO_PHASE_TYPE* p);
void CoordinateTransfer_PhaseToStatorClean( PHASE_TO_STATOR_TYPE* p);
void CoordinateTransfer_StatorToRotorClean( STATOR_TO_ROTOR_TYPE* p);
void CoordinateTransfer_RotorToStatorClean( ROTOR_TO_STATOR_TYPE* p);
void CoordinateTransfer_StatorToPhaseClean( STATOR_TO_PHASE_TYPE* p);
extern void CoordinateTransferInit( PHASE_TO_STATOR_TYPE* p1, STATOR_TO_ROTOR_TYPE* p2, ROTOR_TO_STATOR_TYPE* p3, STATOR_TO_PHASE_TYPE* p4 );

#define PHASE_TO_STATOR_DEFAULT 			\
{											\
	0.0f,									\
	0.0f,									\
	(pPhaseToStator)CoordinateTransfer_PhaseToStator,		\
	(pPhaseToStatorClean)CoordinateTransfer_PhaseToStatorClean	\
}

#define STATOR_TO_ROTOR_DEFAULT				\
{											\
	0.0f,									\
	0.0f,									\
	(pStatorToRotor)CoordinateTransfer_StatorToRotor,		\
	(pStatorToRotorClean)CoordinateTransfer_StatorToRotorClean	\
}

#define ROTOR_TO_STATOR_DEFAULT 			\
{											\
	0.0f,									\
	0.0f,									\
	(pRotorToStator)CoordinateTransfer_RotorToStator,		\
	(pRotorToStatorClean)CoordinateTransfer_RotorToStatorClean	\
}

#define STATOR_TO_PHASE_DEFAULT 			\
{											\
	0.0f,									\
	0.0f,									\
	0.0f,									\
	(pStatorToPhase)CoordinateTransfer_StatorToPhase,		\
	(pStatorToPhaseClean)CoordinateTransfer_StatorToPhaseClean	\
}

#define COORDINATE_TRANSFER_GET_SIN_COS( Angle, SinValue, CosValue )	\
		CordicMath_GetSinCosValue_Macro(Angle,SinValue,CosValue);		\


#define COORDINATE_TRANSFER_PHASE_TO_STATOR_MACRO( U, V, W, p)				\
	p->Alpha = 0.666666667f * U - 0.333333333f * ( V + W );					\
	p->Beta = 0.577350269f * ( V - W );										\


#define COORDINATE_TRANSFER_STATOR_TO_ROTOR_MACRO( Alpha, Beta, SinValue, CosValue, p)	\
	p->D = Alpha * CosValue + Beta * SinValue;											\
	p->Q = -Alpha * SinValue + Beta * CosValue;											\

#define COORDINATE_TRANSFER_ROTOR_TO_STATOR_MACRO( D, Q, SinValue, CosValue, p)		\
	p->Alpha = D * CosValue - Q * SinValue;											\
	p->Beta = D * SinValue + Q * CosValue;											\


#define COORDINATE_TRANSFER_STATOR_TO_PHASE_MACRO( Alpha, Beta, p)			\
	p->U = Alpha;															\
	p->V = -0.5f * Alpha + 0.866025403f * Beta;								\
	p->W = -0.5f * Alpha - 0.866025403f * Beta;								\



#endif /* INC_COORDINATETRANSFER_H_ */
