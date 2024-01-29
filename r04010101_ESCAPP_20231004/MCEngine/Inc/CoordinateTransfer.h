/*
 * CoordinateTransfer.h
 *
 *  Created on: 2019年12月2日
 *      Author: Fernando
 */

#ifndef INC_COORDINATETRANSFER_H_
#define INC_COORDINATETRANSFER_H_

typedef void (*pPhaseToStatorClean)( void*);
typedef void (*pStatorToRotorClean)( void*);
typedef void (*pRotorToStatorClean)( void*);
typedef void (*pStatorToPhaseClean)( void*);


typedef struct
{
	float Alpha;
	float Beta;
	pPhaseToStatorClean Clean;
} PHASE_TO_STATOR_TYPE;

typedef struct
{
	float D;
	float Q;
	pStatorToRotorClean Clean;
} STATOR_TO_ROTOR_TYPE;

typedef struct
{
	float Alpha;
	float Beta;
	pRotorToStatorClean Clean;
} ROTOR_TO_STATOR_TYPE;

typedef struct
{
	float U;
	float V;
	float W;
	pStatorToPhaseClean Clean;
} STATOR_TO_PHASE_TYPE;

void CoordinateTransfer_PhaseToStatorClean( PHASE_TO_STATOR_TYPE* p);
void CoordinateTransfer_StatorToRotorClean( STATOR_TO_ROTOR_TYPE* p);
void CoordinateTransfer_RotorToStatorClean( ROTOR_TO_STATOR_TYPE* p);
void CoordinateTransfer_StatorToPhaseClean( STATOR_TO_PHASE_TYPE* p);
extern void CoordinateTransferInit( PHASE_TO_STATOR_TYPE* p1, STATOR_TO_ROTOR_TYPE* p2, ROTOR_TO_STATOR_TYPE* p3, STATOR_TO_PHASE_TYPE* p4 );

#define PHASE_TO_STATOR_DEFAULT 			\
{											\
	0.0f,									\
	0.0f,									\
	(pPhaseToStatorClean)CoordinateTransfer_PhaseToStatorClean	\
}

#define STATOR_TO_ROTOR_DEFAULT				\
{											\
	0.0f,									\
	0.0f,									\
	(pStatorToRotorClean)CoordinateTransfer_StatorToRotorClean	\
}

#define ROTOR_TO_STATOR_DEFAULT 			\
{											\
	0.0f,									\
	0.0f,									\
	(pRotorToStatorClean)CoordinateTransfer_RotorToStatorClean	\
}

#define STATOR_TO_PHASE_DEFAULT 			\
{											\
	0.0f,									\
	0.0f,									\
	0.0f,									\
	(pStatorToPhaseClean)CoordinateTransfer_StatorToPhaseClean	\
}

#define COORDINATE_TRANSFER_GET_SIN_COS( Angle, SinValue, CosValue )	\
		CordicMath_GetSinCosValue_Macro(Angle,SinValue,CosValue);		\


__STATIC_FORCEINLINE void COORDINATE_TRANSFER_Phase_to_Stator_Calc( float U, float V, float W, PHASE_TO_STATOR_TYPE* p)
{
	p->Alpha = 0.666666667f * U - 0.333333333f * ( V + W );
	p->Beta = 0.577350269f * ( V - W );
}

__STATIC_FORCEINLINE void COORDINATE_TRANSFER_Stator_to_Rotor_Calc( float Alpha, float Beta, float SinValue, float CosValue, STATOR_TO_ROTOR_TYPE* p)
{
	p->D = Alpha * CosValue + Beta * SinValue;
	p->Q = -Alpha * SinValue + Beta * CosValue;
}

__STATIC_FORCEINLINE void COORDINATE_TRANSFER_Rotor_to_Stator_Calc( float D, float Q, float SinValue, float CosValue, ROTOR_TO_STATOR_TYPE* p)
{
	p->Alpha = D * CosValue - Q * SinValue;
	p->Beta = D * SinValue + Q * CosValue;
}


__STATIC_FORCEINLINE void COORDINATE_TRANSFER_Stator_to_Phase_Calc( float Alpha, float Beta, STATOR_TO_PHASE_TYPE* p)
{
	p->U = Alpha;
	p->V = -0.5f * Alpha + 0.866025403f * Beta;
	p->W = -0.5f * Alpha - 0.866025403f * Beta;
}

__STATIC_FORCEINLINE void COORDINATE_TRANSFER_GetSinCos_LT(uint16_t Pos, float *sin, float *cos)
    {
	uint16_t Pos_Cos = Pos;

        if (Pos >= 4096)
        	Pos -= 4096;

        Pos_Cos = Pos + 1024; //1024=4096/4 90degree
        if (Pos_Cos >= 4096)
            Pos_Cos -= 4096;

        *sin = sin_LT[Pos];
        *cos = sin_LT[Pos_Cos];
    }

#endif /* INC_COORDINATETRANSFER_H_ */
