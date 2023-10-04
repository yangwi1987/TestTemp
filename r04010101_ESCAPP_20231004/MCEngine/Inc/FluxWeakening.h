/*
 * FluxWeakening.h
 *
 *  Created on: 2019年12月13日
 *      Author: Fernando
 */

#ifndef INC_FLUXWEAKENING_H_
#define INC_FLUXWEAKENING_H_

//#define FLUX_WEAKENING_INIT_OK								0
//#define FLUX_WEAKENING_INIT_ERROR_ILIMIT_IS_NOT_POSITIVE	0x8001

enum FLUX_WEAKENING_INIT_STATUS_ENUM
{
	FLUX_WEAKENING_INIT_OK = 0,
	FLUX_WEAKENING_INIT_ERROR_ILIMIT_IS_NOT_POSITIVE = 0x8001
};

typedef uint16_t (*pFluxWeakeningForCurrentControlInit)( void*, float, float, float, float, float);
typedef void (*pFluxWeakeningForCurrentControlClean)( void* );
typedef void (*pFluxWeakeningForCurrentControlCalc)( void*, float, float, float, float);


typedef struct
{
	float IdComp;
	float Ilimit;
	float IdCmd;
	float IqCmd;
	PI_TYPE FluxWeakeningRegulator;
	pFluxWeakeningForCurrentControlInit Init;
	pFluxWeakeningForCurrentControlClean Clean;
	pFluxWeakeningForCurrentControlCalc Calc;
}CURRENT_CONTROL_FLUX_WEAKENING_TYPE;

uint16_t MotorControl_FluxWeakeningForCurrentControlInit( CURRENT_CONTROL_FLUX_WEAKENING_TYPE *p, float Ilimit, float IdMin, float Period, float Kp, float Ki);
void MotorControl_FluxWeakeningForCurrentControlClean( CURRENT_CONTROL_FLUX_WEAKENING_TYPE *p );
void MotorControl_FluxWeakeningForCurrentControlCalc( CURRENT_CONTROL_FLUX_WEAKENING_TYPE *p, float VcmdAmp, float VbusLimit, float OrgIdCmd, float OrgIqCmd);

#define CURRENT_CONTROL_FLUX_WEAKENING_DEFAULT 			\
{														\
	0.0f,												\
	0.0f,												\
	0.0f,												\
	0.0f,												\
	PI_DEFAULT,											\
	(pFluxWeakeningForCurrentControlInit)MotorControl_FluxWeakeningForCurrentControlInit, 	\
	(pFluxWeakeningForCurrentControlClean)MotorControl_FluxWeakeningForCurrentControlClean,	\
	(pFluxWeakeningForCurrentControlCalc)MotorControl_FluxWeakeningForCurrentControlCalc		\
}

#define MOTOR_CONTROL_FLUX_WEAKENING_FOR_CURRENT_CONTROL( p, VcmdAmp, VbusLimit, OrgIdCmd, OrgIqCmd)							\
	float IqSign;																												\
	float IdCmdSquare;																											\
	float IcmdSquare;																											\
	float IlimitSquare;																											\
	\
	p->FluxWeakeningRegulator.Calc( VbusLimit, VcmdAmp, &(p->FluxWeakeningRegulator));											\
	p->IdComp = p->FluxWeakeningRegulator.Output;																				\
	p->IdComp = ( p->IdComp < p->FluxWeakeningRegulator.LowerLimit ) ? p->FluxWeakeningRegulator.LowerLimit : p->IdComp;		\
	p->IdComp = ( p->IdComp > p->FluxWeakeningRegulator.UpperLimit ) ? p->FluxWeakeningRegulator.UpperLimit : p->IdComp;		\
	p->IdCmd = p->IdComp + OrgIdCmd;																							\
	p->IdCmd = ( p->IdCmd < p->FluxWeakeningRegulator.LowerLimit ) ? p->FluxWeakeningRegulator.LowerLimit : p->IdCmd;			\
	p->IdCmd = ( p->IdCmd > p->FluxWeakeningRegulator.UpperLimit ) ? p->FluxWeakeningRegulator.UpperLimit : p->IdCmd;			\
	\
	IdCmdSquare = p->IdCmd * p->IdCmd;																							\
	IcmdSquare = OrgIqCmd * OrgIqCmd + IdCmdSquare;																				\
	IlimitSquare = p->Ilimit * p->Ilimit;																						\
	\
	if ( IcmdSquare > IlimitSquare)																								\
	{																															\
		IqSign = ( OrgIqCmd > 0.0f) ? 1.0f : -1.0f;																				\
		p->IqCmd = sqrtf( IlimitSquare - IdCmdSquare );																			\
		p->IqCmd = p->IqCmd * IqSign;																							\
	}																															\
	else																														\
	{																															\
		p->IqCmd = OrgIqCmd;																									\
	}																															\


#endif /* INC_FLUXWEAKENING_H_ */
