/*
 * PositionSensor.c
 *
 *  Created on: 2023年12月6日
 *      Author: User
 */

#include "PositionSensor.h"
#include "AngleObserver.h"
static void PositionSensor_ReadPosViaPWM(PS_t* v);
// todo 4096 linear table in inverter
//static float ABZtoMechPos[4096] = ABZtoMechPos_Default;
//uint16_t tempABZ = 0;
//float tempMechPosition = 0.0f;

#define JEFF_TEST2 1

#if JEFF_TEST2
AngleObserver_t MR_AngleObsvr = ANGLE_OBSERVER_DEFAULT;
#endif

void PositionSensor_Init(PS_t* v, uint16_t MechPosZeroOffset, uint16_t MechPosCompCoefBySpeed)
{
/*
 * To do:
 * load v->MechPosCompCoefBySpeed
 * Load pole pairs
 */
	FILTER_INIT_BILINEAR_1ORDER_TYPE FilterSettingTmp = FILTER_INIT_BILINEAR_1ORDER_DEFAULT;
	FilterSettingTmp.BandwithHz = 400.0f;
	FilterSettingTmp.Period =  1.0f / (float)INITIAL_CURRENT_LOOP_FREQ;
	FilterSettingTmp.Type = FILTER_TYPE_LPF;
	v->CalcMechSpeedLPF.Init(&(v->CalcMechSpeedLPF),&FilterSettingTmp);
#if USE_REVERVE_MR_DIRECTION
	v->MechPosZeroOffset = _2PI - ((float)( MechPosZeroOffset - 32768 ) * 0.0001f);
#else
	v->MechPosZeroOffset = (float)( MechPosZeroOffset - 32768 ) * 0.0001f;
#endif
	v->MechPosCompCoefBySpeed = (float)(MechPosCompCoefBySpeed) * 0.0001f;

#if JEFF_TEST2
	float AngleObserverPole =  -20.0f * 5.0f * _2PI;
	AngleObserverInitParm_t AngleObserverSetting = ANGLE_OBSERVER_INIT_PARAM_DEFAULT;
	AngleObserverSetting.Period = 1.0f / (float)INITIAL_CURRENT_LOOP_FREQ;
	AngleObserverSetting.J = 0.00697189f;
	AngleObserverSetting.L1 = -3.0f * AngleObserverPole ;
	AngleObserverSetting.L2 = 3.0f * AngleObserverPole * AngleObserverPole;
	AngleObserverSetting.L3 = -AngleObserverSetting.J * AngleObserverPole * AngleObserverPole * AngleObserverPole;
	AngleObserverSetting.SpeedLowerLimit = -7853.9815f;	//1500RPM/60*5*2*pi = 7853.9815 rad/s(Electrical)
	AngleObserverSetting.SpeedUpperLimit = 7853.9815f;	//1500RPM/60*5*2*pi = 7853.9815 rad/s(Electrical)
	AngleObserverSetting.AcelLowerLimit = -111374.6392f;
	AngleObserverSetting.AcelUpperLimit = 111374.6392f;

	MR_AngleObsvr.Init( &(MR_AngleObsvr) , &AngleObserverSetting );
#endif
}

void PositionSesnor_DoPLCLoop(PS_t* v)
{
    switch ( v->PositionSensor_StateMachine )
    {
      case PS_SM_INIT_POSITION_SENSOR:
      {
    	  static uint16_t init_cnt = 0;
    	  if ( init_cnt < 500 )  //delay 500ms, TBD with EE
    	  {
    		  init_cnt++;
    	  }
    	  else
    	  {
    		  //TODO: error handle if no valid duty with timeout
    		  if (( v->DutyFromPwm > 4.5f ) && ( v->DutyFromPwm < 95.5f ))
		      {
                  v->PositionSensor_StateMachine = PS_SM_PROCESSING_READ_INITI_POSITION;
		      }
    	  }

    	  break;
      }
      case PS_SM_PROCESSING_READ_INITI_POSITION:
      {
		  PositionSensor_ReadPosViaPWM(v);
		  v->MechPosition = v->InitMechPosition;
#if USE_REVERVE_MR_DIRECTION
		  v->PreMechPosition = _2PI - v->MechPosition;
#else
		  v->PreMechPosition = v->MechPosition;
#endif
          htim2.Instance->CNT = (uint32_t)(v->MechPosition * (float)DEFAULT_ABZ_RESOLUTION_PER_MEC_REVOLUTION / _2PI);
#if JEFF_TEST2
          MR_AngleObsvr.Angle = v->PreMechPosition;
#endif
          HAL_NVIC_DisableIRQ(TIM20_CC_IRQn);
    	  HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);
    	  v->DoCurrentLoop = (functypePositionSensor_DoCurrentLoop)&PositionSensor_ReadPosViaABZ;
    	  v->PositionSensor_StateMachine = PS_SM_PROCESSING_READ_OPERATING_POSITION;
    	  break;
      }
      case PS_SM_PROCESSING_READ_OPERATING_POSITION:
      {
//		  PositionSensor_ReadPosViaPWM(v);
    	  break;
      }
      case PS_SM_ERROR:
      {
    	  //TODO Eooro handling
    	  break;
      }
      default:
	  {
    	  break;
	  }
    }
}

__attribute__(( section(".ram_function"))) void __attribute__((optimize("Ofast"))) PositionSensor_ReadPosViaABZ(PS_t* v)
{
	static float oldMechSpeedRaw = 0.0f;
//	tempMechPosCompensation = v->MechPosCompCoefBySpeed  * v->MechSpeed;
//	tempABZ = v->CntFromABZ & 0x0FFF;
//	tempMechPosition = ABZtoMechPos[tempABZ];

//	static float tempPos = 0.0f;
//	static float tempSpeed = 0.0f;
//
//	tempSpeed = tempSpeed + 10.0f * MR_AngleObsvr.Period;
//	if ( tempSpeed >= 1000.0f)
//		tempSpeed = tempSpeed - 1000.0f;
//
//	tempPos =  tempPos + 0.076699f;//tempSpeed * MR_AngleObsvr.Period;
//	if ( tempPos >= _2PI)
//		tempPos = tempPos -_2PI ;
//
//	v->MechPosition = tempPos;
#if JEFF_TEST2
	float AngleIn = (float)v->CntFromABZ * _2PI / DEFAULT_ABZ_RESOLUTION_PER_MEC_REVOLUTION;

//	 AngleIn = tempPos;
	MR_AngleObsvr.AngleError = ( AngleIn - ( MR_AngleObsvr.Angle + MR_AngleObsvr.Speed * MR_AngleObsvr.Period ));
	MR_AngleObsvr.Quotient = (int16_t)( MR_AngleObsvr.AngleError * INV_2PI);
	MR_AngleObsvr.Quotient = ( MR_AngleObsvr.Quotient >= 0 ) ? MR_AngleObsvr.Quotient : MR_AngleObsvr.Quotient - 1;
	MR_AngleObsvr.CorrespondingAngleError = MR_AngleObsvr.AngleError - ((float)MR_AngleObsvr.Quotient) * _2PI;
	MR_AngleObsvr.CorrespondingAngleError = ( MR_AngleObsvr.CorrespondingAngleError > _PI ) ? ( MR_AngleObsvr.CorrespondingAngleError - _2PI ) : MR_AngleObsvr.CorrespondingAngleError;
	MR_AngleObsvr.CorrespondingAngleError = ( MR_AngleObsvr.CorrespondingAngleError < -_PI ) ? ( MR_AngleObsvr.CorrespondingAngleError + _2PI ) : MR_AngleObsvr.CorrespondingAngleError;
	MR_AngleObsvr.L1Result = MR_AngleObsvr.CorrespondingAngleError * MR_AngleObsvr.L1;
	MR_AngleObsvr.L2Result = MR_AngleObsvr.CorrespondingAngleError * MR_AngleObsvr.L2;
	MR_AngleObsvr.L3Result = MR_AngleObsvr.CorrespondingAngleError * MR_AngleObsvr.L3;
	MR_AngleObsvr.AcelTmp += ( MR_AngleObsvr.L3Result * MR_AngleObsvr.Period );
	MR_AngleObsvr.AcelTmp = ( MR_AngleObsvr.AcelTmp > MR_AngleObsvr.AcelUpperLimit ) ? MR_AngleObsvr.AcelUpperLimit : MR_AngleObsvr.AcelTmp;
	MR_AngleObsvr.AcelTmp = ( MR_AngleObsvr.AcelTmp < -MR_AngleObsvr.AcelUpperLimit ) ? -MR_AngleObsvr.AcelUpperLimit : MR_AngleObsvr.AcelTmp;
	MR_AngleObsvr.Acel = MR_AngleObsvr.L2Result + MR_AngleObsvr.AcelTmp;
	MR_AngleObsvr.Acel = ( MR_AngleObsvr.Acel > MR_AngleObsvr.AcelUpperLimit ) ? MR_AngleObsvr.AcelUpperLimit : MR_AngleObsvr.Acel;
	MR_AngleObsvr.Acel = ( MR_AngleObsvr.Acel < -MR_AngleObsvr.AcelUpperLimit ) ? -MR_AngleObsvr.AcelUpperLimit : MR_AngleObsvr.Acel;
	MR_AngleObsvr.SpeedTmp += ( MR_AngleObsvr.Acel * MR_AngleObsvr.Period );
	MR_AngleObsvr.SpeedTmp = ( MR_AngleObsvr.SpeedTmp > MR_AngleObsvr.SpeedUpperLimit ) ? MR_AngleObsvr.SpeedUpperLimit : MR_AngleObsvr.SpeedTmp;
	MR_AngleObsvr.SpeedTmp = ( MR_AngleObsvr.SpeedTmp < -MR_AngleObsvr.SpeedUpperLimit ) ? -MR_AngleObsvr.SpeedUpperLimit : MR_AngleObsvr.SpeedTmp;
	MR_AngleObsvr.Speed = MR_AngleObsvr.L1Result + MR_AngleObsvr.SpeedTmp;
	MR_AngleObsvr.Speed = ( MR_AngleObsvr.Speed > MR_AngleObsvr.SpeedUpperLimit ) ? MR_AngleObsvr.SpeedUpperLimit : MR_AngleObsvr.Speed;
	MR_AngleObsvr.Speed = ( MR_AngleObsvr.Speed < -MR_AngleObsvr.SpeedUpperLimit ) ? -MR_AngleObsvr.SpeedUpperLimit : MR_AngleObsvr.Speed;
	MR_AngleObsvr.Angle += ( MR_AngleObsvr.Speed * MR_AngleObsvr.Period );
	MR_AngleObsvr.Angle = ( MR_AngleObsvr.Angle >= _2PI ) ? MR_AngleObsvr.Angle - _2PI : MR_AngleObsvr.Angle;
	MR_AngleObsvr.Angle = ( MR_AngleObsvr.Angle < 0 ) ? MR_AngleObsvr.Angle + _2PI : MR_AngleObsvr.Angle;

	v->MechPosition = MR_AngleObsvr.Angle + v->MechPosZeroOffset;
	v->MechSpeed = MR_AngleObsvr.Speed;

#else
	v->MechPosition = (float)v->CntFromABZ * _2PI / DEFAULT_ABZ_RESOLUTION_PER_MEC_REVOLUTION;
#if USE_REVERVE_MR_DIRECTION
	v->MechPosition = _2PI - v->MechPosition;
	if ( v->Direction == PS_DIRECTION_DOWNCOUNTER )
	{
#else
	if ( v->Direction == PS_DIRECTION_UPCOUNTER )
	{
#endif
		v->MechSpeedRaw = ( v->MechPosition >= v->PreMechPosition ) ? \
				                                ( v->MechPosition - v->PreMechPosition ) * (float)INITIAL_CURRENT_LOOP_FREQ : \
												( v->MechPosition - v->PreMechPosition + _2PI ) * (float)INITIAL_CURRENT_LOOP_FREQ;

	}
	else
	{
		v->MechSpeedRaw = ( v->MechPosition <= v->PreMechPosition ) ? \
				                                ( v->MechPosition - v->PreMechPosition ) * (float)INITIAL_CURRENT_LOOP_FREQ : \
												( v->MechPosition - v->PreMechPosition - _2PI ) * (float)INITIAL_CURRENT_LOOP_FREQ;
	}



	v->MechSpeedRaw = ABS(v->MechSpeedRaw) > DEFAULT_ABNORMAL_SPEED ? oldMechSpeedRaw : v->MechSpeedRaw;
	oldMechSpeedRaw = v->MechSpeedRaw;

	v->MechSpeed = Filter_Bilinear1OrderCalc_LPF_inline(&(v->CalcMechSpeedLPF), v->MechSpeedRaw);

	v->PreMechPosition = v->MechPosition;

#endif

	if ( v->MechPosition >= _2PI )
	{
		v->MechPosition = v->MechPosition - _2PI;
	}
	else if ( v->MechPosition < 0.0f )
	{
		v->MechPosition = v->MechPosition + _2PI;
	}

	v->ElecSpeed = v->MechSpeed  * DEFAULT_POLE_PAIRS;
	v->ElecPosition = fmod( v->MechPosition * DEFAULT_POLE_PAIRS, _2PI );
}

static void PositionSensor_ReadPosViaPWM(PS_t* v)
{
	v->InitMechPosition = (( v->DutyFromPwm - 5.0f ) * _2PI ) / ( 95.0f - 5.0f );
	v->InitMechPosition = v->InitMechPosition > _2PI ? _2PI : v->InitMechPosition < 0.0f ? 0.0f : v->InitMechPosition;
}

void PositionSensor_ReadPosIdle(PS_t* v)
{
	v->ElecPosition = fmod( v->MechPosition * DEFAULT_POLE_PAIRS, _2PI );
}
