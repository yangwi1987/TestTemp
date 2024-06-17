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
	float Pole1 =  10.0f * _2PI * 1.5f;
	float Pole2 =  50.0f * _2PI * 1.5f;
	float Pole3 =  110.0f * _2PI * 1.5f;
	AngleObserverInitParm_t AngleObserverSetting = ANGLE_OBSERVER_INIT_PARAM_DEFAULT;
	AngleObserverSetting.Period = 1.0f / (float)INITIAL_CURRENT_LOOP_FREQ;
	AngleObserverSetting.J = 0.00697189f;
	AngleObserverSetting.L1 = Pole1 + Pole2 + Pole3 ;
	AngleObserverSetting.L2 = Pole1 * Pole2 + Pole2 * Pole3 + Pole3 * Pole1;
	AngleObserverSetting.L3 = Pole1 * Pole2 * Pole3;
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
	float tempMechPosCompensation = 0.0f;
	static float oldMechSpeedRaw = 0.0f;
	tempMechPosCompensation = v->MechPosCompCoefBySpeed  * v->MechSpeed;
//	tempABZ = v->CntFromABZ & 0x0FFF;
//	tempMechPosition = ABZtoMechPos[tempABZ];
	v->MechPosition = v->CntFromABZ * _2PI / DEFAULT_ABZ_RESOLUTION_PER_MEC_REVOLUTION + tempMechPosCompensation + v->MechPosZeroOffset;

	if ( v->MechPosition >= _2PI )
	{
		v->MechPosition = v->MechPosition - _2PI;
	}
	else if ( v->MechPosition < 0.0f )
	{
		v->MechPosition = v->MechPosition + _2PI;
	}

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

#if JEFF_TEST2
	float tempMechPos = v->MechPosition;
	ANGLE_OBSERVER_CALC_MACRO( (&(MR_AngleObsvr)), tempMechPos );
	v->MechPosition = MR_AngleObsvr.Angle;
	v->MechSpeed = MR_AngleObsvr.Speed;
#endif

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
