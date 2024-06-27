/*
 * PositionSensor.c
 *
 *  Created on: 2023年12月6日
 *      Author: User
 */

#include "PositionSensor.h"

static void PositionSensor_ReadPosViaPWM(PS_t* v);
// todo 4096 linear table in inverter
//static float ABZtoMechPos[4096] = ABZtoMechPos_Default;
//uint16_t tempABZ = 0;
//float tempMechPosition = 0.0f;

#define JeffTest 1
#if JeffTest
FILTER_BILINEAR_1ORDER_TYPE CalcMechSpeedLPF = FILTER_BILINEAR_1ORDER_DEFAULT;
#endif

void PositionSensor_Init(PS_t* v, uint16_t MechPosZeroOffset, uint16_t MechPosCompCoefBySpeed)
{
#if JeffTest
	FILTER_INIT_BILINEAR_1ORDER_TYPE FilterSettingTmp = FILTER_INIT_BILINEAR_1ORDER_DEFAULT;
	FilterSettingTmp.BandwithHz = 400.0f;
	FilterSettingTmp.Period =  1.0f / (float)INITIAL_CURRENT_LOOP_FREQ;
	FilterSettingTmp.Type = FILTER_TYPE_LPF;
	CalcMechSpeedLPF.Init(&(CalcMechSpeedLPF),&FilterSettingTmp);

#endif

	MOTOR_CONTROL_PARAMETER_DEFAULT_TYPE MotorControlSetting = MotorDefault;

	v->PolePair = MotorControlSetting.PmMotorPolepair;
	v->ABZResolution = MotorControlSetting.ABZResolution;

#if USE_REVERVE_MR_DIRECTION
	v->MechPosZeroOffset = _2PI - ((float)( MechPosZeroOffset - 32768 ) * 0.0001f);
#else
	v->MechPosZeroOffset = (float)( MechPosZeroOffset - 32768 ) * 0.0001f;
#endif
	v->MechPosCompCoefBySpeed = (float)(MechPosCompCoefBySpeed) * 0.0001f;

	float AngleObserverPole =  -MotorControlSetting.AngleObserverHz * _2PI;
	AngleObserverInitParm_t AngleObserverSetting = ANGLE_OBSERVER_INIT_PARAM_DEFAULT;
	AngleObserverSetting.Period = MotorControlSetting.PwmPeriod;
	AngleObserverSetting.J = MotorControlSetting.PmMotorJ;
	AngleObserverSetting.L1 = -3.0f * AngleObserverPole ;
	AngleObserverSetting.L2 = 3.0f * AngleObserverSetting.J * AngleObserverPole * AngleObserverPole;
	AngleObserverSetting.L3 = -AngleObserverSetting.J * AngleObserverPole * AngleObserverPole * AngleObserverPole;
	AngleObserverSetting.SpeedLowerLimit = -7853.9815f;	//1500RPM/60*5*2*pi = 7853.9815 rad/s(Electrical)
	AngleObserverSetting.SpeedUpperLimit = 7853.9815f;	//1500RPM/60*5*2*pi = 7853.9815 rad/s(Electrical)
	AngleObserverSetting.AcelLowerLimit = -111374.6392f;
	AngleObserverSetting.AcelUpperLimit = 111374.6392f;

	v->AngleObsvr.Init( &(v->AngleObsvr) , &AngleObserverSetting );

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
          htim2.Instance->CNT = (uint32_t)(v->MechPosition * v->ABZResolution / _2PI);
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
//	tempMechPosCompensation = v->MechPosCompCoefBySpeed  * v->MechSpeed;
//	tempABZ = v->CntFromABZ & 0x0FFF;
//	tempMechPosition = ABZtoMechPos[tempABZ];

	v->PreMechPosition = ((float)(v->CntFromABZ)) * _2PI / v->ABZResolution;
	v->AngleObsvr.AngleError = ( v->PreMechPosition - ( v->AngleObsvr.Angle + v->AngleObsvr.Speed * v->AngleObsvr.Period ));
	v->AngleObsvr.Quotient = (int16_t)( v->AngleObsvr.AngleError * INV_2PI);
	v->AngleObsvr.Quotient = ( v->AngleObsvr.Quotient >= 0 ) ? v->AngleObsvr.Quotient : v->AngleObsvr.Quotient - 1;
	v->AngleObsvr.CorrespondingAngleError = v->AngleObsvr.AngleError - ((float)v->AngleObsvr.Quotient) * _2PI;
	v->AngleObsvr.CorrespondingAngleError = ( v->AngleObsvr.CorrespondingAngleError > _PI ) ? ( v->AngleObsvr.CorrespondingAngleError - _2PI ) : v->AngleObsvr.CorrespondingAngleError;
	v->AngleObsvr.CorrespondingAngleError = ( v->AngleObsvr.CorrespondingAngleError < -_PI ) ? ( v->AngleObsvr.CorrespondingAngleError + _2PI ) : v->AngleObsvr.CorrespondingAngleError;
	v->AngleObsvr.L1Result = v->AngleObsvr.CorrespondingAngleError * v->AngleObsvr.L1;
	v->AngleObsvr.L2Result = v->AngleObsvr.CorrespondingAngleError * v->AngleObsvr.L2;
	v->AngleObsvr.L3Result = v->AngleObsvr.CorrespondingAngleError * v->AngleObsvr.L3;
	v->AngleObsvr.Te += ( v->AngleObsvr.L3Result * v->AngleObsvr.Period );
	v->AngleObsvr.Acel = (v->AngleObsvr.L2Result + v->AngleObsvr.Te) * v->AngleObsvr.DivideJ;
	v->AngleObsvr.Acel = ( v->AngleObsvr.Acel > v->AngleObsvr.AcelUpperLimit ) ? v->AngleObsvr.AcelUpperLimit : v->AngleObsvr.Acel;
	v->AngleObsvr.Acel = ( v->AngleObsvr.Acel < -v->AngleObsvr.AcelUpperLimit ) ? -v->AngleObsvr.AcelUpperLimit : v->AngleObsvr.Acel;
	v->AngleObsvr.SpeedTmp += ( v->AngleObsvr.Acel * v->AngleObsvr.Period );
	v->AngleObsvr.SpeedTmp = ( v->AngleObsvr.SpeedTmp > v->AngleObsvr.SpeedUpperLimit ) ? v->AngleObsvr.SpeedUpperLimit : v->AngleObsvr.SpeedTmp;
	v->AngleObsvr.SpeedTmp = ( v->AngleObsvr.SpeedTmp < -v->AngleObsvr.SpeedUpperLimit ) ? -v->AngleObsvr.SpeedUpperLimit : v->AngleObsvr.SpeedTmp;
	v->AngleObsvr.Speed = v->AngleObsvr.L1Result + v->AngleObsvr.SpeedTmp;
	v->AngleObsvr.Speed = ( v->AngleObsvr.Speed > v->AngleObsvr.SpeedUpperLimit ) ? v->AngleObsvr.SpeedUpperLimit : v->AngleObsvr.Speed;
	v->AngleObsvr.Speed = ( v->AngleObsvr.Speed < -v->AngleObsvr.SpeedUpperLimit ) ? -v->AngleObsvr.SpeedUpperLimit : v->AngleObsvr.Speed;
	v->AngleObsvr.Angle += (v->AngleObsvr.Speed * v->AngleObsvr.Period );
	v->AngleObsvr.Angle = ( v->AngleObsvr.Angle >= _2PI ) ? v->AngleObsvr.Angle - _2PI : v->AngleObsvr.Angle;
	v->AngleObsvr.Angle = ( v->AngleObsvr.Angle < 0 ) ? v->AngleObsvr.Angle + _2PI : v->AngleObsvr.Angle;

#if JeffTest
	static float MechPosition = 0.0f;
	static float PreMechPosition = 0.0f;
	static float oldMechSpeedRaw = 0.0f;

	MechPosition = (float)v->CntFromABZ * _2PI / v->ABZResolution;

	if ( v->Direction == PS_DIRECTION_UPCOUNTER )
	{
		v->MechSpeedRaw = ( MechPosition >= PreMechPosition ) ? \
				                                ( MechPosition - PreMechPosition ) * (float)INITIAL_CURRENT_LOOP_FREQ : \
												( MechPosition - PreMechPosition + _2PI ) * (float)INITIAL_CURRENT_LOOP_FREQ;

	}
	else
	{
		v->MechSpeedRaw = ( MechPosition <= PreMechPosition ) ? \
				                                ( MechPosition - PreMechPosition ) * (float)INITIAL_CURRENT_LOOP_FREQ : \
												( MechPosition - PreMechPosition - _2PI ) * (float)INITIAL_CURRENT_LOOP_FREQ;
	}



	v->MechSpeedRaw = ABS(v->MechSpeedRaw) > DEFAULT_ABNORMAL_SPEED ? oldMechSpeedRaw : v->MechSpeedRaw;
	oldMechSpeedRaw = v->MechSpeedRaw;

	PreMechPosition = MechPosition;

	if ((uint8_t)( 0xFF & DriveParams.PCUParams.DebugParam2 ) == 1)
	{
		v->MechPosition = MechPosition + v->MechPosZeroOffset;
		v->MechSpeed = Filter_Bilinear1OrderCalc_LPF_inline(&(CalcMechSpeedLPF), v->MechSpeedRaw);;
	}
	else
	{
	    v->MechPosition = v->AngleObsvr.Angle + v->MechPosZeroOffset;
	    v->MechSpeed = v->AngleObsvr.Speed;
	}
#else
	v->MechPosition = v->AngleObsvr.Angle + v->MechPosZeroOffset;
	v->MechSpeed = v->AngleObsvr.Speed;
#endif

	if ( v->MechPosition >= _2PI )
	{
		v->MechPosition = v->MechPosition - _2PI;
	}
	else if ( v->MechPosition < 0.0f )
	{
		v->MechPosition = v->MechPosition + _2PI;
	}

	v->ElecSpeed = v->MechSpeed  * v->PolePair;
	v->ElecPosition = fmod( v->MechPosition * v->PolePair, _2PI );
}

static void PositionSensor_ReadPosViaPWM(PS_t* v)
{
	v->InitMechPosition = (( v->DutyFromPwm - 5.0f ) * _2PI ) / ( 95.0f - 5.0f );
	v->InitMechPosition = v->InitMechPosition > _2PI ? _2PI : v->InitMechPosition < 0.0f ? 0.0f : v->InitMechPosition;
	v->AngleObsvr.Angle = v->InitMechPosition;
}

void PositionSensor_ReadPosIdle(PS_t* v)
{
	v->ElecPosition = fmod( v->MechPosition * v->PolePair, _2PI );
}
