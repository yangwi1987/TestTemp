/*
 * PositionSensor.c
 *
 *  Created on: 2023年12月6日
 *      Author: User
 */

#include "PositionSensor.h"

static void PositionSensor_ReadPosViaPWM(PS_t* v);

void PositionSensor_Init(PS_t* v, uint16_t MechPosZeroOffset, uint16_t MechPosCompCoefBySpeed)
{
/*
 * To do:
 * load v->MechPosCompCoefBySpeed
 * Load pole pairs
 */
	FILTER_INIT_BILINEAR_1ORDER_TYPE FilterSettingTmp = FILTER_INIT_BILINEAR_1ORDER_DEFAULT;
	FilterSettingTmp.BandwithHz = 400.0f;
	FilterSettingTmp.Period =  0.0001f;
	FilterSettingTmp.Type = FILTER_TYPE_LPF;
	v->CalcMechSpeedLPF.Init(&(v->CalcMechSpeedLPF),&FilterSettingTmp);

	v->MechPosZeroOffset = (float)( MechPosZeroOffset - 32768 ) * 0.0001f;
	v->MechPosCompCoefBySpeed = (float)(MechPosCompCoefBySpeed) * 0.0001f;
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
	tempMechPosCompensation = v->MechPosCompCoefBySpeed  * v->MechSpeed;
	v->MechPosition = v->CntFromABZ * _2PI / DEFAULT_ABZ_RESOLUTION_PER_MEC_REVOLUTION + tempMechPosCompensation + v->MechPosZeroOffset;
	if ( v->MechPosition >= _2PI )
	{
		v->MechPosition = v->MechPosition - _2PI;
	}
	else if ( v->MechPosition < 0.0f )
	{
		v->MechPosition = v->MechPosition + _2PI;
	}

	if ( v->Direction == PS_DIRECTION_UPCOUNTER )
	{
		v->MechSpeedRaw = ( v->MechPosition >= v->PreMechPosition ) ? \
				                                ( v->MechPosition - v->PreMechPosition ) * 10000.0f : \
												( v->MechPosition - v->PreMechPosition + _2PI ) * 10000.0f;

	}
	else
	{
		v->MechSpeedRaw = ( v->MechPosition <= v->PreMechPosition ) ? \
				                                ( v->MechPosition - v->PreMechPosition ) * 10000.0f : \
												( v->MechPosition - v->PreMechPosition - _2PI ) * 10000.0f;
	}

	v->MechSpeed = Filter_Bilinear1OrderCalc_LPF_inline(&(v->CalcMechSpeedLPF), v->MechSpeedRaw);

	v->PreMechPosition = v->MechPosition;

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
