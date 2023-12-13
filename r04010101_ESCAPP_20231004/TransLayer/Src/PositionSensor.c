/*
 * PositionSensor.c
 *
 *  Created on: 2023年12月6日
 *      Author: User
 */

#include "PositionSensor.h"

static void PositionSensor_ReadPosViaPWM(PS_t* v);

void PositionSensor_Init(PS_t* v)
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
}

void PositionSesnor_DoPLCLoop(PS_t* v)
{
    switch ( v->PositionSensor_StateMachine )
    {
      case PS_SM_INIT_POSITION_SENSOR:
      {
    	  static uint8_t init_cnt = 0;
    	  if ( init_cnt < 100 )  //delay 100ms, TBD with EE
    	  {
    		  init_cnt++;
    	  }
    	  else
    	  {
              v->PositionSensor_StateMachine = PS_SM_PROCESSING_READ_INITI_POSITION;
    	  }

    	  break;
      }
      case PS_SM_PROCESSING_READ_INITI_POSITION:
      {
		  PositionSensor_ReadPosViaPWM(v);
		  v->MechPosition = v->InitMechPosition;
          htim4.Instance->CNT = v->MechPosition * DEFAULT_ABZ_RESOLUTION_PER_MEC_REVOLUTION / _2PI;
//          HAL_NVIC_DisableIRQ(TIM3_IRQn);
    	  HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_ALL);
    	  v->DoCurrentLoop = (functypePositionSensor_DoCurrentLoop)&PositionSensor_ReadPosViaABZ;
    	  v->PositionSensor_StateMachine = PS_SM_PROCESSING_READ_OPERATING_POSITION;
    	  break;
      }
      case PS_SM_PROCESSING_READ_OPERATING_POSITION:
      {
		  PositionSensor_ReadPosViaPWM(v);
    	  break;
      }
      case PS_SM_ERROR:
      {
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
	v->MechPosition = v->CntFromABZ * _2PI / DEFAULT_ABZ_RESOLUTION_PER_MEC_REVOLUTION + tempMechPosCompensation;

	if ( v->Direction == PS_DIRECTION_UPCOUNTER )
	{
		v->MechSpeedRaw = ( v->MechPosition - v->PreMechPosition ) >= 0.0f ? \
				                                ( v->MechPosition - v->PreMechPosition ) * 10000.0f : \
												( v->MechPosition - v->PreMechPosition + _2PI ) * 10000.0f;

	}
	else
	{
		v->MechSpeedRaw = ( v->MechPosition - v->PreMechPosition ) <= 0.0f ? \
				                                ( v->MechPosition - v->PreMechPosition ) * 10000.0f : \
												( v->MechPosition - v->PreMechPosition - _2PI ) * 10000.0f;
	}

	v->PreMechPosition = v->MechPosition;
	v->MechSpeed = Filter_Bilinear1OrderCalc_LPF_inline(&(v->CalcMechSpeedLPF), v->MechSpeedRaw);

	v->ElecSpeed = v->MechSpeed  * DEFAULT_POLE_PAIRS;
	v->ElecPosition = fmod( v->MechPosition * DEFAULT_POLE_PAIRS, _2PI );
}

static void PositionSensor_ReadPosViaPWM(PS_t* v)
{
	v->InitMechPosition = (( v->DutyFromPwm - 5.0f ) * _2PI ) / ( 95.0f - 5.0f );
}

void PositionSensor_ReadPosIdle(PS_t* v)
{

}
