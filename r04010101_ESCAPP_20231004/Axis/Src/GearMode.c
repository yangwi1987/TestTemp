/*
 * GearMode.c
 *
 *  Created on: 2024年1月25日
 *      Author: Jeff.Chang
 *
 *      Integrate with FourQuadControl In the next stage!!!
 */

#include "GearMode.h"

static uint8_t EnableBoostGearMode = FUNCTION_DISABLE;
static uint8_t EnableReverseGearMode = FUNCTION_DISABLE;

void GearMode_Init ( GearMode_Var_t* v, int16_t MaxMotorRPMToEnRev )
{
	v->MaxMotorRPMToEnRev = MaxMotorRPMToEnRev;
}

void GearMode_DoPLCLoop ( GearMode_Var_t* v )
{
	if ( v->BoostState == BOOST_COOLDOWN )
	{
    	if ( v->BoostCoolCnt++ >= BOOST_COOLDOWN_TIME )
    	{
    		v->BoostCoolCnt = 0;
    	    v->BoostState = BOOST_READY;
    	}
	}
	else if ( v->BoostState == BOOST_MODE_ENABLE )
	{
		if (( v->BoostCnt++ >= BOOST_CONTINUE_TIME ) || ( EnableBoostGearMode == FUNCTION_DISABLE ))
		{
    		v->BoostCnt = 0;
    	    v->BoostState = BOOST_COOLDOWN;
		}
	}

    switch ( v->GearPositionState )
    {
        case VIRTUAL_GEAR_N:
        {
        	if ( v->ServoOnCommand == 1 )
        	{
        		v->GearPositionState = VIRTUAL_GEAR_D;
        	}
        	break;
        }
        case VIRTUAL_GEAR_D:
        {
        	if ( v->ServoOnCommand == 0 )
        	{
        		v->GearPositionState = VIRTUAL_GEAR_N;
        	}
        	else if (( v->IsReverseBtnPressed  == BTN_REVERSE_PRESS ) && ( EnableReverseGearMode == FUNCTION_ENABLE ) \
        			&& ( v->MotorRPM <= v->MaxMotorRPMToEnRev) && ( v->APPReleaseFlag == 1 ))
        	{
    			v->GearPositionState  = VIRTUAL_GEAR_R;
        	}
    		else if (( v->IsBoostBtnPressed == BTN_BOOST_PRESS ) && ( EnableBoostGearMode == FUNCTION_ENABLE ) && ( v->BoostState == BOOST_READY ))
    		{
    			v->BoostState  = BOOST_MODE_ENABLE;
    		}
        	break;
        }
        case VIRTUAL_GEAR_R:
        {
        	if ( v->ServoOnCommand == 0 )
        	{
        		v->GearPositionState = VIRTUAL_GEAR_N;
        	}
        	else if ( v->IsReverseBtnPressed  == BTN_REVERSE_RELEASE )
        	{
        	    v->GearPositionState  = VIRTUAL_GEAR_D;
        	}
        	break;
        }
        default:
        {
        	break;
        }
    }
}

void GearMode_EnableBoostMode (void)
{
	EnableBoostGearMode = FUNCTION_ENABLE;
}

void GearMode_DisableBoostMode (void)
{
	EnableBoostGearMode = FUNCTION_DISABLE;
}

void GearMode_EnableReverseMode (void)
{
	EnableReverseGearMode = FUNCTION_ENABLE;
}

void GearMode_DisableReverseMode (void)
{
	EnableReverseGearMode = FUNCTION_DISABLE;
}
