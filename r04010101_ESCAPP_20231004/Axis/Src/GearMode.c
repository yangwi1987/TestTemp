/*
 * GearMode.c
 *
 *  Created on: 2024年1月25日
 *      Author: Jeff.Chang
 */

#include "GearMode.h"

static uint8_t EnableBoostGearMode = FUNCTION_DISABLE;
static uint8_t EnableReverseGearMode = FUNCTION_DISABLE;

void GearMode_Init ( GearMode_Var_t* v )
{
/*
 *  Load setup in the future
 */
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
    switch ( v->GearModeSelect )
    {
        case NORMAL_MODE:
        {
    		if (( v->IsBoostBtnPressed == BOOST_BTN_PRESSED ) && (v->IsReverseBtnPressed == REVERSE_BTN_RELEASED) && ( EnableBoostGearMode == FUNCTION_ENABLE ))
    		{
            	if ( v->BoostState == BOOST_READY )
            	{
            		 v->GearModeSelect  = BOOST_MODE;
            	}
    		}
    		else if (( v->IsReverseBtnPressed  == REVERSE_BTN_PRESSED ) && (v->IsBoostBtnPressed == BOOST_BTN_RELEASED) && ( EnableReverseGearMode == FUNCTION_ENABLE ))
        	{
    			v->GearModeSelect  = REVERSE_MODE;
        	}

        	break;
        }
        case BOOST_MODE:
        {
        	if (( v->BoostCnt++ >= BOOST_CONTINUE_TIME ) || ( EnableBoostGearMode == FUNCTION_DISABLE ))
        	{
        		v->BoostCnt = 0;
        	    v->BoostState = BOOST_COOLDOWN;
        	    v->GearModeSelect  = NORMAL_MODE;
        	}
        	else if (( v->IsReverseBtnPressed  == REVERSE_BTN_PRESSED ) && ( EnableReverseGearMode == FUNCTION_ENABLE ))
        	{
        		v->BoostCnt = 0;
        	    v->BoostState = BOOST_COOLDOWN;
        	    v->GearModeSelect  = REVERSE_MODE;
        	}
        	break;
        }
        case REVERSE_MODE:
        {
        	if ( v->IsReverseBtnPressed  == REVERSE_BTN_RELEASED )
        	{
        	    v->GearModeSelect  = NORMAL_MODE;
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
