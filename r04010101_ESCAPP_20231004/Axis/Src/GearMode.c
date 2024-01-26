/*
 * GearMode.c
 *
 *  Created on: 2024年1月25日
 *      Author: Jeff.Chang
 */

#include "GearMode.h"

static uint8_t EnableBoostGearMode = FUNCTION_DISABLE;

void GearMode_Init ( GearMode_Var_t* v )
{
/*
 *  Load setup in the future
 */
}

void GearMode_DoPLCLoop ( GearMode_Var_t* v )
{
    switch ( v->GearModeSelect )
    {
        case NORMAL_MODE:
        {
        	if ( v->BoostState == BOOST_READY )
        	{
        		if (( v->IsBoostBtnPressed == BOOST_BTN_PRESSED ) && ( EnableBoostGearMode == FUNCTION_ENABLE ))
        		{
        			 v->GearModeSelect  = BOOST_MODE;
        		}
        	}
        	else  // if ( v->BoostState == BOOST_COOLDOWN )
        	{
            	if ( v->BoostStateCnt++ >= BOOST_COOLDOWN_TIME )
            	{
            		v->BoostStateCnt = 0;
            	    v->BoostState = BOOST_READY;
            	}
        	}

        	break;
        }
        case BOOST_MODE:
        {
        	if ( v->BoostStateCnt++ >= BOOST_CONTINUE_TIME )
        	{
        		v->BoostStateCnt = 0;
        	    v->BoostState = BOOST_COOLDOWN;
        	    v->GearModeSelect  = NORMAL_MODE;
        	}
        	break;
        }
//        case REVERSE_MODE:
//        {
//        	break;
//        }
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
