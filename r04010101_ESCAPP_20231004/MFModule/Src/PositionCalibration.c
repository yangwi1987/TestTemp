/*
 * PositionCalibration.c
 *
 *  Created on: 2024年1月17日
 *      Author: User
 */

#include "PositionCalibration.h"

static PS_CALI_t PS_CALI_Vars = PS_CALI_DEFAULT;
#if DEFAULT_POLE_PAIRS == 4
static uint32_t LinearElePosCmd[32] = { 7854,	15707,	23562,	31416,	39270,	47123,	54978,	0,	7854,	15707,	23562,	\
		31416,	39270,	47123,	54978,	0,7854,	15707,	23562,	31416,	39270,	47123,	54978,	0,7854,	15707,	23562,	31416,	39270,	\
		47123,	54978,	0,};  //According to pole pairs, calculate 11.25 mech degrees interval between each points
#elif DEFAULT_POLE_PAIRS == 5
static uint32_t LinearElePosCmd[32] = { 9817,	19635,	29452,	39270,	49087,	58905,	5890,	15708,	25525,	35343,	45160,	\
		54978,	1963,	11781,	21598,	31416,	41233,	51051,	60868,	7854,	17671,	27489,	37306,	47124,	56941,	3927,	13744,	\
		23562,	33379,	43197,	53014, 0 };  //According to pole pairs, calculate 11.25 mech degrees interval between each points
#endif
float LinearPointsMechPosRad[32] = { 0.0f };

static void PositionCalibration_Auto_Zero_Offset_Process(PS_t *u );
static void PositionCalibration_Linear_Process(PS_t *u );
static void PositionCalibration_Start_IF_Control(void);
static void PositionCalibration_Rotate_Until_Across_Mech_Zero( float MechPosition );
static void PositionCalibration_STOP_IF_Control(void);

void PositionCalibration_Routine(uint32_t *PosCaliSel, PS_t *u )
{
    if ( *PosCaliSel != 0 )
    {
    	PS_CALI_Vars.Calibration_Metho_Select = *PosCaliSel;
        switch ( PS_CALI_Vars.Calibration_Metho_Select )
        {
            case PS_CALI_SEL_AUTO_ZERO_OFFSET:
            {
            	PositionCalibration_Auto_Zero_Offset_Process(u);
            	if (( PS_CALI_Vars.Zero_Offset_State == PS_CALI_ZERO_SM_FINISHED ) || ( PS_CALI_Vars.Zero_Offset_State == PS_CALI_ZERO_SM_ERROR ))
            	{
            		*PosCaliSel = 0;
            	}
            	break;
            }
            case PS_CALI_SEL_LINEARIZATION:
            {
            	PositionCalibration_Linear_Process(u);
            	if (( PS_CALI_Vars.Linear_State == PS_CALI_LINEAR_SM_FINISHED ) || ( PS_CALI_Vars.Linear_State == PS_CALI_LINEAR_SM_ERROR ))
            	{
            		*PosCaliSel = 0;
            	}
            	break;
            }
            default:
            {
            	*PosCaliSel = 0;
            	break;
            }
        }
    }
}

static void PositionCalibration_Auto_Zero_Offset_Process(PS_t *u )
{
	static uint16_t Positioning_Cnt = 0;
    switch ( PS_CALI_Vars.Zero_Offset_State )
    {
        case PS_CALI_ZERO_SM_NONE:
        {
        	Positioning_Cnt = 0;
        	u->MechPosZeroOffset = 0.0f;
        	PositionCalibration_Start_IF_Control();
        	PS_CALI_Vars.Zero_Offset_State = PS_CALI_ZERO_SM_FIND_MECH_ZERO;
        	break;
        }
        case PS_CALI_ZERO_SM_FIND_MECH_ZERO:
        {
        	if ( Positioning_Cnt < DEFAULT_DELAY_TIME_FOR_POSITIONING )
        	{
        		Positioning_Cnt++;
        	}
        	else
        	{
        		PositionCalibration_Rotate_Until_Across_Mech_Zero( u->MechPosition );
        		if ( PS_CALI_Vars.Find_Mech_Zero_State == PS_CALI_FIND0_END )
        		{
        			PS_CALI_Vars.Zero_Offset_State = PS_CALI_ZERO_SM_CAL_OFFSET;
        		}
        		else if ( PS_CALI_Vars.Find_Mech_Zero_State == PS_CALI_FIND0_POSITIONING )
        		{
        			Positioning_Cnt = 0;
        		}
        		else if ( PS_CALI_Vars.Find_Mech_Zero_State == PS_CALI_FIND0_ERROR )
        		{
        			PositionCalibration_STOP_IF_Control();
        			PS_CALI_Vars.Zero_Offset_State = PS_CALI_ZERO_SM_ERROR;
        		}
        	}
        	break;
        }
        case PS_CALI_ZERO_SM_CAL_OFFSET:
        {
        	u->MechPosZeroOffset = u->MechPosition < _PI ? -u->MechPosition : _2PI -u->MechPosition;
        	PositionCalibration_STOP_IF_Control();
        	PS_CALI_Vars.Zero_Offset_State = PS_CALI_ZERO_SM_SAVING_PARAM;
        	break;
        }
        case PS_CALI_ZERO_SM_SAVING_PARAM:
        {
        	DriveParams.SystemParams.MechPositionZeroOffset = (int16_t)( u->MechPosZeroOffset * 10000.0f ) + 32768;
        	DriveFnRegs[FN_PARAM_BACKUP_EMEMORY - FN_BASE] = 1;
        	PS_CALI_Vars.Zero_Offset_State = PS_CALI_ZERO_SM_FINISHED;
        	break;
        }
        case PS_CALI_ZERO_SM_ERROR://TODO: send error messages if it is called by Routine control
        {
    		PS_CALI_Vars.Zero_Offset_State = PS_CALI_ZERO_SM_NONE;
        	break;
        }
        default:
        {
        	break;
        }
    }
}

static void PositionCalibration_Linear_Process(PS_t *u )
{
	static uint16_t Positioning_Cnt = 0;
	static uint8_t LinearPointNow = 0;
    switch ( PS_CALI_Vars.Linear_State )
    {
        case PS_CALI_LINEAR_SM_NONE:
        {
        	Positioning_Cnt = 0;
        	u->MechPosZeroOffset = 0.0f;
        	PositionCalibration_Start_IF_Control();
        	PS_CALI_Vars.Linear_State = PS_CALI_LINEAR_SM_FIND_MECH_ZERO;
        	break;
        }
        case PS_CALI_LINEAR_SM_FIND_MECH_ZERO:
        {
        	if ( Positioning_Cnt < DEFAULT_DELAY_TIME_FOR_POSITIONING )
        	{
        		Positioning_Cnt++;
        	}
        	else
        	{
        		PositionCalibration_Rotate_Until_Across_Mech_Zero( u->MechPosition );
        		if ( PS_CALI_Vars.Find_Mech_Zero_State == PS_CALI_FIND0_END )
        		{
        			PS_CALI_Vars.Linear_State = PS_CALI_LINEAR_SM_FIND_POINTS;
        		}
        		else if ( PS_CALI_Vars.Find_Mech_Zero_State == PS_CALI_FIND0_POSITIONING )
        		{
        			Positioning_Cnt = 0;
        		}
        		else if ( PS_CALI_Vars.Find_Mech_Zero_State == PS_CALI_FIND0_ERROR )
        		{
        			PositionCalibration_STOP_IF_Control();
        			PS_CALI_Vars.Linear_State = PS_CALI_LINEAR_SM_ERROR;
        		}
        	}
        	break;
        }
        case PS_CALI_LINEAR_SM_FIND_POINTS:
        {
        	if ( Positioning_Cnt < DEFAULT_DELAY_TIME_FOR_POSITIONING )
        	{
        		Positioning_Cnt++;
        	}
        	else
        	{
        		if ( LinearPointNow < 31 )
        		{
        			LinearPointsMechPosRad[LinearPointNow] = u->MechPosition;
        			DriveFnRegs[ FN_OPEN_POSITION_CMD - FN_BASE ] =  LinearElePosCmd[LinearPointNow] ;
               		LinearPointNow++;
            		Positioning_Cnt = 0;
        		}
        		else
        		{
        			LinearPointsMechPosRad[LinearPointNow] = u->MechPosition;
        			PositionCalibration_STOP_IF_Control();
               		LinearPointNow = 0;
            		Positioning_Cnt = 0;
            		PS_CALI_Vars.Linear_State = PS_CALI_LINEAR_SM_FINISHED;  //TODO: use communication to configure position sensor in the future.
                	DriveParams.SystemParams.MechPositionZeroOffset = 32768;
                	DriveFnRegs[FN_PARAM_BACKUP_EMEMORY - FN_BASE] = 1;
        		}
        	}
        	break;
        }
        case PS_CALI_LINEAR_SM_SEND_RESULT:
        {
        	break;
        }
        case PS_CALI_LINEAR_SM_ERROR:   //TODO: send error messages if it is called by Routine control
        {
    		PS_CALI_Vars.Linear_State = PS_CALI_LINEAR_SM_NONE;
        	break;
        }
        default:
        {
        	break;
        }
    }
}

static void PositionCalibration_Start_IF_Control(void)
{
	DriveFnRegs[ FN_MF_FUNC_SEL-FN_BASE ] = FN_MF_FUNC_SEL_IF;
	DriveFnRegs[ FN_OPEN_POSITION_CMD_ENABLE - FN_BASE ] = FUNCTION_ENABLE;
	DriveFnRegs[ FN_OPEN_SPD_V_I_LIMIT - FN_BASE ] = DEFAULT_CURRENT_CMD_FOR_POS_CALI;
	DriveFnRegs[ FN_OPEN_POSITION_CMD - FN_BASE ] = 0;
	DriveFnRegs[ FN_ENABLE - FN_BASE ] = FN_ENABLE_MF_START;
	PS_CALI_Vars.Find_Mech_Zero_State = PS_CALI_FIND0_POSITIONING;
	PS_CALI_Vars.Rotate_Direction = UP;
	PS_CALI_Vars.Positioning_Trying_Cnt = 0;
}

static void PositionCalibration_STOP_IF_Control(void)
{
	DriveFnRegs[ FN_MF_FUNC_SEL-FN_BASE ] = FN_MF_FUNC_SEL_RESERVED;
	DriveFnRegs[ FN_OPEN_POSITION_CMD_ENABLE - FN_BASE ] = FUNCTION_DISABLE;
	DriveFnRegs[ FN_OPEN_SPD_V_I_LIMIT - FN_BASE ] = 0;
	DriveFnRegs[ FN_OPEN_POSITION_CMD - FN_BASE ] = 0;
	DriveFnRegs[ FN_ENABLE - FN_BASE ] = FN_ENABLE_STOP;
	PS_CALI_Vars.Positioning_Trying_Cnt = 0;
}

static void PositionCalibration_Rotate_Until_Across_Mech_Zero( float MechPosition )
{
	static float previous_MechPosition = 0.0;
	static uint16_t Rotating_Cnt = 0;
    if ( PS_CALI_Vars.Find_Mech_Zero_State == PS_CALI_FIND0_ROTATING )
    {
	   	if ( Rotating_Cnt < ROTATE_STEPS_FOR_ELE_POS )
	   	{
	   		if ( PS_CALI_Vars.Rotate_Direction == UP )
	   		{
	   	        DriveFnRegs[ FN_OPEN_POSITION_CMD - FN_BASE ] = (uint32_t)(ROTATE_ELE_POS_PER_MS * (float)Rotating_Cnt);
	   		}
	   		else
	   		{
	   	        DriveFnRegs[ FN_OPEN_POSITION_CMD - FN_BASE ] = (uint32_t)(( _2PI * 10000.0f ) - ( ROTATE_ELE_POS_PER_MS * (float)Rotating_Cnt ));
	   		}
	   	    Rotating_Cnt++;
	   	}
	   	else
	   	{
	   		DriveFnRegs[ FN_OPEN_POSITION_CMD - FN_BASE ] = 0;
	   		PS_CALI_Vars.Find_Mech_Zero_State = PS_CALI_FIND0_POSITIONING;
	   		Rotating_Cnt = 0;
	   	}
    }
    else if ( PS_CALI_Vars.Find_Mech_Zero_State == PS_CALI_FIND0_POSITIONING )
    {
   		if ( PS_CALI_Vars.Rotate_Direction == UP )
   		{
            if ( MechPosition >= previous_MechPosition )
            {
            	previous_MechPosition = MechPosition;
           		if ( PS_CALI_Vars.Positioning_Trying_Cnt++ > POSITIONING_TRYING_TIMES )
           		{
            	    PS_CALI_Vars.Find_Mech_Zero_State = PS_CALI_FIND0_ERROR;
           		}
           		else
           		{
            	    PS_CALI_Vars.Find_Mech_Zero_State = PS_CALI_FIND0_ROTATING;
           		}

            }
            else
            {
            	if ( MechPosition > MECH_ZERO_UP_BONDARY )
            	{
            		PS_CALI_Vars.Rotate_Direction = DOWN;
            		PS_CALI_Vars.Find_Mech_Zero_State = PS_CALI_FIND0_ROTATING;
            	}
            	else
            	{
            		previous_MechPosition = 0.0f;
            		PS_CALI_Vars.Find_Mech_Zero_State = PS_CALI_FIND0_END;
            	}
            }
   		}
   		else
   		{
        	if (( MechPosition > MECH_ZERO_UP_BONDARY ) && ( MechPosition < MECH_ZERO_LOW_BONDARY ))
        	{
        		previous_MechPosition = 0.0f;
    		    PS_CALI_Vars.Find_Mech_Zero_State = PS_CALI_FIND0_ERROR;
        	}
        	else
        	{
        		previous_MechPosition = 0.0f;
        		PS_CALI_Vars.Find_Mech_Zero_State = PS_CALI_FIND0_END;
        	}
   		}
    }

}
