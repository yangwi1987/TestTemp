/*
 * FourQuadControl.c
 *
 *  Created on: 2020年3月11日
 *      Author: Mike.Wen.SFW
 */


#include "math.h"
#include "MathFunctionInclude.h"
#include "FourQuadControl.h"

/*================================================================================================================
 * Private Functions Begin
 ================================================================================================================*/

void FourQuadControl_ResetDrive( FourQuadControl *v )
{
	v->Driving_TNIndexPrevious = v->Driving_TNIndex;
}

static void FourQuadControl_DriveTableInit( FourQuadControl *v )
{
	uint16_t n = 0;
	uint16_t m = 0;
	float TimeBase = FOUR_QUAD_TIMEBASE;
	for( n = 0 ; n < DRIVE_TABLE_LENGTH ; n++ )
	{
		for( m = 0 ; m < DRIVE_ALL_PARA_NUM ; m++)
		{
			float Para = ((float)DriveParams.SystemParams.DriveCurve[m][n]);
			Para -= CURVE_PARA_VALUE_SHIFT;
			uint16_t Offset = DRIVE_CURVE_ID_START + m * DRIVE_TABLE_LENGTH + n;
			uint16_t Property = SystemTable.SysParamTableInfoArray[Offset].Property;
			uint16_t Decimal = ( Property & PPD_DECIMAL_MASK ) >> PPD_DECIMAL_SHIFT;
			if ( Decimal > 0 )
			{
				uint16_t i = 0;
				for( i = 0 ; i < Decimal ; i++ )
				{
					Para *= 0.1f;
				}
			}
			else
			{
				uint16_t Power10 = ( Property & PPD_POWER10_MASK ) >> PPD_POWER10_SHIFT;
				if( Power10 == 0 )
				{
					//do nothing
				}
				else
				{
					uint16_t i = 0;
					for( i = 0 ; i < Power10 ; i++ )
					{
						Para *= 10.0f;
					}
				}
			}
			v->DriveCurve[n].Para[m] = Para;
		}
		// Invalid parameter
		if( v->DriveCurve[n].Para[DRIVE_PROPULSION_START] > v->DriveCurve[n].Para[DRIVE_PROPULSION_MAX] || v->DriveCurve[n].Para[DRIVE_PROPULSION_START] < 0.0 || v->DriveCurve[n].Para[DRIVE_SPEED_MAX] < 0.0 || v->DriveCurve[n].Para[DRIVE_PROPULSION_MAX] < 0.0 || v->DriveCurve[n].Para[DRIVE_POWER_MAX] < 0.0 )
		{
			v->DriveCurve[n].Para[DRIVE_PROPULSION_MAX] = 0.0;
			v->DriveCurve[n].Para[DRIVE_SPEED_MAX] = 0.0;
			v->DriveCurve[n].Para[DRIVE_POWER_MAX] = 0.0;
			v->InitFailure = FourQuadState_Driving_I;
		}
	}

	float Para = ((float)DriveParams.SystemParams.DriveRisingRamp);
	Para -= CURVE_PARA_VALUE_SHIFT;
	Para *= 0.1f;
	v->DriveRisingRamp = Para * TimeBase;

	Para = ((float)DriveParams.SystemParams.DriveFallingRamp);
	Para -= CURVE_PARA_VALUE_SHIFT;
	Para *= 0.1f;
	v->DriveFallingRamp = -Para * TimeBase; //Ramp Function Use Absolute Ramp Value

	Para = ((float)DriveParams.SystemParams.DriveRampReverse);
	Para *= 0.1f;
	v->DriveRampReverse = Para * TimeBase; //Ramp Function Use Absolute Ramp Valu

	// Load Limp Transit variable
	float LimpTransitSec = 1.0f;
	float MaxPower = 0.0f;
	ParamMgr_ParaGainHandler( &(DriveParams), &(DriveParams.SystemParams.LimpTransitSec), &LimpTransitSec );
	for( n = 0 ; n < DRIVE_TABLE_LENGTH ; n++ )
	{
		MaxPower = ( MaxPower > v->DriveCurve[n].Para[DRIVE_POWER_MAX] ) ? MaxPower : v->DriveCurve[n].Para[DRIVE_POWER_MAX];
	}
	v->LimpTransitRamp = (( MaxPower - v->DriveCurve[0].Para[DRIVE_POWER_MAX] ) / LimpTransitSec ) * TimeBase;

}

static void FourQuadControl_DCCurrLimitComparatorInit( FourQuadControl *v )
{
	float TimeBase = 0.001f;
	v->DrainRisingRamp = ( ((float)DriveParams.SystemParams.DrainRisingRamp) - CURVE_PARA_VALUE_SHIFT ) * TimeBase;
	v->DrainFallingRamp = -( ((float)DriveParams.SystemParams.DrainFallingRamp) - CURVE_PARA_VALUE_SHIFT ) * TimeBase;
}

static float FourQuadControl_CalcDriveTable( FourQuadControl *v )
{
	DRIVE_TABLE_TYPE *pTable = &(v->DriveCurveNow[v->Driving_TNIndex]);
	float PropulseOut = 0.0f;
	float PropulseMin = 0.0f;
	float Smax = 0.0f;
	float ABSMotorRPM = ABS(v->MotorRPM);

	if ( v->spdPnltyOvrd == 1 )
	{
		Smax = MIN2(pTable->Para[DRIVE_SPEED_MAX], v->SpdPnlty);
	}
	else
	{
		Smax = pTable->Para[DRIVE_SPEED_MAX];
	}

	if ( v->Driving_TNIndex == DRIVE_TABLE_LIMP_NOW )
	{
		v->DrivePowerCmd = Ramp( v->DrivePowerCmd, pTable->Para[DRIVE_POWER_MAX], v->LimpTransitRamp );
	}
	else
	{
		v->DrivePowerCmd =  pTable->Para[DRIVE_POWER_MAX] * v->facPnlty;
	}

	float F1 = ABSMotorRPM * pTable->Para[DRIVE_SLOPE_START] + pTable->Para[DRIVE_PROPULSION_START] ;
	float F2 = pTable->Para[DRIVE_PROPULSION_MAX] * v->facPnlty;
	float F3 = ( ABSMotorRPM > 1.0f ) ? v->DrivePowerCmd / ( ABSMotorRPM * RPM_TO_SPEED ): v->DrivePowerCmd;
	float F4 = ( ABSMotorRPM - Smax ) * pTable->Para[DRIVE_SLOPE_END];
	PropulseOut = ( F1 < F2 ) ? F1 : F2;
	PropulseOut = ( PropulseOut < F3 ) ? PropulseOut : F3;
	PropulseOut = ( PropulseOut < F4 ) ? PropulseOut : F4;
	PropulseOut = ( PropulseOut < PropulseMin ) ? PropulseMin : PropulseOut;

	float ChangeTime = 0.0f;
	if ( PropulseOut >= v->DrivePropulsion )
	{
		if ( v->DriveRisingRamp != 0.0f )
		{
			ChangeTime = ( PropulseOut - v->DrivePropulsion ) / v->DriveRisingRamp * FOUR_QUAD_TIMEBASE;
		}
		else
		{
			ChangeTime = 10000.0f;
		}
	}
	else
	{
		if ( v->DriveFallingRamp != 0.0f )
		{
			ChangeTime = ( v->DrivePropulsion - PropulseOut ) / v->DriveFallingRamp * FOUR_QUAD_TIMEBASE;
		}
		else
		{
			ChangeTime = 10000.0f;
		}
	}
	ChangeTime = ( ChangeTime >= 0.0f ) ? ChangeTime : -ChangeTime;
	v->DriveChangeTime = ChangeTime;
	v->DrivePropulsion = SmoothCurveChange( PropulseOut, v->DrivePropulsion, &(v->Driving_TNIndex) , &(v->Driving_TNIndexPrevious), v->DriveRisingRamp, v->DriveFallingRamp );
	return v->DrivePropulsion;
}

static float FourQuadControl_CalcBackrollTable( FourQuadControl *v )
{
	DRIVE_TABLE_TYPE *pTable = &(v->DriveCurveNow[v->Driving_TNIndex]);
	float PropulseOut = 0.0f;
	float PropulseMin = 0.0f;
	float Smax = pTable->Para[DRIVE_SPEED_MAX];
	float ABSMotorRPM = ABS(v->MotorRPM);

	float F1 = ABSMotorRPM * pTable->Para[DRIVE_SLOPE_START] + pTable->Para[DRIVE_PROPULSION_START] ;
	float F2 = pTable->Para[DRIVE_PROPULSION_MAX] * v->facPnlty;
	float F3 = ( ABSMotorRPM > 1.0f ) ? v->DrivePowerCmd / ( ABSMotorRPM * RPM_TO_SPEED ): v->DrivePowerCmd;
	float F4 = ( ABSMotorRPM - Smax ) * pTable->Para[DRIVE_SLOPE_END];
	PropulseOut = ( F1 < F2 ) ? F1 : F2;
	PropulseOut = ( PropulseOut < F3 ) ? PropulseOut : F3;
	PropulseOut = ( PropulseOut < F4 ) ? PropulseOut : F4;
	PropulseOut = ( PropulseOut < PropulseMin ) ? PropulseMin : PropulseOut;

	if ( v->DrivePropulsion < 0.0f )
	{
		v->DrivePropulsion = Ramp(v->DrivePropulsion, 0.0f, v->DriveRampReverse);
	}
	else
	{
		v->DrivePropulsion = SmoothCurveChange( PropulseOut, v->DrivePropulsion, &(v->Driving_TNIndex) , &(v->Driving_TNIndexPrevious), v->DriveRisingRamp, v->DriveFallingRamp );
	}

	return v->DrivePropulsion;
}

static float FourQuadControl_CalcReverseTable( FourQuadControl *v )
{
	DRIVE_TABLE_TYPE *pTable = &(v->DriveCurveNow[v->Driving_TNIndex]);
	float PropulseOut = 0.0f;
	float PropulseMin = 0.0f;
	float Smax = pTable->Para[DRIVE_SPEED_MAX];
	float ABSMotorRPM = ABS(v->MotorRPM);

	float F1 = ABSMotorRPM * pTable->Para[DRIVE_SLOPE_START] + pTable->Para[DRIVE_PROPULSION_START] ;
	float F2 = pTable->Para[DRIVE_PROPULSION_MAX] * v->facPnlty;
	float F3 = ( ABSMotorRPM > 1.0f ) ? v->DrivePowerCmd / ( ABSMotorRPM * RPM_TO_SPEED ): v->DrivePowerCmd;
	float F4 = ( ABSMotorRPM - Smax ) * pTable->Para[DRIVE_SLOPE_END];
	PropulseOut = ( F1 < F2 ) ? F1 : F2;
	PropulseOut = ( PropulseOut < F3 ) ? PropulseOut : F3;
	PropulseOut = ( PropulseOut < F4 ) ? PropulseOut : F4;
	PropulseOut = ( PropulseOut < PropulseMin ) ? PropulseMin : PropulseOut;

	v->DrivePropulsion = Ramp(v->DrivePropulsion, -PropulseOut, v->DriveRampReverse);
	return v->DrivePropulsion;
}

/*================================================================================================================
 * Public Functions Begin
 ================================================================================================================*/

void FourQuadControl_Init( FourQuadControl *v )
{

	// Calc characteristic points of table, avoid discontinuous points
	FourQuadControl_DriveTableInit(v);
	FourQuadControl_ResetDrive(v);
	FourQuadControl_DCCurrLimitComparatorInit(v);
}

void FourQuadControl_Switch( FourQuadControl *v )
{
	int16_t MotorRPMTmp = (int16_t)( v->MotorRPM );

	//Switch Control
	if( v->GearPositionState == VIRTUAL_GEAR_N )
	{
		uint8_t DRIVE_TABLE_OFFSET = 0;
		v->FourQuadState = FourQuadState_None;

		switch ( v->DriveTableSelect )
		{
		case 0:
		{
			DRIVE_TABLE_OFFSET = DRIVE_TABLE_MINI_NORMAL;
			break;
		}
		default:
		{
			DRIVE_TABLE_OFFSET = 0;
			break;
		}
		}

		for( uint8_t m = 0 ; m < DRIVE_ALL_PARA_NUM ; m++)
		{
		    v->DriveCurveNow[DRIVE_TABLE_LIMP_NOW].Para[m] = v->DriveCurve[DRIVE_TABLE_LIMP].Para[m];
		    v->DriveCurveNow[DRIVE_TABLE_REVERSE_NOW].Para[m] = v->DriveCurve[DRIVE_TABLE_REVERSE].Para[m];
		    v->DriveCurveNow[DRIVE_TABLE_NORMAL_NOW].Para[m] = v->DriveCurve[DRIVE_TABLE_OFFSET].Para[m];
		    v->DriveCurveNow[DRIVE_TABLE_BOOST_NOW].Para[m] = v->DriveCurve[DRIVE_TABLE_OFFSET + 1].Para[m];
		}
	}
	else if( v->GearPositionState == VIRTUAL_GEAR_D )
	{
		if( MotorRPMTmp >= 0 )
		{
			v->FourQuadState = FourQuadState_Driving_I;
		}
		else
		{
			v->FourQuadState = FourQuadState_BackRoll_II;
		}
	}
	else if ( v->GearPositionState == VIRTUAL_GEAR_R )
	{
		if( MotorRPMTmp <= 0 )
		{
			v->FourQuadState = FourQuadState_Reverse_III;
		}
		else
		{
			v->FourQuadState = FourQuadState_Regen_IV;
		}
	}

}

void FourQuadControl_Calc( FourQuadControl *v, uint8_t TriggerLimpHome  )
{
	float CalcTorque = 0.0;
	//Calculate Four-Quad ScooterPropulsion
	switch (v->FourQuadState)
	{
		case FourQuadState_Driving_I:
		{
			if ( TriggerLimpHome == 1)
			{
				v->Driving_TNIndex = DRIVE_TABLE_LIMP_NOW;
			}
			else if( v->BoostState == BOOST_MODE_ENABLE )
			{
				v->Driving_TNIndex = DRIVE_TABLE_BOOST_NOW;
			}
			else
			{
				v->Driving_TNIndex = DRIVE_TABLE_NORMAL_NOW;
			}
			CalcTorque = FourQuadControl_CalcDriveTable(v);
			CalcTorque = CalcTorque * DRIVE_PROPULSION_TOLERANCE;
			break;
		}
		case FourQuadState_BackRoll_II:
		{
			v->Driving_TNIndex = DRIVE_TABLE_LIMP_NOW;
			CalcTorque = FourQuadControl_CalcBackrollTable(v);
			CalcTorque = CalcTorque * DRIVE_PROPULSION_TOLERANCE;
			break;
		}
		case FourQuadState_Reverse_III:
		{
			v->Driving_TNIndex = DRIVE_TABLE_REVERSE_NOW;
			CalcTorque = FourQuadControl_CalcReverseTable(v);
			CalcTorque = CalcTorque * DRIVE_PROPULSION_TOLERANCE;
			break;
		}
		case FourQuadState_Regen_IV:
		{
			v->Driving_TNIndex = DRIVE_TABLE_REVERSE_NOW;
			v->DrivePropulsion = Ramp(v->DrivePropulsion, 0.0f, v->DriveRampReverse);
			CalcTorque = v->DrivePropulsion * DRIVE_PROPULSION_TOLERANCE;
			break;
		}
		default:
		{
			CalcTorque = 0;
			FourQuadControl_ResetDrive(v);
			break;
		}
	}

	v->TorqueCommandOut = v->ratAPP * CalcTorque;
}

void FourQuadControl_Reset( FourQuadControl *v, float DCDrainLimit )
{
	v->TorqueCommandOut = 0.0;
	v->DCDrainLimit = DCDrainLimit;

	DRIVE_TABLE_TYPE *pTable = &(v->DriveCurve[v->Driving_TNIndex]);
	v->DrivePowerCmd = pTable->Para[DRIVE_POWER_MAX];
}

float FourQuadControl_DCCurrLimitComparator( FourQuadControl *v, float DCDrainCurr, float VbusReal, float VbusUsed )
{
	float DCCurrLimit = 0.0f;
	float Target = 0.0f;
	float CurrRamp = 0.0f;

	v->MaxDcLimitRecord = DCDrainCurr;

	Target = v->MaxDcLimitRecord;
	if( Target > v->DCDrainLimit )
	{
		CurrRamp = v->DrainRisingRamp;
	}
	else
	{
		CurrRamp = v->DrainFallingRamp;
	}
	v->DCDrainLimit = Ramp( v->DCDrainLimit, Target, CurrRamp );
	DCCurrLimit = v->DCDrainLimit;

	DCCurrLimit = DCCurrLimit * VbusReal / VbusUsed;
	return DCCurrLimit;
}
