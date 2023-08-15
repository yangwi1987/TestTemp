/*
 * FourQuadControl.c
 *
 *  Created on: 2020年3月11日
 *      Author: Mike.Wen.SFW
 */


#include "math.h"
#include "MathFunctionInclude.h"
#include "FourQuadControl.h"

#define ABS(x) 	( (x) > 0 ? (x) : -(x) )

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
			v->TmaxSpeed = 0.0;
			v->InitFailure = FourQuadState_Driving_I;
		}
	}

	float Para = ((float)DriveParams.SystemParams.DriveRisingRamp);
	Para -= CURVE_PARA_VALUE_SHIFT;
	uint16_t Offset = DRIVE_CURVE_ID_START + DRIVE_ALL_PARA_NUM * DRIVE_TABLE_LENGTH;
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
	v->DriveRisingRamp = Para * TimeBase;

	Para = ((float)DriveParams.SystemParams.DriveFallingRamp);
	Para -= CURVE_PARA_VALUE_SHIFT;
	Offset = DRIVE_CURVE_ID_START + DRIVE_ALL_PARA_NUM * DRIVE_TABLE_LENGTH + 1;
	Property = SystemTable.SysParamTableInfoArray[Offset].Property;
	Decimal = ( Property & PPD_DECIMAL_MASK ) >> PPD_DECIMAL_SHIFT;
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
	v->DriveFallingRamp = -Para * TimeBase; //Ramp Function Use Absolute Ramp Value

	// Load Limp ramp

	Para = ((float)DriveParams.SystemParams.LimpRisingRamp);
	Para -= CURVE_PARA_VALUE_SHIFT;
	Offset = DRIVE_CURVE_ID_START + DRIVE_ALL_PARA_NUM * DRIVE_TABLE_LENGTH + 2;
	Property = SystemTable.SysParamTableInfoArray[Offset].Property;
	Decimal = ( Property & PPD_DECIMAL_MASK ) >> PPD_DECIMAL_SHIFT;
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
	v->LimpRisingRamp = Para * TimeBase;

	Para = ((float)DriveParams.SystemParams.LimpFallingRamp);
	Para -= CURVE_PARA_VALUE_SHIFT;
	Offset = DRIVE_CURVE_ID_START + DRIVE_ALL_PARA_NUM * DRIVE_TABLE_LENGTH + 3;
	Property = SystemTable.SysParamTableInfoArray[Offset].Property;
	Decimal = ( Property & PPD_DECIMAL_MASK ) >> PPD_DECIMAL_SHIFT;
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
	v->LimpFallingRamp = -Para * TimeBase; //Ramp Function Use Absolute Ramp Value

	v->DrivePowerLevelTarget = 1.0f;
	v->DrivePowerLevelGain = 1.0f;
	uint16_t ParaError = 0.0f;
	float DrivePowerRampUpFullThrottle = 0.0f;
	float DrivePowerRampDownFullThrottle = 0.0f;
	float DrivePowerRampUpZeroThrottle = 0.0f;
	float DrivePowerRampDownZeroThrottle = 0.0f;
	if( DriveParams.SystemParams.DrivePowerRampUpFullThrottle > DriveParams.SystemParams.DrivePowerRampUpZeroThrottle )
	{
		ParaError = 1;
	}
	if( DriveParams.SystemParams.DrivePowerRampDownFullThrottle < DriveParams.SystemParams.DrivePowerRampDownZeroThrottle )
	{
		ParaError = 1;
	}
	if( ParaError == 0 )
	{
		ParaError |= ParamMgr_ParaGainHandler( &(DriveParams), &(DriveParams.SystemParams.DrivePowerRampUpFullThrottle), &(DrivePowerRampUpFullThrottle) );
		ParaError |= ParamMgr_ParaGainHandler( &(DriveParams), &(DriveParams.SystemParams.DrivePowerRampDownFullThrottle), &(DrivePowerRampDownFullThrottle) );
		ParaError |= ParamMgr_ParaGainHandler( &(DriveParams), &(DriveParams.SystemParams.DrivePowerRampUpZeroThrottle), &(DrivePowerRampUpZeroThrottle) );
		ParaError |= ParamMgr_ParaGainHandler( &(DriveParams), &(DriveParams.SystemParams.DrivePowerRampDownZeroThrottle), &(DrivePowerRampDownZeroThrottle) );
	}
	else
	{
		//do nothing
	}

	if( ParaError == 0 )
	{
		for( n = 0 ; n < DRIVE_TABLE_LENGTH ; n++ )
		{
			if( v->DriveCurve[n].Para[DRIVE_POWER_MAX] > 1.0f ) //avoid to divide 0
			{
				float ParamA = 0.0f;
				float ParamB = 0.0f;
				// Calculate Parameter of Up Ramp
				ParamA = ( DrivePowerRampUpFullThrottle - DrivePowerRampUpZeroThrottle );
				ParamB = DrivePowerRampUpZeroThrottle;
				v->DrivePowerLevelRampUpParamA[n] =  ParamA / v->DriveCurve[n].Para[DRIVE_POWER_MAX] * TimeBase;
				v->DrivePowerLevelRampUpParamB[n] =  ParamB / v->DriveCurve[n].Para[DRIVE_POWER_MAX] * TimeBase;
				// Calculate Parameter of Down Ramp
				ParamA = ( DrivePowerRampDownFullThrottle - DrivePowerRampDownZeroThrottle );
				ParamB = DrivePowerRampDownZeroThrottle;
				v->DrivePowerLevelRampDownParamA[n] =  -ParamA / v->DriveCurve[n].Para[DRIVE_POWER_MAX] * TimeBase;
				v->DrivePowerLevelRampDownParamB[n] =  -ParamB / v->DriveCurve[n].Para[DRIVE_POWER_MAX] * TimeBase;
			}
			else
			{
				v->DrivePowerLevelRampUpParamA[n] = 0.0f;
				v->DrivePowerLevelRampUpParamB[n] = 0.0f;
				v->DrivePowerLevelRampDownParamA[n] = 0.0f;
				v->DrivePowerLevelRampDownParamB[n] = 0.0f;
			}
		}
	}
	else
	{
		for( n = 0 ; n < DRIVE_TABLE_LENGTH ; n++ )
		{
			v->DrivePowerLevelRampUpParamA[n] = 0.0f;
			v->DrivePowerLevelRampUpParamB[n] = 0.0f;
			v->DrivePowerLevelRampDownParamA[n] = 0.0f;
			v->DrivePowerLevelRampDownParamB[n] = 0.0f;
		}
	}
}

static void FourQuadControl_BackRollTableInit( FourQuadControl *v )
{
	BACK_ROLL_TABLE_TYPE *pTable = &v->BackRollTable;
	// Invalid parameter
	if( pTable->SpeedMax > 0.0 || pTable->PropulsionMax < 0.0 )
	{
		pTable->PropulsionMax = 0.0;
		pTable->SpeedMax = 0.0;
		v->InitFailure = FourQuadState_BackRoll_II;
	}

	// To ensure the torque command smoothness around zero speed
	v->BackRollTable.PropulsionMax = 1415.6; //v->FourQuadTable.PerformanceTable.Para[DRIVE_PROPULSION_START];
}

static void FourQuadControl_DCCurrLimitComparatorInit( FourQuadControl *v )
{
	float TimeBase = 0.001f;
	v->DrainRisingRamp = ( ((float)DriveParams.SystemParams.DrainRisingRamp) - CURVE_PARA_VALUE_SHIFT ) * TimeBase;
	v->DrainFallingRamp = -( ((float)DriveParams.SystemParams.DrainFallingRamp) - CURVE_PARA_VALUE_SHIFT ) * TimeBase;
}

static float FourQuadControl_CalcDriveTable( FourQuadControl *v )
{
	DRIVE_TABLE_TYPE *pTable = &(v->DriveCurve[v->Driving_TNIndex]);
	float PropulseOut = 0.0f;
	float PropulseMin = 0.0f;
	float Smax = pTable->Para[DRIVE_SPEED_MAX] ;

	if( v->DrivePowerLevelTarget > v->DrivePowerLevelGain )
	{
		v->DrivePowerLevelRampUp = v->Throttle * v->DrivePowerLevelRampUpParamA[v->Driving_TNIndex] + v->DrivePowerLevelRampUpParamB[v->Driving_TNIndex];
		v->DrivePowerLevelGain = Ramp( v->DrivePowerLevelGain, v->DrivePowerLevelTarget, v->DrivePowerLevelRampUp );
	}
	else if( v->DrivePowerLevelTarget < v->DrivePowerLevelGain )
	{
		v->DrivePowerLevelRampDown = v->Throttle * v->DrivePowerLevelRampDownParamA[v->Driving_TNIndex] + v->DrivePowerLevelRampDownParamB[v->Driving_TNIndex];
		v->DrivePowerLevelGain = Ramp( v->DrivePowerLevelGain, v->DrivePowerLevelTarget, v->DrivePowerLevelRampDown );
	}
	else
	{
		// do nothing
	}
	v->DrivePowerCmd = pTable->Para[DRIVE_POWER_MAX] * v->DrivePowerLevelGain;
	v->MotorRPM = 5000;
	float F1 = v->MotorRPM * pTable->Para[DRIVE_SLOPE_START] + pTable->Para[DRIVE_PROPULSION_START] ;
	float F2 = pTable->Para[DRIVE_PROPULSION_MAX];
	float F3 = ( v->MotorRPM > 1.0f ) ? v->DrivePowerCmd / ( v->MotorRPM * RPM_TO_SPEED ): v->DrivePowerCmd;
	float F4 = ( v->MotorRPM - Smax ) * pTable->Para[DRIVE_SLOPE_END];
	PropulseOut = ( F1 < F2 ) ? F1 : F2;
	PropulseOut = ( PropulseOut < F3 ) ? PropulseOut : F3;
	PropulseOut = ( PropulseOut < F4 ) ? PropulseOut : F4;
	PropulseOut = ( PropulseOut < PropulseMin ) ? PropulseMin : PropulseOut;

	float ChangeTime = 0.0f;
	float RisingRamp = 0.0f;
	float FallingRamp = 0.0f;

	if ( v->Driving_TNIndex == 0 )
	{
		RisingRamp = v->LimpRisingRamp;
		FallingRamp = v->LimpFallingRamp;
	}
	else
	{
		RisingRamp = v->DriveRisingRamp;
		FallingRamp = v->DriveFallingRamp;
	}

	if ( PropulseOut >= v->DrivePropulsion )
	{
		if ( RisingRamp != 0.0f )
		{
			ChangeTime = ( PropulseOut - v->DrivePropulsion ) / RisingRamp * FOUR_QUAD_TIMEBASE;
		}
		else
		{
			ChangeTime = 10000.0f;
		}
	}
	else
	{
		if ( FallingRamp != 0.0f )
		{
			ChangeTime = ( v->DrivePropulsion - PropulseOut ) / FallingRamp * FOUR_QUAD_TIMEBASE;
		}
		else
		{
			ChangeTime = 10000.0f;
		}
	}
	ChangeTime = ( ChangeTime >= 0.0f ) ? ChangeTime : -ChangeTime;
	v->DriveChangeTime = ChangeTime;
	v->DrivePropulsion = SmoothCurveChange( PropulseOut, v->DrivePropulsion, &(v->Driving_TNIndex) , &(v->Driving_TNIndexPrevious), RisingRamp, FallingRamp );
	return v->DrivePropulsion;
}

static float FourQuadControl_CalcBackRollTable( FourQuadControl *v )
{
	BACK_ROLL_TABLE_TYPE *pTable = &v->BackRollTable;
	float PropulseOut = 0.0;

	if( v->MotorRPM < pTable->SpeedMax )
	{
		PropulseOut = 0.0;
	}
	else
	{
		PropulseOut = v->DriveCurve[v->Driving_TNIndexPrevious].Para[DRIVE_PROPULSION_START] * DRIVE_PROPULSION_TOLERANCE;
	}
	return PropulseOut;
}

/*================================================================================================================
 * Public Functions Begin
 ================================================================================================================*/

void FourQuadControl_Init( FourQuadControl *v )
{
	// Assign Tables
	v->BackRollTable = SystemTable.BackRollTable;

	// Calc characteristic points of table, avoid discontinuous points
	FourQuadControl_DriveTableInit(v);
	FourQuadControl_BackRollTableInit(v);
	FourQuadControl_ResetDrive(v);
	FourQuadControl_DCCurrLimitComparatorInit(v);
}

void FourQuadControl_Switch( FourQuadControl *v )
{
	int16_t MotorRPMTmp = (int16_t)( v->MotorRPM );

	//Switch Control
	if( v->GearPositionState == PcuShiftP )
	{
		v->ServoCmdOut = 0;
		v->FourQuadState = FourQuadState_None;

		if( v->GearPositionCmd == PcuShiftD && v->ThrottleReleaseFlg == 1 )
		{
			v->GearPositionState = PcuShiftD;
		}
	}
	else if( v->GearPositionState == PcuShiftD )
	{
		v->ServoCmdOut = 1;

		if( v->FirstEntryFlg == 1 && v->ThrottleReleaseFlg == 0 )
		{
			v->FourQuadState = FourQuadState_None;
		}
		else if( MotorRPMTmp < 0 )
		{
			v->FourQuadState = FourQuadState_BackRoll_II;
			v->FirstEntryFlg = 0;
		}
		else
		{
			v->FourQuadState = FourQuadState_Driving_I;
			v->FirstEntryFlg = 0;
		}

		if( v->GearPositionCmd == PcuShiftP )
		{
			v->GearPositionState = PcuShiftP;
			v->FirstEntryFlg = 1;
		}
	}

	v->ServoCmdOut = ( v->ServoCmdIn == 0 )? 0 : v->ServoCmdOut;
}

void FourQuadControl_Calc( FourQuadControl *v )
{
	float CalcTorque = 0.0;

	//Calculate Four-Quad ScooterPropulsion
	switch (v->FourQuadState)
	{
		case FourQuadState_Driving_I:
		{
			CalcTorque = FourQuadControl_CalcDriveTable(v);
			CalcTorque = CalcTorque * DRIVE_PROPULSION_TOLERANCE;
			break;
		}
		case FourQuadState_BackRoll_II:
		{
			CalcTorque = FourQuadControl_CalcBackRollTable(v);
			FourQuadControl_ResetDrive(v);
			break;
		}
		default:
		{
			CalcTorque = 0;
			FourQuadControl_ResetDrive(v);
			break;
		}
	}

	v->TorqueCommandOut = CalcTorque;
}

void FourQuadControl_Reset( FourQuadControl *v, float DCDrainLimit )
{
	v->TorqueCommandOut = 0.0;
	v->DCDrainLimit = DCDrainLimit;

	DRIVE_TABLE_TYPE *pTable = &(v->DriveCurve[v->Driving_TNIndex]);
	v->DrivePowerLevelGain = v->DrivePowerLevelTarget;
	v->DrivePowerCmd = pTable->Para[DRIVE_POWER_MAX] * v->DrivePowerLevelGain;
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
