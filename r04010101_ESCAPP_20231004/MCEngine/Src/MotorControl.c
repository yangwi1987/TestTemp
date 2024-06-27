/*
 * MotorControl.c
 *
 *  Created on: 2019年12月6日
 *      Author: Fernando
 */
#include "MotorControl.h"
#include "UtilityBase.h"

inline void MotorControl_CurrentControlSetRegulatorParameter( CURRENT_CONTROL_TYPE *pSet, MOTOR_PARAMETER_TYPE* pMotor, float IdHz, float IqHz, float PwmPeriod );
void MotorControl_FluxWeakeningSetRegulatorParameter( CURRENT_CONTROL_FLUX_WEAKENING_TYPE *pSet, MOTOR_PARAMETER_TYPE* pMotor, float Bandwith, float PwmPeriod, float Ilimit, float IdKp, float IqKp );
inline void MotorControl_SixWaveRegulatorParameter( SIX_WAVE_120_CURRENT_CONTROL_TYPE *pSet, MOTOR_PARAMETER_TYPE* pMotor, float PwmPeriod , float Hz);
void MotorControl_CalcPMFOCDCPPID( float *Ki, float Hz, float Period, float L, float Res);
ROTOR_TO_STATOR_TYPE IstatorCmd = ROTOR_TO_STATOR_DEFAULT;
STATOR_TO_PHASE_TYPE IphaseCmd = STATOR_TO_PHASE_DEFAULT;

PHASE_TO_STATOR_TYPE StatorCurrFbTmp = PHASE_TO_STATOR_DEFAULT;

//__attribute__(( section(".ram_function"))) void __attribute__((optimize("Ofast"))) MotorControl_Algorithm( MOTOR_CONTROL_TYPE *p, uint16_t FunctionMode)

__attribute__(( section(".ram_function"))) void MotorControl_Algorithm( MOTOR_CONTROL_TYPE *p, uint16_t FunctionMode)

{

	float VbusLimit = 0.0f;
	float VsSaturation = 0.0f;
	float DevideVbus = 0.0f;
	float SinValue = 0.0f;
	float CosValue = 0.0f;

	//Assign the related feedback and limitation from Sensor to Control	: 1.6us
	COORDINATE_TRANSFER_Phase_to_Stator_Calc( (p->SensorFb.Iu), (p->SensorFb.Iv), (p->SensorFb.Iw), (&StatorCurrFbTmp) );

	DevideVbus = p->SensorFb.Vbus > 32.0f ? 1.0f / p->SensorFb.Vbus : 0.03125f;  // Vbus value limit for PWM calculation
	VbusLimit = 0.577350269f * (p->SensorFb.Vbus * p->TorqueToIdq.VbusGainForFW); // 0.577350269f line voltage to phase
	VsSaturation = 0.577350269f * (p->SensorFb.Vbus * p->TorqueToIdq.VbusGainForVsaturation);
	if (( FunctionMode == FUNCTION_MODE_NORMAL_CURRENT_CONTROL ))
	{
		//CmdFrom = CMD_FROM_FOC_CTRL;
		if ( p->CurrentControl.EnableDirectIdqCmd == FUNCTION_ENABLE)
		{
			p->CurrentControl.IdCmd = p->Cmd.IdCmd;
			p->CurrentControl.IqCmd = p->Cmd.IqCmd;
		}
		else
		{
			p->Cmd.IdCmd = p->TorqueToIdq.IdCmd;
			p->Cmd.IqCmd = p->TorqueToIdq.IqCmd;
			FLUX_WEAKENING_Calc( (&(p->CurrentControl.FluxWeakening)), (p->VoltCmd.VcmdAmp), (VbusLimit), (p->Cmd.IdCmd), (p->Cmd.IqCmd) );
			p->CurrentControl.IdCmd = p->CurrentControl.FluxWeakening.IdCmd;
			p->CurrentControl.IqCmd = p->CurrentControl.FluxWeakening.IqCmd;
		}

		p->CurrentControl.StatorCurrFb.Alpha = StatorCurrFbTmp.Alpha;
		p->CurrentControl.StatorCurrFb.Beta = StatorCurrFbTmp.Beta;

		// Iab to Idq : 9.4us = 11us - 1.6us
		COORDINATE_TRANSFER_GET_SIN_COS( (p->CurrentControl.EleAngle), SinValue, CosValue );
//		COORDINATE_TRANSFER_GetSinCos_LT((uint16_t)(p->CurrentControl.EleAngle * 651.8986f ), &SinValue, &CosValue);  //
		COORDINATE_TRANSFER_Stator_to_Rotor_Calc( (p->CurrentControl.StatorCurrFb.Alpha), (p->CurrentControl.StatorCurrFb.Beta), SinValue, CosValue, (&(p->CurrentControl.RotorCurrFb)) );

		//CtrlMode = CTRL_MODE_CURRENT_CONTROL;
		p->CurrentControl.IdRegulator.CalcNoLimit( p->CurrentControl.IdCmd, p->CurrentControl.RotorCurrFb.D, &(p->CurrentControl.IdRegulator) );
		p->CurrentControl.IqRegulator.CalcNoLimit( p->CurrentControl.IqCmd, p->CurrentControl.RotorCurrFb.Q, &(p->CurrentControl.IqRegulator) );

		float EleSpeedTmp = 0.0f;
		EleSpeedTmp = p->CurrentControl.EleSpeed;
		p->CurrentControl.Decoupling.PIDWayId.CalcNoLimit( (EleSpeedTmp * p->CurrentControl.IqCmd), (EleSpeedTmp * p->CurrentControl.RotorCurrFb.Q), &(p->CurrentControl.Decoupling.PIDWayId) );
		p->CurrentControl.Decoupling.PIDWayIq.CalcNoLimit( (EleSpeedTmp * p->CurrentControl.IdCmd), (EleSpeedTmp * p->CurrentControl.RotorCurrFb.D), &(p->CurrentControl.Decoupling.PIDWayIq) );
		p->VoltCmd.VdCmd = p->CurrentControl.IdRegulator.Output - p->CurrentControl.Decoupling.PIDWayId.Output;
		p->VoltCmd.VqCmd = p->CurrentControl.IqRegulator.Output + p->CurrentControl.Decoupling.PIDWayIq.Output;

		p->VoltCmd.EleCompAngle = p->CurrentControl.EleAngle + p->CurrentControl.EleSpeed * p->CurrentControl.PwmPeriod * 1.5f;

		//PwmMode = PWM_MODE_SVPWM;
		//Limit voltage command Vdq : 2us = 16.6us-14.6us
		p->VoltCmd.VcmdAmp = sqrtf( p->VoltCmd.VdCmd * p->VoltCmd.VdCmd + p->VoltCmd.VqCmd * p->VoltCmd.VqCmd );

		if ( p->VoltCmd.VcmdAmp > VsSaturation)
		{
			float VgainSatCmd = 0.0f;
			float VgainSatFluxWeakening = 0.0f;

			VgainSatCmd = VsSaturation / p->VoltCmd.VcmdAmp;
			p->VoltCmd.VdCmd *= VgainSatCmd; // limit to 93.5%
			p->VoltCmd.VqCmd *= VgainSatCmd; // limit to 93.5%
			VgainSatFluxWeakening = VgainSatCmd * 1.048f; // limit to 97.988%.

			p->CurrentControl.Decoupling.PIDWayId.Ui *= VgainSatFluxWeakening;
			p->CurrentControl.Decoupling.PIDWayIq.Ui *= VgainSatFluxWeakening;
			p->CurrentControl.IdRegulator.Ui *= VgainSatFluxWeakening;
			p->CurrentControl.IqRegulator.Ui *= VgainSatFluxWeakening;
		}
		//Vdq to Vab : 8.6us = 24.2us - 16.6us
		COORDINATE_TRANSFER_GET_SIN_COS( (p->VoltCmd.EleCompAngle), SinValue, CosValue )
		COORDINATE_TRANSFER_Rotor_to_Stator_Calc( (p->VoltCmd.VdCmd), (p->VoltCmd.VqCmd), SinValue, CosValue, (&(p->VoltCmd.StatorVoltCmd)) );
		//Calcuclate the PWM duty command : 5.8us = 30us - 24.2us
		GENERATE_PWM_DUTY_SVPWM_Calc( (p->VoltCmd.StatorVoltCmd.Alpha), (p->VoltCmd.StatorVoltCmd.Beta), (DevideVbus), (&(p->Svpwm)) );

		p->PwmDutyCmd.MaxDuty = 1.0f;
		p->PwmDutyCmd.MinDuty = 0.0f;

		COORDINATE_TRANSFER_Rotor_to_Stator_Calc( (p->CurrentControl.IdCmd), (p->CurrentControl.IqCmd), SinValue, CosValue,  (&(IstatorCmd)) );
		COORDINATE_TRANSFER_Stator_to_Phase_Calc( (IstatorCmd.Alpha), (IstatorCmd.Beta), (&(IphaseCmd)) );
		p->CompDuty.Compensation( &p->CompDuty, IphaseCmd.U, IphaseCmd.V, IphaseCmd.W );
		p->Svpwm.Duty.Duty[PHASE_U]+=p->CompDuty.CompDuty.Duty[PHASE_U];
		p->Svpwm.Duty.Duty[PHASE_V]+=p->CompDuty.CompDuty.Duty[PHASE_V];
		p->Svpwm.Duty.Duty[PHASE_W]+=p->CompDuty.CompDuty.Duty[PHASE_W];

		//Set command
		GENERATE_PWM_DUTY_DUTY_COOMMAND_180DEGREE( (&(p->Svpwm.Duty)), (&(p->PwmDutyCmd)) );
	}
	else if( FunctionMode == FUNCTION_MODE_BOOTSTRAP )
	{
		//CmdFrom = CMD_FROM_NONE;
		//PosFb = POS_FB_NONE;
		//CtrlMode = CTRL_MODE_NONE;
		//PwmMode = PWM_MODE_BOOTSTRAP;
		DUTY_TYPE BootrapDuty=DUTY_DEFAULT;
		p->PwmDutyCmd.MaxDuty = 1.0f;
		p->PwmDutyCmd.MinDuty = 0.0f;
		BootrapDuty.Duty[0]=p->Cmd.BoostrapDuty;
		BootrapDuty.Duty[1]=p->Cmd.BoostrapDuty;
		BootrapDuty.Duty[2]=p->Cmd.BoostrapDuty;
		GENERATE_PWM_DUTY_DUTY_COOMMAND_180DEGREE( &(BootrapDuty), &(p->PwmDutyCmd) );

	}
	else if ( FunctionMode == FUNCTION_MODE_IF_CONTROL )
	{
		p->IfControl.Calc( &(p->IfControl), p->Cmd.IfRpmTarget );
		p->CurrentControl.IdCmd = p->IfControl.CurrAmp;
		p->CurrentControl.IqCmd = 0.0f;

		p->CurrentControl.EleAngle = p->IfControl.Position.VirtualEleAngle;
		p->CurrentControl.EleSpeed = p->IfControl.Position.VirtualEleSpeed;

		p->CurrentControl.StatorCurrFb.Alpha = StatorCurrFbTmp.Alpha;
		p->CurrentControl.StatorCurrFb.Beta = StatorCurrFbTmp.Beta;
		COORDINATE_TRANSFER_GET_SIN_COS( (p->CurrentControl.EleAngle), SinValue, CosValue )
		COORDINATE_TRANSFER_Stator_to_Rotor_Calc(  (p->CurrentControl.StatorCurrFb.Alpha), (p->CurrentControl.StatorCurrFb.Beta), SinValue, CosValue, (&(p->CurrentControl.RotorCurrFb)) );

		p->CurrentControl.IdRegulator.CalcNoLimit( p->CurrentControl.IdCmd, p->CurrentControl.RotorCurrFb.D, &(p->CurrentControl.IdRegulator) );
		p->CurrentControl.IqRegulator.CalcNoLimit( p->CurrentControl.IqCmd, p->CurrentControl.RotorCurrFb.Q, &(p->CurrentControl.IqRegulator) );

		float EleSpeedTmp = 0.0f;
		EleSpeedTmp = p->CurrentControl.EleSpeed;
		p->CurrentControl.Decoupling.PIDWayId.CalcNoLimit( (EleSpeedTmp * p->CurrentControl.IqCmd), (EleSpeedTmp * p->CurrentControl.RotorCurrFb.Q), &(p->CurrentControl.Decoupling.PIDWayId) );
		p->CurrentControl.Decoupling.PIDWayIq.CalcNoLimit( (EleSpeedTmp * p->CurrentControl.IdCmd), (EleSpeedTmp * p->CurrentControl.RotorCurrFb.D), &(p->CurrentControl.Decoupling.PIDWayIq) );
		p->VoltCmd.VdCmd = p->CurrentControl.IdRegulator.Output - p->CurrentControl.Decoupling.PIDWayId.Output;
		p->VoltCmd.VqCmd = p->CurrentControl.IqRegulator.Output + p->CurrentControl.Decoupling.PIDWayIq.Output;
		p->VoltCmd.EleCompAngle = p->CurrentControl.EleAngle + p->CurrentControl.EleSpeed * p->CurrentControl.PwmPeriod * 1.5f;

		p->VoltCmd.VcmdAmp = sqrtf( p->VoltCmd.VdCmd * p->VoltCmd.VdCmd + p->VoltCmd.VqCmd * p->VoltCmd.VqCmd );
		if ( p->VoltCmd.VcmdAmp > VsSaturation)
		{
			float Vgain = 0.0f;
			Vgain = VsSaturation / p->VoltCmd.VcmdAmp;
			p->VoltCmd.VdCmd *= Vgain;
			p->VoltCmd.VqCmd *= Vgain;

			p->CurrentControl.Decoupling.PIDWayId.Ui *= Vgain;
			p->CurrentControl.Decoupling.PIDWayIq.Ui *= Vgain;
			p->CurrentControl.IdRegulator.Ui *= Vgain;
			p->CurrentControl.IqRegulator.Ui *= Vgain;
		}

		COORDINATE_TRANSFER_GET_SIN_COS( (p->VoltCmd.EleCompAngle), SinValue, CosValue )
		COORDINATE_TRANSFER_Rotor_to_Stator_Calc( (p->VoltCmd.VdCmd), (p->VoltCmd.VqCmd), SinValue, CosValue, (&(p->VoltCmd.StatorVoltCmd)) );

		//Calcuclate the PWM duty command : 5.8us = 30us - 24.2us
		GENERATE_PWM_DUTY_SVPWM_Calc( (p->VoltCmd.StatorVoltCmd.Alpha), (p->VoltCmd.StatorVoltCmd.Beta), (DevideVbus), (&(p->Svpwm)) );
		//Calculate the duty limitation : with div 131cycle ~ 0.77us, with multi 125cycle ~ 0.735us
//		p->PwmDutyCmd.MinDuty = ( p->DriverPara.Mosfet.LowerBridgeMinTime + p->DriverPara.Mosfet.DeadTime ) / p->CurrentControl.PwmPeriod;
//		p->PwmDutyCmd.MaxDuty = 1.0f - p->PwmDutyCmd.MinDuty;
//		if ( p->PwmDutyCmd.MinDuty > 1.0f)
//		{
//			p->PwmDutyCmd.MaxDuty = 1.0f;
//			p->PwmDutyCmd.MinDuty = 0.0f;
//		}
		COORDINATE_TRANSFER_Rotor_to_Stator_Calc( (p->CurrentControl.IdCmd), (p->CurrentControl.IqCmd), SinValue, CosValue,  (&(IstatorCmd)) );
		COORDINATE_TRANSFER_Stator_to_Phase_Calc( (IstatorCmd.Alpha), (IstatorCmd.Beta), (&(IphaseCmd)) );
		p->CompDuty.Compensation( &p->CompDuty, IphaseCmd.U, IphaseCmd.V, IphaseCmd.W );
		p->Svpwm.Duty.Duty[PHASE_U]+=p->CompDuty.CompDuty.Duty[PHASE_U];
		p->Svpwm.Duty.Duty[PHASE_V]+=p->CompDuty.CompDuty.Duty[PHASE_V];
		p->Svpwm.Duty.Duty[PHASE_W]+=p->CompDuty.CompDuty.Duty[PHASE_W];


		//Set command
		GENERATE_PWM_DUTY_DUTY_COOMMAND_180DEGREE( (&(p->Svpwm.Duty)), (&(p->PwmDutyCmd)) );
	}
	else if ( FunctionMode == FUNCTION_MODE_VF_CONTROL )
	{
		p->VfControl.Calc( &(p->VfControl), p->Cmd.VfRpmTarget );

		p->CurrentControl.EleAngle = p->VfControl.Position.VirtualEleAngle;
		p->CurrentControl.EleSpeed = p->VfControl.Position.VirtualEleSpeed;

		p->VoltCmd.VdCmd = 0.0f;
		p->VoltCmd.VqCmd = p->VfControl.VoltAmp;
		p->VoltCmd.EleCompAngle = p->CurrentControl.EleAngle + p->CurrentControl.EleSpeed * p->CurrentControl.PwmPeriod * 1.5f;
		p->VoltCmd.VcmdAmp = sqrtf( p->VoltCmd.VdCmd * p->VoltCmd.VdCmd + p->VoltCmd.VqCmd * p->VoltCmd.VqCmd );
		if ( p->VoltCmd.VcmdAmp > VsSaturation)
		{
			float Vgain = 0.0f;
			Vgain = VsSaturation / p->VoltCmd.VcmdAmp;
			p->VoltCmd.VdCmd *= Vgain;
			p->VoltCmd.VqCmd *= Vgain;

			p->CurrentControl.Decoupling.PIDWayId.Ui *= Vgain;
			p->CurrentControl.Decoupling.PIDWayIq.Ui *= Vgain;
			p->CurrentControl.IdRegulator.Ui *= Vgain;
			p->CurrentControl.IqRegulator.Ui *= Vgain;
		}

		COORDINATE_TRANSFER_GET_SIN_COS( (p->VoltCmd.EleCompAngle), SinValue, CosValue )
		COORDINATE_TRANSFER_Rotor_to_Stator_Calc( (p->VoltCmd.VdCmd), (p->VoltCmd.VqCmd), SinValue, CosValue, (&(p->VoltCmd.StatorVoltCmd)) );

		GENERATE_PWM_DUTY_SVPWM_Calc( (p->VoltCmd.StatorVoltCmd.Alpha), (p->VoltCmd.StatorVoltCmd.Beta), (DevideVbus), (&(p->Svpwm)) );

//		p->PwmDutyCmd.MinDuty = ( p->DriverPara.Mosfet.LowerBridgeMinTime + p->DriverPara.Mosfet.DeadTime ) / p->CurrentControl.PwmPeriod;
//		p->PwmDutyCmd.MaxDuty = 1.0f - p->PwmDutyCmd.MinDuty;
//		if ( p->PwmDutyCmd.MinDuty > 1.0f)
//		{
//			p->PwmDutyCmd.MaxDuty = 1.0f;
//			p->PwmDutyCmd.MinDuty = 0.0f;
//		}
		COORDINATE_TRANSFER_Rotor_to_Stator_Calc( (p->CurrentControl.IdCmd), (p->CurrentControl.IqCmd), SinValue, CosValue,  (&(IstatorCmd)) );
		COORDINATE_TRANSFER_Stator_to_Phase_Calc( (IstatorCmd.Alpha), (IstatorCmd.Beta), (&(IphaseCmd)) );
		p->CompDuty.Compensation( &p->CompDuty, IphaseCmd.U, IphaseCmd.V, IphaseCmd.W );
		p->Svpwm.Duty.Duty[PHASE_U]+=p->CompDuty.CompDuty.Duty[PHASE_U];
		p->Svpwm.Duty.Duty[PHASE_V]+=p->CompDuty.CompDuty.Duty[PHASE_V];
		p->Svpwm.Duty.Duty[PHASE_W]+=p->CompDuty.CompDuty.Duty[PHASE_W];

		//Set command
		GENERATE_PWM_DUTY_DUTY_COOMMAND_180DEGREE( (&(p->Svpwm.Duty)), (&(p->PwmDutyCmd)) );
	}
	else if ( FunctionMode == FUNCTION_MODE_SIX_WAVE_120_CLOSE_LOOP )
	{
		// CmdFrom = CMD_FROM_SIX_WAVE_120_CLOSE_LOOP;
		p->SixWave120CurrentControl.CurrCmd = p->Cmd.SixWaveCurrCmd;

		// PosFb = POS_FB_HALL;
		if ( p->HallSelect == SIX_WAVE_HALL_TYPE_TEST)
		{
			p->TestHall.CalHallAngle(&(p->TestHall),p->SensorFb.TestHallSignal);
		}

		// CtrlMode = CTRL_MODE_SIX_WAVE_120_CLOSE_LOOP;
		p->SixWave120CurrentControl.GetCurrFb(&(p->SixWave120CurrentControl),(p->SixWave120Duty.PwmStatus),p->SensorFb.Iu,p->SensorFb.Iv,p->SensorFb.Iw);
		p->SixWave120CurrentControl.Regulator.UpperLimit = 1.732050807f * VbusLimit; // = VbusAvailable;
		p->SixWave120CurrentControl.Regulator.Calc( p->SixWave120CurrentControl.CurrCmd, p->SixWave120CurrentControl.CurrFb, &(p->SixWave120CurrentControl.Regulator));
		p->SixWave120Duty.VoltCmd = p->SixWave120CurrentControl.Regulator.Output;

		// PwmMode = PWM_MODE_SIX_WAVE_120;
		if ( p->HallSelect == SIX_WAVE_HALL_TYPE_TEST )
		{
			p->SixWave120Duty.HallSection = p->TestHall.HallSection;
		}
		p->SixWave120Duty.TorquePolarityFlag = ( p->SixWave120CurrentControl.CurrCmd >= 0.0f ) ? SIX_WAVE_TORQUE_POLARITY_POSITIVE : SIX_WAVE_TORQUE_POLARITY_NEGATIVE;
		p->SixWave120Duty.Calc(&(p->SixWave120Duty),DevideVbus);
		p->PwmDutyCmd.MinDuty = p->DriverPara.Mosfet.DeadTime / p->CurrentControl.PwmPeriod;
		p->PwmDutyCmd.MaxDuty = 1.0f - ( p->DriverPara.Mosfet.LowerBridgeMinTime + p->DriverPara.Mosfet.DeadTime ) / p->CurrentControl.PwmPeriod;
		p->SixWave120Duty.DutyCmd = p->PwmDutyCmd.MinDuty + p->SixWave120Duty.DutyCmd;
		p->PwmDutyCmd.Set120( p->SixWave120Duty.DutyCmd, &(p->PwmDutyCmd), p->SixWave120Duty.PwmStatus);

	}
	else
	{
    //PWM test mode
	}

}

void MotorControl_CleanParameter( MOTOR_CONTROL_TYPE *p )
{
//	p->CurrentControl.IdCmd = 0.0f;
//	p->CurrentControl.IqCmd = 0.0f;

	p->CurrentControl.StatorCurrFb.Clean( &(p->CurrentControl.StatorCurrFb) );
//	p->CurrentControl.RotorCurrFb.Clean( &(p->CurrentControl.RotorCurrFb) );
	p->VoltCmd.StatorVoltCmd.Clean( &(p->VoltCmd.StatorVoltCmd) );

	p->CurrentControl.IdRegulator.Clean( &(p->CurrentControl.IdRegulator) );
	p->CurrentControl.IqRegulator.Clean( &(p->CurrentControl.IqRegulator) );

	p->VoltCmd.VcmdAmp = 0.0f;
	p->PwmDutyCmd.Clean( &(p->PwmDutyCmd) );

	//20191212 Flux Weakening
	p->CurrentControl.FluxWeakening.Clean( &(p->CurrentControl.FluxWeakening) );

	//20191216 Command Type
	p->Cmd.IdCmd = 0.0f;
	p->Cmd.IqCmd = 0.0f;
	p->Cmd.IfRpmTarget = 0.0f;
	p->Cmd.VfRpmTarget = 0.0f;
	p->Cmd.BoostrapDuty = 0.0f;

	//20191216 IF/VF Type
	p->IfControl.Clean( &(p->IfControl) );
	p->VfControl.Clean( &(p->VfControl) );

	//20200226 Torque To Current related paramters
	p->Cmd.TorqueCmd = 0.0f;
	p->SensorFb.AllowFluxRec = 0.0f;

	//20200702 Six Wave Clean;
	p->SixWave120CurrentControl.Clean(&(p->SixWave120CurrentControl));
	p->Cmd.SixWaveVoltCmd = 0.0f;
	p->Cmd.SixWaveCurrCmd = 0.0f;

	//ComplexVector
	p->CurrentControl.Decoupling.PIDWayId.Clean( &(p->CurrentControl.Decoupling.PIDWayId) );
	p->CurrentControl.Decoupling.PIDWayIq.Clean( &(p->CurrentControl.Decoupling.PIDWayIq) );

	//Sensorless
//	p->Sensorless.Clean(&(p->Sensorless),0.0f,0.0f);

//	p->CurrentControl.EleAngle = 0.0f;
}


uint16_t MotorControl_InitParameter( MOTOR_CONTROL_TYPE *p, MOTOR_CONTROL_PARAMETER_DEFAULT_TYPE *pSetting)
{
	float IdHz = 0.0f;
	float IqHz = 0.0f;

	//Initialize the parameters of current control from setting
	p->MotorPara.PM.Flux = pSetting->PmMotorFlux;
	p->MotorPara.PM.Ld = pSetting->PmMotorLd;
	p->MotorPara.PM.Lq = pSetting->PmMotorLq;
	p->MotorPara.PM.Res = pSetting->PmMotorRes;
	p->MotorPara.PM.Polepair = pSetting->PmMotorPolepair;
	p->MotorPara.PM.J = pSetting->PmMotorJ;

	p->DriverPara.Mosfet.DeadTime = pSetting->MosfetDriverDeadTime;
	p->DriverPara.Mosfet.LowerBridgeMinTime = pSetting->MosfetDriverLowerBridgeMinTime;
	//p->DriverPara.Mosfet.PwmPeriodCnt = pSetting->MosfetDriverPwmPeriodCnt;
	//p->DriverPara.Mosfet.ShuntRisingTime = pSetting->MosfetDriverShuntRisingTime;
	p->DriverPara.Mosfet.MinTimeEleSpeedAbs = pSetting->MosfetDriverMinTimeChangeEleSpeedAbs;

	p->CurrentControl.PwmPeriod = pSetting->PwmPeriod; // TODO some are PWM period, some are current period
	p->CurrentControl.PwmHz = 1.0f / p->CurrentControl.PwmPeriod;
	IdHz = pSetting->IdHz;
	IqHz = pSetting->IqHz;

	//Initialize the PWM module of current control
	//p->DriverPara.Mosfet.DeadTimeDuty = p->DriverPara.Mosfet.DeadTime / p->CurrentControl.PwmPeriod;
	//p->DriverPara.Mosfet.DeadTimeCnt = p->DriverPara.Mosfet.DeadTimeDuty * p->DriverPara.Mosfet.PwmPeriodCnt;
	//p->DriverPara.Mosfet.ShuntRisingCnt = ( 1.0f - p->DriverPara.Mosfet.ShuntRisingTime / p->CurrentControl.PwmPeriod ) * pSetting->MosfetDriverPwmPeriodCnt;
	p->ControlInitStatus = p->PwmDutyCmd.Init( p->CurrentControl.PwmPeriod, p->DriverPara.Mosfet.DeadTime, p->DriverPara.Mosfet.LowerBridgeMinTime, &(p->PwmDutyCmd) );
	if (( p->ControlInitStatus & 0x8000 ) !=0 ) return CONTROL_INIT_STATUS_NUMBER_PWM_DUTY;

	//Initialize the PWM module of current control
	MotorControl_CurrentControlSetRegulatorParameter( &(p->CurrentControl), &(p->MotorPara), p->CurrentControl.PwmPeriod, IdHz, IqHz );
	p->ControlInitStatus = p->CurrentControl.IdRegulator.Init( p->CurrentControl.PwmPeriod, p->CurrentControl.IdRegulator.Kp, p->CurrentControl.IdRegulator.Ki, 1.0f, -1.0f, &(p->CurrentControl.IdRegulator) );
	if (( p->ControlInitStatus & 0x8000 ) !=0 ) return CONTROL_INIT_STATUS_NUMBER_ID_REGULATOR;
	p->ControlInitStatus = p->CurrentControl.IqRegulator.Init( p->CurrentControl.PwmPeriod, p->CurrentControl.IqRegulator.Kp, p->CurrentControl.IqRegulator.Ki, 1.0f, -1.0f, &(p->CurrentControl.IqRegulator) );
	if (( p->ControlInitStatus & 0x8000 ) !=0 ) return CONTROL_INIT_STATUS_NUMBER_IQ_REGULATOR;

	//Clean the parameters of current control (only once)
	p->CurrentControl.EleAngle = 0.0f;
	p->CurrentControl.EleSpeed = 0.0f;
	p->VoltCmd.EleCompAngle = 0.0f;

	//20191212 Flux Weakening
	MotorControl_FluxWeakeningSetRegulatorParameter( &(p->CurrentControl.FluxWeakening), &(p->MotorPara), pSetting->FWHz, p->CurrentControl.PwmPeriod, pSetting->Ilimit, p->CurrentControl.IdRegulator.Kp, p->CurrentControl.IqRegulator.Kp );
	p->ControlInitStatus = p->CurrentControl.FluxWeakening.Init( &(p->CurrentControl.FluxWeakening), pSetting->Ilimit, p->CurrentControl.FluxWeakening.FluxWeakeningRegulator.LowerLimit, p->CurrentControl.PwmPeriod, p->CurrentControl.FluxWeakening.FluxWeakeningRegulator.Kp , p->CurrentControl.FluxWeakening.FluxWeakeningRegulator.Ki);
	if (( p->ControlInitStatus & 0x8000 ) !=0 ) return CONTROL_INIT_STATUS_NUMBER_FLUX_WEAKENING_REGULATOR;

	//20191216 IF/VF control
	p->ControlInitStatus = p->IfControl.Init(&(p->IfControl),p->CurrentControl.PwmPeriod,pSetting->IfGain,pSetting->IfAecel,pSetting->IfDecel,p->MotorPara.PM.Polepair);
	if (( p->ControlInitStatus & 0x8000 ) !=0 ) return CONTROL_INIT_STATUS_NUMBER_IF_CONTROL;
	p->ControlInitStatus = p->VfControl.Init(&(p->VfControl),p->CurrentControl.PwmPeriod,pSetting->VfGain,pSetting->VfAecel,pSetting->VfDecel,p->MotorPara.PM.Polepair);
	if (( p->ControlInitStatus & 0x8000 ) !=0 ) return CONTROL_INIT_STATUS_NUMBER_VF_CONTROL;

	//clean the parameter of motor control (in servo off status)
	p->Clean(p);

	// Idq CMD Init and TorqueToIdq Init
	p->ControlInitStatus = p->TorqueToIdq.IdCmdLut.Init(&(p->TorqueToIdq.IdCmdLut),pSetting->pMotorTableHeader->IdCmdHeader.Para);
	if (( p->ControlInitStatus & 0x8000 ) !=0 ) return CONTROL_INIT_STATUS_NUMBER_ID_CMD_TABLE;
	p->ControlInitStatus = p->TorqueToIdq.IqCmdLut.Init(&(p->TorqueToIdq.IqCmdLut),pSetting->pMotorTableHeader->IqCmdHeader.Para);
	if (( p->ControlInitStatus & 0x8000 ) !=0 ) return CONTROL_INIT_STATUS_NUMBER_IQ_CMD_TABLE;
	p->ControlInitStatus = p->TorqueToIdq.MaxTorqueLut.Init(&(p->TorqueToIdq.MaxTorqueLut),pSetting->pMotorTableHeader->MaxTorqueHeader.Para);
	if (( p->ControlInitStatus & 0x8000 ) !=0 ) return CONTROL_INIT_STATUS_NUMBER_MAX_TORQUE_TABLE;
	FILTER_INIT_BILINEAR_1ORDER_TYPE FilterSettingTmp = FILTER_INIT_BILINEAR_1ORDER_DEFAULT;
	FilterSettingTmp.BandwithHz = 100.0f;
	FilterSettingTmp.Period = 0.001f;
	FilterSettingTmp.Type = FILTER_TYPE_LPF;
	Filter_Bilinear1OrderInit( &(p->TorqueToIdq.EleSpeedFilter), &FilterSettingTmp );
//	p->TorqueToIdq.VbusGainForIcmd = 1 - 2 * (p->DriverPara.Mosfet.DeadTime * 2 + p->DriverPara.Mosfet.LowerBridgeMinTime) / p->CurrentControl.PwmPeriod;
//	p->TorqueToIdq.VbusGainForFW = 1 - 2 * (p->DriverPara.Mosfet.DeadTime * 2 + p->DriverPara.Mosfet.LowerBridgeMinTime) / p->CurrentControl.PwmPeriod;
//	p->TorqueToIdq.VbusGainForVsaturation = 1 - (p->DriverPara.Mosfet.DeadTime * 2 + p->DriverPara.Mosfet.LowerBridgeMinTime) / p->CurrentControl.PwmPeriod;
	//20200702 Six Wave Init;
	p->SensorFb.HallSignal = ( p->SensorFb.HallSignal == 0) ? 5 : p->SensorFb.HallSignal;
	p->SensorFb.TestHallSignal = 5;
	p->TestHall.HallType = 	SIX_WAVE_HALL_TYPE_TEST;
	MotorControl_SixWaveRegulatorParameter( &(p->SixWave120CurrentControl), &(p->MotorPara), p->CurrentControl.PwmPeriod, 250.0f );
	p->ControlInitStatus = p->SixWave120CurrentControl.Init(&(p->SixWave120CurrentControl),p->SixWave120CurrentControl.Regulator.Kp,p->SixWave120CurrentControl.Regulator.Ki,p->CurrentControl.PwmPeriod);
	if (( p->ControlInitStatus & 0x8000 ) !=0 ) return CONTROL_INIT_STATUS_NUMBER_SIX_WAVE;

	//Decoupling
	IdHz = pSetting->IdDCPHz;
	IqHz = pSetting->IqDCPHz;
	MotorControl_CalcPMFOCDCPPID( &(p->CurrentControl.Decoupling.PIDWayId.Ki), IdHz, p->CurrentControl.PwmPeriod, p->MotorPara.PM.Lq, p->MotorPara.PM.Res);
	MotorControl_CalcPMFOCDCPPID( &(p->CurrentControl.Decoupling.PIDWayIq.Ki), IqHz, p->CurrentControl.PwmPeriod, p->MotorPara.PM.Ld, p->MotorPara.PM.Res);
	p->CurrentControl.Decoupling.PIDWayId.Init( p->CurrentControl.PwmPeriod, 0.0f, p->CurrentControl.Decoupling.PIDWayId.Ki, 1.0f, -1.0f, &(p->CurrentControl.Decoupling.PIDWayId) );
	p->CurrentControl.Decoupling.PIDWayIq.Init( p->CurrentControl.PwmPeriod, 0.0f, p->CurrentControl.Decoupling.PIDWayId.Ki, 1.0f, -1.0f, &(p->CurrentControl.Decoupling.PIDWayIq) );

	//Compensation Deadtime
	p->CompDuty.Init(&(p->CompDuty),pSetting->DeadtimeDutyP,pSetting->DeadtimeSlopeP,pSetting->DeadtimeDutyN,pSetting->DeadtimeSlopeN);

	//Vbus filter
	FILTER_INIT_BILINEAR_1ORDER_TYPE VbusFilterSettingTmp = FILTER_INIT_BILINEAR_1ORDER_DEFAULT;
	VbusFilterSettingTmp.BandwithHz = pSetting->VbusHz;
	VbusFilterSettingTmp.Period = p->CurrentControl.PwmPeriod;
	VbusFilterSettingTmp.Type = FILTER_TYPE_LPF;
	Filter_Bilinear1OrderInit( &(p->SensorFb.VbusFilter), &VbusFilterSettingTmp );

	p->StartUpWay = FUNCTION_MODE_NORMAL_CURRENT_CONTROL;

	return CONTROL_INIT_OK;
}

void MotorControl_CalcPMFOCPID( float *Kp, float *Ki, float Hz, float Period, float L, float Res)
{
	float P1 = Hz * _2PI;
	float P2 = Res/L;
	float Z1 = expf(-P1*Period);
	float Z2 = expf(-P2*Period);
	(*Kp) = -(Res*expf(-Period*Res/L) - Res*Z1*Z2)/(expf(-Period*Res/L) - 1);
	(*Ki) = ((Z1 - 1)*(Res - Res*Z2))/(Period*(expf(-Period*Res/L) - 1));
}

void MotorControl_CalcPMFOCDCPPID( float *Ki, float Hz, float Period, float L, float Res)
{
	float P1 = Hz * _2PI;
	float P2 = Res/L;
	float Z1 = expf(-P1*Period);
	float Z2 = expf(-P2*Period);
	(*Ki) = -(Res*expf(-Period*Res/L) - Res*Z1*Z2)/(expf(-Period*Res/L) - 1);
}

void MotorControl_CurrentControlSetRegulatorParameter( CURRENT_CONTROL_TYPE *pSet, MOTOR_PARAMETER_TYPE* pMotor, float PwmPeriod , float IdHz, float IqHz)
{
	MotorControl_CalcPMFOCPID( &(pSet->IdRegulator.Kp), &(pSet->IdRegulator.Ki), IdHz, PwmPeriod, pMotor->PM.Ld, pMotor->PM.Res);
	MotorControl_CalcPMFOCPID( &(pSet->IqRegulator.Kp), &(pSet->IqRegulator.Ki), IqHz, PwmPeriod, pMotor->PM.Lq, pMotor->PM.Res);
}

void MotorControl_FluxWeakeningSetRegulatorParameter( CURRENT_CONTROL_FLUX_WEAKENING_TYPE *pSet, MOTOR_PARAMETER_TYPE* pMotor, float Bandwith, float PwmPeriod, float Ilimit, float IdKp, float IqKp )
{
	float PIDGain = sqrtf( IdKp * IdKp + IqKp * IqKp );
	float TimeConstant = 1.0f / ( Bandwith * _2PI );
	pSet->FluxWeakeningRegulator.Kp = 0.0f;
	pSet->FluxWeakeningRegulator.Ki = 1.0f / TimeConstant * PIDGain / PwmPeriod;
	pSet->FluxWeakeningRegulator.LowerLimit = -Ilimit;
}

void MotorControl_SixWaveRegulatorParameter( SIX_WAVE_120_CURRENT_CONTROL_TYPE *pSet, MOTOR_PARAMETER_TYPE* pMotor, float PwmPeriod , float Hz)
{
	pSet->Regulator.Kp = pMotor->PM.Ld * Hz * _2PI;
	pSet->Regulator.Ki = pMotor->PM.Res * Hz * _2PI;
}



