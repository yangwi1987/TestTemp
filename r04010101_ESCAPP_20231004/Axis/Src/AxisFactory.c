/*
* AxisFactory.c
*
*  Created on: 2019年12月11日
*      Author: MikeSFWen
*/

#include "ParamMgr.h"
#include "UiApp.h"
#include "AxisFactory.h"


static uint8_t CurrToPLCCnt = 0;
extern uint16_t IsUseDigitalFoilSensor;

void AxisSync_SyncFourQuadParams( Axis_t *v )
{
    v->FourQuadCtrl.Init( &v->FourQuadCtrl );
}

void AxisFactory_OnParamValueChanged( Axis_t *v, uint16_t ParamNumber )
{
    switch( ParamNumber )
    {
    case PN_SC_GEAR_NUM:
    case PN_SC_GEAR_DEN:
    case PN_SC_GEARBOX_DIR:
    case PN_SC_GEARBOX_EFF:
    case PN_SC_TIRE_RADIUS:
        AxisSync_SyncFourQuadParams(v);
        break;

    case PN_DEBUG_PARAM_1:
    case PN_DEBUG_PARAM_2:
    case PN_DEBUG_PARAM_3:
    case PN_DEBUG_PARAM_4:
    case PN_DEBUG_PARAM_5:
    case PN_DEBUG_PARAM_6:
    case PN_DEBUG_PARAM_7:
    case PN_DEBUG_PARAM_8:
    case PN_DEBUG_PARAM_9:
    case PN_DEBUG_PARAM_10:
        break;
    case PN_POLE_PAIR:
    case PN_LD:
    case PN_LQ:
    case PN_RS:
    case PN_LAMBDA_M:
    case PN_DEADTIME:
        break;

    case PN_HIGH_PWM_SPEED:
    case PN_LOW_PWM_SPEED:
    case PN_PWM_FREQ_TIMES_N:
    case PN_CURRENT_FREQ:
        break;
    }
}

void AxisFactory_UpdateCANRxInterface( Axis_t *v )
{
    v->pCANRxInterface->BatCurrentDrainLimit = DEFAULT_DC_LIMIT;

    v->ThrotMapping.TnSelect = v->pCANRxInterface->OutputModeCmd;
    if( ( v->pCANRxInterface->ReceivedCANID & RECEIVED_BAT_ID_1 ) == RECEIVED_BAT_ID_1)
    {
        v->AlarmDetect.CAN1Timeout.Counter = 0;
    }
    v->pCANRxInterface->ReceivedCANID = 0;
}

void AxisFactory_UpdateCANTxInterface( Axis_t *v )
{
    uint16_t i;

	if(v->HasCriAlarm == 1)
	{
		v->pCANTxInterface->DebugU8[TX_INTERFACE_DBG_IDX_ERROR_FLAG] |= CAN_TX_CRI_ALARM_MASK;
	}
	else
	{
		v->pCANTxInterface->DebugU8[TX_INTERFACE_DBG_IDX_ERROR_FLAG] &= ~CAN_TX_CRI_ALARM_MASK;
	}

	if(v->HasNonCriAlarm == 1)
	{
		v->pCANTxInterface->DebugU8[TX_INTERFACE_DBG_IDX_ERROR_FLAG] |= CAN_TX_NON_CRI_ALARM_MASK;
	}
	else
	{
		v->pCANTxInterface->DebugU8[TX_INTERFACE_DBG_IDX_ERROR_FLAG] &= ~CAN_TX_NON_CRI_ALARM_MASK;
	}

	if(v->HasWarning == 1)
	{
		v->pCANTxInterface->DebugU8[TX_INTERFACE_DBG_IDX_ERROR_FLAG] |= CAN_TX_WARNING_MASK;
	}
	else
	{
		v->pCANTxInterface->DebugU8[TX_INTERFACE_DBG_IDX_ERROR_FLAG] &= ~CAN_TX_WARNING_MASK;
	}

    v->pCANTxInterface->DeratingSrc = v->ThermoStrategy.ThermoDeratingSrc;

    // debug

    if(v->pCANTxInterface->DebugU8[TX_INTERFACE_DBG_IDX_LOG_SAMPLE_FLAG] == 1)
    {
        v->pCANTxInterface->Debugf[IDX_MOTOR0_TEMP] = v->pAdcStation->AdcTraOut.MOTOR_NTC_0;		//Motor0
        v->pCANTxInterface->Debugf[IDX_MOS1_TEMP] = v->pAdcStation->AdcTraOut.PCU_NTC[MOS_NTC_1]; //MOS1
        v->pCANTxInterface->Debugf[IDX_MOS2_TEMP] = v->pAdcStation->AdcTraOut.PCU_NTC[MOS_NTC_2]; //MOS2
        v->pCANTxInterface->Debugf[IDX_CAP_TEMP] = v->pAdcStation->AdcTraOut.PCU_NTC[CAP_NTC];	//CAP
        v->pCANTxInterface->Debugf[IDX_MOTOR1_TEMP] = v->pAdcStation->AdcTraOut.MOTOR_NTC_1;	//Motor1
        v->pCANTxInterface->Debugf[IDX_MOTOR2_TEMP] = v->pAdcStation->AdcTraOut.MOTOR_NTC_2;	//Motor2

        for(i = 0; i < 10; i++){
            v->pCANTxInterface->DebugError[i] = v->pAlarmStack->NowAlarmID[i];
        }

        v->pCANTxInterface->Debugf[IDX_ID_CMD] = v->MotorControl.CurrentControl.IdCmd;
        v->pCANTxInterface->Debugf[IDX_IQ_CMD] = v->MotorControl.CurrentControl.IqCmd;
        v->pCANTxInterface->Debugf[IDX_ID_FBK] = v->MotorControl.CurrentControl.RotorCurrFb.D;
        v->pCANTxInterface->Debugf[IDX_IQ_FBK] = v->MotorControl.CurrentControl.RotorCurrFb.Q;
        v->pCANTxInterface->Debugf[IDX_AC_LIMIT_CMD] = v->TorqCommandGenerator.AcCurrLimit;
        v->pCANTxInterface->Debugf[IDX_AC_LIMIT_TQ] = v->TorqCommandGenerator.ACLimitedTorqueCommand;
        v->pCANTxInterface->Debugf[IDX_DC_LIMIT_CMD] = v->TorqCommandGenerator.DcCurrLimit;
        v->pCANTxInterface->Debugf[IDX_DC_LIMIT_TQ] = v->TorqCommandGenerator.DCLimitedTorqueCommand;
        v->pCANTxInterface->Debugf[IDX_PERFROMANCE_TQ] = v->TorqCommandGenerator.Out;
        v->pCANTxInterface->Debugf[IDX_VD_CMD] = v->MotorControl.VoltCmd.VdCmd;
        v->pCANTxInterface->Debugf[IDX_VQ_CMD] = v->MotorControl.VoltCmd.VqCmd;
        v->pCANTxInterface->Debugf[IDX_MOTOR_RPM] = v->SpeedInfo.MotorMechSpeedRPM;
        v->pCANTxInterface->Debugf[IDX_DC_VOLT] = v->pAdcStation->AdcTraOut.BatVdc;
        v->pCANTxInterface->Debugf[IDX_THROTTLE_RAW] = v->pAdcStation->ThrotADCRawRatio;
        v->pCANTxInterface->Debugf[IDX_THROTTLE_FINAL] = v->ThrotMapping.PercentageTarget;
        v->pCANTxInterface->Debugf[IDX_ACC_PEDAL1_VOLT] = v->pAdcStation->AdcTraOut.Pedal_V1;
        v->pCANTxInterface->Debugf[IDX_EA5V] = v->pAdcStation->AdcTraOut.EA5V;
        v->pCANTxInterface->Debugf[IDX_INSTANT_AC_POWER]= AcPwrInfo.InstPower;
        v->pCANTxInterface->Debugf[IDX_AVERAGE_AC_POWER]= AcPwrInfo.AvgPower;
        v->pCANTxInterface->Debugf[IDX_E5V] = v->pAdcStation->AdcTraOut.E5V;
        v->pCANTxInterface->Debugf[IDX_ES5V] = v->pAdcStation->AdcTraOut.ES5V;
        v->pCANTxInterface->Debugf[IDX_IU_FBK] = v->pAdcStation->AdcTraOut.Iu[0];
        v->pCANTxInterface->Debugf[IDX_IV_FBK] = v->pAdcStation->AdcTraOut.Iv[0];
        v->pCANTxInterface->Debugf[IDX_IW_FBK] = v->pAdcStation->AdcTraOut.Iw[0];
        v->pCANTxInterface->Debugf[IDX_PREC] = v->pAdcStation->AdcTraOut.PreC;
        v->pCANTxInterface->Debugf[IDX_ACC_PEDAL2_VOLT] = v->pAdcStation->AdcTraOut.Pedal_V2;
        v->pCANTxInterface->Debugf[IDX_S13V8] = v->pAdcStation->AdcTraOut.S13V8;
        v->pCANTxInterface->HWID[0] = v->pAdcStation->AdcDmaData[v->pAdcStation->RegCh[HW_ID1].AdcGroupIndex][v->pAdcStation->RegCh[HW_ID1].AdcRankIndex];
        v->pCANTxInterface->HWID[1] = v->pAdcStation->AdcDmaData[v->pAdcStation->RegCh[HW_ID2].AdcGroupIndex][v->pAdcStation->RegCh[HW_ID2].AdcRankIndex];
        v->pCANTxInterface->ServoOnOffState = v->ServoOnOffState;
//        v->pCANTxInterface->Debugf[IDX_DC_LIMIT_CANRX_DC_CURR] = v->pCANRxInterface->BatCurrentDrainLimit;
#if USE_MOTOR_CTRL_DEBUG
        v->pCANTxInterface->Debugf[IDX_ID_CMD_ORI] = v->MotorControl.Cmd.IdCmd;
        v->pCANTxInterface->Debugf[IDX_IQ_CMD_ORI] = v->MotorControl.Cmd.IqCmd;
        v->pCANTxInterface->Debugf[IDX_VD_ORI] = v->MotorControl.CurrentControl.IdRegulator.Output;
        v->pCANTxInterface->Debugf[IDX_VQ_ORI] = v->MotorControl.CurrentControl.IqRegulator.Output;
        v->pCANTxInterface->Debugf[IDX_DCP_D] = v->MotorControl.CurrentControl.Decoupling.PIDWayId.Output;
        v->pCANTxInterface->Debugf[IDX_DCP_Q] = v->MotorControl.CurrentControl.Decoupling.PIDWayIq.Output;
        v->pCANTxInterface->Debugf[IDX_VS] = v->MotorControl.VoltCmd.VcmdAmp;
        v->pCANTxInterface->Debugf[IDX_ID_ERR] = v->MotorControl.CurrentControl.IdRegulator.Error;
        v->pCANTxInterface->Debugf[IDX_IQ_ERR] = v->MotorControl.CurrentControl.IqRegulator.Error;
        v->pCANTxInterface->Debugf[IDX_DCP_D_ERR] = v->MotorControl.CurrentControl.Decoupling.PIDWayId.Error;
        v->pCANTxInterface->Debugf[IDX_DCP_Q_ERR] = v->MotorControl.CurrentControl.Decoupling.PIDWayIq.Error;
#endif
    }

}


static void AxisFactory_ConfigAlarmSystemInPLCLoop( Axis_t *v )
{

}

// This function execute in current loop.
void AxisFactory_RunMotorStateMachine( Axis_t *v )
{
    // invalid condition
    if( v->VCUServoOnCommand == 1 && v->CtrlUiEnable == 1 )
    {
        v->ServoOn = 0;
        return;
    }

    int ServoOnEnable = (v->HasCriAlarm == 0) &&
    					((v->VCUServoOnCommand == 1) || (v->CtrlUiEnable == 1)) &&
						(v->pParamMgr->ECUSoftResetEnable == 0);

    switch( v->ServoOnOffState )
    {
        case MOTOR_STATE_OFF:
            v->MotorCtrlMode = FUNCTION_MODE_BOOTSTRAP;
            v->BootstrapCounter = 0;
            if( ServoOnEnable && v->pAdcStation->ZeroCalibInjDone )
            {
				v->ServoOnOffState = MOTOR_STATE_WAIT_BOOT;
            }
            else
            {
                v->pPwmStation->AxisChannelLock( v->pPwmStation, v->AxisID - 1 );
            }
            break;

        case MOTOR_STATE_WAIT_BOOT:
            if( ServoOnEnable )
            {
                if( v->BootstrapCounter >= v->BootstrapMaxCounter )
                {
                	v->ServoOnOffState = MOTOR_STATE_ON;

                    if( v->PhaseLoss.Enable == FUNCTION_ENABLE )
                    {
                        v->PhaseLoss.Start = FUNCTION_YES;
                    }
                    else
                    {
                        v->PhaseLoss.Start = FUNCTION_NO;
                    }
                }
                else
                {
                    if( v->BootstrapCounter == 1 )
                    {
                        v->pPwmStation->AxisChannelUnlock( v->pPwmStation, v->AxisID - 1 );
                    }
                    v->BootstrapCounter++;
                }
            }
            else
            {
                v->ServoOnOffState = MOTOR_STATE_SHUTDOWN_START;
                v->pPwmStation->AxisChannelLock(v->pPwmStation, v->AxisID - 1);
            }
            break;

        case MOTOR_STATE_ON:
            if( v->PhaseLoss.Enable == FUNCTION_ENABLE )
            {
                v->PhaseLoss.Detection( &(v->PhaseLoss), &(v->MotorControl), &(v->MotorCtrlMode));
            }
            else
            {
                v->MotorCtrlMode = CtrlUi.MotorCtrlMode;
            }

            if( !ServoOnEnable )
            {
                v->ServoOnOffState = MOTOR_STATE_SHUTDOWN_START;
            }
            break;

        case MOTOR_STATE_SHUTDOWN_START:
            v->ServoOnOffState = MOTOR_STATE_OFF;
            break;
    }

    if( (v->ServoOnOffState == MOTOR_STATE_OFF) || (v->ServoOnOffState == MOTOR_STATE_SHUTDOWN_START) )
    {
        v->ServoOn = 0;
    }
    else
    {
        v->ServoOn = 1;
    }
}

void AxisFactory_CleanParameter( void )
{
    CtrlUi.MotorCtrlMode = FUNCTION_MODE_NORMAL_CURRENT_CONTROL;
    DriveFnRegs[ FN_TORQ_COMMAND - FN_BASE ] = 32768;
    DriveFnRegs[ FN_OPEN_SPD_COMMAND - FN_BASE ] = 0;
    DriveFnRegs[ FN_OPEN_SPD_V_I_LIMIT - FN_BASE ] = 0;
    DriveFnRegs[ FN_OPEN_POSITION_CMD - FN_BASE ] = 0;
    DriveFnRegs[ FN_CURRENT_ID_CMD - FN_BASE ] = 32768;
    DriveFnRegs[ FN_CURRENT_IQ_CMD - FN_BASE ] = 32768;
    DriveFnRegs[ FN_CURRENT_IS_CMD - FN_BASE ] = 0;
    DriveFnRegs[ FN_CURRENT_THETA_CMD - FN_BASE ] = 0;

}

void AxisFactory_GetSetting( Axis_t *v )
{
    if ( v->ServoOn == MOTOR_STATE_OFF ) //Allow that PCU change the MF mode setting value in servo off status.
    {
        CtrlUi.MfFunMode = DriveFnRegs[FN_MF_FUNC_SEL-FN_BASE];
        switch (CtrlUi.MfFunMode)
        {
        	case FN_MF_FUNC_SEL_TORQUE_MODE:
			{
				CtrlUi.MotorCtrlMode = v->MotorControl.StartUpWay;
				DriveFnRegs[ FN_TORQ_COMMAND - FN_BASE ] = 32768;
				break;
			}
            case FN_MF_FUNC_SEL_VF:
            {
                CtrlUi.MotorCtrlMode = FUNCTION_MODE_VF_CONTROL;
                v->MotorControl.VfControl.Position.RpmAccel = DriveFnRegs[ FN_RPM_SLOPE_CMD - FN_BASE ];
                v->MotorControl.VfControl.Position.RpmDecel = v->MotorControl.VfControl.Position.RpmAccel;
                v->MotorControl.VfControl.Gain = ((float)(DriveFnRegs[ FN_RPM_GAIN_CMD - FN_BASE ])) * 0.01f;
                v->MotorControl.VfControl.Position.EnablePositionCmd = DriveFnRegs[ FN_OPEN_POSITION_CMD_ENABLE - FN_BASE ];
                break;
            }
            case FN_MF_FUNC_SEL_IF:
            {
                CtrlUi.MotorCtrlMode = FUNCTION_MODE_IF_CONTROL;
                v->MotorControl.IfControl.Position.RpmAccel = DriveFnRegs[ FN_RPM_SLOPE_CMD - FN_BASE ];
                v->MotorControl.IfControl.Position.RpmDecel = v->MotorControl.IfControl.Position.RpmAccel;
                v->MotorControl.IfControl.Gain = ((float)(DriveFnRegs[ FN_RPM_GAIN_CMD - FN_BASE ])) * 0.01f;
                v->MotorControl.IfControl.CurrLimit = ((float)(DriveFnRegs[ FN_OPEN_SPD_V_I_LIMIT - FN_BASE ])) * 0.1f;
                v->MotorControl.IfControl.Position.EnablePositionCmd = DriveFnRegs[ FN_OPEN_POSITION_CMD_ENABLE - FN_BASE ];
                break;
            }
            case FN_MF_FUNC_SEL_IDQ:
            {
            	CtrlUi.MotorCtrlMode = FUNCTION_MODE_NORMAL_CURRENT_CONTROL;
            	DriveFnRegs[ FN_CURRENT_ID_CMD - FN_BASE ] = 32768;
            	DriveFnRegs[ FN_CURRENT_IQ_CMD - FN_BASE ] = 32768;
            	v->MotorControl.CurrentControl.EnableDirectIdqCmd = FUNCTION_ENABLE;
            	break;
            }
            case FN_MF_FUNC_SEL_ISTHETA:
            {
            	CtrlUi.MotorCtrlMode = FUNCTION_MODE_NORMAL_CURRENT_CONTROL;
            	v->MotorControl.CurrentControl.EnableDirectIdqCmd = FUNCTION_ENABLE;
            	break;
            }
            case FN_MF_FUNC_SEL_PWM:
            {
                CtrlUi.MotorCtrlMode = FUNCTION_MODE_PWM;
            	v->MotorControl.PwmDutyCmd.DutyLimitation.Duty[0] = 0;
            	v->MotorControl.PwmDutyCmd.DutyLimitation.Duty[1] = 0;
            	v->MotorControl.PwmDutyCmd.DutyLimitation.Duty[2] = 0;
            	switch ( DriveFnRegs[ FN_PWM_Mode - FN_BASE ] )
            	{
            	    case FN_PWM_MODE_COMPLEMENTARY:
            	    {
            	    	v->MotorControl.PwmDutyCmd.DutyLimitation.PwmMode = PWM_COMPLEMENTARY;
            	    	break;
            	    }
            	    case FN_PWM_MODE_UV:
            	    {
            	    	v->MotorControl.PwmDutyCmd.DutyLimitation.PwmMode = PWM_UbVbWx;
            	    	break;
            	    }
            	    case FN_PWM_MODE_VW:
            	    {
            	    	v->MotorControl.PwmDutyCmd.DutyLimitation.PwmMode = PWM_UxVbWb;
            	    	break;
            	    }
            	    case FN_PWM_MODE_UW:
            	    {
            	    	v->MotorControl.PwmDutyCmd.DutyLimitation.PwmMode = PWM_UbVxWb;
            	    	break;
            	    }
            	    default:
            	    {
            	    	v->MotorControl.PwmDutyCmd.DutyLimitation.PwmMode = PWM_COMPLEMENTARY;
            	    	break;
            	    }

            	}
            	break;
            }
            default:
            {
                AxisFactory_CleanParameter();
                break;
            }
        }
    }
    switch (CtrlUi.MfFunMode)
    {
    	case FN_MF_FUNC_SEL_TORQUE_MODE:
        case FN_MF_FUNC_SEL_VF:
        case FN_MF_FUNC_SEL_IF:
        case FN_MF_FUNC_SEL_IDQ:
        case FN_MF_FUNC_SEL_ISTHETA:
        case FN_MF_FUNC_SEL_PWM:
        {
            v->CtrlUiEnable = ( DriveFnRegs[ FN_ENABLE - FN_BASE ] == FN_ENABLE_MF_START) ? 1 : 0;
            break;
        }
        default:
        {
            AxisFactory_CleanParameter();
            break;
        }
    }   
}

#if BME
void AxisFactory_GetScooterThrottle( Axis_t *v )
{
    //Throttle detect code
    v->ThrotMapping.ThrottleDI = ENABLE;

    v->ThrotMapping.ThrottleRawIn = (float)(v->pCANRxInterface->ThrottleCmd)*0.01f;

    v->ThrotMapping.ChangeTime = v->FourQuadCtrl.DriveChangeTime;
    v->ThrotMapping.Calc( &v->ThrotMapping );
    v->ThrotMapping.Ramp( &v->ThrotMapping );
    v->ThrotMapping.TnSelectDelay = v->ThrotMapping.TnSelect;
}
#elif E10
void AxisFactory_GetScooterThrottle( Axis_t *v )
{
    //Throttle detect code
	//TODO: should refer break signal as well?
    v->ThrotMapping.ThrottleDI = ENABLE;

    v->ThrotMapping.ThrottleRawIn = v->pAdcStation->AdcTraOut.Throttle;

    v->ThrotMapping.ChangeTime = v->FourQuadCtrl.DriveChangeTime;
    v->ThrotMapping.Calc( &v->ThrotMapping );
    v->ThrotMapping.Ramp( &v->ThrotMapping );
    v->ThrotMapping.TnSelectDelay = v->ThrotMapping.TnSelect;
}
#endif

void AxisFactory_GetUiStatus( Axis_t *v )
{
    switch (CtrlUi.MfFunMode)
    {
    	case FN_MF_FUNC_SEL_TORQUE_MODE:
        case FN_MF_FUNC_SEL_VF:
        case FN_MF_FUNC_SEL_IF:
        case FN_MF_FUNC_SEL_IDQ:
        case FN_MF_FUNC_SEL_ISTHETA:
        {
            AxisFactory_GetScooterThrottle( v );
            v->FourQuadCtrl.ThrottleReleaseFlg = v->ThrotMapping.ThrottleReleaseFlag;
            v->FourQuadCtrl.Switch( &v->FourQuadCtrl );
            v->VCUServoOnCommand = v->FourQuadCtrl.ServoCmdOut;
            break;
        }
        default:
        {
            AxisFactory_GetScooterThrottle( v );
            v->FourQuadCtrl.ThrottleReleaseFlg = v->ThrotMapping.ThrottleReleaseFlag;
            v->FourQuadCtrl.Switch( &v->FourQuadCtrl );
            v->VCUServoOnCommand = 0;
            break;
        }
    }
}

void AxisFactory_GetUiCmd( Axis_t *v )
{
    switch (CtrlUi.MfFunMode)
    {
		case FN_MF_FUNC_SEL_TORQUE_MODE:
		{
			float TempTorqueCommandOut = 0.0f;

			// Get Allow Flux
			v->MotorControl.TorqueToIdq.GetAllowFluxRec( &(v->MotorControl.TorqueToIdq), v->MotorControl.SensorFb.EleSpeed, \
			v->MotorControl.SensorFb.Vbus, v->MotorControl.PwmDutyCmd.MaxDuty, &(v->MotorControl.SensorFb.AllowFluxRec), &(v->TorqCommandGenerator.AllowFluxRec) );

			// Renew the value of VbusUsed, VbusReal & MotorSpeed
			v->TorqCommandGenerator.VbusUsed = v->MotorControl.TorqueToIdq.VbusUsed;
			v->TorqCommandGenerator.MotorSpeed = v->SpeedInfo.MotorMechSpeedRad;

			// Decide the Torque Output( TO DO: Negative Torque )
			TempTorqueCommandOut = (( (float)DriveFnRegs[ FN_TORQ_COMMAND - FN_BASE ] - 32768.0f )) * 0.1f;
			v->FourQuadCtrl.TorqueCommandOut = TempTorqueCommandOut;
			v->ThrotMapping.PercentageOut = 1.0f;

			// Decide the AC Curr Limit
			v->TorqCommandGenerator.AcCurrLimit = v->ThermoStrategy.ACCurrentLimitOut;

			// Decide the DC Curr Limit
			v->TorqCommandGenerator.DcCurrLimit = \
				v->FourQuadCtrl.DCCurrLimitComparator( &v->FourQuadCtrl, v->pCANRxInterface->BatCurrentDrainLimit, \
				v->MotorControl.TorqueToIdq.VbusReal, v->MotorControl.TorqueToIdq.VbusUsed );

			v->TorqCommandGenerator.Calc( &v->TorqCommandGenerator, &v->FourQuadCtrl, v->ThrotMapping.PercentageOut );
			v->MotorControl.TorqueToIdq.GetIdqCmd( &(v->MotorControl.TorqueToIdq), v->TorqCommandGenerator.Out, v->MotorControl.SensorFb.AllowFluxRec);
			v->MotorControl.Cmd.SixWaveCurrCmd = v->MotorControl.TorqueToIdq.IqCmd;
			break;
		}
        case FN_MF_FUNC_SEL_VF:
        {
            v->MotorControl.Cmd.VfRpmTarget = DriveFnRegs[ FN_OPEN_SPD_COMMAND - FN_BASE ];
            v->MotorControl.VfControl.Position.PositionCmd = (float)DriveFnRegs[ FN_OPEN_POSITION_CMD - FN_BASE ] * 0.0001f;
            break;
        }
        case FN_MF_FUNC_SEL_IF:
        {
            v->MotorControl.Cmd.IfRpmTarget = DriveFnRegs[ FN_OPEN_SPD_COMMAND - FN_BASE ];
            v->MotorControl.IfControl.Position.PositionCmd = (float)DriveFnRegs[ FN_OPEN_POSITION_CMD - FN_BASE ] * 0.0001f;
            break;
        }
        case FN_MF_FUNC_SEL_IDQ:
        {
        	v->MotorControl.Cmd.IdCmd = ((float)DriveFnRegs[ FN_CURRENT_ID_CMD - FN_BASE ] - 32768) * 0.1f;
        	v->MotorControl.Cmd.IqCmd = ((float)DriveFnRegs[ FN_CURRENT_IQ_CMD - FN_BASE ] - 32768) * 0.1f;
        	break;
        }
        case FN_MF_FUNC_SEL_ISTHETA:
        {
        	float SinValue = 0.0f;
        	float CosValue = 0.0f;
 //       	COORDINATE_TRANSFER_GET_SIN_COS( ((float)DriveFnRegs[ FN_CURRENT_THETA_CMD - FN_BASE ] * 0.0001f), SinValue, CosValue );
        	float tempThetaCmd = ((float)DriveFnRegs[ FN_CURRENT_THETA_CMD - FN_BASE ] * 0.0001f);
        	SinValue = sinf(tempThetaCmd);
        	CosValue = cosf(tempThetaCmd);
        	v->MotorControl.Cmd.IdCmd = (float)DriveFnRegs[ FN_CURRENT_IS_CMD - FN_BASE ] * 0.1f * CosValue;
        	v->MotorControl.Cmd.IqCmd = (float)DriveFnRegs[ FN_CURRENT_IS_CMD - FN_BASE ] * 0.1f * SinValue;
        	break;
        }
        case FN_MF_FUNC_SEL_PWM:
        {
        	v->MotorControl.PwmDutyCmd.DutyLimitation.Duty[0] = (float)DriveFnRegs[ FN_PWM_U_CMD - FN_BASE ] * 0.0001;
        	v->MotorControl.PwmDutyCmd.DutyLimitation.Duty[1] = (float)DriveFnRegs[ FN_PWM_V_CMD - FN_BASE ] * 0.0001;
        	v->MotorControl.PwmDutyCmd.DutyLimitation.Duty[2] = (float)DriveFnRegs[ FN_PWM_W_CMD - FN_BASE ] * 0.0001;
        	break;
        }
        default:
        {
            AxisFactory_CleanParameter();
            break;
        }
    }
}

void AxisFactory_Init( Axis_t *v, uint16_t AxisIndex )
{
    v->AxisID = AXIS_INDEX_TO_AXIS_ID ( AxisIndex );
    v->pDriveParams = &DriveParams;
    v->pPwmStation = &PwmStation1;
    v->pAdcStation = &AdcStation1;
    v->pAlarmStack = &AlarmStack[AxisIndex];
    v->pCANStaion = &ExtranetCANStation;
    v->pCANRxInterface = &ExtranetCANStation.RxInfo;
    v->pCANTxInterface = &ExtranetCANStation.TxInfo;
    v->pParamMgr = &ParamMgr1;

    // Prototype, Set Motor Parameters, then call motor control init
    MOTOR_CONTROL_PARAMETER_DEFAULT_TYPE MotorControlSetting = MotorDefault;

    v->MotorControl.Init( &v->MotorControl, &MotorControlSetting );
    v->AlarmDetect.Init( &v->AlarmDetect, v->AxisID, v->pAdcStation,
            v->pPwmStation, &v->PhaseLoss,
            &v->MotorControl, &v->SpeedInfo, &v->PwmRc );
    v->AlarmDetect.RegisterAxisAlarm = (functypeAlarmDetect_RegisterAxisAlarm)RegisterAxisAlarm;
    v->ThrotMapping.Init( &v->ThrotMapping, &DriveParams );
    AxisSync_SyncFourQuadParams(v);
    v->TorqCommandGenerator.AcCurrLimitLut.Init(&(v->TorqCommandGenerator.AcCurrLimitLut), MotorDefault.pMotorTableHeader->AcCurrLimitHeader.Para);
    v->TorqCommandGenerator.DcCurrLimitLut.Init(&(v->TorqCommandGenerator.DcCurrLimitLut), MotorDefault.pMotorTableHeader->DcCurrLimitHeader.Para);

    v->BootstrapMaxCounter = (int16_t)(((float)v->BoostrapTimeMs) * 0.001f / v->MotorControl.CurrentControl.PwmPeriod + 0.5f);

    v->PhaseLoss.Init(&(v->PhaseLoss),16.0f,20.8f,0.004f,v->MotorControl.CurrentControl.PwmPeriod);

    v->SpeedInfo.Init(&(v->SpeedInfo),v->MotorControl.MotorPara.PM.Polepair);

#if USE_HIGH_RESO_MOTOR_TABLE
    HiResoMotorTable_Init();
#endif
}

__attribute__(( section(".ram_function"))) void AxisFactory_DoCurrentLoop( Axis_t *v )
{
    uint16_t AxisIndex = AXIS_ID_TO_AXIS_INDEX( v->AxisID );

    // detect current loop signals
    v->AlarmDetect.DoCurrentLoop( &v->AlarmDetect );

    // Run motor state machine & bootstrap
    AxisFactory_RunMotorStateMachine(v);

    if( v->ServoOn )
    {
        // do FOC calculations
        v->MotorControl.Process( &v->MotorControl, v->MotorCtrlMode);

        // Update Pwm
        if ( v->pPwmStation->ASC_Enable == 0 )
        {
            v->pPwmStation->DutyCmd.Duty[CH_PWM_UP] = v->MotorControl.PwmDutyCmd.DutyLimitation.Duty[0];
            v->pPwmStation->DutyCmd.Duty[CH_PWM_WP] = v->MotorControl.PwmDutyCmd.DutyLimitation.Duty[1];
            v->pPwmStation->DutyCmd.Duty[CH_PWM_VP] = v->MotorControl.PwmDutyCmd.DutyLimitation.Duty[2];
        }
        else
        {
            v->pPwmStation->DutyCmd.Duty[CH_PWM_UP] = 0.0f;
            v->pPwmStation->DutyCmd.Duty[CH_PWM_WP] = 0.0f;
            v->pPwmStation->DutyCmd.Duty[CH_PWM_VP] = 0.0f;
        }

        v->pPwmStation->AxisDutyToPwmCount( v->pPwmStation, AxisIndex, v->MotorControl.PwmDutyCmd.DutyLimitation.PwmMode);

        v->PhaseLoss.Acc10KhzCurrSqrU += v->MotorControl.SensorFb.Iu * v->MotorControl.SensorFb.Iu;
        v->PhaseLoss.Acc10KhzCurrSqrV += v->MotorControl.SensorFb.Iv * v->MotorControl.SensorFb.Iv;
        v->PhaseLoss.Acc10KhzCurrSqrW += v->MotorControl.SensorFb.Iw * v->MotorControl.SensorFb.Iw;
        CurrToPLCCnt++;
    }
    else
    {
        v->MotorControl.Clean( &v->MotorControl );
        CurrToPLCCnt = 0;
    }
}

void AxisFactory_DoPLCLoop( Axis_t *v )
{
    // Update Speed
	v->SpeedInfo.MotorMechSpeedRad = PSStation1.MechSpeed;
    v->SpeedInfo.MotorMechSpeedRPM = v->SpeedInfo.MotorMechSpeedRad * RAD_2_RPM;
    v->SpeedInfo.ElecSpeed = v->SpeedInfo.MotorMechSpeedRad * (float)v->SpeedInfo.Polepair;
    v->SpeedInfo.MotorMechSpeedRPMAbs = ABS(v->SpeedInfo.MotorMechSpeedRPM);
    v->SpeedInfo.ElecSpeedAbs = ABS(v->SpeedInfo.ElecSpeed);
    v->MotorControl.SensorFb.EleSpeed = v->SpeedInfo.ElecSpeed;


    // Detect PLC loop signals and register alarm.
    v->AlarmDetect.DoPLCLoop( &v->AlarmDetect );

    //GearMode
    v->GearModeVar.IsBoostBtnPressed = Btn_StateRead(BTN_IDX_BST_BTN);
    v->GearModeVar.IsReverseBtnPressed = Btn_StateRead(BTN_IDX_REV_BTN);
    GearMode_DoPLCLoop( &v->GearModeVar );
    v->FourQuadCtrl.DriveGearModeSelect = v->GearModeVar.GearModeSelect;
    v->pCANRxInterface->OutputModeCmd = ( v->GearModeVar.GearModeSelect == NORMAL_MODE ) ? 1 : ( v->GearModeVar.GearModeSelect == BOOST_MODE ) ? 2 : 0;

    // Because RCCommCtrl.MsgDecoder(&RCCommCtrl) execute in DoHouseKeeping loop and DoPLCLoop has higher priority.
    // Rewrite TN to limp home mode (TN0) and power level = 10 before AxisFactory_UpdateCANRxInterface here.
    // It makes sure that the rewrite take effect.
    if( v->TriggerLimpHome == 1)
    {
        v->pCANRxInterface->OutputModeCmd = 0;
    }
    else
    {
    	// In other states, use the original power level and output mode command.
    }

    // Update scooter speed for report
    v->FourQuadCtrl.MotorSpeedRadps = v->SpeedInfo.MotorMechSpeedRad;
    v->FourQuadCtrl.MotorRPM = v->SpeedInfo.MotorMechSpeedRPM;

    AxisFactory_UpdateCANRxInterface( v );
    AxisFactory_UpdateCANTxInterface( v );
    AxisFactory_ConfigAlarmSystemInPLCLoop( v );

//  if ((DriveFnRegs[FN_ENABLE-FN_BASE] | DriveFnRegs[FN_MF_FUNC_SEL-FN_BASE] | DriveFnRegs[FN_RD_FUNC_SEL-FN_BASE]) == 0)
    if ( v->MfOrRDFunctionDisable )
    {
        //Do nothing, there is a bug for changing DriveFnRegs[FN_MF_FUNC_SEL-FN_BASE] from 1 to 0.
    }
    else
    {
    	// todo do AxisFactory_CleanParameter() out of the AxisFactory_GetSetting; Avoid modify DriveFnRegs in AxisFactory_GetSetting.
        AxisFactory_GetSetting( v );
    }

    if ( v->MfOrRDFunctionDisable )
    {
        //Throttle detect code
        AxisFactory_GetScooterThrottle( v );
        v->FourQuadCtrl.ThrottleReleaseFlg = v->ThrotMapping.ThrottleReleaseFlag;
        v->FourQuadCtrl.Switch( &v->FourQuadCtrl );
        v->VCUServoOnCommand = v->FourQuadCtrl.ServoCmdOut;
    }
    else
    {
        AxisFactory_GetUiStatus( v );
    }

    // Change MinTime and calculate new VbusGain
    v->MotorControl.DriverPara.Mosfet.LowerBridgeMinTime = ( v->SpeedInfo.ElecSpeedAbs > v->MotorControl.DriverPara.Mosfet.MinTimeEleSpeedAbs ) ? 0.0f : MotorDefault.MosfetDriverLowerBridgeMinTime;
    v->MotorControl.TorqueToIdq.VbusGainForFW = 1 - 2 * ( v->MotorControl.DriverPara.Mosfet.DeadTime * 2 + v->MotorControl.DriverPara.Mosfet.LowerBridgeMinTime ) * v->MotorControl.CurrentControl.PwmHz;
    v->MotorControl.TorqueToIdq.VbusGainForVsaturation = 1 - ( v->MotorControl.DriverPara.Mosfet.DeadTime * 2 + v->MotorControl.DriverPara.Mosfet.LowerBridgeMinTime ) * v->MotorControl.CurrentControl.PwmHz;

    if( v->ServoOn )
    {
        if( CurrToPLCCnt >= 10) //  current loop routines is ten times of PLC loop.
        {
            v->PhaseLoss.Avg1KhzCurrSqrU = v->PhaseLoss.Acc10KhzCurrSqrU * 0.1f;
            v->PhaseLoss.Avg1KhzCurrSqrV = v->PhaseLoss.Acc10KhzCurrSqrV * 0.1f;
            v->PhaseLoss.Avg1KhzCurrSqrW = v->PhaseLoss.Acc10KhzCurrSqrW * 0.1f;

            v->PhaseLoss.Acc10KhzCurrSqrU = 0.0f;
            v->PhaseLoss.Acc10KhzCurrSqrV = 0.0f;
            v->PhaseLoss.Acc10KhzCurrSqrW = 0.0f;

            CurrToPLCCnt = 0;
        }
        else
        {
            // timer setting is abnormal, but do nothing now.
        }

        v->PhaseLoss.PLCLoopVcmdAmp = v->MotorControl.VoltCmd.VcmdAmp;
        v->PhaseLoss.PLCLoopElecSpeedAbs = v->SpeedInfo.ElecSpeedAbs;
        v->PhaseLoss.PLCLoopTorqueCmd = v->TorqCommandGenerator.Out;
        if(v->PhaseLoss.Enable == ALARM_ENABLE)
        {
        	v->PhaseLoss.RunTimeDetect( &v->PhaseLoss );
        }
        if ( v->MfOrRDFunctionDisable )	//Normal Mode
        {
            v->FourQuadCtrl.Driving_TNIndex = v->pCANRxInterface->OutputModeCmd;
            // Input throttle command, and calculate torque command for FOC.
            v->MotorControl.TorqueToIdq.GetAllowFluxRec( &(v->MotorControl.TorqueToIdq), v->MotorControl.SensorFb.EleSpeed, \
                    v->MotorControl.SensorFb.Vbus, v->MotorControl.PwmDutyCmd.MaxDuty, &(v->MotorControl.SensorFb.AllowFluxRec), &(v->TorqCommandGenerator.AllowFluxRec) );

            v->TorqCommandGenerator.VbusUsed = v->MotorControl.TorqueToIdq.VbusUsed;
            v->TorqCommandGenerator.MotorSpeed = v->SpeedInfo.MotorMechSpeedRad;
            v->FourQuadCtrl.Throttle = v->ThrotMapping.PercentageOut;
            v->FourQuadCtrl.Calc( &v->FourQuadCtrl );

            v->TorqCommandGenerator.AcCurrLimit = v->ThermoStrategy.ACCurrentLimitOut;



            v->TorqCommandGenerator.DcCurrLimit = \
                v->FourQuadCtrl.DCCurrLimitComparator( &v->FourQuadCtrl, v->pCANRxInterface->BatCurrentDrainLimit, \
                v->MotorControl.TorqueToIdq.VbusReal, v->MotorControl.TorqueToIdq.VbusUsed );

            v->TorqCommandGenerator.Calc( &v->TorqCommandGenerator, &v->FourQuadCtrl, v->ThrotMapping.PercentageOut );

            v->MotorControl.TorqueToIdq.GetIdqCmd( &(v->MotorControl.TorqueToIdq), v->TorqCommandGenerator.Out, v->MotorControl.SensorFb.AllowFluxRec );
            v->MotorControl.Cmd.SixWaveCurrCmd = v->MotorControl.TorqueToIdq.IqCmd;
        }
        else
        {
            AxisFactory_GetUiCmd(v);
        }
    }
    else
    {
        v->PhaseLoss.RunTimeClean( &v->PhaseLoss );
        if ( v->MfOrRDFunctionDisable )	//Normal Mode
        {
            CtrlUi.MotorCtrlMode = v->MotorControl.StartUpWay;
        }
        else
        {
            //do nothing
        }
        v->FourQuadCtrl.Reset( &v->FourQuadCtrl, v->pCANRxInterface->BatCurrentDrainLimit );
        v->ThrotMapping.TnSelectEmpty = v->ThrotMapping.TnSelect;
        v->ThrotMapping.TnSelectHalf = v->ThrotMapping.TnSelect;
        v->ThrotMapping.TnSelectFull = v->ThrotMapping.TnSelect;
    }

    if( v->AlarmDetect.BufICEnable )
    {
        HAL_GPIO_WritePin( BUF_ENA_DO_GPIO_Port, BUF_ENA_DO_Pin, GPIO_PIN_SET );		//Disable Buffer Enable
    }
    else
    {
        HAL_GPIO_WritePin( BUF_ENA_DO_GPIO_Port, BUF_ENA_DO_Pin, GPIO_PIN_RESET );	//Enable  Buffer Enable
    }

}

void AxisFactory_Do100HzLoop( Axis_t *v )
{
	float ACPowerTemp = 0;

    v->PwmRc.AlarmDet( &v->PwmRc );
    v->AlarmDetect.Do100HzLoop( &v->AlarmDetect );
    if( v->ServoOn )
    {
        if( (CtrlUi.MotorCtrlMode==FUNCTION_MODE_VF_CONTROL) || (CtrlUi.MotorCtrlMode==FUNCTION_MODE_IF_CONTROL) )
        {

        }
        else
        {
        	if( v->MotorStall.Enable == ALARM_ENABLE)
        	{
				float MaxABSPhaseCurrent =  MAX3( ABS(v->pAdcStation->AdcTraOut.Iu[v->AxisID-1]), ABS(v->pAdcStation->AdcTraOut.Iv[v->AxisID-1]), ABS(v->pAdcStation->AdcTraOut.Iw[v->AxisID-1]) ) ;
				v->MotorStall.Calc( &v->MotorStall, MaxABSPhaseCurrent, ABS(v->SpeedInfo.MotorMechSpeedRPM) );
				if( v->MotorStall.IsMotorStall ) //todo AlarmDetect.Do100Hzloop
				{
					v->AlarmDetect.RegisterAxisAlarm( &v->AlarmDetect, ALARMID_MOTORSTALL, SystemTable.AlarmTableInfo[ALARMID_MOTORSTALL].AlarmType );
				}
        	}
        }
    }
    else
    {
        v->MotorStall.Reset( &v->MotorStall );
    }

    /* Calculate AC instantaneous and average Power */
    ACPowerTemp = 1.5 * (v->MotorControl.CurrentControl.RotorCurrFb.D * v->MotorControl.VoltCmd.VdCmd +
    		v->MotorControl.CurrentControl.RotorCurrFb.Q * v->MotorControl.VoltCmd.VqCmd);
    AcPwrInfo.do100HzLoop(&AcPwrInfo, ACPowerTemp);
}

void AxisFactory_Do10HzLoop( Axis_t *v )
{
    if( v->ServoOn )
    {
#if USE_THERMAL_DERATING==USE_FUNCTION
        v->ThermoStrategy.Calc( &v->ThermoStrategy );
#else
        v->ThermoStrategy.ACCurrentLimitOut = MAX_AC_PHASE_CURRENT;
#endif
    }
}
