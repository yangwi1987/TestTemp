/*
* AxisFactory.c
*
*  Created on: 2019年12月11日
*      Author: MikeSFWen
*/

#include "ParamMgr.h"
#include "UiApp.h"
#include "AxisFactory.h"

#define ABS(x) 	( (x) > 0 ? (x) : -(x) )
#define MAX3(x,y,z)   (( (x > y) ? x : y ) > z ? ( x > y ? x : y ) : z)

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
	if(v->pCANTxInterface->DebugU8[TX_INTERFACE_DBG_IDX_BMS_COMM_ENABLE] == 1)
	{
		/* check if BMS send shutdown request */
		if(v->pCANRxInterface->BmsReportInfo.BmsShutdownRequest == 1)
		{
			v->pCANRxInterface->PcuStateCmd = PcuCmd_Shutdown;
		}
		else
		{
			v->pCANRxInterface->PcuStateCmd = PcuCmd_Enable;
		}

		// Load DC Current Limit from BMS communication, or use default value.
		v->pCANRxInterface->BatCurrentDrainLimit = v->pCANRxInterface->BmsReportInfo.DCCurrentLimit;
	}
	else
	{
		// BMS communication is not available, put pseudo value to BMS main and prch state 
		v->pCANRxInterface->PcuStateCmd = PcuCmd_Enable;
        v->pCANRxInterface->BmsReportInfo.PrchSM = BMS_PRECHG_STATE_SUCCESSFUL;
        v->pCANRxInterface->BmsReportInfo.MainSm = BMS_ACTIVE_STATE_DISCHARGE;
        v->pCANRxInterface->BatCurrentDrainLimit = DEFAULT_DC_LIMIT;
	}

    v->ThrotMapping.TnSelect = v->pCANRxInterface->OutputModeCmd;
    if( ( v->pCANRxInterface->ReceivedCANID & RECEIVED_BAT_ID_1 ) == RECEIVED_BAT_ID_1)
    {
        v->AlarmDetect.CAN1Timeout.Counter = 0;
    }
    v->pCANRxInterface->ReceivedCANID = 0;
    v->FourQuadCtrl.DrivePowerLevelTarget = ((float)v->pCANRxInterface->PowerLevel) * 0.1f;
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

// set BMS LED control in Drive_ESCOPVehicleStateMachine
/*	v->pCANTxInterface->DebugU8[TX_INTERFACE_DBG_IDX_LED_CTRL_CMD] =
		(v->pCANTxInterface->DebugU8[TX_INTERFACE_DBG_IDX_ERROR_FLAG] == 0) ? BAT_LED_SHOW_NO_ERROR : BAT_LED_SHOW_ESC_ERROR;
*/
    if(v->pCANTxInterface->DebugU8[TX_INTERFACE_DBG_IDX_LOG_SAMPLE_FLAG] == 1){
        v->pCANTxInterface->NTCTemp[0] = (int16_t)v->pAdcStation->AdcTraOut.MOTOR_NTC;	//Motor
        v->pCANTxInterface->NTCTemp[1] = (int16_t)v->pAdcStation->AdcTraOut.PCU_NTC[MOS_NTC_CENTER]; //MOS1
        v->pCANTxInterface->NTCTemp[2] = (int16_t)v->pAdcStation->AdcTraOut.PCU_NTC[MOS_NTC_SIDE]; //MOS2
        v->pCANTxInterface->NTCTemp[3] = (int16_t)v->pAdcStation->AdcTraOut.PCU_NTC[CAP_NTC]; //CAP
    }

    if( v->PcuPowerState == PWR_SM_INITIAL )
    {
        v->pCANTxInterface->PcuStateReport = PCU_STATE_INITIAL;
    }
    else if( v->PcuPowerState == PWR_SM_POWER_ON )
    {
        if (v->FourQuadCtrl.ServoCmdOut == ENABLE ){
            v->pCANTxInterface->PcuStateReport = PCU_STATE_OUTPUT;
        }else{
            v->pCANTxInterface->PcuStateReport = PCU_STATE_STANDBY;
        }
    }
    else if( v->PcuPowerState == PWR_SM_POWER_OFF )
    {
        v->pCANTxInterface->PcuStateReport = PCU_STATE_POWER_OFF;
    }
    else 
    {
        // undefined value, register error
        v->pCANTxInterface->PcuStateReport = PCU_STATE_ERROR;
    }

    if(v->pCANTxInterface->DebugU8[TX_INTERFACE_DBG_IDX_LOG_SAMPLE_FLAG] == 1){
        for(i=0;i<10;i++){
            v->pCANTxInterface->DebugError[i] = v->pAlarmStack->NowAlarmID[i];
        }
    }

    v->pCANTxInterface->DeratingSrc = v->ThermoStrategy.ThermoDeratingSrc;

    // debug
    v->pCANTxInterface->MotorRpm = (int16_t)v->SpeedInfo.MotorMechSpeedRPM;
    v->pCANTxInterface->VoltDcBu0P1V = (int16_t)( v->pAdcStation->AdcTraOut.BatVdc * 10 );
    if(v->pCANTxInterface->DebugU8[TX_INTERFACE_DBG_IDX_LOG_SAMPLE_FLAG] == 1){
        v->pCANTxInterface->Id_cmd = v->MotorControl.CurrentControl.IdCmd;
        v->pCANTxInterface->Iq_cmd = v->MotorControl.CurrentControl.IqCmd;
        v->pCANTxInterface->Id_fbk = v->MotorControl.CurrentControl.RotorCurrFb.D;
        v->pCANTxInterface->Iq_fbk = v->MotorControl.CurrentControl.RotorCurrFb.Q;
        v->pCANTxInterface->Debugf[IDX_AC_LIMIT_CMD] = v->TorqCommandGenerator.AcCurrLimit;
        v->pCANTxInterface->Debugf[IDX_AC_LIMIT_TQ] = v->TorqCommandGenerator.ACLimitedTorqueCommand;
        v->pCANTxInterface->Debugf[IDX_DC_LIMIT_CMD] = v->TorqCommandGenerator.DcCurrLimit;
        v->pCANTxInterface->Debugf[IDX_DC_LIMIT_TQ] = v->TorqCommandGenerator.DCLimitedTorqueCommand;
        v->pCANTxInterface->Debugf[IDX_PERFROMANCE_TQ] = v->TorqCommandGenerator.Out;
        v->pCANTxInterface->Debugf[IDX_VD_CMD] = v->MotorControl.VoltCmd.VdCmd;
        v->pCANTxInterface->Debugf[IDX_VQ_CMD] = v->MotorControl.VoltCmd.VqCmd;
        v->pCANTxInterface->Debugf[IDX_MOTOR_RPM] = v->SpeedInfo.MotorMechSpeedRPM;
        v->pCANTxInterface->Debugf[IDX_DC_VOLT] = v->pAdcStation->AdcTraOut.BatVdc;
        v->pCANTxInterface->Debugf[IDX_THROTTLE_RAW] =v->PwmRc.DutyRaw;
        v->pCANTxInterface->Debugf[IDX_THROTTLE_FINAL]= v->ThrotMapping.PercentageTarget;
        v->pCANTxInterface->Debugf[IDX_INSTANT_AC_POWER]= AcPwrInfo.InstPower;
        v->pCANTxInterface->Debugf[IDX_AVERAGE_AC_POWER]= AcPwrInfo.AvgPower;
#if USE_ANALOG_FOIL_SENSOR_FUNC
        v->pCANTxInterface->Debugf[IDX_FOIL_SENSOR_VOLT] = v->pAdcStation->AdcTraOut.Foil;
#endif
        v->pCANTxInterface->Debugf[IDX_DC_LIMIT_CANRX_DC_CURR] = v->pCANRxInterface->BatCurrentDrainLimit;
        v->pCANTxInterface->Debugf[IDX_RESERVERD] =  0.0f;
        v->pCANTxInterface->Debugf[IDX_DC_LIMIT_DCBUS_REAL] = v->MotorControl.TorqueToIdq.VbusReal;
    }

}


static void AxisFactory_ConfigAlarmSystemInPLCLoop( Axis_t *v )
{
    switch(v->PcuPowerState)
    {
        case PWR_SM_POWER_ON:
        {
        	if(v->pCANTxInterface->DebugU8[TX_INTERFACE_DBG_IDX_BMS_COMM_ENABLE] == 0)
            {
        		v->AlarmDetect.CAN1Timeout.AlarmInfo.AlarmEnable = ALARM_DISABLE;
            }
        	else
        	{
        		v->AlarmDetect.CAN1Timeout.AlarmInfo.AlarmEnable = ALARM_ENABLE;
        	}

            v->AlarmDetect.UVP_Bus.AlarmInfo.AlarmEnable = ALARM_ENABLE;
            break;
        }
        case PWR_SM_INITIAL :
        case PWR_SM_POWER_OFF :
        case PWR_SM_WAIT_FOR_RESET :
        default :
        {
            v->AlarmDetect.CAN1Timeout.AlarmInfo.AlarmEnable = ALARM_DISABLE;
            v->AlarmDetect.UVP_Bus.AlarmInfo.AlarmEnable = ALARM_DISABLE;
            break;
        }
    }
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

    int ServoOnEnable = (v->HasCriAlarm == 0) && ((v->VCUServoOnCommand == 1) || (v->CtrlUiEnable == 1));

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
                	if ( v->DriveLockInfo.DriveStateFlag == Drive_Start_Flag )
                	{
                		v->ServoOnOffState = MOTOR_STATE_ON;
                	}

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
#if	USE_EEMF==USE_FUNCTION
                if( (CtrlUi.MotorCtrlMode==FUNCTION_MODE_VF_CONTROL) || (CtrlUi.MotorCtrlMode==FUNCTION_MODE_IF_CONTROL) )
                {

                }
                else
                {
                    v->MotorCtrlMode = ( v->MotorControl.Sensorless.EEMF.Start == FUNCTION_YES ) ? FUNCTION_MODE_EEMF : v->MotorCtrlMode;
                }
#else
                v->MotorCtrlMode = CtrlUi.MotorCtrlMode;
#endif
#if	USE_HFI_SIN==USE_FUNCTION
                if( (CtrlUi.MotorCtrlMode==FUNCTION_MODE_VF_CONTROL) || (CtrlUi.MotorCtrlMode==FUNCTION_MODE_IF_CONTROL) )
                {

                }
                else
                {
                    v->MotorCtrlMode = ( v->MotorControl.Sensorless.HFISin.Start == FUNCTION_YES ) ? FUNCTION_MODE_HFI_SIN : v->MotorCtrlMode;
                }
#else
                v->MotorCtrlMode = CtrlUi.MotorCtrlMode;
#endif
            }

            if( !ServoOnEnable )
            {
                v->ServoOnOffState = MOTOR_STATE_SHUTDOWN_START;
            }
            else if ( v->DriveLockInfo.DriveStateFlag == Drive_Stop_Flag )
            {
            	v->ServoOnOffState = MOTOR_STATE_WAIT_BOOT;
            	v->MotorCtrlMode = FUNCTION_MODE_BOOTSTRAP;
            	v->MotorControl.Clean( &v->MotorControl );
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
    DriveFnRegs[ FN_OPEN_SPD_COMMAND - FN_BASE ] = 0;
}

void AxisFactory_GetSetting( Axis_t *v )
{
    if ( v->ServoOn == MOTOR_STATE_OFF ) //Allow that PCU change the MF mode setting value in servo off status.
    {
        CtrlUi.MfFunMode = DriveFnRegs[FN_MF_FUNC_SEL-FN_BASE];
        switch (CtrlUi.MfFunMode)
        {
            case FN_MF_FUNC_SEL_VF:
            {
                CtrlUi.MotorCtrlMode = FUNCTION_MODE_VF_CONTROL;
                v->MotorControl.VfControl.Position.RpmAccel = DriveFnRegs[ FN_RPM_SLOPE_CMD - FN_BASE ];
                v->MotorControl.VfControl.Position.RpmDecel = v->MotorControl.VfControl.Position.RpmAccel;
                v->MotorControl.VfControl.Gain = ((float)(DriveFnRegs[ FN_RPM_GAIN_CMD - FN_BASE ])) * 0.01f;
                break;
            }
            case FN_MF_FUNC_SEL_IF:
            {
                CtrlUi.MotorCtrlMode = FUNCTION_MODE_IF_CONTROL;
                v->MotorControl.IfControl.Position.RpmAccel = DriveFnRegs[ FN_RPM_SLOPE_CMD - FN_BASE ];
                v->MotorControl.IfControl.Position.RpmDecel = v->MotorControl.IfControl.Position.RpmAccel;
                v->MotorControl.IfControl.Gain = ((float)(DriveFnRegs[ FN_RPM_GAIN_CMD - FN_BASE ])) * 0.01f;
                v->MotorControl.IfControl.CurrLimit = ((float)(DriveFnRegs[ FN_OPEN_SPD_V_I_LIMIT - FN_BASE ])) * 0.1f;
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
        case FN_MF_FUNC_SEL_VF:
        {
            v->CtrlUiEnable = ( DriveFnRegs[ FN_ENABLE - FN_BASE ] == FN_ENABLE_MF_START) ? 1 : 0;
            break;
        }
        case FN_MF_FUNC_SEL_IF:
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
        case FN_MF_FUNC_SEL_VF:
        {
            AxisFactory_GetScooterThrottle( v );
            v->FourQuadCtrl.ThrottleReleaseFlg = v->ThrotMapping.ThrottleReleaseFlag;
            v->FourQuadCtrl.Switch( &v->FourQuadCtrl );
            v->VCUServoOnCommand = v->FourQuadCtrl.ServoCmdOut;
            break;
        }
        case FN_MF_FUNC_SEL_IF:
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
        case FN_MF_FUNC_SEL_VF:
        {
            v->MotorControl.Cmd.VfRpmTarget = DriveFnRegs[ FN_OPEN_SPD_COMMAND - FN_BASE ];
            break;
        }
        case FN_MF_FUNC_SEL_IF:
        {
            v->MotorControl.Cmd.IfRpmTarget = DriveFnRegs[ FN_OPEN_SPD_COMMAND - FN_BASE ];
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

    // Init analog foil sensor boundary
    v->AnalogFoilInfo.MaxSurf = v->pDriveParams->SystemParams.MaxAnaFoilSenSurf0p1V * 0.1f;
    v->AnalogFoilInfo.MinSurf = v->pDriveParams->SystemParams.MinAnaFoilSenSurf0p1V * 0.1f;
    v->AnalogFoilInfo.MaxFoil = v->pDriveParams->SystemParams.MaxAnaFoilSenFoil0p1V * 0.1f;
    v->AnalogFoilInfo.MinFoil = v->pDriveParams->SystemParams.MinAnaFoilSenFoil0p1V * 0.1f;

    //Load TimeToStopDriving_InPLCLoop, convert s to ms
    v->DriveLockInfo.IsUseDriveLockFn = v->pDriveParams->PCUParams.DebugParam2;
    v->DriveLockInfo.TimeToStopDriving_InPLCLoop = v->pDriveParams->SystemParams.SecTimeThresholdForDriveLock * 1000;
    v->DriveLockInfo.RpmToStartCntDriveLock = v->pDriveParams->SystemParams.RpmToStartCntDriveLock;
}

void AxisFactory_DoCurrentLoop( Axis_t *v )
{
    uint16_t AxisIndex = AXIS_ID_TO_AXIS_INDEX( v->AxisID );

    // detect current loop signals
    v->AlarmDetect.DoCurrentLoop( &v->AlarmDetect );

    // Run motor state machine & bootstrap
    AxisFactory_RunMotorStateMachine(v);

    if( v->ServoOn )
    {
        if( v->SpeedInfo.ElecSpeedAbs < EEMF_START_SPEED  )
        {
            if( v->MotorControl.Sensorless.EEMF.Start == FUNCTION_NO )
            {
                v->MotorControl.Sensorless.HFISin.HFISinCalcProcess = SENSORLESS_CALC_PROCESS_EXE;
            }
            else
            {
                if( v->SpeedInfo.ElecSpeedAbs < ( EEMF_START_SPEED + EEMF_CALC_SPEED ) * 0.5f )
                {
                    v->MotorControl.Sensorless.HFISin.HFISinCalcProcess = SENSORLESS_CALC_PROCESS_ENTERING;
                }
                else
                {
                    if( v->MotorControl.Sensorless.HFISin.HFISinCalcProcess == SENSORLESS_CALC_PROCESS_ENTERING )
                    {
                        // do nothing
                    }
                    else
                    {
                        v->MotorControl.Sensorless.HFISin.HFISinCalcProcess = SENSORLESS_CALC_PROCESS_CLEAN;
                    }
                }
            }
        }
        else
        {
            v->MotorControl.Sensorless.HFISin.HFISinCalcProcess = SENSORLESS_CALC_PROCESS_CLEAN;
        }

        if( ( v->SpeedInfo.ElecSpeedAbs < EEMF_CALC_SPEED ) )
        {
            v->MotorControl.Sensorless.EEMF.EEMFCalcProcess = SENSORLESS_CALC_PROCESS_CLEAN;
        }
        else
        {
            if( v->MotorControl.Sensorless.EEMF.EEMFCalcProcess == SENSORLESS_CALC_PROCESS_CLEAN )
            {
                v->MotorControl.Sensorless.EEMF.EEMFCalcProcess = SENSORLESS_CALC_PROCESS_ENTERING;
            }
            else
            {
                v->MotorControl.Sensorless.EEMF.EEMFCalcProcess = SENSORLESS_CALC_PROCESS_EXE;
            }
        }
        //
        // do FOC calculations
        v->MotorControl.Process( &v->MotorControl, v->MotorCtrlMode);

        // Update Pwm
        v->pPwmStation->DutyCmd.Duty[CH_PWM_UP] = v->MotorControl.PwmDutyCmd.DutyLimitation.Duty[0];
        v->pPwmStation->DutyCmd.Duty[CH_PWM_VP] = v->MotorControl.PwmDutyCmd.DutyLimitation.Duty[1];
        v->pPwmStation->DutyCmd.Duty[CH_PWM_WP] = v->MotorControl.PwmDutyCmd.DutyLimitation.Duty[2];

        v->pPwmStation->AxisDutyToPwmCount( v->pPwmStation, AxisIndex, v->MotorControl.PwmDutyCmd.DutyLimitation.PwmMode);

#if USE_EEMF==USE_FUNCTION
        if( v->SpeedInfo.ElecSpeedAbs > EEMF_START_SPEED )
        {
#if USE_EEMF==USE_FUNCTION
            v->MotorControl.Sensorless.EEMF.Start = FUNCTION_YES;
            v->MotorControl.Sensorless.SensorlessState = SensorlessState_Using_EEMF_Algorithm;
#else
            v->MotorControl.Sensorless.EEMF.Start = FUNCTION_NO;
#endif
            v->MotorControl.Sensorless.HFISin.Start = FUNCTION_NO;
        }
        else if( v->SpeedInfo.ElecSpeedAbs < EEMF_CALC_SPEED )
        {
            v->MotorControl.Sensorless.EEMF.Start = FUNCTION_NO;
#if USE_HFI_SIN==USE_FUNCTION
            v->MotorControl.Sensorless.HFISin.Start = ( v->MotorControl.Sensorless.AngleInit.Start == FUNCTION_YES ) ? FUNCTION_NO : FUNCTION_YES;
            v->MotorControl.Sensorless.SensorlessState = ( v->MotorControl.Sensorless.AngleInit.Start == FUNCTION_YES ) ? \
                                                            v->MotorControl.Sensorless.SensorlessState : SensorlessState_Using_HFI_Algorithm;
#else
            v->MotorControl.Sensorless.HFISin.Start = FUNCTION_NO;
#endif
        }
        else
        {
            //do nothing
            v->MotorControl.Sensorless.SensorlessState = ( v->MotorControl.Sensorless.SensorlessState == SensorlessState_Using_HFI_Algorithm ) ? \
            SensorlessState_Switching_from_HFI_to_EEMF : (( v->MotorControl.Sensorless.SensorlessState == SensorlessState_Using_EEMF_Algorithm ) ? \
            SensorlessState_Switching_from_EEMF_to_HFI : v->MotorControl.Sensorless.SensorlessState );
        }
    #endif

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
#if USE_HFI_SIN==USE_FUNCTION
    if( v->MotorControl.Sensorless.EEMF.Start == FUNCTION_YES )
    {
        v->SpeedInfo.MotorMechSpeedRad = v->MotorControl.Sensorless.EEMF.AngleObserver.SpeedTmp * v->SpeedInfo.DividePolepair;
    }
    else
    {
        v->SpeedInfo.MotorMechSpeedRad = v->MotorControl.Sensorless.HFISin.AngleObserver.SpeedTmp * v->SpeedInfo.DividePolepair;
    }
#endif
    v->SpeedInfo.MotorMechSpeedRPM = v->SpeedInfo.MotorMechSpeedRad * RAD_2_RPM;
    v->SpeedInfo.ElecSpeed = v->SpeedInfo.MotorMechSpeedRad * (float)v->SpeedInfo.Polepair;
    v->SpeedInfo.MotorMechSpeedRPMAbs = ABS(v->SpeedInfo.MotorMechSpeedRPM);
    v->SpeedInfo.ElecSpeedAbs = ABS(v->SpeedInfo.ElecSpeed);
    v->MotorControl.SensorFb.EleSpeed = v->SpeedInfo.ElecSpeed;


    // Detect PLC loop signals and register alarm.
    v->AlarmDetect.DoPLCLoop( &v->AlarmDetect );

    // Detect Digital Foil Position Sensor Read
    v->FoilState.Bit.FOIL_DI2= HAL_GPIO_ReadPin( FOIL_DI2_GPIO_Port, FOIL_DI2_Pin );
    v->FoilState.Bit.FOIL_DI3= HAL_GPIO_ReadPin( FOIL_DI3_GPIO_Port, FOIL_DI3_Pin );

    // Detect foil sensor by different hardware setting
    if(IsUseDigitalFoilSensor == 1) // use digitals foil sensor
    {
        if( v->FoilState.All== MAST_PADDLE )
        {
            v->pCANRxInterface->OutputModeCmd = DRIVE_PADDLE;
            v->pCANTxInterface->FoilPos = FOIL_POS_PADDLE;
        }
        else if( v->FoilState.All== MAST_SURF )
        {
            v->pCANRxInterface->OutputModeCmd = DRIVE_SURF;
            v->pCANTxInterface->FoilPos = FOIL_POS_SURF;
        }
        else if( v->FoilState.All== MAST_FOIL )
        {
            v->pCANRxInterface->OutputModeCmd = DRIVE_FOIL;
            v->pCANTxInterface->FoilPos = FOIL_POS_FOIL;
        }
        else
        {
            v->pCANRxInterface->OutputModeCmd = DRIVE_NONE;
        }
    }
    else // use Analog foil sensor
    {
#if USE_ANALOG_FOIL_SENSOR_FUNC
        if( ( v->pAdcStation->AdcTraOut.Foil >= v->AnalogFoilInfo.MinFoil ) && ( v->pAdcStation->AdcTraOut.Foil <= v->AnalogFoilInfo.MaxFoil  ) )  // Foil mode
        {
            v->pCANRxInterface->OutputModeCmd = DRIVE_FOIL;
            v->pCANTxInterface->FoilPos = FOIL_POS_FOIL;
        }
        else if ( ( v->pAdcStation->AdcTraOut.Foil >= v->AnalogFoilInfo.MinSurf ) && ( v->pAdcStation->AdcTraOut.Foil <= v->AnalogFoilInfo.MaxSurf ) )	// Surf mode
        {
            v->pCANRxInterface->OutputModeCmd = DRIVE_SURF;
        v->pCANTxInterface->FoilPos = FOIL_POS_SURF;
        }
        else	// PADDLE mode
        {
            v->pCANRxInterface->OutputModeCmd = DRIVE_PADDLE;
        v->pCANTxInterface->FoilPos = FOIL_POS_PADDLE;
        }
#endif
    }

    // Because RCCommCtrl.MsgDecoder(&RCCommCtrl) execute in DoHouseKeeping loop and DoPLCLoop has higher priority.
    // Rewrite TN to limp home mode (TN0) and power level = 10 before AxisFactory_UpdateCANRxInterface here.
    // It makes sure that the rewrite take effect.
    if( v->TriggerLimpHome == 1)
    {
        v->pCANRxInterface->OutputModeCmd = 0;
        v->pCANRxInterface->PowerLevel = 10;
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
    v->MotorControl.TorqueToIdq.VbusGain = 1 - 2 * ( v->MotorControl.DriverPara.Mosfet.DeadTime * 2 + v->MotorControl.DriverPara.Mosfet.LowerBridgeMinTime ) * v->MotorControl.CurrentControl.PwmHz;

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

            /*
             * code below is to fix a motor reverse bug.
             * We found if user pull the trigger before angle initial align complete, motor reverse may occur
             * We thing the cause is in this condition, the throttle filter is bypassed, step torque command cause motor react too fast make sensorless failed.
             * Solution is replace throttle command (after filter) as 0 before angle initial align complete, then the throttle filter can work well.
             */

            v->ThrotMapping.PercentageOut = v->MotorControl.Sensorless.AngleInit.Start == FUNCTION_NO ? v->ThrotMapping.PercentageOut : 0.0f;

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
        HAL_GPIO_WritePin( BUF_ENA_GPIO_Port, BUF_ENA_Pin, GPIO_PIN_SET );		//Disable Buffer Enable
    }
    else
    {
        HAL_GPIO_WritePin( BUF_ENA_GPIO_Port, BUF_ENA_Pin, GPIO_PIN_RESET );	//Enable  Buffer Enable
    }

    //check Drive lock state.
    if ( v->DriveLockInfo.IsUseDriveLockFn )
    {
        if ( v->DriveLockInfo.DriveStateFlag == Drive_Start_Flag )
        {
            if (( RCCommCtrl.pRxInterface->RcConnStatus <= RC_CONN_STATUS_RC_THROTTLE_LOCKED ) && ( v->SpeedInfo.MotorMechSpeedRPMAbs < (float)v->DriveLockInfo.RpmToStartCntDriveLock ))
            {
            	if ( v->DriveLockInfo.TimeToStopDriving_cnt >= v->DriveLockInfo.TimeToStopDriving_InPLCLoop )
            	{
            		v->DriveLockInfo.DriveStateFlag = Drive_Stop_Flag;
            		v->DriveLockInfo.TimeToStopDriving_cnt = 0;
            	}
            	else
            	{
            		v->DriveLockInfo.TimeToStopDriving_cnt++;
            	}
            }
            else
            {
            	v->DriveLockInfo.TimeToStopDriving_cnt = 0;
            }
        }
        else
        {
        	if ( RCCommCtrl.pRxInterface->RcConnStatus > RC_CONN_STATUS_RC_THROTTLE_LOCKED )
        	{
        		v->DriveLockInfo.DriveStateFlag = Drive_Start_Flag;
        	}
        }
    }
    else
    {
    	v->DriveLockInfo.DriveStateFlag = Drive_Start_Flag;
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
