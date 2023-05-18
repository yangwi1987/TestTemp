/*
 * Drive.c
 *
 *  Created on: Dec 19, 2019
 *      Author: MikeSFWen
 */

#include "UtilityBase.h"
#include "UiApp.h"
#include "Drive.h"
#define ABS(x) 	( (x) > 0 ? (x) : -(x) )

/*
 * Header Information
 * Note: AppVersion, BomNumber and SystemSupplyerID are put in the ".AppVerSpace", those order cannot be changed.
 */
__attribute__((__section__(".AppVerSpace"),used)) const uint16_t AppVersion[5] = APP_VERSION;
__attribute__((__section__(".AppVerSpace"),used)) const uint8_t BomNumber[PART_NUM_IDX] = PART_NUMBER;
__attribute__((__section__(".AppVerSpace"),used)) const uint8_t SystemSupplyerID[SYS_SUP_ID_IDX] = SUPPLYER_ID;

/*
 * Boot-loader Default Version
 */
const uint16_t BootlaoderVer[BOOT_VER_IDX] = BOOT_VER_NUMBER;

/*
 * Header Information -end-
 */
__attribute__((__section__(".AppCheckWordSpace"),used)) const uint16_t AppCheckWord = APP_CHECK_WORD;

uint8_t FWVerNumber[SW_VER_NUM_IDX] = { 'S', 'W', (uint8_t)PROJECT_CODE_X1, '.',
		                                         (uint8_t)PRODUCT_AND_BIN_TYPE_X2, '.',
												 (uint8_t)MAIN_RELEASE_NUMBER_YY, '.',
												 (uint8_t)MINOR_RELEASE_NUMBER_ZZ };

uint8_t HWVerNumber[HW_VER_NUM_IDX] = { 'H', 'W', (uint8_t)0, '.',
		                                       (uint8_t)0, '.',
											   (uint8_t)0 };


uint8_t PSBVerNumber[MOT_SW_VER_NUM_IDX] = { 'S', 'W', (uint8_t)0, '.',
		                                         (uint8_t)0, '.',
												 (uint8_t)0, '.',
												 (uint8_t)0 };

DriveParams_t DriveParams;
Axis_t Axis[MAX_AXIS_NUM] = {AXIS_DEFAULT, AXIS_DEFAULT};
AlarmMgr_t AlarmMgr1 = ALARM_MGR_DEFAULT;
ParamMgr_t ParamMgr1 = PARAM_MGR_DEFAULT;
CPUCounter_t CPUCounter = CPU_COUNTER_DEFAULT;
PwmStation PwmStation1 = PWM_STATION_DEFAULT;
AdcStation AdcStation1 = ADC_STATION_DEFAULT;
ExtranetCANStation_t ExtranetCANStation = EXTRANET_CAN_STATION_DEFAULT;

ExtFlash_t ExtFlash1 = EXT_FLASH_DEFAULT;
uint16_t BootAppTrig __attribute__((section(".dta_ss.word1st"))) = 0;
NetWorkService_t IntranetCANStation = NetWorkService_t_Default;
MFStation MFStation1 = MF_STATION_DEFAULT;
UdsSecurityAccessCtrl_t PcuAuthorityCtrl = UDS_SECURITY_ACCESS_CTRL_DEFAULT;
uint16_t IsPcuInitReady = PcuInitState_Inital;
uint16_t IsUseDigitalFoilSensor = 0;
IntFlashCtrl_t IntFlashCtrl = INT_FLASH_CTRL_DEFAULT;
// Declare total time
TotalTime_t TotalTime1 = TOTAL_TIME_DEFAULT;
int32_t AccessParam( uint16_t TargetID, uint16_t Index, int32_t *pData, uint16_t RW , uint8_t *pResult);
void Drive_OnParamValueChanged( uint16_t AxisID, uint16_t PN );

uint32_t VersionAddressArray[5] =
{
		APP_START_ADDRESS,
		SYS_TAB_START_ADDRESS,
		PCU_TAB_START_ADDRESS,
		MOT_1_TAB_START_ADDRESS,
		MOT_2_TAB_START_ADDRESS
};

/*
 * Boot-loader function declare
 */
pfunction Jump2app;		//Jump Code Pointer Function type declare
void (*Jump2app)();		//Jump Code Pointer Function declare
void JumpCtrlFunction( void );

/*
 * Version Check & CheckWord Check function
 */
uint16_t Drive_BinVersionCompare( const uint16_t *dta);
uint16_t Drive_BinCheckWordCompare( const uint16_t *dta);

int32_t drive_GetStatus(uint16_t AxisID, uint16_t no)
{
	int32_t RetValue;
	uint16_t DataBuf;
	uint16_t *BinAddress;
	int16_t Select;
//	uint16_t UidBuff[6];
	int AxisIndex = AxisID - 1;

	switch(no)
	{
	case DN_THROTTLE_RAW_ADCDATA:		// PWM RC RAW DATA
		RetValue = (int32_t)Axis[0].pCANRxInterface->ThrottleCmd;
		break;

	case DN_FOURQUAD_STATE:
		RetValue = (int32_t)( Axis[AxisIndex].FourQuadCtrl.FourQuadState );
		break;

	case DN_OUTPUT_TORQ_CMD:
		RetValue = (int32_t)(Axis[AxisIndex].TorqCommandGenerator.Out * 10.0f);
		break;

	case DN_SERVO_ON_STATE:
		RetValue = (int32_t)(Axis[AxisIndex].ServoOnOffState);
		break;

	case DN_SECURITY_ACCESS_LV:
		RetValue = PcuAuthorityCtrl.SecureLvNow;
		break;

	case DN_IO_STATUS:
		RetValue = (int32_t)( MFStation1.MFGpioInfo.AllBit );
		break;

	case DN_BUS_VOLTAGE:
		RetValue = (int32_t)( AdcStation1.AdcTraOut.BatVdc * 10.0f );
		break;

	case DN_IA_CURR:
		RetValue = (int32_t)( AdcStation1.AdcTraOut.Iu[AxisIndex] * 10.0f );
		break;

	case DN_IB_CURR:
		RetValue = (int32_t)( AdcStation1.AdcTraOut.Iv[AxisIndex] * 10.0f );
		break;

	case DN_IC_CURR:
		RetValue = (int32_t)( AdcStation1.AdcTraOut.Iw[AxisIndex] * 10.0f );
		break;

	case DN_IQ_CURR:
		RetValue = (int32_t)(Axis[AxisIndex].MotorControl.Cmd.IqCmd * 10.0f);
		break;

	case DN_MOTOR_SPEED:
		RetValue = (int32_t)( Axis[AxisIndex].SpeedInfo.MotorMechSpeedRPM );
		break;

	case DN_MOTOR_0_NTC_TEMP:
		RetValue = (int32_t)( AdcStation1.AdcTraOut.MOTOR_NTC * 10.0f );
		break;

	case DN_PCU_NTC_0_TEMP:
		RetValue = (int32_t)( AdcStation1.AdcTraOut.PCU_NTC[PCU_NTC_0] * 10.0f );
		break;

	case DN_PCU_NTC_1_TEMP:
		RetValue = (int32_t)( AdcStation1.AdcTraOut.PCU_NTC[PCU_NTC_1] * 10.0f );
		break;

	case DN_GATE_DRIVE_VOLT:	// 13V VOltage
		RetValue = (int32_t)( AdcStation1.AdcTraOut.V13 * 100.0f );
		break;

	case DN_IA_CURR_RMS:
		RetValue = (int32_t)( MFStation1.CurrRms.Iu[AxisIndex] * 10.0f );
		break;

	case DN_IB_CURR_RMS:
		RetValue = (int32_t)( MFStation1.CurrRms.Iv[AxisIndex] * 10.0f );
		break;

	case DN_IC_CURR_RMS:
		RetValue = (int32_t)( MFStation1.CurrRms.Iw[AxisIndex] * 10.0f );
		break;

	case DN_PCU_NTC_2_TEMP:
		RetValue = (int32_t)( AdcStation1.AdcTraOut.PCU_NTC[PCU_NTC_2] * 10.0f );
		break;

	case DN_FOIL_VOLTAGE:
		RetValue = (int32_t)( AdcStation1.AdcTraOut.Foil * 100.0f );
		break;
	case DC_FOIL_ADC:
		RetValue = (int32_t)AdcStation1.AdcDmaData[AdcStation1.RegCh[FOIL_AD].AdcGroupIndex][AdcStation1.RegCh[FOIL_AD].AdcRankIndex];
		break;

	case DN_U_CURR_ADC:
		RetValue = (int32_t)( AdcStation1.AdcRawData.Inj[ISE_U_A0].RawAdcValue );
		break;

	case DN_V_CURR_ADC:
		RetValue = (int32_t)( AdcStation1.AdcRawData.Inj[ISE_V_A0].RawAdcValue );
		break;

	case DN_W_CURR_ADC:
		RetValue = (int32_t)( AdcStation1.AdcRawData.Inj[ISE_W_A0].RawAdcValue );
		break;

	case DN_BAT_ADC:
		RetValue = (int32_t)( AdcStation1.AdcRawData.Inj[BAT_VDC].RawAdcValue );
		break;

	case DN_TN_SELECT:
		RetValue = (int32_t)(Axis[0].FourQuadCtrl.Driving_TNIndex );
		break;

	case DN_THROT_MAPPING_RST:
		RetValue = (int32_t)(Axis[0].ThrotMapping.PercentageOut*1000.0f );
		break;

	case DN_BOOTUP_TIME:
		RetValue = CPUCounter.PLCLoopCounter / 1000;
		break;

	case DN_STATUS_WORD1:
		//			Variable,										/*bit*/
		RetValue = 	Axis[AxisIndex].ServoOn							<< 0 |
					Axis[AxisIndex].VCUServoOnCommand				<< 1 |
					Axis[AxisIndex].CtrlUiEnable					<< 2 |
					Axis[AxisIndex].FourQuadCtrl.ServoCmdIn			<< 3 |
					Axis[AxisIndex].HasAlarm						<< 4 |
					Axis[AxisIndex].pAdcStation->ZeroCalibInjDone	<< 5 |
					Axis[AxisIndex].PhaseLoss.Enable				<< 6 |
					Axis[AxisIndex].HasWarning						<< 7 ;
		break;

	case DN_ID_VER_READ:
		Select = ( DriveFnRegs[ FN_READ_VERSION_CTRL - FN_BASE ] > 7 ) ? 0: DriveFnRegs[ FN_READ_VERSION_CTRL - FN_BASE ];
		if( Select == 0 )
		{
			RetValue = 0;
			break;
		}
		else if( Select == 7 )
		{
			DataBuf = *( ( uint16_t* )( BOOT_VER_ADDRESS + VERSION_PROJECT_CODE_OFFSET ) );
			if( DataBuf == (0x0000|0xFFFF) )
			{
				BinAddress = (uint16_t*)&BootlaoderVer[ VERSION_PROJECT_CODE_OFFSET ];
			}
			else
			{
				BinAddress = (uint16_t*)( BOOT_VER_ADDRESS + VERSION_PROJECT_CODE_OFFSET );
			}
			RetValue = (int32_t) *( BinAddress );
			break;
		}
		else if( Select == 6 )
		{
			BinAddress = (uint16_t*)ExtFlash1.ParamPack.Header.ExtFlashVersion;
			RetValue = (int32_t) *( BinAddress + VERSION_PROJECT_CODE_OFFSET );
			break;
		}
		else
		{
			BinAddress = (uint16_t*)VersionAddressArray[ ( Select - 1 ) ] ;
			RetValue = (int32_t) *( BinAddress + VERSION_PROJECT_CODE_OFFSET );
			break;
		}
	case DN_INFO_VER_READ:
		Select = ( DriveFnRegs[ FN_READ_VERSION_CTRL - FN_BASE ] > 7 ) ? 0: DriveFnRegs[ FN_READ_VERSION_CTRL - FN_BASE ];
		if( Select == 0 )
		{
			RetValue = 0;
			break;
		}
		else if( Select == 7 )
		{
			DataBuf = *( ( uint16_t* )( BOOT_VER_ADDRESS + VERSION_BINTYPE_OFFSET ) );
			if( DataBuf == (0x0000|0xFFFF) )
			{
				BinAddress = (uint16_t*)&BootlaoderVer[ VERSION_BINTYPE_OFFSET ];
			}
			else
			{
				BinAddress = (uint16_t*)( BOOT_VER_ADDRESS + VERSION_BINTYPE_OFFSET );
			}
			RetValue = (int32_t) *( BinAddress );
			break;
		}
		else if( Select == 6 )
		{
			BinAddress = (uint16_t*)ExtFlash1.ParamPack.Header.ExtFlashVersion;
			RetValue = (int32_t) *( BinAddress + VERSION_BINTYPE_OFFSET );
			break;
		}
		else
		{
			BinAddress = (uint16_t*)VersionAddressArray[ ( Select - 1 ) ] ;
			RetValue = (int32_t) *( BinAddress + VERSION_BINTYPE_OFFSET );
			break;
		}
	case DN_FRAME_VER_READ:
		Select = ( DriveFnRegs[ FN_READ_VERSION_CTRL - FN_BASE ] > 7 ) ? 0: DriveFnRegs[ FN_READ_VERSION_CTRL - FN_BASE ];
		if( Select == 0 )
		{
			RetValue = 0;
			break;
		}
		else if( Select == 7 )
		{
			DataBuf = *( ( uint16_t* )( BOOT_VER_ADDRESS + VERSION_MAIN_RELEASE_OFFSET ) );
			if( DataBuf == (0x0000|0xFFFF) )
			{
				BinAddress = (uint16_t*)&BootlaoderVer[ VERSION_MAIN_RELEASE_OFFSET ];
			}
			else
			{
				BinAddress = (uint16_t*)( BOOT_VER_ADDRESS + VERSION_MAIN_RELEASE_OFFSET );
			}
			RetValue = (int32_t) *( BinAddress );
			break;
		}
		else if( Select == 6 )
		{
			BinAddress = (uint16_t*)ExtFlash1.ParamPack.Header.ExtFlashVersion;
			RetValue = (int32_t) *( BinAddress + VERSION_MAIN_RELEASE_OFFSET );
			break;
		}
		else
		{
			BinAddress = (uint16_t*)VersionAddressArray[ ( Select - 1 ) ] ;
			RetValue = (int32_t) *( BinAddress + VERSION_MAIN_RELEASE_OFFSET );
			break;
		}
	case DN_RELEASE_VER_READ:
		Select = (DriveFnRegs[ FN_READ_VERSION_CTRL - FN_BASE] > 7 ) ? 0 : DriveFnRegs[ FN_READ_VERSION_CTRL - FN_BASE];
		if( Select == 0 )
		{
			RetValue = 0;
			break;
		}
		else if( Select == 7 )
		{
			DataBuf = *( ( uint16_t* )( BOOT_VER_ADDRESS + VERSION_MINOR_RELEASE_OFFSET ) );
			if( DataBuf == (0x0000|0xFFFF) )
			{
				BinAddress = (uint16_t*)&BootlaoderVer[ VERSION_MINOR_RELEASE_OFFSET ];
			}
			else
			{
				BinAddress = (uint16_t*)( BOOT_VER_ADDRESS + VERSION_MINOR_RELEASE_OFFSET );
			}
			RetValue = (int32_t) *( BinAddress );
			break;
		}
		else if( Select == 6 )
		{
			BinAddress = (uint16_t*)ExtFlash1.ParamPack.Header.ExtFlashVersion;
			RetValue = (int32_t)*(BinAddress + VERSION_MINOR_RELEASE_OFFSET);
			break;
		}
		else
		{
			BinAddress = (uint16_t*)VersionAddressArray[(Select - 1)];
			RetValue = (int32_t)*(BinAddress + VERSION_MINOR_RELEASE_OFFSET);
			break;
		}
	case DN_RELEASE_CANDIDATE_READ:
		Select = ( DriveFnRegs[ FN_READ_VERSION_CTRL - FN_BASE ] > 6 ) ? 0: DriveFnRegs[ FN_READ_VERSION_CTRL - FN_BASE ];
		if( Select == 0 || Select == 6 )
		{
			RetValue = 0;
			break;
		}
		else
		{
			BinAddress = (uint16_t*)VersionAddressArray[ ( Select - 1 ) ] ;
			RetValue = (int32_t) *( BinAddress + VERSION_RC_OFFSET );
			break;
		}
	case DN_ALRAM_ID_01:
	case DN_ALRAM_ID_23:
	case DN_ALRAM_ID_45:
	case DN_ALRAM_ID_67:
	case DN_ALRAM_ID_89:
		if( DriveFnRegs[ FN_ALARM_ID_READ_SRC - FN_BASE] == 0 )
		{
			RetValue =(int32_t)( (0xFF & AlarmStack[0].NowAlarmID[(no - DN_ALRAM_ID_01)*2]) + ((0xFF & AlarmStack[0].NowAlarmID[(no - DN_ALRAM_ID_01)*2+1])<<8));
		}
		else
		{
			RetValue =(int32_t)( (0xFF & AlarmStack[0].HistoryAlarmID[(no - DN_ALRAM_ID_01)*2]) + ((0xFF & AlarmStack[0].HistoryAlarmID[(no - DN_ALRAM_ID_01)*2+1])<<8));
		}
		break;

	case DN_INTERNAL_FLASH_OP_RESULT :
		RetValue = 0;
		break;

	case DN_DEBUG_SCOPE_1:
		RetValue = 0;
		break;
	case DN_DEBUG_SCOPE_2:
		RetValue = 0;
		break;
	case DN_DEBUG_SCOPE_3:
		RetValue = 0;
		break;
	case DN_DEBUG_SCOPE_4:
		RetValue = 0;
		break;
	case DN_DEBUG_SCOPE_5:
		RetValue = 0;
		break;
	case DN_DEBUG_SCOPE_6:
		RetValue = 0;
		break;
	case DN_DEBUG_SCOPE_7:
		RetValue = 0;
		break;
	case DN_DEBUG_SCOPE_8:
		RetValue = 0;
		break;
	case DN_DEBUG_SCOPE_9:
		RetValue = 0;
		break;
	case DN_DEBUG_SCOPE_10:
		RetValue = 0;
		break;
	case DN_THIS_OP_TIME:
		RetValue = (uint16_t)(TotalTime1.LocalThisTime * 0.05); // in minute
		break;
	case DN_TOTAL_OP_TIME:
		RetValue = (uint16_t)(roundf( (float)TotalTime1.LocalTotalTime * 0.0008333333f )); // in hour
		break;
	default :
		RetValue = 0;
		break;
	}

	return RetValue;
}

int32_t AccessParam( uint16_t TargetID, uint16_t PN, int32_t *pData, uint16_t RW, uint8_t *pResult )
{
	int32_t Data = 0; // avoid to be used before initialization.

	if( PN < 0 )
	{
		if(pResult!=NULL)
			*pResult = PARAM_ACCESS_FAIL_PARAM_ID_NOT_DEFINED;
		return 0;
	}
	// P0-00 ~ P5-99
	else if( PN < PN_UI_BASE )
	{
		// write
		if( RW == 1 )
		{
			Data = ParamMgr1.WriteParam( &ParamMgr1, TargetID, PN, *pData, pResult );
		} else {
			Data = ParamMgr1.ReadParam( &ParamMgr1, TargetID, PN, pResult );
		}
	}
	// P6-00 ~ P6-99
	else if( PN < PN_SCOPE_BASE )
	{
		if(pResult!=NULL)
			*pResult = PARAM_ACCESS_FAIL_PARAM_ID_NOT_DEFINED;
		return 0;
	}
	// P7-00 ~ P8-99
	else if( PN < FN_BASE )
	{
		if(RW == 1 )
		{
			if(pResult!=NULL)
			{
				*pResult = PARAM_ACCESS_FAIL_WRITE_NOT_SUPPORTED;
			}
		}
		else
		{
			if(pResult!=NULL)
			{
				*pResult = PARAM_ACCESS_SUCCESS;
			}
			Data = drive_GetStatus( TargetID, PN );
		}
	}
	// P9-00 ~ P9-99
	else if( PN < PN_MAX )
	{
		// write
		if( RW == 1 )
		{
			Data = ParamMgr1.WriteFnRegs( &ParamMgr1, PN, *pData, pResult ); // here PN means FN
		} else {
			Data = ParamMgr1.ReadFnRegs( &ParamMgr1, PN, pResult ); // here PN means FN
		}
	}
	else //>p10-00
	{
		//out out range of param ID, reply error
		if(pResult!=NULL)
		{
			*pResult = PARAM_ACCESS_FAIL_PARAM_ID_NOT_DEFINED;
		}
	}

	return Data;
}

void Drive_OnParamValueChanged( uint16_t AxisID, uint16_t PN )
{
	if( AxisID == 0 )
		return;

	int AxisIndex = AXIS_ID_TO_AXIS_INDEX (AxisID );
	Axis[AxisIndex].OnParamValueChanged(&Axis[AxisIndex], PN);
}

void Drive_PcuPowerStateMachine( void )
{
	switch( Axis[0].PcuPowerState )
	{
		case PowerOnOff_Initial:

			// normal transition
			if( IsPcuInitReady == PcuInitState_Ready )
			{
				Axis[0].PcuPowerState = PowerOnOff_Ready;
			}

			// error situation
			if( Axis[0].HasAlarm == ENABLE )
			{
				Axis[0].PcuPowerState = PowerOnOff_ShutdownStart;
			}
			break;

		case PowerOnOff_Ready:

			// normal transition
			if( Axis[0].pCANRxInterface->PcuStateCmd == PcuCmd_Shutdown )
			{
				Axis[0].PcuPowerState = PowerOnOff_ShutdownStart;
			}

			// error situation
			if( Axis[0].HasAlarm == ENABLE )
			{
				Axis[0].PcuPowerState = PowerOnOff_ShutdownStart;
			}
			break;

		case PowerOnOff_ShutdownStart:
			// normal transition
			if( Axis[0].HasAlarm == DISABLE )
			{
				Axis[0].PcuPowerState = PowerOnOff_NormalShutdown;
				GlobalAlarmDetect_ConfigAlarmSystem(); // disable register alarm
			}
			// error situation
			else
			{
				Axis[0].PcuPowerState = PowerOnOff_EmergencyShutDown;
			}
			break;

		case PowerOnOff_NormalShutdown:

			// Restart by external command. (Normal transition)
			if( Axis[0].pCANRxInterface->PcuStateCmd == PcuCmd_Enable )
			{
				// If no alarm exists, change state directly.
				if( Axis[0].HasAlarm == DISABLE )
				{
					Axis[0].PcuPowerState = PowerOnOff_Ready;
					GlobalAlarmDetect_ConfigAlarmSystem(); // enable register alarm
				}
				// If alarm exist, wait for jumping function
				else
				{
					Axis[0].PcuPowerState = PowerOnOff_WaitForReset;
				}
			}

			break;

		case PowerOnOff_EmergencyShutDown:

			// power off after alarm occur
			if( Axis[0].pCANRxInterface->PcuStateCmd == PcuCmd_Shutdown )
			{
				Axis[0].PcuPowerState = PowerOnOff_NormalShutdown;
				GlobalAlarmDetect_ConfigAlarmSystem(); // disable register alarm
			}
			break;

		case PowerOnOff_WaitForReset:
			// Do nothing and wait for JumpCtrlFunction.
			break;

		// abnormal PcuPowerState value
		case PowerOnOff_Error:
		default:
			Axis[0].PcuPowerState = PowerOnOff_Error;
			break;

	}
}

void HAL_TIM_Base_Start_TOTAL_TIME( TIM_HandleTypeDef *htim )
{
	/* clear first update interrupt flag from UG bit which is set by TIM_Base_SetConfig */
	__HAL_TIM_CLEAR_IT( htim, TIM_IT_UPDATE );
	/* Enable the TIM Update interrupt */
	__HAL_TIM_ENABLE_IT( htim, TIM_IT_UPDATE );
	/* Enable the Peripheral */
	__HAL_TIM_ENABLE( htim );
}

void drive_Init(void)
{
	uint16_t AxisIndex;
	uint16_t IoState;
	uint16_t IoState_2;
	// Start timer 2 and avoid first interrupt request.
	HAL_TIM_Base_Start_TOTAL_TIME(&htim2);
	ParamMgr1.OnParamValueChanged = &Drive_OnParamValueChanged;
	IntranetCANStation.AccessParam = &AccessParam;
	IntranetCANStation.pParamMgr = &ParamMgr1;

	// Initialize total time.
	ExtFlash1.pBufferTotalTimeQW = &TotalTime1.BufferTotalTimeQW;
	ExtFlash_Init( &ExtFlash1 );
	TotalTime1.pExtFlash = &ExtFlash1;
	TotalTime1.Init( &TotalTime1 );

	// Read Parameter From IntFlash / ExtFlash ( except for MR)
	// TODO when add motor2 parameters, check if the check sum mechanism is still OK?
	ParamMgr1.Init( &ParamMgr1, &ExtFlash1 );

	// set flag after DriveParam has been updated. and before AlarmDetect init.
	IsUseDigitalFoilSensor = DriveParams.PCUParams.DebugParam1;

	// notice that: parameters in each axis have to be independent.
	for ( AxisIndex = 0; AxisIndex < ACTIVE_AXIS_NUM; AxisIndex++ )
	{
		// Init Axis object. Notice that: initialize axis after reading data from rxt.flash and PSB.
		Axis[AxisIndex].Init ( &Axis[AxisIndex], AxisIndex );

		// AlarmMgr1.init( &AlarmMgr1, &Axis[AxisIndex].HasAlarm, &Axis[AxisIndex].HasWarning, AxisIndex )
		AlarmMgr1.pHasAlarm[AxisIndex] = &Axis[AxisIndex].HasAlarm;
		AlarmMgr1.pHasWarning[AxisIndex] = &Axis[AxisIndex].HasWarning;
		Axis[AxisIndex].RequestResetWarningCNT = 0;
		AlarmMgr1.pRequestResetWarningCNT[AxisIndex] = &Axis[AxisIndex].RequestResetWarningCNT;

	}

	// Init Axis1 Motor Stall Table
	Axis[0].MotorStall.Init( &Axis[0].MotorStall );

	// Init Axis1 Thermal Table
	Axis[0].ThermoStrategy.Init( &Axis[0].ThermoStrategy, &SystemTable.WindingDeratingInfo, &SystemTable.MosDeratingInfo, &SystemTable.CapDeratingInfo, &AdcStation1 );

	// Init Buffer IC (Pull low BUffer enable gpio)
	IoState = HAL_GPIO_ReadPin( BUF_FB_GPIO_Port, BUF_FB_Pin);
	IoState_2 = HAL_GPIO_ReadPin( BUF_ENA_GPIO_Port, BUF_ENA_Pin);
	if( ( IoState == SIGNAL_HIGH ) && ( IoState_2 == SIGNAL_HIGH )  )
	{
		HAL_GPIO_WritePin( BUF_ENA_GPIO_Port, BUF_ENA_Pin, GPIO_PIN_RESET );
		Axis[0].AlarmDetect.BufICEnable = PULL_LOW;
	}
	else
	{
		// Buffer IC Error State
		Axis[0].AlarmDetect.BufICEnable = PULL_HIGH;
	}

	// Init Pwm Station
	PwmStation1.Init( &PwmStation1 );

	// Init ADC from tables
	AdcStation1.Init( &AdcStation1 );
	/*
	 * To calculate the throttle gain after read the external memory data
	 */
#if USE_THROTTLE_CALIBRATION == USE_FUNCTION
	drive_ThrottleGainInit( &DriveParams, &AdcStation1 );
#endif
	drive_DoLoad_DataToAdcGain();
#if USE_VOLTAGE_CALIBRATION == USE_FUNCTION
	drive_DcBusGainInit( &DriveParams, &AdcStation1 );
#endif
	// Init CAN external
	ExtranetCANStation.TxInfo.pAlarmStack = &AlarmStack[0];
	ExtranetCANStation.DriveSetup.LoadParam( &ExtranetCANStation.DriveSetup, &CANModuleConfigExtra, LscCanIdTableExtra );
	ExtranetCANStation.Init( &ExtranetCANStation,&ExtranetInformInSystemTableExample, &hfdcan2);

	// Init CAN internal
	IntranetCANStation.NetWork.DriveSetup.LoadParam ( &IntranetCANStation.NetWork.DriveSetup, &CANModuleConfigIntra, LscCanIdTableIntra );
	IntranetCANStation.Init ( &IntranetCANStation, &hfdcan2, &PcuAuthorityCtrl );

	Drive_BinVersionCompare( AppVersion );
	Drive_BinCheckWordCompare( &AppCheckWord );

	// MCU LED light On
	HAL_GPIO_WritePin( MCU_State_LED_GPIO_Port, MCU_State_LED_Pin, GPIO_PIN_SET );


	IntFlashCtrl.Init ( &IntFlashCtrl );

	ExtranetCANStation.Enable = ENABLE;
	ExtranetCANStation.ForceDisable = DISABLE;
	RCCommCtrl.Init(&RCCommCtrl,&huart5,&hcrc,Axis[0].pCANTxInterface,Axis[0].pCANRxInterface);

	// Register alarm depend on AlarmTableInfo table and error status of each module.
	GlobalAlarmDetect_init();

	// Register ready in the end of Drive_init.
	IsPcuInitReady = PcuInitState_Ready;
}

void drive_DoLoad_DataToAdcGain(void)
{
//  Current Gain  Axis 1
	Axis[0].pAdcStation->AdcExeGain[ ISE_U_A0 ].Uint16Type[ 0 ] = DriveParams.PCUParams.Axis1_Iu_Scale[ 0 ];
	Axis[0].pAdcStation->AdcExeGain[ ISE_U_A0 ].Uint16Type[ 1 ] = DriveParams.PCUParams.Axis1_Iu_Scale[ 1 ];
	Axis[0].pAdcStation->AdcExeGain[ ISE_V_A0 ].Uint16Type[ 0 ] = DriveParams.PCUParams.Axis1_Iv_Scale[ 0 ];
	Axis[0].pAdcStation->AdcExeGain[ ISE_V_A0 ].Uint16Type[ 1 ] = DriveParams.PCUParams.Axis1_Iv_Scale[ 1 ];
	Axis[0].pAdcStation->AdcExeGain[ ISE_W_A0 ].Uint16Type[ 0 ] = DriveParams.PCUParams.Axis1_Iw_Scale[ 0 ];
	Axis[0].pAdcStation->AdcExeGain[ ISE_W_A0 ].Uint16Type[ 1 ] = DriveParams.PCUParams.Axis1_Iw_Scale[ 1 ];

	Axis[0].pAdcStation->AdcExeZeroP[ ISE_U_A0 ] = ( ( DriveParams.PCUParams.Axis1_Iu_ZeroPoint > 2000 ) && \
												 ( DriveParams.PCUParams.Axis1_Iu_ZeroPoint < 2100 ) )? DriveParams.PCUParams.Axis1_Iu_ZeroPoint: 2048;
	Axis[0].pAdcStation->AdcExeZeroP[ ISE_V_A0 ] = ( ( DriveParams.PCUParams.Axis1_Iv_ZeroPoint > 2000 ) && \
												 ( DriveParams.PCUParams.Axis1_Iv_ZeroPoint < 2100 ) )? DriveParams.PCUParams.Axis1_Iv_ZeroPoint: 2048;
	Axis[0].pAdcStation->AdcExeZeroP[ ISE_W_A0 ] = ( ( DriveParams.PCUParams.Axis1_Iw_ZeroPoint > 2000 ) && \
												 ( DriveParams.PCUParams.Axis1_Iw_ZeroPoint < 2100 ) )? DriveParams.PCUParams.Axis1_Iw_ZeroPoint: 2048;
	Axis[0].pAdcStation->AdcExeZeroP[ BAT_VDC ]  = 0;

	Axis[0].pAdcStation->AdcExeGain[ ISE_U_A0 ].FDta = ( ( Axis[ 0 ].pAdcStation->AdcExeGain[ ISE_U_A0 ].FDta > 0.0f ) && \
													 ( Axis[ 0 ].pAdcStation->AdcExeGain[ ISE_U_A0 ].FDta < 0.35f ) )? Axis[ 0 ].pAdcStation->AdcExeGain[ ISE_U_A0 ].FDta: Axis[ 0 ].pAdcStation->InjCh[ ISE_U_A0 ].GainValue;
	Axis[0].pAdcStation->AdcExeGain[ ISE_V_A0 ].FDta = ( ( Axis[ 0 ].pAdcStation->AdcExeGain[ ISE_V_A0 ].FDta > 0.0f ) && \
													 ( Axis[ 0 ].pAdcStation->AdcExeGain[ ISE_V_A0 ].FDta < 0.35f ) )? Axis[ 0 ].pAdcStation->AdcExeGain[ ISE_V_A0 ].FDta: Axis[ 0 ].pAdcStation->InjCh[ ISE_V_A0 ].GainValue;
	Axis[0].pAdcStation->AdcExeGain[ ISE_W_A0 ].FDta = ( ( Axis[ 0 ].pAdcStation->AdcExeGain[ ISE_W_A0 ].FDta > 0.0f ) && \
													 ( Axis[ 0 ].pAdcStation->AdcExeGain[ ISE_W_A0 ].FDta < 0.35f ) )? Axis[ 0 ].pAdcStation->AdcExeGain[ ISE_W_A0 ].FDta: Axis[ 0 ].pAdcStation->InjCh[ ISE_W_A0 ].GainValue;
#if USE_VOLTAGE_CALIBRATION == UNUSE_FUNCTION
	Axis[0].pAdcStation->AdcExeGain[ BAT_VDC ].FDta  = ( Axis[ 0 ].pAdcStation->AdcExeGain[ BAT_VDC ].FDta > 0 )? Axis[ 0 ].pAdcStation->AdcExeGain[ BAT_VDC ].FDta: Axis[ 0 ].pAdcStation->InjCh[ BAT_VDC ].GainValue;
#endif
	Axis[ 0 ].pAdcStation->ZeroCalibInjDone = ( Axis[ 0 ].pAdcStation->ZeroCalibInjChCount == 0 )? ADC_DONE:ADC_NONE;

//  Throttle Gain and Zero Point load
	if( Axis[0].ThrottleGainState == GAIN_STATE_NORMAL )
	{
		Axis[ 0 ].pAdcStation->AdcExeThrotGain.FDta = Axis[ 0 ].ThrottleGain;
	}else;
	Axis[ 0 ].pAdcStation->AdcExeThrotZero = ( DriveParams.SystemParams.ThrottleMinAdc > 0 )? DriveParams.SystemParams.ThrottleMinAdc: 0;
	Axis[ 0 ].pAdcStation->AdcExeThrotMax = ( DriveParams.SystemParams.ThrottleMaxAdc > 2048 )? DriveParams.SystemParams.ThrottleMaxAdc: 4095;

}

#if USE_THROTTLE_CALIBRATION == USE_FUNCTION
void drive_ThrottleGainInit( DriveParams_t *d, AdcStation *a )
{
	if( ( d->SystemParams.ThrottleMaxAdc == 2300 ) && (d->SystemParams.ThrottleMinAdc == 130) )
	{
		Axis[0].ThrottleGainState = GAIN_STATE_EMPTY;
		Axis[0].ThrottleGain = GAIN_STATE_EMPTY;
	}
	else if( d->SystemParams.ThrottleMaxAdc < d->SystemParams.ThrottleMinAdc )
	{
		Axis[0].ThrottleGainState = GAIN_STATE_ABNORMAL;
		Axis[0].ThrottleGain = GAIN_STATE_EMPTY;
	}
	else
	{
		Axis[0].ThrottleGainState = GAIN_STATE_NORMAL;
		Axis[0].ThrottleGain = ( float )( 1.0f /( float )( d->SystemParams.ThrottleMaxAdc - d->SystemParams.ThrottleMinAdc ) );
	}
}
#endif

#if USE_VOLTAGE_CALIBRATION == USE_FUNCTION
void drive_DcBusGainInit( DriveParams_t *d, AdcStation *a )
{
	if( ( d->PCUParams.DcBusCalibValue[0] == 0 ) && ( d->PCUParams.DcBusCalibValue[1] == 0) )	//0: Minimum value, 1: Maximum value
	{
		Axis[0].DcBusGainState = GAIN_STATE_EMPTY;
		a->AdcExeGain[BAT_VDC].FDta = a->InjCh[ BAT_VDC ].GainValue;
	}
	else if( d->PCUParams.DcBusCalibValue[1]< d->PCUParams.DcBusCalibValue[0] )					//0: Minimum value, 1: Maximum value
	{
		Axis[0].DcBusGainState = GAIN_STATE_ABNORMAL;
		a->AdcExeGain[BAT_VDC].FDta = a->InjCh[ BAT_VDC ].GainValue;
	}
	else
	{
		Axis[0].DcBusGainState = GAIN_STATE_NORMAL;
		a->AdcExeGain[BAT_VDC].FDta = ( float )( (d->SystemParams.BattVoltMax- d->SystemParams.BattVoltMin )/10.0f/( float )( d->PCUParams.DcBusCalibValue[1] - d->PCUParams.DcBusCalibValue[0] ) );
	}
}
#endif

#if USE_PWM_RC_FUNCTION == (USE_FUNCTION & EVT)
void drive_DoPwmRcCatch(void)
{
	uint32_t PeriodCntTmp = 0;
	uint32_t UlTemp32=0;
	float PeriodCnt = 0.0F;
	float DutyCnt = 0.0F;
//	uint64_t PeriodTmpU64 = 0;

	if( htim5.Channel == HAL_TIM_ACTIVE_CHANNEL_2 )
	{
		/* Get the Input Capture value - period count */
		Axis[0].PwmRc.PeriodEdgeValue = HAL_TIM_ReadCapturedValue( &htim5, TIM_CHANNEL_2 );
		Axis[0].PwmRc.DutyEdgeValue = HAL_TIM_ReadCapturedValue( &htim5, TIM_CHANNEL_1 );

//		PeriodCntTmp = ((Axis[0].PwmRc.PeriodEdgeValue- Axis[0].PwmRc.PrevPeriodEdgeValue)&0x00007FFF);


		if( Axis[0].PwmRc.PeriodEdgeValue >= Axis[0].PwmRc.PrevPeriodEdgeValue ){
			PeriodCntTmp = Axis[0].PwmRc.PeriodEdgeValue - Axis[0].PwmRc.PrevPeriodEdgeValue;
		}else{
			PeriodCntTmp  = Axis[0].PwmRc.PeriodEdgeValue + 0xFFFF;
			PeriodCntTmp -= Axis[0].PwmRc.PrevPeriodEdgeValue;
		}

//		if( ( PeriodCntTmp < 10002 ) & ( PeriodCntTmp> 9800 ) )
		if( ( PeriodCntTmp < 10200 ) & ( PeriodCntTmp> 9800 ) )
 		{
			PeriodCnt = (float)PeriodCntTmp;
			//DutyCnt = ( float )((Axis[0].PwmRc.DutyEdgeValue-Axis[0].PwmRc.PrevPeriodEdgeValue)&0x00007FFF);

			if( Axis[0].PwmRc.DutyEdgeValue >= Axis[0].PwmRc.PrevPeriodEdgeValue ){
				UlTemp32 = Axis[0].PwmRc.DutyEdgeValue - Axis[0].PwmRc.PrevPeriodEdgeValue;
			}else{
				UlTemp32 = Axis[0].PwmRc.DutyEdgeValue + 0xFFFF ;
				UlTemp32 -= Axis[0].PwmRc.PrevPeriodEdgeValue;
			}

			if(UlTemp32 < 2500){
				DutyCnt = (float)UlTemp32;
				Axis[0].PwmRc.DutyRaw = DutyCnt / PeriodCnt;
			}
		}
	}
	Axis[0].PwmRc.PrevPeriodEdgeValue = Axis[0].PwmRc.PeriodEdgeValue;
	Axis[0].PwmRc.AbnormalCnt = DISABLE;
	Axis[0].PwmRc.StartFlag = ENABLE;
}
#endif

void drive_DoCurrentLoop(void)
{
	int i;

	AdcStation1.DoCurrentLoop( &AdcStation1 );
#if USE_CALC_SUM_ROOT==USE_FUNCTION
	MFStation1.CalSumRoot( &MFStation1, &AdcStation1, PcuAuthorityCtrl.SecureLvNow );
#endif

	//TODO add "GlobalAlarmDetect_Accumulation" here.
	for( i = 0; i < ACTIVE_AXIS_NUM; i++ )
	{
		// Update Control Feedback
		Axis[i].MotorControl.SensorFb.Iu = AdcStation1.AdcTraOut.Iu[i];
		Axis[i].MotorControl.SensorFb.Iv = AdcStation1.AdcTraOut.Iv[i];
		Axis[i].MotorControl.SensorFb.Iw = AdcStation1.AdcTraOut.Iw[i];
		Axis[i].MotorControl.SensorFb.Vbus = AdcStation1.AdcTraOut.BatVdc;

		// Do Current Loop Tasks
		Axis[i].DoCurrentLoop(&Axis[i]);
	}
}

// 1kHz
void drive_DoPLCLoop(void)
{
	int i;
	AdcStation1.DoPLCLoop( &AdcStation1 );

	//TODO add "GlobalAlarmDetect_Accumulation" here.
	for( i = 0; i < ACTIVE_AXIS_NUM; i++ )
	{
		//Fernando test start
		//UartBuffer.Rx(&UartBuffer,&CtrlUi);
		//Fernando test end
		Axis[i].DoPLCLoop(&Axis[i]);
	}

	switch ( ParamMgr1.Session )
	{
	case Session_0x01_Default:
		break;
	case Session_0x02_Programming:
		if( PcuAuthorityCtrl.SecureLvNow > Authority_VehicleDealer )
		{
			BootAppTrig = BOOT_ENA;
		}
		break;
	case Session_0x03_ExtendedDiagnostic:
		break;
	case Session_0x04_SafetySystemDiagnostic:
		break;
	case Session_0x40_VehicleManufacturerSpecific:
		break;
	case Session_0x60_SystemSupplierSpecific:
		MFStation1.CalMaxAvgCnt( &MFStation1, DriveFnRegs[ FN_OPEN_SPD_COMMAND - FN_BASE ], PcuAuthorityCtrl.SecureLvNow  );
//		MFStation1.GpioMfinfo( &MFStation1, PcuAuthorityCtrl.SecureLvNow  );
		MFStation1.RMS( &MFStation1, PcuAuthorityCtrl.SecureLvNow  );
#if USE_CURRENT_CALIBRATION
		MFStation1.CalibCurrent.RDModeAndAuth( &MFStation1.CalibCurrent, DriveFnRegs[ FN_MF_FUNC_SEL - FN_BASE ], PcuAuthorityCtrl.SecureLvNow );
		MFStation1.CalibCurrent.ReadCmdInfo( &MFStation1.CalibCurrent, DriveFnRegs[ FN_MF_CURR_CALIB_SETUP - FN_BASE ], &DriveFnRegs[ FN_MF_CURR_CALIB_START - FN_BASE ] );
		MFStation1.CalibCurrent.RCurrentAdc( &MFStation1.CalibCurrent, &AdcStation1, &DriveFnRegs[ FN_MF_CURR_CALIB_START - FN_BASE ] );
		MFStation1.CalibCurrent.CalibGainandZPoint( &MFStation1.CalibCurrent, &DriveFnRegs[ FN_MF_CURR_CALC - FN_BASE ] );
#endif
#if USE_VOLTAGE_CALIBRATION
		MFStation1.CalibDcBusVoltage.TempVal = MFStation1.CalibDcBusVoltage.RVoltAdc( &MFStation1.CalibDcBusVoltage, &AdcStation1, &DriveFnRegs[ FN_MF_VOLT_CALIB_START - FN_BASE ] );
		if( MFStation1.CalibDcBusVoltage.TempVal> 0 )
		{
			DriveParams.PCUParams.DcBusCalibValue[ MFStation1.CalibDcBusVoltage.Calib_StartFlag ] = MFStation1.CalibDcBusVoltage.TempVal;
			MFStation1.CalibDcBusVoltage.Calib_StartFlag = DISABLE;
		}
#endif
		break;
	default:
		break;
	}
	ExtranetCANStation.DisableRst( &ExtranetCANStation );
	if ( ExtranetCANStation.Enable )
	{
		ExtranetCANStation.DoPlcLoop ( &ExtranetCANStation );
	}
	IntranetCANStation.DoPlcLoop( &IntranetCANStation );

	Drive_PcuPowerStateMachine(); // note: Tx response will delay one PLCLoop(1ms)


}

void drive_Do100HzLoop(void)
{
	int i;
	AdcStation1.Do100HzLoop( &AdcStation1 );
	Axis[0].MotorControl.Sensorless.EEMF.WindingTemp = AdcStation1.AdcTraOut.MOTOR_NTC;
	for( i = 0; i < ACTIVE_AXIS_NUM; i++ )
	{
		Axis[i].Do100HzLoop(&Axis[i]);
	}
	MFStation1.GpioMfinfo( &MFStation1 );

}

void drive_Do10HzLoop(void)
{
	int i;
	for( i = 0; i < ACTIVE_AXIS_NUM; i++ )
	{
		Axis[i].Do10HzLoop(&Axis[i]);

	}
	RCCommCtrl._10HzLoop(&RCCommCtrl);
}

void drive_DoTotalTime(void)
{
	// When TIM2 ARPE = 0, ARR change directly. The setting influence the end of THIS CNT.
	if( TotalTime1.BufferServoOnState == MOTOR_STATE_OFF)
	{
		TIM2->ARR = (15000-1); // 3 sec
	}
	else
	{
		TIM2->ARR = (150000-1); // 30 sec
	}

	// In THIS interrupt, time elapsed should consider last servo on/off state.
	if( TotalTime1.NowServoOnState == MOTOR_STATE_OFF)
	{
		TotalTime1.Do3secLoop( &TotalTime1 );
		TotalTime1.NowServoOnState = TotalTime1.BufferServoOnState;
	}
	else
	{
		TotalTime1.Do30secLoop( &TotalTime1 );
		TotalTime1.NowServoOnState = TotalTime1.BufferServoOnState;
	}
	TotalTime1.RecTotalTime( &TotalTime1 );
}

void drive_Do1HzLoop(void)
{
	// do nothing
}

void Drive_ResetWarningCNTandStatus(Axis_t *v, AlarmMgr_t *pAlarmMgr)
{
	// Reset All warning counter before reset alarm stack and clear hasWarning.
	// Reset WarningCNT  todo create a new function in alarmDetect
	v->AlarmDetect.CAN1Timeout.Counter = 0;
	v->AlarmDetect.FOIL_SENSOR_BREAK.Counter = 0;
	v->AlarmDetect.FOIL_SENSOR_SHORT.Counter = 0;
	v->AlarmDetect.BREAK_NTC_PCU_0.Counter = 0;
	v->AlarmDetect.BREAK_NTC_PCU_1.Counter = 0;
	v->AlarmDetect.BREAK_NTC_PCU_2.Counter = 0;
	v->AlarmDetect.BREAK_NTC_Motor_0.Counter = 0;
	v->AlarmDetect.SHORT_NTC_PCU_0.Counter = 0;
	v->AlarmDetect.SHORT_NTC_PCU_1.Counter = 0;
	v->AlarmDetect.SHORT_NTC_PCU_2.Counter = 0;
	v->AlarmDetect.SHORT_NTC_Motor_0.Counter = 0;
	v->AlarmDetect.OTP_PCU_0_WARNING.Counter = 0;
	v->AlarmDetect.OTP_PCU_1_WARNING.Counter = 0;
	v->AlarmDetect.OTP_PCU_2_WARNING.Counter = 0;
	v->AlarmDetect.OTP_Motor_0_WARNING.Counter = 0;
	v->AlarmDetect.RC_INVALID.Counter = 0;


	// Only if PCU is servo off, then PCU can reset warning.
	pAlarmMgr->ResetAllWarning( pAlarmMgr );
	v->RequestResetWarningCNT = 0;
}

void drive_DoHouseKeeping(void)
{
	uint16_t lIdxTemp = 0;
	//do active code operation
	uint8_t InfFlashOpBuffer[20];

	//latchTotalTimeservoonstate,toavoidAxis0.ServoOnchangebyotherIRQ.
	TotalTime1.BufferServoOnState=Axis[0].ServoOn;
	// PCU SN OPERATION
	if( DriveFnRegs[ FN_PCU_SN_OPERATION-FN_BASE ] == 1 )
	{
		for ( lIdxTemp = 0; lIdxTemp < 9; lIdxTemp++ )
		{
			DriveParams.PCUParams.PCUSNCode[ lIdxTemp ] = DriveFnRegs[ FN_DATA_BUFFER_0 - FN_BASE + lIdxTemp ];
		}
		DriveFnRegs[ FN_PARAM_BACKUP_EMEMORY - FN_BASE ] = 1;
	}
	DriveFnRegs[ FN_PCU_SN_OPERATION - FN_BASE ]=0;

	// Do param saving at servo-off only

	ExtFlash1.ParamBackupRequest = DriveFnRegs[FN_PARAM_BACKUP_EMEMORY - FN_BASE];

	if( ( ExtFlash1.ParamBackupRequest == Target_EFlash ) && ( Axis[0].ServoOn == 0 ) )
	{
		// Param Backup
		ExtFlash1.ParamBackup( &ExtFlash1, &DriveParams );

		ExtFlash1.ParamBackupRequest = 0;
		DriveFnRegs[FN_PARAM_BACKUP_EMEMORY - FN_BASE] = 0;
	}

	IntFlashCtrl.IDSectionErase( &IntFlashCtrl, &DriveFnRegs[FN_FLASH_ID_SECTION_EREASE - FN_BASE], PcuAuthorityCtrl.SecureLvNow );

	Axis[0].pCANTxInterface->DebugU8[IDX_LOG_ENABLE_FLAG] =(uint8_t)( 0xFF & DriveParams.PCUParams.DebugParam10 );
	Axis[0].pCANTxInterface->DebugU8[IDX_BMS_COMM_ENABLE] =(uint8_t)( 0xFF & DriveParams.PCUParams.DebugParam9 );

	/*
	 * Ext-Flash Table Reset switch the data into initial value
	 */
	drive_DoExtFlashTableRst( &DriveFnRegs[FN_EXTFLASH_DATA_RST_SET - FN_BASE], \
			                  &DriveFnRegs[FN_EXTFLASH_DATA_RST_ENA - FN_BASE], \
							  &DriveFnRegs[FN_PARAM_BACKUP_EMEMORY - FN_BASE], \
							  &SystemTable, \
							  &DriveParams.SystemParams, \
							  &PCUTable, \
							  &DriveParams.PCUParams);


	// Check error flag in housekeeping every time. If error flag is on and alarm is enable, register alarm "again".
	GlobalAlarmDetect_DoHouseKeeping();

	//RCCommCtrl.MsgHandler(&RCCommCtrl,RCCommCtrl.RxBuff,Axis[0].pCANTxInterface,Axis[0].pCANRxInterface);
	RCCommCtrl.MsgDecoder(&RCCommCtrl);

	// If status is servo off, warning exist, and user commands servo on again.
	if( Axis[0].RequestResetWarningCNT == 2 &&  Axis[0].ServoOn == 0)
	{
		// Reset All warning counter before reset alarm stack and clear hasWarning.
		// Reset WarningCNT  todo Create a new function in alarmDetect.
		// Now Drive simulate Axis to reset local CNT.
		Drive_ResetWarningCNTandStatus( &Axis[0], &AlarmMgr1 );
	}
}

void drive_DoExtFlashTableRst( uint32_t *Setup, uint32_t *Ena, uint32_t *BackUpExMemEna, const System_Table_t_Linker *Ts, SystemParams_t *pSysT, const PCU_Table_t_Linker *Tp, PCUParams_t *pPcuT )
{
	uint16_t tCnt = 0;
//	uint16_t IdxCnt = 0;

	if( *Setup > PcuAuthorityCtrl.SecureLvNow )
	{
		*Setup = PcuAuthorityCtrl.SecureLvNow;
	}

	if( ( *Setup > Authority_EndUser ) && ( *Ena > 0 ) )
	{
		if( *Ena == 1) // Reset the System Table
		{
			for( tCnt = 0; tCnt< SYS_PARAM_SIZE; tCnt++)
			{
				if( ( Ts->SysParamTableInfoArray[tCnt].Property & 0x0007 ) <= PcuAuthorityCtrl.SecureLvNow)
				{
					*((&pSysT->Reserved000) + tCnt) =  Ts->SysParamTableInfoArray[tCnt].Default;
				} else;
			}
			*BackUpExMemEna = ENABLE;
			*Setup = DISABLE;
			*Ena = DISABLE;
		}
		else if( *Ena == 2 ) // Reset the PCUSystem Table
		{
			for( tCnt = 0; tCnt< PCU_PARAM_SIZE; tCnt++)
			{
				if( ( Tp->PcuParamTableInfoArray[tCnt].Property & 0x0007 ) <= PcuAuthorityCtrl.SecureLvNow)
				{
					*((&pPcuT->UartBaudrateSelect) + tCnt) =  Tp->PcuParamTableInfoArray[tCnt].Default;
				} else;
			}
			*BackUpExMemEna = ENABLE;
			*Setup = DISABLE;
			*Ena = DISABLE;
		} else; // Do nothing
	} else;
}

uint16_t Drive_BinVersionCompare( const uint16_t *dta)
{
	uint16_t Rtn;
	return Rtn = 0;
}

uint16_t Drive_BinCheckWordCompare( const uint16_t *dta)
{
	uint16_t Rtn;
	return Rtn = 0;
}

/*
 * Application Jump to Boot-loader
 */
void JumpCtrlFunction( void )
{
	_Static_assert ( ACTIVE_AXIS_NUM == 1 , "Conditions of jump function should take Axis[i].ServoOn in consideration");
	if( BootAppTrig == BOOT_ENA )
	{
		if( ( Axis[0].ServoOn == MOTOR_STATE_OFF ) )
		{
			HAL_Delay(5);
			JumpCode_ApplicationToBootloader( BOOT_ADDRESS );
		}
		else
		{
			BootAppTrig = BOOT_DIS;
		}
	}
	else if( ( DriveFnRegs[ FN_PCU_RESET_OPERATION - FN_BASE ] == BOOT_ENA ) &&	\
			 ( PcuAuthorityCtrl.SecureLvNow > Authority_VehicleDealer ) &&				\
			 ( Axis[0].ServoOn == MOTOR_STATE_OFF ) )
	{
		HAL_Delay(100);
		DriveFnRegs[FN_PCU_RESET_OPERATION - FN_BASE] = 0;
		JumpCode_ApplicationToBootloader( BOOT_ADDRESS );
	}
	else if( (Axis[0].PcuPowerState == PowerOnOff_WaitForReset) && \
			 (Axis[0].ServoOn == MOTOR_STATE_OFF) )
	{
		HAL_Delay(100);
		 JumpCode_ApplicationToBootloader( BOOT_ADDRESS );
	 }
	else;
}

void JumpCode_ApplicationToBootloader( uint32_t BootAddress )
{
//	HAL_NVIC_DisableIRQ();
	/*
	 * Setup Jump address
	 */
	Jump2app = (void (*)())*(__IO uint32_t*) (BootAddress + 4);

	/*
	 *  Disable Module
	 */
	DisableMcuModule();

    /*
     * DeInit RCC & HAL
     */
	HAL_RCC_DeInit();
	HAL_DeInit();
	HAL_MspInit();

	/* Initialize user application's Stack Pointer */
	__set_MSP(*(__IO uint32_t*) BootAddress);
	Jump2app();
}

void DisableMcuModule( void )
{
	/*
	 * DeInit All Communication Function
	 */
	// CAN 1
	HAL_FDCAN_DeactivateNotification(&hfdcan1, FDCAN_IT_RX_FIFO0_NEW_MESSAGE);
	HAL_FDCAN_MspDeInit(&hfdcan1);

	// CAN 2
	HAL_FDCAN_DeactivateNotification(&hfdcan2, FDCAN_IT_RX_FIFO0_NEW_MESSAGE);
	HAL_FDCAN_MspDeInit(&hfdcan2);
	__HAL_RCC_FDCAN_FORCE_RESET();

	//UART
//	HAL_UART_MspDeInit(&huart5);
//	__HAL_RCC_UART5_FORCE_RESET();

	//USART
//	HAL_UART_MspDeInit(&huart3);
//	__HAL_RCC_USART3_FORCE_RESET();

	//USART
//	HAL_USART_MspDeInit(&husart2);
//	__HAL_RCC_USART2_FORCE_RESET();

	//SPI
	HAL_SPI_MspDeInit(&hspi1);
	__HAL_RCC_SPI1_FORCE_RESET();

	/*
	 * DeInit GPIO State/ Flag/ IT
	 */
	__HAL_RCC_GPIOA_CLK_DISABLE();

	__HAL_RCC_GPIOB_CLK_DISABLE();

	__HAL_RCC_GPIOC_CLK_DISABLE();

	__HAL_RCC_GPIOD_CLK_DISABLE();

	__HAL_RCC_GPIOE_CLK_DISABLE();

	__HAL_RCC_GPIOF_CLK_DISABLE();

	__HAL_RCC_GPIOG_CLK_DISABLE();

    /*
     * DeInit Timer
     */

	// Timer 2
    HAL_TIM_IC_Stop_IT(&htim2, TIM_CHANNEL_ALL);
    __HAL_TIM_DISABLE_IT( &htim2, TIM_IT_IDX );
    HAL_TIM_Base_DeInit( &htim2 );
    __HAL_RCC_TIM2_FORCE_RESET();

	// Timer 3
//    HAL_TIM_Base_Stop_IT( &htim3 );
//	__HAL_TIM_DISABLE_IT( &htim3, TIM_IT_IDX );
//    HAL_TIM_Base_MspDeInit( &htim3 );
//	__HAL_RCC_TIM3_FORCE_RESET();

	// Timer 4
//    HAL_TIM_Base_Stop_IT( &htim4 );
//    HAL_TIM_Base_MspDeInit( &htim4 );
//    __HAL_RCC_TIM4_FORCE_RESET();

	// Timer 5
#if BME&EVT
    HAL_TIM_IC_Stop_IT(&htim5, TIM_CHANNEL_ALL);
    __HAL_TIM_DISABLE_IT( &htim5, TIM_IT_IDX );
    HAL_TIM_Base_DeInit( &htim5 );
    __HAL_RCC_TIM2_FORCE_RESET();
#endif
    // Timer 6
    HAL_TIM_Base_Stop_IT( &htim6 );
    HAL_TIM_Base_MspDeInit( &htim6 );
    __HAL_RCC_TIM6_FORCE_RESET();

    // Timer 7
    HAL_TIM_Base_Stop_IT( &htim7 );
    HAL_TIM_Base_MspDeInit( &htim7 );
    __HAL_RCC_TIM7_FORCE_RESET();

    // Timer 16
    HAL_TIM_Base_Stop_IT( &htim16 );
    HAL_TIM_Base_MspDeInit( &htim16 );
    __HAL_RCC_TIM16_FORCE_RESET();

	// Timer 20
	HAL_TIM_Base_Stop_IT( &htim20 );
	HAL_TIM_Base_MspDeInit( &htim20 );
	__HAL_RCC_TIM20_FORCE_RESET();

	/*
	 * DeInit ADC & DAC
	 */
	//ADC
	HAL_ADCEx_InjectedStop_IT(&hadc1);
	HAL_ADC_MspDeInit(&hadc1);

	// ADC 2
	HAL_ADCEx_InjectedStop_IT(&hadc2);
	HAL_ADC_MspDeInit(&hadc2);
	__HAL_RCC_ADC12_FORCE_RESET();

	// ADC 3
	HAL_ADCEx_InjectedStop_IT(&hadc3);
	HAL_ADC_MspDeInit(&hadc3);
	__HAL_RCC_ADC345_FORCE_RESET();

	//DAC
	HAL_DAC_MspDeInit(&hdac1);
	__HAL_RCC_DAC1_FORCE_RESET();

	/*
	 * Other Functions
	 */
	HAL_CORDIC_MspInit(&hcordic);
	__HAL_RCC_CORDIC_FORCE_RESET();
}
