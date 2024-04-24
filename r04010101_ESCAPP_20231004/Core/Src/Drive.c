/*
 * Drive.c
 *
 *  Created on: Dec 19, 2019
 *      Author: MikeSFWen
 */

#include "UtilityBase.h"
#include "UiApp.h"
#include "Drive.h"

/*
 * Header Information
 * Note: AppVersion, BomNumber and SystemSupplyerID are put in the ".AppVerSpace", those order cannot be changed.
 */
__attribute__((__section__(".AppVerSpace"),used)) const uint16_t AppVersion[VERSION_CODE_NUMBER] = APP_VERSION;
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
PS_t PSStation1 = PS_DEFAULT;

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

// Declare Remaining time
RemainingTime_t RemainingTime1 = REMAININGTIME_DEFAULT;

int32_t AccessParam( uint16_t TargetID, uint16_t Index, int32_t *pData, uint16_t RW , uint8_t *pResult);
INV_OP_STATE_e INVMainState = INV_OP_INITIALIZING;
VEHICLE_STATE_e VehicleMainState = VEHICLE_STATE_INITIALIZING;
uint16_t DualBtnTimeCnt = 0;
uint8_t ButtonReleasedFlags = 0;
/*For BRP UDS implementation*/

EnumUdsBRPNRC drive_RDBI_Function (UdsDIDParameter_e DID, LinkLayerCtrlUnit_t *pRx, LinkLayerCtrlUnit_t *pTx);
__STATIC_FORCEINLINE EnumUdsBRPNRC drive_RDBI_CopyF32toTx( LinkLayerCtrlUnit_t *pRx, LinkLayerCtrlUnit_t *pTx, float input );
__STATIC_FORCEINLINE void drive_DTC_Pickup_Freeze_Frame_data( DTCStation_t *v, uint8_t DTC_Record_Number );
__STATIC_FORCEINLINE void drive_DTC_Pickup_Data_to_Store( AlarmStack_t *AlarmStack, DTCStation_t *v );

DTCStation_t DTCStation1 = DTC_STATION_DEFFAULT;
void Drive_OnParamValueChanged( uint16_t AxisID, uint16_t PN );
extern const CANProtocol ExtranetInformInSystemTableExample;

#if JUDGE_FUNCTION_DELAY
extern Judge_Delay TIM8INT_Judge_Delay;
extern Judge_Delay CurrentLoop_Judge_Delay;
extern Judge_Delay PLCLoop_Judge_Delay;
extern Judge_Delay _100HzLoop_Judge_Delay;
#endif
#if MEASURE_CPU_LOAD
extern float Max_100Hz_Load_pct;
extern float Max_PLCLoop_Load_pct;
extern float Max_CurrentLoop_Load_pct;
extern float Ave_100Hz_Load_pct;
extern float Ave_PLCLoop_Load_pct;
extern float Ave_CurrentLoop_Load_pct;
#endif
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
		RetValue = (int32_t)( AdcStation1.AdcTraOut.MOTOR_NTC_0 * 10.0f );
		break;

	case DN_PCU_NTC_0_TEMP:
		RetValue = (int32_t)( AdcStation1.AdcTraOut.PCU_NTC[MOS_NTC_1] * 10.0f );
		break;

	case DN_PCU_NTC_1_TEMP:
		RetValue = (int32_t)( AdcStation1.AdcTraOut.PCU_NTC[MOS_NTC_2] * 10.0f );
		break;

	case DN_GATE_DRIVE_VOLT:	// 13V VOltage
		RetValue = (int32_t)( AdcStation1.AdcTraOut.S13V8 * 100.0f );
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
		RetValue = (int32_t)( AdcStation1.AdcTraOut.PCU_NTC[CAP_NTC] * 10.0f );
		break;

	case DN_CURRENT_LIMIT :
		RetValue = (int32_t)(Axis[0].pCANRxInterface->BatCurrentDrainLimit * 10.0f);
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
					Axis[AxisIndex].HasCriAlarm						<< 4 |
					Axis[AxisIndex].pAdcStation->ZeroCalibInjDone	<< 5 |
					Axis[AxisIndex].PhaseLoss.Enable				<< 6 |
					Axis[AxisIndex].HasNonCriAlarm					<< 7 ;
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
	case DN_ACC_CAN_ERROR_CNT :
		RetValue = Axis[0].pCANRxInterface->AccCANErrorCnt;
		break;
#if JUDGE_FUNCTION_DELAY
    case DN_MAX_TIM8INT_INTERVAL   :
		RetValue = (uint16_t)(TIM8INT_Judge_Delay.Max_Intervals_us * 100.0f);
		break;
    case DN_AVE_TIM8INT_INTERVAL   :
		RetValue = (uint16_t)(TIM8INT_Judge_Delay.Ave_Intervals_us * 100.0f);
		break;
    case DN_MAX_CURRENTLOOP_INTERVAL:
		RetValue = (uint16_t)(CurrentLoop_Judge_Delay.Max_Intervals_us * 100.0f);
		break;
    case DN_AVE_CURRENTLOOP_INTERVAL:
		RetValue = (uint16_t)(CurrentLoop_Judge_Delay.Ave_Intervals_us * 100.0f);
		break;
    case DN_MAX_PLCLOOP_INTERVAL    :
		RetValue = (uint16_t)(PLCLoop_Judge_Delay.Max_Intervals_us * 10.0f);
		break;
    case DN_AVE_PLCLOOP_INTERVAL    :
		RetValue = (uint16_t)(PLCLoop_Judge_Delay.Ave_Intervals_us * 10.0f);
		break;
    case DN_MAX_100HZLOOP_INTERVAL  :
		RetValue = (uint16_t)(_100HzLoop_Judge_Delay.Max_Intervals_us);
		break;
    case DN_AVE_100HZLOOP_INTERVAL  :
		RetValue = (uint16_t)(_100HzLoop_Judge_Delay.Ave_Intervals_us);
		break;
#endif
#if MEASURE_CPU_LOAD
    case DN_MAX_100HZLOOP_LOAD_PCT  :
		RetValue = (uint16_t)(Max_100Hz_Load_pct * 100.0f);
		break;
    case DN_MAX_PLCLOOP_LOAD_PCT    :
		RetValue = (uint16_t)(Max_PLCLoop_Load_pct * 100.0f);
		break;
    case DN_MAX_CURRENTLOOP_LOAD_PCT:
		RetValue = (uint16_t)(Max_CurrentLoop_Load_pct * 100.0f);
		break;
    case DN_AVE_100HZLOOP_LOAD_PCT  :
		RetValue = (uint16_t)(Ave_100Hz_Load_pct * 100.0f);
		break;
    case DN_AVE_PLCLOOP_LOAD_PCT    :
		RetValue = (uint16_t)(Ave_PLCLoop_Load_pct * 100.0f);
		break;
    case DN_AVE_CURRENTLOOP_LOAD_PCT:
		RetValue = (uint16_t)(Ave_CurrentLoop_Load_pct * 100.0f);
		break;
#endif
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
EnumUdsBRPNRC drive_RDBI_Function (UdsDIDParameter_e DID, LinkLayerCtrlUnit_t *pRx, LinkLayerCtrlUnit_t *pTx )
{
	EnumUdsBRPNRC tempRsp = NRC_0x10_GR;
    switch ( DID )
    {
//        case DID_0xF190_Vehicle_Identification_Number:
//        {
//        	tempRsp = drive_RDBI_From_IntFlash( &IntFlashCtrl, DID_0xF190_Vehicle_Identification_Number, pRx, pTx );
//        	break;
//        }
        case DID_0xF180_Boot_Version:
        {
    	    pTx->Data[0] = pRx->Data[0] + POSITIVE_RESPONSE_OFFSET;
    	    pTx->Data[1] = pRx->Data[1];
    	    pTx->Data[2] = pRx->Data[2];
    	    memcpy( &(pTx->Data[3]),  (uint32_t*)BOOT_VER_ADDRESS, 4 );
    	    pTx->LengthTotal = 7;
	    tempRsp = NRC_0x00_PR;
        	break;
        }
//        case DID_0xF185_Fingerprint:
//        {
//        	tempRsp = drive_RDBI_From_IntFlash( &IntFlashCtrl, DID_0xF185_Fingerprint, pRx, pTx );
//        	break;
//        }
        case DID_0xF186_Active_Diagnostic_Session:
        {
    	    pTx->Data[0] = pRx->Data[0] + POSITIVE_RESPONSE_OFFSET;
    	    pTx->Data[1] = pRx->Data[1];
    	    pTx->Data[2] = pRx->Data[2];
    	    pTx->Data[3] = IntranetCANStation.ServiceCtrlBRP.DiagnosticSession;
    	    pTx->LengthTotal = 4;
    	    tempRsp = NRC_0x00_PR;
        	break;
        }
//        case DID_0xF188_BRP_Sw_Number:
//        {
//        	tempRsp = drive_RDBI_From_IntFlash( &IntFlashCtrl, DID_0xF188_BRP_Sw_Number, pRx, pTx );
//        	break;
//        }
//        case DID_0xF191_BRP_Hw_Number:
//        {
//        	tempRsp = drive_RDBI_From_IntFlash( &IntFlashCtrl, DID_0xF191_BRP_Hw_Number, pRx, pTx );
//        	break;
//        }
        case DID_0xF192_Manufacturer_Hw_Number:
        {
        	break;
        }
        case DID_0xF194_Manufacturer_Sw_Number:
        {
    	    pTx->Data[0] = pRx->Data[0] + POSITIVE_RESPONSE_OFFSET;
    	    pTx->Data[1] = pRx->Data[1];
    	    pTx->Data[2] = pRx->Data[2];
    		for( uint16_t i = 0; i < 4; i++ )
    		{
    			pTx->Data[i + 3] = ( AppVersion[i] );
    		}
    		pTx->LengthTotal = 7;
    	    tempRsp = NRC_0x00_PR;
        	break;
        }
        case DID_0xF18B_ECU_Build_Date:
        {
//        	tempRsp = drive_RDBI_From_IntFlash( &IntFlashCtrl, DID_0xF18B_ECU_Build_Date, pRx, pTx );
        	//contents below are dummy info for EOL test
    	    pTx->Data[0] = pRx->Data[0] + POSITIVE_RESPONSE_OFFSET;
    	    pTx->Data[1] = pRx->Data[1];
    	    pTx->Data[2] = pRx->Data[2];
    	    pTx->Data[3] = 99;
    	    pTx->Data[4] = 1;
    	    pTx->Data[5] = 1;
    		pTx->LengthTotal = 6;
    	    tempRsp = NRC_0x00_PR;
        	break;
        }
        case DID_0xF18C_ECU_Serial_Number:
        {
//        	tempRsp = drive_RDBI_From_IntFlash( &IntFlashCtrl, DID_0xF18C_ECU_Serial_Number, pRx, pTx );
        	//contents below are dummy info for EOL test
    	    pTx->Data[0] = pRx->Data[0] + POSITIVE_RESPONSE_OFFSET;
    	    pTx->Data[1] = pRx->Data[1];
    	    pTx->Data[2] = pRx->Data[2];
    	    pTx->Data[3] = '2';
    	    pTx->Data[4] = '7';
    	    pTx->Data[5] = '8';
    	    pTx->Data[6] = '0';
    	    pTx->Data[7] = '0';
    	    pTx->Data[8] = '4';
    	    pTx->Data[9] = '1';
    	    pTx->Data[10] = '0';
    	    pTx->Data[11] = '3';
    	    pTx->Data[12] = 'A';
    	    pTx->Data[13] = '1';
    	    pTx->Data[14] = '2';
    	    pTx->Data[15] = '3';
    	    pTx->Data[16] = '4';
    	    pTx->Data[17] = '5';
    	    pTx->Data[18] = '6';
    	    pTx->Data[19] = '7';
    	    pTx->Data[20] = '8';
    		pTx->LengthTotal = 21;
    	    tempRsp = NRC_0x00_PR;
        	break;
        }
//        case DID_0xF1A6_Vehicle_Model:
//        {
//        	tempRsp = drive_RDBI_From_IntFlash( &IntFlashCtrl, DID_0xF1A6_Vehicle_Model, pRx, pTx );
//        	break;
//        }
//        case DID_0xF1A0_Customer_Name:
//        {
//        	tempRsp = drive_RDBI_From_IntFlash( &IntFlashCtrl, DID_0xF1A0_Customer_Name, pRx, pTx );
//        	break;
//        }
//        case DID_0xF1A1_Delivery_Date:
//        {
//        	tempRsp = drive_RDBI_From_IntFlash( &IntFlashCtrl, DID_0xF1A1_Delivery_Date, pRx, pTx );
//        	break;
//        }
//        case DID_0xF1CD_CDID_Raw_Value:
//        {
//        	tempRsp = drive_RDBI_From_IntFlash( &IntFlashCtrl, DID_0xF1CD_CDID_Raw_Value, pRx, pTx );
//        	break;
//        }
        case DID_0xF100_Diagnostic_Code:
        {
//        	tempRsp = drive_RDBI_From_IntFlash( &IntFlashCtrl, DID_0xF100_Diagnostic_Code, pRx, pTx );
    	    pTx->Data[0] = pRx->Data[0] + POSITIVE_RESPONSE_OFFSET;
    	    pTx->Data[1] = pRx->Data[1];
    	    pTx->Data[2] = pRx->Data[2];
    	    pTx->Data[3] = 0xE3;
    	    pTx->Data[4] = 0;
    	    pTx->Data[5] = 0;
    	    pTx->Data[6] = 0;
    		pTx->LengthTotal = 7;
    	    tempRsp = NRC_0x00_PR;
        	break;
        }

        case DID_0xC000_Battery_Voltage:
        {

        	break;
        }
        case DID_0xC001_Throttle_Raw:
        {
    	    tempRsp = drive_RDBI_CopyF32toTx( pRx, pTx, AdcStation1.ThrotADCRawRatio * 10000.0f);
        	break;
        }
        case DID_0xC002_Throttle_Position:
        {
    	    tempRsp = drive_RDBI_CopyF32toTx( pRx, pTx, Axis[0].ThrotMapping.PercentageOut * 100 );
        	break;
        }
        case DID_0xC003_Odometer                                 :
        {
        	break;
        }
        case DID_0xC004_Vehicle_Hour                             :
        {
        	uint32_t tempCehicleHour = TotalTime1.LocalTotalTime * 3;
    	    pTx->Data[0] = pRx->Data[0] + POSITIVE_RESPONSE_OFFSET;
    	    pTx->Data[1] = pRx->Data[1];
    	    pTx->Data[2] = pRx->Data[2];
    		memcpy( &(pTx->Data[3]), &tempCehicleHour, 4 );
    		pTx->LengthTotal = 7;
    	    tempRsp = NRC_0x00_PR;
        	break;
        }
        case DID_0xC005_Dc_Bus_Voltage                           :
        {
    	    tempRsp = drive_RDBI_CopyF32toTx( pRx, pTx, AdcStation1.AdcTraOut.BatVdc );
        	break;
        }
        case DID_0xC006_Motor_Current                            :
        {
        	float tempId = Axis[0].MotorControl.CurrentControl.RotorCurrFb.D;
        	float tempIq = Axis[0].MotorControl.CurrentControl.RotorCurrFb.Q;
        	float tempIs = sqrtf(( tempId * tempId ) + ( tempIq * tempIq ));
        	tempRsp = drive_RDBI_CopyF32toTx( pRx, pTx, tempIs );
        	break;
        }
        case DID_0xC007_Motor_Input_Power                           :
        {
            float tempInputPower = ( Axis[0].MotorControl.CurrentControl.IdCmd * Axis[0].MotorControl.VoltCmd.VdCmd + \
       	                             Axis[0].MotorControl.CurrentControl.IqCmd * Axis[0].MotorControl.VoltCmd.VqCmd ) * Factor_to_cal_power_from_dq;
	        tempRsp = drive_RDBI_CopyF32toTx( pRx, pTx, tempInputPower );
    	    break;
        }
        case DID_0xC008_Motor_Temperature                        :
        {
        	tempRsp = drive_RDBI_CopyF32toTx( pRx, pTx, AdcStation1.AdcTraOut.MOTOR_NTC_0 );
        	break;
        }
        case DID_0xC009_Motor_Temperature_Minimum                :
        {
        	tempRsp = drive_RDBI_CopyF32toTx( pRx, pTx, IntranetCANStation.ServiceCtrlBRP.Motor_Temp_Rec.Temperature_Min );
        	break;
        }
        case DID_0xC00A_Motor_Temperature_Maximum                :
        {
        	tempRsp = drive_RDBI_CopyF32toTx( pRx, pTx, IntranetCANStation.ServiceCtrlBRP.Motor_Temp_Rec.Temperature_Max );
        	break;
        }
        case DID_0xC00B_Motor_Speed                              :
        {
        	tempRsp = drive_RDBI_CopyF32toTx( pRx, pTx, Axis[0].SpeedInfo.MotorMechSpeedRPM );
        	break;
        }
        case DID_0xC00C_Torque_Reference                         :
        {
        	tempRsp = drive_RDBI_CopyF32toTx( pRx, pTx, Axis[0].TorqCommandGenerator.Out );
        	break;
        }
        case DID_0xC00E_ESC_Mosfets_Center_Temperature           :
        {
        	tempRsp = drive_RDBI_CopyF32toTx( pRx, pTx, AdcStation1.AdcTraOut.PCU_NTC[MOS_NTC_1] );
        	break;
        }
        case DID_0xC00F_ESC_Mosfets_Center_Temperature_Minimum   :
        {
        	tempRsp = drive_RDBI_CopyF32toTx( pRx, pTx, IntranetCANStation.ServiceCtrlBRP.ESC_Mosfets_Center_Temp_Rec.Temperature_Min );
        	break;
        }
        case DID_0xC010_ESC_Mosfets_Center_Temperature_Maximum   :
        {
        	tempRsp = drive_RDBI_CopyF32toTx( pRx, pTx, IntranetCANStation.ServiceCtrlBRP.ESC_Mosfets_Center_Temp_Rec.Temperature_Max );
        	break;
        }
        case DID_0xC011_ESC_Mosfets_Side_Temperature             :
        {
        	tempRsp = drive_RDBI_CopyF32toTx( pRx, pTx, AdcStation1.AdcTraOut.PCU_NTC[MOS_NTC_2] );
        	break;
        }
        case DID_0xC012_ESC_Mosfets_Side_Temperature_Minimum     :
        {
        	tempRsp = drive_RDBI_CopyF32toTx( pRx, pTx, IntranetCANStation.ServiceCtrlBRP.ESC_Mosfets_Side_Temp_Rec.Temperature_Min );
        	break;
        }
        case DID_0xC013_ESC_Mosfets_Side_Temperature_Maximum     :
        {
        	tempRsp = drive_RDBI_CopyF32toTx( pRx, pTx, IntranetCANStation.ServiceCtrlBRP.ESC_Mosfets_Side_Temp_Rec.Temperature_Max );
        	break;
        }
        case DID_0xC014_ESC_Capacitor_Temperature                :
        {
        	tempRsp = drive_RDBI_CopyF32toTx( pRx, pTx, AdcStation1.AdcTraOut.PCU_NTC[CAP_NTC] );
        	break;
        }
        case DID_0xC015_ESC_Capacitor_Temperature_Minimum        :
        {
        	tempRsp = drive_RDBI_CopyF32toTx( pRx, pTx, IntranetCANStation.ServiceCtrlBRP.ESC_Capacitor_Temp_Rec.Temperature_Min );
        	break;
        }
        case DID_0xC016_ESC_Capacitor_Temperature_Maximum        :
        {
        	tempRsp = drive_RDBI_CopyF32toTx( pRx, pTx, IntranetCANStation.ServiceCtrlBRP.ESC_Capacitor_Temp_Rec.Temperature_Max );
        	break;
        }

        case DID_0xC017_Sensorless_State                         :
        {
        	UdsDIDSensorlessState_e tempSensorlessState = Axis[0].MotorControl.Sensorless.SensorlessState;
    	    pTx->Data[0] = pRx->Data[0] + POSITIVE_RESPONSE_OFFSET;
    	    pTx->Data[1] = pRx->Data[1];
    	    pTx->Data[2] = pRx->Data[2];
    		pTx->Data[3] = tempSensorlessState;
    		pTx->LengthTotal = 4;
    	    tempRsp = NRC_0x00_PR;
        	break;
        }
        case DID_0xC018_ESC_Operation_State                      :
        {
        	uint8_t tempINVOpState = INVMainState;
    	    pTx->Data[0] = pRx->Data[0] + POSITIVE_RESPONSE_OFFSET;
    	    pTx->Data[1] = pRx->Data[1];
    	    pTx->Data[2] = pRx->Data[2];
    		pTx->Data[3] = tempINVOpState;
    		pTx->LengthTotal = 4;
    	    tempRsp = NRC_0x00_PR;
        	break;
        }
        case DID_0xC019_Current_Limit                            :
        {
        	break;
        }
        case DID_0xC01A_Motor_Phase_U_Current                    :
        {
        	tempRsp = drive_RDBI_CopyF32toTx( pRx, pTx, AdcStation1.AdcTraOut.Iu[0] );
        	break;
        }
        case DID_0xC01B_Motor_Phase_V_Current                    :
        {
        	tempRsp = drive_RDBI_CopyF32toTx( pRx, pTx, AdcStation1.AdcTraOut.Iv[0] );
        	break;
        }
        case DID_0xC01C_Motor_Phase_W_Current                    :
        {
        	tempRsp = drive_RDBI_CopyF32toTx( pRx, pTx, AdcStation1.AdcTraOut.Iw[0] );
        	break;
        }
        case DID_0xC01D_Motor_Direct_Axis_Current                :
        {
        	tempRsp = drive_RDBI_CopyF32toTx( pRx, pTx, Axis[0].MotorControl.CurrentControl.RotorCurrFb.D );
        	break;
        }
        case DID_0xC01E_Motor_Quadrature_Axis_Current            :
        {
        	tempRsp = drive_RDBI_CopyF32toTx( pRx, pTx, Axis[0].MotorControl.CurrentControl.RotorCurrFb.Q );
        	break;
        }
        case DID_0xC01F_Motor_Stator_Current_Is                  :
        {
        	float tempId = Axis[0].MotorControl.CurrentControl.RotorCurrFb.D;
        	float tempIq = Axis[0].MotorControl.CurrentControl.RotorCurrFb.Q;
        	float tempIs = sqrtf(( tempId * tempId ) + ( tempIq * tempIq ));
        	tempRsp = drive_RDBI_CopyF32toTx( pRx, pTx, tempIs );
        	break;
        }
        case DID_0xC020_Set_Point_For_Id                         :
        {
        	tempRsp = drive_RDBI_CopyF32toTx( pRx, pTx, Axis[0].MotorControl.CurrentControl.IdCmd );
        	break;
        }
        case DID_0xC021_Set_Point_For_Iq                         :
        {
        	tempRsp = drive_RDBI_CopyF32toTx( pRx, pTx, Axis[0].MotorControl.CurrentControl.IqCmd );
        	break;
        }
        case DID_0xC022_PWM_Frequency                            :
        {
        	tempRsp = drive_RDBI_CopyF32toTx( pRx, pTx, Axis[0].MotorControl.CurrentControl.PwmHz );
        	break;
        }
        case DID_0xC023_Set_Point_For_Vd                         :
        {
        	tempRsp = drive_RDBI_CopyF32toTx( pRx, pTx, Axis[0].MotorControl.VoltCmd.VdCmd );
        	break;
        }
        case DID_0xC024_Set_Point_For_Vq                         :
        {
        	tempRsp = drive_RDBI_CopyF32toTx( pRx, pTx, Axis[0].MotorControl.VoltCmd.VqCmd );
        	break;
        }
        case DID_0xC025_Modulation_Index                         :
        {
            float tempVsMax = AdcStation1.AdcTraOut.BatVdc > 5.0f ? AdcStation1.AdcTraOut.BatVdc : 5.0f;
            tempVsMax = tempVsMax * Root_of_One_Third;
        	tempRsp = drive_RDBI_CopyF32toTx( pRx, pTx, Axis[0].MotorControl.VoltCmd.VcmdAmp / tempVsMax );
        	break;
        }
        case DID_0xC026_Motor_Pole_Pairs                              :
        {
        	tempRsp = drive_RDBI_CopyF32toTx( pRx, pTx, Axis[0].MotorControl.MotorPara.PM.Polepair );
        	break;
        }
        case DID_0xC027_Motor_R                                  :
        {
        	tempRsp = drive_RDBI_CopyF32toTx( pRx, pTx, Axis[0].MotorControl.Sensorless.EEMF.Res );
        	break;
        }
        case DID_0xC028_Motor_Rs_Max                             :
        {
        	tempRsp = drive_RDBI_CopyF32toTx( pRx, pTx, IntranetCANStation.ServiceCtrlBRP.Res_Max_Rec );
        	break;
        }
        case DID_0xC029_Motor_Ld                                 :
        {
        	tempRsp = drive_RDBI_CopyF32toTx( pRx, pTx, Axis[0].MotorControl.MotorPara.PM.Ld );
        	break;
        }
        case DID_0xC02A_Motor_Lq                                 :
        {
        	tempRsp = drive_RDBI_CopyF32toTx( pRx, pTx, Axis[0].MotorControl.MotorPara.PM.Lq );
        	break;
        }
        case DID_0xC02B_Motor_Flux                                     :
        {
        	tempRsp = drive_RDBI_CopyF32toTx( pRx, pTx, Axis[0].MotorControl.MotorPara.PM.Flux );
        	break;
        }
        case DID_0xC02C_ESC_Internal_circuit_voltage             :
        {
        	tempRsp = drive_RDBI_CopyF32toTx( pRx, pTx, AdcStation1.AdcTraOut.S13V8 );
        	break;
        }
        case DID_0xC02D_DC_Current_Limit                    :
        {
        	break;
        }
        case DID_0xC02E_Power_Level                          :
        {
        	uint8_t tempPowerLevel = 0;//Axis[0].pCANRxInterface->PowerLevel;
    	    pTx->Data[0] = pRx->Data[0] + POSITIVE_RESPONSE_OFFSET;
    	    pTx->Data[1] = pRx->Data[1];
    	    pTx->Data[2] = pRx->Data[2];
    		pTx->Data[3] = tempPowerLevel;
    		pTx->LengthTotal = 4;
    	    tempRsp = NRC_0x00_PR;
        	break;
        }
        case DID_0xC02F_Foil_Position_State                      :
        {
        	UdsDIDFoilPositionState_e tempFoilPosState = 0;
    	    pTx->Data[0] = pRx->Data[0] + POSITIVE_RESPONSE_OFFSET;
    	    pTx->Data[1] = pRx->Data[1];
    	    pTx->Data[2] = pRx->Data[2];
    		pTx->Data[3] = tempFoilPosState;
    		pTx->LengthTotal = 4;
    	    tempRsp = NRC_0x00_PR;
        	break;
        }
        case DID_0xC030_Tether_Cord_State                        :
        {
        	UdsDIDTetherCordState_e tempTetherCordState = 0;//HAL_GPIO_ReadPin( SAFTYSSR_GPIO_Port, SAFTYSSR_Pin );
    	    pTx->Data[0] = pRx->Data[0] + POSITIVE_RESPONSE_OFFSET;
    	    pTx->Data[1] = pRx->Data[1];
    	    pTx->Data[2] = pRx->Data[2];
    		pTx->Data[3] = tempTetherCordState;
    		pTx->LengthTotal = 4;
    	    tempRsp = NRC_0x00_PR;
        	break;
        }
        case DID_0xC031_Session_Time                             :
        {
        	uint32_t tempSessionTime = TotalTime1.LocalThisTime * 3;
    	    pTx->Data[0] = pRx->Data[0] + POSITIVE_RESPONSE_OFFSET;
    	    pTx->Data[1] = pRx->Data[1];
    	    pTx->Data[2] = pRx->Data[2];
    		memcpy( &(pTx->Data[3]), &tempSessionTime, 4 );
    		pTx->LengthTotal = 7;
    	    tempRsp = NRC_0x00_PR;
        	break;
        }
        case DID_0xC032_Session_Distance                         :
        {
        	break;
        }
        case DID_0xC033_ESC_Mosfet_Center_NTC_Status                    :
        {
        	UdsDIDNTCStatus_e tempNTCStatus = ( AlarmStack->FlagRead( AlarmStack, ALARMID_BREAK_NTC_PCU_0 )) ? NTC_Break : \
        	                                  ( AlarmStack->FlagRead( AlarmStack, ALARMID_SHORT_NTC_PCU_0 ) ? NTC_Short : NTC_Normal );
    	    pTx->Data[0] = pRx->Data[0] + POSITIVE_RESPONSE_OFFSET;
    	    pTx->Data[1] = pRx->Data[1];
    	    pTx->Data[2] = pRx->Data[2];
    		pTx->Data[3] = tempNTCStatus;
    		pTx->LengthTotal = 4;
    	    tempRsp = NRC_0x00_PR;
        	break;
        }
        case DID_0xC034_ESC_Mosfet_Side_NTC_Status                      :
        {
        	UdsDIDNTCStatus_e tempNTCStatus = ( AlarmStack->FlagRead( AlarmStack, ALARMID_BREAK_NTC_PCU_1 )) ? NTC_Break : \
        	                                  ( AlarmStack->FlagRead( AlarmStack, ALARMID_SHORT_NTC_PCU_1 ) ? NTC_Short : NTC_Normal );
    	    pTx->Data[0] = pRx->Data[0] + POSITIVE_RESPONSE_OFFSET;
    	    pTx->Data[1] = pRx->Data[1];
    	    pTx->Data[2] = pRx->Data[2];
    		pTx->Data[3] = tempNTCStatus;
    		pTx->LengthTotal = 4;
    	    tempRsp = NRC_0x00_PR;
        	break;
        }
        case DID_0xC035_ESC_Mosfet_Cap_NTC_Status                       :
        {
        	UdsDIDNTCStatus_e tempNTCStatus = ( AlarmStack->FlagRead( AlarmStack, ALARMID_BREAK_NTC_PCU_2 )) ? NTC_Break : \
        	                                  ( AlarmStack->FlagRead( AlarmStack, ALARMID_SHORT_NTC_PCU_2 ) ? NTC_Short : NTC_Normal );
    	    pTx->Data[0] = pRx->Data[0] + POSITIVE_RESPONSE_OFFSET;
    	    pTx->Data[1] = pRx->Data[1];
    	    pTx->Data[2] = pRx->Data[2];
    		pTx->Data[3] = tempNTCStatus;
    		pTx->LengthTotal = 4;
    	    tempRsp = NRC_0x00_PR;
        	break;
        }
        case DID_0xC036_Motor_NTC_Status                         :
        {
        	UdsDIDNTCStatus_e tempNTCStatus = ( AlarmStack->FlagRead( AlarmStack, ALARMID_BREAK_NTC_MOTOR_0 )) ? NTC_Break : \
        				                      ( AlarmStack->FlagRead( AlarmStack, ALARMID_SHORT_NTC_MOTOR_0 ) ? NTC_Short : NTC_Normal );
    	    pTx->Data[0] = pRx->Data[0] + POSITIVE_RESPONSE_OFFSET;
    	    pTx->Data[1] = pRx->Data[1];
    	    pTx->Data[2] = pRx->Data[2];
    		pTx->Data[3] = tempNTCStatus;
    		pTx->LengthTotal = 4;
    	    tempRsp = NRC_0x00_PR;
        	break;
        }
        case DID_0xC037_RC_connetion_status                      :
        {
        	UdsDIDRCConnectionStatus_e tempRCConnct = 0;//Axis[0].pCANRxInterface->RcConnStatus;
    	    pTx->Data[0] = pRx->Data[0] + POSITIVE_RESPONSE_OFFSET;
    	    pTx->Data[1] = pRx->Data[1];
    	    pTx->Data[2] = pRx->Data[2];
    		pTx->Data[3] = tempRCConnct;
    		pTx->LengthTotal = 4;
    	    tempRsp = NRC_0x00_PR;
        	break;
        }
        case DID_0xC038_Error_Code_from_RF                       :
        {
        	break;
        }
        case DID_0xC039_BMS_State_Read_by_ESC                    :
        {
        	break;
        }
        case DID_0xC03A_Estimated_Time_Remaining                 :
        {
        	break;
        }
        case DID_0xC03B_Foil_Position_Voltage                 :
        {
//        	tempRsp = drive_RDBI_CopyF32toTx( pRx, pTx, AdcStation1.AdcTraOut.Foil );
        	break;
        }
        case DID_0xC03C_ESC_Error_Code                 :
        {

    	    pTx->Data[0] = pRx->Data[0] + POSITIVE_RESPONSE_OFFSET;
    	    pTx->Data[1] = pRx->Data[1];
    	    pTx->Data[2] = pRx->Data[2];
            for( uint8_t i = 0; i < MAX_NOW_ALARM_SIZE; i++){
            	pTx->Data[3 + i] = Axis[0].pAlarmStack->NowAlarmID[i];
            }
    		pTx->LengthTotal = 13;
    	    tempRsp = NRC_0x00_PR;
        	break;
        }
        case DID_0xC03D_Motor_Mechanical_Position_rad                 :
        {
        	tempRsp = drive_RDBI_CopyF32toTx( pRx, pTx, PSStation1.MechPosition );
        	break;
        }
        case DID_0xC040_Position_Linear_Points_In_Degree                 :
        {
        	static uint8_t PositionLinearPointsCnt = 0;
        	if ( PositionLinearPointsCnt < 32 )
        	{
        		float tempLinearPointsMechPosRadDegree = 0.0f;
        		tempLinearPointsMechPosRadDegree = LinearPointsMechPosRad[PositionLinearPointsCnt] * 360.0f / _2PI;
            	tempRsp = drive_RDBI_CopyF32toTx( pRx, pTx, tempLinearPointsMechPosRadDegree );
            	PositionLinearPointsCnt++;
        	}
        	else
        	{
        		PositionLinearPointsCnt = 0;
        		tempRsp = NRC_0x22_CNC;
        	}

        	break;
        }
#if USE_DATA_RECORDER
        case DID_0xC050_DATA_RECORDER_CH0                 :
        {
        	if ( DataRECORDERCnt0 < MAX_RECORD_NUM )
        	{
            	tempRsp = drive_RDBI_CopyF32toTx( pRx, pTx, RecordedData[0][DataRECORDERCnt0] );
            	DataRECORDERCnt0++;
        	}
        	else
        	{
        		DataRECORDERCnt0 = 0;
        		tempRsp = NRC_0x22_CNC;
        	}

        	break;
        }
        case DID_0xC051_DATA_RECORDER_CH1                 :
        {
        	if ( DataRECORDERCnt1 < MAX_RECORD_NUM )
        	{
            	tempRsp = drive_RDBI_CopyF32toTx( pRx, pTx, RecordedData[1][DataRECORDERCnt1] );
            	DataRECORDERCnt1++;
        	}
        	else
        	{
        		DataRECORDERCnt1 = 0;
        		tempRsp = NRC_0x22_CNC;
        	}

        	break;
        }
        case DID_0xC052_DATA_RECORDER_CH2                 :
        {
        	if ( DataRECORDERCnt2 < MAX_RECORD_NUM )
        	{
            	tempRsp = drive_RDBI_CopyF32toTx( pRx, pTx, RecordedData[2][DataRECORDERCnt2] );
            	DataRECORDERCnt2++;
        	}
        	else
        	{
        		DataRECORDERCnt2 = 0;
        		tempRsp = NRC_0x22_CNC;
        	}

        	break;
        }
        case DID_0xC053_DATA_RECORDER_CH3                 :
        {
        	if ( DataRECORDERCnt3 < MAX_RECORD_NUM )
        	{
            	tempRsp = drive_RDBI_CopyF32toTx( pRx, pTx, RecordedData[3][DataRECORDERCnt3] );
            	DataRECORDERCnt3++;
        	}
        	else
        	{
        		DataRECORDERCnt3 = 0;
        		tempRsp = NRC_0x22_CNC;
        	}

        	break;
        }
#endif
        case DID_0xC100_This_Driving_Cycle_Information  :
        {
        	break;
        }
        case DID_0xC200_Last_Driving_Cycle_Information_1  :
        {
        	break;
        }
        case DID_0xC201_Last_Driving_Cycle_Information_2  :
        {
        	break;
        }
        case DID_0xC202_Last_Driving_Cycle_Information_3  :
        {
        	break;
        }
        case DID_0xC203_Last_Driving_Cycle_Information_4  :
        {
        	break;
        }
        case DID_0xC204_Last_Driving_Cycle_Information_5  :
        {
        	break;
        }
        case DID_0xC205_Time_Spent_Over_Speed_Derating_Occurred_during_Vehicle_Life  :
        {
        	break;
        }
        case DID_0xF1F2_RF_FW_VERSION  :
        {
			if((RCCommCtrl.RcInfoQueryCompleteFlag & RC_COMM_RC_INFO_QUERY_COMPLETE_FLAG_MASK_RF_FW_VERSION) !=0)
			{
				pTx->Data[0] = pRx->Data[0] + POSITIVE_RESPONSE_OFFSET;
				pTx->Data[1] = pRx->Data[1];
				pTx->Data[2] = pRx->Data[2];
				memcpy(&pTx->Data[3], RCCommCtrl.RFFwVer, RC_COMM_RF_FW_VER_SIZE);
				pTx->LengthTotal = 3 + RC_COMM_RF_FW_VER_SIZE;
				tempRsp = NRC_0x00_PR;
        	}
			else
			{
				tempRsp = NRC_0x22_CNC;
			} 
			break;
        }
		case DID_0xF1F3_RF_SERAIL_NUMBER  :
		{		
			/* The buffer was loaded, reply the value */
			if((RCCommCtrl.RcInfoQueryCompleteFlag & RC_COMM_RC_INFO_QUERY_COMPLETE_FLAG_MASK_RF_SN) !=0)
			{
				pTx->Data[0] = pRx->Data[0] + POSITIVE_RESPONSE_OFFSET;
				pTx->Data[1] = pRx->Data[1];
				pTx->Data[2] = pRx->Data[2];
				memcpy(&pTx->Data[3], RCCommCtrl.RFSN, RC_COMM_RF_SN_SIZE);
				pTx->LengthTotal = 3 + RC_COMM_RF_SN_SIZE;
				tempRsp = NRC_0x00_PR;
			}
			else
			{
				tempRsp = NRC_0x22_CNC;
			}
        	break;
        }
		case DID_0xF1F4_RC_FW_VERSION  :
        {
			if((RCCommCtrl.RcInfoQueryCompleteFlag & RC_COMM_RC_INFO_QUERY_COMPLETE_FLAG_MASK_RC_FW_VERSION) !=0)
			{
				pTx->Data[0] = pRx->Data[0] + POSITIVE_RESPONSE_OFFSET;
				pTx->Data[1] = pRx->Data[1];
				pTx->Data[2] = pRx->Data[2];
				memcpy(&pTx->Data[3], RCCommCtrl.RCFwVer, RC_COMM_RC_FW_VER_SIZE);
				pTx->LengthTotal = 3 + RC_COMM_RC_FW_VER_SIZE;
				tempRsp = NRC_0x00_PR;
			}
			else
			{
				tempRsp = NRC_0x22_CNC;
			}
        	break;
        }
		case DID_0xF1F5_RC_SERIAL_NUMBER  :
        {
			if((RCCommCtrl.RcInfoQueryCompleteFlag & RC_COMM_RC_INFO_QUERY_COMPLETE_FLAG_MASK_RC_SN) !=0)
			{
				pTx->Data[0] = pRx->Data[0] + POSITIVE_RESPONSE_OFFSET;
				pTx->Data[1] = pRx->Data[1];
				pTx->Data[2] = pRx->Data[2];
				memcpy(&pTx->Data[3], RCCommCtrl.RCSN, RC_COMM_RC_SN_SIZE);
				pTx->LengthTotal = 3 + RC_COMM_RC_SN_SIZE;
				tempRsp = NRC_0x00_PR;
			}
			else
			{
				tempRsp = NRC_0x22_CNC;	
			}
			break;
        }
        default:
        {
        	tempRsp = NRC_0x31_ROOR;
        	break;
        }

    }
    if ( pTx->LengthTotal > MAX_BUFFER_SIZE )
    	tempRsp = NRC_0x14_RTL;
    return tempRsp;
}

__STATIC_FORCEINLINE EnumUdsBRPNRC drive_RDBI_CopyF32toTx( LinkLayerCtrlUnit_t *pRx, LinkLayerCtrlUnit_t *pTx, float input )
{
	EnumUdsBRPNRC tempRsp = NRC_0x10_GR;
    pTx->Data[0] = pRx->Data[0] + POSITIVE_RESPONSE_OFFSET;
    pTx->Data[1] = pRx->Data[1];
    pTx->Data[2] = pRx->Data[2];
	memcpy( &(pTx->Data[3]), &input, 4 );
	pTx->LengthTotal = 7;
    tempRsp = NRC_0x00_PR;

    return tempRsp;
}
__STATIC_FORCEINLINE void drive_DTC_Pickup_Data_to_Store( AlarmStack_t *AlarmStack, DTCStation_t *v )
{
	for ( uint8_t i = 0; i < AlarmStack->TopIndicator; i++ )
	{
		uint8_t tempDTC_Number = 0;
		switch ( AlarmStack->NowAlarmID[i] )
		{
		    case ALARMID_UNDER_VOLTAGE_BUS:
		    {
		    	tempDTC_Number = DTC_RecordNumber_P0562_System_voltage_low;

		    	break;
		    }
		    case ALARMID_OVER_VOLTAGE_BUS:
		    {
		    	tempDTC_Number = DTC_RecordNumber_P0563_System_voltage_high;
	        	break;
		    }
		    case ALARMID_POWER_TRANSISTOR_OC:
		    {
		    	continue;
		    	//tempDTC_Number = DTC_RecordNumber_P1F01_ESC_Over_current;
	        	break;
		    }
		    case ALARMID_UNDER_VOLTAGE_13V:
		    {
		    	tempDTC_Number = DTC_RecordNumber_P1F09_ESC_Internal_circuit_voltage_out_of_range;
	        	break;
		    }
		    case ALARMID_OT_PCU_0:
		    case ALARMID_OT_PCU_1:
		    {
		    	tempDTC_Number = DTC_RecordNumber_P1F02_ESC_Mosfet_High_temperature;
	        	break;
		    }
		    case ALARMID_OT_PCU_2:
		    {
		    	tempDTC_Number = DTC_RecordNumber_P1F03_ESC_Capacitor_High_temperature;
	        	break;
		    }
		    case ALARMID_OT_MOTOR_0:
		    {
		    	tempDTC_Number = DTC_RecordNumber_P1F04_Motor_High_temperature;
	        	break;
		    }
		    case ALARMID_BUFFER_IC_ERROR:
		    {
		    	tempDTC_Number = DTC_RecordNumber_P1F0A_ESC_Internal_circuit_logical_failure;
	        	break;
		    }
		    case ALARMID_PHASE_LOSS:
		    {
		    	tempDTC_Number = DTC_RecordNumber_P0C05_Motor_Phase_lost;
	        	break;
		    }
		    case ALARMID_MOTOR_OVER_SPEED:
		    {
		    	tempDTC_Number = DTC_RecordNumber_P0219_Motor_Overspeed;
	        	break;
		    }
		    case ALARMID_ACC_PEDAL_BREAK:
		    case ALARMID_ACC_PEDAL_SHORT:
		    {
		    	tempDTC_Number = DTC_RecordNumber_P18A6_Foil_Position_sensor_abnormal;
	        	break;
		    }
		    case ALARMID_BREAK_NTC_PCU_0 :
		    case ALARMID_SHORT_NTC_PCU_0 :
		    case ALARMID_BREAK_NTC_PCU_1 :
		    case ALARMID_SHORT_NTC_PCU_1 :
		    {
		    	tempDTC_Number = DTC_RecordNumber_P0666_ESC_Mosfet_Temperature_sensor_abnormal;
	        	break;
		    }
		    case ALARMID_BREAK_NTC_PCU_2 :
		    case ALARMID_SHORT_NTC_PCU_2 :
		    {
		    	tempDTC_Number = DTC_RecordNumber_P0667_ESC_Capacitor_Temperature_sensor_abnormal;
	        	break;
		    }
		    case ALARMID_BREAK_NTC_MOTOR_0 :
		    case ALARMID_SHORT_NTC_MOTOR_0 :
		    {
		    	tempDTC_Number = DTC_RecordNumber_P0A2A_Motor_Temperature_sensor_abnormal;
	        	break;
		    }
		    case ALARMID_MOTORSTALL :
		    {
		    	tempDTC_Number = DTC_RecordNumber_P1F00_Motor_Stalled;
	        	break;
		    }
		    case ALARMID_CAN1_TIMEOUT :
		    {
		    	tempDTC_Number = DTC_RecordNumber_U0111_Lost_communication_with_BMS;
	        	break;
		    }
		    case ALARMID_FLASH_UNINITIALIZED :
		    case ALARMID_FLASH_READ_FAILED :
		    case ALARMID_FLASH_DAMAGED :
		    {
		    	tempDTC_Number = DTC_RecordNumber_P0605_Internal_Control_Module_ROM_Error;
	        	break;
		    }
		    case ALARMID_OT_PCU_0_WARNING :
		    case ALARMID_OT_PCU_1_WARNING :
		    {
		    	tempDTC_Number = DTC_RecordNumber_P1F12_ESC_Mosfet_High_temperature_warning;
	        	break;
		    }
		    case ALARMID_OT_PCU_2_WARNING :
		    {
		    	tempDTC_Number = DTC_RecordNumber_P1F13_ESC_Capacitor_High_temperature_warning;
	        	break;
		    }
		    case ALARMID_OT_MOTOR_0_WARNING :
		    {
		    	tempDTC_Number = DTC_RecordNumber_P1F14_Motor_High_temperature_warning;
	        	break;
		    }
		    default:
		    {
		    	break;
		    }
		}

    	v->StatusOfDTC_Realtime[tempDTC_Number].Test_Failed = 1;
    	if ( v->DTCStorePackge[tempDTC_Number].DTC_Store_State == DTC_Store_State_None && v->StatusOfDTC_Realtime[tempDTC_Number].Test_Failed == TRUE )
    	{
    		 v->DTCStorePackge[tempDTC_Number].DTC_Store_State = DTC_Store_State_Confirmed_and_wait_for_Store;
    		drive_DTC_Pickup_Freeze_Frame_data( v, tempDTC_Number );

    	    v->State = DTC_Process_State_Write;
    	}
	}

//    v->StatusOfDTC_Realtime[DTC_RecordNumber_P1F05_ESC_current_sensor_abnormal              ].Test_Failed =  ;
//    v->StatusOfDTC_Realtime[DTC_RecordNumber_P1F06_System_voltage_sensor_abnormal           ].Test_Failed =  ;
//
//    v->StatusOfDTC_Realtime[DTC_RecordNumber_U0107_Lost_communication_with_RF               ].Test_Failed =  ;
//    v->StatusOfDTC_Realtime[DTC_RecordNumber_P060E_Throttle_position_performance            ].Test_Failed =  ;
//    v->StatusOfDTC_Realtime[DTC_RecordNumber_U0412_Invalid_data_received_from_BMS           ].Test_Failed =  ;

}

__STATIC_FORCEINLINE void drive_DTC_Pickup_Freeze_Frame_data( DTCStation_t *v, uint8_t DTC_Record_Number )
{

//	v->DTCStorePackge[DTC_Record_Number].StoreContent.DTCStoredData.Battery_Voltage =
	v->DTCStorePackge[DTC_Record_Number].StoreContent.DTCStoredData.Dc_Bus_Voltage = AdcStation1.AdcTraOut.BatVdc;
	/*
	 * Move to housekeeping
	 *	v->DTCStorePackge[DTC_Record_Number].StoreContent.DTCStoredData.Motor_Current = sqrtf(( Axis[0].MotorControl.CurrentControl.RotorCurrFb.D * Axis[0].MotorControl.CurrentControl.RotorCurrFb.D ) + \
	 *	                                                                              ( Axis[0].MotorControl.CurrentControl.RotorCurrFb.Q * Axis[0].MotorControl.CurrentControl.RotorCurrFb.Q ));
	 *  v->DTCStorePackge[DTC_Record_Number].StoreContent.DTCStoredData.Motor_Input_Power =  ( Axis[0].MotorControl.CurrentControl.IdCmd * Axis[0].MotorControl.VoltCmd.VdCmd + \
     *                                                                                      Axis[0].MotorControl.CurrentControl.IqCmd * Axis[0].MotorControl.VoltCmd.VqCmd ) * Factor_to_cal_power_from_dq;
	 *  v->DTCStorePackge[DTC_Record_Number].StoreContent.DTCStoredData.Modulation_Index = Axis[0].MotorControl.VoltCmd.VcmdAmp / ( Root_of_One_Third * AdcStation1.AdcTraOut.BatVdc );
	 *
	 *
	 */
	v->DTCStorePackge[DTC_Record_Number].StoreContent.DTCStoredData.Motor_Direct_Axis_Current = Axis[0].MotorControl.CurrentControl.RotorCurrFb.D;
	v->DTCStorePackge[DTC_Record_Number].StoreContent.DTCStoredData.Motor_Quadrature_Axis_Current = Axis[0].MotorControl.CurrentControl.RotorCurrFb.Q;
	v->DTCStorePackge[DTC_Record_Number].StoreContent.DTCStoredData.Set_Point_For_Id = Axis[0].MotorControl.CurrentControl.IdCmd;
	v->DTCStorePackge[DTC_Record_Number].StoreContent.DTCStoredData.Set_Point_For_Iq = Axis[0].MotorControl.CurrentControl.IqCmd;
	v->DTCStorePackge[DTC_Record_Number].StoreContent.DTCStoredData.Set_Point_For_Vd = Axis[0].MotorControl.VoltCmd.VdCmd;
	v->DTCStorePackge[DTC_Record_Number].StoreContent.DTCStoredData.Set_Point_For_Vq = Axis[0].MotorControl.VoltCmd.VqCmd;
//	v->DTCStorePackge[DTC_Record_Number].StoreContent.DTCStoredData.DC_Current_Limit
	v->DTCStorePackge[DTC_Record_Number].StoreContent.DTCStoredData.Electrical_Angle  = Axis[0].MotorControl.CurrentControl.EleAngle;
//	v->DTCStorePackge[DTC_Record_Number].StoreContent.DTCStoredData.ESC_Internal_circuit_voltage = AdcStation1.AdcTraOut.V13;
	v->DTCStorePackge[DTC_Record_Number].StoreContent.DTCStoredData.Throttle_Position = Axis[0].ThrotMapping.PercentageOut * 100.0f;
	v->DTCStorePackge[DTC_Record_Number].StoreContent.DTCStoredData.Motor_Speed = Axis[0].SpeedInfo.MotorMechSpeedRPM;
	v->DTCStorePackge[DTC_Record_Number].StoreContent.DTCStoredData.Torque_Reference = Axis[0].TorqCommandGenerator.Out;
	v->DTCStorePackge[DTC_Record_Number].StoreContent.DTCStoredData.Motor_Temperature = AdcStation1.AdcTraOut.MOTOR_NTC_0;
	v->DTCStorePackge[DTC_Record_Number].StoreContent.DTCStoredData.ESC_Mosfets_Center_Temperature = AdcStation1.AdcTraOut.PCU_NTC[MOS_NTC_1];
	v->DTCStorePackge[DTC_Record_Number].StoreContent.DTCStoredData.ESC_Mosfets_Side_Temperature = AdcStation1.AdcTraOut.PCU_NTC[MOS_NTC_2];
	v->DTCStorePackge[DTC_Record_Number].StoreContent.DTCStoredData.ESC_Capacitor_Temperature = AdcStation1.AdcTraOut.PCU_NTC[CAP_NTC];
//	v->DTCStorePackge[DTC_Record_Number].StoreContent.DTCStoredData.Foil_Position_Voltage = AdcStation1.AdcTraOut.Foil;
	v->DTCStorePackge[DTC_Record_Number].StoreContent.DTCStoredData.ESC_Mosfet_Center_NTC_Status = ( AlarmStack->FlagRead( AlarmStack, ALARMID_BREAK_NTC_PCU_0 )) ? NTC_Break : \
                                                                                      ( AlarmStack->FlagRead( AlarmStack, ALARMID_SHORT_NTC_PCU_0 ) ? NTC_Short : NTC_Normal );
	v->DTCStorePackge[DTC_Record_Number].StoreContent.DTCStoredData.ESC_Mosfet_Side_NTC_Status = ( AlarmStack->FlagRead( AlarmStack, ALARMID_BREAK_NTC_PCU_1 )) ? NTC_Break : \
                                                                                    ( AlarmStack->FlagRead( AlarmStack, ALARMID_SHORT_NTC_PCU_1 ) ? NTC_Short : NTC_Normal );
	v->DTCStorePackge[DTC_Record_Number].StoreContent.DTCStoredData.ESC_Cap_NTC_Status = ( AlarmStack->FlagRead( AlarmStack, ALARMID_BREAK_NTC_PCU_2 )) ? NTC_Break : \
                                                                            ( AlarmStack->FlagRead( AlarmStack, ALARMID_SHORT_NTC_PCU_2 ) ? NTC_Short : NTC_Normal );
	v->DTCStorePackge[DTC_Record_Number].StoreContent.DTCStoredData.Motor_NTC_Status =( AlarmStack->FlagRead( AlarmStack, ALARMID_BREAK_NTC_MOTOR_0 )) ? NTC_Break : \
			                                                             ( AlarmStack->FlagRead( AlarmStack, ALARMID_SHORT_NTC_MOTOR_0 ) ? NTC_Short : NTC_Normal );
	v->DTCStorePackge[DTC_Record_Number].StoreContent.DTCStoredData.Sensorless_State = Axis[0].MotorControl.Sensorless.SensorlessState;
	v->DTCStorePackge[DTC_Record_Number].StoreContent.DTCStoredData.ESC_Operation_State = INVMainState;
//	v->DTCStorePackge[DTC_Record_Number].StoreContent.DTCStoredData.RC_Connection_Status = Axis[0].pCANRxInterface->RcConnStatus;
//	v->DTCStorePackge[DTC_Record_Number].StoreContent.DTCStoredData.BMS_Status_Read_By_ESC
	v->DTCStorePackge[DTC_Record_Number].StoreContent.DTCStoredData.Session_Time = TotalTime1.LocalThisTime * 3;
	v->DTCStorePackge[DTC_Record_Number].StoreContent.DTCStoredData.Vehicle_Hour = TotalTime1.LocalTotalTime * 3;
	/*
	 * Move to housekeeping
	 */
//    v->DTCStorePackge[DTC_Record_Number].StoreContent.DTCStoredDataRecordNumberOfIdentifiers = 1;
//    v->DTCStorePackge[DTC_Record_Number].StoreContent.DataIdentifierHi = DID_0xC2FF_Environmental_Data >> 8;
//    v->DTCStorePackge[DTC_Record_Number].StoreContent.DataIdentifierLow = DID_0xC2FF_Environmental_Data & 0xFF;
}


void Drive_PcuPowerStateMachine( void )
{

}

// Change INV operating state according Axis alarm status and servo on status.
void Drive_INVStateMachine( void )
{
	switch( INVMainState )
	{
		case INV_OP_INITIALIZING:

			// normal transition
			if( IsPcuInitReady == PcuInitState_Ready )
			{
				// clear error BMS LED
				Axis[0].pCANTxInterface->BmsCtrlCmd.LedCtrlCmd.All = BAT_LED_SHOW_NO_ERROR;
				INVMainState = INV_OP_STANDBY;
			}
			break;

		case INV_OP_STANDBY:

			// error situation
			if( Axis[0].HasCriAlarm == 1 )
			{
				// set error BMS LED
				Axis[0].pCANTxInterface->BmsCtrlCmd.LedCtrlCmd.All = BAT_LED_SHOW_INV_ERROR;
				INVMainState = INV_OP_ALARM;
			}
			else if( Axis[0].HasNonCriAlarm == 1 )
			{
				// set error BMS LED
				Axis[0].pCANTxInterface->BmsCtrlCmd.LedCtrlCmd.All = BAT_LED_SHOW_INV_ERROR;
				INVMainState = INV_OP_LIMPHOME;
			}
			else if( Axis[0].HasWarning == 1 )
			{
				// set error BMS LED
				Axis[0].pCANTxInterface->BmsCtrlCmd.LedCtrlCmd.All = BAT_LED_SHOW_INV_ERROR;
				INVMainState = INV_OP_WARNING;
			}
			else if( Axis[0].ServoOn == 1 ) // normal transitions
			{
				// clear error BMS LED
				Axis[0].pCANTxInterface->BmsCtrlCmd.LedCtrlCmd.All = BAT_LED_SHOW_NO_ERROR;
				INVMainState = INV_OP_NORMAL;
			}
			else
			{
				// keep in the same state
				// do nothing
			}
			break;

		case INV_OP_NORMAL:

			// error situation
			if( Axis[0].HasCriAlarm == 1 )
			{
				// set error BMS LED
				Axis[0].pCANTxInterface->BmsCtrlCmd.LedCtrlCmd.All = BAT_LED_SHOW_INV_ERROR;
				INVMainState = INV_OP_ALARM;
			}
			else if( Axis[0].HasNonCriAlarm == 1 )
			{
				// set error BMS LED
				Axis[0].pCANTxInterface->BmsCtrlCmd.LedCtrlCmd.All = BAT_LED_SHOW_INV_ERROR;
				INVMainState = INV_OP_LIMPHOME;
			}
			else if( Axis[0].HasWarning == 1 )
			{
				// set error BMS LED
				Axis[0].pCANTxInterface->BmsCtrlCmd.LedCtrlCmd.All = BAT_LED_SHOW_INV_ERROR;
				INVMainState = INV_OP_WARNING;
			}
			else if( Axis[0].ServoOn == 0 ) // normal transitions
			{
				// clear error BMS LED
				Axis[0].pCANTxInterface->BmsCtrlCmd.LedCtrlCmd.All = BAT_LED_SHOW_NO_ERROR;
				INVMainState = INV_OP_STANDBY;
			}
			else
			{
				// keep in the same state
				// do DC limit, AC limit, generate Tq from FourQuadCtrl at AxisFactory_Do10HzLoop
			}
			break;

		case INV_OP_WARNING:

			// error situation
			if( Axis[0].HasCriAlarm == 1 )
			{
				// set error BMS LED
				Axis[0].pCANTxInterface->BmsCtrlCmd.LedCtrlCmd.All = BAT_LED_SHOW_INV_ERROR;
				INVMainState = INV_OP_ALARM;
			}
			else if( Axis[0].HasNonCriAlarm == 1 )
			{
				// set error BMS LED
				Axis[0].pCANTxInterface->BmsCtrlCmd.LedCtrlCmd.All = BAT_LED_SHOW_INV_ERROR;
				INVMainState = INV_OP_LIMPHOME;
			}

			// if warning is reset
			else if( Axis[0].HasWarning == 0 )
			{
				// clear error BMS LED
				Axis[0].pCANTxInterface->BmsCtrlCmd.LedCtrlCmd.All = BAT_LED_SHOW_NO_ERROR;
				if( Axis[0].ServoOn == 1 )
				{
					INVMainState = INV_OP_NORMAL;
				}
				else
				{
					INVMainState = INV_OP_STANDBY;
				}
			}
			else
			{
				// keep in the same state
				// do DC limit, AC limit, generate Tq from FourQuadCtrl
			}
			break;

		case INV_OP_LIMPHOME:

			// error situation
			if( Axis[0].HasCriAlarm == 1 )
			{
				// set error BMS LED
				Axis[0].pCANTxInterface->BmsCtrlCmd.LedCtrlCmd.All = BAT_LED_SHOW_INV_ERROR;
				INVMainState = INV_OP_ALARM;
			}
			else
			{
				// keep in the same state
				// do DC limit, AC limit, generate Tq from FourQuadCtrl
			}

			/* todo if (ESC is rebooting)
			{
				INVMainState = ESCOP_PowerOff;
			}
			*/
			break;

		case INV_OP_ALARM:

			// error situation
			if( Axis[0].HasCriAlarm == 0 )
			{
				// set error BMS LED
				Axis[0].pCANTxInterface->BmsCtrlCmd.LedCtrlCmd.All = BAT_LED_SHOW_INV_ERROR;
				INVMainState = INV_OP_LIMPHOME;
			}
			else
			{
				// keep in the same state
				// ESC is servo off due to Axis[0].HasCriAlarm == 0 in AxisFactory_RunMotorStateMachine.
			}
			break;

		// abnormal PcuPowerState value
		case INV_OP_POWER_OFF:
		default:
			break;
	}
	  Axis[0].pCANTxInterface->InvState = INVMainState;
}


void Inv_ServoOnReq()
{
  Axis[0].FourQuadCtrl.ServoCmdIn = ENABLE;
  Axis[0].FourQuadCtrl.GearPositionCmd = PCU_SHIFT_D;

  /* Enable boost mode function */
  GearMode_EnableBoostMode();
  GearMode_EnableReverseMode();
}

void Inv_ServoOffReq()
{
  Axis[0].FourQuadCtrl.ServoCmdIn = DISABLE;
  Axis[0].FourQuadCtrl.GearPositionCmd = PCU_SHIFT_P;

  /* Disable boost mode function */
  GearMode_DisableBoostMode();

}

__STATIC_FORCEINLINE void EnterVehicleAlarmState( void )
{
	// todo set RC alarm flag
	// todo clear RC warning/limp flag

	/* put INV to servo off */
  Inv_ServoOffReq();
  Led_CtrlReq(LED_IDX_FRONT, LED_MODE_BLINK, LED_NBR_TO_BLINK_FOREVER, LED_BLINK_CONFIG_4HZ);
  VehicleMainState = VEHICLE_STATE_ALARM;
}

__STATIC_FORCEINLINE void EnterVehicleLimpHomeState( void )
{
	// set Axis trigger limp home flag
	Axis[0].TriggerLimpHome = 1;
	// todo set RC limp home flag;
	// todo clear RC warning
	// todo set DC ramp?
  VehicleMainState = VEHICLE_STATE_LIMPHOME;  
}

__STATIC_FORCEINLINE void EnterVehicleWarningState( void )
{
	/* Disable boost mode function */
	GearMode_DisableBoostMode();
	Led_CtrlReq(LED_IDX_FRONT, LED_MODE_BLINK, LED_NBR_TO_BLINK_FOREVER, LED_BLINK_CONFIG_1HZ);
  VehicleMainState = VEHICLE_STATE_WARNING;
}

__STATIC_FORCEINLINE void EnterVehicleDriveState( void )
{
	// set Axis trigger limp home flag
	Axis[0].TriggerLimpHome = 0;
  
	if(VehicleMainState == VEHICLE_STATE_STANDBY)
	{
		ButtonReleasedFlags = 0;
	}

	DualBtnTimeCnt = 0;
	Led_TurnOnReq(LED_IDX_FRONT);

	VehicleMainState = VEHICLE_STATE_DRIVE;
	HAL_GPIO_WritePin(Front_sig_DO_GPIO_Port, Front_sig_DO_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(Rear_sig_DO_GPIO_Port, Rear_sig_DO_Pin, GPIO_PIN_SET);
}

__STATIC_FORCEINLINE void EnterVehicleIdleState( void )
{
	SocLightModeSet(SOC_DISPLAY_MODE_DIRECTLY_ACCESS);
	Led_TurnOffReq(LED_IDX_REAR);
	Led_CtrlReq(LED_IDX_FRONT, LED_MODE_BLINK, 5, LED_BLINK_CONFIG_5HZ);
  VehicleMainState = VEHICLE_STATE_IDLE;
}

__STATIC_FORCEINLINE void EnterVehicleStartupState( void )
{
  /* Enable global alarm detection */
  AlarmMgr1.State = ALARM_MGR_STATE_ENABLE;
  BatStation.PwrOnReq();
  Led_CtrlReq(LED_IDX_FRONT, LED_MODE_BLINK, 5, LED_BLINK_CONFIG_5HZ);
  VehicleMainState = VEHICLE_STATE_STARTUP;
}

__STATIC_FORCEINLINE void EnterVehicleStandbyState( void )
{
  // set Axis trigger limp home flag
  Axis[0].TriggerLimpHome = 0;
  DualBtnTimeCnt = 0;
  Led_CtrlReq(LED_IDX_FRONT, LED_MODE_BLINK, 5, LED_BLINK_CONFIG_5HZ);
  SocLightModeSet(SOC_DISPLAY_MODE_SOC_SINGLE_LED);
  /* Enable CAN1 timeout detection */
  Axis[0].AlarmDetect.CAN1Timeout.AlarmInfo.AlarmEnable = ALARM_ENABLE;
  /* Enable UVP detection */
  Axis[0].AlarmDetect.UVP_Bus.AlarmInfo.AlarmEnable = ALARM_ENABLE;
  /* Clear button release flags */
  ButtonReleasedFlags = 0;
  /* Put INV to servo-off */
  Inv_ServoOffReq();

  VehicleMainState = VEHICLE_STATE_STANDBY;
}

__STATIC_FORCEINLINE void EnterVehicleShutdownState( void )
{

  /* Disable CAN1 timeout detection */
  Axis[0].AlarmDetect.CAN1Timeout.AlarmInfo.AlarmEnable = ALARM_DISABLE;

  /* Disable UVP detection */
  /* Note : the INV level Alarm detection control should be applied by INV station
    but not in vehicle station, this is a short-term solution before we have a 
    individual INV control module
   */
  Axis[0].AlarmDetect.UVP_Bus.AlarmInfo.AlarmEnable = ALARM_DISABLE;

  /* Request BMS to turn off battery */
  BatStation.PwrOffReq();
  Led_CtrlReq(LED_IDX_FRONT, LED_MODE_BLINK, 5, LED_BLINK_CONFIG_5HZ);


  VehicleMainState = VEHICLE_STATE_SHUTDOWN;
}

__STATIC_FORCEINLINE void EnterVehiclePowerOffState( void )
{
  /* Disable global alarm detection */
  AlarmMgr1.State = ALARM_MGR_STATE_DISABLE;
  
  VehicleMainState = VEHICLE_STATE_POWER_OFF;
}

__STATIC_FORCEINLINE void EnterVehicleInitialState( void )
{
  /* Disable global alarm detection */
  AlarmMgr1.State = ALARM_MGR_STATE_DISABLE;
  BootAppTrig = BOOT_ENA;
  INVMainState = INV_OP_INITIALIZING;
  Led_CtrlReq(LED_IDX_FRONT, LED_MODE_BLINK, 5, LED_BLINK_CONFIG_5HZ);
  SocLightModeSet(SOC_DISPLAY_MODE_DIRECTLY_ACCESS);
  Led_TurnOffReq(LED_IDX_REAR);

  VehicleMainState = VEHICLE_STATE_INITIALIZING;

  /* Disable CAN1 timeout detection */
  Axis[0].AlarmDetect.CAN1Timeout.AlarmInfo.AlarmEnable = ALARM_DISABLE;
  /* Disable UVP detection */
  Axis[0].AlarmDetect.UVP_Bus.AlarmInfo.AlarmEnable = ALARM_DISABLE;
}

void Drive_VehicleStateMachine( void )
{
  switch( VehicleMainState )
  {
    case VEHICLE_STATE_INITIALIZING:

      // normal transition
      if( INVMainState == INV_OP_STANDBY )
      {
        EnterVehicleIdleState();
      }

      // action

      break;

    case VEHICLE_STATE_IDLE:
    
      /* waiting for killing switch to transfer to startup state */
			if(Btn_StateRead(BTN_IDX_KILL_SW) == BTN_KILL_SW_RELEASE)
      {
        EnterVehicleStartupState();
      }

      break;

    case VEHICLE_STATE_STARTUP:

      if ((INVMainState == INV_OP_ALARM) || (BatStation.MainSMGet() == BAT_MAIN_ALARM))
      {
        EnterVehicleAlarmState();
      }
      else if(Btn_StateRead(BTN_IDX_KILL_SW) == BTN_KILL_SW_PRESS) /* Kill SW pressed*/
      {
          EnterVehicleShutdownState();
      }
      else
      {
        switch (BatStation.MainSMGet())
        {
          case BAT_MAIN_ACTIVATED:
            EnterVehicleStandbyState();
            break;

          default:
            break;
        }
      }

      break;

    case VEHICLE_STATE_STANDBY:

      // error situation
      if ((INVMainState == INV_OP_ALARM) || (BatStation.MainSMGet() == BAT_MAIN_ALARM))
      {
        EnterVehicleAlarmState();
      }
      else if (Btn_StateRead(BTN_IDX_KILL_SW) == BTN_KILL_SW_PRESS) /* Kill SW pressed*/
      { 
        EnterVehicleShutdownState();
      }
      else if (ButtonReleasedFlags != VEHICLE_SM_CTRL_ALL_BTN_RELEASED_FLAG)  /* hold until user release both buttons*/
      {
        if(Btn_StateRead(BTN_IDX_BST_BTN)== BTN_BOOST_RELEASE)
        {
          ButtonReleasedFlags |= VEHICLE_SM_CTRL_BOOST_BTN_RELEASED_FLAG;
        }

        if(Btn_StateRead(BTN_IDX_REV_BTN)== BTN_REVERSE_RELEASE)
        {
          ButtonReleasedFlags |= VEHICLE_SM_CTRL_REVERSE_BTN_RELEASED_FLAG;
        }

      }
      else
      {
        if ((Btn_StateRead(BTN_IDX_BST_BTN)== BTN_BOOST_PRESS) &&
            (Btn_StateRead(BTN_IDX_REV_BTN)== BTN_REVERSE_PRESS) &&
            (Axis[0].ThrotMapping.PercentageTarget < 0.01 ))
        {
          DualBtnTimeCnt ++;

          if(DualBtnTimeCnt > 100)
          {
          	EnterVehicleDriveState();
          }
        }
        else
        {
          DualBtnTimeCnt = 0;
        }

      }

      break;

    case VEHICLE_STATE_DRIVE:

      // error situation
      if( INVMainState == INV_OP_ALARM || Bat_MainSMGet() == BAT_MAIN_ALARM )
      {
        EnterVehicleAlarmState();
      }
	  else if( Btn_StateRead(BTN_IDX_KILL_SW) == BTN_KILL_SW_PRESS)
      {
        EnterVehicleStandbyState();
      }
			else if( INVMainState == INV_OP_WARNING)
      {
        EnterVehicleWarningState();
      }
      else if( INVMainState == INV_OP_LIMPHOME)
      {
        EnterVehicleLimpHomeState();
      }
      else if (ButtonReleasedFlags != VEHICLE_SM_CTRL_ALL_BTN_RELEASED_FLAG)  /* hold until user release both buttons*/
      {
        if(Btn_StateRead(BTN_IDX_BST_BTN)== BTN_BOOST_RELEASE)
        {
          ButtonReleasedFlags |= VEHICLE_SM_CTRL_BOOST_BTN_RELEASED_FLAG;
        }

        if(Btn_StateRead(BTN_IDX_REV_BTN)== BTN_REVERSE_RELEASE)
        {
          ButtonReleasedFlags |= VEHICLE_SM_CTRL_REVERSE_BTN_RELEASED_FLAG;
        }

        if(ButtonReleasedFlags == VEHICLE_SM_CTRL_ALL_BTN_RELEASED_FLAG)
        {
      		/* Put INV to servo on */
      		Inv_ServoOnReq();
        }
      }
      else
      { 
        if ((Btn_StateRead(BTN_IDX_BST_BTN)== BTN_BOOST_PRESS) &&
            (Btn_StateRead(BTN_IDX_REV_BTN)== BTN_REVERSE_PRESS) &&
            (Axis[0].ThrotMapping.PercentageTarget < 0.01 ) &&
            (Axis[0].SpeedInfo.MotorMechSpeedRPMAbs < 100.0))
        {
          DualBtnTimeCnt ++;

          if(DualBtnTimeCnt > 300)
          {
            EnterVehicleStandbyState();
          }
        }
        else
        {
        	DualBtnTimeCnt = 0;
        }
      }
      break;

    case VEHICLE_STATE_WARNING:
      if( INVMainState == INV_OP_ALARM || Bat_MainSMGet() == BAT_MAIN_ALARM)	// error situation
      {
        EnterVehicleAlarmState();
      }
	    else if( Btn_StateRead(BTN_IDX_KILL_SW) == BTN_KILL_SW_PRESS)
      {
        EnterVehicleStandbyState();
      }
      else if( INVMainState == INV_OP_NORMAL )
      {
        EnterVehicleDriveState();
      }
      else if (ButtonReleasedFlags != VEHICLE_SM_CTRL_ALL_BTN_RELEASED_FLAG)  /* hold until user release both buttons*/
      {
        if(Btn_StateRead(BTN_IDX_BST_BTN)== BTN_BOOST_RELEASE)
        {
          ButtonReleasedFlags |= VEHICLE_SM_CTRL_BOOST_BTN_RELEASED_FLAG;
        }

        if(Btn_StateRead(BTN_IDX_REV_BTN)== BTN_REVERSE_RELEASE)
        {
          ButtonReleasedFlags |= VEHICLE_SM_CTRL_REVERSE_BTN_RELEASED_FLAG;
        }

        if(ButtonReleasedFlags == VEHICLE_SM_CTRL_ALL_BTN_RELEASED_FLAG)
        {
      		/* Put INV to servo on */
      		Inv_ServoOnReq();
        }
      }
      else
      { 
        if ((Btn_StateRead(BTN_IDX_BST_BTN)== BTN_BOOST_PRESS) &&
            (Btn_StateRead(BTN_IDX_REV_BTN)== BTN_REVERSE_PRESS) &&
            (Axis[0].ThrotMapping.PercentageTarget < 0.01 ) &&
            (Axis[0].SpeedInfo.MotorMechSpeedRPMAbs < 100.0))
        {
          DualBtnTimeCnt ++;

          if(DualBtnTimeCnt > 300)
          {
            EnterVehicleStandbyState();
          }
        }
        else
        {
        	DualBtnTimeCnt = 0;
        }
      }
      break;

    case VEHICLE_STATE_LIMPHOME:
      /* for E10-P0, Transfer to warning state directly */
      EnterVehicleWarningState();
      break;

    case VEHICLE_STATE_ALARM:

      // error situation
      if (Btn_StateRead(BTN_IDX_KILL_SW) == BTN_KILL_SW_PRESS)
      {
          EnterVehicleShutdownState();
      }
      break;

    case VEHICLE_STATE_SHUTDOWN:
      
      /* check if shutdown process is completed*/
      if(Btn_StateRead(BTN_IDX_KILL_SW) == BTN_KILL_SW_RELEASE)
      {
    	  EnterVehicleStartupState();
      }
      else if((Bat_MainSMGet() == BAT_MAIN_SM_IDLE) || (Bat_MainSMGet() == BAT_MAIN_ALARM))
      {
        EnterVehicleInitialState();
      }

      break; 

    case VEHICLE_STATE_POWER_OFF:
    default:
      break;

  }

  Axis[0].pCANTxInterface->VehicleState = VehicleMainState;
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
	// Start timer 3 for total time function and avoid first interrupt request.
	HAL_TIM_Base_Start_TOTAL_TIME(&htim3);
	ParamMgr1.OnParamValueChanged = &Drive_OnParamValueChanged;
	IntranetCANStation.AccessParam = &AccessParam;
	IntranetCANStation.pParamMgr = &ParamMgr1;
	IntranetCANStation.ServiceCtrlBRP.RDBI_Function = &drive_RDBI_Function;
	IntranetCANStation.ServiceCtrlBRP.pDTCStation = &DTCStation1;
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
	for( AxisIndex = 0; AxisIndex < ACTIVE_AXIS_NUM; AxisIndex++ )
	{
		// Init Axis object. Notice that: initialize axis after reading data from rxt.flash and PSB.
		Axis[AxisIndex].Init ( &Axis[AxisIndex], AxisIndex );
	}

	for( AxisIndex = 0; AxisIndex < MAX_AXIS_NUM; AxisIndex++ )
	{
		AlarmMgr1.pHasWarning[AxisIndex] = &Axis[AxisIndex].HasWarning;
		AlarmMgr1.pHasNonCriAlarm[AxisIndex] = &Axis[AxisIndex].HasNonCriAlarm;
		AlarmMgr1.pHasCriAlarm[AxisIndex] = &Axis[AxisIndex].HasCriAlarm;
	}

	// Init Axis1 Motor Stall Table
	Axis[0].MotorStall.Enable = SystemTable.AlarmTableInfo[ALARMID_MOTORSTALL].AlarmEnable;
	Axis[0].MotorStall.Init( &Axis[0].MotorStall );

	// Init Axis1 Thermal Table
	Axis[0].ThermoStrategy.Init( &Axis[0].ThermoStrategy, &SystemTable.WindingDeratingInfo, &SystemTable.MosDeratingInfo, &SystemTable.CapDeratingInfo, &AdcStation1 );

	// Init Buffer IC (Pull low BUffer enable gpio)
	IoState = HAL_GPIO_ReadPin( BUF_FB_DI_GPIO_Port, BUF_FB_DI_Pin);
	IoState_2 = HAL_GPIO_ReadPin( BUF_ENA_DO_GPIO_Port, BUF_ENA_DO_Pin);
	if( ( IoState == SIGNAL_HIGH ) && ( IoState_2 == SIGNAL_HIGH )  )
	{
		HAL_GPIO_WritePin( BUF_ENA_DO_GPIO_Port, BUF_ENA_DO_Pin, GPIO_PIN_RESET );
		Axis[0].AlarmDetect.BufICEnable = PULL_LOW;
	}
	else
	{
		// Buffer IC Error State
		Axis[0].AlarmDetect.BufICEnable  = PULL_HIGH;
	}

	// Init Pwm Station
	PwmStation1.Init( &PwmStation1 );

	// Init ADC from tables
	AdcStation1.Init( &AdcStation1 );

	PSStation1.Init( &PSStation1, DriveParams.SystemParams.MechPositionZeroOffset, DriveParams.SystemParams.MechPositionCompCoefBySpeed );
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
	ExtranetCANStation.DriveSetup.LoadParam( &ExtranetCANStation.DriveSetup, &CANModuleConfigExtra, CanIdTableExtra );
	ExtranetCANStation.Init( &ExtranetCANStation,&ExtranetInformInSystemTableExample, &hfdcan2);

	// Init CAN internal
	IntranetCANStation.NetWork.DriveSetup.LoadParam ( &IntranetCANStation.NetWork.DriveSetup, &CANModuleConfigIntra, LscCanIdTableIntra );
	IntranetCANStation.Init ( &IntranetCANStation, &hfdcan2, &PcuAuthorityCtrl );

	//Assign the security from external flash after read data from external flash.
	PcuAuthorityCtrl.SecureLvNow = DriveParams.SystemParams.ParamMgrSecurity;
	// note: IntranetCANStation.ServiceCtrlBRP.pSecurityCtrl = IntranetCANStation.pSecurityCtrl = &PcuAuthorityCtrl.
	ParamMgr1.Security = IntranetCANStation.pSecurityCtrl->SecureLvNow;

	Drive_BinVersionCompare( AppVersion );
	Drive_BinCheckWordCompare( &AppCheckWord );

	// MCU LED light On
//	HAL_GPIO_WritePin( MCU_State_LED_GPIO_Port, MCU_State_LED_Pin, GPIO_PIN_SET );


	IntFlashCtrl.Init ( &IntFlashCtrl );

	ExtranetCANStation.Enable = ENABLE;
	ExtranetCANStation.ForceDisable = DISABLE;

//	RCCommCtrl.VerConfig = DriveParams.PCUParams.DebugParam8;
//	RCCommCtrl.Init(&RCCommCtrl,&huart3,&hcrc,Axis[0].pCANTxInterface,Axis[0].pCANRxInterface);

	// DTC Init
	DTCStation1.Init( &DTCStation1 );

	// Register alarm depend on AlarmTableInfo table and error status of each module.
	GlobalAlarmDetect_init();

	/* Button control init */
	Btn_Init();
	
	/*Init BAT control unit*/
	BatStation.CanHandleLoad(&ExtranetCANStation);
			AlarmMgr1.State = ALARM_MGR_STATE_ENABLE;  //temporary solution for alarm after power on
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
	//TODO: use calibration values in the future
//	if( Axis[0].ThrottleGainState == GAIN_STATE_NORMAL )
//	{
		Axis[ 0 ].pAdcStation->AdcExeThrotGain.FDta = Axis[ 0 ].ThrottleGain;
//	}else;

	Axis[ 0 ].pAdcStation->AdcExeThrotZero = (float)DriveParams.SystemParams.ThrottleMinRawRatio * 0.0001;
	Axis[ 0 ].pAdcStation->AdcExeThrotMax = (float)DriveParams.SystemParams.ThrottleMaxRawRatio * 0.0001;

}

#if USE_THROTTLE_CALIBRATION == USE_FUNCTION
void drive_ThrottleGainInit( DriveParams_t *d, AdcStation *a )
{
//	if( ( d->SystemParams.ThrottleMaxAdc == 4095 ) && (d->SystemParams.ThrottleMinAdc == 0) )
//	{
//		Axis[0].ThrottleGainState = GAIN_STATE_EMPTY;
//		Axis[0].ThrottleGain = GAIN_STATE_EMPTY;
//	}
//	else
		if( d->SystemParams.ThrottleMaxRawRatio < d->SystemParams.ThrottleMinRawRatio )
	{
		Axis[0].ThrottleGainState = GAIN_STATE_ABNORMAL;
		Axis[0].ThrottleGain = 0;
	}
	else
	{
		Axis[0].ThrottleGainState = GAIN_STATE_NORMAL;
		Axis[0].ThrottleGain = ( float )( 10000.0f /( float )( d->SystemParams.ThrottleMaxRawRatio - d->SystemParams.ThrottleMinRawRatio ) );
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

void drive_DoPwmPositionCatch(TIM_HandleTypeDef *htim)
{
	uint32_t IC2Val = 0;
    if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_2)
    {
          /* Get the Input Capture value */
        IC2Val = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_2);
        if (IC2Val != 0)
        {
            /* Duty cycle computation */
            PSStation1.DutyFromPwm = ((float)(HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1) * 100)) / (float)IC2Val;

            PSStation1.FreqFromPwm = ( (float)HAL_RCC_GetSysClockFreq()  ) / ((float)(IC2Val) * 17.0f);

        }
        else
        {
        	PSStation1.DutyFromPwm = 0;
        	PSStation1.FreqFromPwm = 0;
        }
    }

}

__attribute__(( section(".ram_function"))) void drive_DoCurrentLoop(void)
{
//	int i;

	AdcStation1.DoCurrentLoop( &AdcStation1 );

	PSStation1.DoCurrentLoop( &PSStation1 );
#if USE_CALC_SUM_ROOT==USE_FUNCTION
	MFStation1.CalSumRoot( &MFStation1, &AdcStation1, PcuAuthorityCtrl.SecureLvNow );
#endif

	//TODO add "GlobalAlarmDetect_Accumulation" here.
//	for( i = 0; i < ACTIVE_AXIS_NUM; i++ )
//	{
		// Update Control Feedback
		Axis[0].MotorControl.SensorFb.Iu = AdcStation1.AdcTraOut.Iu[0];
		Axis[0].MotorControl.SensorFb.Iw = AdcStation1.AdcTraOut.Iv[0];
		Axis[0].MotorControl.SensorFb.Iv = AdcStation1.AdcTraOut.Iw[0];
		Axis[0].MotorControl.SensorFb.Vbus = AdcStation1.AdcTraOut.BatVdc;

		Axis[0].MotorControl.CurrentControl.EleAngle = PSStation1.ElecPosition;
		Axis[0].MotorControl.CurrentControl.EleSpeed = PSStation1.ElecSpeed;

		// Do Current Loop Tasks
		Axis[0].DoCurrentLoop(&Axis[0]);
//	}

#if USE_DATA_RECORDER
		DataRecorder_Routine(\
				AdcStation1.AdcTraOut.Iu[0], \
				AdcStation1.AdcTraOut.Iv[0], \
				AdcStation1.AdcTraOut.Iw[0], \
				Axis[0].MotorControl.SensorFb.EleSpeed \
				);
/*
		DataRecorder_Routine(\
				Axis[0].MotorControl.CurrentControl.IdCmd, \
				Axis[0].MotorControl.CurrentControl.RotorCurrFb.D, \
				Axis[0].MotorControl.CurrentControl.IqCmd, \
				Axis[0].MotorControl.CurrentControl.RotorCurrFb.Q \
				);
*/


#endif
}

void Session_DoPLCLoop(void)
{
	switch ( ParamMgr1.Session )
	{
	case Session_0x01_Default:
		Axis[0].MfOrRDFunctionDisable = 1;
		break;
	case Session_0x02_Programming:
		Axis[0].MfOrRDFunctionDisable = 1;
		//todo check security level in bootloader. Remove the this condition in the future.
		if( PcuAuthorityCtrl.SecureLvNow >= Security_Level_2 )
		{
			BootAppTrig = BOOT_ENA;
		}
		break;
	case Session_0x03_ExtendedDiagnostic:
		Axis[0].MfOrRDFunctionDisable = 1;
		break;
	case Session_0x04_SafetySystemDiagnostic:
		Axis[0].MfOrRDFunctionDisable = 1;
		break;
	case Session_0x40_VehicleManufacturerSpecific:
		if( PcuAuthorityCtrl.SecureLvNow >= Security_Level_5 )
		{
			if ((DriveFnRegs[FN_ENABLE-FN_BASE] | DriveFnRegs[FN_MF_FUNC_SEL-FN_BASE] | DriveFnRegs[FN_RD_FUNC_SEL-FN_BASE]) == 0)
			{
				Axis[0].MfOrRDFunctionDisable = 1;
			}
			else
			{
				// enable AxisFactory_GetSetting;  AxisFactory_GetUiStatus; AxisFactory_GetUiCmd;
				Axis[0].MfOrRDFunctionDisable = 0;
			}
		}
		break;
	case Session_0x60_SystemSupplierSpecific:
		if( PcuAuthorityCtrl.SecureLvNow >= Security_Level_5 )
		{
			if ((DriveFnRegs[FN_ENABLE-FN_BASE] | DriveFnRegs[FN_MF_FUNC_SEL-FN_BASE] | DriveFnRegs[FN_RD_FUNC_SEL-FN_BASE]) == 0)
			{
				Axis[0].MfOrRDFunctionDisable = 1;
			}
			else
			{
				// enable AxisFactory_GetSetting;  AxisFactory_GetUiStatus; AxisFactory_GetUiCmd;
				Axis[0].MfOrRDFunctionDisable = 0;
			}

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
			PositionCalibration_Routine(&DriveFnRegs[ FN_MF_POS_CALIB_START - FN_BASE ], &PSStation1);

#if USE_DATA_RECORDER
			if ( DriveFnRegs[ FN_MF_DATA_RECORDER_ACTIVE - FN_BASE ] == 1)
			{
			    IsRecordActive = USE_FUNCTION;
			    DataRECORDERCnt0 = 0;
			    DataRECORDERCnt1 = 0;
			    DataRECORDERCnt2 = 0;
			    DataRECORDERCnt3 = 0;
			    DriveFnRegs[ FN_MF_DATA_RECORDER_ACTIVE - FN_BASE ] = 0;
			}
#endif
		}
		break;
	default:
		break;
	}
}

__STATIC_FORCEINLINE void ResetMFWhenSessionChange(Axis_t *pAxis)
{
	DriveFnRegs[FN_ENABLE-FN_BASE] = 0;
	DriveFnRegs[FN_MF_FUNC_SEL-FN_BASE] = 0;
	DriveFnRegs[FN_RD_FUNC_SEL-FN_BASE] = 0;
	DriveFnRegs[FN_OPEN_SPD_COMMAND-FN_BASE] = 0;
	DriveFnRegs[FN_OPEN_SPD_V_I_LIMIT-FN_BASE] = 0;
	DriveFnRegs[FN_RPM_GAIN_CMD-FN_BASE] = 0;
	DriveFnRegs[FN_RPM_SLOPE_CMD-FN_BASE] = 0;
	pAxis->CtrlUiEnable = 0;
	CtrlUi.MfFunMode = FN_MF_FUNC_SEL_RESERVED;
}
__STATIC_FORCEINLINE void EnableAlarmWhenSessionChange(Axis_t *pAxis)
{
	pAxis->AlarmDetect.BREAK_NTC_PCU_0.AlarmInfo.AlarmEnable = ALARM_ENABLE;
	pAxis->AlarmDetect.BREAK_NTC_PCU_1.AlarmInfo.AlarmEnable = ALARM_ENABLE;
	pAxis->AlarmDetect.BREAK_NTC_PCU_2.AlarmInfo.AlarmEnable = ALARM_ENABLE;
	pAxis->AlarmDetect.BREAK_NTC_Motor_0.AlarmInfo.AlarmEnable = ALARM_ENABLE;
	pAxis->AlarmDetect.pPhaseLoss->Enable = ALARM_ENABLE;
	pAxis->MotorStall.Enable = ALARM_ENABLE;
	pAxis->AlarmDetect.ACC_PEDAL_SENSOR_BREAK.AlarmInfo.AlarmEnable = ALARM_ENABLE;
}

__STATIC_FORCEINLINE void DisableAlarmWhenSessionChange(Axis_t *pAxis)
{
	pAxis->AlarmDetect.BREAK_NTC_PCU_0.AlarmInfo.AlarmEnable = ALARM_DISABLE;
	pAxis->AlarmDetect.BREAK_NTC_PCU_1.AlarmInfo.AlarmEnable = ALARM_DISABLE;
	pAxis->AlarmDetect.BREAK_NTC_PCU_2.AlarmInfo.AlarmEnable = ALARM_DISABLE;
	pAxis->AlarmDetect.BREAK_NTC_Motor_0.AlarmInfo.AlarmEnable = ALARM_DISABLE;
	pAxis->AlarmDetect.pPhaseLoss->Enable = ALARM_DISABLE;
	pAxis->MotorStall.Enable = ALARM_DISABLE;
	pAxis->AlarmDetect.ACC_PEDAL_SENSOR_BREAK.AlarmInfo.AlarmEnable = ALARM_DISABLE;
}

void Session_DoWhileSessionChange(void)
{
	// Set flag or do action according to the next session
	switch ( ParamMgr1.NextSession )
	{
	case Session_0x01_Default:
		Axis[0].MfOrRDFunctionDisable = 1;
		ResetMFWhenSessionChange( &Axis[0] );
		EnableAlarmWhenSessionChange( &Axis[0] );
		break;
	case Session_0x02_Programming:
		Axis[0].MfOrRDFunctionDisable = 1;
		ResetMFWhenSessionChange( &Axis[0] );
		EnableAlarmWhenSessionChange( &Axis[0] );
		break;
	case Session_0x03_ExtendedDiagnostic:
		Axis[0].MfOrRDFunctionDisable = 1;
		ResetMFWhenSessionChange( &Axis[0] );
		EnableAlarmWhenSessionChange( &Axis[0] );
		break;
	case Session_0x04_SafetySystemDiagnostic:
		Axis[0].MfOrRDFunctionDisable = 1;
		ResetMFWhenSessionChange( &Axis[0] );
		EnableAlarmWhenSessionChange( &Axis[0] );
		break;
	case Session_0x40_VehicleManufacturerSpecific:
//		AlarmMgr1.State = ALARM_MGR_STATE_ENABLE;
		EnableAlarmWhenSessionChange( &Axis[0] );
		break;
	case Session_0x60_SystemSupplierSpecific:
		// reset all alarm
		AlarmMgr1.State = ALARM_MGR_STATE_DISABLE;
		ResetAllAlarm( &AlarmMgr1 );
		AlarmMgr1.State = ALARM_MGR_STATE_ENABLE;
		DisableAlarmWhenSessionChange( &Axis[0] );
		break;
	default:
		break;
	}
}

// 1kHz
void drive_DoPLCLoop(void)
{
	int i;
	AdcStation1.DoPLCLoop( &AdcStation1 );
	PSStation1.DoPLCLoop( &PSStation1 );
	//TODO add "GlobalAlarmDetect_Accumulation" here.
	for( i = 0; i < ACTIVE_AXIS_NUM; i++ )
	{
		Axis[i].DoPLCLoop(&Axis[i]);
	}

	Session_DoPLCLoop();

	ExtranetCANStation.DisableRst( &ExtranetCANStation );
	if ( ExtranetCANStation.Enable )
	{
		ExtranetCANStation.DoPlcLoop ( &ExtranetCANStation );
	}
	IntranetCANStation.ServiceCtrlBRP.ServoOnOffState = Axis[0].ServoOn;
	IntranetCANStation.DoPlcLoop( &IntranetCANStation );

	if ( Axis[0].HasCriAlarm || Axis[0].HasNonCriAlarm)
	{
	    drive_DTC_Pickup_Data_to_Store( &AlarmStack[0], &DTCStation1 );
	}

	Drive_PcuPowerStateMachine(); // note: Tx response will delay one PLCLoop(1ms)

}

void drive_Do100HzLoop(void)
{
	static uint8_t IsNotFirstLoop = 0;
	int i;
	AdcStation1.Do100HzLoop( &AdcStation1 );
	//Axis[0].MotorControl.Sensorless.EEMF.WindingTemp = AdcStation1.AdcTraOut.MOTOR_NTC_0;
	for( i = 0; i < ACTIVE_AXIS_NUM; i++ )
	{
		Axis[i].Do100HzLoop(&Axis[i]);
	}
	MFStation1.GpioMfinfo( &MFStation1 );

	if( ParamMgr1.Session != ParamMgr1.NextSession )
	{
		Session_DoWhileSessionChange();
		ParamMgr1.Session = ParamMgr1.NextSession;
	}

	/*update max and min value*/
	/*To do: currently, MCU enter 100Hz earlier than PLC loop, temporary solution is bypass the first entering*/
	if ( IsNotFirstLoop == 1 )
	{
	    IntranetCANStation.ServiceCtrlBRP.ESC_Capacitor_Temp_Rec.Temperature_Max = \
            MAX2( IntranetCANStation.ServiceCtrlBRP.ESC_Capacitor_Temp_Rec.Temperature_Max, AdcStation1.AdcTraOut.PCU_NTC[CAP_NTC]);
	    IntranetCANStation.ServiceCtrlBRP.ESC_Capacitor_Temp_Rec.Temperature_Min = \
			MIN2( IntranetCANStation.ServiceCtrlBRP.ESC_Capacitor_Temp_Rec.Temperature_Min, AdcStation1.AdcTraOut.PCU_NTC[CAP_NTC]);

	    IntranetCANStation.ServiceCtrlBRP.ESC_Mosfets_Center_Temp_Rec.Temperature_Max = \
			MAX2( IntranetCANStation.ServiceCtrlBRP.ESC_Mosfets_Center_Temp_Rec.Temperature_Max, AdcStation1.AdcTraOut.PCU_NTC[MOS_NTC_1]);
	    IntranetCANStation.ServiceCtrlBRP.ESC_Mosfets_Center_Temp_Rec.Temperature_Min = \
			MIN2( IntranetCANStation.ServiceCtrlBRP.ESC_Mosfets_Center_Temp_Rec.Temperature_Min, AdcStation1.AdcTraOut.PCU_NTC[MOS_NTC_1]);

	    IntranetCANStation.ServiceCtrlBRP.ESC_Mosfets_Side_Temp_Rec.Temperature_Max = \
	        MAX2( IntranetCANStation.ServiceCtrlBRP.ESC_Mosfets_Side_Temp_Rec.Temperature_Max, AdcStation1.AdcTraOut.PCU_NTC[MOS_NTC_2]);
	    IntranetCANStation.ServiceCtrlBRP.ESC_Mosfets_Side_Temp_Rec.Temperature_Min = \
			MIN2( IntranetCANStation.ServiceCtrlBRP.ESC_Mosfets_Side_Temp_Rec.Temperature_Min, AdcStation1.AdcTraOut.PCU_NTC[MOS_NTC_2]);

	    IntranetCANStation.ServiceCtrlBRP.Motor_Temp_Rec.Temperature_Max = \
	        MAX2( IntranetCANStation.ServiceCtrlBRP.Motor_Temp_Rec.Temperature_Max, AdcStation1.AdcTraOut.MOTOR_NTC_0);
	    IntranetCANStation.ServiceCtrlBRP.Motor_Temp_Rec.Temperature_Min = \
			MIN2( IntranetCANStation.ServiceCtrlBRP.Motor_Temp_Rec.Temperature_Min, AdcStation1.AdcTraOut.MOTOR_NTC_0);

	    IntranetCANStation.ServiceCtrlBRP.Res_Max_Rec = MAX2( IntranetCANStation.ServiceCtrlBRP.Res_Max_Rec, Axis[0].MotorControl.Sensorless.EEMF.Res );

	}
	else
	{
	    IsNotFirstLoop = 1;
	}

	Btn_SignalWrite(BTN_IDX_KILL_SW, HAL_GPIO_ReadPin(Kill_Switch_DI_GPIO_Port, Kill_Switch_DI_Pin));
	Btn_SignalWrite(BTN_IDX_BST_BTN, HAL_GPIO_ReadPin(Boost_DI_GPIO_Port,Boost_DI_Pin));
	Btn_SignalWrite(BTN_IDX_REV_BTN, HAL_GPIO_ReadPin(Reverse_DI_GPIO_Port,Reverse_DI_Pin));
	Btn_Do100HzLoop();

	Drive_VehicleStateMachine();
	Drive_INVStateMachine();
	BatStation.InvDcVoltSet(Axis[0].pAdcStation->AdcTraOut.BatVdc);
	BatStation.Do100HzLoop();
	SocValueSet(Bat_SocGet(BAT_INSTANCE_MAIN));
	Led_Do100HzLoop();

	/*LED output*/
	HAL_GPIO_WritePin(Front_sig_DO_GPIO_Port, Front_sig_DO_Pin, Led_CmdGet(LED_IDX_FRONT));
	HAL_GPIO_WritePin(Rear_sig_DO_GPIO_Port, Rear_sig_DO_Pin, Led_CmdGet(LED_IDX_REAR));
}

// Check if "any" component's temperature is higher then minimum temperature in relative derating table.
static uint8_t IsCompTempOverWarningTemp( ThermoStrategy_t *v )
{
	uint8_t IsOverTempFlag = 0;
	if( *(v->TempNow[MOS_NTC_1]) > v->MosDerating.X.InputMin )
	{
		IsOverTempFlag = 1;
		v->ThermoDeratingSrc |= MOS_DERATING;
	}
	else if( *(v->TempNow[MOS_NTC_2]) > v->MosDerating.X.InputMin )
	{
		IsOverTempFlag = 1;
		v->ThermoDeratingSrc |= MOS_DERATING;
	}
	else
	{
		v->ThermoDeratingSrc &= ~MOS_DERATING;
	}

	if( *(v->TempNow[MOTOR_NTC_0_A0]) > v->WindingDerating.X.InputMin )
	{
		IsOverTempFlag = 1;
		v->ThermoDeratingSrc |= MOTOR_DERATING;
	}
	else
	{
		v->ThermoDeratingSrc &= ~MOTOR_DERATING;
	}

	if( *(v->TempNow[CAP_NTC]) > v->CapDerating.X.InputMin )
	{
		IsOverTempFlag = 1;
		v->ThermoDeratingSrc |= CAP_DERATING;
	}
	else
	{
		v->ThermoDeratingSrc &= ~CAP_DERATING;

	}

	return IsOverTempFlag;
}

void drive_Do10HzLoop(void)
{
	int i; // Axis index

	for( i = 0; i < ACTIVE_AXIS_NUM; i++ )
	{
		// auto set and reset warning depending on NTC
		if( IsCompTempOverWarningTemp( &Axis[i].ThermoStrategy) )
		{
			AlarmMgr1.RegisterWarning( &AlarmMgr1, AXIS_INDEX_TO_AXIS_ID(i) );
		}
		else
		{
			AlarmMgr1.ResetWarning( &AlarmMgr1, AXIS_INDEX_TO_AXIS_ID(i) );
		}
		Axis[i].Do10HzLoop(&Axis[i]);

	}
//	RCCommCtrl._10HzLoop(&RCCommCtrl);

	// Because ESC does not receive RECEIVED_BAT_ID_1 for 100 ms, CAN1Timeout.Counter will be greater than 10;
	// Note: CAN1Timeout.Counter increase every 100Hz in AlarmDetect_Do100HzLoop function
	// If ESC does not receive RECEIVED_BAT_ID_1 for 100 ms(A.K.A. CAN1Timeout.Counter>10),
	// then AccCANErrorCnt increase.
	if( Axis[0].AlarmDetect.CAN1Timeout.Counter >= 10)
	{
		if(	Axis[0].pCANRxInterface->AccCANErrorCnt <65535 )
		{
			Axis[0].pCANRxInterface->AccCANErrorCnt++;
		}
	}
}

void drive_DoTotalTime(void)
{
	// When TIM2 ARPE = 0, ARR change directly. The setting influence the end of THIS CNT.
	if( TotalTime1.BufferServoOnState == MOTOR_STATE_OFF)
	{
		TIM3->ARR = (15000-1); // 3 sec
	}
	else
	{
		TIM3->ARR = (150000-1); // 30 sec
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
	// do remaining time calculation
	uint16_t FCC = 2419;   //default set do designed capacity    Uint: Wh
	uint8_t Related_SoC = Axis[0].pCANRxInterface->BmsReportInfo.Soc;    // Uint: %
	uint16_t Insta_Power = 0;
	float temp_Insta_Power = Axis[0].pCANRxInterface->BmsReportInfo.DcVolt * Axis[0].pCANRxInterface->BmsReportInfo.Current;

	Insta_Power = ( temp_Insta_Power >= 0 ) ? (uint16_t)temp_Insta_Power : 0;

	RemainingTime1.Do1secLoop ( &RemainingTime1, FCC, Related_SoC, Insta_Power, Axis[0].TriggerLimpHome );

	Axis[0].pCANTxInterface->Debugf[IDX_REMAIN_TIME] = (float)RemainingTime1.Remaining_Time_Min;

}

// old function definition before 3.1.1.11, not necessary now
/*
void Drive_ResetNonCriAlarmCNTandStatus(Axis_t *v, AlarmMgr_t *pAlarmMgr)
{
	v->RequestResetNonCriAlarmCNT = RESET_NonCriAlarm_RESETING;
	// Reset All NonCriAlarmr before reset alarm stack and clear hasNonCriAlarm.
	// Reset NonCriAlarmCNT
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
	pAlarmMgr->ResetAllNonCriAlarm( pAlarmMgr );
	v->RequestResetNonCriAlarmCNT = RESET_NonCriAlarm_IDLE;
}
*/

void drive_DoHouseKeeping(void)
{
	uint16_t lIdxTemp = 0;

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

	if( Axis[0].ServoOn == 0 )
	{
		if( ExtFlash1.ParamBackupRequest == Target_EFlash )
		{
			// Param Backup
			ExtFlash1.ParamBackup( &ExtFlash1, &DriveParams );

			ExtFlash1.ParamBackupRequest = 0;
			DriveFnRegs[FN_PARAM_BACKUP_EMEMORY - FN_BASE] = 0;
		}
		else if( DriveFnRegs[FN_ORIGIN_PARAM_BACKUP - FN_BASE] )
		{
			// Param Backup
			ExtFlash1.ParamBackup( &ExtFlash1, &DriveParams );

			// Backup Current Calibration to another section of external flash
			ExtFlash1.Curr_Calib_Store.CurrentCalibrationBackup( &ExtFlash1, &DriveParams );

			DriveFnRegs[FN_ORIGIN_PARAM_BACKUP - FN_BASE] = 0;
		}
	}

	Axis[0].pCANTxInterface->DebugU8[TX_INTERFACE_DBG_IDX_LOG_ENABLE_FLAG] =(uint8_t)( 0xFF & DriveParams.PCUParams.DebugParam10 );
	Axis[0].pCANTxInterface->DebugU8[TX_INTERFACE_DBG_IDX_BMS_COMM_ENABLE] =(uint8_t)( 0xFF & DriveParams.PCUParams.DebugParam9 );

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

	// Load Current Calibration from another section of external flash
	ParamMgr1.LoadCurrentCalibFromSecExtFlash( &ExtFlash1 );

	// Check error flag in housekeeping every time. If error flag is on and alarm is enable, register alarm "again".
	GlobalAlarmDetect_DoHouseKeeping();

	//RCCommCtrl.MsgHandler(&RCCommCtrl,RCCommCtrl.RxBuff,Axis[0].pCANTxInterface,Axis[0].pCANRxInterface);
//	RCCommCtrl.MsgDecoder(&RCCommCtrl);
	
	//DTC process to ExtFlash
	if ( Axis[0].ServoOn == MOTOR_STATE_OFF)
	{
        DTCStation1.DoHouseKeeping( &DTCStation1, &ExtFlash1 );
        if ( DTCStation1.State == DTC_Process_State_Clear_Failed )
        {
        	IntranetCANStation.NetWork.Tx.Data[0] = 0x7F;
        	IntranetCANStation.NetWork.Tx.Data[1] = SID_0x14_CDTCI_without_SF;
        	IntranetCANStation.NetWork.Tx.Data[2] = NRC_0x72_GPF;
        	IntranetCANStation.NetWork.Tx.LengthTotal = 3;
        	IntranetCANStation.NetWork.Tx.Status = Tx_Request;
        }
        else if ( DTCStation1.State == DTC_Process_State_Clear )
        {
        	IntranetCANStation.NetWork.Tx.Data[0] = SID_0x14_CDTCI_without_SF + POSITIVE_RESPONSE_OFFSET;
        	IntranetCANStation.NetWork.Tx.LengthTotal = 1;
        	IntranetCANStation.NetWork.Tx.Status = Tx_Request;
        	DTCStation1.State = DTC_Process_State_Idle;
        	for ( uint8_t i = 0; i < DTC_RecordNumber_Total; i++ )
        	{
        	DTCStation1.DTCStorePackge[i].DTC_Store_State = DTC_Store_State_None;
        	}
        }
	}

	// if reset register is not 0
	if(DriveFnRegs[ FN_PCU_ERR_CNT_RESET - FN_BASE ] != 0)
	{
		if(DriveFnRegs[ FN_PCU_ERR_CNT_RESET - FN_BASE ] == 1)
		{
			RCCommCtrl.AccUARTErrorCnt = 0;
		}
		else if(DriveFnRegs[ FN_PCU_ERR_CNT_RESET - FN_BASE ] == 2)
		{
			Axis[0].pCANRxInterface->AccCANErrorCnt = 0;
		}
		else if(DriveFnRegs[ FN_PCU_ERR_CNT_RESET - FN_BASE ] == 3)
		{
			RCCommCtrl.AccUARTErrorCnt = 0;
			Axis[0].pCANRxInterface->AccCANErrorCnt = 0;
		}
		// reset DriveFnRegs[ FN_PCU_ERR_CNT_RESET - FN_BASE ] to 0 anyway.
		DriveFnRegs[ FN_PCU_ERR_CNT_RESET - FN_BASE ] = 0;
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

	if( ( *Setup > Security_Level_0 ) && ( *Ena > 0 ) )
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
		}
		else if( *Ena == 3 ) // Reset the System Table and the PCUSystem Table
		{
			// Reset the System Table
			for( tCnt = 0; tCnt< SYS_PARAM_SIZE; tCnt++)
			{
				if( ( Ts->SysParamTableInfoArray[tCnt].Property & 0x0007 ) <= PcuAuthorityCtrl.SecureLvNow)
				{
					*((&pSysT->Reserved000) + tCnt) =  Ts->SysParamTableInfoArray[tCnt].Default;
				} else;
			}

			// Reset the PCUSystem Table
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

			ExtFlash1.Curr_Calib_Store.LoadCurrCalibRequest = 1;
		} else; // Do nothing
	} else;
}

void drive_DoHWOCPIRQ(void)
{
	if((Axis[0].AlarmDetect.POWER_TRANSISTOR_OC.AlarmInfo.AlarmEnable == ALARM_ENABLE) && (AlarmMgr1.State == ALARM_MGR_STATE_ENABLE))
	{
		if( PwmStation1.PwmCh[Axis[0].AlarmDetect.AxisID-1].Group->Instance != 0 )
		{
			Axis[0].AlarmDetect.RegisterAxisAlarm( &Axis[0].AlarmDetect, Axis[0].AlarmDetect.POWER_TRANSISTOR_OC.AlarmInfo.AlarmID, Axis[0].AlarmDetect.POWER_TRANSISTOR_OC.AlarmInfo.AlarmType );
		}

		DTCStation1.StatusOfDTC_Realtime[DTC_RecordNumber_P1F01_ESC_Over_current].Test_Failed = 1;

		if ( DTCStation1.DTCStorePackge[DTC_RecordNumber_P1F01_ESC_Over_current].DTC_Store_State == DTC_Store_State_None && DTCStation1.StatusOfDTC_Realtime[DTC_RecordNumber_P1F01_ESC_Over_current].Test_Failed == TRUE )
		{
			DTCStation1.DTCStorePackge[DTC_RecordNumber_P1F01_ESC_Over_current].DTC_Store_State = DTC_Store_State_Confirmed_and_wait_for_Store;
			drive_DTC_Pickup_Freeze_Frame_data( &DTCStation1, DTC_RecordNumber_P1F01_ESC_Over_current );
			DTCStation1.State = DTC_Process_State_Write;
		}
	}
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
			BootAppTrig = BOOT_DIS;
			HAL_Delay(5);
			JumpCode_ApplicationToBootloader( BOOT_ADDRESS );
		}
		else
		{
			BootAppTrig = BOOT_DIS;
		}
	}
	else if( ( DriveFnRegs[ FN_PCU_RESET_OPERATION - FN_BASE ] == BOOT_ENA ) &&	\
			 ( Axis[0].ServoOn == MOTOR_STATE_OFF ) )
	{
		if( (ParamMgr1.Session == Session_0x01_Default) || \
			(ParamMgr1.Session == Session_0x40_VehicleManufacturerSpecific) || \
			(ParamMgr1.Session == Session_0x60_SystemSupplierSpecific) )
		{
			HAL_Delay(100);
			DriveFnRegs[FN_PCU_RESET_OPERATION - FN_BASE] = 0;
			JumpCode_ApplicationToBootloader( BOOT_ADDRESS );
		}
	}
	else;

	if( ( Axis[0].ServoOn == MOTOR_STATE_OFF ) &&
		( ( IntranetCANStation.ServiceCtrlBRP.BRPECUSoftResetEnable == 1 ) || \
		  ( IntranetCANStation.ECUSoftResetEnable == 1 ) || \
		  ( ParamMgr1.ECUSoftResetEnable == 1 ) ) )
	{

		HAL_Delay(100);
		 JumpCode_ApplicationToBootloader( APP_IN_START_ADDRESS );

	}
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

	// CAN 2
	HAL_FDCAN_DeactivateNotification(&hfdcan2, FDCAN_IT_RX_FIFO0_NEW_MESSAGE);
	HAL_FDCAN_MspDeInit(&hfdcan2);
	__HAL_RCC_FDCAN_FORCE_RESET();

	//UART
//	HAL_UART_MspDeInit(&huart3);
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
    HAL_TIM_Base_Stop_IT( &htim2 );
	__HAL_TIM_DISABLE_IT( &htim2, TIM_IT_IDX );
    HAL_TIM_Base_MspDeInit( &htim2 );
	__HAL_RCC_TIM2_FORCE_RESET();

	// Timer 3
    HAL_TIM_Base_Stop_IT( &htim3 );
    HAL_TIM_Base_MspDeInit( &htim3 );
    __HAL_RCC_TIM3_FORCE_RESET();

    // Timer 6
    HAL_TIM_Base_Stop_IT( &htim6 );
    HAL_TIM_Base_MspDeInit( &htim6 );
    __HAL_RCC_TIM6_FORCE_RESET();

    // Timer 7
    HAL_TIM_Base_Stop_IT( &htim7 );
    HAL_TIM_Base_MspDeInit( &htim7 );
    __HAL_RCC_TIM7_FORCE_RESET();

    // Timer 8
    HAL_TIM_Base_Stop_IT( &htim8 );
    HAL_TIM_Base_MspDeInit( &htim8 );
    __HAL_RCC_TIM8_FORCE_RESET();

	// Timer 20
	HAL_TIM_Base_Stop_IT( &htim20 );
	HAL_TIM_Base_MspDeInit( &htim20 );
	__HAL_RCC_TIM20_FORCE_RESET();

	/*
	 * DeInit ADC & DAC
	 */
	//ADC2
	HAL_ADCEx_InjectedStop_IT(&hadc2);
	HAL_ADC_MspDeInit(&hadc2);
	__HAL_RCC_ADC12_FORCE_RESET();
	// ADC 3
	HAL_ADCEx_InjectedStop_IT(&hadc3);
	HAL_ADC_MspDeInit(&hadc3);

	// ADC 4
	HAL_ADCEx_InjectedStop_IT(&hadc4);
	HAL_ADC_MspDeInit(&hadc4);
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
