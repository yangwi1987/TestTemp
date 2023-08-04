/*
 * ICANInterface.h
 *
 *  Created on: 2020年5月27日
 *      Author: Mike.Wen.SFW
 */

#ifndef INC_ICANINTERFACE_H_
#define INC_ICANINTERFACE_H_

#include "stdio.h"
#include "AlarmMgr.h"
#include "Protocol.h"

#define ID_MATCH	0
#define ID_NO_MATCH	1

#define TX_ID_NUM	10
#define REPORT_ALARM_DIVIDER 10

typedef enum {
	RECEIVED_VCU_ID_1 = (uint8_t)1,
	RECEIVED_VCU_ID_2 = (uint8_t)2,
	RECEIVED_BAT_ID_1 = (uint8_t)4,
	RECEIVED_BAT_ID_2 = (uint8_t)8,
} RECEIVED_CAN_ID_ENUM;

typedef enum {
	PcuShiftP = 0,
	PcuShiftD,
} ENUM_SHIFT;

typedef enum {
	FourQ_Drive = 1,
	FourQ_Backroll,
	FourQ_Reverse,
	FourQ_Regen,
} ENUM_4QuadStatus;

typedef enum {
	PcuState_Inital = 0,
	PcuState_Ready,
	PcuState_SERVO_ON,
	PcuState_SERVO_OFF,
	PcuState_Shutdown_Start,
	PcuState_Shutdown_Finish,
	PcuState_Error,
} ENUM_PcuState;

typedef enum {
	PcuCmd_Idle = 0,
	PcuCmd_Enable,
	PcuCmd_Shutdown,
} PcuCmd_e;

typedef enum {
	ServoOnCmd_ServoOff = 0,
	ServoOnCmd_ServoOn,
} ServoOnCmd_e;

typedef enum{
	PowerOnOff_Initial,
	PowerOnOff_Ready,
	PowerOnOff_ShutdownStart,
	PowerOnOff_NormalShutdown,
	PowerOnOff_EmergencyShutDown,
	PowerOnOff_WaitForReset,
	PowerOnOff_Error,
}PowerOnOffSeq_e;

/*=========================================
 * Definition of Authendata
 =========================================*/
typedef struct
{
	uint8_t *pdata;
	uint8_t Size;
} STRUCT_Authendata;

/*=========================================
 * Definition of Short Status
 =========================================*/
typedef struct
{
	uint16_t	MotorNTC:1;
	uint16_t	CoolantNTC:1;
	uint16_t	MOSNTC:1;
	uint16_t	Throttle:1;
	uint16_t	Sensor4:1;
	uint16_t	Sensor5:1;
	uint16_t	Sensor6:1;
	uint16_t	Sensor7:1;
	uint16_t	Sensor8:1;
	uint16_t	Sensor9:1;
	uint16_t	Sensor10:1;
	uint16_t	Sensor11:1;
	uint16_t	Sensor12:1;
	uint16_t	Sensor13:1;
	uint16_t	Sensor14:1;
	uint16_t	Sensor15:1;
} STRUCT_ShortStatus;

typedef union
{
	uint16_t U16All;
	STRUCT_ShortStatus bits;
} UINON_ShortStatus;

/*=========================================
 * Definition of Break Status
 =========================================*/
typedef struct
{
	uint16_t	MotorNTC:1;
	uint16_t	CoolantNTC:1;
	uint16_t	MOSNTC:1;
	uint16_t	Throttle:1;
	uint16_t	Sensor4:1;
	uint16_t	Sensor5:1;
	uint16_t	Sensor6:1;
	uint16_t	Sensor7:1;
	uint16_t	Sensor8:1;
	uint16_t	Sensor9:1;
	uint16_t	Sensor10:1;
	uint16_t	Sensor11:1;
	uint16_t	Sensor12:1;
	uint16_t	Sensor13:1;
	uint16_t	Sensor14:1;
	uint16_t	Sensor15:1;
} STRUCT_BreakStatus;

typedef union
{
	uint16_t U16All;
	STRUCT_BreakStatus bits;
} UINON_BreakStatus;

/*=========================================
 * Definition of Fault Status
 =========================================*/
typedef struct
{
	uint32_t	MotorOT			:1;
	uint32_t	CoolantOT		:1;
	uint32_t	MOS_OT			:1;
	uint32_t	ThrottleFault	:1;
	uint32_t	MotorFault	:1;
	uint32_t	InitialError:1;
	uint32_t	RunningError:1;
	uint32_t	OVP			:1;
	uint32_t	UVP		:1;
	uint32_t	OCP		:1;
	uint32_t	PSBError:1;
	uint32_t	PumpErrorDerating:1;
	uint32_t	HWerror		:1;
	uint32_t	OSP			:1;
	uint32_t	PhaseLost	:1;
	uint32_t	MotorStall	:1;
	uint32_t	FwError 	 	:1;
	uint32_t    ThrottleZero 	:1;
	uint32_t	FinalDerating:1;
	uint32_t	DynamicDerating:1;
	uint32_t	BatCmdDerating:1;
	uint32_t	Rsvd	:11;
} STRUCT_FaultStatus;

typedef union
{
	uint16_t U32All;
	STRUCT_FaultStatus bits;
} UINON_FaultStatus;

/*=========================================
 * Definition of CtrlIO
 =========================================*/
typedef struct
{
	uint16_t	Ignite			:1;
	uint16_t	MidStand		:1;
	uint16_t	SideStand		:1;
	uint16_t	Reversebutton	:1;
	uint16_t	Reversebutton2	:1;
	uint16_t	ReservedBit1	:1;
	uint16_t	ReservedBit2	:1;
	uint16_t	ReservedBit3	:1;
	uint16_t	Reserved		:8;
} STRUCT_CtrlIO;

typedef union
{
	uint16_t All;
	STRUCT_CtrlIO Bits;
} UNION_CtrlIO;

/*=========================================
 * Definition of Precharge
 =========================================*/
typedef struct
{
	uint8_t PrchMOS :1;
	uint8_t BypassMOS :1;
	uint8_t Reserved6 :6;
} STRUCT_PRCH_CTRL;

typedef union
{
	uint8_t All;
	STRUCT_PRCH_CTRL bit;
} UNION_PRCH_CTRL;

/*=========================================
 * Definition of Bat Condition
 =========================================*/
typedef struct
{
	uint16_t	OverVolt :1;
	uint16_t	UnderVolt :1;
	uint16_t	OverCharge :1;
	uint16_t	OverDischarge :1;
	uint16_t	CircuitShort :1;
	uint16_t	OverTmp :1;
	uint16_t	Undertemp :1;
	uint16_t	Unbalance :1;
	uint16_t	reserved :8;
} STRUCT_BAT_CONDOTION;

typedef union
{
	uint16_t All;
	STRUCT_BAT_CONDOTION Bits;
} UNION_BAT_CONDOTION;

typedef struct
{
	uint8_t CCSMode:1;
	uint8_t SLMMode:1;
	uint8_t Reserved2;
	uint8_t Reserved3;
	uint8_t Reserved4;
	uint8_t Reserved5;
	uint8_t Reserved6;
	uint8_t Reserved7;
} STRUCT_DRIVE_STATE;

typedef union
{
	uint8_t All;
	STRUCT_DRIVE_STATE Bits;
} UNION_DRIVE_STATE;


typedef struct{
	uint8_t WarningFlag 	:1;
	uint8_t SafetyFailFlag	:1;
	uint8_t FailFlag		:1;
	uint8_t Resvd			:5;
}StructBATErrorFlag;

typedef union{
	uint8_t All;
	StructBATErrorFlag bit;
}UnionBATErrorFlag;

typedef struct{
	uint8_t MainSm;		/* main State machine */
	uint8_t PrchSm;		/* precharge state machine */
	uint8_t Resvd;
} StructBMSStatus;


/*=========================================
 * BMS Interface
 =========================================*/
typedef struct{
	UnionBATErrorFlag Error;
	StructBMSStatus Status;
}StructBMSInfo;

/*=========================================
 * Interface
 =========================================*/
typedef struct
{
	int32_t	BatCurrentDrainLimit0P1A;
	int32_t	MEPowerOutputLimit0P1W;
	uint32_t RTCData;
	uint8_t ThrottleCmd;
	uint8_t PowerLevel;
	uint8_t RcConnStatus; // 1: means RC is connected to RF now, 0: RC is disconnected now.
	uint8_t ShiftCmd;
	uint8_t OutputModeCmd;
	uint8_t PcuStateCmd;
	uint8_t ServoOnCmd;
	uint8_t UrgentShutdown;
	uint8_t ExtPumpStatus;
	uint8_t ReceivedCANID;
	uint16_t AccCANErrorCnt;
	UNION_CtrlIO ExtSignalCmd;
	UNION_PRCH_CTRL PrchCtrlFB;
	STRUCT_Authendata RxAuthData[2];
	uint8_t Buffer[4];
	BmsReportInfo_t BmsReportInfo;
} STRUCT_CANRxInterface;

typedef enum TxInterfaceDbgIdx_e
{
  TX_INTERFACE_DBG_IDX_FOIL_POSITION = 0,
  TX_INTERFACE_DBG_IDX_ERROR_FLAG = 1, // include critical alarm, non-critical alarm, and warning
  TX_INTERFACE_DBG_IDX_LED_CTRL_CMD	= 4,
  TX_INTERFACE_DBG_IDX_BMS_COMM_ENABLE = 5,
  TX_INTERFACE_DBG_IDX_LOG_ENABLE_FLAG = 6,
  TX_INTERFACE_DBG_IDX_LOG_SAMPLE_FLAG = 7,
  TX_INTERFACE_DBG_IDX_MAX = 8,
} TxInterfaceDbgIdx_t;

typedef enum DebugFloatIdx_e
{
  IDX_AC_LIMIT_TQ =0,
  IDX_AC_LIMIT_CMD,
  IDX_DC_LIMIT_TQ,
  IDX_DC_LIMIT_CMD,
  IDX_PERFROMANCE_TQ,
  IDX_VD_CMD,
  IDX_VQ_CMD,
  IDX_MOTOR_RPM,
  IDX_DC_VOLT,
  IDX_THROTTLE_RAW,
  IDX_THROTTLE_FINAL,
  IDX_FOIL_SENSOR_VOLT,
  IDX_DC_LIMIT_CANRX_DC_CURR,
  IDX_RESERVERD,
  IDX_DC_LIMIT_DCBUS_REAL,
  IDX_DC_LIMIT_DCBUS_USE,
  IDX_INSTANT_AC_POWER,
  IDX_AVERAGE_AC_POWER,
  IDX_REMAIN_TIME,

  IDC_DEBUG_FLOAT_MAX = 24, /* no more than this value*/
} DebugFloatIdx_t;

typedef struct
{
	// debug
	uint8_t Th_Raw_Percent;
	uint8_t Th_DI;
	uint8_t Th_Out_Percent;
	uint8_t FourQuadState;
	int16_t Torquecmd;
	float Id_cmd;
	float Iq_cmd;
	float Id_fbk;
	float Iq_fbk;
	float Debugf[IDC_DEBUG_FLOAT_MAX];
	uint8_t DebugU8[TX_INTERFACE_DBG_IDX_MAX];
	uint8_t DebugError[10];
	// debug done
	uint16_t NowAlarmID;
	int16_t MotorRpm;
	int16_t	MotorPower0P1KW;
	int16_t	IU0P1A;
	int16_t	IV0P1A;
	int16_t	IW0P1A;
	int16_t IDcBus0P1A;
	int16_t VoltDcBu0P1V;
	FoilPos_t FoilPos;
	uint8_t DeratingSrc;
	uint8_t ShiftReport;
	uint8_t _4Quad;
	uint8_t PcuStateReport;
	uint16_t ThrottleADC;
	uint16_t ThrottlePercent;
	uint8_t RegenModeStatus;
	int16_t PhaseCurrent;
	int16_t LeadingAngle;
	UINON_ShortStatus ShortFlag;
	UINON_BreakStatus BreakFlag;
	UINON_FaultStatus FaultFlag;
	UNION_CtrlIO ExtSignalReport;
	UNION_PRCH_CTRL PrchCtrlReqst;
	int16_t	NTCTemp[8];
	STRUCT_Authendata TxAuthData[2];
	uint16_t AlarmSendIdx;
	AlarmStack_t *pAlarmStack;
	float TqOutNow;
	float WheelSpdKmh;

	uint8_t Buffer[4];
	uint8_t ShutDownReq;
	uint16_t AlarmDivider;
	UNION_DRIVE_STATE DriveState;

} STRUCT_CANTxInterface;

typedef uint8_t (*pRxTranslate)( uint32_t, uint8_t*, void *, void *);
typedef uint8_t (*pTxTranslate)( uint32_t, uint8_t*, void *, void *);

typedef struct {
	uint16_t				TxPeriodMs;
	uint16_t				PeriodicTxIDNumber;
	uint32_t				PeriodicTxIDTable[10];
	pRxTranslate			RxTranslate;
	pTxTranslate			TxTranslate;
} CANProtocol;

#define CANRXINFO_DEFAULT { \
	10000,  /* BatCurrentDrainLimit0P1A */\
	10000,  /* MEPowerOutputLimit0P1W */\
	0,      /* RTCData */\
	0,      /* ThrottleCmd */\
	1,      /* PowerLevel */\
	0,      /* RcConnStatus */\
	0,      /* ShiftCmd */\
	0,      /* OutputModeCmd */\
	0,      /* PcuStateCmd */\
	0,      /* ServoOnCmd */\
	0,      /* UrgentShutdown */\
	0,      /* ExtPumpStatus */\
	0,      /* ReceivedCANID */\
	0,      /* AccCANErrorCnt */\
	{0},    /* ExtSignalCmd */\
	{0},    /* PrchCtrlFB */\
	{{NULL,0},{NULL,0}},      /* RxAuthData[2] */\
	{0,0,0,0},                /* Buffer[4] */\
	BMS_REPORT_INFO_DEFAULT,  /* BmsReportInfo */\
}\

#define CANTXINFO_DEFAULT { \
		0, /* Th_Raw_Percent */ \
		0, /* Th_DI */ \
		0, /* Th_Out_Percent */ \
		0, /* FourQuadState */ \
		0, /* Torquecmd */ \
		0.0f, /* Id_cmd */ \
		0.0f, /* Iq_cmd */ \
		0.0f, /* Id_fbk */ \
		0.0f, /* Iq_fbk */ \
		{0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f}, /* Debugf[16] */ \
		{0,0,0,0,0,0,0,0,}, /* debug done */ \
		{0,0,0,0,0,0,0,0,0,0},	/*uint8_t DebugError[10]*/ \
		0, /* NowAlarmID */ \
		0, /* MotorRpm */ \
		0, /* MotorPower0P1KW */ \
		0, /* IU0P1A */ \
		0, /* IV0P1A */ \
		0, /* IW0P1A */ \
		0, /* IDcBus0P1A */ \
		0, /* VoltDcBu0P1V */ \
		FOIL_POS_PADDLE, /*FoilPos*/ \
		0,	/* DeratingSrc*/\
		0, /* ShiftReport */ \
		0, /* _4Quad */ \
		0, /* PcuStateReport */ \
		0, /* ThrottleADC */ \
		0, /* ThrottlePercent */ \
		0, /* RegenModeStatus */ \
		0, /* PhaseCurrent */ \
		0, /* LeadingAngle */ \
		{0}, /* ShortFlag */ \
		{0}, /* BreakFlag */ \
		{0}, /* FaultFlag */ \
		{0}, /* ExtSignalReport */ \
		{0}, /* rchCtrlReqst */ \
		{0,0,0,0,0,0,0,0}, /* NTCTemp[8] */ \
		{{NULL,0},{NULL,0}}, /* TxAuthData[2] */ \
		0, /* AlarmSendIdx */ \
		0, /* *pAlarmStack */ \
		0.0f, /* TqOutNow */ \
		0.0f, /* WheelSpdKmh */ \
		{0,0,0,0}, /* Buffer[4] */ \
		0, \
}

#endif /* INC_ICANINTERFACE_H_ */
