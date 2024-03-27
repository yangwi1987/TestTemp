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
#define DEFAULT_DC_LIMIT 1000.0f

typedef enum {
	RECEIVED_VCU_ID_1 = (uint8_t)1,
	RECEIVED_VCU_ID_2 = (uint8_t)2,
	RECEIVED_BAT_ID_1 = (uint8_t)4,
	RECEIVED_BAT_ID_2 = (uint8_t)8,
} RECEIVED_CAN_ID_ENUM;

typedef enum {
	PCU_SHIFT_P = 0,
	PCU_SHIFT_D,
} ENUM_SHIFT;

typedef enum {
	FourQ_Drive = 1,
	FourQ_Backroll,
	FourQ_Reverse,
	FourQ_Regen,
} ENUM_4QuadStatus;

typedef enum PcuState_e
{
	PCU_STATE_INITIAL = 0,
	PCU_STATE_READY,
	PCU_STATE_OUTPUT,
	PCU_STATE_STANDBY,
	PCU_STATE_SHUTDOWN_START,
	PCU_STATE_SHUTDOWN_FINISH,
	PCU_STATE_POWER_OFF,
	PCU_STATE_ERROR,
} PcuState_t;

typedef enum {
	PcuCmd_Idle = 0,
	PcuCmd_Enable,
	PcuCmd_Shutdown,
} PcuCmd_e;

typedef enum {
	ServoOnCmd_ServoOff = 0,
	ServoOnCmd_ServoOn,
} ServoOnCmd_e;

typedef enum PowerStateMachine_e
{
	PWR_SM_INITIAL,
	PWR_SM_POWER_ON,
	PWR_SM_POWER_OFF,
	PWR_SM_WAIT_FOR_RESET,
} PowerStateMachine_t;

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
	float	BatCurrentDrainLimit;
	uint8_t ThrottleCmd;
	uint8_t OutputModeCmd;
	uint8_t ServoOnCmd;
	uint8_t ReceivedCANID;
	uint16_t AccCANErrorCnt;
	BmsReportInfo_t BmsReportInfo;
} STRUCT_CANRxInterface;

typedef enum TxInterfaceDbgIdx_e
{
  TX_INTERFACE_DBG_IDX_FOIL_POSITION = 0,
  TX_INTERFACE_DBG_IDX_ERROR_FLAG = 1, // include critical alarm, non-critical alarm, and warning

  TX_INTERFACE_DBG_IDX_BMS_COMM_ENABLE = 5,
  TX_INTERFACE_DBG_IDX_LOG_ENABLE_FLAG = 6,
  TX_INTERFACE_DBG_IDX_LOG_SAMPLE_FLAG = 7,
  TX_INTERFACE_DBG_IDX_MAX = 64,
} TxInterfaceDbgIdx_t;

typedef enum DebugFloatIdx_e
{
  IDX_MOTOR0_TEMP = 0,
  IDX_MOS1_TEMP,
  IDX_MOS2_TEMP,
  IDX_CAP_TEMP,
  IDX_MOTOR1_TEMP,
  IDX_MOTOR2_TEMP,
  IDX_THROTTLE_RAW,
  IDX_THROTTLE_FINAL,

  IDX_DC_VOLT,
  IDX_MOTOR_RPM,

  IDX_ID_CMD,
  IDX_IQ_CMD,
  IDX_ID_FBK,
  IDX_IQ_FBK,

  IDX_AC_LIMIT_TQ,
  IDX_AC_LIMIT_CMD,
  IDX_DC_LIMIT_TQ,
  IDX_DC_LIMIT_CMD,

  IDX_PERFROMANCE_TQ,
  IDX_VD_CMD,
  IDX_VQ_CMD,
  IDX_ACC_PEDAL1_VOLT,
  IDX_EA5V,

  IDX_INSTANT_AC_POWER,
  IDX_AVERAGE_AC_POWER,
  IDX_REMAIN_TIME,

  IDX_E5V,
  IDX_ES5V,

  IDX_IU_FBK,
  IDX_IV_FBK,
  IDX_IW_FBK,
  IDX_PREC,

  IDX_ACC_PEDAL2_VOLT,
  IDX_S13V8,

  IDC_DEBUG_FLOAT_MAX = 64, /* no more than this value*/
} DebugFloatIdx_t;

typedef struct
{
	// debug
	float Debugf[IDC_DEBUG_FLOAT_MAX];
	uint8_t DebugU8[TX_INTERFACE_DBG_IDX_MAX];
	uint8_t DebugError[10];
	// debug done
    uint8_t VehicleState;    //TODO: use defined enum?
    uint8_t WarningFlag;
    uint8_t AlarmFlag;
    uint8_t OutputMode;
    uint8_t LimpHomeSrc;
    uint8_t DeratingSrc;
	uint8_t ShutDownReq;
	uint8_t InvState;
	uint8_t ServoOnOffState;
	uint16_t HWID[2];
	BmsCtrlCmd_t BmsCtrlCmd;
	AlarmStack_t *pAlarmStack;
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
	DEFAULT_DC_LIMIT,  /* BatCurrentDrainLimit */\
	0,      /* ThrottleCmd */\
	0,      /* OutputModeCmd */\
	0,      /* ServoOnCmd */\
	0,      /* ReceivedCANID */\
	0,      /* AccCANErrorCnt */\
	BMS_REPORT_INFO_DEFAULT,  /* BmsReportInfo */\
}\

#define CANTXINFO_DEFAULT { \
	{0.0f}, /*Debugf*/\
	{0}, /*DebugU8*/\
	{0},/*DebugError*/\
	0, /*VehicleState*/\
	0, /*WarningFlag*/\
	0, /*AlarmFlag*/\
	0, /*OutputMode*/\
	0, /*LimpHomeSrc*/\
	0, /*DeratingSrc*/\
	0, /*ShutDownReq*/\
	0, /*InvState*/\
	0, /*ServoOnOffState*/\
	{0},/*HWID[2]*/\
	{0}, /*BmsCtrlCmd*/\
	0, /**pAlarmStack*/\
}\

#endif /* INC_ICANINTERFACE_H_ */
