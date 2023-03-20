/*
 * Protocol.h
 *
 *  Created on: 2020年5月27日
 *      Author: Mike.Wen.SFW
 */

#if BME
#ifndef INC_PROTOCOL_H_
#define INC_PROTOCOL_H_

#include "stdio.h"
#include "string.h"
#include "CANDrive.h"
#include "ICANInterface.h"
#include "math.h"

typedef enum
{
	SLM_CMD_STOP = 0,
	SLM_CMD_EXE,
}
E_SLM_CMD;

#define GEAR_SHIFT_CMD_P 0x30
#define GEAR_SHIFT_CMD_D 0x36
#define GEAR_SHIFT_CMD_R 0x39

typedef enum
{
	STATUS_FB_SHUTDOWN_FINISH 	= 0x00,
	STATUS_FB_INITIAL 			= 0x01,
	STATUS_FB_ENABLE			= 0x0A,
	STATUS_FB_SHUTDOWN_START 	= 0x0F,	//Status should not exceed 0x0F !!
}PcuStatusFb_e;

#define IDX_MOTOR_NTC				0
#define IDX_PCU_NTC					1
#define IDX_THROTTLE_WIRE			2


#define RATIO_CONST  31.831f
#define RATIO_RPM_2_RPS		0.1164f	// = 2*PI()/60/0.9 where 0.9 is a constant efficiency
typedef struct
{
	uint8_t EnableReqByVCU		:1;
	uint8_t ShutdownReqByBMS	:1;
	uint8_t	Rsvd				:6;
}ProtocolReqFlags_t;

typedef struct
{
	uint8_t Status		:4;
	uint8_t AlarmFlag	:1;
	uint8_t	Rsvd		:6;
}ProtocolFbkFlags_t;

#define IDX_REQ_FLAG	0
#define IDX_FBK_FLAG	0
/*=========================================
 * Definition of PcuFaultFlagA
 =========================================*/
typedef struct
{
	uint8_t	MotorOTP			:1;
	uint8_t	PcuOTP				:1;
	uint8_t	ThrottleAbnormal	:1;
	uint8_t	MotorFault			:1;
	uint8_t	OCP					:1;
	uint8_t	UVP					:1;
	uint8_t	OVP					:1;
	uint8_t	Rsvd7				:1;
} PcuFaultFlagA_t;

typedef union
{
	uint8_t	All;
	PcuFaultFlagA_t	Bits;
} PcuFaultFlagA_u;

/*======================================
 *  Enum definition
 *  ==================================*/
typedef enum{
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
	IDX_DC_LIMIT_DCBUS_USE

}EnumDbFloatIndex;

typedef enum{
	BMS_MAIN_SM_OFF = 0,
	BMS_MAIN_SM_INIT,
	BMS_MAIN_SM_IDLE,
	BMS_MAIN_SM_ACTIVE,
	BMS_MAIN_SM_ERROR,
	BMS_MAIN_SM_PROGRAMMING,
	BMS_MAIN_SM_SHUTDOWN,
}EnumBmsMainSm;

typedef enum{
	BMS_PRCH_SM_OFF = 0,
	BMS_PRCH_SM_PRCH,
	BMS_PRCH_SM_FINISH,
	BMS_PRCH_SM_DONE,
}EnumBmsPrchSm;




/*=========================================
 * Definition of BMS_State_Monitor(0x152)
 =========================================*/
typedef struct{
	uint8_t PwrFactorDCHG_H6 	:1;
	uint8_t PwrFactorCHG  		:7;
	uint8_t BMSstate_H1			:2;
	uint8_t PwrFactorDCHG_L  	:6;
	uint8_t BMSPackQuantity 	:3;
	uint8_t BMSMultiInterconReady 	:1;
	uint8_t WakeupMethod		:2;
	uint8_t BMSFETstate			:1;
	uint8_t BMSstate_L			:1;
}StructBME060RxID152h;


/*=========================================
 * Definition of BMS_State_Monitor(0x151)
 =========================================*/
typedef struct{
	uint8_t BMSCurrentNow_H8 	:8;
	uint8_t BMSCurrentNow_L  	:8;
	uint8_t BMSVoltNow_H5		:8;
	uint8_t Resvd1 				:3;
	uint8_t BMSVoltNow_L  		:5;
	uint8_t Resvd2 				:3;
	uint8_t BMSRecuperationAllow:1;
	uint8_t BMSCHGFinished		:1;
	uint8_t BMSWarning			:1;
	uint8_t BMSSafetyFailure	:1;
	uint8_t BMSFailure			:1;
}StructBME060RxID151h;

/*=========================================
 * Definition of BMS_CV_Info(0x361)
 =========================================*/
typedef struct{
	uint8_t BMSCurrentCHG		:8;
	uint8_t BMSCurrentDCHG_H1	:8;
	uint8_t BMSVoltCHG_H6		:7;
	uint8_t BMSCurrentDCHG_L	:1;
	uint8_t BMSVoltDCHG_H11		:2;
	uint8_t BMSVoltCHG_L		:6;
	uint8_t BMSVoltDCHG_H3		:8;
	uint8_t BMSSoc_H2			:5;
	uint8_t BMSVoltDCHG_L		:3;
	uint8_t Resvd				:6;
	uint8_t BMSSoc_L			:2;
}StructBME060RxID361h;



/*=========================================
 * Definition of BMS_State_machine_01(0x401)
 =========================================*/
typedef struct{
	uint8_t 	Resvd0;
	uint8_t 	Resvd1;
	uint8_t 	BmsMainSM;
	uint8_t 	BmsPrchSM;
	uint8_t		WakeUpReason;
	uint8_t		ErrorCode_L;
	uint8_t		ErrorCode_H;
	uint8_t		Resvd7;
}StructBME060RxID401h;



/*=========================================
 * Definition of VCU_BAT_Handling(0x150)
 =========================================*/
typedef struct{

	uint8_t Resvd				:1;
	uint8_t VcuEmergencyShutDownReq	:1;
	uint8_t VcuInterConnReq 	:6;
	uint8_t Resvd2				:2;
	uint8_t VcuShutdownReq 		:6;

}StructBME060TxID150h;


/*=========================================
 * Definition of VCU_BMS_Control_01(0x500)
 =========================================*/
typedef struct{

	uint8_t Resvd0;
	uint8_t Resvd1;
	uint8_t ConnReq;
	uint8_t DisconnReq;
	uint8_t ShutdownReq;
	uint8_t Resvd5;
	uint8_t Resvd6;
	uint8_t Resvd7;
}StructBME060TxID500h;

typedef struct{
	uint8_t MotorTemp;
	uint8_t EscMos1Temp;
	uint8_t EscMos2Temp;
	uint8_t EscCapTemp;
	uint8_t	OpCmdAndFinal;
	uint8_t ThrottleFinal;
	uint16_t ThrottleRaw;
}StructBME060TxID700h;

typedef struct{
	uint16_t DcVoltU16;
	int16_t MotorRpmI16;
	uint8_t EscState;
	uint8_t SafetySensor;
	uint8_t BmsMainState;
	uint8_t BmsPrchState;
}StructBME060TxID701h;
typedef struct{

	int16_t IdCmdI16;
	int16_t IqCmdI16;
	int16_t IdFbkI16;
	int16_t IqFbkI16;
}StructBME060TxID702h;

typedef struct{
	int16_t AcLimitCmd;
	int16_t AcLimitTq;
	int16_t DcLimitCmd;
	int16_t DcLimitTq;
}StructBME060TxID703h;

typedef struct{
	int16_t VdCmdI16;
	int16_t VqCmdI16;
	int16_t PerformanceTqI16;
	uint8_t Error01[2];
}StructBME060TxID704h;

typedef struct{
	uint8_t Error29[8];
}StructBME060TxID705h;

typedef struct{
	uint16_t 	_361hCurrentCHGLimit;
	int16_t 	_361hCurrentDCHGLimit;
	float 		_361hVoltCHG;
	float 		_361hVoltDCHG;
	uint8_t 	_361hSOC;
	float	 	_151hBatCurrentNow;
	float 		_151hBatVoltNow;
	uint8_t		_151hRecuperationAllowed;
	uint8_t		_151hChargeFinished;
	uint8_t		_151hWarning;
	uint8_t		_151hSafetyFailure;
	uint8_t		_151hFailure;
	uint8_t		_152hPwrFactorCHG;
	uint8_t		_152hPwrFactorDCHG;
	uint8_t		_152hPackQuantity;
	uint8_t		_152hInterConnReady;
	uint8_t		_152hWakeupMethod;
	uint8_t		_152hFETStatus;
	uint8_t		_152hBMSState;
}StructBMSInform;

typedef struct{
	uint8_t 	_401hBmsMainStateMachine;
	uint8_t 	_401hBmsPrchStateMachine;
	uint8_t		_401hWakeupReason;
	uint16_t 	_401hBmsErrorCode;
}StructBMSInformV0617;



/*=========================================
 * Definition of TxRxDATA
 =========================================*/
typedef union
{
	StructBME060RxID401h	ID0x401;
	uint8_t DataU8[8];
	int16_t DataI16[4];
	uint32_t DataU32[2];
} UnionRxDataV0617;

typedef union
{

	StructBME060TxID500h	ID0x500;
	StructBME060TxID700h	ID0x700;
	StructBME060TxID701h	ID0x701;
	StructBME060TxID702h	ID0x702;
	StructBME060TxID703h	ID0x703;
	StructBME060TxID704h	ID0x704;
	StructBME060TxID705h	ID0x705;
	uint8_t DataU8[8];
	int16_t DataI16[4];
	uint32_t DataU32[2];
} UnionTxDataV0617;


typedef union
{
	StructBME060RxID151h	ID0x151;
	StructBME060RxID152h	ID0x152;
	StructBME060RxID361h	ID0x361;
	uint8_t DataU8[8];
	int16_t DataI16[4];
	uint32_t DataU32[2];
} UnionRxData;

typedef union
{

	StructBME060TxID150h	ID0x150;
	uint8_t DataU8[8];
	int16_t DataI16[4];
	uint32_t DataU32[2];
} UnionTxData;


uint8_t BME060CAN_RxDataTranslateV0617( uint32_t pIdIn, uint8_t *pDataIn, STRUCT_CANRxInterface *v, STRUCT_CANTxInterface *t );
uint8_t BME060CAN_TxDataTranslateV0617( uint32_t pIdIn, uint8_t *pDataIn, STRUCT_CANTxInterface *v, STRUCT_CANRxInterface *r );

uint8_t BME060CAN_RxDataTranslate( uint32_t pIdIn, uint8_t *pDataIn, STRUCT_CANRxInterface *v, STRUCT_CANTxInterface *t );
uint8_t BME060CAN_TxDataTranslate( uint32_t pIdIn, uint8_t *pDataIn, STRUCT_CANTxInterface *v, STRUCT_CANRxInterface *r );

extern StructBMSInform Test;
extern const CANProtocol ExtranetInformInSystemTableExample;
extern const CanIdConfig_t LscCanIdTableExtra[];

#endif /* INC_PROTOCOL_H_ */
#endif /* BME */
