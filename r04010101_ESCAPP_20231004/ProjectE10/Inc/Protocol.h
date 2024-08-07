/*
 * Protocol.h
 *
 *  Created on: 2020年5月27日
 *      Author: Mike.Wen.SFW
 */

#if E10
#ifndef INC_PROTOCOL_H_
#define INC_PROTOCOL_H_

#include "stdio.h"
#include "string.h"
#include "CANDrive.h"
#include "math.h"
#include "ConstantParamAndUseFunction.h"


/* ==== macro for CANTX IDs for transmit ==== */

#define CANTXID_INV_LOG_INFO_0  0x720   /*Debug information for Inv develop 00*/
#define CANTXID_INV_LOG_INFO_1  0x721   /*Debug information for Inv develop 01*/
#define CANTXID_INV_LOG_INFO_2  0x722   /*Debug information for Inv develop 02*/
#define CANTXID_INV_LOG_INFO_3  0x723   /*Debug information for Inv develop 03*/
#define CANTXID_INV_LOG_INFO_4  0x724   /*Debug information for Inv develop 04*/
#define CANTXID_INV_LOG_INFO_5  0x725   /*Debug information for Inv develop 05*/
#define CANTXID_INV_LOG_INFO_6  0x726   /*Debug information for Inv develop 06*/
#define CANTXID_INV_LOG_INFO_7  0x727   /*Debug information for Inv develop 07*/
#define CANTXID_INV_LOG_INFO_8  0x728   /*Debug information for Inv develop 08*/
#define CANTXID_INV_LOG_INFO_9  0x729   /*Debug information for Inv develop 09*/

/* ==== macro for CANRX IDs for receive ==== */
#define CANRXID_BMS_FILTER_START_01   0x402   /* info shows BMS status */
#define CANRXID_BMS_FILTER_START_02   0x403   /* info shows BMS status */

#define CAN_TX_CRI_ALARM_MASK 0x01
#define CAN_TX_NON_CRI_ALARM_MASK 0x02
#define CAN_TX_WARNING_MASK 0x04


/*========CAN RX ID definition========*/

#define CAN_ID_BMS_MASK     0xFFFF00FF
#define CAN_ID_BMS_FILTER   0x08020017

#define CAN_ID_DEV_CMD_START	0x710
#define CAN_ID_DEV_CMD_END		0x71F

/*======================================
 *  Enum definition
 *  ==================================*/


/*this value should always < 15*/
typedef enum
{
	ST_INVR_OFF = 0x00,
	ST_INVR_STARTUP,
	ST_INVR_STANDBY,
	ST_INVR_ENERGIZED,
	ST_INVR_TQ_GENERATOR,
	ST_INVR_CHARGE,
	ST_INVR_PRE_SHUTDOWN,
	ST_INVR_SHUTDOWN,
	ST_INVR_ERROR,
} StInvr_e;
/*=========================================
 * Definition of id512_200_INV CAN Tx Msg to SWM <need to be modified>
 =========================================*/
typedef struct
{
	struct
	{
		uint8_t stLedSoc1 : 1;
		uint8_t stLedSoc2 : 1;
		uint8_t stLedSoc3 : 1;
		uint8_t stLedSoc4 : 1;
		uint8_t stLedSoc5 : 1;
		uint8_t stLedMil  : 1;
		uint8_t stLedBoost : 1;
		uint8_t stLedDrivInd : 1;
	}Bits;
	uint8_t All;
} id512_200_INV_Byte0_u;

typedef struct
{
	id512_200_INV_Byte0_u Byte0;
	uint8_t Byte1;
	uint8_t Byte2;
	uint8_t Byte3;
	uint8_t Byte4;
	uint8_t Byte5;
	uint8_t Byte6;
	uint8_t Byte7;
} CanTxMsg_id512_200_INV_t;

typedef struct
{
  uint8_t Motor0Temp;        /* unit: 'C, offset: -40 */
  uint8_t InvMos1Temp;      /* unit: 'C, offset: -40 */
  uint8_t InvMos2Temp;      /* unit: 'C, offset: -40 */
  uint8_t InvCapTemp;       /* unit: 'C, offset: -40 */
  uint8_t Motor1Temp;        /* unit: 'C, offset: -40 */
  uint8_t Motor2Temp;        /* unit: 'C, offset: -40 */
  uint8_t ThrottleRaw;      /* unit 1%, throttle command received from RC */
  uint8_t ThrottleFinal;    /* unit 1%, throttle command handled by throttle mapping strategy */
} CanTxMsg_InvLogInfo0_t;

typedef struct
{
  uint16_t DcVoltU16;     /* unit: 0.1V */ 
  int16_t MotorRpmI16;    /* unit: rpm */
  uint8_t VehicleState :4;    /* refer to "VehicleMainState" defined in "Drive.c" */
  uint8_t WarnFlag :2;    /* 1: warning detected, 0: nothing */
  uint8_t AlarmFlag :2;   /* 1: Alarm detected, 0: nothing */
  uint8_t OutputMode:4;     /* 0: limpHome mode , 1: Normal mode, 2: Boost Mode*/
  uint8_t InvState:4;     /* refer to "INVMainState" defined in "Drive.c" */
  uint8_t LimpHomeFlag;    /* 1: limpHomde detected, 0: nothing */
  uint8_t DeratingSrc;    /* 0: derating is not activated, bit0: mos derating, bit1: cap derating, bit2: motor derating*/
} CanTxMsg_InvLogInfo1_t;

typedef struct 
{
  int16_t IdCmdI16;       /* unit : 0.1A */
  int16_t IqCmdI16;       /* unit : 0.1A */
  int16_t IdFbkI16;       /* unit : 0.1A */
  int16_t IqFbkI16;       /* unit : 0.1A */
} CanTxMsg_InvLogInfo2_t;

typedef struct
{
  int16_t AcLimitCmd;     /* unit : 0.1A */   
  int16_t AcLimitTq;      /* unit : 0.1Nm */
  int16_t DcLimitCmd;     /* unit : 0.1A */
  int16_t DcLimitTq;      /* unit : 0.1Nm */
} CanTxMsg_InvLogInfo3_t;

typedef struct
{
  int16_t VdCmdI16;         /* unit: 0.1V */
  int16_t VqCmdI16;         /* unit: 0.1V */
  int16_t PerformanceTqI16; /* unit: 0.1Nm */
  uint8_t AccPedal1Volt;    /* unit: 0.02V */
  uint8_t EA5V;             /* unit: 0.02V */
} CanTxMsg_InvLogInfo4_t;

typedef struct
{
  uint8_t AlarmCode[8];     /* Alarm code list in order of detected time, refer to "ALARMID_XXxxXXxx" defined in AlramTable.h */
} CanTxMsg_InvLogInfo5_t;

#if USE_MOTOR_CTRL_DEBUG
typedef struct
{
  int16_t IdCmdOriI16;       /* unit : 0.1A */
  int16_t IqCmdOriI16;       /* unit : 0.1A */
  int16_t VdOriI16;       /* unit : 0.01V */
  int16_t VqOriI16;       /* unit : 0.01V */
} CanTxMsg_InvLogInfo6_t;

typedef struct
{
  int16_t Vs;       /* unit : 0.01V */
  int16_t DCP_D;       /* unit : 0.01V */
  int16_t DCP_Q;       /* unit : 0.01V */
  uint8_t ServoOnOffState;
  uint8_t Reserved;
} CanTxMsg_InvLogInfo7_t;

typedef struct
{
  int16_t IdErr;     /* unit : 0.02A */
  int16_t IqErr;     /* unit : 0.02A */
  int16_t DCP_D_Err;     /* unit : 0.02A */
  int16_t DCP_Q_Err;      /* unit : 0.02A */
} CanTxMsg_InvLogInfo8_t;
#else
typedef struct
{
  int16_t InstPwr;          /* unit: W, instant output power */
  int16_t AvgPwr;           /* unit: W, average output power */
  uint16_t TimeRemain;      /* unit: sec, operation time remained */
  //bit 48~51
  uint8_t KillSwitchDI:1;   /* digital input flag of kill switch */
  uint8_t BoostDI:1; 		/* digital input flag of Boost Button */
  uint8_t ReverseDI:1; 		/* digital input flag of Reverse Button */
  uint8_t BrakeDI:1; 		/* digital input flag of Brake Button */
  //bit 52~55
  uint8_t Reserve52to55bits:4;
  //bit 56~59
  uint8_t RearLedFaultDI:1;	/* digital input flag of fault of rear LED */
  uint8_t FrontLedFaultDI:1;/* digital input flag of fault of Front LED */
  uint8_t BufFbDI:1;		/* digital input flag of the feedback pin in buffer IC*/
  uint8_t Reserve59bit:1;
  //bit 60~63
  uint8_t ISenUFaultDI:1;	/* digital input flag of fault of U current sensor */
  uint8_t ISenVFaultDI:1;	/* digital input flag of fault of V current sensor */
  uint8_t ISenWFaultDI:1;	/* digital input flag of fault of W current sensor */
  uint8_t Reserve63bit:1;
} CanTxMsg_InvLogInfo6_t;

typedef struct
{
  uint8_t MainBatSoc;                   /* unit: % */
  uint8_t BatMainSm:4;       /* BMS main state machine value report by BMS, refer to "BatMainSM_t" */
  uint8_t ServoOnOffState:4;
  uint8_t BatPwrOnState:4;       /* BMS precharge state machine value report by BMS, refer to "BatPwrOnSM_t" */
  uint8_t BatPwrOffState:4;       /* BMS precharge state machine value report by BMS, refer to "BatPwrOffSM_t" */
  uint8_t E5V;					          /* unit: 0.025V */
  uint8_t ES5V;					          /* unit: 0.025V */
  uint8_t HWID_ID1First8bits;			  /* HWID 1 first 8 bits */
  uint8_t HWID_ID2First4bits:4;			  /* HWID 2 first 4 bits*/
  uint8_t HWID_ID1Last4bits:4;            /* HWID 1 last 4 bits   */
  uint8_t HWID_ID2Last8bits;			  /* HWID 2 last 8 bits */
} CanTxMsg_InvLogInfo7_t;

typedef struct
{
  int16_t IuFbk;     /* unit : 0.02A */
  int16_t IvFbk;     /* unit : 0.02A */
  int16_t IwFbk;     /* unit : 0.02A */
  uint16_t PreC;      /* unit : 0.1V */
} CanTxMsg_InvLogInfo8_t;
#endif
typedef struct
{
  uint8_t AccPedal2Volt;    /* unit: 0.02V */
  uint8_t S13V8;                   /* unit: 0.1V */
  uint8_t Max10kHzLoopLoad;  /* unit : %*/
  uint8_t AveCurrentLoopLoad;  /* unit : %*/
  uint8_t MaxPLCLoopLoad;  /* unit : %*/
  uint8_t AvePLCLoopLoad;  /* unit : %*/
  uint8_t Max100HzLoopLoad;  /* unit : %*/
  uint8_t SecBatSoc;                   /* unit: % */;					          /* unused byte 07 */
} CanTxMsg_InvLogInfo9_t;

/*=========================================
 * Definition of TxRxDATA
 =========================================*/

typedef union InvCanRxCmd_u
{

  uint8_t DataU8[8];
  int8_t DataI8[8];
  uint16_t DataU16[4];
  int16_t DataI16[4];
  uint32_t DataU32[2];
  int32_t DataI32[2];
  float DataF[2];
} InvCanRxInfo_t;

typedef union InvCanTxCmd_u
{
  CanTxMsg_id512_200_INV_t id512_200_INV;
  CanTxMsg_InvLogInfo0_t InvLogInfo0;
  CanTxMsg_InvLogInfo1_t InvLogInfo1;
  CanTxMsg_InvLogInfo2_t InvLogInfo2;
  CanTxMsg_InvLogInfo3_t InvLogInfo3;
  CanTxMsg_InvLogInfo4_t InvLogInfo4;
  CanTxMsg_InvLogInfo5_t InvLogInfo5;
  CanTxMsg_InvLogInfo6_t InvLogInfo6;
  CanTxMsg_InvLogInfo7_t InvLogInfo7;
  CanTxMsg_InvLogInfo8_t InvLogInfo8;
  CanTxMsg_InvLogInfo9_t InvLogInfo9;
  uint8_t DataU8[8];
  int8_t DataI8[8];
  uint16_t DataU16[4];
  int16_t DataI16[4];
  uint32_t DataU32[2];
  int32_t DataI32[2];
  float DataF[2];
} InvCanTxCmd_t;

/* define the informations reported from BMS */
/* Todo : check the value of BMS_SN_BYTE_NUMBER with BMS vendor */
#define BMS_VERSION_CODE_NUMBER 5 
#define BMS_SN_BYTE_NUMBER 20


extern const CanIdConfig_t CanIdTableExtra[];

#if MEASURE_CPU_LOAD
extern float Max_100Hz_Load_pct;
extern float Max_PLCLoop_Load_pct;
extern float Max_CurrentLoop_Load_pct;
extern float Ave_100Hz_Load_pct;
extern float Ave_PLCLoop_Load_pct;
extern float Ave_CurrentLoop_Load_pct;
#endif

#endif /* INC_PROTOCOL_H_ */
#endif /* E10 */
