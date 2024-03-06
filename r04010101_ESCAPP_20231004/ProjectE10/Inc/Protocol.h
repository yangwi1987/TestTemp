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

typedef enum BmsActiveState_e
{
  BMS_ACTIVE_STATE_STARTUP = 0x00,
  BMS_ACTIVE_STATE_PRECHARGE,
  BMS_ACTIVE_STATE_DISCHARGE,
  BMS_ACTIVE_STATE_REQUPERATION,
  BMS_ACTIVE_STATE_CHARGE,
  BMS_ACTIVE_STATE_STANDBY,
  BMS_ACTIVE_STATE_REVERSIBLE_ERROR,
  BMS_ACTIVE_STATE_PERMERNENT_ERROR,
  BMS_ACTIVE_STATE_SHUTDOWN,
  BMS_ACTIVE_STATE_MAX,
} BmsActiveState_t;

typedef enum BmsPreChgState_e
{
  BMS_PRECHG_STATE_OFF = 0x00,
  BMS_PRECHG_STATE_ACTIVE,
  BMS_PRECHG_STATE_FAILED,
  BMS_PRECHG_STATE_SUCCESSFUL,
  BMS_PRECHG_STATE_MAX
} BmsPreChgState_t;

typedef enum LedCtrlCode_e
{
  LED_CTRL_OFF = 0,
  LED_CTRL_GREEN,
  LED_CTRL_BLUE,
  LED_CTRL_RED,
} LedCtrlCode_t;

typedef union
{
  /* data */
  uint8_t All;
  struct
  {
    LedCtrlCode_t Led0:2;
    LedCtrlCode_t Led1:2;
    LedCtrlCode_t Led2:2;
    LedCtrlCode_t Led3:2;
  } Leds;
} BatPackLedCtrl_t;

#define BAT_LED_SHOW_BMS_ERROR 		0b11110000	/* RED, RED, OFF, OFF */
#define BAT_LED_SHOW_INV_ERROR 		0b00001111	/* OFF, OFF, RED, RED */
#define BAT_LED_SHOW_OTHER_ERROR	0b00111100	/* OFF, RED, RED, OFF */
#define BAT_LED_SHOW_NO_ERROR     	0b00000000  /* OFF, OFF, OFF, OFF */

/*=========================================
 * Definition of BMS_Status_01(0x402)
 =========================================*/
typedef struct
{
  uint8_t DischargeCurrentLimit_h;
  uint8_t BmsShutdownRequest      : 1;  /* A notification shows that BMS detects a turn-off command from USER (a press button action or others) */
  uint8_t WarningFlag             : 1;  /* A flag shows that a battery warning condition is detected by BMS */
  uint8_t AlarmFlag               : 1;  /* A flag shows that a battery alarm condition is detected by BMS */
  uint8_t LimpFlag                : 1;  /* A flag shows limp mode output request from BMS */
  uint8_t DischargeCurrentLimit_l : 4;  /* discharge current limitation of battery ,unit:1A, offset = 0 */
  uint8_t Soc;                          /* Battery SOC */
  uint8_t TerminalVoltage_h;
  uint8_t Current_h               : 6;
  uint8_t TerminalVoltage_l       : 2;  /* battery terminal voltage, unit=0.1V, offset = 0 */           
  uint8_t Current_l;                 	/* current from battery, unit=0.05A, offset = 0, positive:charging, negative: discharging */
  uint8_t MaxCellTemp;                  /* max temperature of cell, unit= 1'C, offset = -40 */
  uint8_t Byte07;
} CanRxMsg_BmsStatus01_t;


/*=========================================
 * Definition of BMS_Status_02(0x403)
 =========================================*/
typedef struct
{
  uint8_t Byte00;
  BmsActiveState_t BmsActiveState;       /* todo: confirm with BMS vender to signal definition */
  BmsPreChgState_t PreCHGState;          /* todo: confirm with BMS vender to signal definition */
  uint8_t BmsFrameCnt;                   /* Verify if this counter varies with time and range from 0 ~ 255 */ 
  uint8_t Byte04;
  uint8_t Byte05;
  uint8_t Byte06;
  uint8_t Byte07;
} CanRxMsg_BmsStatus02_t;

typedef enum BmsCtrlReq_e
{
  BMS_CTRL_NO_REQ = 0x00,
  BMS_CTRL_REQ,
}BmsCtrlReq_t;

/*=========================================
 * Definition of VCU_BMS_Control_01(0x500)
 =========================================*/
typedef struct
{
  uint8_t ConnectionRequest     : 2;  /* 0 : no request, 1 request */
  uint8_t DisconnectionRequest  : 2;  /* 0 : no request, 1 request */
  uint8_t ShutdownRequest       : 2;  /* 0 : no request, 1 request */
  uint8_t Byte0Reseved          : 2;
  uint8_t Byte01;
  uint8_t Byte02;
  uint8_t Byte03;
  uint8_t Byte04;
  uint8_t Byte05;
  uint8_t Byte06;
  uint8_t Byte07;
} CanTxMsg_BmsCtrl01_t;

/*=========================================
 * Definition of VCU_BMS_Control_02(0x501)
 =========================================*/
typedef struct
{
  BatPackLedCtrl_t LedCtrl;
  uint8_t InvFrameCnt;        /* this value should loop from 0~255 */
  uint8_t Byte02;
  uint8_t Byte03;
  uint8_t Byte04;
  uint8_t Byte05;
  uint8_t Byte06;
  uint8_t Byte07;
} CanTxMsg_BmsCtrl02_t;

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
  CanRxMsg_BmsStatus01_t BmsStatus01;
  CanRxMsg_BmsStatus02_t BmsStatus02;
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
  CanTxMsg_BmsCtrl01_t BmsCtrl01;
  CanTxMsg_BmsCtrl02_t BmsCtrl02;
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

typedef struct 
{
	uint8_t ShutDownReq;
	uint8_t ConnectReq;
	uint8_t DisconnectReq;
	BatPackLedCtrl_t LedCtrlCmd;
} BmsCtrlCmd_t;

#define BMS_CTRL_CMD_DEFAULT 	\
{  				  				\
	0,		/*ShutDownReq*/		\
	0,		/*ConnectReq*/		\
	0,		/*DisconnectReq*/		\
	{0},	/*LedCtrlCmd*/		\
}								\


typedef struct
{
  uint8_t BmsFrameCnt;
  uint8_t Soc;
  uint8_t LimpFlag;
  uint8_t WarningFlag;
  uint8_t AlarmFlag;
  uint8_t BmsShutdownRequest;
  uint8_t BmsFwVer[BMS_VERSION_CODE_NUMBER];
  uint8_t BmsSN[BMS_SN_BYTE_NUMBER];
  int16_t MaxCellTemp;
  float Current;
  float DcVolt;
  float DCCurrentLimit;
  float BatInstPwr;							/* Battery instantaneous power= Vbat x Ibat */
} BmsReportInfo_t;

#define BMS_REPORT_INFO_DEFAULT \
{                               \
  0,                            \
  0,  /* SOC */                 \
  0,  /* LimpFlag */  			    \
  0,  /* WarningFlag */   		  \
  0,  /* AlarmFlag */			      \
  0,  /*BmsShutdownRequest*/	  \
  {0,0,0,0,0},	/*BmsFwVer[BMS_VERSION_CODE_NUMBER]*/	\
  {								              \
   0,0,0,0,0,0,0,0,0,0,			    \
   0,0,0,0,0,0,0,0,0,0			    \
  }, 	/*BmsSN[BMS_SN_BYTE_NUMBER]*/	\
  0, /*MaxCellTemp*/        	  \
  0.0f,	/*Current*/				      \
  0.0f,	/*DcVolt*/				      \
  0.0f,	/*DCCurrentLimit*/		  \
  0.0f,	/*BatInstPwr*/		  \
}                               \

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
