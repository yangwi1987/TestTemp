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



/* ==== macro for CANTX IDs for transmit ==== */

#define CANTXID_ESC_LOG_INFO_0  0x720   /*Debug information for ESC develop 00*/
#define CANTXID_ESC_LOG_INFO_1  0x721   /*Debug information for ESC develop 01*/
#define CANTXID_ESC_LOG_INFO_2  0x722   /*Debug information for ESC develop 02*/
#define CANTXID_ESC_LOG_INFO_3  0x723   /*Debug information for ESC develop 03*/
#define CANTXID_ESC_LOG_INFO_4  0x724   /*Debug information for ESC develop 04*/
#define CANTXID_ESC_LOG_INFO_5  0x725   /*Debug information for ESC develop 05*/
#define CANTXID_ESC_LOG_INFO_6  0x726   /*Debug information for ESC develop 06*/
#define CANTXID_ESC_LOG_INFO_7  0x727   /*Debug information for ESC develop 07*/

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




/*=========================================
 * Definition of CMD to BMS : stay alive TOD = 0x001 (send this msg every 500ms)
 =========================================*/
typedef struct
{
  uint8_t Byte0;            
  uint8_t Byte1;
  uint8_t Byte2;
  uint8_t Byte3;
  uint8_t Byte4;
  uint8_t Byte5;    
  uint8_t Byte6;
  uint8_t Byte7;
} CanTxMsg_BmsCtrTod0x01_t;

/*=========================================
 * Definition of CMD to BMS : Power control, TOD = 0x002 (send this msg while needed)
 =========================================*/
typedef struct
{
  uint8_t PwrCtrl;    /* 0 = off, 1 = on */            
  uint8_t Byte1;
  uint8_t Byte2;
  uint8_t Byte3;
  uint8_t Byte4;
  uint8_t Byte5;    
  uint8_t Byte6;
  uint8_t Byte7;
} CanTxMsg_BmsCtrTod0x02_t;

/*=========================================
 * Definition of CMD to BMS : Broadcast Control(Bc), TOD = 0x003 (send this msg while needed)
 =========================================*/
typedef struct
{
  uint8_t BcCtrl;    /* 0 = req BMS to stop broadcasting, 1 = req BMS to start broadcasting */            
  uint8_t Byte1;
  uint8_t Byte2;
  uint8_t Byte3;
  uint8_t Byte4;
  uint8_t Byte5;    
  uint8_t Byte6;
  uint8_t Byte7;
} CanTxMsg_BmsCtrTod0x03_t;



typedef struct
{
  uint8_t MotorTemp;        /* unit: 'C, offset: -40 */
  uint8_t EscMos1Temp;      /* unit: 'C, offset: -40 */
  uint8_t EscMos2Temp;      /* unit: 'C, offset: -40 */
  uint8_t EscCapTemp;       /* unit: 'C, offset: -40 */
  uint8_t TetherSensor :4;  /* 0: not ready, 1: ready to go */ 
  uint8_t Reserved :4;  /* 0 = Paddle, 1= Surf, 2 = Foil, refer to "FoilPos_t" */
  uint8_t FoilSensorVolt;   /* unit: 0.1V */
  uint8_t ThrottleRaw;      /* unit 1%, throttle command received from RC */
  uint8_t ThrottleFinal;    /* unit 1%, throttle command handled by throttle mapping strategy */
} CanTxMsg_EscLogInfo0_t;

typedef struct
{
  uint16_t DcVoltU16;     /* unit: 0.1V */ 
  int16_t MotorRpmI16;    /* unit: rpm */
  uint8_t EscState :4;    /* refer to "PcuState_e" defined in "ICANInterface.h" */
  uint8_t WarnFlag :2;    /* 1: warning detected, 0: nothing */
  uint8_t AlarmFlag :2;   /* 1: Alarm detected, 0: nothing */
  uint8_t OutputMode;     /* 0: limpHome mode , 1: Paddle mode, 2: Surf Mode, 3 = Foil mode*/
  uint8_t LimpHomeFlag;    /* 1: limpHomde detected, 0: nothing */
  uint8_t DeratingSrc;    /* 0: derating is not activated, bit0: mos derating, bit1: cap derating, bit2: motor derating*/
} CanTxMsg_EscLogInfo1_t;

typedef struct 
{
  int16_t IdCmdI16;       /* unit : 0.1A */
  int16_t IqCmdI16;       /* unit : 0.1A */
  int16_t IdFbkI16;       /* unit : 0.1A */
  int16_t IqFbkI16;       /* unit : 0.1A */
} CanTxMsg_EscLogInfo2_t;

typedef struct
{
  int16_t AcLimitCmd;     /* unit : 0.1A */   
  int16_t AcLimitTq;      /* unit : 0.1Nm */
  int16_t DcLimitCmd;     /* unit : 0.1A */
  int16_t DcLimitTq;      /* unit : 0.1Nm */
} CanTxMsg_EscLogInfo3_t;

typedef struct
{
  int16_t VdCmdI16;         /* unit: 0.1V */
  int16_t VqCmdI16;         /* unit: 0.1V */
  int16_t PerformanceTqI16; /* unit: 0.1Nm */
  uint8_t Byte06;           /* unused byte 06 */
  uint8_t Byte07;           /* unused byte 07 */
} CanTxMsg_EscLogInfo4_t;

typedef struct
{
  uint8_t AlarmCode[8];     /* Alarm code list in order of detected time, refer to "ALARMID_XXxxXXxx" defined in AlramTable.h */
} CanTxMsg_EscLogInfo5_t;       

typedef struct
{
  int16_t InstPwr;          /* unit: W, instant output power */
  int16_t AvgPwr;           /* unit: W, average output power */
  uint16_t TimeRemain;      /* unit: sec, operation time remained */
  uint8_t RcConnStatus:4;   /* 0: RC is not connected, 1: RC is connected */
  uint8_t PwrLv:4;          /* power level applied now */
  uint8_t Byte07;           /* unused byte 07 */
} CanTxMsg_EscLogInfo6_t;



typedef struct
{
  uint8_t Byte0;					          /* unused byte 04 */
  uint8_t BatSoc;                   /* unit: % */
  BmsActiveState_t BmsMainSm;       /* BMS main state mechine value report by BMS, refer to "BmsMainSm_t" */
  BmsPreChgState_t BmsPrchSm;       /* BMS precharge state mechine value report by BMS, refer to "BmsPrchSm_t" */
  uint8_t Byte4;					          /* unused byte 04 */
  uint8_t Byte5;					          /* unused byte 05 */
  uint8_t Byte6;					          /* unused byte 06 */
  uint8_t Byte7;					          /* unused byte 07 */
} CanTxMsg_EscLogInfo7_t;


/*=========================================
 * Definition of TxRxDATA
 =========================================*/

typedef union EscCanRxCmd_u
{
  uint8_t DataU8[8];
  int8_t DataI8[8];
  uint16_t DataU16[4];
  int16_t DataI16[4];
  uint32_t DataU32[2];
  int32_t DataI32[2];
  float DataF[2];
} EscCanRxInfo_t;

typedef union EscCanTxCmd_u
{
  CanTxMsg_BmsCtrTod0x01_t BmsCtrlTod0x01;
  CanTxMsg_BmsCtrTod0x02_t BmsCtrlTod0x02;
  CanTxMsg_BmsCtrTod0x03_t BmsCtrlTod0x03;
  CanTxMsg_EscLogInfo0_t EscLogInfo0;
  CanTxMsg_EscLogInfo1_t EscLogInfo1;
  CanTxMsg_EscLogInfo2_t EscLogInfo2;
  CanTxMsg_EscLogInfo3_t EscLogInfo3;
  CanTxMsg_EscLogInfo4_t EscLogInfo4;
  CanTxMsg_EscLogInfo5_t EscLogInfo5;
  CanTxMsg_EscLogInfo6_t EscLogInfo6;
  CanTxMsg_EscLogInfo7_t EscLogInfo7;
  uint8_t DataU8[8];
  int8_t DataI8[8];
  uint16_t DataU16[4];
  int16_t DataI16[4];
  uint32_t DataU32[2];
  int32_t DataI32[2];
  float DataF[2];
} EscCanTxCmd_t;

/* define the informations reported from BMS */
/* Todo : check the value of BMS_SN_BYTE_NUMBER with BMS vendor */
#define BMS_VERSION_CODE_NUMBER 5 
#define BMS_SN_BYTE_NUMBER 20

typedef struct
{
	uint8_t ShutDownReq;
	uint8_t ConnectReq;
	uint8_t DisconnectReq;
} BmsCtrlCmd_t;

#define BMS_CTRL_CMD_DEFAULT 	\
{  				  				\
	0,		/*ShutDownReq*/		\
	0,		/*ConnectReq*/		\
	0,		/*DisconnectReq*/		\
}								\


typedef struct
{
  BmsActiveState_t MainSm;        /* will be set to discharge when both battery are able to be discharge */
  BmsPreChgState_t PrchSM;        /* will be set to complete when both battery are able to be discharge */
  uint8_t BmsFrameCnt;            /* not used in this proj @p0 */
  uint8_t Soc;                    /* u8, unit = % */
  uint8_t Soh;                    /* u8, unit = % */
  uint8_t LimpFlag;               /* not used in this proj @p0 */
  uint8_t WarningFlag;            /* not used in this proj @p0 */
  uint8_t AlarmFlag;              /* will be set when any error is reported from battery @p0 */
  uint8_t BmsShutdownRequest;     /* not used in this proj @p0 */
  uint8_t BmsFwVer[BMS_VERSION_CODE_NUMBER];  /* not used in this proj @p0 */
  uint8_t BmsSN[BMS_SN_BYTE_NUMBER];          /* not used in this proj @p0 */
  uint16_t ErrorFlags ;           /* u16, 2 bytes error flags to*/
  int16_t MaxCellTemp;            /* i16, uint = 'C */
  float CapRemain;		            /* float, unit = Ah */
  float Current;                  /* float, unit = A */
  float DcVolt;                   /* float, unit = V */
  float DCCurrentLimit;           /* float, unit = A */
  float BatInstPwr;				        /* Battery instantaneous power= Vbat x Ibat */
} BmsReportInfo_t;

#define BMS_REPORT_INFO_DEFAULT \
{                               \
  BMS_ACTIVE_STATE_STARTUP,     \
  BMS_PRECHG_STATE_OFF,         \
  0,                            \
  0,  /* SOC */                 \
  0,  /* SOH */                 \
  0,  /* LimpFlag */  			    \
  0,  /* WarningFlag */   		  \
  0,  /* AlarmFlag */			      \
  0,  /*BmsShutdownRequest*/	  \
  {0,0,0,0,0},	/*BmsFwVer[BMS_VERSION_CODE_NUMBER]*/	\
  {								              \
   0,0,0,0,0,0,0,0,0,0,			    \
   0,0,0,0,0,0,0,0,0,0			    \
  }, 	/*BmsSN[BMS_SN_BYTE_NUMBER]*/	\
  0, /*ErrorFlags*/        	  \
  0, /*MaxCellTemp*/        	  \
  0.0f,	/*CapRemain*/				      \
  0.0f,	/*Current*/				      \
  0.0f,	/*DcVolt*/				      \
  0.0f,	/*DCCurrentLimit*/		  \
  0.0f,	/*BatInstPwr*/		  \
}                               \

extern const CanIdConfig_t LscCanIdTableExtra[];

#endif /* INC_PROTOCOL_H_ */
#endif /* E10 */
