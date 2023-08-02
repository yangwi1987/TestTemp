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
#include "math.h"


/* ==== macro for CANTX IDs for transmit ==== */
#define CANTXID_BMS_CONTROL_01  0x500   /* command to control BMS actions #1 */
#define CANTXID_BMS_CONTROL_02  0x501   /* command to control BMS actions #2 */

#define CANTXID_ESC_LOG_INFO_0  0x720   /*Debug information for ESC develop 00*/
#define CANTXID_ESC_LOG_INFO_1  0x721   /*Debug information for ESC develop 01*/
#define CANTXID_ESC_LOG_INFO_2  0x722   /*Debug information for ESC develop 02*/
#define CANTXID_ESC_LOG_INFO_3  0x723   /*Debug information for ESC develop 03*/
#define CANTXID_ESC_LOG_INFO_4  0x724   /*Debug information for ESC develop 04*/
#define CANTXID_ESC_LOG_INFO_5  0x725   /*Debug information for ESC develop 05*/
#define CANTXID_ESC_LOG_INFO_6  0x726   /*Debug information for ESC develop 06*/
#define CANTXID_ESC_LOG_INFO_7  0x727   /*Debug information for ESC develop 07*/

/* ==== macro for CANRX IDs for receive ==== */
#define CANRXID_BMS_STATUS_01         0x402   /* info shows BMS status */
#define CANRXID_BMS_STATUS_02         0x403   /* info shows BMS status */

#define CAN_TX_CRI_ALARM_MASK 0x01
#define CAN_TX_NON_CRI_ALARM_MASK 0x02
#define CAN_TX_WARNING_MASK 0x04


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

typedef enum FoilPos_e
{
  FOIL_POS_PADDLE = 1,
  FOIL_POS_SURF,
  FOIL_POS_FOIL,
} FoilPos_t;

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
#define BAT_LED_SHOW_ESC_ERROR 		0b00001111	/* OFF, OFF, RED, RED */
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
  uint8_t EscFrameCnt;        /* this value should loop from 0~255 */
  uint8_t Byte02;
  uint8_t Byte03;
  uint8_t Byte04;
  uint8_t Byte05;
  uint8_t Byte06;
  uint8_t Byte07;
} CanTxMsg_BmsCtrl02_t;

typedef struct
{
  uint8_t MotorTemp;        /* unit: 'C, offset: -40 */
  uint8_t EscMos1Temp;      /* unit: 'C, offset: -40 */
  uint8_t EscMos2Temp;      /* unit: 'C, offset: -40 */
  uint8_t EscCapTemp;       /* unit: 'C, offset: -40 */
  uint8_t TetherSensor :4;  /* 0: not ready, 1: ready to go */ 
  FoilPos_t	FoilPosition :4;  /* 0 = Paddle, 1= Surf, 2 = Foil, refer to "FoilPos_t" */
  uint8_t FoilSensorVolt;   /* unit: 0.1V */
  uint8_t ThrottleRaw;      /* unit 1%, throttle command received from RC */
  uint8_t ThrottleFinal;    /* unit 1%, throttle command handled by throttle mapping strategy */
} CanTxMsg_EscLogInfo0_t;
typedef struct
{
  uint16_t DcVoltU16;     /* unit: 0.1V */ 
  int16_t MotorRpmI16;    /* unit: rpm */
  uint8_t EscState :4;    /* refer to "ENUM_PcuState" defined in "ICANInterface.h" */
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
  BatPackLedCtrl_t BatPackLedCtrl;  /* LED actione code sent to BMS by ESC , total size = 8bit (uint8_t) */
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
  CanRxMsg_BmsStatus01_t BmsStatus01;
  CanRxMsg_BmsStatus02_t BmsStatus02;
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
  CanTxMsg_BmsCtrl01_t BmsCtrl01;
  CanTxMsg_BmsCtrl02_t BmsCtrl02;
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
  BmsActiveState_t MainSm;
  BmsPreChgState_t PrchSM;
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
} BmsReportInfo_t;

#define BMS_REPORT_INFO_DEFAULT \
{                               \
  BMS_ACTIVE_STATE_STARTUP,     \
  BMS_PRECHG_STATE_OFF,         \
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
}                               \

extern const CanIdConfig_t LscCanIdTableExtra[];

#endif /* INC_PROTOCOL_H_ */
#endif /* BME */
