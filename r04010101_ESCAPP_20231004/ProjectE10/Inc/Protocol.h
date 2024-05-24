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
  uint16_t IU_ADC;        /* unit: 'C, offset: -40 */
  uint16_t IV_ADC;      /* unit: 'C, offset: -40 */
  uint16_t IW_ADC;      /* unit: 'C, offset: -40 */
  uint16_t VDC_ADC;      /* unit: 'C, offset: -40 */
} CanTxMsg_InvLogInfo0_t;


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
