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
#define CANTXID_BMS_CONTROL_01  0x500   /* command to control BMS actions*/

#define CANTXID_ESC_LOG_INFO_0  0x720   /*Debug information for ESC develop 00*/
#define CANTXID_ESC_LOG_INFO_1  0x721   /*Debug information for ESC develop 01*/
#define CANTXID_ESC_LOG_INFO_2  0x722   /*Debug information for ESC develop 02*/
#define CANTXID_ESC_LOG_INFO_3  0x723   /*Debug information for ESC develop 03*/
#define CANTXID_ESC_LOG_INFO_4  0x724   /*Debug information for ESC develop 04*/
#define CANTXID_ESC_LOG_INFO_5  0x725   /*Debug information for ESC develop 05*/
#define CANTXID_ESC_LOG_INFO_6  0x726   /*Debug information for ESC develop 06*/
#define CANTXID_ESC_LOG_INFO_7  0x727   /*Debug information for ESC develop 07*/

/* ==== macro for CANRX IDs for receive ==== */

#define CANRXID_BMS_VOLT_02           0x301   /* info shows BMS Volt value group2 */
#define CANRXID_BMS_VOLT_03           0x302   /* info shows BMS Volt value group3 */
#define CANRXID_BMS_VOLT_04           0x303   /* info shows BMS Volt value group4 */
#define CANRXID_BMS_VOLT_05           0x304   /* info shows BMS Volt value group5 */
#define CANRXID_BMS_VOLT_06           0x305   /* info shows BMS Volt value group6 */
#define CANRXID_BMS_VOLT_01           0x306   /* info shows BMS Volt value group1 */

#define CANRXID_BMS_STATE_MACHINE_01  0x401   /* info shows BMS state value */
#define CANRXID_BMS_STATUS_01         0x402   /* info shows BMS status */
#define CANRXID_BMS_CURRENT_01        0x404   /* info shows BMS current value */
#define CANRXID_BMS_TEMP_01           0x600   /* info shows BMS temperature value group1 */
#define CANRXID_BMS_TEMP_02           0x601   /* info shows BMS temperature value group2 */
#define CANRXID_BMS_TEMP_03           0x602   /* info shows BMS temperature value group3 */
#define CANRXID_BMS_TEMP_04           0x603   /* info shows BMS temperature value group4 */

#define CAN_TX_ALARM_MASK 0x01
#define CAN_TX_WARNING_MASK 0x02

/*======================================
 *  Enum definition
 *  ==================================*/


typedef enum BmsMainSm_e
{
  BMS_MAIN_SM_OFF = 0,
  BMS_MAIN_SM_INIT,
  BMS_MAIN_SM_IDLE,
  BMS_MAIN_SM_ACTIVE,
  BMS_MAIN_SM_ERROR,
  BMS_MAIN_SM_PROGRAMMING,
  BMS_MAIN_SM_SHUTDOWN,
} BmsMainSm_t;

typedef enum BmsPrchSm_e
{
  BMS_PRCH_SM_OFF = 0,
  BMS_PRCH_SM_PRCH,
  BMS_PRCH_SM_FINISH,
  BMS_PRCH_SM_DONE,
} BmsPrchSm_t;

typedef enum BmsWakeUpReason_e
{
  BMS_WAKEUP_REASON_UNKNOWN = 0,
  BMS_WAKEUP_REASON_12V,
  BMS_WAKEUP_REASON_PUSH_BTN,
} BmsWakeUpReason_t;

typedef enum FoilPos_e
{
  FOIL_POS_PADDLE = 1,
  FOIL_POS_SURF,
  FOIL_POS_FOIL,
} FoilPos_t;


/*=========================================
 * Definition of BMS_State_machine_01(0x401)
 =========================================*/
typedef struct
{
  uint8_t LimpHomeActive  :1;
  uint8_t WarningActive   :1;
  uint8_t Revsd27         :6;
  uint8_t BatteryOperation;   
  BmsMainSm_t BmsMainSM;
  BmsPrchSm_t BmsPrchSM;
  BmsWakeUpReason_t WakeUpReason;
  uint8_t ErrorCode_L;
  uint8_t ErrorCode_H;
  uint8_t Byte07;             /* unused byte 07*/
} CanRxMsg_BmsSm_t;

/*=========================================
 * Definition of BMS_Status_01(0x402)
 =========================================*/
typedef struct
{
  uint8_t BmsStatus01_CRC;
  uint8_t BmsStatus01_BZ  :4;
  uint8_t ChipRevision    :4;
  uint8_t SwVer_internal;   
  uint8_t SwVer_Major;
  uint8_t SwVer_Minor;
  uint8_t SwVer_BugFix;
  uint8_t SwVer_RC;
  uint8_t	PackID;             /* unused byte 07*/
} CanRxMsg_BmsStatus_t;

/*=========================================
 * Definition of BMS_Current_01(0x404)
 =========================================*/
typedef struct
{
  uint8_t BmsCurrent01_CRC;
  uint8_t BmsCurrent01_BZ   :4;
  uint8_t Byte1b47          :4;   /* unused bit 4~7 in byte 1 */
  uint8_t Current1[3];            /* 3 bytes value of current#1, unit = 0.01A, offset= 0, byte order : LSB first (intel) */  
  uint8_t Current2[3];            /* 3 bytes value of current#2, unit = 0.01A, offset= 0, byte order : LSB first (intel)*/
} CanRxMsg_BmsCurrent_t;

typedef struct
{
  int16_t Volt[4];                /* unit: 0.00015V, offset: 1.5V*/
} CanRxMsg_BmsVolt01to04_t;

typedef struct
{
  float PackVolt_AFE;                /* unit: V */
  float PackVolt_Calculated;         /* unit: V */
} CanRxMsg_BmsVolt05_t;

typedef struct
{
  int16_t CellMaxVolt;            /* unit: 0.00015V, offset: 1.5V*/
  uint8_t CellMaxVoltIdx;         
  uint8_t CellMinVolt_L;            /* CellMinVolt LSB ,unit: 0.00015V, offset: 1.5V*/
  uint8_t CellMinVolt_H;            /* CellMinVolt HSB ,unit: 0.00015V, offset: 1.5V*/
  uint8_t CellMinVoltIdx;         
  uint16_t CellVoltDelta;         /* unit: 0.001V*/  
} CanRxMsg_BmsVolt06_t;


typedef struct
{
  float DIE_Temp;       /* unit:'C */
  int16_t TempPrch;     /* unit: 0.01'C , offset: -40'C */
  int16_t TempBalance;  /* unit: 0.01'C , offset: -40'C */      
} CanRxMsg_BmsTemp01_t;

typedef struct
{
  int16_t CellTemp[4];       /* unit: 0.01'C , offset: -40'C */
} CanRxMsg_BmsTemp02to04_t;



typedef enum BmsCtrlReq_e
{
  BMS_CTRL_NO_REQ = 0x00,
  BMS_CTRL_REQ,
}BmsCtrlReq_t;

/*=========================================
 * Definition of VCU_BMS_Control_01(0x500)
 =========================================*/
typedef struct{

	uint8_t Byte0;
	uint8_t Byte1;
	BmsCtrlReq_t ConnReq;     
	BmsCtrlReq_t DisconnReq;
	BmsCtrlReq_t ShutdownReq;
	uint8_t Byte5;
	uint8_t Byte6;
	uint8_t Byte7;
} CanTxMsg_BmsCtrl01_t;

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
  uint8_t OutputMode;     /* 0: limphome mode , 1: Paddle mode, 2: Surf Mode, 3 = Foil mode*/
  uint8_t LimpHomeSrc;    /* 0: limphome mode is not activated, others: the trigger source of limp home mode, refer to (TBD enum) */
  uint8_t DeratingSrc;    /* 0: derating is not activated, others: the trigger source of derating, refer to (TBD enum) */
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

typedef struct
{
  BatPackLedCtrl_t BatPackLedCtrl;  /* LED actione code sent to BMS by ESC , total size = 8bit (uint8_t) */
  uint8_t BatSoc;                   /* unit: % */
  BmsMainSm_t BmsMainSm;            /* BMS main state mechine value report by BMS, refer to "BmsMainSm_t" */
  BmsPrchSm_t BmsPrchSm;            /* BMS precharge state mechine value report by BMS, refer to "BmsPrchSm_t" */
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
  CanRxMsg_BmsCurrent_t BmsCurrent;
  CanRxMsg_BmsSm_t BmsSm;
  CanRxMsg_BmsStatus_t BmsStatus;
  CanRxMsg_BmsTemp01_t BmsTemp01;
  CanRxMsg_BmsTemp02to04_t BmsTemp02to04;
  CanRxMsg_BmsVolt01to04_t BmsVolt01to04;
  CanRxMsg_BmsVolt05_t BmsVolt05;
  CanRxMsg_BmsVolt06_t BmsVolt06;
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
typedef struct 
{
  BmsMainSm_t MainSm;
  BmsPrchSm_t PrchSM;
  BmsWakeUpReason_t WakeUpReason;
  uint8_t Soc;
  uint8_t LimpHomeActiveFlag;
  uint8_t WarningActiveFlag;
  uint16_t ErrorCode;
  float Current01;
  float Current02;
  float DcVolt;
  float MaxCellTemp;
  float CellTemp[10];
  float TempDie;
  float TempPrch;
  float TempBalance;
  float TempShunt;
} BmsReportInfo_t;

#define BMS_REPORT_INFO_DEFAULT \
{                               \
  BMS_MAIN_SM_OFF,              \
  BMS_PRCH_SM_OFF,              \
  BMS_WAKEUP_REASON_UNKNOWN,    \
  0,  /* SOC */                 \
  0,  /* LimpHomeActiveFlag */  \
  0,  /* WarningActiveFlag */   \
  0,  /* Error code */          \
  0.0f, /*Current01*/           \
  0.0f, /*Current02*/           \
  0.0f, /*DcVolt*/              \
  0.0f, /*MaxCellTemp*/         \
  {0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f},	/*CellTemp[10]*/\
  0.0f, /*TempDie*/             \
  0.0f, /*TemPrch*/             \
  0.0f, /*TempBalance*/         \
  0.0f, /*TempShunt*/           \
}                               \



/* define the command to control BMS behavior */
typedef struct 
{
  BmsCtrlReq_t ConnReq;     
  BmsCtrlReq_t DisconnReq;
  BmsCtrlReq_t ShutdownReq;     
} BmsCtrlCmd_t;

#define BMS_CTRL_CMD_DEFAULT          \
{                                     \
  BMS_CTRL_NO_REQ,  /* ConnReq */     \
  BMS_CTRL_NO_REQ,  /* DisconnReq */  \
  BMS_CTRL_NO_REQ,  /* ShutdownReq */ \
}                                     \

extern const CanIdConfig_t LscCanIdTableExtra[];

#endif /* INC_PROTOCOL_H_ */
#endif /* BME */
