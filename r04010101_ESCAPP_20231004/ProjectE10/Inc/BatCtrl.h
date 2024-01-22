/*
 * BatApp.h
 *
 *  Created on: 2024年1月3日
 *      Author: egghe
 */

#ifndef INC_BATCTRL_H_
#define INC_BATCTRL_H_

#include "ExtranetCANStation.h"

#define BMS_TOD_RXED_MSK_0X20	0x0001
#define BMS_TOD_RXED_MSK_0X21	0x0002
#define BMS_TOD_RXED_MSK_ALL	0x0003

typedef enum BatIdx_e
{
  BAT_IDX_MAIN = 0,
  BAT_IDX_SEC,
  BAT_IDX_ALL,
} BatIdx_t ;

typedef enum BatInstance_e
{
  BAT_INSTANCE_MAIN = 1,
  BAT_INSTANCE_SEC,
} BatInstance_t ;


typedef struct
{
  /* data */
  uint32_t  Device    :8;
  uint32_t  Instance  :4;
  uint32_t  Tod       :12;
  uint32_t  Priority  :5;
  uint32_t  Reserved  :3;
} CanIdField_t;

typedef union
{
  uint32_t All;
  CanIdField_t Bits;
}CanIdField_u;


typedef enum CanDevice_e
{
  CAN_DEVICE_TO_BAT       = 0x07,
  CAN_DEVICE_FROM_BAT     = 0x17,
} CanDevice_t;

typedef enum CanTod_e
{
  CAN_TOD_STAY_ALIVE      = 0x001,
  CAN_TOD_POWER_CTRL      = 0x002,
  CAN_TOD_BROADCAST_CTRL  = 0x003,
  CAN_TOD_MEASUREMENT     = 0x020,
  CAN_TOD_STATUS          = 0x021,
  CAN_TOD_BAT_CELL_VOLT01 = 0x022,
  CAN_TOD_BAT_CELL_VOLT02 = 0x023,
  CAN_TOD_BAT_CELL_VOLT03 = 0x024,
  CAN_TOD_BAT_CELL_VOLT04 = 0x025,
} CanTod_t;

#define CAN_ID_FIELD_TO_BMS_DEFAULT \
{ \
  CAN_DEVICE_TO_BAT,  /* Device */\
  0x00,               /* Instance */\
  0x00,               /* Tod */\
  0x10,               /* Priority */\
  0x00,               /* Reserved*/\
}\

typedef struct
{
	uint16_t Chg		:1;		/* Charge MOS state: 0 = off, 1 = on */
	uint16_t Dsg		:1;		/* Discharge MOS state: 0 = off, 1 = on */
	uint16_t XChg	 	:1;		/* Charging Disabled: 1 = disabled */
	uint16_t XDsg 	 	:1;		/* Discharging Disabled: 1 = disabled */
	uint16_t Reserved 	:12;
} BatMosFetStatus_t;

typedef union
{
	uint16_t All;
	BatMosFetStatus_t Bits;
} BatMosFetStatus_u;

#define BAT_MOSFET_STATUS_DEFAULT {0}

typedef struct
{
	float DCVolt;			    /* unit = V */
	float DcCurrent;		    /* unit = A */
	float CapRemain;		    /* unit = 10mAh */
	uint16_t Soc;			    /* unit = %*/
	uint16_t ErrorFlags;	  	/* error flags*/
	int16_t CellTemp;		    /* unit = 'C*/
	BatMosFetStatus_u FetStatus;
	uint8_t Soh;			    /* unit = %* */
	uint8_t TodsRcved;       		/* Tod received flags, bit0 = tod0x20, bit1 = tod0x21,*/
} BatInfo_t;

#define CAN_INFORM_FORM_BAT_DEFAULT \
{                                   \
  0.0f,                       /*DCVolt*/ \
  0.0f,                       /*DcCurrent*/ \
  0.0f,                       /*CapRemain*/ \
  0,                          /*Soc*/ \
  0,                          /*ErrorFlags*/ \
  0,                          /*CellTemp*/ \
  BAT_MOSFET_STATUS_DEFAULT,  /*FetStauts*/ \
  0,                          /*Soh*/ \
  0,                          /*Reserved*/ \
}                                   \

/*=========================================
 * Definition of TOD 0x20
 =========================================*/
typedef struct
{
	uint16_t DCVolt;			  /* unit = 0.01V*/
	int16_t DCCurrent;			/* unit = A */
	uint16_t CapRemain;			/* unit = 0.01Ah*/
	uint16_t Soc;				    /* unit = % */
} BatCanMsg_Tod0x020_t;

/*=========================================
 * Definition of TOD 0x21
 =========================================*/
typedef struct
{
	int16_t CellTemp;				    /* unit = 'C*/
	BatMosFetStatus_u FetStatus;		/*  */
	uint16_t ErrorFlags;			    /* unit = 0.01Ah*/
	uint8_t Byte6;					    /* Reserved byte #6*/
	uint8_t Soh;					    /* unit = % */
} BatCanMsg_Tod0x021_t;

typedef union
{
	uint8_t u8[8];
	int8_t i8[8];
	uint16_t u16[4];
	int16_t i16[4];
	uint32_t u32[2];
	int32_t i32[2];
	BatCanMsg_Tod0x020_t Tod0x20;
	BatCanMsg_Tod0x021_t Tod0x21;
}BatCanMsgs_u;

typedef enum BatMainSM_e
{
  BAT_MAIN_SM_INIT = 0,
  BAT_MAIN_SM_IDLE,
  BAT_MAIN_PWR_ON,
  BAT_MAIN_ACTIVATED,
  BAT_MAIN_PWR_OFF,
  BAT_MAIN_ALARM,
} BatMainSM_t;

typedef enum BatPwrOnSM_e
{
  BAT_PWR_ON_SM_IDLE = 0,
  BAT_PWR_ON_SM_START,
  BAT_PWR_ON_SM_WAIT_FOR_VOLT,
  BAT_PWR_ON_SM_WAIT_FOR_MOSFET_STATUS,
  BAT_PWR_ON_SM_COMPLETE,
  BAT_PWR_ON_SM_FAIL,
} BatPwrOnSM_t;

typedef enum BatPwrOffSM_e
{
  BAT_PWR_OFF_SM_IDLE = 0,
  BAT_PWR_OFF_SM_START,
  BAT_PWR_OFF_SM_WAIT_FOR_MOSFET_OFF,
  BAT_PWR_OFF_SM_STOP_BROADCAST,
  BAT_PWR_OFF_SM_COMPLETE,
  BAT_PWR_OFF_SM_FAIL,
} BatPwrOffSM_t;

typedef enum BatPwrCtrl_e
{
	BAT_PWR_CTRL_OFF = 0x00,
	BAT_PWR_CTRL_ON = 0x01
}BatPwrCtrl_t;

typedef enum BatBdCtrl_e
{
	BAT_BD_CTRL_OFF = 0x00,
	BAT_BD_CTRL_ON = 0x01
}BatBdCtrl_t;

#define BAT_VDIFF_BAT_INV_THRESHOLD_V	1.0f	/*unit V*/
#define BAT_INV_DCBUS_VOLT_THRESHOLD_V	30.0f	/*unit V*/
#define BAT_WAIT_FOR_VOLT_TIME_THRESHOLD_MS 2000 -1
#define BAT_WAIT_FOR_MOSFET_TIME_THRESHOLD_MS 2000 -1
#define BAT_WAIT_FOR_STOP_BROADCAST_TIME_THRESHOLD_MS 20 -1

typedef enum BatPwrOnErr_e
{
	BAT_PWR_ON_ERR_OK = 0,
	BAT_PWR_ON_ERR_DCV_TOO_LOW,
	BAT_PWR_ON_ERR_BAT_INFO_NOT_ENOUGH,
	BAT_PWR_ON_ERR_VDIFF_TOO_HIGH,
	BAT_PWR_ON_ERR_MOSFET_CTRL_FAIL,
	BAT_PWR_ON_ERR_BMS_ERROR_REPORTED,
	BAT_PWR_ON_ERR_UNDEFINED_STATE,
} BatPwrOnErr_t;

typedef enum BatPwrOffErr_e
{
	BAT_PWR_OFF_ERR_OK = 0,
	BAT_PWR_OFF_ERR_BAT_INFO_NOT_ENOUGH,
	BAT_PWR_OFF_ERR_MOSFET_CTRL_FAIL,
	BAT_PWR_OFF_ERR_BMS_ERROR_REPORTED,
	BAT_PWR_OFF_ERR_UNDEFINED_STATE,
} BatPwrOffErr_t;

typedef struct
{
	BatPwrCtrl_t PwrCtrlCmd;
	BatBdCtrl_t BdCtrlCmd;
	BatMainSM_t MainSm;
	BatPwrOnSM_t PwrOnSM;
	BatPwrOnErr_t PwrOnErr;
	uint16_t PwrOnToCnt;
	BatPwrOffSM_t PwrOffSM;
	BatPwrOffErr_t PwrOffErr;
	uint16_t PwrOffToCnt;
	uint16_t TimerPrescaler;
	uint16_t ReservedU16;
	float DcBusVoldNow;
	ExtranetCANStation_t *pExCANStation;
} BatCtrl_t;

extern void Bat_CanMsgLoad(uint32_t Id, uint8_t *pData);
extern void Bat_InvDcVoltSet(float VdcIn);
extern void Bat_Do100HzLoop (void);
extern BatMainSM_t Bat_MainSMGet (void);
extern BatPwrOnSM_t Bat_PwrOnSMGet (void);
extern BatPwrOffSM_t Bat_PwrOffSMGet (void);
extern void Bat_PwrOffReq(void);
extern void Bat_PwrOnReq(void);
extern void Bat_CanHandleLoad(ExtranetCANStation_t *pIn);

typedef void (*functypBat_CanMsgLoad)(uint32_t, void*);
typedef void (*functypBat_InvDcVoltSet)(float);
typedef void (*functypBat_Do100HzLoop)(void);
typedef BatMainSM_t (*functypBat_MainSMGet)(void);
typedef BatPwrOnSM_t (*functypBat_PwrOnSMGet)(void);
typedef BatPwrOffSM_t (*functypBat_PwrOffSMGet)(void);
typedef void (*functypBat_PwrOffReq)(void);
typedef void (*functypBat_PwrOnReq)(void);
typedef void (*functypBat_CanHandleLoad)(void *);

typedef struct
{
	functypBat_CanMsgLoad CanMsgLoad;
	functypBat_InvDcVoltSet InvDcVoltSet;
	functypBat_Do100HzLoop Do100HzLoop;
	functypBat_MainSMGet MainSMGet;
	functypBat_PwrOnSMGet PwrOnSMGet;
	functypBat_PwrOffSMGet PwrOffSMGet;
	functypBat_PwrOffReq PwrOffReq;
	functypBat_PwrOnReq PwrOnReq;
	functypBat_CanHandleLoad CanHandleLoad;
} BatStation_t;

#define BAT_STATION_DEFAULT	\
{	\
		(functypBat_CanMsgLoad)Bat_CanMsgLoad,		/*CanMsgLoad*/ \
		(functypBat_InvDcVoltSet)Bat_InvDcVoltSet,	/*InvDcVoltSet*/ \
		(functypBat_Do100HzLoop)Bat_Do100HzLoop,	/*Do100HzLoop*/ \
		(functypBat_MainSMGet)Bat_MainSMGet,		/*MainSMGet*/ \
		(functypBat_PwrOnSMGet)Bat_PwrOnSMGet,		/*PwrOnSMGet*/ \
		(functypBat_PwrOffSMGet)Bat_PwrOffSMGet,	/*PwrOffSMGet*/ \
		(functypBat_PwrOffReq)Bat_PwrOffReq,		/*PwrOffReq*/ \
		(functypBat_PwrOnReq)Bat_PwrOnReq,			/*PwrOnReq*/ \
		(functypBat_CanHandleLoad)Bat_CanHandleLoad,/*CanHandleLoad*/ \
}	\

extern BatStation_t BatStation;

#endif /* INC_BATCTRL_H_ */
