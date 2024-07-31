/*
 * BatApp.h
 *
 *  Created on: 2024年1月3日
 *      Author: egghe
 */

#ifndef INC_BATCTRL_H_
#define INC_BATCTRL_H_

#include "ExtranetCANStation.h"

#define BAT_CANID_FILTER_START_01	0x30C
#define BAT_CANID_FILTER_END_01		0x31C

#define BAT_CANID_FILTER_START_02	0x507
#define BAT_CANID_FILTER_END_02		0x509

typedef enum
{
	BYTE_ORDER_SMALL_ENDIAN = 0x00,
	BYTE_ORDER_BIG_ENDIAN,
} ByteOrderMapping_e;

typedef enum
{
	BAT_IDX_MASTER = 0x00,
	BAT_IDX_SLAVE,
	BAT_IDX_SUMMERY,
	BAT_IDX_NUMBER,
} Bat_Idx;

typedef enum
{
  BAT_MAIN_SM_OFF = 0x00,
  BAT_MAIN_SM_STANDBY = 0x01,
  BAT_MAIN_SM_ENERGIZE,
  BAT_MAIN_SM_CHARGE,
  BAT_MAIN_SM_SHUTDOWN,
  BAT_MAIN_SM_ERROR,
} BatMainSM_e;

typedef enum
{
	BAT_FLG_OFF = 0x00,
	BAT_FLG_ON,
} Bat_flg_e;

typedef enum
{
	BAT_DCDC_STATE_OFF = 0x00,
	BAT_DCDC_STATE_ON,
	BAT_DCDC_STATE_ERROR,
} Bat_Dcdc_e;

typedef enum
{
	BAT_ST_BATT_PRCH_OFF = 0x00,
	BAT_ST_BATT_PRCH_ACTIVE,
	BAT_ST_BATT_PRCH_FAILED,
	BAT_ST_BATT_PRCH_SUCCESSFUL,
} Bat_stBattPreChrgSt_e;

typedef enum
{
	BAT_ISO_MON_DEV_UNKOWN = 0x00,
	BAT_ISO_MON_DEV_ISO_OK,
	BAT_ISO_MON_DEV_WARNING,
	BAT_ISO_MON_DEV_ERROR,
} Bat_IsoMonDev_e;

typedef enum
{
	BAT_CHARGER_STATE_DISABLE = 0x00,
	BAT_CHARGER_STATE_CHARGE,
	BAT_CHARGER_STATE_ERROR,
} Bat_ChrgState_e;

/*
 * todo: need more detailed charge mode
 */
typedef enum
{
	BAT_CHARGE_MODE_undefind = 0x00,
}Bat_ChrgMod_e;

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
} CanIdField_u;

typedef enum
{
	BAT_INFO_IDX_DC_VOLT_NOW,		/* float, unit = V */
	BAT_INFO_IDX_DC_CURR_NOW,		/* float, unit = A */
	BAT_INFO_IDX_CAPACITY_REMAIN,	/* float, unit = AH */
	BAT_INFO_IDX_DISCHARGE_CURR_LIMIT,	/* float, unit = A */
	BAT_INFO_IDX_CHARGE_CURR_LIMIT,	/* float, unit = A */
	BAT_INFO_IDX_REGEN_CURR_LIMIT,	/* float, unit = A */
	BAT_INFO_IDX_MAIN_SM,
	BAT_INFO_IDX_PRCH_SM,
	BAT_INFO_IDX_FLAG_ALARM,
	BAT_INFO_IDX_FLAG_WARNING,
	BAT_INFO_IDX_FLAG_SHUTDOWN_REQ,
	BAT_INFO_IDX_FLAG_BALANCE_BUSY,
	BAT_INFO_IDX_FLAG_CHARGE_BUSY,
	BAT_INFO_IDX_SOC,
	BAT_INFO_IDX_MAX,
} BatInfoIdx_e;

typedef struct
{
	float DCVolt;			    /* unit = V */
	float DcCurrent;		    /* unit = A */
	float CapRemain;		    /* unit = Ah */
	float DchrgLimit;			/* Discharge current limit from battery, unit = A */
	float ChrgLimit;			/* Charge current limit from battery, unit = A */
	float RgnLimit;				/* regen current limit from battery, unit = A */
	BatMainSM_e MainSM;
	Bat_stBattPreChrgSt_e PrchSM;
	union
	{
		uint8_t All;
		struct
		{
			uint8_t Alarm :1;
			uint8_t Warn :1;
			uint8_t ShutdownReq :1;
			uint8_t BalBsy :1;
			uint8_t ChrgBsy :1;
			uint8_t Rsved:3;
		}Bit;
	}Flags;
	uint8_t Soc;			    /* unit = %*/
} BatInfo_t;

#define CAN_INFORM_FORM_BAT_DEFAULT \
{                                   \
  0.0f,                       /*DCVolt*/ \
  0.0f,                       /*DcCurrent*/ \
  0.0f,                       /*CapRemain*/ \
  0.0f,                       /*DchrgLimit*/ \
  0.0f,                       /*ChrgLimit*/ \
  0.0f,                       /*RgnLimit*/ \
  BAT_MAIN_SM_OFF,            /*MainSM*/ \
  BAT_ST_BATT_PRCH_OFF,		  /*PrchSM*/\
  {0},						  /*Flags*/\
  0,                          /*Soc*/ \
}                                   \

typedef struct
{
	Bat_flg_e shutdownReq;
} BatCmd_t;

/*=========================================
 * Definition of id31E_50_INV CAN Tx Msg to Battery <Reverted>
 * for CAN ID 0x31E
 * Note : This Reverted structure definition is for Motorola byte order(big Endian),
 *        when receiving messages, the 8byte data array need to be Reverted before mapping to this structure
 *        when transmitting messages, the 8byte data array need to be Reverted after value is assigned to this structure
 * =========================================*/
typedef struct
{
	uint8_t nrMsgChksXor06;
	union
	{
		uint8_t All;
		struct
		{
			uint8_t cntrMsg15 : 4;
			uint8_t b4To7 : 4;
		} Bit;
	}Byte6;

	uint8_t Byte5;
	uint8_t Byte4;
	uint8_t Byte3;
	uint8_t Byte2;
	uint8_t Byte1;
	union
	{
		uint8_t All;
		struct
		{
			uint8_t stInvr : 4;
			uint8_t flgInvrShdwnRdy : 4;
		} Bit;
	}Byte0;
} BatCanMsgTx_id31E_50_INV_Rvrt_t;

#define BATCANMSGTX_IDid31E_50_INV_DEFAULT \
{\
	0,		\
	{0},	\
	0,		\
	0,		\
	0,		\
	0,		\
	0,		\
	{0},	\
}\

/*=========================================
 * Definition of id30C_50_BMSE10 CAN Rx Msg From Battery <Reverted>
 * for ID 0x30C and 0x30D
 * Note : This Reverted structure definition is for Motorola byte order(big Endian),
 *        the 8byte data array need to be Reverted before mapping to this structure
 =========================================*/
typedef struct
{
	uint8_t nrMsgChksXor06;
	union
	{
		uint8_t All;
		struct
		{
			uint8_t cntrMsg15 :4;
			uint8_t b4To7 :4;
		} Bit;
	} Byte6;

	uint8_t Byte5;
	uint8_t Byte4;
	uint8_t stBattPreChrgSt;
	union
	{
		uint8_t All;
		struct
		{
			uint8_t stMstSlave :2;		// undefined signal, unused for now,
			uint8_t Reserved23 :2;		// reserved bit 2 to bit 3
			Bat_Dcdc_e stBatDcdc :3;	// status of DCDC in Battery
			Bat_flg_e flgCoolFan :1;	// flag to control cool fan in charger
		}Bis;
	} Byte2;

	union
	{
		uint8_t All;
		struct
		{
			Bat_flg_e flgBattWarn	:1;			// Battery warning flag
			Bat_flg_e flgBattShutdownReq :1;	// An shutdown request from battery
			Bat_flg_e flgBattLimp	:1;			// battery limp Home flag
			Bat_flg_e flgBattChrgReq:1;			//
			Bat_flg_e flgBattAlarm	:1;			// Battery Alarming flag
			Bat_flg_e flgBalBsy		:1;			// Battery balancing busy flag
			Bat_IsoMonDev_e stsoMonDev :2;		// Iso test indication <defined by each battery vender>
		}bits;
	} Byte1;

	union
	{
		uint8_t All;
		struct
		{
			BatMainSM_e StBatt 		:3;		// BMS State of this battery
			BatMainSM_e StBattCmd 	:3;		// BMS state control command to other battery
			Bat_flg_e flgChrgCnct	:1;		// charger connection indication
			Bat_flg_e flgChrgBsy	:1;		// Battery Charging busy flag
		}Bit;
	} Byte0;
} BatCanMsgRx_id30C_50_BMSE10_Rvrt_t;


/* =========================================
 * Definition of id30F_50_BMSE10 CAN Rx Msg From Battery <Reverted>
 * for CAN ID 0x30F, 0x318 and 0x31A
 * Note : This Reverted structure definition is for Motorola byte order(big Endian),
 *        the 8byte data array need to be Reverted before mapping to this structure
 * =========================================*/
typedef struct
{
	uint8_t nrMsgChksXor06;
	union
	{
		uint8_t All;
		struct
		{
			uint8_t cntrMsg15 :4;
			uint8_t b4To7 :4;
		} Bit;
	} Byte6;

	uint8_t Byte5;
	uint64_t RsvedB4b0ToB1:2;
	uint64_t iRgnLimn :8;		//factor=1, unit 1A
	uint64_t iDchaLimn :8;		//factor=1, unit 1A
	uint64_t iChrgLimn :8;		//factor=1, unit 1A
	int64_t iBattCurr :14;		//factor=1, unit 1A

} BatCanMsgRx_id30F_50_BMSE10_Rvrt_t;

/* =========================================
 * Definition of id31B_50_BMSE10 CAN Rx Msg From Charger <Reverted>
 * for CAN ID 0x31B, 0x31C
 * Note : This Reverted structure definition is for Motorola byte order(big Endian),
 *        the 8byte data array need to be Reverted before mapping to this structure
 * =========================================*/
typedef struct
{
	uint8_t nrMsgChksXor06;
	union
	{
		uint8_t All;
		struct
		{
			uint8_t cntrMsg15 :4;
			uint8_t b4To7 :4;
		} Bit;
	} Byte6;

	uint8_t Byte5;
	uint8_t uCellMinIdx;
	uint32_t uCellMaxIdx :8;
	uint32_t uCellMaxDelta :14;		//Factor = 1, unit = mV
	uint32_t uBattTerminal :10;		//Factor = 0.1, unit = V
} BatCanMsgRx_id31B_50_BMSE10_Rvrt_t;

/* =========================================
 * Definition of id31D_50_BMSE10 CAN Rx Msg From Charger <Reverted>
 * for CAN ID 0x31D
 * Note : This Reverted structure definition is for Motorola byte order(big Endian),
 *        the 8byte data array need to be Reverted before mapping to this structure
 * =========================================*/
typedef struct
{
	uint8_t nrMsgChksXor06;
	union
	{
		uint8_t All;
		struct
		{
			uint8_t cntrMsg15 :4;
			uint8_t b4To7 :4;
		} Bit;
	} Byte6;

	uint8_t Byte5;
	uint8_t Byte4;
	uint8_t Byte3;
	uint8_t Byte2;
	uint8_t Byte1;
	union
	{
		uint8_t All;
		struct
		{
			Bat_ChrgState_e stChrgr :2;
			Bat_ChrgMod_e chrgMod :3;
			uint8_t Rsvedb5Tob7 :3;
		} Bit;
	} Byte0;

} BatCanMsgRx_id31D_50_Chrgr_Rvrt_t;

/* =========================================
 * Definition of id507_200_BMSE10 CAN Rx Msg From battery <Reverted>
 * for CAN ID 0x507, 0x508
 * Note : This Reverted structure definition is for Motorola byte order(big Endian),
 *        the 8byte data array need to be Reverted before mapping to this structure
 * =========================================*/
typedef struct
{
	uint8_t nrMsgChksXor06;
	union
	{
		uint8_t All;
		struct
		{
			uint8_t cntrMsg15 :4;
			uint8_t b4To7 :4;
		} Bit;
	} Byte6;

	uint8_t Byte5;
	uint8_t Byte4;
	uint8_t ratSoh;			// factor = 1, unit = %
	uint8_t ratBattSoc;		// factor = 1, unit = %
	uint16_t qBattSoc;		// factor = 0.001, unit = Ah

} BatCanMsgRx_id507_200_BMSE10_Rvrt_t;


typedef union
{
	uint8_t u8[8];
	int8_t i8[8];
	uint16_t u16[4];
	int16_t i16[4];
	uint32_t u32[2];
	int32_t i32[2];
	uint64_t u64;
	int64_t i64;
	float f32[2];
	BatCanMsgRx_id30C_50_BMSE10_Rvrt_t id30C;
	BatCanMsgRx_id30F_50_BMSE10_Rvrt_t id30F;
	BatCanMsgRx_id31B_50_BMSE10_Rvrt_t id31B;
	BatCanMsgRx_id31D_50_Chrgr_Rvrt_t id31D;
	BatCanMsgRx_id507_200_BMSE10_Rvrt_t id507;

} BatCanMsgRx_u;


typedef struct
{
	BatMainSM_e MainSm;
	uint16_t TimerPrescaler;
	ExtranetCANStation_t *pExCANStation;
} BatCtrl_t;

extern void Bat_CanMsgLoad(uint32_t Id, uint8_t *pData);
extern void Bat_InfoReset(void);
extern void Bat_Do100HzLoop (void);
extern void Bat_ShutdownReq(void);
extern void Bat_InfoGet(void* pOut, BatInfoIdx_e InfoIdxIn, Bat_Idx BatIdxIn);
extern void Bat_CanHandleLoad(ExtranetCANStation_t *pIn);
extern BatMainSM_e Bat_MainSMGet (Bat_Idx Idx);
extern Bat_stBattPreChrgSt_e Bat_PrchSMGet (Bat_Idx Idx);
extern uint8_t Bat_SocGet (Bat_Idx Idx);
extern uint8_t Bat_ShutdownReqFlgGet(Bat_Idx Idx);

#endif /* INC_BATCTRL_H_ */
