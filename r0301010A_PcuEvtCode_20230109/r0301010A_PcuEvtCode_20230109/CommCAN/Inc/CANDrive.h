/*
 * INC_CAN_DRIVE_H.h
 *
 *  Created on: Jan 10, 2020
 *      Author: Will
 */

#ifndef INC_SLC_CAN_H_
#define INC_SLC_CAN_H_

#include "stm32g4xx_hal.h"

/*Define*/
#define BUFFER_IDX_RESERVED 		0
#define BUFFER_IDX_0 				0
#define BUFFER_IDX_1 				1
#define BUFFER_IDX_2 				2
#define CAN_ID_CONFIG_ARRAY_SIZE	6

#define CAN_ID_CONFIG_RESERVED 		(uint8_t)0
#define CAN_ID_CONFIG_USED 	 		(uint8_t)1
#define CAN_ID_CONIFG_TYPE_STANDARD (uint8_t)0
#define CAN_ID_CONIFG_TYPE_EXTENDED (uint8_t)1

typedef struct
{
	uint32_t	Id1;
	uint32_t	Id2;
	union
	{
		struct
		{
			uint32_t FilterType	:4;
			uint32_t IdType		:4;
			uint32_t ConfigUsage:4;
			uint32_t RxFifo 	:4;
			uint32_t Reserved	:16;
		}Bits;
		uint32_t All;
	}Setup;
} CanIdConfig_t;

typedef struct
{
	uint32_t ItSelect;
	uint32_t ItActivateItem;
	uint32_t ItLineNumber;
	uint32_t RxFifoSrc;
}CanModuleConfig_t;

typedef struct
{
	uint32_t SFID2 	:11;
	uint32_t Res	:5;
	uint32_t SFID1	:11;
	uint32_t SFEC	:3;
	uint32_t SFT	:2;
}StdCanIdFilter_t;
/*Variable*/


/*function*/
typedef void(*funcTypeCANDrive_ModuleConfig)(void* ,const void*);
typedef void(*funcTypeCANDrive_IdConfig)(void* ,const void*);
typedef void(*funcTypeCANDrive_LoadParam)(void*, const void*, const void*);
typedef void(*funcTypeCANDrive_Init)(void*, void*);




typedef struct
{
	const CanModuleConfig_t		*pModuleConfigData;
	const CanIdConfig_t 		*pIdConfigTable;
	funcTypeCANDrive_ModuleConfig	ModuleConfig;
	funcTypeCANDrive_IdConfig 		IdCondfig;
	funcTypeCANDrive_LoadParam		LoadParam;
	funcTypeCANDrive_Init			Init;
}CanDriveSetup_t;



void CANDrive_ModuleConfig( FDCAN_HandleTypeDef *p, const CanModuleConfig_t *v );
void CANDrive_IdConfig(FDCAN_HandleTypeDef *p, const CanIdConfig_t *v);
void CANDrive_LoadParam(CanDriveSetup_t *v, const CanModuleConfig_t *m, const CanIdConfig_t *p );
void CANDrive_Init(FDCAN_HandleTypeDef *p,  CanDriveSetup_t *v );
void ByteSwap( uint32_t *Data, uint8_t Size);
#define CAN_DRIVE_SETUP_DEFAULT		\
{									\
	0,0,							\
	(funcTypeCANDrive_ModuleConfig)CANDrive_ModuleConfig,	\
	(funcTypeCANDrive_IdConfig)CANDrive_IdConfig,			\
	(funcTypeCANDrive_LoadParam)CANDrive_LoadParam,			\
	(funcTypeCANDrive_Init)CANDrive_Init,				\
}									\

#endif /* INC_CAN_DRIVE_H_ */
