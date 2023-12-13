/*
 * Drive.h
 *
 *  Created on: Dec 19, 2019
 *      Author: MikeSFWen
 */

#ifndef INC_DRIVE_H_
#define INC_DRIVE_H_

#include "UtilityBase.h"
//#include <AsIc.h>
//#include <PsbCtrl_PCU.h>
#include "stm32g4xx_hal.h"
#include "stm32g4xx_ll_pwr.h"
#include "ExtFlash.h"
#include "ParamMgr.h"
#include "UartStation.h"
#include "PWM_TRA.h"
#include "ADC_DRI.h"
#include "UartStation.h"
#include "ExtranetCANStation.h"
#include "ParamMgr.h"
#include "ExtFlash.h"
#include "TotalTime.h"
#include "CANDrive.h"
#include "UdsServiceCtrl.h"
#include "MFCodeInc.h"
#include "GlobalAlarmDetect.h"
#include "UdsSecurityAccess.h"
#include "IntFlash.h"
#include "PWM_RC.h"
#include "RcUartComm.h"
#include "DiagnosticTroubleCode.h"
#include "RemainingTime.h"
#include "PositionSensor.h"

#if JUDGE_FUNCTION_DELAY || MEASURE_CPU_LOAD
typedef struct
{
	uint32_t previousTimestamp;
	uint32_t deltaCnt;
	uint32_t maxDelta;
	float AveDelta;
	float Intervals_us;
	float Max_Intervals_us;
	float Ave_Intervals_us;
}Judge_Delay;
#endif

typedef struct {
	uint32_t PWMCounter;
	uint32_t ADCCounter;
	uint32_t PLCLoopCounter;
	uint32_t Loop100HzCounter;
	uint32_t Loop10HzCounter;
} CPUCounter_t;

typedef enum
{
	Target_None		=0,
	Target_EFlash	=1,
	Target_RAM		=2,
	Target_PSB		=3,
}TargetDefine_e;

typedef enum{
	VEHICLE_STATE_INITIALIZING,
	VEHICLE_STATE_STANDBY,
	VEHICLE_STATE_NORMAL,
	VEHICLE_STATE_WARNING,
	VEHICLE_STATE_LIMPHOME,
	VEHICLE_STATE_ALARM,
	VEHICLE_STATE_POWER_OFF,
}VEHICLE_STATE_e;

typedef enum{
	ESC_OP_INITIALIZING,
	ESC_OP_STANDBY,
	ESC_OP_NORMAL,
	ESC_OP_WARNING,
	ESC_OP_LIMPHOME,
	ESC_OP_ALARM,
	ESC_OP_POWER_OFF,
}ESC_OP_STATE_e; // ESC operation

// Define the number of different data index
#define HW_VER_NUM_IDX		7 // PCU hardware version
#define PART_NUM_IDX		24 // PCU part number
#define SW_VER_NUM_IDX		9 // PCU software version
#define SYS_SUP_ID_IDX		8 // system supplier ID
#define SERIAL_NUM_IDX		18 // PCU serial number
#define BOOT_VER_IDX		4 // PCU boot-loader version
#define MOT_SERIAL_NUM_IDX	30 // motor(PSB) serial number
#define MOT_HW_VER_NUM_IDX	4 // motor(PSB) hardware version number
#define MOT_SW_VER_NUM_IDX	9 // motor(PSB) software version number
#define MAX_UDS_DATA_BUF	15 // Maximum UDS data buffer is (MOT_SERIAL_NUM_IDX>>1)

#define CPU_COUNTER_DEFAULT { \
	0, \
	0, \
	0, \
	0, \
	0 }

extern CPUCounter_t CPUCounter;
//extern UART_Module UART_Module1;
extern AdcStation AdcStation1;
extern PwmStation PwmStation1;
extern AlarmMgr_t AlarmMgr1;
extern ExtranetCANStation_t ExtranetCANStation;
extern PS_t PSStation1;

extern FDCAN_HandleTypeDef hfdcan1;
extern FDCAN_HandleTypeDef hfdcan2;
extern TIM_HandleTypeDef htim6;
extern TIM_HandleTypeDef htim7;
extern TIM_HandleTypeDef htim16;
extern DAC_HandleTypeDef hdac1;
extern CORDIC_HandleTypeDef hcordic;
extern UART_HandleTypeDef huart5;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim4;
extern TIM_HandleTypeDef htim5;
extern USART_HandleTypeDef husart2;
extern CRC_HandleTypeDef hcrc;
//extern UART_HandleTypeDef huart3;

extern uint8_t PCUStatus;

extern const uint8_t BomNumber[PART_NUM_IDX];
extern const uint8_t SystemSupplyerID[SYS_SUP_ID_IDX];
extern uint8_t FWVerNumber[ SW_VER_NUM_IDX ];
extern uint8_t HWVerNumber[ HW_VER_NUM_IDX ];
extern uint8_t PSBVerNumber[ MOT_SW_VER_NUM_IDX ];

extern void drive_Init(void);
extern void drive_DoCurrentLoop(void);
extern void drive_DoPLCLoop(void);
extern void drive_Do100HzLoop(void);
extern void drive_Do10HzLoop(void);
extern void drive_Do1HzLoop(void);
extern void drive_DoTotalTime(void);
extern void drive_DoHouseKeeping(void);
extern void drive_DoPwmRcCatch(void);
extern void drive_DoPwmPositionCatch(TIM_HandleTypeDef *htim);
extern void drive_DoLoad_DataToAdcGain(void);
extern void drive_ThrottleGainInit( DriveParams_t *d, AdcStation *a );
extern void drive_DcBusGainInit( DriveParams_t *d, AdcStation *a );
extern void drive_DoExtFlashTableRst( uint32_t *Setup, uint32_t *Ena, uint32_t *BackUpExMemEna, const System_Table_t_Linker *Ts, SystemParams_t *pSysT, const PCU_Table_t_Linker *Tp, PCUParams_t *pPcuT );
extern void drive_DoHWOCPIRQ(void);
/*
 * Version Read #define
 */
#define APP_START_ADDRESS			0x08040000
#define APP_IN_START_ADDRESS        0x08040200
#define SYS_TAB_START_ADDRESS		0x08008800
#define PCU_TAB_START_ADDRESS		0x0800A800
#define MOT_1_TAB_START_ADDRESS		0x0800C800
#define	MOT_2_TAB_START_ADDRESS		0x08011800

#define SUPPLYER_ID	{ \
		ELEMENT_1ST, \
		ELEMENT_2ND, \
		ELEMENT_3RD, \
		ELEMENT_4TH, \
		ELEMENT_5TH, \
		ELEMENT_6TH, \
		ELEMENT_7TH, \
		ELEMENT_8TH, \
}

#define PART_NUMBER { 	\
		NUMBER_00,		\
		NUMBER_01,		\
		NUMBER_02,		\
		NUMBER_03,		\
		NUMBER_04,		\
						\
		NUMBER_05,		\
		NUMBER_06,		\
		NUMBER_07,		\
		NUMBER_08,		\
		NUMBER_09,		\
						\
		NUMBER_10,		\
		NUMBER_11,		\
		NUMBER_12,		\
		NUMBER_13,		\
		NUMBER_14,		\
						\
		NUMBER_15,		\
		NUMBER_16,		\
		NUMBER_17,		\
		NUMBER_18,		\
		NUMBER_19,		\
						\
		NUMBER_20,		\
		NUMBER_21,		\
		NUMBER_22,		\
		NUMBER_23,		\
}

#define BOOT_VER_NUMBER { 	\
		BOOT_VER_BYTE_0,		\
		BOOT_VER_BYTE_1,		\
		BOOT_VER_BYTE_2,		\
		BOOT_VER_BYTE_3,		\
}

/*
 * Boot-loader Function #define & enum
 */
// #define
#define BOOT_ADDRESS		0x08000000
#define BOOT_VER_ADDRESS	0x08007FF5

// Enum
enum Boot_Trig_Enum{
	BOOT_DIS = 0,
	BOOT_ENA
};

/*
 * Boot-loader Function Variable
 */
extern uint16_t BootAppTrig;

/*
 * Boot-loader Function declare
 */
typedef void (*pfunction)(void);		//Jump Code Pointer Function type define
extern void JumpCtrlFunction( void );
extern void JumpCode_ApplicationToBootloader( uint32_t BootAddress );
extern void DisableMcuModule( void );

typedef enum {
	PcuInitState_Inital = 0,
	PcuInitState_Ready,
} PcuInitState_e;


#endif /* INC_DRIVE_H_ */
