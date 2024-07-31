/*
 * Vsm.h
 *
 *  Created on: 2024年6月28日
 *      Author: egghe
 */

#ifndef INC_VEHICLECTRL_H_
#define INC_VEHICLECTRL_H_

#include "stm32g4xx.h"
#include "E10App.h"


/* Marco define */
#define VEHICLE_SM_OFFLINE_TEST			1
#define VEHICLE_TIMER_LOOP_PERIOD_MS	10
#define VEHICLE_DRIVE_ENABLE_TIME_MS	1000				/* regard as parameter "tDriveEna" in TRD document */
#define VEHICLE_DRIVE_ENABLE_TIME_CNT	VEHICLE_DRIVE_ENABLE_TIME_MS/VEHICLE_TIMER_LOOP_PERIOD_MS
#define VEHICLE_MOTOR_STEADY_SPD_RPM_THRESHOLD		100		/* regard as  parameter "nMotOff" in TRD document */
#define VEHICLE_DC_BUS_WORKING_VOLT_THRESHOLD		40		/* regarded as parameter "MinBattVltg" in TRD document */
#define VEHICLE_THROTTLE_RELEASE_THRESHOLD			1		/* throttle release threshold in % */
#define VEHICLE_BTN_RELEASE_FLAG_NONE
#define VEHICLE_BTN_RELEASE_FLAG_BOOST   	0x0001
#define VEHICLE_BTN_RELEASE_FLAG_REVERSE 	0x0002
#define VEHICLE_BTN_RELEASE_FLAG_ALL    	(VEHICLE_BTN_RELEASE_FLAG_BOOST | VEHICLE_BTN_RELEASE_FLAG_REVERSE)


/*Enum definition*/


/*
 * Note : this enum value should never exceed "15(0x0F)"
 */
typedef enum
{
	VEHICLE_SM_NOT_READY,			// Regard as "STARTUP" state, 12V is supplied and MCU is awake"
	VEHICLE_SM_STANDBY,
	VEHICLE_SM_ENERGIZED,			// 48V is available
	VEHICLE_SM_TQCTRL,				// Torque control / Torque Generator
	VEHICLE_SM_SPDCTRL,				// not used in P1
	VEHICLE_SM_DISCHARGE,			// not used in P1
	VEHICLE_SM_ERROR,				// Error state
	VEHICLE_SM_ANGLE_LEARNING,		// not used in P1
	VEHICLE_SM_POWERDOWN,			// not used in P1
	VEHICLE_SM_PRE_SHUTDOWN,
	VEHICLE_SM_SHUTDOWN,
	VEHICLE_SM_CHARGE,
} Vehicle_Sm_e;


/*Structure definition*/

typedef struct
{
	Vehicle_Sm_e VSm;
	uint8_t DualBtnReleaseFlag;
	uint16_t DualBtnTimeCnt;
} VehicleCtrl_t;

/* public functions */
extern void Vehicle_Do100HzLoop(void);
extern Vehicle_Sm_e Vehicle_MainSMGet(void);


#endif /* INC_VEHICLECTRL_H_ */
