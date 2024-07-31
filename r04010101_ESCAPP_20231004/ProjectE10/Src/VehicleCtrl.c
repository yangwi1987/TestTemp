/*
 * Vsm.c
 *
 *  Created on: 2024年6月28日
 *      Author: Will
 */
#include "VehicleCtrl.h"
#include "Drive.h"
#include "BatCtrl.h"
#include "E10App.h"
#include "AlarmTable.h"


/*private variable */
VehicleCtrl_t VehicleCtrl = {0};

/*global/public variable*/


/* Function definition*/



void Vehicle_EnterStandByState()
{
	/*clear battery information*/
	Bat_InfoReset();
	/* Enable CAN1 timeout detection */
	drive_AlarmEnableReq(ALARMID_CAN1_TIMEOUT, ENABLE);
	/* Disable UVP detection */
	drive_AlarmEnableReq(ALARMID_UNDER_VOLTAGE_BUS, DISABLE);
	/*Enable global Alarm detection*/
//	drive_GlobalAlarmEnableReq(ENABLE);
	drive_GlobalAlarmEnableReq(DISABLE);
	/* Put INV to servo-off */
	drive_ServoCtrlReq(DRIVE_SERVO_CTRL_OFF);
	VehicleCtrl.VSm = VEHICLE_SM_STANDBY;
}

void Vehicle_EnterEnergizedState()
{
	/* Enable UVP detection */
	drive_AlarmEnableReq(ALARMID_UNDER_VOLTAGE_BUS, ENABLE);
	drive_ServoCtrlReq(DRIVE_SERVO_CTRL_OFF);
	VehicleCtrl.DualBtnTimeCnt = 0;
	VehicleCtrl.DualBtnReleaseFlag = 0;
	VehicleCtrl.VSm = VEHICLE_SM_ENERGIZED;
}

void Vehicle_EnterTqGeneratorState()
{
	VehicleCtrl.DualBtnTimeCnt = 0;
	VehicleCtrl.DualBtnReleaseFlag = 0;
	VehicleCtrl.VSm = VEHICLE_SM_TQCTRL;
}

void Vehicle_EnterErrordState()
{
	/*executing data log save to flash/eeprom process if required */
	/* Disable global Alarm detection*/
	drive_GlobalAlarmEnableReq(DISABLE);
	/* Disable UVP detection */
	drive_AlarmEnableReq(ALARMID_UNDER_VOLTAGE_BUS, DISABLE);
	/* Disable CAN1 timeout detection */
	drive_AlarmEnableReq(ALARMID_CAN1_TIMEOUT, DISABLE);
	/* put INVERTER to SERVO OFF*/
	drive_ServoCtrlReq(DRIVE_SERVO_CTRL_OFF);

	Bat_ShutdownReq();
	VehicleCtrl.VSm = VEHICLE_SM_ERROR;
}

void Vehicle_EnterPreShutdownState()
{
	drive_ServoCtrlReq(DRIVE_SERVO_CTRL_OFF);
	VehicleCtrl.VSm = VEHICLE_SM_PRE_SHUTDOWN;
}

void Vehicle_EnterShutdownState()
{
	drive_ServoCtrlReq(DRIVE_SERVO_CTRL_OFF);
	VehicleCtrl.VSm = VEHICLE_SM_SHUTDOWN;
	Bat_ShutdownReq();
}

void Vehicle_EnterChargeState()
{
	drive_ServoCtrlReq(DRIVE_SERVO_CTRL_OFF);
	VehicleCtrl.VSm = VEHICLE_SM_CHARGE;
}

/*
 * Description:
 * Perform vehicle state machine
 * Params:
 *	N/A
 * Note:
 * put this function in a periodical running timer loop
 */

void Vehicle_StateMachineCtrl()
{

	switch (VehicleCtrl.VSm)
	{
		case VEHICLE_SM_NOT_READY: // regard as VSM Startup state
			drive_GlobalAlarmEnableReq(DISABLE);
			drive_ServoCtrlReq(DRIVE_SERVO_CTRL_OFF);
			Vehicle_EnterStandByState();
			break;

		case VEHICLE_SM_STANDBY:

			if((Bat_MainSMGet(BAT_IDX_MASTER) == BAT_MAIN_SM_ERROR) ||
			   //(DIO_KillSwStateGet() == DIO_KILL_SWITCH_STATE_OPEN) ||
			   (Btn_StateRead(BTN_IDX_KILL_SW) == BTN_KILL_SW_PRESS) ||
			   (drive_GetStatus(0, DN_INV_MAIN_STATE) == INV_OP_ALARM))
			{
				Vehicle_EnterErrordState();
			}
			else if((Bat_MainSMGet(BAT_IDX_MASTER) == BAT_MAIN_SM_ENERGIZE) &&
					(Bat_PrchSMGet(BAT_IDX_MASTER) == BAT_ST_BATT_PRCH_SUCCESSFUL) &&
					(float)drive_GetStatus(0, DN_BUS_VOLTAGE) >= (40.0f * 10.0f) )
			{
				Vehicle_EnterEnergizedState();
			}
			else if(Bat_MainSMGet(BAT_IDX_MASTER) == BAT_MAIN_SM_CHARGE)
			{
				Vehicle_EnterChargeState();
			}
			else if(Bat_ShutdownReqFlgGet(BAT_IDX_MASTER) == BAT_FLG_ON)
			{
				Vehicle_EnterPreShutdownState();
			}

			break;

		case VEHICLE_SM_ENERGIZED:

			if((Bat_MainSMGet(BAT_IDX_MASTER) == BAT_MAIN_SM_ERROR) ||
			   //(DIO_KillSwStateGet() == DIO_KILL_SWITCH_STATE_OPEN) ||
    		   (Btn_StateRead(BTN_IDX_KILL_SW) == BTN_KILL_SW_PRESS)||
			   (drive_GetStatus(0, DN_INV_MAIN_STATE) == INV_OP_ALARM))
			{
				Vehicle_EnterErrordState();
			}
			else if(Bat_MainSMGet(BAT_IDX_MASTER) == BAT_MAIN_SM_CHARGE)
			{
				Vehicle_EnterChargeState();
			}
			else if(Bat_ShutdownReqFlgGet(BAT_IDX_MASTER) == BAT_FLG_ON)
			{
				Vehicle_EnterPreShutdownState();
			}
			else
			{
				if(VehicleCtrl.DualBtnReleaseFlag == VEHICLE_BTN_RELEASE_FLAG_ALL)
				{
					if ((Btn_StateRead(BTN_IDX_BST_BTN)== BTN_BOOST_PRESS) &&
						(Btn_StateRead(BTN_IDX_REV_BTN)== BTN_REVERSE_PRESS) &&
						(drive_GetStatus(0, DN_THROT_MAPPING_RST) <= VEHICLE_THROTTLE_RELEASE_THRESHOLD))
					{
						VehicleCtrl.DualBtnTimeCnt ++;

						if(VehicleCtrl.DualBtnTimeCnt > VEHICLE_DRIVE_ENABLE_TIME_CNT)
						{
						  Vehicle_EnterTqGeneratorState();
						}
					}
					else
					{
						VehicleCtrl.DualBtnTimeCnt = 0;
					}
				}
				else
				{
					if(Btn_StateRead(BTN_IDX_BST_BTN)== BTN_BOOST_RELEASE)
					{
						VehicleCtrl.DualBtnReleaseFlag |= VEHICLE_BTN_RELEASE_FLAG_BOOST;
					}

					if(Btn_StateRead(BTN_IDX_REV_BTN)== BTN_REVERSE_RELEASE)
					{
						VehicleCtrl.DualBtnReleaseFlag |= VEHICLE_BTN_RELEASE_FLAG_REVERSE;
					}
				}
			}
			break;

		case VEHICLE_SM_TQCTRL:

			if((Bat_MainSMGet(BAT_IDX_MASTER) == BAT_MAIN_SM_ERROR) ||
  			   (Btn_StateRead(BTN_IDX_KILL_SW) == BTN_KILL_SW_PRESS) ||
				//(DIO_KillSwStateGet() == DIO_KILL_SWITCH_STATE_OPEN) ||
			   (drive_GetStatus(0, DN_INV_MAIN_STATE) == INV_OP_ALARM))
			{
				Vehicle_EnterErrordState();
			}
			else if(Bat_MainSMGet(BAT_IDX_MASTER) == BAT_MAIN_SM_CHARGE)
			{
				Vehicle_EnterChargeState();
			}
			else if(Bat_ShutdownReqFlgGet(BAT_IDX_MASTER) == BAT_FLG_ON)
			{
				Vehicle_EnterPreShutdownState();
			}
			else
			{
				if(VehicleCtrl.DualBtnReleaseFlag == VEHICLE_BTN_RELEASE_FLAG_ALL)
				{
					if ((Btn_StateRead(BTN_IDX_BST_BTN)== BTN_BOOST_PRESS) &&
						(Btn_StateRead(BTN_IDX_REV_BTN)== BTN_REVERSE_PRESS) &&
						(drive_GetStatus(0,DN_THROT_MAPPING_RST) <= VEHICLE_THROTTLE_RELEASE_THRESHOLD))
					{
						VehicleCtrl.DualBtnTimeCnt ++;

						if(VehicleCtrl.DualBtnTimeCnt > VEHICLE_DRIVE_ENABLE_TIME_CNT)
						{
						  Vehicle_EnterEnergizedState();
						}
					}
					else
					{
						VehicleCtrl.DualBtnTimeCnt = 0;
					}
				}
				else
				{
					if(Btn_StateRead(BTN_IDX_BST_BTN) == BTN_BOOST_RELEASE)
					{
						VehicleCtrl.DualBtnReleaseFlag |= VEHICLE_BTN_RELEASE_FLAG_BOOST;
					}

					if(Btn_StateRead(BTN_IDX_REV_BTN)== BTN_REVERSE_RELEASE)
					{
						VehicleCtrl.DualBtnReleaseFlag |= VEHICLE_BTN_RELEASE_FLAG_REVERSE;
					}

					if(VehicleCtrl.DualBtnReleaseFlag == VEHICLE_BTN_RELEASE_FLAG_ALL)
					{
						drive_ServoCtrlReq(DRIVE_SERVO_CTRL_ON);
					}
				}
			}
			break;

		case VEHICLE_SM_ERROR:
			break;

		case VEHICLE_SM_PRE_SHUTDOWN:
			if((Bat_MainSMGet(BAT_IDX_MASTER) == BAT_MAIN_SM_ERROR) ||
			   (Btn_StateRead(BTN_IDX_KILL_SW) == BTN_KILL_SW_PRESS) ||
					//(DIO_KillSwStateGet() == DIO_KILL_SWITCH_STATE_OPEN) ||
			   (drive_GetStatus(0, DN_INV_MAIN_STATE) == INV_OP_ALARM))
			{
				Vehicle_EnterErrordState();
			}
			else
			{
				if(drive_GetStatus(0, DN_MOTOR_SPEED) < VEHICLE_MOTOR_STEADY_SPD_RPM_THRESHOLD)
				{
					Vehicle_EnterShutdownState();
				}
			}
			break;

		case VEHICLE_SM_SHUTDOWN:
			break;

		case VEHICLE_SM_CHARGE:
			if((Bat_MainSMGet(BAT_IDX_MASTER) == BAT_MAIN_SM_ERROR) ||
			   (DIO_KillSwStateGet() == DIO_KILL_SWITCH_STATE_OPEN) ||
			   (drive_GetStatus(0, DN_INV_MAIN_STATE) == INV_OP_ALARM))
			{
				Vehicle_EnterErrordState();
			}
			else if((Bat_MainSMGet(BAT_IDX_MASTER) == BAT_MAIN_SM_STANDBY) &&
					(float)drive_GetStatus(0, DN_BUS_VOLTAGE) < (40.0f * 10.0f) )
			{
				Vehicle_EnterStandByState();
			}
			break;

		default:
			Vehicle_EnterErrordState();
			break;
	}
}


/*
 * Public functions
 */

void Vehicle_Do100HzLoop(void)
{
	Vehicle_StateMachineCtrl();
}

Vehicle_Sm_e Vehicle_MainSMGet(void)
{
	return VehicleCtrl.VSm;
}
