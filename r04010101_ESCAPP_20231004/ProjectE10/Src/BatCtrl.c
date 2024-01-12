#include <BatCtrl.h>
#include "stdint.h"
#include "string.h"
#include "stdlib.h"
#include "Protocol.h"

#define ABS(x) ((x) > 0 ? (x) : -(x))

BatStation_t BatStation = BAT_STATION_DEFAULT;
BatCtrl_t BatCtrl = {0};
BatInfo_t BatInfo[BAT_IDX_ALL] ={CAN_INFORM_FORM_BAT_DEFAULT, CAN_INFORM_FORM_BAT_DEFAULT};



uint8_t Bat_BroadcastReq(uint8_t BatNum, BatBdCtrl_t Cmd)
{
    /* request both */
    STRUCT_CAN_DATA CANDataTemp;
    CanIdField_u CanID;
    uint8_t lStatus = QUEUE_OK;

    CanID.Bits.Device = CAN_DEVICE_TO_BAT;
    CanID.Bits.Tod = CAN_TOD_BROADCAST_CTRL;
    CanID.Bits.Priority = 0x10;
    CanID.Bits.Reserved = 0;

    for(uint8_t i=0; i < BatNum; i++)
    {
       CanID.Bits.Instance = i + 1;	/* index start from 1 */
       CANDataTemp.ID.ExtFrame.ExtID = CanID.All;
       CANDataTemp.ID.ExtFrame.XTD = 1;
       CANDataTemp.Size = 1;
       CANDataTemp.Data[0] = Cmd ;
       lStatus = BatCtrl.pExCANStation->TxQ.EnQ(&BatCtrl.pExCANStation->TxQ, &CANDataTemp);
    }

    return lStatus;
}

uint8_t Bat_PowerCtrlReq( uint8_t BatNum, BatPwrCtrl_t Cmd)
{
    /* request both */
    STRUCT_CAN_DATA CANDataTemp;
    CanIdField_u CanID;
    uint8_t lStatus = QUEUE_OK;

    CanID.Bits.Device = CAN_DEVICE_TO_BAT;
    CanID.Bits.Tod = CAN_TOD_POWER_CTRL;
    CanID.Bits.Priority = 0x10;
    CanID.Bits.Reserved = 0;

    for(uint8_t i=0; i < BatNum; i++)
    {
       CanID.Bits.Instance = i + 1;	/* index start from 1 */
       CANDataTemp.ID.ExtFrame.ExtID = CanID.All;
       CANDataTemp.ID.ExtFrame.XTD = 1;
       CANDataTemp.Size = 1;
       CANDataTemp.Data[0] = Cmd ;
       lStatus = BatCtrl.pExCANStation->TxQ.EnQ(&BatCtrl.pExCANStation->TxQ, &CANDataTemp);
    }

    return lStatus;
}


uint8_t Bat_StayAliveReq()
{
    /* request both */
    STRUCT_CAN_DATA CANDataTemp;
    CanIdField_u CanID;
    uint8_t lStatus = QUEUE_OK;

    CanID.Bits.Device = CAN_DEVICE_TO_BAT;
    CanID.Bits.Tod = CAN_TOD_STAY_ALIVE;
    CanID.Bits.Priority = 0x10;
    CanID.Bits.Reserved = 0;
    CanID.Bits.Instance = 0;
    CANDataTemp.ID.ExtFrame.ExtID = CanID.All;
    CANDataTemp.ID.ExtFrame.XTD = 1;
    CANDataTemp.Size = 0;
    lStatus = BatCtrl.pExCANStation->TxQ.EnQ(&BatCtrl.pExCANStation->TxQ, &CANDataTemp);
    return lStatus;
}



void Bat_CanMsgLoad(uint32_t Id, uint8_t *pData)
{
	CanIdField_t CanId;
	BatCanMsgs_u *BatCanMsg;

	BatCanMsg = (BatCanMsgs_u*)pData;
	memcpy((uint8_t*)&CanId, &Id, 4);

	if ((CanId.Device == CAN_DEVICE_FROM_BAT) &&
		(CanId.Instance >= BAT_INSTANCE_MAIN && CanId.Instance <= BAT_INSTANCE_SEC) &&
	    (CanId.Priority == 0x08))
	{
		switch (CanId.Tod)
		{
			case CAN_TOD_MEASUREMENT:

				BatInfo[CanId.Instance - 1].DCVolt = (float)BatCanMsg->Tod0x20.DCVolt * 0.01; 	/* The instance starts from 1 to 2 */
				BatInfo[CanId.Instance - 1].DcCurrent = (float)BatCanMsg->Tod0x20.DCCurrent;
				BatInfo[CanId.Instance - 1].Soc = (uint8_t)(BatCanMsg->Tod0x20.Soc & 0x00FF);
				BatInfo[CanId.Instance - 1].CapRemain = (float)BatCanMsg->Tod0x20.CapRemain * 0.01;
				BatInfo[CanId.Instance - 1].TodsRcved |= BMS_TOD_RXED_MSK_0X20;
				break;
			case CAN_TOD_STATUS:
				BatInfo[CanId.Instance - 1].FetStatus.All = BatCanMsg->Tod0x21.FetStatus.All;
				BatInfo[CanId.Instance - 1].Soh = BatCanMsg->Tod0x21.Soh;
				BatInfo[CanId.Instance - 1].ErrorFlags = BatCanMsg->Tod0x21.ErrorFlags;
				BatInfo[CanId.Instance - 1].TodsRcved |= BMS_TOD_RXED_MSK_0X21;
				break;

			case CAN_TOD_BAT_CELL_VOLT01:
			case CAN_TOD_BAT_CELL_VOLT02:
			case CAN_TOD_BAT_CELL_VOLT03:
			case CAN_TOD_BAT_CELL_VOLT04:
			/* do nothing to cell volt for P0 */
				break;

			default :
				break;
		}
	}
}

void Bat_CanTxCmDHandle()
{
	if(BatCtrl.MainSm > BAT_MAIN_SM_IDLE)
	{
		if( BatCtrl.TimerPrescaler++ >= 9)
		{
			Bat_PowerCtrlReq(2, BatCtrl.PwrCtrlCmd);
			Bat_BroadcastReq(2, BatCtrl.BdCtrlCmd);
			Bat_StayAliveReq();
			BatCtrl.TimerPrescaler = 0;
		}
	}
	else
	{
		BatCtrl.TimerPrescaler = 0;
	}
}

void Bat_InvDcVoltSet(float VdcIn)
{
	BatCtrl.DcBusVoldNow = VdcIn;
}

void Bat_RcvedTodClear(void)
{
	for(uint8_t i =0; i<2; i++)
	{
		BatInfo[i].TodsRcved = 0;
		BatInfo[i].DCVolt = 0;
		BatInfo[i].ErrorFlags = 0;
		BatInfo[i].FetStatus.All = 0;
	}
}

void Bat_PwrOnReq(void)
{
	switch (BatCtrl.MainSm)
	{
		case BAT_MAIN_SM_IDLE:
		case BAT_MAIN_PWR_OFF:
			BatCtrl.PwrOffSM = BAT_PWR_OFF_SM_IDLE;
			BatCtrl.PwrOnSM = BAT_PWR_ON_SM_START;
			BatCtrl.MainSm = BAT_MAIN_PWR_ON;
			break;

		case BAT_MAIN_PWR_ON:		/*power on sequence is processing, do nothing*/
		case BAT_MAIN_SM_INIT:		/*not a valid entrance of power on sequence, do nothing*/
		case BAT_MAIN_ALARM:		/*not a valid entrance of power on sequence, do nothing*/
		case BAT_MAIN_ACTIVATED:	/*not a valid entrance of power on sequence, do nothing*/
		default:
			break;
	}
}

void Bat_PwrOffReq(void)
{
	switch (BatCtrl.MainSm)
	{
		case BAT_MAIN_SM_IDLE:
		case BAT_MAIN_ALARM:
		case BAT_MAIN_PWR_ON:
		case BAT_MAIN_ACTIVATED:
			BatCtrl.PwrOffSM = BAT_PWR_OFF_SM_START;
			BatCtrl.PwrOnSM = BAT_PWR_ON_SM_IDLE;
			BatCtrl.MainSm = BAT_MAIN_PWR_OFF;
			break;

		case BAT_MAIN_PWR_OFF:		/* power off sequence is processing, do nothing */
		case BAT_MAIN_SM_INIT:		/*not a valid entrance of power on sequence, do nothing*/
		default:
			break;
	}
}

void Bat_PwrOnCtrl(void)
{
	uint8_t i = 0;
	BatPwrOnErr_t statusTemp = BAT_PWR_ON_ERR_UNDEFINED_STATE;

  switch (BatCtrl.PwrOnSM)
  {
    case BAT_PWR_ON_SM_IDLE:
    /* DO nothing */
      break;

    case BAT_PWR_ON_SM_START:
		Bat_RcvedTodClear();
		BatCtrl.PwrOnSM = BAT_PWR_ON_SM_WAIT_FOR_VOLT;
		BatCtrl.PwrOnToCnt = 0;
		BatCtrl.PwrOnErr = BAT_PWR_ON_ERR_OK;
		BatCtrl.PwrCtrlCmd = BAT_PWR_CTRL_OFF;
		BatCtrl.BdCtrlCmd = BAT_BD_CTRL_ON;
		break;

    case BAT_PWR_ON_SM_WAIT_FOR_VOLT:
      /* Send broadcast ctrl on command */
    	BatCtrl.BdCtrlCmd = BAT_BD_CTRL_ON;

      /* Check if CAN MSGs from both battery are received */
		for (i = 0; i< BAT_IDX_ALL; i++)
		{
			if(BatInfo[i].ErrorFlags != 0)
			{
				statusTemp = BAT_PWR_ON_ERR_BMS_ERROR_REPORTED;
				break;
			}

			if(BatInfo[i].TodsRcved != BMS_TOD_RXED_MSK_ALL)
			{
				statusTemp = BAT_PWR_ON_ERR_BAT_INFO_NOT_ENOUGH;
				break;
			}

			/* check if voltage difference of both battery and VDC is acceptable
			* if acceptable, transfer to next state */
			if((BatCtrl.DcBusVoldNow > BAT_INV_DCBUS_VOLT_THRESHOLD_V) && (ABS(BatInfo[i].DCVolt -BatCtrl.DcBusVoldNow) < BAT_VDIFF_BAT_INV_THRESHOLD_V))
			{
				statusTemp = BAT_PWR_ON_ERR_OK;
			}
			else
			{
				statusTemp = (BatCtrl.DcBusVoldNow > BAT_INV_DCBUS_VOLT_THRESHOLD_V) ? BAT_PWR_ON_ERR_VDIFF_TOO_HIGH : BAT_PWR_ON_ERR_DCV_TOO_LOW;
				break;
			}
		}

		if(statusTemp != BAT_PWR_ON_ERR_OK)
		{
			if((BatCtrl.PwrOnToCnt++ > BAT_WAIT_FOR_VOLT_TIME_THRESHOLD_MS) ||
			   (statusTemp == BAT_PWR_ON_ERR_BMS_ERROR_REPORTED))
			{
				BatCtrl.PwrOnSM = BAT_PWR_ON_SM_FAIL;
			}
		}
		else
		{
			BatCtrl.PwrOnSM = BAT_PWR_ON_SM_WAIT_FOR_MOSFET_STATUS;
			BatCtrl.PwrOnToCnt = 0;
		}

		BatCtrl.PwrOnErr = statusTemp;

      break;

    case BAT_PWR_ON_SM_WAIT_FOR_MOSFET_STATUS:
      /* Send power on command */
    	BatCtrl.PwrCtrlCmd = BAT_PWR_CTRL_ON;
      /* check if can msg from both battery are received */
      /* check if both battery turn on the dicharge MosFet*/
      /* if so, change state to complete*/
    	for (i = 0; i< BAT_IDX_ALL; i++)
		{
			if(BatInfo[i].ErrorFlags != 0)
			{
				statusTemp = BAT_PWR_ON_ERR_BMS_ERROR_REPORTED;
				break;
			}

    		if(BatInfo[i].TodsRcved != BMS_TOD_RXED_MSK_ALL)
			{
    			statusTemp = BAT_PWR_ON_ERR_BAT_INFO_NOT_ENOUGH;
				break;
			}

			/* Check if all MOSFET of both batteries are turned on */
			if((BatInfo[i].FetStatus.All & 0x0F) != 0x03)
			{
				statusTemp = BAT_PWR_ON_ERR_MOSFET_CTRL_FAIL;
				break;
			}
			else
			{
				statusTemp = BAT_PWR_ON_ERR_OK;
			}
		}

		if(statusTemp != BAT_PWR_ON_ERR_OK)
		{
			if((BatCtrl.PwrOnToCnt++ > BAT_WAIT_FOR_MOSFET_TIME_THRESHOLD_MS) ||
			   (statusTemp == BAT_PWR_ON_ERR_BMS_ERROR_REPORTED))
			{
				BatCtrl.PwrOnSM = BAT_PWR_ON_SM_FAIL;
			}
		}
		else
		{
			/* MOSFET is on, switch to next state */
			BatCtrl.PwrOnSM = BAT_PWR_ON_SM_COMPLETE;
			BatCtrl.PwrOnToCnt = 0;
		}

		BatCtrl.PwrOnErr = statusTemp;

		break;

    case BAT_PWR_ON_SM_COMPLETE:
    	/*do nothing */
    	break;

    case BAT_PWR_ON_SM_FAIL:
    	/*todo :report alarm*/

    	break;

    default:
    	BatCtrl.PwrOnSM = BAT_PWR_ON_SM_FAIL;
    	statusTemp = BAT_PWR_ON_ERR_UNDEFINED_STATE;
		BatCtrl.PwrOnErr = statusTemp;
    	break;
  }

}


void Bat_PwrOffCtrl(void)
{
	uint8_t i = 0;
	BatPwrOffErr_t statusTemp = BAT_PWR_OFF_ERR_UNDEFINED_STATE;

	switch(BatCtrl.PwrOffSM)
	{
		case BAT_PWR_OFF_SM_IDLE:
			break;

		case BAT_PWR_OFF_SM_START:
			Bat_RcvedTodClear();
			BatCtrl.PwrOffErr = BAT_PWR_OFF_ERR_OK;
			BatCtrl.PwrOffToCnt = 0;
			BatCtrl.PwrOffSM = BAT_PWR_OFF_SM_WAIT_FOR_MOSFET_OFF;
			break;

		case BAT_PWR_OFF_SM_WAIT_FOR_MOSFET_OFF:
			BatCtrl.PwrCtrlCmd = BAT_PWR_CTRL_OFF;

			for (i = 0; i< BAT_IDX_ALL; i++)
			{

				if((BatInfo[i].TodsRcved & BMS_TOD_RXED_MSK_0X21) == 0)
				{
					statusTemp = BAT_PWR_OFF_ERR_BAT_INFO_NOT_ENOUGH;
					break;
				}

				/* Check if all MOSFET of both batteries are turned off */
				if((BatInfo[i].FetStatus.All & 0x03) == 0)
				{
					statusTemp = BAT_PWR_OFF_ERR_OK;
				}
				else
				{
					statusTemp = BAT_PWR_OFF_ERR_MOSFET_CTRL_FAIL;
					break;

				}
			}

			if(statusTemp != BAT_PWR_OFF_ERR_OK)
			{
				if(BatCtrl.PwrOffToCnt ++ > BAT_WAIT_FOR_MOSFET_TIME_THRESHOLD_MS)
				{
					BatCtrl.PwrOffSM = BAT_PWR_OFF_SM_FAIL;
				}
			}
			else
			{
				/* MOSFET are all off, switch to next state */
				BatCtrl.PwrOffSM = BAT_PWR_OFF_SM_STOP_BROADCAST;
				BatCtrl.PwrOffToCnt = 0;
				/* stop broadcasting */

			}

			BatCtrl.PwrOffErr = statusTemp;

			break;

		case BAT_PWR_OFF_SM_STOP_BROADCAST:
			BatCtrl.BdCtrlCmd = BAT_BD_CTRL_OFF;

			if(BatCtrl.PwrOffToCnt++ > BAT_WAIT_FOR_STOP_BROADCAST_TIME_THRESHOLD_MS)
			{
				BatCtrl.PwrOffToCnt = 0;
				BatCtrl.PwrOffSM = BAT_PWR_OFF_SM_COMPLETE;
			}

			break;

		case BAT_PWR_OFF_SM_COMPLETE:
	    	/* Do nothing */
			break;

		case BAT_PWR_OFF_SM_FAIL:
			/* Todo: report error */
			break;

		default:
			break;
	}

}


void Bat_SMCtrl(void)
{
	switch (BatCtrl.MainSm)
	{
  	  case BAT_MAIN_SM_INIT:
  		BatCtrl.PwrOnToCnt = 0;
  		BatCtrl.MainSm = BAT_MAIN_SM_INIT;
  		BatCtrl.PwrOnSM = BAT_PWR_ON_SM_IDLE;
  		BatCtrl.PwrOnErr = BAT_PWR_ON_ERR_OK;
  		BatCtrl.PwrOffSM = BAT_PWR_OFF_SM_IDLE;
  		BatCtrl.PwrOffErr = BAT_PWR_OFF_ERR_OK;
  		BatCtrl.TimerPrescaler = 0;
  		BatCtrl.MainSm = BAT_MAIN_SM_IDLE;
  		break;

  	  case BAT_MAIN_SM_IDLE:
  		  /* do nothing*/
  		  break;

		case BAT_MAIN_PWR_ON:

			Bat_PwrOnCtrl();

			if(BatCtrl.PwrOnSM == BAT_PWR_ON_SM_COMPLETE)
			{
				BatCtrl.MainSm = BAT_MAIN_ACTIVATED;
			}
			else if(BatCtrl.PwrOnSM == BAT_PWR_ON_SM_FAIL)
			{
				BatCtrl.MainSm = BAT_MAIN_ALARM;
			}
		  break;

		case BAT_MAIN_ACTIVATED:
			/* Vehicle is able to drain current form batteries under this state*/
			/* todo: keep monitoring error flags reported from both BMS */
			for(uint8_t i=0; i<BAT_IDX_ALL; i++)
			{
				if(BatInfo[i].ErrorFlags != 0)
				{
					BatCtrl.MainSm = BAT_MAIN_ALARM;
					break;
				}
			}
		  break;

		case BAT_MAIN_PWR_OFF:

			Bat_PwrOffCtrl();

			if(BatCtrl.PwrOffSM == BAT_PWR_OFF_SM_COMPLETE)
			{
				BatCtrl.MainSm = BAT_MAIN_SM_IDLE;
			}
			else if(BatCtrl.PwrOffSM == BAT_PWR_OFF_SM_FAIL)
			{
				BatCtrl.MainSm = BAT_MAIN_ALARM;
				/*report alarm here*/
			}
		  break;

		case BAT_MAIN_ALARM:
			/*todo: report error */

		  break;

		default:
		  break;
	}
}


void Bat_Do100HzLoop (void)
{
	Bat_SMCtrl();
	Bat_CanTxCmDHandle();
}

BatMainSM_t Bat_MainSMGet (void)
{
	return BatCtrl.MainSm;
}

BatPwrOnSM_t Bat_PwrOnSMGet (void)
{
	return BatCtrl.PwrOnSM;
}

BatPwrOffSM_t Bat_PwrOffSMGet (void)
{
	return BatCtrl.PwrOffSM;
}

void Bat_CanHandleLoad(ExtranetCANStation_t *pIn)
{
	BatCtrl.pExCANStation = pIn;
}
