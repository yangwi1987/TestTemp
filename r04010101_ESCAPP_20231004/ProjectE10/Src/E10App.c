/*
 * BMEApp.c
 *
 *  Created on: 20230714
 *      Author: Will
 */


#include "E10App.h"

AcPwrInfo_t AcPwrInfo = AC_POWER_INFO_DEFAULT;

void AcPowerInfo_Reset(AcPwrInfo_t *p)
{
	p->AvgPower = 0;
	p->InstPower = 0;
	p->InstPwrSum = 0;
	p->TotalPower = 0;
	p->LogCnt = 0;
	p->TotalLogCnt = 0;
}

void AcPowerInfo_do100HzLoop(AcPwrInfo_t *p, float PowerIn)
{
	p->InstPwrSum += PowerIn;
	p->LogCnt++;

	if(p->LogCnt >= INSTANT_POWER_LOG_SIZE)
	{
		p->InstPower = p->InstPwrSum * INSTANT_POWER_LOG_SIZE_INVERSE;			/* get "averaged" instantaneous power */
		p->TotalPower += p->InstPower;											/* J= sum ( P * time interval) */
		p->TotalLogCnt++;

		if(p->TotalLogCnt > 0)
		{
			p->AvgPower = p->TotalPower / (float)p->TotalLogCnt;				/* get the average power */											/* Load value */
		}
		p->LogCnt = 0;															/* clear log counter */
		p->InstPwrSum = 0;														/* clear the accumlate value */
	}
}

/*=============== Button handle start ===============*/
static BtnStateFb_t BtnTable[BTN_IDX_MAX];

void Btn_Reset (BtnIdx_e Idx)
{
    BtnTable[Idx].Before = BTN_STATE_INITIALIZING;
    BtnTable[Idx].Now = 0;
    BtnTable[Idx].SignalTimeCnt = 0;
    BtnTable[Idx].EvtRecord = BTN_EVENT_INITIALIZING;
}

/*
 * Put this handling function in 100Hz timer loop
 * */
void Btn_Handle (BtnIdx_e Idx)
{
    int8_t i8Temp;

    if (BtnTable[Idx].EvtRecord == BTN_EVENT_INITIALIZING)
    {
        if(BtnTable[Idx].SignalTimeCnt++ > BTN_INITIAL_TIME_COUNT)
        {
            BtnTable[Idx].SignalTimeCnt = 0;
            BtnTable[Idx].Before = BtnTable[Idx].Now;
            BtnTable[Idx].EvtRecord = BTN_EVENT_IDLE;
        }
        else
        {
            BtnTable[Idx].Before = BTN_STATE_INITIALIZING;
            BtnTable[Idx].EvtRecord = BTN_EVENT_INITIALIZING;
        }
    }
    else
    {
        i8Temp = BtnTable[Idx].Now - BtnTable[Idx].Before;

        if(i8Temp != 0) /*signal change is detected */
        {
            BtnTable[Idx].SignalTimeCnt++;

            if (BtnTable[Idx].SignalTimeCnt > BTN_DEBOUNCE_TIME_COUNT)
            {
                BtnTable[Idx].Before = BtnTable[Idx].Now;
                BtnTable[Idx].EvtRecord = (i8Temp > 0) ? BTN_EVENT_RISING : BTN_EVENT_FALLING;
                BtnTable[Idx].SignalTimeCnt = 0;
                BtnTable[Idx].EvtTimeCnt = 0;
            }
        }
        else
        {
            BtnTable[Idx].SignalTimeCnt = 0;
            
            if ((BtnTable[Idx].EvtRecord == BTN_EVENT_RISING) ||
            	(BtnTable[Idx].EvtRecord == BTN_EVENT_FALLING))
            {
                BtnTable[Idx].EvtTimeCnt ++;

                if(BtnTable[Idx].EvtTimeCnt > BTN_RECORDED_EVENT_AUTOCLEAR_TIME_THRESHOLD)
                {
                    BtnTable[Idx].EvtTimeCnt = 0;
                    BtnTable[Idx].EvtRecord = BTN_EVENT_IDLE;
                }
            }
        }
    }
}

/*
 * Read the latest event detected by IO handler
 */
BtnEvent_e Btn_EvtRead (BtnIdx_e Idx)
{
    BtnEvent_e ret;
    ret = BtnTable[Idx].EvtRecord;
    BtnTable[Idx].EvtRecord = (BtnTable[Idx].EvtRecord ==BTN_EVENT_INITIALIZING) ? BTN_EVENT_INITIALIZING : BTN_EVENT_IDLE;
    return ret;
}

/*
 * Read the handled state of signal
 */
BtnState_e Btn_StateRead (BtnIdx_e Idx)
{
    return BtnTable[Idx].Before;
}

/*
 * write the Digital value detected by MCU into IO handler anywhere
 */
void Btn_SignalWrite (BtnIdx_e Idx, int8_t DataIn)
{
    BtnTable[Idx].Now = DataIn;
}

/*
 * Btn period loop
 */
void Btn_Do100HzLoop (void)
{
    for(uint8_t i = 0 ; i < BTN_IDX_MAX; i++)
    {
        Btn_Handle(i);
    }
}

void Btn_Init()
{
    for(uint8_t i = 0 ; i < BTN_IDX_MAX; i++)
    {
        Btn_Reset(i);
    }
}

/*=============== Button handle End ===============*/

/*=============== DO Indication control start ===============*/
DOCtrl_t DOCtrlArray[DO_IDX_MAX];
SocLightCtrl_t SocLightCtrl;


void DO_CtrlReq(DOIdx_e Idx, DOMode_e ModeIn, uint16_t NbrToBlink, DOBlinkConfig_t BlinkConfig)
{
  DOCtrlArray[Idx].Mode = ModeIn;
  DOCtrlArray[Idx].TimeCnt = 0;
  DOCtrlArray[Idx].NbrCnt = 0;
  DOCtrlArray[Idx].NbrToBlink = NbrToBlink;
  DOCtrlArray[Idx].BlinkConfig = BlinkConfig;
}

void DO_TurnOnReq(DOIdx_e Idx)
{
	DO_CtrlReq(Idx, DO_MODE_ON, 0, DO_BLINK_CONFIG_STEADY);
}

void DO_TurnOffReq(DOIdx_e Idx)
{
	DO_CtrlReq(Idx, DO_MODE_OFF, 0, DO_BLINK_CONFIG_STEADY);
}



void DO_Do100HzLoop(void)
{

  for (uint8_t i = 0; i < DO_IDX_MAX; i++)
  {
    switch (DOCtrlArray[i].Mode)
    {
      case DO_MODE_OFF:
      default:
        DOCtrlArray[i].Cmd = DO_CMD_OFF;
        break;
      
      case DO_MODE_ON:
        DOCtrlArray[i].Cmd = DO_CMD_ON;
        break;

      case DO_MODE_BLINK:

        if(DOCtrlArray[i].TimeCnt < DOCtrlArray[i].BlinkConfig.OnTime)
        {
          /* Turn on DO*/
          DOCtrlArray[i].Cmd = DO_CMD_ON;
        }
        else if(DOCtrlArray[i].TimeCnt < DOCtrlArray[i].BlinkConfig.Period)
        {
          /* Turn off DO*/
          DOCtrlArray[i].Cmd = DO_CMD_OFF;
        }
        else
        {
          DOCtrlArray[i].TimeCnt = 0;

          /* increase the blink conter if nbr to blink is greater than "DO_NBR_TO_BLINK_FOREVER" */
          if(DOCtrlArray[i].NbrToBlink > DO_NBR_TO_BLINK_FOREVER)
          {
        	  DOCtrlArray[i].NbrCnt++;

        	  if(DOCtrlArray[i].NbrCnt >= DOCtrlArray[i].NbrToBlink)
        	  {
        		  DOCtrlArray[i].Mode = DO_MODE_OFF;
        	  }
          }
        }
        
        DOCtrlArray[i].TimeCnt++;

        break;
    }
  }
}

DOCmd_e DO_CmdGet(DOIdx_e Idx)
{
  return DOCtrlArray[Idx].Cmd;
}

/**/
void SocLightModeSet( SocDisplayMode_e ModeIn)
{
  SocLightCtrl.DisplayMode = ModeIn;
  SocLightCtrl.ReqFlag  = 1;
  /* Force to clear the DO indication first */
  for (uint8_t i = 0; i < 4 ; i++)
  {
    for (uint8_t j = 0; j < 3; j++)
    {
	  DO_TurnOffReq((DO_IDX_SOC1_R + j) + (i * 3));
    }
  }
}

void SocCtrlSet(DORGBIdx_e ColorIn, DOMode_e ModeIn, DOBlinkConfig_t DOConfig)
{
  SocLightCtrl.ColorReq = ColorIn;
  SocLightCtrl.DOMode = ModeIn;
  SocLightCtrl.BlinkConfig = DOConfig;
  SocLightCtrl.ReqFlag  = 1;
}
/*
 *
 * 	Note	To write/update the current SOC value when required
	Input	SocIn : the SOC value to write, should be 0~100%
	Output	NA
 *
 * */
void SocValueSet( uint8_t SocIn)
{

  if(SocLightCtrl.Soc != SocIn)
  {
	  SocLightCtrl.ReqFlag  = 1;
  }

  SocLightCtrl.Soc = SocIn;
}

void SocIndicationByBLinking(DOIdx_e DOIdxIn)
{
  if (SocLightCtrl.Soc >= 75)
  {
    DO_CtrlReq( DOIdxIn, DO_MODE_ON, 0, DO_BLINK_CONFIG_STEADY);
  }
  else if ( SocLightCtrl.Soc >= 50 )
  {
    DO_CtrlReq( DOIdxIn, DO_MODE_BLINK, DO_NBR_TO_BLINK_FOREVER, DO_BLINK_CONFIG_1HZ);
  }
  else if ( SocLightCtrl.Soc >= 25 )
  {
    DO_CtrlReq( DOIdxIn, DO_MODE_BLINK, DO_NBR_TO_BLINK_FOREVER, DO_BLINK_CONFIG_2HZ);
  }
  else if ( SocLightCtrl.Soc >= 5 )
  {
    DO_CtrlReq( DOIdxIn, DO_MODE_BLINK, DO_NBR_TO_BLINK_FOREVER, DO_BLINK_CONFIG_4HZ);
  }
  else 
  {
    DO_CtrlReq( DOIdxIn, DO_MODE_OFF, 0, DO_BLINK_CONFIG_STEADY);
  }
}

void SocLightCtrl_Do100HzLoop(void)
{
  uint8_t ScaledSoc;

  if(SocLightCtrl.ReqFlag == 1)
  {
    if(SocLightCtrl.DisplayMode == SOC_DISPLAY_MODE_SOC_LED_ARRAY)
    {
      /* decide the # of DO to turn on */
      ScaledSoc = ( SocLightCtrl.Soc ) / 25;

      if(SocLightCtrl.Soc > 0)
      {
        ScaledSoc += 1;
      }

      /* Light the DO according to number, blink pattern and color selected by user */
      for(uint8_t i = 0; i < 4 ; i++)
      {
        for(uint8_t j = 0; j < 3; j++)
        {
        	DO_TurnOffReq((DO_IDX_SOC1_R + j) + (i * 3));
        }

        if( i < ScaledSoc )
        {
        	DO_CtrlReq((DO_IDX_SOC1_R + SocLightCtrl.ColorReq + i * 3), SocLightCtrl.DOMode, 0, SocLightCtrl.BlinkConfig);
        }
      }
    }
    else if(SocLightCtrl.DisplayMode == SOC_DISPLAY_MODE_SOC_SINGLE_LED)
    {
      SocIndicationByBLinking(DO_IDX_REAR);
    }

    SocLightCtrl.ReqFlag = 0;
  }
}




/*=============== DO Indication control End ===============*/

uint16_t KillSwErrorCnt = 0;
uint8_t DIO_KillSwState = 0;

/*
 * Description:
 * 	From P1 stage, the KillSW-DO will connect to KillSw-DI through the Kill Switch,
 * once the signal from DI signal is different from DO command for a certain time period,
 * we will regard the KILL switch is opened
 * Params:
 * 	NULL
 */
void DIO_KillSWSignalCheck(void)
{
	if(KillSwErrorCnt < 20)
	{
		if((BtnTable[BTN_IDX_KILL_SW].EvtRecord != BTN_EVENT_INITIALIZING) &&
		   (BtnTable[BTN_IDX_KILL_SW].Now != DOCtrlArray[DO_IDX_KILLSW].Cmd))
		{
			KillSwErrorCnt++;
		}
		else
		{
			KillSwErrorCnt = 0;
		}
	}
}

/*
 * Description:
 * 	Returns the Kill Switch status
 * Params:
 * (Output) : if the KillSW is opened, return "DIO_KILL_SWITCH_STATE_OPEN",
 *  		  otherwise return "DIO_KILL_SWITCH_STATE_CLOSED"
 */

DIO_KillSwState_e DIO_KillSwStateGet(void)
{
	DIO_KillSwState_e ret = DIO_KILL_SWITCH_STATE_OPEN;

	if(KillSwErrorCnt < 20)
	{
		ret = DIO_KILL_SWITCH_STATE_CLOSED;
	}
	return ret;
}



/*
 * Description:
 * Perform all required routine(100hz) tasks of Digital input and output module,
 * Params:
 * N/A
 */
void DIO_Do100HzLoop(void)
{
	/* Perform routine tasks for digital input signal*/
	Btn_Do100HzLoop();

	/* Perform routine tasks for digital output signal*/
	DO_Do100HzLoop();

	/* Check Kill switch status by*/
	DIO_KillSWSignalCheck();
}


/*
 * Description:
 * Perform all required initial tasks of Digital input and output module,
 * Params:
 * N/A
 */
void DIO_Init(void)
{
	/* Perform routine tasks for digital input signal*/
	Btn_Init();

	/* Perform routine tasks for digital output signal*/
	// N/A

	/* others */
	/* set kill switch output signal*/
	KillSwErrorCnt = 0;
	DO_CtrlReq(DO_IDX_KILLSW, DO_MODE_BLINK, DO_NBR_TO_BLINK_FOREVER, DO_BLINK_CONFIG_5HZ);
}
