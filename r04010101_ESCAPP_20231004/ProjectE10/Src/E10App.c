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

/*=============== LED Indication control start ===============*/
LedCtrl_t LedCtrlArray[LED_IDX_MAX];
SocLightCtrl_t SocLightCtrl;


void Led_CtrlReq(LedIdx_e Idx, LedMode_e ModeIn, uint16_t NbrToBlink, LedBlinkConfig_t BlinkConfig)
{
  LedCtrlArray[Idx].Mode = ModeIn;
  LedCtrlArray[Idx].TimeCnt = 0;
  LedCtrlArray[Idx].NbrCnt = 0;
  LedCtrlArray[Idx].NbrToBlink = NbrToBlink;
  LedCtrlArray[Idx].BlinkConfig = BlinkConfig;
}

void Led_TurnOnReq(LedIdx_e Idx)
{
	Led_CtrlReq(Idx, LED_MODE_ON, 0, LED_BLINK_CONFIG_STEADY);
}

void Led_TurnOffReq(LedIdx_e Idx)
{
	Led_CtrlReq(Idx, LED_MODE_OFF, 0, LED_BLINK_CONFIG_STEADY);
}

void Led_Do100HzLoop(void)
{

  SocLightCtrl_Do100HzLoop();

  for (uint8_t i = 0; i < LED_IDX_MAX; i++)
  {
    switch (LedCtrlArray[i].Mode)
    {
      case LED_MODE_OFF:
      default:
        LedCtrlArray[i].Cmd = LED_CMD_OFF;
        break;
      
      case LED_MODE_ON:
        LedCtrlArray[i].Cmd = LED_CMD_ON;
        break;

      case LED_MODE_BLINK:

        if(LedCtrlArray[i].TimeCnt < LedCtrlArray[i].BlinkConfig.OnTime)
        {
          /* Turn on LED*/  
          LedCtrlArray[i].Cmd = LED_CMD_ON;
        }
        else if(LedCtrlArray[i].TimeCnt < LedCtrlArray[i].BlinkConfig.Period)
        {
          /* Turn off LED*/
          LedCtrlArray[i].Cmd = LED_CMD_OFF;
        }
        else
        {
          LedCtrlArray[i].TimeCnt = 0;  

          /* increase the blink conter if nbr to blink is greater than "LED_NBR_TO_BLINK_FOREVER" */
          if(LedCtrlArray[i].NbrToBlink > LED_NBR_TO_BLINK_FOREVER)
          {
        	  LedCtrlArray[i].NbrCnt++;

        	  if(LedCtrlArray[i].NbrCnt >= LedCtrlArray[i].NbrToBlink)
        	  {
        		  LedCtrlArray[i].Mode = LED_MODE_OFF;
        	  }
          }
        }
        
        LedCtrlArray[i].TimeCnt++;

        break;
    }
  }
}

LedCmd_e Led_CmdGet(LedIdx_e Idx)
{
  return LedCtrlArray[Idx].Cmd; 
}

/**/
void SocLightModeSet( SocDisplayMode_e ModeIn)
{
  SocLightCtrl.DisplayMode = ModeIn;
  SocLightCtrl.ReqFlag  = 1;
  /* Force to clear the LED indication first */
  for (uint8_t i = 0; i < 4 ; i++)
  {
    for (uint8_t j = 0; j < 3; j++)
    {
	  Led_TurnOffReq((LED_IDX_SOC1_R + j) + (i * 3));
    }
  }
}

void SocCtrlSet(LedRGBIdx_e ColorIn, LedMode_e ModeIn, LedBlinkConfig_t LedConfig)
{
  SocLightCtrl.ColorReq = ColorIn;
  SocLightCtrl.LedMode = ModeIn;
  SocLightCtrl.BlinkConfig = LedConfig;
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

void SocIndicationByBLinking(LedIdx_e LedIdxIn)
{
  if (SocLightCtrl.Soc >= 75)
  {
    Led_CtrlReq( LedIdxIn, LED_MODE_ON, 0, LED_BLINK_CONFIG_STEADY);
  }
  else if ( SocLightCtrl.Soc >= 50 )
  {
    Led_CtrlReq( LedIdxIn, LED_MODE_BLINK, LED_NBR_TO_BLINK_FOREVER, LED_BLINK_CONFIG_1HZ);
  }
  else if ( SocLightCtrl.Soc >= 25 )
  {
    Led_CtrlReq( LedIdxIn, LED_MODE_BLINK, LED_NBR_TO_BLINK_FOREVER, LED_BLINK_CONFIG_2HZ);    
  }
  else if ( SocLightCtrl.Soc >= 5 )
  {
    Led_CtrlReq( LedIdxIn, LED_MODE_BLINK, LED_NBR_TO_BLINK_FOREVER, LED_BLINK_CONFIG_4HZ);    
  }
  else 
  {
    Led_CtrlReq( LedIdxIn, LED_MODE_OFF, 0, LED_BLINK_CONFIG_STEADY);        
  }
}


void SocLightCtrl_Do100HzLoop(void)
{
  uint8_t ScaledSoc;

  if(SocLightCtrl.ReqFlag == 1)
  {
    if(SocLightCtrl.DisplayMode == SOC_DISPLAY_MODE_SOC_LED_ARRAY)
    {
      /* decide the # of LED to turn on */
      ScaledSoc = ( SocLightCtrl.Soc ) / 25;

      if(SocLightCtrl.Soc > 0)
      {
        ScaledSoc += 1;
      }

      /* Light the LED according to number, blink pattern and color selected by user */
      for(uint8_t i = 0; i < 4 ; i++)
      {
        for(uint8_t j = 0; j < 3; j++)
        {
        Led_TurnOffReq((LED_IDX_SOC1_R + j) + (i * 3));
        }

        if( i < ScaledSoc )
        {
        Led_CtrlReq((LED_IDX_SOC1_R + SocLightCtrl.ColorReq + i * 3), SocLightCtrl.LedMode, 0, SocLightCtrl.BlinkConfig);
        }
      }
    }
    else if(SocLightCtrl.DisplayMode == SOC_DISPLAY_MODE_SOC_SINGLE_LED)
    {
      SocIndicationByBLinking(LED_IDX_REAR);
    }

    SocLightCtrl.ReqFlag = 0;
  }

}

/*=============== LED Indication control End ===============*/
