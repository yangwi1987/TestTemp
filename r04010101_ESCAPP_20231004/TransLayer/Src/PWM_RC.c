/*
 * PWM_RC.c
 *
 *  Created on: Feb 10, 2022
 *      Author: Hank.Chen.CHC
 */

#include "Drive.h"

void PWM_RC_Init( PWM_RC_TYPE *p )
{
//	p->tim = TIM2;
//	p->Captim = &htim2;
//	HAL_TIM_IC_Start_IT( p->Captim, TIM_CHANNEL_1 );
//	HAL_TIM_IC_Start_IT( p->Captim, TIM_CHANNEL_2 );
}

void PWM_RC_Capture( PWM_RC_TYPE *p )
{
//	float PeriodCnt = 0.0F;
//	float DutyCnt = 0.0F;
//
//	if( p->Captim->Channel == HAL_TIM_ACTIVE_CHANNEL_1 )
//	{
//		/* Get the Input Capture value - period count */
//		p->PwmPeriodEdgeValue = HAL_TIM_ReadCapturedValue( p->Captim, TIM_CHANNEL_1 );
//
//		/* Calculate the period of MR sensor */
//		PeriodCnt = ( float )p->PwmPeriodEdgeValue;
//
//		if( p->PwmPeriodEdgeValue > 0 )
//		{
//			/* Get the Input capture value - duty count*/
//			p->PwmDutyEdgeValue = HAL_TIM_ReadCapturedValue( p->Captim, TIM_CHANNEL_2 );
//
//			/* Calculate the duty of rotor */
//			DutyCnt = ( float )p->PwmDutyEdgeValue - ( ( float )p->PwmPeriodEdgeValue );
//
//			/* Calculate the degree of rotor */
//			p->PwmDuty = DutyCnt / PeriodCnt;
//		}
//	}
}

void PWM_RC_Abnormal_Detect( PWM_RC_TYPE *p )
{
	p->AbnormalCnt++;
	if( p->AbnormalCnt >= MAX_CNT_ALARM_TRIG  )
	{
		p->DutyRaw = 0.1f;
		p->AbnormalCnt = MAX_CNT_ALARM_TRIG;
	}

	else;
}
