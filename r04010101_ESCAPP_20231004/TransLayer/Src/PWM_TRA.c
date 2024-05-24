/*
 * PWM_DRI.c
 *
 *  Created on: Dec 30, 2019
 *      Author: Hank Chen
 */

#include "PWM_TRA.h"
/*
 * Function implement / declare
 */
uint32_t PwmModePrevious=0;

void TIM_CCxNChannelCmd_Local(TIM_TypeDef *TIMx, uint32_t Channel, uint32_t ChannelNState)
{
	uint32_t tmp;

	tmp = TIM_CCER_CC1NE << (Channel & 0x1FU); /* 0x1FU = 31 bits max shift */

	/* Reset the CCxNE Bit */
	TIMx->CCER &=  ~tmp;

	/* Set or reset the CCxNE Bit */
	TIMx->CCER |= (uint32_t)(ChannelNState << (Channel & 0x1FU)); /* 0x1FU = 31 bits max shift */
}

void TIM_CCxAllChannelCmdClose_Local(TIM_TypeDef *TIMx, uint32_t Channel, uint32_t ChannelNState)
{
	uint32_t tmp;

	tmp = (TIM_CCER_CC1NE | TIM_CCER_CC1E ) << (Channel & 0x1FU); /* 0x1FU = 31 bits max shift */

	/* Reset the CCxNE Bit */
	TIMx->CCER &=  ~tmp;

	/* Set or reset the CCxNE Bit */
	TIMx->CCER |= (uint32_t)(ChannelNState << (Channel & 0x1FU)); /* 0x1FU = 31 bits max shift */
}

void PwmStation_Init( PwmStation *v )
{
	v->PwmTimerParam[AXIS_0] = PCUTable.Pwm_Tim_Initializer_Axis1;

	// Axis 1 channel mapping
	v->PwmCh[AXIS_0].Group = &htim8;
	HAL_TIM_Base_Start_IT((TIM_HandleTypeDef *)v->PwmCh[AXIS_0].Group);
}

