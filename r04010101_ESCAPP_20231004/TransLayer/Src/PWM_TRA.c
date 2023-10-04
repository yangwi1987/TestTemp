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
	v->Duty2PwmCountGain = v->PwmTimerParam[AXIS_0].Period;

	PwmTIMx_Init_Func( &v->PwmTimerParam[AXIS_0] );

	// Axis 1 channel mapping
	PwmStation_AxisChannelMapping(v, AXIS_0, PCUTable.PwmStartUpTable_Axis1);
	PwmStation_AxisChannelStartup(v, AXIS_0, PCUTable.PwmStartUpTable_Axis1);
}

void PwmStation_AxisChannelMapping( PwmStation *v, uint16_t AxisID, const PWM_START_TABLE_INFO *pTable )
{
	switch( pTable[0].Group )
	{
	case TIM_1:
		v->PwmCh[AxisID].Group = &htim1;
		break;

	case TIM_8:
		v->PwmCh[AxisID].Group = &htim8;
		break;

	case TIM_20:
		v->PwmCh[AxisID].Group = &htim20;
		break;
	}

	switch( pTable[CH_PWM_UP].Channel )
	{
	case TIM_CH1:
		v->PwmCh[AxisID].UpChannel = TIM_CHANNEL_1;
		v->PwmCh[AxisID].UnChannel = TIM_CHANNEL_1;
		break;

	case TIM_CH2:
		v->PwmCh[AxisID].UpChannel = TIM_CHANNEL_2;
		v->PwmCh[AxisID].UnChannel = TIM_CHANNEL_2;
		break;

	case TIM_CH3:
		v->PwmCh[AxisID].UpChannel = TIM_CHANNEL_3;
		v->PwmCh[AxisID].UnChannel = TIM_CHANNEL_3;
		break;

	case TIM_CH4:
		v->PwmCh[AxisID].UpChannel = TIM_CHANNEL_4;
		v->PwmCh[AxisID].UnChannel = TIM_CHANNEL_4;
		break;

	case TIM_CH5:
		v->PwmCh[AxisID].UpChannel = TIM_CHANNEL_5;
		v->PwmCh[AxisID].UnChannel = TIM_CHANNEL_5;
		break;

	case TIM_CH6:
		v->PwmCh[AxisID].UpChannel = TIM_CHANNEL_6;
		v->PwmCh[AxisID].UnChannel = TIM_CHANNEL_6;
		break;
	}

	switch( pTable[CH_PWM_VP].Channel )
	{
	case TIM_CH1:
		v->PwmCh[AxisID].VpChannel = TIM_CHANNEL_1;
		v->PwmCh[AxisID].VnChannel = TIM_CHANNEL_1;
		break;

	case TIM_CH2:
		v->PwmCh[AxisID].VpChannel = TIM_CHANNEL_2;
		v->PwmCh[AxisID].VnChannel = TIM_CHANNEL_2;
		break;

	case TIM_CH3:
		v->PwmCh[AxisID].VpChannel = TIM_CHANNEL_3;
		v->PwmCh[AxisID].VnChannel = TIM_CHANNEL_3;
		break;

	case TIM_CH4:
		v->PwmCh[AxisID].VpChannel = TIM_CHANNEL_4;
		v->PwmCh[AxisID].VnChannel = TIM_CHANNEL_4;
		break;

	case TIM_CH5:
		v->PwmCh[AxisID].VpChannel = TIM_CHANNEL_5;
		v->PwmCh[AxisID].VnChannel = TIM_CHANNEL_5;
		break;

	case TIM_CH6:
		v->PwmCh[AxisID].VpChannel = TIM_CHANNEL_6;
		v->PwmCh[AxisID].VnChannel = TIM_CHANNEL_6;
		break;
	}

	switch( pTable[CH_PWM_WP].Channel )
	{
	case TIM_CH1:
		v->PwmCh[AxisID].WpChannel = TIM_CHANNEL_1;
		v->PwmCh[AxisID].WnChannel = TIM_CHANNEL_1;
		break;

	case TIM_CH2:
		v->PwmCh[AxisID].WpChannel = TIM_CHANNEL_2;
		v->PwmCh[AxisID].WnChannel = TIM_CHANNEL_2;
		break;

	case TIM_CH3:
		v->PwmCh[AxisID].WpChannel = TIM_CHANNEL_3;
		v->PwmCh[AxisID].WnChannel = TIM_CHANNEL_3;
		break;

	case TIM_CH4:
		v->PwmCh[AxisID].WpChannel = TIM_CHANNEL_4;
		v->PwmCh[AxisID].WnChannel = TIM_CHANNEL_4;
		break;

	case TIM_CH5:
		v->PwmCh[AxisID].WpChannel = TIM_CHANNEL_5;
		v->PwmCh[AxisID].WnChannel = TIM_CHANNEL_5;
		break;

	case TIM_CH6:
		v->PwmCh[AxisID].WpChannel = TIM_CHANNEL_6;
		v->PwmCh[AxisID].WnChannel = TIM_CHANNEL_6;
		break;
	}
}

void PwmStation_AxisChannelStartup( PwmStation *v, uint16_t AxisID, const PWM_START_TABLE_INFO *pTable )
{
	if( pTable[0].GroupEnable == PWM_ENABLE )
	{
		v->AxisChannelLock( v, AxisID );
		HAL_TIM_PWM_Start_IT( (TIM_HandleTypeDef *)v->PwmCh[AxisID].Group, v->PwmCh[AxisID].UpChannel );
		HAL_TIM_PWM_Start_IT( (TIM_HandleTypeDef *)v->PwmCh[AxisID].Group, v->PwmCh[AxisID].VpChannel );
		HAL_TIM_PWM_Start_IT( (TIM_HandleTypeDef *)v->PwmCh[AxisID].Group, v->PwmCh[AxisID].WpChannel );
		HAL_TIMEx_PWMN_Start_IT( (TIM_HandleTypeDef *)v->PwmCh[AxisID].Group, v->PwmCh[AxisID].UnChannel );
		HAL_TIMEx_PWMN_Start_IT( (TIM_HandleTypeDef *)v->PwmCh[AxisID].Group, v->PwmCh[AxisID].VnChannel );
		HAL_TIMEx_PWMN_Start_IT( (TIM_HandleTypeDef *)v->PwmCh[AxisID].Group, v->PwmCh[AxisID].WnChannel );
//		v->AxisChannelLock( v, AxisID );

		// Enable Time base interrupt
		HAL_TIM_Base_Start_IT((TIM_HandleTypeDef *)v->PwmCh[AxisID].Group);
	}
}

void PwmStation_AxisChannelLock( PwmStation *v, uint16_t AxisID )
{
	TIM_TypeDef *pTimGroup = v->PwmCh[AxisID].Group->Instance;

//	TIM_CCxChannelCmd(pTimGroup, v->PwmCh[AxisID].UpChannel, TIM_CCx_DISABLE);
//	TIM_CCxChannelCmd(pTimGroup, v->PwmCh[AxisID].VpChannel, TIM_CCx_DISABLE);
//	TIM_CCxChannelCmd(pTimGroup, v->PwmCh[AxisID].WpChannel, TIM_CCx_DISABLE);
//	TIM_CCxNChannelCmd_Local(pTimGroup, v->PwmCh[AxisID].UnChannel, TIM_CCxN_DISABLE);
//	TIM_CCxNChannelCmd_Local(pTimGroup, v->PwmCh[AxisID].VnChannel, TIM_CCxN_DISABLE);
//	TIM_CCxNChannelCmd_Local(pTimGroup, v->PwmCh[AxisID].WnChannel, TIM_CCxN_DISABLE);

	TIM_CCxAllChannelCmdClose_Local(pTimGroup, v->PwmCh[AxisID].UpChannel, TIM_CCx_DISABLE | TIM_CCxN_DISABLE);
	TIM_CCxAllChannelCmdClose_Local(pTimGroup, v->PwmCh[AxisID].VpChannel, TIM_CCx_DISABLE | TIM_CCxN_DISABLE);
	TIM_CCxAllChannelCmdClose_Local(pTimGroup, v->PwmCh[AxisID].WpChannel, TIM_CCx_DISABLE | TIM_CCxN_DISABLE);
}

void PwmStation_AxisChannelUnlock( PwmStation *v, uint16_t AxisID )
{
	TIM_TypeDef *pTimGroup = v->PwmCh[AxisID].Group->Instance;

	TIM_CCxChannelCmd(pTimGroup, v->PwmCh[AxisID].UpChannel, TIM_CCx_ENABLE);
	TIM_CCxChannelCmd(pTimGroup, v->PwmCh[AxisID].VpChannel, TIM_CCx_ENABLE);
	TIM_CCxChannelCmd(pTimGroup, v->PwmCh[AxisID].WpChannel, TIM_CCx_ENABLE);
	TIM_CCxNChannelCmd_Local(pTimGroup, v->PwmCh[AxisID].UnChannel, TIM_CCxN_ENABLE);
	TIM_CCxNChannelCmd_Local(pTimGroup, v->PwmCh[AxisID].VnChannel, TIM_CCxN_ENABLE);
	TIM_CCxNChannelCmd_Local(pTimGroup, v->PwmCh[AxisID].WnChannel, TIM_CCxN_ENABLE);
}

uint16_t PwmStation_AxisDutyToPwmCount( PwmStation *v, uint16_t AxisID, uint16_t PwmMode )
{
	uint16_t Rtn = 0;
	uint32_t AnsU, AnsV, AnsW = 0;
	TIM_HandleTypeDef *Group;
	TIM_TypeDef *pTimGroup = v->PwmCh[AxisID].Group->Instance;

	Group = v->PwmCh[AxisID].Group;
	v->PwmMode = PwmMode;

//	uint32_t UpEnable = TIM_CCx_ENABLE;
//	uint32_t UnEnable = TIM_CCxN_ENABLE;
//	uint32_t VpEnable = TIM_CCx_ENABLE;
//	uint32_t VnEnable = TIM_CCxN_ENABLE;
//	uint32_t WpEnable = TIM_CCx_ENABLE;
//	uint32_t WnEnable = TIM_CCxN_ENABLE;

	if( v->PwmTimerParam[AxisID].OCPolarity == TIM_OCPOLARITY_LOW )
	{
		AnsU = ( 1.0f - v->DutyCmd.Duty[CH_PWM_UP] ) * v->Duty2PwmCountGain;
		AnsV = ( 1.0f - v->DutyCmd.Duty[CH_PWM_VP] ) * v->Duty2PwmCountGain;
		AnsW = ( 1.0f - v->DutyCmd.Duty[CH_PWM_WP] ) * v->Duty2PwmCountGain;
	}
	else
	{
		AnsU = v->DutyCmd.Duty[CH_PWM_UP] * v->Duty2PwmCountGain;
		AnsV = v->DutyCmd.Duty[CH_PWM_VP] * v->Duty2PwmCountGain;
		AnsW = v->DutyCmd.Duty[CH_PWM_WP] * v->Duty2PwmCountGain;
	}
	Group->Instance->CCR2 = AnsU;
	Group->Instance->CCR1 = AnsV;
	Group->Instance->CCR4 = AnsW;
//	__HAL_TIM_SET_COMPARE( Group, v->PwmCh[AxisID].UpChannel, AnsU );
//	__HAL_TIM_SET_COMPARE( Group, v->PwmCh[AxisID].VpChannel, AnsV );
//	__HAL_TIM_SET_COMPARE( Group, v->PwmCh[AxisID].WpChannel, AnsW );
	if (PwmMode!=PwmModePrevious)
	{
		uint32_t UpEnable = TIM_CCx_ENABLE;
		uint32_t UnEnable = TIM_CCxN_ENABLE;
		uint32_t VpEnable = TIM_CCx_ENABLE;
		uint32_t VnEnable = TIM_CCxN_ENABLE;
		uint32_t WpEnable = TIM_CCx_ENABLE;
		uint32_t WnEnable = TIM_CCxN_ENABLE;
		switch (PwmMode)
		{
			case PWM_COMPLEMENTARY:
			{
				break;
			}
			case PWM_UbVbWx:
			{
				WpEnable = TIM_CCx_DISABLE;
				WnEnable = TIM_CCxN_DISABLE;
				break;
			}
			case PWM_UxVbWb:
			{
				UpEnable = TIM_CCx_DISABLE;
				UnEnable = TIM_CCxN_DISABLE;
				break;
			}
			case PWM_UbVxWb:
			{
				VpEnable = TIM_CCx_DISABLE;
				VnEnable = TIM_CCxN_DISABLE;
				break;
			}
			default:
			{
				break;
			}
		}
		TIM_CCxChannelCmd(pTimGroup, v->PwmCh[AxisID].UpChannel, UpEnable);
		TIM_CCxChannelCmd(pTimGroup, v->PwmCh[AxisID].VpChannel, VpEnable);
		TIM_CCxChannelCmd(pTimGroup, v->PwmCh[AxisID].WpChannel, WpEnable);
		TIM_CCxNChannelCmd_Local(pTimGroup, v->PwmCh[AxisID].UnChannel, UnEnable);
		TIM_CCxNChannelCmd_Local(pTimGroup, v->PwmCh[AxisID].VnChannel, VnEnable);
		TIM_CCxNChannelCmd_Local(pTimGroup, v->PwmCh[AxisID].WnChannel, WnEnable);
		PwmModePrevious=PwmMode;
	}
	Rtn = PWM_SUCCESS;

	return Rtn;

}
