/*
 * PWM_Init_Table.c
 *
 *  Created on: 2020年2月6日
 *      Author: MikeSFWen
 */
#if BME
#include "PWM_Init_Table.h"

/*
 * Structure or array define / declare
 */

void PwmTIMx_Init_Func( const PWM_TIM_INIT_PARA *Para )
{
  TIM_HandleTypeDef *Group;
  /*
   * Declare the variable in Function
   */
  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIMEx_BreakInputConfigTypeDef sBreakInputConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  Group = Para->TimGroup;
  /*
   * Load in the basic timer Parameter of Pwm timer
   */
  Group->Instance = Para->GroupIdx;
  Group->Init.Prescaler = ( Para->PreScale - 1 );
  Group->Init.CounterMode = Para->CounterMode;
  Group->Init.Period = ( Para->Period - 1 );
  Group->Init.ClockDivision = Para->Division;
  Group->Init.RepetitionCounter = Para->RcrPara;
  Group->Init.AutoReloadPreload = Para->AutoPreloadMode;
  if (HAL_TIM_Base_Init( Para->TimGroup) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = Para->ClkSource;
  if (HAL_TIM_ConfigClockSource( Para->TimGroup, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init( Para->TimGroup) != HAL_OK)
  {
    Error_Handler();
  }
  /*
   * Setup the trigger source
   */
  sMasterConfig.MasterOutputTrigger = Para->TrigSource1;
  sMasterConfig.MasterOutputTrigger2 = Para->TrigSource2;
  sMasterConfig.MasterSlaveMode = Para->SlaveMode;
  if (HAL_TIMEx_MasterConfigSynchronization( Para->TimGroup, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /*
   * Setup the break in source ( Trip Zone )
   */
  sBreakInputConfig.Source = Para->BreakSource;
  sBreakInputConfig.Enable = Para->BreakSourceEanble;
  sBreakInputConfig.Polarity = Para->BreakSourcePriority;
  if (HAL_TIMEx_ConfigBreakInput( Para->TimGroup,  Para->BreakSource, &sBreakInputConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /*
   * Setup the Pwm output channel
   */
#if  0// ESC_VERSION_1.0
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = Para->InitDutyCntCh1st;
  sConfigOC.OCPolarity = Para->OCPolarity;
  sConfigOC.OCNPolarity = Para->OCNPolarity;
  sConfigOC.OCFastMode = TIM_OCMODE_PWM1;
#else // ESC_VERSION_1.3
  sConfigOC.OCMode = Para->OCMode;
  sConfigOC.Pulse = Para->InitDutyCntCh1st;
  sConfigOC.OCPolarity = Para->OCPolarity;
  sConfigOC.OCNPolarity = Para->OCNPolarity;
  sConfigOC.OCFastMode = Para->OCMode;
#endif
  sConfigOC.OCIdleState = Para->OCIdleState;
  sConfigOC.OCNIdleState = Para->OCNIdleState;
  if (HAL_TIM_PWM_ConfigChannel( Para->TimGroup, &sConfigOC, Para->Channel1) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = Para->OCMode;
  sConfigOC.OCPolarity = Para->OCPolarity;
  sConfigOC.OCNPolarity = Para->OCNPolarity;
  sConfigOC.OCFastMode = Para->OCMode;
  sConfigOC.Pulse = Para->InitDutyCntCh2ed;
  if (HAL_TIM_PWM_ConfigChannel( Para->TimGroup, &sConfigOC,  Para->Channel2) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = Para->OCMode;
  sConfigOC.OCPolarity = Para->OCPolarity;
  sConfigOC.OCNPolarity = Para->OCNPolarity;
  sConfigOC.OCFastMode = Para->OCMode;
  sConfigOC.Pulse = Para->InitDutyCntCh3rd;
  if (HAL_TIM_PWM_ConfigChannel( Para->TimGroup, &sConfigOC,  Para->Channel3) != HAL_OK)
  {
    Error_Handler();
  }
  /*
   * Setup the break parameter
   */
  HAL_TIMEx_EnableDeadTimePreload( Para->TimGroup);
  sBreakDeadTimeConfig.OffStateRunMode = Para->OffStateRunMode;
  sBreakDeadTimeConfig.OffStateIDLEMode = Para->OffStateIDLEMode;
  sBreakDeadTimeConfig.LockLevel = Para->LockLevel;
  sBreakDeadTimeConfig.DeadTime = Para->DeadTimePara;
  sBreakDeadTimeConfig.BreakState = Para->BreakState;
  sBreakDeadTimeConfig.BreakPolarity = Para->BreakPolarity;
  sBreakDeadTimeConfig.BreakFilter = Para->BreakFilter;
  sBreakDeadTimeConfig.BreakAFMode = Para->BreakAFMode;
  sBreakDeadTimeConfig.Break2State = Para->Break2State;
  sBreakDeadTimeConfig.Break2Polarity = Para->Break2Polarity;
  sBreakDeadTimeConfig.Break2Filter = Para->Break2Filter;
  sBreakDeadTimeConfig.Break2AFMode = Para->Break2AFMode;
  sBreakDeadTimeConfig.AutomaticOutput = Para->AutomaticOutput;
  /*
   * Load the Break information into the Reg
   */
  if (HAL_TIMEx_ConfigBreakDeadTime( Para->TimGroup, &sBreakDeadTimeConfig ) != HAL_OK)
  {
    Error_Handler();
  }
  HAL_TIM_MspPostInit( Para->TimGroup);
}
#endif
