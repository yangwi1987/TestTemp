/*
 * Drive.c
 *
 *  Created on: Dec 19, 2019
 *      Author: MikeSFWen
 */

#include "UtilityBase.h"
#include "UiApp.h"
#include "Drive.h"


DriveParams_t DriveParams;
PwmStation PwmStation1 = PWM_STATION_DEFAULT;
AdcStation AdcStation1 = ADC_STATION_DEFAULT;
ExtranetCANStation_t ExtranetCANStation = EXTRANET_CAN_STATION_DEFAULT;

extern const CANProtocol ExtranetInformInSystemTableExample;


void drive_Init(void)
{


	// Init Pwm Station
	PwmStation1.Init( &PwmStation1 );

	// Init ADC from tables
	AdcStation1.Init( &AdcStation1 );

	// Init CAN external
	ExtranetCANStation.DriveSetup.LoadParam( &ExtranetCANStation.DriveSetup, &CANModuleConfigExtra, CanIdTableExtra );
	ExtranetCANStation.Init( &ExtranetCANStation,&ExtranetInformInSystemTableExample, &hfdcan2);

	ExtranetCANStation.Enable = ENABLE;
	ExtranetCANStation.ForceDisable = DISABLE;

}


// 1kHz
void drive_DoPLCLoop(void)
{

	ExtranetCANStation.TxInfo.Debugf[IDX_IU_FBK] = AdcStation1.AdcRawData.Inj[ISE_U_A0].RawAdcValue;
    ExtranetCANStation.TxInfo.Debugf[IDX_IV_FBK] = AdcStation1.AdcRawData.Inj[ISE_V_A0].RawAdcValue;
    ExtranetCANStation.TxInfo.Debugf[IDX_IW_FBK] = AdcStation1.AdcRawData.Inj[ISE_W_A0].RawAdcValue;
    ExtranetCANStation.TxInfo.Debugf[IDX_PREC] = AdcStation1.AdcRawData.Inj[BAT_VDC].RawAdcValue;
	ExtranetCANStation.DisableRst( &ExtranetCANStation );

	ExtranetCANStation.DoPlcLoop ( &ExtranetCANStation );


}

/*
 * Application Jump to Boot-loader
 */


void DisableMcuModule( void )
{
	/*
	 * DeInit All Communication Function
	 */

	// CAN 2
	HAL_FDCAN_DeactivateNotification(&hfdcan2, FDCAN_IT_RX_FIFO0_NEW_MESSAGE);
	HAL_FDCAN_MspDeInit(&hfdcan2);
	__HAL_RCC_FDCAN_FORCE_RESET();

	//UART
//	HAL_UART_MspDeInit(&huart3);
//	__HAL_RCC_UART5_FORCE_RESET();

	//USART
//	HAL_UART_MspDeInit(&huart3);
//	__HAL_RCC_USART3_FORCE_RESET();

	//USART
//	HAL_USART_MspDeInit(&husart2);
//	__HAL_RCC_USART2_FORCE_RESET();

	//SPI
	HAL_SPI_MspDeInit(&hspi1);
	__HAL_RCC_SPI1_FORCE_RESET();

	/*
	 * DeInit GPIO State/ Flag/ IT
	 */
	__HAL_RCC_GPIOA_CLK_DISABLE();

	__HAL_RCC_GPIOB_CLK_DISABLE();

	__HAL_RCC_GPIOC_CLK_DISABLE();

	__HAL_RCC_GPIOD_CLK_DISABLE();

	__HAL_RCC_GPIOE_CLK_DISABLE();

	__HAL_RCC_GPIOF_CLK_DISABLE();

	__HAL_RCC_GPIOG_CLK_DISABLE();

    /*
     * DeInit Timer
     */

	// Timer 2
    HAL_TIM_Base_Stop_IT( &htim2 );
	__HAL_TIM_DISABLE_IT( &htim2, TIM_IT_IDX );
    HAL_TIM_Base_MspDeInit( &htim2 );
	__HAL_RCC_TIM2_FORCE_RESET();

	// Timer 3
    HAL_TIM_Base_Stop_IT( &htim3 );
    HAL_TIM_Base_MspDeInit( &htim3 );
    __HAL_RCC_TIM3_FORCE_RESET();

    // Timer 6
    HAL_TIM_Base_Stop_IT( &htim6 );
    HAL_TIM_Base_MspDeInit( &htim6 );
    __HAL_RCC_TIM6_FORCE_RESET();

    // Timer 7
    HAL_TIM_Base_Stop_IT( &htim7 );
    HAL_TIM_Base_MspDeInit( &htim7 );
    __HAL_RCC_TIM7_FORCE_RESET();

    // Timer 8
    HAL_TIM_Base_Stop_IT( &htim8 );
    HAL_TIM_Base_MspDeInit( &htim8 );
    __HAL_RCC_TIM8_FORCE_RESET();

	// Timer 20
	HAL_TIM_Base_Stop_IT( &htim20 );
	HAL_TIM_Base_MspDeInit( &htim20 );
	__HAL_RCC_TIM20_FORCE_RESET();

	/*
	 * DeInit ADC & DAC
	 */
	//ADC2
	HAL_ADCEx_InjectedStop_IT(&hadc2);
	HAL_ADC_MspDeInit(&hadc2);
	__HAL_RCC_ADC12_FORCE_RESET();
	// ADC 3
	HAL_ADCEx_InjectedStop_IT(&hadc3);
	HAL_ADC_MspDeInit(&hadc3);

	// ADC 4
	HAL_ADCEx_InjectedStop_IT(&hadc4);
	HAL_ADC_MspDeInit(&hadc4);
	__HAL_RCC_ADC345_FORCE_RESET();

	//DAC
	HAL_DAC_MspDeInit(&hdac1);
	__HAL_RCC_DAC1_FORCE_RESET();

	/*
	 * Other Functions
	 */
	HAL_CORDIC_MspInit(&hcordic);
	__HAL_RCC_CORDIC_FORCE_RESET();
}
