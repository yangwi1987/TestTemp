/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32g4xx_it.c
  * @brief   Interrupt Service Routines.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32g4xx_it.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "Drive.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */

/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
 
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */

#if JUDGE_FUNCTION_DELAY
Judge_Delay TIM20INT_Judge_Delay = { 0 };
Judge_Delay CurrentLoop_Judge_Delay = { 0 };
Judge_Delay PLCLoop_Judge_Delay = { 0 };
Judge_Delay _100HzLoop_Judge_Delay = { 0 };
float aveDelta_filter_coef = 0.001;
#endif
#if MEASURE_CPU_LOAD
uint32_t Max_100Hz_Cnt = 0.0f;
float Max_100Hz_Load_pct = 100.0f;
uint32_t Max_PLCLoop_Cnt = 0.0f;
float Max_PLCLoop_Load_pct = 100.0f;
uint32_t Max_CurrentLoop_Cnt = 0.0f;
float Max_CurrentLoop_Load_pct = 100.0f;
uint32_t Max_ADC_Inj_Cnt = 0.0f;
float Max_ADC_Inj_Load_pct = 100.0f;

uint32_t Ave_100Hz_Cnt = 0.0f;
float Ave_100Hz_Load_pct = 100.0f;
uint32_t Ave_PLCLoop_Cnt = 0.0f;
float Ave_PLCLoop_Load_pct = 100.0f;
uint32_t Ave_CurrentLoop_Cnt = 0.0f;
float Ave_CurrentLoop_Load_pct = 100.0f;

float aveLoad_filter_coef = 0.001;
#endif

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void USER_HAL_TIM_IRQHandler( TIM_HandleTypeDef *htim );
void USER_HAL_TIM_7_IRQHandler( TIM_HandleTypeDef *htim );
void USER_HAL_ADC_IRQHandler( ADC_HandleTypeDef *hadc );
/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern DMA_HandleTypeDef hdma_adc1;
extern DMA_HandleTypeDef hdma_adc2;
extern DMA_HandleTypeDef hdma_adc3;
extern ADC_HandleTypeDef hadc1;
extern ADC_HandleTypeDef hadc2;
extern FDCAN_HandleTypeDef hfdcan1;
extern FDCAN_HandleTypeDef hfdcan2;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim5;
extern TIM_HandleTypeDef htim7;
extern TIM_HandleTypeDef htim20;
extern DMA_HandleTypeDef hdma_uart5_rx;
extern DMA_HandleTypeDef hdma_uart5_tx;
extern USART_HandleTypeDef husart2;
/* USER CODE BEGIN EV */

extern TIM_HandleTypeDef htim16;
/* USER CODE END EV */

/******************************************************************************/
/*           Cortex-M4 Processor Interruption and Exception Handlers          */ 
/******************************************************************************/
/**
  * @brief This function handles Non maskable interrupt.
  */
void NMI_Handler(void)
{
  /* USER CODE BEGIN NonMaskableInt_IRQn 0 */

  /* USER CODE END NonMaskableInt_IRQn 0 */
  /* USER CODE BEGIN NonMaskableInt_IRQn 1 */

  /* USER CODE END NonMaskableInt_IRQn 1 */
}

/**
  * @brief This function handles Hard fault interrupt.
  */
void HardFault_Handler(void)
{
  /* USER CODE BEGIN HardFault_IRQn 0 */

  /* USER CODE END HardFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_HardFault_IRQn 0 */
    /* USER CODE END W1_HardFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Memory management fault.
  */
void MemManage_Handler(void)
{
  /* USER CODE BEGIN MemoryManagement_IRQn 0 */

  /* USER CODE END MemoryManagement_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_MemoryManagement_IRQn 0 */
    /* USER CODE END W1_MemoryManagement_IRQn 0 */
  }
}

/**
  * @brief This function handles Prefetch fault, memory access fault.
  */
void BusFault_Handler(void)
{
  /* USER CODE BEGIN BusFault_IRQn 0 */

  /* USER CODE END BusFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_BusFault_IRQn 0 */
    /* USER CODE END W1_BusFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Undefined instruction or illegal state.
  */
void UsageFault_Handler(void)
{
  /* USER CODE BEGIN UsageFault_IRQn 0 */

  /* USER CODE END UsageFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_UsageFault_IRQn 0 */
    /* USER CODE END W1_UsageFault_IRQn 0 */
  }
}

/**
  * @brief This function handles System service call via SWI instruction.
  */
void SVC_Handler(void)
{
  /* USER CODE BEGIN SVCall_IRQn 0 */

  /* USER CODE END SVCall_IRQn 0 */
  /* USER CODE BEGIN SVCall_IRQn 1 */

  /* USER CODE END SVCall_IRQn 1 */
}

/**
  * @brief This function handles Debug monitor.
  */
void DebugMon_Handler(void)
{
  /* USER CODE BEGIN DebugMonitor_IRQn 0 */

  /* USER CODE END DebugMonitor_IRQn 0 */
  /* USER CODE BEGIN DebugMonitor_IRQn 1 */

  /* USER CODE END DebugMonitor_IRQn 1 */
}

/**
  * @brief This function handles Pendable request for system service.
  */
void PendSV_Handler(void)
{
  /* USER CODE BEGIN PendSV_IRQn 0 */

  /* USER CODE END PendSV_IRQn 0 */
  /* USER CODE BEGIN PendSV_IRQn 1 */

  /* USER CODE END PendSV_IRQn 1 */
}

/**
  * @brief This function handles System tick timer.
  */
void SysTick_Handler(void)
{
  /* USER CODE BEGIN SysTick_IRQn 0 */

  /* USER CODE END SysTick_IRQn 0 */
  HAL_IncTick();
  /* USER CODE BEGIN SysTick_IRQn 1 */

  /* USER CODE END SysTick_IRQn 1 */
}

/******************************************************************************/
/* STM32G4xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32g4xx.s).                    */
/******************************************************************************/

/**
  * @brief This function handles DMA1 channel1 global interrupt.
  */
void DMA1_Channel1_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Channel1_IRQn 0 */

  /* USER CODE END DMA1_Channel1_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_adc1);
  /* USER CODE BEGIN DMA1_Channel1_IRQn 1 */

  /* USER CODE END DMA1_Channel1_IRQn 1 */
}

/**
  * @brief This function handles DMA1 channel2 global interrupt.
  */
void DMA1_Channel2_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Channel2_IRQn 0 */

  /* USER CODE END DMA1_Channel2_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_adc2);
  /* USER CODE BEGIN DMA1_Channel2_IRQn 1 */

  /* USER CODE END DMA1_Channel2_IRQn 1 */
}

/**
  * @brief This function handles DMA1 channel3 global interrupt.
  */
void DMA1_Channel3_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Channel3_IRQn 0 */

  /* USER CODE END DMA1_Channel3_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_adc3);
  /* USER CODE BEGIN DMA1_Channel3_IRQn 1 */


  /* USER CODE END DMA1_Channel3_IRQn 1 */
}

/**
  * @brief This function handles DMA1 channel4 global interrupt.
  */
void DMA1_Channel4_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Channel4_IRQn 0 */

  /* USER CODE END DMA1_Channel4_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_uart5_rx);
  /* USER CODE BEGIN DMA1_Channel4_IRQn 1 */

  /* USER CODE END DMA1_Channel4_IRQn 1 */
}

/**
  * @brief This function handles DMA1 channel5 global interrupt.
  */
void DMA1_Channel5_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Channel5_IRQn 0 */

  /* USER CODE END DMA1_Channel5_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_uart5_tx);
  /* USER CODE BEGIN DMA1_Channel5_IRQn 1 */
  RCCommCtrl.TxFlag = RC_COMM_TX_STATE_IDLE;
  RCCommCtrl.pTarget->gState = HAL_UART_STATE_READY;
  /* USER CODE END DMA1_Channel5_IRQn 1 */
}
/**
  * @brief This function handles ADC1 and ADC2 global interrupt.
  */
void ADC1_2_IRQHandler(void)
{
  /* USER CODE BEGIN ADC1_2_IRQn 0 */
#if MEASURE_CPU_LOAD_ADC_INJ
  uint32_t CurrentTimeStamp = DWT->CYCCNT;
  uint32_t EndTimeStamp = 0;
#endif
#if 0
  /* USER CODE END ADC1_2_IRQn 0 */
  HAL_ADC_IRQHandler(&hadc1);
  HAL_ADC_IRQHandler(&hadc2);
  /* USER CODE BEGIN ADC1_2_IRQn 1 */
#endif
  USER_HAL_ADC_IRQHandler( &hadc1 );
  USER_HAL_ADC_IRQHandler( &hadc2 );
#if MEASURE_CPU_LOAD_ADC_INJ
  EndTimeStamp =  DWT->CYCCNT;
  Max_ADC_Inj_Cnt = Max_ADC_Inj_Cnt > ( EndTimeStamp - CurrentTimeStamp ) ? Max_ADC_Inj_Cnt : ( EndTimeStamp - CurrentTimeStamp );
#endif
  /* USER CODE END ADC1_2_IRQn 1 */
}

/**
  * @brief This function handles FDCAN1 interrupt 0.
  */
void FDCAN1_IT0_IRQHandler(void)
{
  /* USER CODE BEGIN FDCAN1_IT0_IRQn 0 */
  ExtranetCANStation.ReadRxFIFO(&ExtranetCANStation,FDCAN_RX_FIFO0);
  /* USER CODE END FDCAN1_IT0_IRQn 0 */
  HAL_FDCAN_IRQHandler(&hfdcan1);
  /* USER CODE BEGIN FDCAN1_IT0_IRQn 1 */
  /* USER CODE END FDCAN1_IT0_IRQn 1 */
}

/**
  * @brief This function handles FDCAN1 interrupt 1.
  */
void FDCAN1_IT1_IRQHandler(void)
{
  /* USER CODE BEGIN FDCAN1_IT1_IRQn 0 */

  /* USER CODE END FDCAN1_IT1_IRQn 0 */
  HAL_FDCAN_IRQHandler(&hfdcan1);
  /* USER CODE BEGIN FDCAN1_IT1_IRQn 1 */

  /* USER CODE END FDCAN1_IT1_IRQn 1 */
}

/**
  * @brief This function handles USART2 global interrupt / USART2 wake-up interrupt through EXTI line 26.
  */
void USART2_IRQHandler(void)
{
  /* USER CODE BEGIN USART2_IRQn 0 */

  /* USER CODE END USART2_IRQn 0 */
  HAL_USART_IRQHandler(&husart2);
  /* USER CODE BEGIN USART2_IRQn 1 */

  /* USER CODE END USART2_IRQn 1 */
}

/**
  * @brief This function handles TIM2 global interrupt.
  */
void TIM2_IRQHandler(void)
{
  /* USER CODE BEGIN TIM2_IRQn 0 */
  drive_DoTotalTime();
  /* USER CODE END TIM2_IRQn 0 */
  HAL_TIM_IRQHandler(&htim2);
  /* USER CODE BEGIN TIM2_IRQn 1 */

  /* USER CODE END TIM2_IRQn 1 */
}

/**
  * @brief This function handles TIM5 global interrupt.
  */
void TIM5_IRQHandler(void)
{
  /* USER CODE BEGIN TIM5_IRQn 0 */

  /* USER CODE END TIM5_IRQn 0 */
  HAL_TIM_IRQHandler(&htim5);
  /* USER CODE BEGIN TIM5_IRQn 1 */

  /* USER CODE END TIM5_IRQn 1 */
}

/**
  * @brief This function handles TIM7 global interrupt, DAC2 and DAC4 channel underrun error interrupts.
  */
void TIM7_DAC_IRQHandler(void)
{
  /* USER CODE BEGIN TIM7_DAC_IRQn 0 */

#if MEASURE_CPU_LOAD || JUDGE_FUNCTION_DELAY
	uint32_t CurrentTimeStamp = DWT->CYCCNT;
	uint32_t EndTimeStamp = 0;
#endif
#if JUDGE_FUNCTION_DELAY
    uint32_t delta = CurrentTimeStamp - _100HzLoop_Judge_Delay.previousTimestamp;
    static uint8_t initial_ignore = 0;
    _100HzLoop_Judge_Delay.deltaCnt = delta;
    _100HzLoop_Judge_Delay.previousTimestamp = CurrentTimeStamp;
    _100HzLoop_Judge_Delay.Intervals_us = (float)delta / 170.0f;
    if ( initial_ignore > 10 )
    {
    	_100HzLoop_Judge_Delay.maxDelta = delta > _100HzLoop_Judge_Delay.maxDelta ? delta : _100HzLoop_Judge_Delay.maxDelta;
    	_100HzLoop_Judge_Delay.AveDelta = _100HzLoop_Judge_Delay.AveDelta - aveDelta_filter_coef * 100.0f* ( _100HzLoop_Judge_Delay.AveDelta - (float)delta );
        _100HzLoop_Judge_Delay.Max_Intervals_us = (float)_100HzLoop_Judge_Delay.maxDelta / 170.0f;
    	CurrentLoop_Judge_Delay.Max_Intervals_us = (float)CurrentLoop_Judge_Delay.maxDelta / 170.0f;
    	PLCLoop_Judge_Delay.Max_Intervals_us = (float)PLCLoop_Judge_Delay.maxDelta / 170.0f;
    	TIM20INT_Judge_Delay.Max_Intervals_us = (float)TIM20INT_Judge_Delay.maxDelta / 170.0f;
        _100HzLoop_Judge_Delay.Ave_Intervals_us = (float)_100HzLoop_Judge_Delay.AveDelta / 170.0f;
    	CurrentLoop_Judge_Delay.Ave_Intervals_us = (float)CurrentLoop_Judge_Delay.AveDelta / 170.0f;
    	PLCLoop_Judge_Delay.Ave_Intervals_us = (float)PLCLoop_Judge_Delay.AveDelta / 170.0f;
    	TIM20INT_Judge_Delay.Ave_Intervals_us = (float)TIM20INT_Judge_Delay.AveDelta / 170.0f;
    }
    else
    {
    	initial_ignore++;
    }
#endif

#if 0
  /* USER CODE END TIM7_DAC_IRQn 0 */
  HAL_TIM_IRQHandler(&htim7);
  /* USER CODE BEGIN TIM7_DAC_IRQn 1 */
#endif
  USER_HAL_TIM_7_IRQHandler(&htim7);

#if MEASURE_CPU_LOAD
    EndTimeStamp =  DWT->CYCCNT;
    Max_100Hz_Cnt = Max_100Hz_Cnt > ( EndTimeStamp - CurrentTimeStamp ) ? Max_100Hz_Cnt : ( EndTimeStamp - CurrentTimeStamp );
#endif
#if MEASURE_CPU_LOAD
    Max_100Hz_Load_pct = ((float)Max_100Hz_Cnt / 17000.0f );// Max_100Hz_Cnt / 170000000.0f * 100.0f * 100.0f
    Max_PLCLoop_Load_pct = ((float)Max_PLCLoop_Cnt / 1700.0f );// Max_CurrentLoop_Cnt / 170000000.0f * 100.0f * 1000.0f
    Max_CurrentLoop_Load_pct = ((float)Max_CurrentLoop_Cnt / 170.0f );// Max_CurrentLoop_Cnt / 170000000.0f * 100.0f * 10000.0f
    Max_ADC_Inj_Load_pct = ((float)Max_ADC_Inj_Cnt / 170.0f );// Max_CurrentLoop_Cnt / 170000000.0f * 100.0f * 10000.0f

    Ave_100Hz_Cnt = Ave_100Hz_Cnt - aveLoad_filter_coef * 100.0f* ( Ave_100Hz_Cnt - (float)( EndTimeStamp - CurrentTimeStamp ) );
    Ave_100Hz_Load_pct = ((float)Ave_100Hz_Cnt / 17000.0f );// Max_100Hz_Cnt / 170000000.0f * 100.0f * 100.0f
    Ave_PLCLoop_Load_pct = ((float)Ave_PLCLoop_Cnt / 1700.0f );// Max_CurrentLoop_Cnt / 170000000.0f * 100.0f * 1000.0f
    Ave_CurrentLoop_Load_pct = ((float)Ave_CurrentLoop_Cnt / 170.0f );// Max_CurrentLoop_Cnt / 170000000.0f * 100.0f * 10000.0f
#endif

  /* USER CODE END TIM7_DAC_IRQn 1 */
}

/**
  * @brief This function handles TIM20 break interrupt.
  */
void TIM20_BRK_IRQHandler(void)
{
  /* USER CODE BEGIN TIM20_BRK_IRQn 0 */

  /* USER CODE END TIM20_BRK_IRQn 0 */
//  HAL_TIM_IRQHandler(&htim20);
  /* USER CODE BEGIN TIM20_BRK_IRQn 1 */
  drive_DoHWOCPIRQ();
  HAL_NVIC_DisableIRQ(TIM20_BRK_IRQn);
  /* USER CODE END TIM20_BRK_IRQn 1 */
}

/**
  * @brief This function handles TIM20 update interrupt.
  */
void TIM20_UP_IRQHandler(void)
{
  /* USER CODE BEGIN TIM20_UP_IRQn 0 */

#if  JUDGE_FUNCTION_DELAY
uint32_t CurrentTimeStamp = DWT->CYCCNT;
uint32_t EndTimeStamp = 0;
uint32_t delta = CurrentTimeStamp - TIM20INT_Judge_Delay.previousTimestamp;
static uint8_t initial_ignore = 0;
TIM20INT_Judge_Delay.deltaCnt = delta;
TIM20INT_Judge_Delay.previousTimestamp = CurrentTimeStamp;
TIM20INT_Judge_Delay.Intervals_us = (float)delta / 170.0f;
if ( initial_ignore > 10 )
{
	TIM20INT_Judge_Delay.maxDelta = delta > TIM20INT_Judge_Delay.maxDelta ? delta : TIM20INT_Judge_Delay.maxDelta;
	TIM20INT_Judge_Delay.AveDelta = TIM20INT_Judge_Delay.AveDelta - aveDelta_filter_coef * ( TIM20INT_Judge_Delay.AveDelta - (float)delta );
}
else
{
	initial_ignore++;
}
#endif
  CPUCounter.PWMCounter++;
#if 0
  /* USER CODE END TIM20_UP_IRQn 0 */
  HAL_TIM_IRQHandler(&htim20);
  /* USER CODE BEGIN TIM20_UP_IRQn 1 */
#endif
  USER_HAL_TIM_IRQHandler(&htim20);
  AdcStation1.AdcInjGroup = 0;		// Clear ADC Flag

  /* USER CODE END TIM20_UP_IRQn 1 */
}

/**
  * @brief This function handles FDCAN2 interrupt 0.
  */
void FDCAN2_IT0_IRQHandler(void)
{
  /* USER CODE BEGIN FDCAN2_IT0_IRQn 0 */
  ExtranetCANStation.ReadRxFIFO(&ExtranetCANStation,FDCAN_RX_FIFO0);
  /* USER CODE END FDCAN2_IT0_IRQn 0 */
  HAL_FDCAN_IRQHandler(&hfdcan2);
  /* USER CODE BEGIN FDCAN2_IT0_IRQn 1 */

  /* USER CODE END FDCAN2_IT0_IRQn 1 */
}

/**
  * @brief This function handles FDCAN2 interrupt 1.
  */
void FDCAN2_IT1_IRQHandler(void)
{
  /* USER CODE BEGIN FDCAN2_IT1_IRQn 0 */

  /* USER CODE END FDCAN2_IT1_IRQn 0 */
  HAL_FDCAN_IRQHandler(&hfdcan2);
  /* USER CODE BEGIN FDCAN2_IT1_IRQn 1 */

  /* USER CODE END FDCAN2_IT1_IRQn 1 */
}

/* USER CODE BEGIN 1 */
void HAL_ADCEx_InjectedConvCpltCallback(ADC_HandleTypeDef* hadc)
{
//	AdcStation1.ReadInjectionGroupValue( &AdcStation1, hadc );
//	AdcStation1.MarkInjectionGroupReadFlag( &AdcStation1, hadc );
	ADC_HANDLE_INJECTION_GROUP_MACRO( (&AdcStation1), hadc )

	if( AdcStation1.AdcInjGroup == AdcStation1.AdcInjGroupFlag )
	{
#if USE_DIGITAL_FOIL_TO_MEASURE_CPU_LOAD_ADCINJ_CURRENT
        HAL_GPIO_WritePin( GPIOE, FOIL_DI3_Pin, GPIO_PIN_SET );
#endif
#if MEASURE_CPU_LOAD_CURRENTLOOP || JUDGE_FUNCTION_DELAY
        uint32_t CurrentTimeStamp = DWT->CYCCNT;
        uint32_t EndTimeStamp = 0;
#endif
#if  JUDGE_FUNCTION_DELAY
        uint32_t delta = CurrentTimeStamp - CurrentLoop_Judge_Delay.previousTimestamp;
        static uint8_t initial_ignore = 0;
        CurrentLoop_Judge_Delay.deltaCnt = delta;
        CurrentLoop_Judge_Delay.previousTimestamp = CurrentTimeStamp;
        CurrentLoop_Judge_Delay.Intervals_us = (float)delta / 170.0f;
        if ( initial_ignore > 10 )
        {
        	CurrentLoop_Judge_Delay.maxDelta = delta > CurrentLoop_Judge_Delay.maxDelta ? delta : CurrentLoop_Judge_Delay.maxDelta;
        	CurrentLoop_Judge_Delay.AveDelta = CurrentLoop_Judge_Delay.AveDelta - aveDelta_filter_coef * ( CurrentLoop_Judge_Delay.AveDelta - (float)delta );
        }
        else
        {
        	initial_ignore++;
        }
#endif
		CPUCounter.ADCCounter++;
		drive_DoCurrentLoop();
#if MEASURE_CPU_LOAD_CURRENTLOOP
        EndTimeStamp =  DWT->CYCCNT;
        Max_CurrentLoop_Cnt = Max_CurrentLoop_Cnt > ( EndTimeStamp - CurrentTimeStamp ) ? Max_CurrentLoop_Cnt : ( EndTimeStamp - CurrentTimeStamp );
        Ave_CurrentLoop_Cnt = Ave_CurrentLoop_Cnt - aveLoad_filter_coef * ( Ave_CurrentLoop_Cnt -  (float)( EndTimeStamp - CurrentTimeStamp ) );
#endif
	}
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{

	AdcStation1.MarkRegularGroupReadFlag( &AdcStation1, hadc );

	if( AdcStation1.AdcRegGroup == AdcStation1.AdcRegGroupFlag )
	{
#if MEASURE_CPU_LOAD_PLCLOOP || JUDGE_FUNCTION_DELAY
    uint32_t StartTimestame =  DWT->CYCCNT;
    uint32_t EndTimeStamp =  0;
#endif
#if JUDGE_FUNCTION_DELAY
    uint32_t delta = StartTimestame - PLCLoop_Judge_Delay.previousTimestamp;
    static uint8_t initial_ignore = 0;
    PLCLoop_Judge_Delay.deltaCnt = delta;
    PLCLoop_Judge_Delay.previousTimestamp = StartTimestame;
    PLCLoop_Judge_Delay.Intervals_us = (float)delta / 170.0f;
    if ( initial_ignore > 10 )
    {
	    PLCLoop_Judge_Delay.maxDelta = delta > PLCLoop_Judge_Delay.maxDelta ? delta : PLCLoop_Judge_Delay.maxDelta;
	    PLCLoop_Judge_Delay.AveDelta = PLCLoop_Judge_Delay.AveDelta - aveDelta_filter_coef * 10.0f * ( PLCLoop_Judge_Delay.AveDelta - (float)delta );

    }
    else
    {
	    initial_ignore++;
    }
#endif
		CPUCounter.PLCLoopCounter++;
		drive_DoPLCLoop();
		AdcStation1.AdcRegGroup = 0;
#if MEASURE_CPU_LOAD_PLCLOOP
    EndTimeStamp =  DWT->CYCCNT;
    Max_PLCLoop_Cnt = Max_PLCLoop_Cnt > ( EndTimeStamp - StartTimestame ) ? Max_PLCLoop_Cnt : ( EndTimeStamp - StartTimestame );
    Ave_PLCLoop_Cnt = Ave_PLCLoop_Cnt - aveLoad_filter_coef * 10.0f* ( Ave_PLCLoop_Cnt - (float)( EndTimeStamp - StartTimestame ) );
#endif
	}

}
#if USE_PWM_RC_FUNCTION == (USE_FUNCTION & EVT)
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
	drive_DoPwmRcCatch();
}
#endif

void HAL_TIMEx_EncoderIndexCallback(TIM_HandleTypeDef *htim)
{
	__NOP();
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if( htim->Instance == TIM7 )
	{
		CPUCounter.Loop100HzCounter++;
		drive_Do100HzLoop();

		if( CPUCounter.Loop100HzCounter % 10 == 0 )
		{
		  CPUCounter.Loop10HzCounter++;
		  drive_Do10HzLoop();

		  if( CPUCounter.Loop10HzCounter % 10 == 0 )
		  {
			  drive_Do1HzLoop();
		  }
		}
	}
}

void USER_HAL_TIM_7_IRQHandler( TIM_HandleTypeDef *htim )
{
	__HAL_TIM_CLEAR_IT(htim, TIM_IT_UPDATE);
	HAL_TIM_PeriodElapsedCallback(htim);
}

void USER_HAL_TIM_IRQHandler( TIM_HandleTypeDef *htim )
{
	__HAL_TIM_CLEAR_IT(htim, TIM_IT_CC1);
	htim->Channel = HAL_TIM_ACTIVE_CHANNEL_1;
	htim->Channel = HAL_TIM_ACTIVE_CHANNEL_CLEARED;
	__HAL_TIM_CLEAR_IT(htim, TIM_IT_CC2);
	htim->Channel = HAL_TIM_ACTIVE_CHANNEL_2;
	htim->Channel = HAL_TIM_ACTIVE_CHANNEL_CLEARED;
	__HAL_TIM_CLEAR_IT(htim, TIM_IT_CC4);
	htim->Channel = HAL_TIM_ACTIVE_CHANNEL_4;
	htim->Channel = HAL_TIM_ACTIVE_CHANNEL_CLEARED;
	__HAL_TIM_CLEAR_IT(htim, TIM_IT_UPDATE);
}

void USER_HAL_ADC_IRQHandler(ADC_HandleTypeDef *hadc)
{
	uint32_t tmp_isr = hadc->Instance->ISR;
	uint32_t tmp_ier = hadc->Instance->IER;
	uint32_t tmp_cfgr;
	#if defined(ADC_MULTIMODE_SUPPORT)
	const ADC_TypeDef *tmpADC_Master;
	uint32_t tmp_multimode_config = LL_ADC_GetMultimode(__LL_ADC_COMMON_INSTANCE(hadc->Instance));
	#endif
	if ((((tmp_isr & ADC_FLAG_JEOC) == ADC_FLAG_JEOC) && ((tmp_ier & ADC_IT_JEOC) == ADC_IT_JEOC)) ||
	  (((tmp_isr & ADC_FLAG_JEOS) == ADC_FLAG_JEOS) && ((tmp_ier & ADC_IT_JEOS) == ADC_IT_JEOS)))
	{
		/* Update state machine on conversion status if not in error state */
		if ((hadc->State & HAL_ADC_STATE_ERROR_INTERNAL) == 0UL)
		{
		  /* Set ADC state */
		  SET_BIT(hadc->State, HAL_ADC_STATE_INJ_EOC);
		}

		/* Retrieve ADC configuration */
//		tmp_adc_inj_is_trigger_source_sw_start = LL_ADC_INJ_IsTriggerSourceSWStart(hadc->Instance);
//		tmp_adc_reg_is_trigger_source_sw_start = LL_ADC_REG_IsTriggerSourceSWStart(hadc->Instance);
		/* Get relevant register CFGR in ADC instance of ADC master or slave  */
		/* in function of multimode state (for devices with multimode         */
		/* available).                                                        */
		if ((__LL_ADC_MULTI_INSTANCE_MASTER(hadc->Instance) == hadc->Instance)
			|| (tmp_multimode_config == LL_ADC_MULTI_INDEPENDENT)
			|| (tmp_multimode_config == LL_ADC_MULTI_DUAL_REG_SIMULT)
			|| (tmp_multimode_config == LL_ADC_MULTI_DUAL_REG_INTERL)
		   )
		{
		  tmp_cfgr = READ_REG(hadc->Instance->CFGR);
		}
		else
		{
		  tmpADC_Master = __LL_ADC_MULTI_INSTANCE_MASTER(hadc->Instance);
		  tmp_cfgr = READ_REG(tmpADC_Master->CFGR);
		}

		/* Injected Conversion complete callback */
		/* Note:  HAL_ADCEx_InjectedConvCpltCallback can resort to
				  if( __HAL_ADC_GET_FLAG(&hadc, ADC_FLAG_JEOS)) or
				  if( __HAL_ADC_GET_FLAG(&hadc, ADC_FLAG_JEOC)) to determine whether
				  interruption has been triggered by end of conversion or end of
				  sequence.    */
		HAL_ADCEx_InjectedConvCpltCallback(hadc);

		/* Clear injected group conversion flag */
		__HAL_ADC_CLEAR_FLAG(hadc, ADC_FLAG_JEOC | ADC_FLAG_JEOS);
	}
}

/* USER CODE END 1 */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
