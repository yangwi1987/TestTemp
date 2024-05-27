	/*
 * PCUTableLinker.c
 *
 *  Created on: 2020年5月5日
 *      Author: Mike.Wen.SFW
 */
#if E10
#include "PCUTableLinker.h"

#define THRE_MAX				0xFFFF

__attribute__((__section__(".PcuBin"),used)) const PCU_Table_t_Linker PCUTable =
{
		.Version = {01, 0x03, 1, 1}, /* Version numbers are undefined now. If version number are all zero, const table cannot work in bootloader. */
		.NumOfHeader = 0,
		.CheckSum = 0,

		.AdcSetupTab =
		{
				// Group,		Group Enable,		DMAEna,			InjectedMode
				{ADC_1,			ADC_ENABLE,			ADC_DISABLE,		ADC_DISABLE		},
				{ADC_2,			ADC_ENABLE,			ADC_DISABLE,		ADC_ENABLE		},
				{ADC_3,			ADC_ENABLE,			ADC_DISABLE,		ADC_ENABLE		},
				{ADC_4,			ADC_ENABLE,		    ADC_DISABLE,	    ADC_ENABLE		},
				{ADC_5,			ADC_ENABLE,		    ADC_DISABLE,	    ADC_DISABLE		}
		},

		.AdcInjectionGroupTable =
		{
				//	Axis 1 Current
				//Name			ZeroFlagEnable,	 	Channel Enable, Adc Group,		Adc Rank, 			GainValue,
				{ISE_U_A0,		ADC_DISABLE, 	 	ADC_ENABLE, 	ADC_4,			RANK_1,	  			AC_CURR_GAIN	},
				{ISE_V_A0,		ADC_DISABLE, 	 	ADC_ENABLE, 	ADC_3,			RANK_1,	  			AC_CURR_GAIN	},
				{ISE_W_A0,		ADC_DISABLE, 	 	ADC_ENABLE, 	ADC_2,			RANK_1,   			AC_CURR_GAIN    },
				//	Axis 2 Current
				//Name			ZeroFlagEnable,	 	Channel Enable, Adc Group,		Adc Rank,			GainValue
				{ISE_U_A1,		ADC_DISABLE, 	 	ADC_DISABLE, 	GROUP_DISABLE,	RANK_DISABLE, 		DEFAULT_GAIN	},
				{ISE_V_A1,		ADC_DISABLE, 		ADC_DISABLE, 	GROUP_DISABLE,	RANK_DISABLE, 		DEFAULT_GAIN	},
				{ISE_W_A1,		ADC_DISABLE, 	 	ADC_DISABLE, 	GROUP_DISABLE,	RANK_DISABLE, 		DEFAULT_GAIN	},

				//Name			ZeroFlagEnable,	 	Channel Enable, Adc Group,		Adc Rank,			GainValue
				{BAT_VDC,		ADC_DISABLE, 	 	ADC_ENABLE, 	ADC_3,			RANK_2,				DC_VOLT_GAIN	},
				{INJ_REVD_8,	ADC_DISABLE, 	 	ADC_DISABLE, 	GROUP_DISABLE,	RANK_DISABLE,		DEFAULT_GAIN	},
				{INJ_REVD_9,	ADC_DISABLE, 	 	ADC_DISABLE, 	GROUP_DISABLE,	RANK_DISABLE,		DEFAULT_GAIN	}
		},

		.AdcRegularGroupTable =
		{
				//Name			ZeroFlagEnable,	 	Channel Enable, Adc Group,			Adc Rank,		GainValue
				{REG_REVD_0,    ADC_DISABLE, 	 	ADC_DISABLE, 	GROUP_DISABLE,		RANK_DISABLE,	DEFAULT_GAIN	},
				{REG_REVD_1,  	ADC_DISABLE, 	 	ADC_DISABLE, 	GROUP_DISABLE,		RANK_DISABLE,	DEFAULT_GAIN	},
				{REG_REVD_2,  	ADC_DISABLE, 	 	ADC_DISABLE, 	GROUP_DISABLE,		RANK_DISABLE,	DEFAULT_GAIN	},
				{REG_REVD_3,  	ADC_DISABLE, 	 	ADC_DISABLE, 	GROUP_DISABLE,		RANK_DISABLE,	DEFAULT_GAIN	},
				{REG_REVD_4,  	ADC_DISABLE, 	 	ADC_DISABLE, 	GROUP_DISABLE,		RANK_DISABLE,	DEFAULT_GAIN	},
				{REG_REVD_5,  	ADC_DISABLE, 	 	ADC_DISABLE, 	GROUP_DISABLE,      RANK_DISABLE,	DEFAULT_GAIN	},
				{REG_REVD_6,  	ADC_DISABLE, 	 	ADC_DISABLE, 	GROUP_DISABLE,		RANK_DISABLE,	DEFAULT_GAIN  	},
				{REG_REVD_7,  	ADC_DISABLE, 	 	ADC_DISABLE, 	GROUP_DISABLE,		RANK_DISABLE,	DEFAULT_GAIN    },
				{REG_REVD_8,  	ADC_DISABLE, 	 	ADC_DISABLE, 	GROUP_DISABLE,      RANK_DISABLE,   DEFAULT_GAIN    },
				{REG_REVD_9,	ADC_DISABLE, 	 	ADC_DISABLE, 	GROUP_DISABLE,		RANK_DISABLE,	DEFAULT_GAIN	},
				{REG_REVD_10,	ADC_DISABLE, 	 	ADC_DISABLE, 	GROUP_DISABLE,		RANK_DISABLE,	DEFAULT_GAIN	},
				{REG_REVD_11,	ADC_DISABLE, 	 	ADC_DISABLE, 	GROUP_DISABLE,		RANK_DISABLE,	DEFAULT_GAIN	},
				{REG_REVD_12,	ADC_DISABLE, 	 	ADC_DISABLE, 	GROUP_DISABLE,		RANK_DISABLE,	DEFAULT_GAIN	}
		},

		.AdcThermoTable =
		{
				// PCU Thermal
				//Name			ZeroFlagEnable,		TableIdx,			Channel Enable,		Adc Group,			Adc Rank
				{MOS_NTC_1,		ADC_DISABLE,		NO_TABLE,	ADC_DISABLE,			GROUP_DISABLE,				RANK_DISABLE			},
				{MOS_NTC_2,		ADC_DISABLE,		NO_TABLE,	ADC_DISABLE,			GROUP_DISABLE,				RANK_DISABLE			},
				{CAP_NTC,		ADC_DISABLE,		NO_TABLE,		ADC_DISABLE,			GROUP_DISABLE,			RANK_DISABLE			},
				{PCU_NTC_3,		ADC_DISABLE,		NO_TABLE,			ADC_DISABLE,		GROUP_DISABLE,		RANK_DISABLE	},
				{PCU_NTC_4,		ADC_DISABLE,		NO_TABLE,			ADC_DISABLE,		GROUP_DISABLE,		RANK_DISABLE	},
				// Axis - 0 Thermal
				//Name			ZeroFlagEnable,	 	Table Idx,		Channel Enable, Adc Group,			Adc Rank
				{MOTOR_NTC_0_A0,ADC_DISABLE,		NO_TABLE,	ADC_DISABLE,		GROUP_DISABLE,				RANK_DISABLE	},
				{MOTOR_NTC_1_A0,ADC_DISABLE,		NO_TABLE,	ADC_DISABLE,	    GROUP_DISABLE,		        RANK_DISABLE	},
				{MOTOR_NTC_2_A0,ADC_DISABLE,		NO_TABLE,	ADC_DISABLE,	    GROUP_DISABLE,		        RANK_DISABLE	},
				// Axis - 1 Thermal
				//Name			ZeroFlagEnable,	 	Table Idx,		Channel Enable, Adc Group,			Adc Rank
				{MOTOR_NTC_0_A1,ADC_DISABLE,		NO_TABLE,		ADC_DISABLE,	GROUP_DISABLE,		RANK_DISABLE	},
				{MOTOR_NTC_1_A1,ADC_DISABLE,		NO_TABLE,		ADC_DISABLE,	GROUP_DISABLE,		RANK_DISABLE	},
				{MOTOR_NTC_2_A1,ADC_DISABLE,		NO_TABLE,		ADC_DISABLE,	GROUP_DISABLE,		RANK_DISABLE	}
		},

		.PwmStartUpTable_Axis1 =
		{
			//Name,			GroupEnable,	Group,		Channel,		Reserved
			{ CH_PWM_UP, 	PWM_ENABLE, 	TIM_8,		TIM_CH4,		PWM_NONE	},
			{ CH_PWM_UN, 	PWM_ENABLE,		TIM_8,		TIM_CH4, 		PWM_NONE	},
			{ CH_PWM_VP, 	PWM_ENABLE,		TIM_8,		TIM_CH1, 		PWM_NONE	},
			{ CH_PWM_VN, 	PWM_ENABLE,		TIM_8,		TIM_CH1, 		PWM_NONE	},
			{ CH_PWM_WP, 	PWM_ENABLE,		TIM_8,		TIM_CH3, 		PWM_NONE	},
			{ CH_PWM_WN, 	PWM_ENABLE,		TIM_8,		TIM_CH3, 		PWM_NONE	}
		},

		.Pwm_Tim_Initializer_Axis1 =
		{
			TIM8,								\
			&htim8,								\
			TIM_CHANNEL_4,/*ChannelU*/			\
			TIM_CHANNEL_1,/*ChannelV*/			\
			TIM_CHANNEL_3,/*ChannelW*/			\
			1,	/* PreScale =1, note PSC = 1-1*/\
			TIM_COUNTERMODE_CENTERALIGNED1,		\
			INITIAL_PWM_PERIOD,	/*ARR = 8500-1*/\
			TIM_CLOCKDIVISION_DIV1,	/*CKD = 00*/\
			INITIAL_REPET_COUNTER,				\
			TIM_AUTORELOAD_PRELOAD_ENABLE,		\
			TIM_CLOCKSOURCE_INTERNAL,			\
			TIM_TRGO_UPDATE,					\
			TIM_TRGO2_RESET,					\
			TIM_MASTERSLAVEMODE_DISABLE,		\
			TIM_BREAKINPUTSOURCE_BKIN,			\
			TIM_BREAKINPUTSOURCE_ENABLE,		\
			TIM_BREAKINPUTSOURCE_POLARITY_LOW,	/*BKINP = 1 (low active)*/\
			TIM_OCMODE_PWM2,	/*no use*/		\
			(uint16_t)(((float)SYSTEM_CLK_FREQ/(float)INITIAL_CURRENT_LOOP_FREQ/2.0f) + 0.5f) + 1, /*	InitDutyCntChU	*/		\
			TIM_OCPOLARITY_LOW,	/*CC1P=1 => low active*/ \
			TIM_OCNPOLARITY_LOW, /*CC1NP=1 => low active*/ \
			TIM_OCFAST_DISABLE,					\
			TIM_OCIDLESTATE_RESET,				\
			TIM_OCNIDLESTATE_RESET,				\
			(uint16_t)(((float)SYSTEM_CLK_FREQ/(float)INITIAL_CURRENT_LOOP_FREQ/2.0f) + 0.5f) + 1, /*	InitDutyCntChV	*/		\
			(uint16_t)(((float)SYSTEM_CLK_FREQ/(float)INITIAL_CURRENT_LOOP_FREQ/2.0f) + 0.5f) + 1, /*	InitDutyCntChW	*/		\
			TIM_OSSR_DISABLE,					\
			TIM_OSSI_DISABLE,					\
			TIM_LOCKLEVEL_OFF,					\
			203,	/*DTG 203=>(11+32)x47ns*/	\
			TIM_BREAK_ENABLE,					\
			TIM_BREAKPOLARITY_HIGH, /*BKP = 1*/	\
			0, /*	BreakFilter			*/		\
			TIM_BREAK_AFMODE_INPUT,				\
			TIM_BREAK2_DISABLE,					\
			TIM_BREAK2POLARITY_HIGH,			\
			0, /*	Break2Filter		*/		\
			TIM_BREAK_AFMODE_INPUT,				\
			TIM_AUTOMATICOUTPUT_DISABLE,		\
			5	/*MinimumTime*/	/*todo check with EE*/\
		},

		.CheckWord = 0x4321
};
#endif
