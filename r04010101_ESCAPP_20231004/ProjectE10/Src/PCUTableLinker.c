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
				{ADC_1,			ADC_ENABLE,			ADC_ENABLE,		ADC_DISABLE		},
				{ADC_2,			ADC_ENABLE,			ADC_ENABLE,		ADC_ENABLE		},
				{ADC_3,			ADC_ENABLE,			ADC_ENABLE,		ADC_ENABLE		},
				{ADC_4,			ADC_ENABLE,		    ADC_ENABLE,	    ADC_ENABLE		},
				{ADC_5,			ADC_ENABLE,		    ADC_ENABLE,	    ADC_DISABLE		}
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
				{HW_ID2,	    ADC_DISABLE, 	 	ADC_ENABLE, 	ADC_1,				RANK_4,		    DEFAULT_GAIN	},
				{ES5V_FB,   	ADC_DISABLE, 	 	ADC_ENABLE, 	ADC_1,				RANK_2,			E5V_GAIN		},
				{HW_ID1,    	ADC_DISABLE, 	 	ADC_ENABLE, 	ADC_1,				RANK_3,			DEFAULT_GAIN	},
				{E5V_FB,    	ADC_DISABLE, 	 	ADC_ENABLE, 	ADC_2,				RANK_1,			E5V_GAIN    	},
				{EA5V_FB,   	ADC_DISABLE, 	 	ADC_ENABLE, 	ADC_3,		        RANK_6,	        E5V_GAIN    	},
				{PREC_FB,   	ADC_DISABLE, 	 	ADC_ENABLE, 	ADC_3,		        RANK_4,	        DC_VOLT_GAIN	},
				{S13V8,     	ADC_DISABLE, 	 	ADC_ENABLE, 	ADC_3,		        RANK_5,	        S13V8_GAIN    	},
				{ACC_FB2,   	ADC_DISABLE, 	 	ADC_ENABLE, 	ADC_4,		        RANK_1,	        PEDAL_GAIN	    },
				{ACC_FB1,   	ADC_DISABLE, 	 	ADC_ENABLE, 	ADC_5,        		RANK_1,        	PEDAL_GAIN	    },
				{REG_REVD_9,	ADC_DISABLE, 	 	ADC_DISABLE, 	GROUP_DISABLE,		RANK_DISABLE,	DEFAULT_GAIN	},
				{REG_REVD_10,	ADC_DISABLE, 	 	ADC_DISABLE, 	GROUP_DISABLE,		RANK_DISABLE,	DEFAULT_GAIN	},
				{REG_REVD_11,	ADC_DISABLE, 	 	ADC_DISABLE, 	GROUP_DISABLE,		RANK_DISABLE,	DEFAULT_GAIN	},
				{REG_REVD_12,	ADC_DISABLE, 	 	ADC_DISABLE, 	GROUP_DISABLE,		RANK_DISABLE,	DEFAULT_GAIN	}
		},

		.AdcThermoTable =
		{
				// PCU Thermal
				//Name			ZeroFlagEnable,		TableIdx,			Channel Enable,		Adc Group,			Adc Rank
				{MOS_NTC_1,		ADC_DISABLE,		GWX_LS103H20,	ADC_ENABLE,			ADC_3,				RANK_2			},	// MOS NTC Center, use the same table as Motor NTC at P0 phase
				{MOS_NTC_2,		ADC_DISABLE,		GWX_LS103H20,	ADC_ENABLE,			ADC_3,				RANK_3			},	// MOS NTC Side, use the same table as Motor NTC at P0 phase
				{CAP_NTC,		ADC_DISABLE,		GWX_LS103H12,		ADC_ENABLE,			ADC_3,				RANK_1			},	// CAP NTC
				{PCU_NTC_3,		ADC_DISABLE,		NO_TABLE,			ADC_DISABLE,		GROUP_DISABLE,		RANK_DISABLE	},
				{PCU_NTC_4,		ADC_DISABLE,		NO_TABLE,			ADC_DISABLE,		GROUP_DISABLE,		RANK_DISABLE	},
				// Axis - 0 Thermal
				//Name			ZeroFlagEnable,	 	Table Idx,		Channel Enable, Adc Group,			Adc Rank
				{MOTOR_NTC_0_A0,ADC_DISABLE,		GWX_LS103H20,	ADC_ENABLE,		ADC_1,				RANK_1	},
				{MOTOR_NTC_1_A0,ADC_DISABLE,		GWX_LS103H20,	ADC_ENABLE,	    ADC_2,		        RANK_2	},
				{MOTOR_NTC_2_A0,ADC_DISABLE,		GWX_LS103H20,	ADC_ENABLE,	    ADC_2,		        RANK_3	},
				// Axis - 1 Thermal
				//Name			ZeroFlagEnable,	 	Table Idx,		Channel Enable, Adc Group,			Adc Rank
				{MOTOR_NTC_0_A1,ADC_DISABLE,		NO_TABLE,		ADC_DISABLE,	GROUP_DISABLE,		RANK_DISABLE	},
				{MOTOR_NTC_1_A1,ADC_DISABLE,		NO_TABLE,		ADC_DISABLE,	GROUP_DISABLE,		RANK_DISABLE	},
				{MOTOR_NTC_2_A1,ADC_DISABLE,		NO_TABLE,		ADC_DISABLE,	GROUP_DISABLE,		RANK_DISABLE	}
		},

		.AdcCurveFittingTable =
		{
				// A6,			A5,				A4,				A3,				A2,				A1,				A0
				{ 226.34901f, 	410.00309f, 	-321.93657f, 	46.95519f, 		7.8639097f,		-1.1499592f, 	-0.29832702f},		//	NTC On board number for MOS - NTCG163JX103DT1S
				{ 286.91117f, 	256.03905f, 	-208.99333f, 	31.554341f,		3.6156109f,		-1.5739555f,	0.12358828f	},		//	NTC Motor Winding number - GWX_LS103H20
				{ 243.11277f,	-123.32722f,	8.3642276f,		2.0067563f,		0.26944288f,	-0.35931768f,	0.056648032f}		//  NTC On board for CAP - GWX_LS103H12
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
