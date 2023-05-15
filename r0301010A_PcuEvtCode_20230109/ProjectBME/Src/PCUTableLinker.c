	/*
 * PCUTableLinker.c
 *
 *  Created on: 2020年5月5日
 *      Author: Mike.Wen.SFW
 */
#if BME
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
				{ADC_1,			ADC_ENABLE,			ADC_ENABLE,		ADC_ENABLE		},
				{ADC_2,			ADC_ENABLE,			ADC_ENABLE,		ADC_ENABLE		},
				{ADC_3,			ADC_ENABLE,			ADC_ENABLE,		ADC_DISABLE		},
				{ADC_4,			ADC_DISABLE,		ADC_DISABLE,	ADC_DISABLE		},
				{ADC_5,			ADC_DISABLE,		ADC_DISABLE,	ADC_DISABLE		}
		},

		.AdcInjectionGroupTable =
		{
				//	Axis 1 Current
				//Name			ZeroFlagEnable,	 	Channel Enable, Adc Group,		Adc Rank, 			GainValue,
				{ISE_U_A0,		ADC_DISABLE, 	 	ADC_ENABLE, 	ADC_1,			RANK_1,	  			AC_CURR_GAIN	},
				{ISE_V_A0,		ADC_DISABLE, 	 	ADC_ENABLE, 	ADC_2,			RANK_1,	  			AC_CURR_GAIN	},
				{ISE_W_A0,		ADC_DISABLE, 	 	ADC_ENABLE, 	ADC_1,			RANK_2,   			AC_CURR_GAIN    },
				//	Axis 2 Current
				//Name			ZeroFlagEnable,	 	Channel Enable, Adc Group,		Adc Rank,			GainValue
				{ISE_U_A1,		ADC_DISABLE, 	 	ADC_DISABLE, 	GROUP_DISABLE,	RANK_DISABLE, 		DEFAULT_GAIN	},
				{ISE_V_A1,		ADC_DISABLE, 		ADC_DISABLE, 	GROUP_DISABLE,	RANK_DISABLE, 		DEFAULT_GAIN	},
				{ISE_W_A1,		ADC_DISABLE, 	 	ADC_DISABLE, 	GROUP_DISABLE,	RANK_DISABLE, 		DEFAULT_GAIN	},

				//Name			ZeroFlagEnable,	 	Channel Enable, Adc Group,		Adc Rank,			GainValue
				{BAT_VDC,		ADC_DISABLE, 	 	ADC_ENABLE, 	ADC_1,			RANK_3,				DC_VOLT_GAIN	},
				{INJ_REVD_8,	ADC_DISABLE, 	 	ADC_DISABLE, 	GROUP_DISABLE,	RANK_DISABLE,		DEFAULT_GAIN	},
				{INJ_REVD_9,	ADC_DISABLE, 	 	ADC_DISABLE, 	GROUP_DISABLE,	RANK_DISABLE,		DEFAULT_GAIN	}
		},

		.AdcRegularGroupTable =
		{
				//Name			ZeroFlagEnable,	 	Channel Enable, Adc Group,			Adc Rank,		GainValue
				{FOIL_AD,		ADC_DISABLE, 	 	ADC_ENABLE, 	ADC_3,				RANK_3,			FOIL_GAIN		},
				{P13V_AD,		ADC_DISABLE, 	 	ADC_ENABLE, 	ADC_1,				RANK_1,			V13V_GAIN		},
				{HW_ID1,		ADC_DISABLE, 	 	ADC_ENABLE, 	ADC_3,				RANK_1,			DEFAULT_GAIN	},
				{HW_ID2,		ADC_DISABLE, 	 	ADC_ENABLE, 	ADC_2,				RANK_4,			DEFAULT_GAIN	},
				{REG_REVD_4,	ADC_DISABLE, 	 	ADC_DISABLE, 	GROUP_DISABLE,		RANK_DISABLE,	DEFAULT_GAIN	},
				{REG_REVD_5,	ADC_DISABLE, 	 	ADC_DISABLE, 	GROUP_DISABLE,		RANK_DISABLE,	DEFAULT_GAIN	},
				{REG_REVD_6,	ADC_DISABLE, 	 	ADC_DISABLE, 	GROUP_DISABLE,		RANK_DISABLE,	DEFAULT_GAIN	},
				{REG_REVD_7,	ADC_DISABLE, 	 	ADC_DISABLE, 	GROUP_DISABLE,		RANK_DISABLE,	DEFAULT_GAIN	},
				{REG_REVD_8,	ADC_DISABLE, 	 	ADC_DISABLE, 	GROUP_DISABLE,		RANK_DISABLE,	DEFAULT_GAIN	},
				{REG_REVD_9,	ADC_DISABLE, 	 	ADC_DISABLE, 	GROUP_DISABLE,		RANK_DISABLE,	DEFAULT_GAIN	},
				{REG_REVD_10,	ADC_DISABLE, 	 	ADC_DISABLE, 	GROUP_DISABLE,		RANK_DISABLE,	DEFAULT_GAIN	},
				{REG_REVD_11,	ADC_DISABLE, 	 	ADC_DISABLE, 	GROUP_DISABLE,		RANK_DISABLE,	DEFAULT_GAIN	},
				{REG_REVD_12,	ADC_DISABLE, 	 	ADC_DISABLE, 	GROUP_DISABLE,		RANK_DISABLE,	DEFAULT_GAIN	}
		},

		.AdcThermoTable =
		{
				// PCU Thermal
				//Name			ZeroFlagEnable,		TableIdx,		Channel Enable,	Adc Group,			Adc Rank
				{PCU_NTC_0,		ADC_DISABLE,		NTCS0603E3103,	ADC_ENABLE,		ADC_2,				RANK_2			},	// MOS NTC Center
				{PCU_NTC_1,		ADC_DISABLE,		NTCS0603E3103,	ADC_ENABLE,		ADC_2,				RANK_3			},	// MOS NTC Side
				{PCU_NTC_2,		ADC_DISABLE,		GWX_LS103H12,	ADC_ENABLE,		ADC_2,				RANK_1			},	// CAP NTC
				{PCU_NTC_3,		ADC_DISABLE,		NO_TABLE,		ADC_DISABLE,	GROUP_DISABLE,		RANK_DISABLE	},
				{PCU_NTC_4,		ADC_DISABLE,		NO_TABLE,		ADC_DISABLE,	GROUP_DISABLE,		RANK_DISABLE	},
				// Axis - 0 Thermal
				//Name			ZeroFlagEnable,	 	Table Idx,		Channel Enable, Adc Group,			Adc Rank
				{MOTOR_NTC_0_A0,ADC_DISABLE,		GWX_LS103H20,	ADC_ENABLE,		ADC_3,				RANK_2			},
				{MOTOR_NTC_1_A0,ADC_DISABLE,		NO_TABLE,		ADC_DISABLE,	GROUP_DISABLE,		RANK_DISABLE	},
				{MOTOR_NTC_2_A0,ADC_DISABLE,		NO_TABLE,		ADC_DISABLE,	GROUP_DISABLE,		RANK_DISABLE	},
				// Axis - 1 Thermal
				//Name			ZeroFlagEnable,	 	Table Idx,		Channel Enable, Adc Group,			Adc Rank
				{MOTOR_NTC_0_A1,ADC_DISABLE,		NO_TABLE,		ADC_DISABLE,	GROUP_DISABLE,		RANK_DISABLE	},
				{MOTOR_NTC_1_A1,ADC_DISABLE,		NO_TABLE,		ADC_DISABLE,	GROUP_DISABLE,		RANK_DISABLE	},
				{MOTOR_NTC_2_A1,ADC_DISABLE,		NO_TABLE,		ADC_DISABLE,	GROUP_DISABLE,		RANK_DISABLE	}
		},

		.AdcCurveFittingTable =
		{
				// A6,			A5,				A4,				A3,				A2,				A1,				A0
				{ 226.34901f, 	410.00309f, 	-321.93657f, 	46.95519f, 		7.8639097f,		-1.1499592f, 	-0.29832702f},		//	NTC On board number - NTCG163JX103DT1S
				{ 286.91117f, 	256.03905f, 	-208.99333f, 	31.554341f,		3.6156109f,		-1.5739555f,	0.12358828f	},		//	NTC Motor Winding number - GWX_LS103H20
				{ 243.11277f,	-123.32722f,	8.3642276f,		2.0067563f,		0.26944288f,	-0.35931768f,	0.056648032f}		//  NTC On board for CAP - GWX_LS103H12
		},

		.PwmStartUpTable_Axis1 =
		{
			//Name,			GroupEnable,	Group,		Channel,		Reserved
			{ CH_PWM_UP, 	PWM_ENABLE, 	TIM_20,		TIM_CH2,		PWM_NONE	},
			{ CH_PWM_UN, 	PWM_ENABLE,		TIM_20,		TIM_CH2, 		PWM_NONE	},
			{ CH_PWM_VP, 	PWM_ENABLE,		TIM_20,		TIM_CH1, 		PWM_NONE	},
			{ CH_PWM_VN, 	PWM_ENABLE,		TIM_20,		TIM_CH1, 		PWM_NONE	},
			{ CH_PWM_WP, 	PWM_ENABLE,		TIM_20,		TIM_CH4, 		PWM_NONE	},
			{ CH_PWM_WN, 	PWM_ENABLE,		TIM_20,		TIM_CH4, 		PWM_NONE	}
		},

		.Pwm_Tim_Initializer_Axis1 =
		{
			TIM20,								\
			&htim20,							\
			TIM_CHANNEL_1,						\
			TIM_CHANNEL_2,						\
			TIM_CHANNEL_4,						\
			1,									\
			TIM_COUNTERMODE_CENTERALIGNED1,		\
			INITIAL_PWM_PERIOD,					\
			TIM_CLOCKDIVISION_DIV1,				\
			INITIAL_REPET_COUNTER,				\
			TIM_AUTORELOAD_PRELOAD_ENABLE,		\
			TIM_CLOCKSOURCE_INTERNAL,			\
			TIM_TRGO_UPDATE,					\
			TIM_TRGO2_RESET,					\
			TIM_MASTERSLAVEMODE_DISABLE,		\
			TIM_BREAKINPUTSOURCE_BKIN,			\
			TIM_BREAKINPUTSOURCE_ENABLE,		\
			TIM_BREAKINPUTSOURCE_POLARITY_LOW,	\
			TIM_OCMODE_PWM2,					\
			4250,								\
			TIM_OCPOLARITY_LOW,					\
			TIM_OCNPOLARITY_LOW,				\
			TIM_OCFAST_DISABLE,					\
			TIM_OCIDLESTATE_RESET,				\
			TIM_OCNIDLESTATE_RESET,				\
			4250,								\
			4250,								\
			TIM_OSSR_DISABLE,					\
			TIM_OSSI_DISABLE,					\
			TIM_LOCKLEVEL_OFF,					\
			203,								\
			TIM_BREAK_ENABLE,					\
			TIM_BREAKPOLARITY_HIGH,				\
			0,									\
			TIM_BREAK_AFMODE_INPUT,				\
			TIM_BREAK2_DISABLE,					\
			TIM_BREAK2POLARITY_HIGH,			\
			0,									\
			TIM_BREAK_AFMODE_INPUT,				\
			TIM_AUTOMATICOUTPUT_DISABLE,		\
			5									\
		},

		.PcuParamTableInfoArray =
		{
				// Property = {10^n[10:8], read location[7:6], size[5], decimal[4:3], Authority[2:0]}. See in "ParamTable.h".
				// ============================ P2 PCUParams ============================
				//  Min,	Max,	Default,	Property,		*pAddr
				//  P2-00 ~ P2-04
				{		0,		2,		1,		0x6,		&DriveParams.PCUParams.UartBaudrateSelect},	//P2-00
				{		0,		0,		0,		0x6,		&DriveParams.PCUParams.CAN0CommType},	//P2-01
				{		100,		10000,		1000,		0x6,		&DriveParams.PCUParams.CAN0Timeout},	//P2-02
				{		500,		10000,		500,		0x6,		&DriveParams.PCUParams.CAN0Baudrate},	//P2-03
				{		0,		0,		0,		0x6,		&DriveParams.PCUParams.CAN1CommType},	//P2-04
				{		100,		10000,		1000,		0x6,		&DriveParams.PCUParams.CAN1Timeout},	//P2-05
				{		500,		10000,		500,		0x6,		&DriveParams.PCUParams.CAN1Baudrate},	//P2-06
				{		0,		0,		0,		0,		0},	//P2-07
				{		0,		0,		0,		0,		0},	//P2-08
				{		0,		0,		0,		0,		0},	//P2-09
				{		5000,		20000,		10000,		0x47,		&DriveParams.PCUParams.CurrentFreq},	//P2-10
				{		1,		5000,		1500,		0x6,		&DriveParams.PCUParams.Deadtime},	//P2-11
				{		1,		5000,		4000,		0x6,		&DriveParams.PCUParams.MinConductTime},	//P2-12
				{		1,		5000,		300,		0x6,		&DriveParams.PCUParams.PCUMaxDCCurrent},	//P2-13
				{		1,		5000,		800,		0x6,		&DriveParams.PCUParams.PCUMaxACCurrent},	//P2-14
				{		0,		0,		0,		0,		0},	//P2-15
				{		0,		0,		0,		0,		0},	//P2-16
				{		0,		0,		0,		0,		0},	//P2-17
				{		0,		0,		0,		0,		0},	//P2-18
				{		0,		0,		0,		0,		0},	//P2-19
				{		0,		0,		0,		0x1,		&DriveParams.PCUParams.DIFunctionSelect[0]},	//P2-20
				{		0,		0,		0,		0x1,		&DriveParams.PCUParams.DIFunctionSelect[1]},	//P2-21
				{		0,		0,		0,		0x1,		&DriveParams.PCUParams.DIFunctionSelect[2]},	//P2-22
				{		0,		0,		0,		0x1,		&DriveParams.PCUParams.DIFunctionSelect[3]},	//P2-23
				{		0,		0,		0,		0x1,		&DriveParams.PCUParams.DIFunctionSelect[4]},	//P2-24
				{		0,		0,		0,		0x1,		&DriveParams.PCUParams.DIFunctionSelect[5]},	//P2-25
				{		0,		0,		0,		0x1,		&DriveParams.PCUParams.DIFunctionSelect[6]},	//P2-26
				{		0,		0,		0,		0,		0},	//P2-27
				{		0,		0,		0,		0,		0},	//P2-28
				{		0,		0,		0,		0,		0},	//P2-29
				{		0,		0,		0,		0,		0},	//P2-30
				{		0,		0,		0,		0,		0},	//P2-31
				{		0,		0,		0,		0,		0},	//P2-32
				{		0,		0,		0,		0,		0},	//P2-33
				{		0,		0,		0,		0,		0},	//P2-34
				{		0,		0,		0,		0,		0},	//P2-35
				{		0,		0,		0,		0,		0},	//P2-36
				{		0,		0,		0,		0,		0},	//P2-37
				{		0,		0,		0,		0,		0},	//P2-38
				{		0,		0,		0,		0,		0},	//P2-39
				{		0,		65535,		0,		0x45,		&DriveParams.PCUParams.Axis1_Iu_Scale[0]},	//P2-40
				{		0,		65535,		0,		0x45,		&DriveParams.PCUParams.Axis1_Iu_Scale[1]},	//P2-41
				{		0,		65535,		0,		0x45,		&DriveParams.PCUParams.Axis1_Iv_Scale[0]},	//P2-42
				{		0,		65535,		0,		0x45,		&DriveParams.PCUParams.Axis1_Iv_Scale[1]},	//P2-43
				{		0,		65535,		0,		0x45,		&DriveParams.PCUParams.Axis1_Iw_Scale[0]},	//P2-44
				{		0,		65535,		0,		0x45,		&DriveParams.PCUParams.Axis1_Iw_Scale[1]},	//P2-45
				{		0,		65535,		0,		0x45,		&DriveParams.PCUParams.Axis2_Iu_Scale[0]},	//P2-46
				{		0,		65535,		0,		0x45,		&DriveParams.PCUParams.Axis2_Iu_Scale[1]},	//P2-47
				{		0,		65535,		0,		0x45,		&DriveParams.PCUParams.Axis2_Iv_Scale[0]},	//P2-48
				{		0,		65535,		0,		0x45,		&DriveParams.PCUParams.Axis2_Iv_Scale[1]},	//P2-49
				{		0,		65535,		0,		0x45,		&DriveParams.PCUParams.Axis2_Iw_Scale[0]},	//P2-50
				{		0,		65535,		0,		0x45,		&DriveParams.PCUParams.Axis2_Iw_Scale[1]},	//P2-51
				{		0,		4096,		2048,		0x45,		&DriveParams.PCUParams.Axis1_Iu_ZeroPoint},	//P2-52
				{		0,		4096,		2048,		0x45,		&DriveParams.PCUParams.Axis1_Iv_ZeroPoint},	//P2-53
				{		0,		4096,		2048,		0x45,		&DriveParams.PCUParams.Axis1_Iw_ZeroPoint},	//P2-54
				{		0,		4096,		2048,		0x45,		&DriveParams.PCUParams.Axis2_Iu_ZeroPoint},	//P2-55
				{		0,		4096,		2048,		0x45,		&DriveParams.PCUParams.Axis2_Iv_ZeroPoint},	//P2-56
				{		0,		4096,		2048,		0x45,		&DriveParams.PCUParams.Axis2_Iw_ZeroPoint},	//P2-57
				{		0,		0,		0,		0,		0},	//P2-58
				{		0,		0,		0,		0,		0},	//P2-59
				{		0,		1,		0,		0x45,		&DriveParams.PCUParams.MF_PCB_Station_Flag},	//P2-60
				{		0,		1,		0,		0x45,		&DriveParams.PCUParams.MF_Module_Station_Flag},	//P2-61
				{		0,		1,		0,		0x45,		&DriveParams.PCUParams.MF_Station_Flag},	//P2-62
				{		0,		65535,		0,		0x45,		&DriveParams.PCUParams.StartCode[0]},	//P2-63
				{		0,		65535,		0,		0x45,		&DriveParams.PCUParams.StartCode[1]},	//P2-64
				{		0,		65535,		0,		0x45,		&DriveParams.PCUParams.StartCode[2]},	//P2-65
				{		0,		65535,		0,		0x45,		&DriveParams.PCUParams.StartCode[3]},	//P2-66
				{		0,		65535,		0,		0x45,		&DriveParams.PCUParams.StartCode[4]},	//P2-67
				{		0,		65535,		0,		0x45,		&DriveParams.PCUParams.StartCode[5]},	//P2-68
				{		0,		65535,		0,		0x45,		&DriveParams.PCUParams.PCUSNCode[0]},	//P2-69
				{		0,		65535,		0,		0x45,		&DriveParams.PCUParams.PCUSNCode[1]},	//P2-70
				{		0,		65535,		0,		0x45,		&DriveParams.PCUParams.PCUSNCode[2]},	//P2-71
				{		0,		65535,		0,		0x45,		&DriveParams.PCUParams.PCUSNCode[3]},	//P2-72
				{		0,		65535,		0,		0x45,		&DriveParams.PCUParams.PCUSNCode[4]},	//P2-73
				{		0,		65535,		0,		0x45,		&DriveParams.PCUParams.PCUSNCode[5]},	//P2-74
				{		0,		65535,		0,		0x45,		&DriveParams.PCUParams.PCUSNCode[6]},	//P2-75
				{		0,		65535,		0,		0x45,		&DriveParams.PCUParams.PCUSNCode[7]},	//P2-76
				{		0,		65535,		0,		0x45,		&DriveParams.PCUParams.PCUSNCode[8]},	//P2-77
				{		0,		0,		0,		0,		0},	//P2-78
				{		0,		0,		0,		0,		0},	//P2-79
				{		0,		65535,		0,		0x45,		&DriveParams.PCUParams.DebugParam1},	//P2-80
				{		0,		65535,		0,		0x47,		&DriveParams.PCUParams.DebugParam2},	//P2-81
				{		0,		65535,		0,		0x47,		&DriveParams.PCUParams.DebugParam3},	//P2-82
				{		0,		65535,		0,		0x47,		&DriveParams.PCUParams.DebugParam4},	//P2-83
				{		0,		65535,		0,		0x47,		&DriveParams.PCUParams.DebugParam5},	//P2-84
				{		0,		65535,		0,		0x47,		&DriveParams.PCUParams.DebugParam6},	//P2-85
				{		0,		65535,		0,		0x47,		&DriveParams.PCUParams.DebugParam7},	//P2-86
				{		0,		65535,		0,		0x47,		&DriveParams.PCUParams.DebugParam8},	//P2-87
				{		0,		65535,		0,		0x44,		&DriveParams.PCUParams.DebugParam9},	//P2-88
				{		0,		65535,		0,		0x44,		&DriveParams.PCUParams.DebugParam10},	//P2-89
				{		0,		4095,		0,		0x45,		&DriveParams.PCUParams.DcBusCalibValue[0]},	//P2-90
				{		0,		4095,		0,		0x45,		&DriveParams.PCUParams.DcBusCalibValue[1]},	//P2-91
				{		0,		0,		0,		0,		0},	//P2-92
				{		0,		0,		0,		0,		0},	//P2-93
				{		0,		0,		0,		0,		0},	//P2-94
				{		0,		0,		0,		0,		0},	//P2-95
				{		0,		0,		0,		0,		0},	//P2-96
				{		0,		0,		0,		0,		0},	//P2-97
				{		0,		0,		0,		0,		0},	//P2-98
				{		0,		0,		0,		0,		0},	//P2-99
				{		0,		0,		0,		0,		0},	//P3-00 alarm threshold parameter start
				{		0,		0,		0,		0,		0},	//P3-01
				{		0,		0,		0,		0,		0},	//P3-02
				{		0,		THRE_MAX,		0,		0x45,		&DriveParams.PCUParams.Reserved303},	//P3-03 ALARMID_CAN1_COMM_ERROR
				{		0,		THRE_MAX,		100,		0x45,		&DriveParams.PCUParams.Reserved304},	//P3-04 ALARMID_CAN1_TIMEOUT
				{		0,		0,		0,		0,		0},	//P3-05
				{		0,		0,		0,		0,		0},	//P3-06
				{		0,		0,		0,		0,		0},	//P3-07
				{		0,		0,		0,		0,		0},	//P3-08
				{		0,		0,		0,		0,		0},	//P3-09
				{		0,		THRE_MAX,		THRE_MAX,		0x45,		&DriveParams.PCUParams.Reserved310},	//P3-10 ALARMID_POWER_TRANSISTOR_OC
				{		0,		THRE_MAX,		THRE_MAX,		0x45,		&DriveParams.PCUParams.Reserved311},	//P3-11 ALARMID_BUFFER_IC_ERROR
				{		0,		THRE_MAX,		THRE_MAX,		0x45,		&DriveParams.PCUParams.Reserved312},	//P3-12 ALARMID_PHASE_LOSS
				{		0,		THRE_MAX,		8000,		0x45,		&DriveParams.PCUParams.Reserved313},	//P3-13 ALARMID_MOTOR_OVER_SPEED
				{		0,		THRE_MAX,		62,		0x45,		&DriveParams.PCUParams.Reserved314},	//P3-14 ALARMID_OVER_VOLTAGE_BUS
				{		0,		THRE_MAX,		36,		0x45,		&DriveParams.PCUParams.Reserved315},	//P3-15 ALARMID_UNDER_VOLTAGE_BUS
				{		0,		THRE_MAX,		10,		0x45,		&DriveParams.PCUParams.Reserved316},	//P3-16 ALARMID_UNDER_VOLTAGE_13V
				{		0,		THRE_MAX,		THRE_MAX,		0x45,		&DriveParams.PCUParams.Reserved317},	//P3-17 ALARMID_IU_OCP
				{		0,		THRE_MAX,		THRE_MAX,		0x45,		&DriveParams.PCUParams.Reserved318},	//P3-18 ALARMID_IV_OCP
				{		0,		THRE_MAX,		THRE_MAX,		0x45,		&DriveParams.PCUParams.Reserved319},	//P3-19 ALARMID_IW_OCP
				{		0,		THRE_MAX,		0,		0x45,		&DriveParams.PCUParams.Reserved320},	//P3-20 ALARMID_FLASH_UNINITIALIZED
				{		0,		THRE_MAX,		0,		0x45,		&DriveParams.PCUParams.Reserved321},	//P3-21 ALARMID_FLASH_READ_FAILED
				{		0,		THRE_MAX,		0,		0x45,		&DriveParams.PCUParams.Reserved322},	//P3-22 ALARMID_FLASH_DAMAGED
				{		0,		0,		0,		0,		0},	//P3-23
				{		0,		0,		0,		0,		0},	//P3-24
				{		0,		0,		0,		0,		0},	//P3-25
				{		0,		0,		0,		0,		0},	//P3-26
				{		0,		0,		0,		0,		0},	//P3-27
				{		0,		0,		0,		0,		0},	//P3-28
				{		0,		0,		0,		0,		0},	//P3-29
				{		0,		0,		0,		0,		0},	//P3-30
				{		0,		0,		0,		0,		0},	//P3-31
				{		0,		0,		0,		0,		0},	//P3-32
				{		0,		0,		0,		0,		0},	//P3-33
				{		0,		THRE_MAX,		100,		0x45,		&DriveParams.PCUParams.Reserved334},	//P3-34 ALARMID_FOIL_BREAK
				{		0,		THRE_MAX,		4000,		0x45,		&DriveParams.PCUParams.Reserved335},	//P3-35 ALARMID_FOIL_SHORT
				{		0,		0,		0,		0,		0},	//P3-36
				{		0,		0,		0,		0,		0},	//P3-37
				{		0,		0,		0,		0,		0},	//P3-38
				{		0,		0,		0,		0,		0},	//P3-39
				{		0,		0,		0,		0,		0},	//P3-40
				{		0,		0,		0,		0,		0},	//P3-41
				{		0,		0,		0,		0,		0},	//P3-42
				{		0,		0,		0,		0,		0},	//P3-43
				{		0,		0,		0,		0,		0},	//P3-44
				{		0,		0,		0,		0,		0},	//P3-45
				{		0,		0,		0,		0,		0},	//P3-46
				{		0,		0,		0,		0,		0},	//P3-47
				{		0,		THRE_MAX,		120,		0x45,		&DriveParams.PCUParams.Reserved348},	//P3-48 ALARMID_OT_PCU_0
				{		0,		THRE_MAX,		3900,		0x45,		&DriveParams.PCUParams.Reserved349},	//P3-49 ALARMID_BREAK_NTC_PCU_0
				{		0,		THRE_MAX,		120,		0x45,		&DriveParams.PCUParams.Reserved350},	//P3-50 ALARMID_SHORT_NTC_PCU_0
				{		0,		THRE_MAX,		120,		0x45,		&DriveParams.PCUParams.Reserved351},	//P3-51 ALARMID_OT_PCU_1
				{		0,		THRE_MAX,		3900,		0x45,		&DriveParams.PCUParams.Reserved352},	//P3-52 ALARMID_BREAK_NTC_PCU_1
				{		0,		THRE_MAX,		120,		0x45,		&DriveParams.PCUParams.Reserved353},	//P3-53 ALARMID_SHORT_NTC_PCU_1
				{		0,		THRE_MAX,		115,		0x45,		&DriveParams.PCUParams.Reserved354},	//P3-54 ALARMID_OT_PCU_2
				{		0,		THRE_MAX,		4065,		0x45,		&DriveParams.PCUParams.Reserved355},	//P3-55 ALARMID_BREAK_NTC_PCU_2
				{		0,		THRE_MAX,		100,		0x45,		&DriveParams.PCUParams.Reserved356},	//P3-56 ALARMID_SHORT_NTC_PCU_2
				{		0,		THRE_MAX,		175,		0x45,		&DriveParams.PCUParams.Reserved357},	//P3-57 ALARMID_OT_MOTOR_0
				{		0,		THRE_MAX,		4075,		0x45,		&DriveParams.PCUParams.Reserved358},	//P3-58 ALARMID_BREAK_NTC_MOTOR_0
				{		0,		THRE_MAX,		200,		0x45,		&DriveParams.PCUParams.Reserved359},	//P3-59 ALARMID_SHORT_NTC_MOTOR_0
				{		0,		THRE_MAX,		115,		0x45,		&DriveParams.PCUParams.Reserved360},	//P3-60 ALARMID_OT_PCU_0_WARNING
				{		0,		THRE_MAX,		115,		0x45,		&DriveParams.PCUParams.Reserved361},	//P3-61 ALARMID_OT_PCU_1_WARNING
				{		0,		THRE_MAX,		110,		0x45,		&DriveParams.PCUParams.Reserved362},	//P3-62 ALARMID_OT_PCU_2_WARNING
				{		0,		THRE_MAX,		170,		0x45,		&DriveParams.PCUParams.Reserved363},	//P3-63 ALARMID_OT_MOTOR_0_WARNING
				{		0,		THRE_MAX,		THRE_MAX,		0x45,		&DriveParams.PCUParams.Reserved364},	//P3-64 ALARMID_MOTORSTALL
				{		0,		0,		0,		0,		0},	//P3-65
				{		0,		0,		0,		0,		0},	//P3-66
				{		0,		0,		0,		0,		0},	//P3-67
				{		0,		0,		0,		0,		0},	//P3-68
				{		0,		0,		0,		0,		0},	//P3-69
				{		0,		0,		0,		0,		0},	//P3-70
				{		0,		0,		0,		0,		0},	//P3-71
				{		0,		0,		0,		0,		0},	//P3-72
				{		0,		0,		0,		0,		0},	//P3-73
				{		0,		0,		0,		0,		0},	//P3-74
				{		0,		0,		0,		0,		0},	//P3-75
				{		0,		0,		0,		0,		0},	//P3-76
				{		0,		0,		0,		0,		0},	//P3-77
				{		0,		0,		0,		0,		0},	//P3-78
				{		0,		0,		0,		0,		0},	//P3-79
				{		0,		0,		0,		0,		0},	//P3-80
				{		0,		0,		0,		0,		0},	//P3-81
				{		0,		0,		0,		0,		0},	//P3-82
				{		0,		0,		0,		0,		0},	//P3-83
				{		0,		0,		0,		0,		0},	//P3-84
				{		0,		0,		0,		0,		0},	//P3-85
				{		0,		0,		0,		0,		0},	//P3-86
				{		0,		0,		0,		0,		0},	//P3-87
				{		0,		0,		0,		0,		0},	//P3-88
				{		0,		0,		0,		0,		0},	//P3-89
				{		0,		0,		0,		0,		0},	//P3-90
				{		0,		0,		0,		0,		0},	//P3-91
				{		0,		0,		0,		0,		0},	//P3-92
				{		0,		0,		0,		0,		0},	//P3-93
				{		0,		0,		0,		0,		0},	//P3-94
				{		0,		0,		0,		0,		0},	//P3-95
				{		0,		0,		0,		0,		0},	//P3-96
				{		0,		0,		0,		0,		0},	//P3-97
				{		0,		THRE_MAX,		9,		0x45,		&DriveParams.PCUParams.Reserved398},	//P3-98 ALARMID_RC_ABNORMAL
				{		0,		0,		0,		0,		0}	//P3-99
		},

		.CheckWord = 0x4321
};
#endif
