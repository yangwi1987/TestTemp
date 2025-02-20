/*
 * ParamData.h
 *
 *  Created on: Dec 19, 2019
 *      Author: MikeSFWen
 */

#ifndef INC_PARAMDATA_H_
#define INC_PARAMDATA_H_

// Parameter Number Definition
#define PN_SYS_BASE					0
#define PN_BATT_MAXVOLT				1
#define PN_BATT_MINVOLT				2
#define PN_SC_GEAR_NUM				10
#define PN_SC_GEAR_DEN				11
#define PN_SC_GEARBOX_DIR			12
#define PN_SC_GEARBOX_EFF			13
#define PN_SC_TIRE_RADIUS			14

#define PN_REGEN_SPEED_START_0		20
#define PN_REGEN_SPEED_START_1		21
#define PN_REGEN_SPEED_START_2		22
#define PN_REGEN_SPEED_START_3		23
#define PN_REGEN_SPEED_START_4		24

#define PN_REGEN_PROPULSION_MAX_0	25
#define PN_REGEN_PROPULSION_MAX_1	26
#define PN_REGEN_PROPULSION_MAX_2	27
#define PN_REGEN_PROPULSION_MAX_3	28
#define PN_REGEN_PROPULSION_MAX_4	29

#define PN_REGEN_POWER_MAX_0		30
#define PN_REGEN_POWER_MAX_1		31
#define PN_REGEN_POWER_MAX_2		32
#define PN_REGEN_POWER_MAX_3		33
#define PN_REGEN_POWER_MAX_4		34

#define PN_REGEN_SPEED_MAX_0		35
#define PN_REGEN_SPEED_MAX_1		36
#define PN_REGEN_SPEED_MAX_2		37
#define PN_REGEN_SPEED_MAX_3		38
#define PN_REGEN_SPEED_MAX_4		39

#define PN_REGEN_SLOPE_START_0		40
#define PN_REGEN_SLOPE_START_1		41
#define PN_REGEN_SLOPE_START_2		42
#define PN_REGEN_SLOPE_START_3		43
#define PN_REGEN_SLOPE_START_4		44

#define PN_REGEN_SLOPE_END_0		45
#define PN_REGEN_SLOPE_END_1		46
#define PN_REGEN_SLOPE_END_2		47
#define PN_REGEN_SLOPE_END_3		48
#define PN_REGEN_SLOPE_END_4		49

#define PN_REGEN_RAMP_0				50
#define PN_REGEN_RAMP_1				51
#define PN_REGEN_RAMP_2				52
#define PN_REGEN_RAMP_3				53
#define PN_REGEN_RAMP_4				54

#define PN_DRIVE_PROPULSION_START_0	55
#define PN_DRIVE_PROPULSION_START_1	56
#define PN_DRIVE_PROPULSION_START_2	57
#define PN_DRIVE_PROPULSION_START_3	58
#define PN_DRIVE_PROPULSION_START_4	59

#define PN_DRIVE_PROPULSION_MAX_0	60
#define PN_DRIVE_PROPULSION_MAX_1	61
#define PN_DRIVE_PROPULSION_MAX_2	62
#define PN_DRIVE_PROPULSION_MAX_3	63
#define PN_DRIVE_PROPULSION_MAX_4	64

#define PN_DRIVE_POWER_MAX_0		65
#define PN_DRIVE_POWER_MAX_1		66
#define PN_DRIVE_POWER_MAX_2		67
#define PN_DRIVE_POWER_MAX_3		68
#define PN_DRIVE_POWER_MAX_4		69

#define PN_DRIVE_SPEED_MAX_0		70
#define PN_DRIVE_SPEED_MAX_1		71
#define PN_DRIVE_SPEED_MAX_2		72
#define PN_DRIVE_SPEED_MAX_3		73
#define PN_DRIVE_SPEED_MAX_4		74

#define PN_DRIVE_SLOPE_START_0		75
#define PN_DRIVE_SLOPE_START_1		76
#define PN_DRIVE_SLOPE_START_2		77
#define PN_DRIVE_SLOPE_START_3		78
#define PN_DRIVE_SLOPE_START_4		79

#define PN_DRIVE_SLOPE_END_0		80
#define PN_DRIVE_SLOPE_END_1		81
#define PN_DRIVE_SLOPE_END_2		82
#define PN_DRIVE_SLOPE_END_3		83
#define PN_DRIVE_SLOPE_END_4		84

#define PN_DRIVE_RISING_RAMP		85
#define PN_DRIVE_FALLING_RAMP		86

#define PN_THROT_SHORT_BOUNDARY		124
#define PN_THROT_BREAK_BOUNDARY		125

#define PN_Max_Analog_Foil_ADC		130
#define PN_Min_Analog_Foil_ADC		131
#define PN_Max_AnaFoil_Sen_Surf		132
#define PN_Min_AnaFoil_Sen_Surf		133
#define PN_Max_AnaFoil_Sen_Foil		134
#define PN_Min_AnaFoil_Sen_Foil		135

#define PN_HIGH_PWM_SPEED			180
#define PN_LOW_PWM_SPEED			181
#define PN_PWM_FREQ_TIMES_N			182

#define PN_PCU_BASE					200
#define PN_UART_BAUDRATE			200
#define PN_CAN0_COMM_TYPE			201
#define PN_CAN0_TIMEOUT_MS			202
#define PN_CAN0_BAUDRATE			203
#define PN_CAN1_COMM_TYPE			204
#define PN_CAN1_TIMEOUT_MS			205
#define PN_CAN1_BAUDRATE			206
#define PN_CURRENT_FREQ				210
#define PN_DEADTIME					211
#define PN_MIN_CONDUC_TIME			212
#define PN_PCU_MAX_DCCURR			213
#define PN_PCU_MAX_ACCURR			214
#define PN_DI1_SELECT				220
#define PN_DI2_SELECT				221
#define PN_DI3_SELECT				222
#define PN_DI4_SELECT				223
#define PN_DI5_SELECT				224
#define PN_DI6_SELECT				225
#define PN_DI7_SELECT				226
#define PN_AXIS1_U_CURR_SCALE		240
#define PN_AXIS1_V_CURR_SCALE		242
#define PN_AXIS1_W_CURR_SCALE		244
#define PN_AXIS2_U_CURR_SCALE		246
#define PN_AXIS2_V_CURR_SCALE		248
#define PN_AXIS2_W_CURR_SCALE		250

#define PN_PCBA_MF_FLAG				260
#define PN_MODULE_MF_FLAG			261
#define PN_PCU_MF_FLAG				262

#define PN_DEBUG_PARAM_1			280
#define PN_DEBUG_PARAM_2			281
#define PN_DEBUG_PARAM_3			282
#define PN_DEBUG_PARAM_4			283
#define PN_DEBUG_PARAM_5			284
#define PN_DEBUG_PARAM_6			285
#define PN_DEBUG_PARAM_7			286
#define PN_DEBUG_PARAM_8			287
#define PN_DEBUG_PARAM_9			288
#define PN_DEBUG_PARAM_10			289

#define PN_MOT_BASE					400
#define PN_MOTOR_TYPE				400
#define PN_POLE_PAIR				401
#define PN_LD						402
#define PN_LQ						403
#define PN_RS						404
#define PN_LAMBDA_M					405
#define PN_MAX_SPEED				406
#define PN_MAX_TORQUE				407
#define PN_MOTOR_MAX_AC_CURR		408
#define PN_MOTOR_SERIAL_1			455
#define PN_MOTOR_SERIAL_2			456
#define PN_MOTOR_SERIAL_3			457
#define PN_MOTOR_SERIAL_4			458
#define PN_MOTOR_SERIAL_5			459
#define PN_MOTOR_SERIAL_6			460
#define PN_MOTOR_SERIAL_7			461
#define PN_MOTOR_SERIAL_8			462
#define PN_MOTOR_SERIAL_9			463
#define PN_MOTOR_SERIAL_10			464
#define PN_MOTOR_SERIAL_11			465
#define PN_MOTOR_SERIAL_12			466
#define PN_MOTOR_SERIAL_13			467
#define PN_MOTOR_SERIAL_14			468
#define PN_MOTOR_SERIAL_15			469
#define PN_MOTOR_ROTOR_OFFSET		470
#define PN_MOTOR_DELAY_TIME			471
#define PN_ENCODER_COMM_TYPE		480
#define PN_ENCODER_RES				481
#define PN_ENCODER_POLARITY			482
#define PN_PSB_NONLINEAR_POINT1		483
#define PN_PSB_NONLINEAR_POINT2		484
#define PN_PSB_NONLINEAR_POINT3		485
#define PN_PSB_NONLINEAR_POINT4		486
#define PN_PSB_NONLINEAR_POINT5		487
#define PN_PSB_NONLINEAR_POINT6		488
#define PN_PSB_NONLINEAR_POINT7		489
#define PN_PSB_NONLINEAR_POINT8		490
#define PN_PSB_NONLINEAR_POINT9		491
#define PN_PSB_NONLINEAR_POINT10	492
#define PN_PSB_NONLINEAR_POINT11	493
#define PN_PSB_NONLINEAR_POINT12	494
#define PN_PSB_NONLINEAR_POINT13	495
#define PN_PSB_NONLINEAR_POINT14	496
#define PN_PSB_NONLINEAR_POINT15	497
#define PN_PSB_NONLINEAR_POINT16	498
#define PN_PSB_NONLINEAR_POINT17	499
#define PN_PSB_NONLINEAR_POINT18	500
#define PN_PSB_NONLINEAR_POINT19	501
#define PN_PSB_NONLINEAR_POINT20	502
#define PN_PSB_NONLINEAR_POINT21	503
#define PN_PSB_NONLINEAR_POINT22	504
#define PN_PSB_NONLINEAR_POINT23	505
#define PN_PSB_NONLINEAR_POINT24	506
#define PN_PSB_NL_SIN_PARA1			507
#define PN_PSB_NL_SIN_PARA2			508
#define PN_PSB_NL_SIN_PARA3			509
#define PN_PSB_NL_SIN_PARA4			510
#define PN_PSB_NL_SIN_PARA5			511
#define PN_PSB_NL_SIN_PARA6			512
#define PN_PSB_NL_SIN_PARA7			513
#define PN_PSB_NL_SIN_PARA8			514
#define PN_PSB_NL_SIN_PARA9			515
#define PN_PSB_NL_SIN_PARA10		516
#define PN_PSB_NL_SIN_PARA11		517
#define PN_PSB_NL_SIN_PARA12		518
#define PN_PSB_NL_SIN_PARA13		519
#define PN_PSB_NL_SIN_PARA14		520
#define PN_PSB_NL_SIN_PARA15		521
#define PN_PSB_NL_SIN_PARA16		522
#define PN_PSB_NL_SIN_PARA17		523
#define PN_PSB_NL_SIN_PARA18		524
#define PN_CHECK_SUM				599

#define PN_UI_BASE					600

// Debug Scope Number Definition
#define PN_SCOPE_BASE				700

#define DN_THROTTLE_RAW_ADCDATA		700
#define DN_BRAKE_RAW_ADCDATA		701
#define DN_FOURQUAD_STATE			702
#define DN_THROTTLE_CMD				703
#define DN_MAX_TORQ_CMD				704
#define DN_OUTPUT_TORQ_CMD			705
#define DN_DERATING_INDICATOR		706
#define DN_REGEN_LIMIT				707
#define DN_REGEN_COMMAND			708
#define DN_SERVO_ON_STATE			710

#define DN_SECURITY_ACCESS_LV		712

#define DN_IO_STATUS				720
#define DN_BUS_VOLTAGE				721
#define DN_BUS_CURRENT				722
#define DN_BUS_POWER				723
#define DN_IA_CURR					724
#define DN_IB_CURR					725
#define DN_IC_CURR					726
#define DN_ID_CURR					727
#define DN_IQ_CURR					728
#define DN_IDQ_CURR					729

#define DN_VDQ_CMD					730
#define DN_MOTOR_SPEED				732
#define DN_TRIP						734
#define DN_MOTOR_0_NTC_TEMP			735
#define DN_PCU_NTC_0_TEMP			736
#define DN_PCU_NTC_1_TEMP			737
#define DN_GATE_DRIVE_VOLT			738
#define DN_IA_CURR_RMS				739

#define DN_IB_CURR_RMS				740
#define DN_IC_CURR_RMS				741
#define DN_PCU_NTC_2_TEMP			742
#define DN_CURRENT_LIMIT			747
#define DN_U_CURR_ADC				749

#define DN_V_CURR_ADC				750
#define DN_W_CURR_ADC				751
#define DN_BAT_ADC					752
#define DN_TN_SELECT				753
#define DN_THROT_MAPPING_RST		754

#define DN_ALRAM_ID_01				755
#define DN_ALRAM_ID_23				756
#define DN_ALRAM_ID_45				757
#define DN_ALRAM_ID_67				758
#define DN_ALRAM_ID_89				759

#define DN_BOOTUP_TIME				760
#define DN_STATUS_WORD1				761
#define DN_STATUS_WORD2				762

#define DN_ACC_UART_ERROR_CNT		765
#define DN_ACC_CAN_ERROR_CNT		766


#define DN_ID_VER_READ				780
#define DN_INFO_VER_READ			781
#define DN_FRAME_VER_READ			782
#define DN_RELEASE_VER_READ			783
#define DN_RELEASE_CANDIDATE_READ   784

#define DN_THIS_OP_TIME				790 // this operation time
#define DN_TOTAL_OP_TIME			791 // total operation time

#define DN_INTERNAL_FLASH_OP_RESULT 803

#if JUDGE_FUNCTION_DELAY || MEASURE_CPU_LOAD
#define DN_MAX_TIM8INT_INTERVAL    850
#define DN_AVE_TIM8INT_INTERVAL    851
#define DN_MAX_CURRENTLOOP_INTERVAL 852
#define DN_AVE_CURRENTLOOP_INTERVAL 853
#define DN_MAX_PLCLOOP_INTERVAL     854
#define DN_AVE_PLCLOOP_INTERVAL     855
#define DN_MAX_100HZLOOP_INTERVAL   856
#define DN_AVE_100HZLOOP_INTERVAL   857
#define DN_MAX_100HZLOOP_LOAD_PCT   858
#define DN_MAX_PLCLOOP_LOAD_PCT     859
#define DN_MAX_CURRENTLOOP_LOAD_PCT 860
#define DN_MAX_ADC_INJ_LOAD_PCT     861
#define DN_AVE_100HZLOOP_LOAD_PCT   862
#define DN_AVE_PLCLOOP_LOAD_PCT     863
#define DN_AVE_CURRENTLOOP_LOAD_PCT 864
#endif

#define	DN_ALARMID_REPORT			880

#define	DN_DEBUG_SCOPE_1			890
#define	DN_DEBUG_SCOPE_2			891
#define	DN_DEBUG_SCOPE_3			892
#define	DN_DEBUG_SCOPE_4			893
#define	DN_DEBUG_SCOPE_5			894
#define	DN_DEBUG_SCOPE_6			895
#define	DN_DEBUG_SCOPE_7			896
#define	DN_DEBUG_SCOPE_8			897
#define	DN_DEBUG_SCOPE_9			898
#define	DN_DEBUG_SCOPE_10			899


// Function Register Definition
#define FN_BASE						900
//0 ~ 9
#define FN_AUTHORITY				900
#define FN_ENABLE					901
#define FN_MF_FUNC_SEL				902
#define FN_RD_FUNC_SEL				903
#define FN_TORQ_COMMAND				905
#define FN_AC_CURR_LIMIT			906
#define FN_DC_CURR_LIMIT			907
#define FN_MAX_TORQUE				908
#define FN_RPM_SLOPE_CMD			909

//10 ~ 19
#define FN_RPM_GAIN_CMD				910
#define FN_REINIT					911
#define FN_PARAM_SAVE_INDEX			912
#define FN_PARAM_SAVE_AXISID		913
#define FN_PARAM_SAVE_ADDR			914
#define FN_OPEN_SPD_COMMAND			915
#define FN_PARAM_BACKUP_EMEMORY		916
#define FN_OPEN_SPD_V_I_LIMIT		917
#define FN_FOUR_QUAD_STATES			918
#define FN_PING_CAN1_2_COMMAND		919

//20 ~ 29
#define FN_MF_PARAM_SET_0			920
#define FN_MF_PARAM_SET_1			921
#define FN_MF_PARAM_SET_2			922
#define FN_MF_CURR_CALIB_SETUP		923
#define FN_MF_CURR_CALIB_START      924
#define FN_MF_CURR_CALC				925
#define FN_FUNCTION_SELECT			926
#define FN_FUNCTION_ACTIVATE		927
#define FN_READ_VERSION_CTRL		928
#define FN_ALARM_ID_READ_SRC		929
//30 ~ 39
#define FN_ASIC_CTRL_ADDR_LOW		930
#define FN_ASIC_CTRL_ADDR_HIGH		931
#define FN_ASIC_CTRL_VALUE_LOW		932
#define FN_ASIC_CTRL_VALUE_HIGH		933
#define FN_ASIC_CTRL_OPERATION_CODE	934
#define FN_ASIC_CTRL_AXIS_NUMBER	935
#define FN_THROT_CALIB_READ_AD		936
#define FN_THROT_CALIB_CALC			937


#define FN_PCU_SN_OPERATION			940
#define FN_PCU_ERR_CNT_RESET		941
//40~49

#define FN_DATA_BUFFER_0			941
#define FN_DATA_BUFFER_1			942
#define FN_DATA_BUFFER_2			943
#define FN_DATA_BUFFER_3			944
#define FN_DATA_BUFFER_4			945
#define FN_DATA_BUFFER_5			946
#define FN_DATA_BUFFER_6			947
#define FN_DATA_BUFFER_7			948
#define FN_DATA_BUFFER_8			949
//50~59
#define FN_DATA_BUFFER_9			950

#define FN_PCU_RESET_OPERATION		952
#define FN_FLASH_ID_SECTION_EREASE	953
#define FN_EXTFLASH_DATA_RST_SET	959
#define FN_EXTFLASH_DATA_RST_ENA	960

#define FN_MF_VOLT_CALIB_START      961

#define FN_ORIGIN_PARAM_BACKUP      962

#define FN_MF_POS_CALIB_START       963

#define FN_OPEN_POSITION_CMD_ENABLE 970
#define FN_OPEN_POSITION_CMD        971
#define FN_CURRENT_ID_CMD           972
#define FN_CURRENT_IQ_CMD           973
#define FN_CURRENT_IS_CMD           974
#define FN_CURRENT_THETA_CMD        975
#define FN_PWM_U_CMD                976
#define FN_PWM_V_CMD                977
#define FN_PWM_W_CMD                978
#define FN_PWM_Mode                 979

#define FN_TEST_USAGE				999
#define PN_MAX						1000

#define PARAM_SAVE_ADDR_IS_INTERNAL_FLASH	0
#define PARAM_SAVE_ADDR_IS_EXT_FLASH		1
#define PARAM_SAVE_ADDR_IS_PSB				3

enum FN_ENABLE_ENUM
{
	FN_ENABLE_STOP = 0,
	FN_ENABLE_RD_START,
	FN_ENABLE_MF_START,
};

enum FN_MF_FUNC_SEL_ENUM
{
	FN_MF_FUNC_SEL_RESERVED = 0x0000,
	FN_MF_FUNC_SEL_TORQUE_MODE,
	FN_MF_FUNC_SEL_VF,
	FN_MF_FUNC_SEL_IF,
	FN_MF_FUNC_SEL_IDQ,
	FN_MF_FUNC_SEL_ISTHETA,
	FN_MF_FUNC_SEL_PWM
};

enum FN_PWM_ENUM
{
	FN_PWM_MODE_COMPLEMENTARY = 0,
	FN_PWM_MODE_UV,
	FN_PWM_MODE_VW,
	FN_PWM_MODE_UW
};

#define MAX_POWER_LEVEL 10


#endif /* INC_PARAMDATA_H_ */
