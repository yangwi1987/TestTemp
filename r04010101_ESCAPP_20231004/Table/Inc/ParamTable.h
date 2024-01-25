/*
 * ParamTable.h
 *
 *  Created on: 2020年4月15日
 *      Author: Mike.Wen.SFW
 */

#ifndef INC_PARAMTABLE_H_
#define INC_PARAMTABLE_H_

#include "stdint.h"
#include "FourQuadTable.h"

#define MAX_AXIS_NUM			2
#define PARAM_GROUP				6
#define PARAM_SIZEPERGROUP		100  								// 100 parameters(uint16_t)
#define SYS_PARAM_SIZE			(2 * PARAM_SIZEPERGROUP)			// 200 parameters(uint16_t)
#define PCU_PARAM_SIZE			(2 * PARAM_SIZEPERGROUP)
#define MOT_PARAM_SIZE			(2 * PARAM_SIZEPERGROUP)
#define PARAM_NUMBER_SIZE		(PARAM_GROUP * PARAM_SIZEPERGROUP)	// 600 parameters(uint16_t)

#define EnableBitMask 			0b00000001

/* Parameter Property
 * bit 0-2 Authority
 * bit 3-4 decimal
 * bit 5 size, 0 = 16bit, 1 = 32bit
 * bit 6-7 Read location. 0 = Internal Flash, 1 = ExtFlash, 2 = MR-IC
 * bit 8-10 10^n unit. 0~7 = unit is 10^(0~7)
 */

// Parameter Property Definition
#define PPD_AUTHORITY_MASK		0x0007
#define PPD_DECIMAL_MASK		0x0018
#define PPD_SIZE_LONG			0x0020
#define PPD_READ_LOC			0x00C0
#define PPD_POWER10_MASK		0x0700
#define PPD_SHIFT_MASK			0x0800

#define AUTHORITY_DEVELOPER		7
#define AUTHORITY_MANUFACTURE	2
#define AUTHORITY_DEPOT			1
#define AUTHORITY_END_USER		0

#define READ_FROM_DEFAULT		0x00
#define READ_FROM_EXTFLASH		0x40
#define READ_FROM_MR			0x80

#define PPD_DECIMAL_SHIFT		3
#define PPD_POWER10_SHIFT		8

#define PARA_VALUE_SHIFT	32768.0f

typedef struct /*__attribute__((__packed__))*/
{
	uint32_t Min;
	uint32_t Max;
	uint32_t Default;
	uint16_t Property; // {HasShift[11], Power10[10:8], read location[7:6], size[5], decimal[4:3], Authority[2:0]}.
	uint16_t *pAddr;
} ParamTableInfo_t;

typedef struct {
	uint16_t Reserved000;	//P0-00
	uint16_t BattVoltMax;	//P0-01
	uint16_t BattVoltMin;	//P0-02
	uint16_t Reserved003;	//P0-03
	uint16_t Reserved004;	//P0-04
	uint16_t Reserved005;	//P0-05
	uint16_t Reserved006;	//P0-06
	uint16_t Reserved007;	//P0-07
	uint16_t Reserved008;	//P0-08
	uint16_t Reserved009;	//P0-09
	uint16_t Reserved010;	//P0-10
	uint16_t Reserved011;	//P0-11
	uint16_t Reserved012;	//P0-12
	uint16_t Reserved013;	//P0-13
	uint16_t Reserved014;	//P0-14
	uint16_t Reserved015;	//P0-15
	uint16_t Reserved016;	//P0-16
	uint16_t Reserved017;	//P0-17
	uint16_t Reserved018;	//P0-18
	uint16_t Reserved019;	//P0-19
	uint16_t Reserved020;	//P0-20
	uint16_t Reserved021;	//P0-21
	uint16_t Reserved022;	//P0-22
	uint16_t Reserved023;	//P0-23
	uint16_t Reserved024;	//P0-24
	uint16_t Reserved025;	//P0-25
	uint16_t Reserved026;	//P0-26
	uint16_t Reserved027;	//P0-27
	uint16_t Reserved028;	//P0-28
	uint16_t Reserved029;	//P0-29
	uint16_t Reserved030;	//P0-30
	uint16_t Reserved031;	//P0-31
	uint16_t Reserved032;	//P0-32
	uint16_t Reserved033;	//P0-33
	uint16_t Reserved034;	//P0-34
	uint16_t Reserved035;	//P0-35
	uint16_t Reserved036;	//P0-36
	uint16_t Reserved037;	//P0-37
	uint16_t Reserved038;	//P0-38
	uint16_t Reserved039;	//P0-39
	uint16_t Reserved040;	//P0-40
	uint16_t Reserved041;	//P0-41
	uint16_t Reserved042;	//P0-42
	uint16_t Reserved043;	//P0-43
	uint16_t Reserved044;	//P0-44
	uint16_t Reserved045;	//P0-45
	uint16_t Reserved046;	//P0-46
	uint16_t Reserved047;	//P0-47
	uint16_t Reserved048;	//P0-48
	uint16_t Reserved049;	//P0-49
	uint16_t Reserved050;	//P0-50
	uint16_t Reserved051;	//P0-51
	uint16_t Reserved052;	//P0-52
	uint16_t Reserved053;	//P0-53
	uint16_t Reserved054;	//P0-54
	uint16_t DriveCurve[DRIVE_ALL_PARA_NUM][DRIVE_TABLE_LENGTH];
	uint16_t DriveRisingRamp;	//P0-85
	uint16_t DriveFallingRamp;	//P0-86
	uint16_t LimpTransitSec;	//P0-87
	uint16_t PosZeroOffset;	//P0-88
	uint16_t PosCompBySpeed;	//P0-89
	uint16_t Reserved090;	//P0-90
	uint16_t Reserved091;	//P0-91
	uint16_t Reserved092;	//P0-92
	uint16_t Reserved093;	//P0-93
	uint16_t Reserved094;		//P0-94
	uint16_t Reserved095;	//P0-95
	uint16_t DrainRisingRamp;	//P0-96
	uint16_t DrainFallingRamp;	//P0-97
	uint16_t Reserved098;	//P0-98
	uint16_t Reserved099;	//P0-99
	uint16_t ThrottleMaxAdc;	//P1-00
	uint16_t ThrottleMinAdc;	//P1-01
	uint16_t ThrottleScale[2];	//P1-02
	uint16_t ThrottleEmptyPt[5];	//P1-04
	uint16_t ThrottleHalfPt[5];	//P1-09
	uint16_t ThrottleFullPt[5];	//P1-14
	uint16_t ThrottleRiseRamp[5];	//P1-19
	uint16_t ThrottleFallRamp[5];	//P1-24
	uint16_t Reserved129;	//P1-29
	uint16_t Reserved130;	//P1-30
	uint16_t Reserved131;	//P1-31
	uint16_t Reserved132;	//P1-32
	uint16_t Reserved133;	//P1-33
	uint16_t Reserved134;	//P1-34
	uint16_t Reserved135;	//P1-35
	uint16_t Reserved136;	//P1-36
	uint16_t Reserved137;	//P1-37
	uint16_t Reserved138;	//P1-38
	uint16_t Reserved139;	//P1-39
	uint16_t ParamMgrSecurity;	//P1-40
	uint16_t Reserved141;	//P1-41
	uint16_t Reserved142;	//P1-42
	uint16_t Reserved143;	//P1-43
	uint16_t Reserved144;	//P1-44
	uint16_t Reserved145;	//P1-45
	uint16_t Reserved146;	//P1-46
	uint16_t Reserved147;	//P1-47
	uint16_t Reserved148;	//P1-48
	uint16_t Reserved149;	//P1-49
	uint16_t Reserved150;	//P1-50
	uint16_t Reserved151;	//P1-51
	uint16_t Reserved152;	//P1-52
	uint16_t Reserved153;	//P1-53
	uint16_t Reserved154;	//P1-54
	uint16_t Reserved155;	//P1-55
	uint16_t Reserved156;	//P1-56
	uint16_t Reserved157;	//P1-57
	uint16_t Reserved158;	//P1-58
	uint16_t Reserved159;	//P1-59
	uint16_t Reserved160;	//P1-60
	uint16_t Reserved161;	//P1-61
	uint16_t Reserved162;	//P1-62
	uint16_t Reserved163;	//P1-63
	uint16_t Reserved164;	//P1-64
	uint16_t Reserved165;	//P1-65
	uint16_t Reserved166;	//P1-66
	uint16_t Reserved167;	//P1-67
	uint16_t Reserved168;	//P1-68
	uint16_t Reserved169;	//P1-69
	uint16_t Reserved170;	//P1-70
	uint16_t Reserved171;	//P1-71
	uint16_t Reserved172;	//P1-72
	uint16_t Reserved173;	//P1-73
	uint16_t Reserved174;	//P1-74
	uint16_t Reserved175;	//P1-75
	uint16_t Reserved176;	//P1-76
	uint16_t Reserved177;	//P1-77
	uint16_t Reserved178;	//P1-78
	uint16_t Reserved179;	//P1-79
	uint16_t Reserved180;	//P1-80
	uint16_t Reserved181;	//P1-81
	uint16_t Reserved182;	//P1-82
	uint16_t Reserved183;	//P1-83
	uint16_t Reserved184;	//P1-84
	uint16_t Reserved185;	//P1-85
	uint16_t Reserved186;	//P1-86
	uint16_t Reserved187;	//P1-87
	uint16_t Reserved188;	//P1-88
	uint16_t Reserved189;	//P1-89
	uint16_t MotorDeratingTempOffset;	//P1-90
	uint16_t MotorDeratingScale;	//P1-91
	uint16_t MosDeratingTempOffset;	//P1-92
	uint16_t MOSDeratingScale;	//P1-93
	uint16_t Reserved194;	//P1-94
	uint16_t Reserved195;	//P1-95
	uint16_t Reserved196;	//P1-96
	uint16_t Reserved197;	//P1-97
	uint16_t Reserved198;	//P1-98
	uint16_t Reserved199;	//P1-99
} SystemParams_t;

typedef struct {
	uint16_t UartBaudrateSelect;	//P2-00
	uint16_t CAN0CommType;	//P2-01
	uint16_t CAN0Timeout;	//P2-02
	uint16_t CAN0Baudrate;	//P2-03
	uint16_t CAN1CommType;	//P2-04
	uint16_t CAN1Timeout;	//P2-05
	uint16_t CAN1Baudrate;	//P2-06
	uint16_t Reserved207;	//P2-07
	uint16_t Reserved208;	//P2-08
	uint16_t Reserved209;	//P2-09
	uint16_t CurrentFreq;	//P2-10
	uint16_t Deadtime;	//P2-11
	uint16_t MinConductTime;	//P2-12
	uint16_t PCUMaxDCCurrent;	//P2-13
	uint16_t PCUMaxACCurrent;	//P2-14
	uint16_t Reserved215;	//P2-15
	uint16_t Reserved216;	//P2-16
	uint16_t Reserved217;	//P2-17
	uint16_t Reserved218;	//P2-18
	uint16_t Reserved219;	//P2-19
	uint16_t DIFunctionSelect[7];	//P2-20
	uint16_t Reserved227;	//P2-27
	uint16_t Reserved228;	//P2-28
	uint16_t Reserved229;	//P2-29
	uint16_t Reserved230;	//P2-30
	uint16_t Reserved231;	//P2-31
	uint16_t Reserved232;	//P2-32
	uint16_t Reserved233;	//P2-33
	uint16_t Reserved234;	//P2-34
	uint16_t Reserved235;	//P2-35
	uint16_t Reserved236;	//P2-36
	uint16_t Reserved237;	//P2-37
	uint16_t Reserved238;	//P2-38
	uint16_t Reserved239;	//P2-39
	uint16_t Axis1_Iu_Scale[2];	//P2-40
	uint16_t Axis1_Iv_Scale[2];	//P2-42
	uint16_t Axis1_Iw_Scale[2];	//P2-44
	uint16_t Axis2_Iu_Scale[2];	//P2-46
	uint16_t Axis2_Iv_Scale[2];	//P2-48
	uint16_t Axis2_Iw_Scale[2];	//P2-50
	uint16_t Axis1_Iu_ZeroPoint;	//P2-52
	uint16_t Axis1_Iv_ZeroPoint;	//P2-53
	uint16_t Axis1_Iw_ZeroPoint;	//P2-54
	uint16_t Axis2_Iu_ZeroPoint;	//P2-55
	uint16_t Axis2_Iv_ZeroPoint;	//P2-56
	uint16_t Axis2_Iw_ZeroPoint;	//P2-57
	uint16_t Reserved258;	//P2-58
	uint16_t Reserved259;	//P2-59
	uint16_t MF_PCB_Station_Flag;	//P2-60
	uint16_t MF_Module_Station_Flag;	//P2-61
	uint16_t MF_Station_Flag;	//P2-62
	uint16_t StartCode[6];	//P2-63 ~ 68
	uint16_t PCUSNCode[9];	//P2-69 ~ 77
	uint16_t Reserved278;	//P2-78
	uint16_t Reserved279;	//P2-79
	uint16_t DebugParam1;	//P2-80
	uint16_t DebugParam2;	//P2-81
	uint16_t DebugParam3;	//P2-82
	uint16_t DebugParam4;	//P2-83
	uint16_t DebugParam5;	//P2-84
	uint16_t DebugParam6;	//P2-85
	uint16_t DebugParam7;	//P2-86
	uint16_t DebugParam8;	//P2-87
	uint16_t DebugParam9;	//P2-88
	uint16_t DebugParam10;	//P2-89
	uint16_t DcBusCalibValue[2];	//P2-90~ 91
	uint16_t Reserved292;	//P2-92
	uint16_t Reserved293;	//P2-93
	uint16_t Reserved294;	//P2-94
	uint16_t Reserved295;	//P2-95
	uint16_t Reserved296;	//P2-96
	uint16_t Reserved297;	//P2-97
	uint16_t Reserved298;	//P2-98
	uint16_t Reserved299;	//P2-99
	uint16_t Reserved300;	//P3-00
	uint16_t Reserved301;	//P3-01
	uint16_t Reserved302;	//P3-02
	uint16_t Reserved303;	//P3-03
	uint16_t Reserved304;	//P3-04
	uint16_t Reserved305;	//P3-05
	uint16_t Reserved306;	//P3-06
	uint16_t Reserved307;	//P3-07
	uint16_t Reserved308;	//P3-08
	uint16_t Reserved309;	//P3-09
	uint16_t Reserved310;	//P3-10
	uint16_t Reserved311;	//P3-11
	uint16_t Reserved312;	//P3-12
	uint16_t Reserved313;	//P3-13
	uint16_t Reserved314;	//P3-14
	uint16_t Reserved315;	//P3-15
	uint16_t Reserved316;	//P3-16
	uint16_t Reserved317;	//P3-17
	uint16_t Reserved318;	//P3-18
	uint16_t Reserved319;	//P3-19
	uint16_t Reserved320;	//P3-20
	uint16_t Reserved321;	//P3-21
	uint16_t Reserved322;	//P3-22
	uint16_t Reserved323;	//P3-23
	uint16_t Reserved324;	//P3-24
	uint16_t Reserved325;	//P3-25
	uint16_t Reserved326;	//P3-26
	uint16_t Reserved327;	//P3-27
	uint16_t Reserved328;	//P3-28
	uint16_t Reserved329;	//P3-29
	uint16_t Reserved330;	//P3-30
	uint16_t Reserved331;	//P3-31
	uint16_t Reserved332;	//P3-32
	uint16_t Reserved333;	//P3-33
	uint16_t Reserved334;	//P3-34
	uint16_t Reserved335;	//P3-35
	uint16_t Reserved336;	//P3-36
	uint16_t Reserved337;	//P3-37
	uint16_t Reserved338;	//P3-38
	uint16_t Reserved339;	//P3-39
	uint16_t Reserved340;	//P3-40
	uint16_t Reserved341;	//P3-41
	uint16_t Reserved342;	//P3-42
	uint16_t Reserved343;	//P3-43
	uint16_t Reserved344;	//P3-44
	uint16_t Reserved345;	//P3-45
	uint16_t Reserved346;	//P3-46
	uint16_t Reserved347;	//P3-47
	uint16_t Reserved348;	//P3-48
	uint16_t Reserved349;	//P3-49
	uint16_t Reserved350;	//P3-50
	uint16_t Reserved351;	//P3-51
	uint16_t Reserved352;	//P3-52
	uint16_t Reserved353;	//P3-53
	uint16_t Reserved354;	//P3-54
	uint16_t Reserved355;	//P3-55
	uint16_t Reserved356;	//P3-56
	uint16_t Reserved357;	//P3-57
	uint16_t Reserved358;	//P3-58
	uint16_t Reserved359;	//P3-59
	uint16_t Reserved360;	//P3-60
	uint16_t Reserved361;	//P3-61
	uint16_t Reserved362;	//P3-62
	uint16_t Reserved363;	//P3-63
	uint16_t Reserved364;	//P3-64
	uint16_t Reserved365;	//P3-65
	uint16_t Reserved366;	//P3-66
	uint16_t Reserved367;	//P3-67
	uint16_t Reserved368;	//P3-68
	uint16_t Reserved369;	//P3-69
	uint16_t Reserved370;	//P3-70
	uint16_t Reserved371;	//P3-71
	uint16_t Reserved372;	//P3-72
	uint16_t Reserved373;	//P3-73
	uint16_t Reserved374;	//P3-74
	uint16_t Reserved375;	//P3-75
	uint16_t Reserved376;	//P3-76
	uint16_t Reserved377;	//P3-77
	uint16_t Reserved378;	//P3-78
	uint16_t Reserved379;	//P3-79
	uint16_t Reserved380;	//P3-80
	uint16_t Reserved381;	//P3-81
	uint16_t Reserved382;	//P3-82
	uint16_t Reserved383;	//P3-83
	uint16_t Reserved384;	//P3-84
	uint16_t Reserved385;	//P3-85
	uint16_t Reserved386;	//P3-86
	uint16_t Reserved387;	//P3-87
	uint16_t Reserved388;	//P3-88
	uint16_t Reserved389;	//P3-89
	uint16_t Reserved390;	//P3-90
	uint16_t Reserved391;	//P3-91
	uint16_t Reserved392;	//P3-92
	uint16_t Reserved393;	//P3-93
	uint16_t Reserved394;	//P3-94
	uint16_t Reserved395;	//P3-95
	uint16_t Reserved396;	//P3-96
	uint16_t Reserved397;	//P3-97
	uint16_t Reserved398;	//P3-98
	uint16_t Reserved399;	//P3-99
} PCUParams_t;

typedef struct {
	uint16_t Reserved400;	//P4-00
	uint16_t Reserved401;	//P4-01
	uint16_t Reserved402;	//P4-02
	uint16_t Reserved403;	//P4-03
	uint16_t Reserved404;	//P4-04
	uint16_t Reserved405;	//P4-05
	uint16_t Reserved406;	//P4-06
	uint16_t Reserved407;	//P4-07
	uint16_t Reserved408;	//P4-08
	uint16_t Reserved409;	//P4-09
	uint16_t Reserved410;	//P4-10
	uint16_t Reserved411;	//P4-11
	uint16_t Reserved412;	//P4-12
	uint16_t Reserved413;	//P4-13
	uint16_t Reserved414;	//P4-14
	uint16_t Reserved415;	//P4-15
	uint16_t Reserved416;	//P4-16
	uint16_t Reserved417;	//P4-17
	uint16_t Reserved418;	//P4-18
	uint16_t Reserved419;	//P4-19
	uint16_t Reserved420;	//P4-20
	uint16_t Reserved421;	//P4-21
	uint16_t Reserved422;	//P4-22
	uint16_t Reserved423;	//P4-23
	uint16_t Reserved424;	//P4-24
	uint16_t Reserved425;	//P4-25
	uint16_t Reserved426;	//P4-26
	uint16_t Reserved427;	//P4-27
	uint16_t Reserved428;	//P4-28
	uint16_t Reserved429;	//P4-29
	uint16_t Reserved430;	//P4-30
	uint16_t Reserved431;	//P4-31
	uint16_t Reserved432;	//P4-32
	uint16_t Reserved433;	//P4-33
	uint16_t Reserved434;	//P4-34
	uint16_t Reserved435;	//P4-35
	uint16_t Reserved436;	//P4-36
	uint16_t Reserved437;	//P4-37
	uint16_t Reserved438;	//P4-38
	uint16_t Reserved439;	//P4-39
	uint16_t Reserved440;	//P4-40
	uint16_t Reserved441;	//P4-41
	uint16_t Reserved442;	//P4-42
	uint16_t Reserved443;	//P4-43
	uint16_t Reserved444;	//P4-44
	uint16_t Reserved445;	//P4-45
	uint16_t Reserved446;	//P4-46
	uint16_t Reserved447;	//P4-47
	uint16_t Reserved448;	//P4-48
	uint16_t Reserved449;	//P4-49
	uint16_t Reserved450;	//P4-50
	uint16_t Reserved451;	//P4-51
	uint16_t Reserved452;	//P4-52
	uint16_t Reserved453;	//P4-53
	uint16_t Reserved454;	//P4-54
	uint16_t Reserved455;	//P4-55
	uint16_t Reserved456;	//P4-56
	uint16_t Reserved457;	//P4-57
	uint16_t Reserved458;	//P4-58
	uint16_t Reserved459;	//P4-59
	uint16_t Reserved460;	//P4-60
	uint16_t Reserved461;	//P4-61
	uint16_t Reserved462;	//P4-62
	uint16_t Reserved463;	//P4-63
	uint16_t Reserved464;	//P4-64
	uint16_t Reserved465;	//P4-65
	uint16_t Reserved466;	//P4-66
	uint16_t Reserved467;	//P4-67
	uint16_t Reserved468;	//P4-68
	uint16_t Reserved469;	//P4-69
	uint16_t Reserved470;	//P4-70
	uint16_t Reserved471;	//P4-71
	uint16_t Reserved472;	//P4-72
	uint16_t Reserved473;	//P4-73
	uint16_t Reserved474;	//P4-74
	uint16_t Reserved475;	//P4-75
	uint16_t Reserved476;	//P4-76
	uint16_t Reserved477;	//P4-77
	uint16_t Reserved478;	//P4-78
	uint16_t Reserved479;	//P4-79
	uint16_t Reserved480;	//P4-80
	uint16_t Reserved481;	//P4-81
	uint16_t Reserved482;	//P4-82
	uint16_t Reserved483;	//P4-83
	uint16_t Reserved484;	//P4-84
	uint16_t Reserved485;	//P4-85
	uint16_t Reserved486;	//P4-86
	uint16_t Reserved487;	//P4-87
	uint16_t Reserved488;	//P4-88
	uint16_t Reserved489;	//P4-89
	uint16_t Reserved490;	//P4-90
	uint16_t Reserved491;	//P4-91
	uint16_t Reserved492;	//P4-92
	uint16_t Reserved493;	//P4-93
	uint16_t Reserved494;	//P4-94
	uint16_t Reserved495;	//P4-95
	uint16_t Reserved496;	//P4-96
	uint16_t Reserved497;	//P4-97
	uint16_t Reserved498;	//P4-98
	uint16_t Reserved499;	//P4-99
	uint16_t Reserved500;	//P5-00
	uint16_t Reserved501;	//P5-01
	uint16_t Reserved502;	//P5-02
	uint16_t Reserved503;	//P5-03
	uint16_t Reserved504;	//P5-04
	uint16_t Reserved505;	//P5-05
	uint16_t Reserved506;	//P5-06
	uint16_t Reserved507;	//P5-07
	uint16_t Reserved508;	//P5-08
	uint16_t Reserved509;	//P5-09
	uint16_t Reserved510;	//P5-00
	uint16_t Reserved511;	//P5-11
	uint16_t Reserved512;	//P5-12
	uint16_t Reserved513;	//P5-13
	uint16_t Reserved514;	//P5-14
	uint16_t Reserved515;	//P5-15
	uint16_t Reserved516;	//P5-16
	uint16_t Reserved517;	//P5-17
	uint16_t Reserved518;	//P5-18
	uint16_t Reserved519;	//P5-19
	uint16_t Reserved520;	//P5-20
	uint16_t Reserved521;	//P5-21
	uint16_t Reserved522;	//P5-22
	uint16_t Reserved523;	//P5-23
	uint16_t Reserved524;	//P5-24
	uint16_t Reserved525;	//P5-25
	uint16_t Reserved526;	//P5-26
	uint16_t Reserved527;	//P5-27
	uint16_t Reserved528;	//P5-28
	uint16_t Reserved529;	//P5-29
	uint16_t Reserved530;	//P5-30
	uint16_t Reserved531;	//P5-31
	uint16_t Reserved532;	//P5-32
	uint16_t Reserved533;	//P5-33
	uint16_t Reserved534;	//P5-34
	uint16_t Reserved535;	//P5-35
	uint16_t Reserved536;	//P5-36
	uint16_t Reserved537;	//P5-37
	uint16_t Reserved538;	//P5-38
	uint16_t Reserved539;	//P5-39
	uint16_t Reserved540;	//P5-40
	uint16_t Reserved541;	//P5-41
	uint16_t Reserved542;	//P5-42
	uint16_t Reserved543;	//P5-43
	uint16_t Reserved544;	//P5-44
	uint16_t Reserved545;	//P5-45
	uint16_t Reserved546;	//P5-46
	uint16_t Reserved547;	//P5-47
	uint16_t Reserved548;	//P5-48
	uint16_t Reserved549;	//P5-49
	uint16_t Reserved550;	//P5-50
	uint16_t Reserved551;	//P5-51
	uint16_t Reserved552;	//P5-52
	uint16_t Reserved553;	//P5-53
	uint16_t Reserved554;	//P5-54
	uint16_t Reserved555;	//P5-55
	uint16_t Reserved556;	//P5-56
	uint16_t Reserved557;	//P5-57
	uint16_t Reserved558;	//P5-58
	uint16_t Reserved559;	//P5-59
	uint16_t Reserved560;	//P5-60
	uint16_t Reserved561;	//P5-61
	uint16_t Reserved562;	//P5-62
	uint16_t Reserved563;	//P5-63
	uint16_t Reserved564;	//P5-64
	uint16_t Reserved565;	//P5-65
	uint16_t Reserved566;	//P5-66
	uint16_t Reserved567;	//P5-67
	uint16_t Reserved568;	//P5-68
	uint16_t Reserved569;	//P5-69
	uint16_t Reserved570;	//P5-70
	uint16_t Reserved571;	//P5-71
	uint16_t Reserved572;	//P5-72
	uint16_t Reserved573;	//P5-73
	uint16_t Reserved574;	//P5-74
	uint16_t Reserved575;	//P5-75
	uint16_t Reserved576;	//P5-76
	uint16_t Reserved577;	//P5-77
	uint16_t Reserved578;	//P5-78
	uint16_t Reserved579;	//P5-79
	uint16_t Reserved580;	//P5-80
	uint16_t Reserved581;	//P5-81
	uint16_t Reserved582;	//P5-82
	uint16_t Reserved583;	//P5-83
	uint16_t Reserved584;	//P5-84
	uint16_t Reserved585;	//P5-85
	uint16_t Reserved586;	//P5-86
	uint16_t Reserved587;	//P5-87
	uint16_t Reserved588;	//P5-88
	uint16_t Reserved589;	//P5-89
	uint16_t Reserved590;	//P5-90
	uint16_t Reserved591;	//P5-91
	uint16_t Reserved592;	//P5-92
	uint16_t Reserved593;	//P5-93
	uint16_t Reserved594;	//P5-94
	uint16_t Reserved595;	//P5-95
	uint16_t Reserved596;	//P5-96
	uint16_t Reserved597;	//P5-97
	uint16_t Reserved598;	//P5-98
	uint16_t Reserved599;	//P5-99

} MotorEncoderParams_t;

typedef struct {
	SystemParams_t SystemParams;
	PCUParams_t PCUParams;
	MotorEncoderParams_t MotorEncoderParams[MAX_AXIS_NUM];
} DriveParams_t;

extern DriveParams_t DriveParams;
extern const ParamTableInfo_t SysParamTableInfoArray[SYS_PARAM_SIZE];
extern const ParamTableInfo_t PcuParamTableInfoArray[PCU_PARAM_SIZE];
extern const ParamTableInfo_t MotEncParamTableInfoArray[MOT_PARAM_SIZE];

#endif /* INC_PARAMTABLE_H_ */
