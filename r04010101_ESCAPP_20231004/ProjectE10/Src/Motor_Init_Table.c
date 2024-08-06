/*
 * MotorTable.c
 *
 *  Created on: Feb 20, 2020
 *      Author: Fernando
 */
#if E10
#include "Motor_Init_Table.h"




__attribute__((__section__(".Motor1Bin"),used)) const MOTOR_TABLE_TYPE MotorTable =
{
	.Version = {1, 0x04, 1, 1}, /* Version numbers are undefined now. If version number are all zero, const table cannot work in bootloader. */
	.NumOfHeader = 5,
	.CheckSum = 0,

#if MOTOR_STAGE==MOTOR_P0
		//IdCmd
	.IdCmdLutTable=
	{
			{	0,	-748,	-1249,	-1654,	-2276,	-2882,	-3451,	-4467,	-7411,	-8791,	-9756,	-10769,	-11745,	-12821,	-14099,	},
			{	0,	-696,	-1221,	-1641,	-2287,	-2866,	-3424,	-4459,	-7498,	-8902,	-9874,	-10900,	-11871,	-12976,	-14409,	},
			{	0,	-681,	-1218,	-1668,	-2302,	-2873,	-3449,	-4485,	-7599,	-8992,	-10009,	-11035,	-12028,	-13165,	-14649,	},
			{	0,	-682,	-1224,	-1689,	-2317,	-2895,	-3470,	-4534,	-7672,	-9088,	-10125,	-11134,	-12184,	-13360,	-14833,	},
			{	0,	-687,	-1233,	-1706,	-2331,	-2934,	-3486,	-4563,	-7750,	-9137,	-10183,	-11231,	-12308,	-13548,	-14949,	},
			{	0,	-692,	-1241,	-1729,	-2342,	-2953,	-3495,	-4577,	-7779,	-9158,	-10209,	-11276,	-12365,	-13559,	-15049,	},
			{	0,	-695,	-1249,	-1768,	-2351,	-2961,	-3500,	-4578,	-7774,	-9162,	-10219,	-11294,	-12385,	-13560,	-15093,	},
			{	0,	-697,	-1256,	-1797,	-2358,	-2965,	-3502,	-4572,	-7734,	-9154,	-10220,	-11299,	-12388,	-13556,	-15110,	},
			{	0,	-696,	-1262,	-1834,	-2363,	-2965,	-3502,	-4561,	-7636,	-9143,	-10218,	-11298,	-12386,	-13551,	-15117,	},
			{	0,	-695,	-1265,	-1850,	-2366,	-2964,	-3500,	-4551,	-7590,	-9132,	-10216,	-11296,	-12382,	-13545,	-15120,	},
			{	0,	-692,	-1265,	-1856,	-2368,	-2964,	-3499,	-4543,	-7569,	-9124,	-10213,	-11293,	-12378,	-13540,	-15120,	},
			{	0,	-689,	-1264,	-1858,	-2371,	-2963,	-3497,	-4538,	-7559,	-9118,	-10210,	-11289,	-12373,	-13533,	-15120,	},
			{	0,	-685,	-1261,	-1859,	-2377,	-2962,	-3493,	-4534,	-7553,	-9114,	-10206,	-11284,	-12366,	-13524,	-15119,	},
			{	0,	-678,	-1256,	-1859,	-2394,	-2961,	-3487,	-4527,	-7546,	-9107,	-10199,	-11275,	-12356,	-13512,	-15116,	},
			{	0,	-667,	-1246,	-1857,	-2433,	-2958,	-3477,	-4517,	-7535,	-9096,	-10186,	-11260,	-12339,	-13493,	-15111,	},
			{	0,	-651,	-1229,	-1848,	-2466,	-2948,	-3459,	-4498,	-7517,	-9076,	-10165,	-11236,	-12312,	-13465,	-15105,	},
			{	0,	-627,	-1200,	-1829,	-2463,	-2927,	-3431,	-4470,	-7488,	-9046,	-10132,	-11202,	-12277,	-13431,	-15102,	},
			{	0,	-595,	-1153,	-1799,	-2439,	-2894,	-3395,	-4434,	-7448,	-9004,	-10091,	-11163,	-12243,	-13403,	-15117,	},
			{	0,	-563,	-1086,	-1765,	-2406,	-2859,	-3364,	-4406,	-7410,	-8962,	-10057,	-11145,	-12244,	-13422,	-15186,	},
			{	0,	-552,	-1113,	-1752,	-2396,	-2858,	-3378,	-4432,	-7410,	-8957,	-10079,	-11215,	-12368,	-13583,	-15388,	},
			{	-606,	-606,	-1207,	-1818,	-2480,	-2971,	-3536,	-4627,	-7535,	-9077,	-10268,	-11520,	-12791,	-14080,	-15881,	},
			{	-825,	-825,	-1455,	-2089,	-2803,	-3368,	-4027,	-5220,	-7956,	-9502,	-10840,	-12332,	-13831,	-15261,	-16946,	},
			{	-1409,	-1409,	-2085,	-2800,	-3629,	-4348,	-5189,	-6635,	-8965,	-10549,	-12172,	-14125,	-16029,	-17736,	-17736,	},
			{	-2716,	-2716,	-3473,	-4358,	-5406,	-6421,	-7603,	-9034,	-11051,	-12756,	-14717,	-16877,	-18848,	-20515,	-20515,	},
			{	-4887,	-4887,	-5660,	-6653,	-7795,	-8990,	-10362,	-11888,	-13763,	-15631,	-17675,	-19928,	-21840,	-22943,	-22943,	},
			{	-7303,	-7303,	-8096,	-9102,	-10302,	-11617,	-13002,	-14658,	-16556,	-18479,	-20541,	-22748,	-24383,	-24383,	-24383,	},
			{	-9631,	-9631,	-10487,	-11548,	-12646,	-14044,	-15491,	-17134,	-19081,	-21200,	-23295,	-25251,	-25251,	-25251,	-25251,	},
			{	-11805,	-11805,	-12758,	-13793,	-14789,	-16220,	-17723,	-19406,	-21358,	-23511,	-25608,	-27288,	-27288,	-27288,	-27288,	},
			{	-13780,	-13780,	-14650,	-15776,	-16760,	-18170,	-19722,	-21463,	-23430,	-25597,	-27660,	-27660,	-27660,	-27660,	-27660,	},
			{	-15491,	-15491,	-16439,	-17577,	-18603,	-19885,	-21685,	-23293,	-25453,	-27393,	-29195,	-29195,	-29195,	-29195,	-29195,	},
			{	-17192,	-17192,	-17983,	-19228,	-20331,	-21504,	-23434,	-25041,	-27193,	-29007,	-29007,	-29007,	-29007,	-29007,	-29007,	},
			{	-18962,	-18962,	-19442,	-20704,	-21737,	-23141,	-24953,	-26640,	-28570,	-30292,	-30292,	-30292,	-30292,	-30292,	-30292,	},
			{	-20692,	-20692,	-20928,	-22133,	-23085,	-24525,	-26568,	-27965,	-29814,	-29814,	-29814,	-29814,	-29814,	-29814,	-29814,	},
			{	-21820,	-21820,	-22377,	-23414,	-24415,	-25896,	-27986,	-29226,	-30909,	-30909,	-30909,	-30909,	-30909,	-30909,	-30909,	},
			{	-22924,	-22924,	-23542,	-24362,	-25765,	-27281,	-29264,	-30397,	-30397,	-30397,	-30397,	-30397,	-30397,	-30397,	-30397,	},
			{	-24634,	-24634,	-24844,	-25436,	-27001,	-28519,	-30171,	-31416,	-31416,	-31416,	-31416,	-31416,	-31416,	-31416,	-31416,	},
			{	-25679,	-25679,	-25751,	-26372,	-27947,	-29398,	-31160,	-31160,	-31160,	-31160,	-31160,	-31160,	-31160,	-31160,	-31160,	},
			{	-26605,	-26605,	-26746,	-27664,	-29110,	-30386,	-31876,	-31876,	-31876,	-31876,	-31876,	-31876,	-31876,	-31876,	-31876,	},
			{	-27834,	-27834,	-28349,	-28771,	-30077,	-31162,	-31162,	-31162,	-31162,	-31162,	-31162,	-31162,	-31162,	-31162,	-31162,	},
			{	-29932,	-29932,	-29522,	-30234,	-31334,	-32124,	-32124,	-32124,	-32124,	-32124,	-32124,	-32124,	-32124,	-32124,	-32124,	},
	},
	.IdCmdLutInfo=
	{
		.XLength = 15,
		.XInterval = 1.43836,
		.XMin = 0,
		.YLength = 40,
		.YInterval = 7.44581,
		.YMin = 7.44581,
		.Scale = 0.0165,
		.pTableStart = &MotorTable.IdCmdLutTable[0][0],
	},
	.IdCmdHeader=
	{
		TABLE_TYPE_LUT_INT16_2DIM,
		&MotorTable.IdCmdLutInfo,
	},

	//IqCmd
	.IqCmdLutTable=
	{
			{	0,	3106,	4885,	6313,	8218,	10362,	12378,	14241,	15457,	17243,	19124,	21137,	23068,	25098,	26951,	},
			{	0,	3058,	4873,	6346,	8323,	10390,	12462,	14424,	15579,	17415,	19352,	21408,	23334,	25398,	27244,	},
			{	0,	3044,	4882,	6468,	8388,	10436,	12612,	14572,	15764,	17566,	19613,	21675,	23636,	25703,	27541,	},
			{	0,	3044,	4898,	6538,	8435,	10507,	12697,	14748,	15907,	17744,	19838,	21867,	23927,	25962,	27803,	},
			{	0,	3049,	4917,	6593,	8472,	10635,	12747,	14837,	16066,	17839,	19949,	22055,	24163,	26108,	27980,	},
			{	0,	3054,	4938,	6669,	8504,	10692,	12773,	14879,	16142,	17884,	20000,	22143,	24278,	26330,	28159,	},
			{	0,	3058,	4960,	6815,	8532,	10716,	12784,	14898,	16179,	17904,	20018,	22178,	24331,	26432,	28246,	},
			{	0,	3060,	4983,	6925,	8556,	10723,	12787,	14906,	16210,	17912,	20022,	22188,	24350,	26473,	28287,	},
			{	0,	3062,	5003,	7066,	8576,	10724,	12786,	14911,	16259,	17916,	20020,	22188,	24355,	26486,	28304,	},
			{	0,	3063,	5017,	7129,	8589,	10723,	12784,	14914,	16279,	17918,	20017,	22185,	24353,	26488,	28310,	},
			{	0,	3064,	5024,	7156,	8600,	10723,	12782,	14915,	16287,	17919,	20014,	22182,	24351,	26487,	28311,	},
			{	0,	3064,	5026,	7169,	8614,	10726,	12780,	14915,	16290,	17920,	20013,	22181,	24350,	26486,	28311,	},
			{	0,	3064,	5027,	7181,	8647,	10735,	12780,	14915,	16292,	17921,	20014,	22181,	24350,	26488,	28311,	},
			{	0,	3064,	5028,	7200,	8725,	10753,	12780,	14916,	16293,	17922,	20016,	22184,	24354,	26492,	28312,	},
			{	0,	3066,	5032,	7226,	8898,	10778,	12782,	14919,	16297,	17926,	20020,	22190,	24361,	26500,	28314,	},
			{	0,	3072,	5045,	7255,	9073,	10805,	12787,	14923,	16302,	17933,	20029,	22200,	24373,	26513,	28316,	},
			{	0,	3083,	5075,	7285,	9156,	10828,	12794,	14931,	16312,	17944,	20041,	22214,	24388,	26529,	28317,	},
			{	0,	3100,	5141,	7313,	9195,	10847,	12803,	14941,	16324,	17958,	20056,	22230,	24403,	26540,	28307,	},
			{	0,	3122,	5280,	7337,	9216,	10862,	12812,	14949,	16335,	17972,	20067,	22234,	24399,	26527,	28267,	},
			{	0,	3141,	5328,	7350,	9222,	10868,	12814,	14941,	16333,	17971,	20054,	22196,	24336,	26440,	28154,	},
			{	0,	3150,	5328,	7335,	9203,	10851,	12785,	14884,	16288,	17924,	19972,	22051,	24130,	26181,	27885,	},
			{	0,	3134,	5276,	7261,	9126,	10777,	12687,	14707,	16144,	17765,	19739,	21677,	23637,	25574,	27308,	},
			{	0,	3065,	5127,	7068,	8925,	10579,	12450,	14279,	15802,	17385,	19211,	20868,	22615,	24313,	24313,	},
			{	0,	2901,	4793,	6652,	8486,	10145,	11957,	13542,	15098,	16595,	18242,	19730,	21277,	22895,	22895,	},
			{	0,	2552,	4347,	6142,	7858,	9490,	11207,	12731,	14283,	15715,	17187,	18636,	20086,	21747,	21747,	},
			{	0,	2128,	3834,	5558,	7177,	8765,	10409,	11893,	13360,	14720,	16119,	17513,	19049,	19049,	19049,	},
			{	0,	1699,	3343,	4960,	6560,	8084,	9649,	11075,	12462,	13807,	15122,	16543,	16543,	16543,	16543,	},
			{	0,	1304,	2867,	4467,	6033,	7508,	8996,	10372,	11742,	12996,	14260,	15785,	15785,	15785,	15785,	},
			{	0,	956,	2473,	4026,	5567,	7018,	8415,	9784,	11055,	12275,	13541,	13541,	13541,	13541,	13541,	},
			{	0,	662,	2138,	3649,	5126,	6592,	7919,	9287,	10450,	11658,	12943,	12943,	12943,	12943,	12943,	},
			{	0,	383,	1858,	3303,	4775,	6194,	7502,	8845,	9950,	11115,	11115,	11115,	11115,	11115,	11115,	},
			{	0,	104,	1597,	3031,	4485,	5847,	7146,	8429,	9550,	10622,	10622,	10622,	10622,	10622,	10622,	},
			{	0,	-157,	1348,	2796,	4232,	5550,	6791,	8084,	9193,	9193,	9193,	9193,	9193,	9193,	9193,	},
			{	0,	-324,	1121,	2582,	3989,	5293,	6506,	7791,	8793,	8793,	8793,	8793,	8793,	8793,	8793,	},
			{	0,	-482,	957,	2419,	3758,	5066,	6280,	7522,	7522,	7522,	7522,	7522,	7522,	7522,	7522,	},
			{	0,	-718,	757,	2224,	3539,	4858,	6119,	7279,	7279,	7279,	7279,	7279,	7279,	7279,	7279,	},
			{	0,	-864,	622,	2069,	3376,	4701,	5945,	5945,	5945,	5945,	5945,	5945,	5945,	5945,	5945,	},
			{	0,	-983,	489,	1878,	3184,	4511,	5821,	5821,	5821,	5821,	5821,	5821,	5821,	5821,	5821,	},
			{	0,	-1110,	296,	1733,	3042,	4367,	4367,	4367,	4367,	4367,	4367,	4367,	4367,	4367,	4367,	},
			{	0,	-1286,	207,	1571,	2886,	4200,	4200,	4200,	4200,	4200,	4200,	4200,	4200,	4200,	4200,	},

	},
	.IqCmdLutInfo=
	{
		.XLength = 15,
		.XInterval = 1.43836,
		.XMin = 0,
		.YLength = 40,
		.YInterval = 7.44581,
		.YMin = 7.44581,
		.Scale = 0.0165,
		.pTableStart = &MotorTable.IqCmdLutTable[0][0],
	},
	.IqCmdHeader=
	{
		TABLE_TYPE_LUT_INT16_2DIM,
		&MotorTable.IqCmdLutInfo,
	},

	//AcCurrLimit
	.AcCurrLimitLutTable=
	{
		{	0,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211},
		{	0,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211},
		{	0,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211},
		{	0,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211},
		{	0,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211},
		{	0,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211},
		{	0,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211},
		{	0,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211},
		{	0,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211},
		{	0,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211},
		{	0,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211},
		{	0,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211},
		{	0,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211},
		{	0,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211},
		{	0,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211},
		{	0,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211},
		{	0,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211},
		{	0,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211},
		{	0,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211},
		{	0,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211},
		{	0,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211},
		{	0,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211},
		{	0,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211}
	},
	.AcCurrLimitLutInfo=
	{
		.XLength = 27,
		.XInterval = 10,
		.XMin = 0,
		.YLength = 23,
		.YInterval = 7.44581,
		.YMin = 148.916204,
		.Scale = 0.00390625,
		.pTableStart = &MotorTable.AcCurrLimitLutTable[0][0],
	},
	.AcCurrLimitHeader=
	{
		TABLE_TYPE_LUT_INT16_2DIM,
		&MotorTable.AcCurrLimitLutInfo,
	},

	//DcCurrLimit
	.DcCurrLimitLutTable=
	{
		{	0,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211},
		{	0,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211},
		{	0,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211},
		{	0,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211},
		{	0,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211},
		{	0,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211},
		{	0,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211},
		{	0,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211},
		{	0,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211},
		{	0,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211},
		{	0,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211},
		{	0,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211},
		{	0,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211},
		{	0,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211},
		{	0,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211},
		{	0,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211},
		{	0,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211},
		{	0,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211},
		{	0,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211},
		{	0,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211},
		{	0,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211},
		{	0,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211},
		{	0,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211},
		{	0,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211},
		{	0,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211},
		{	0,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211},
		{	0,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211},
		{	0,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211},
		{	0,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211},
		{	0,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211},
		{	0,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211},
		{	0,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211},
		{	0,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211},
		{	0,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211},
		{	0,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211},
		{	0,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211},
		{	0,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211},
		{	0,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211},
		{	0,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211},
		{	0,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211},
		{	0,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211,	5211}
	},
	.DcCurrLimitLutInfo=
	{
		.XLength = 26,
		.XInterval = 10,
		.XMin = 0,
		.YLength = 41,
		.YInterval = 7.44581,
		.YMin = 0,
		.Scale = 0.00390625,
		.pTableStart = &MotorTable.DcCurrLimitLutTable[0][0],
	},
	.DcCurrLimitHeader=
	{
		TABLE_TYPE_LUT_INT16_2DIM,
		&MotorTable.DcCurrLimitLutInfo,
	},

	//MaxTorque
	.MaxTorqueLutTable=
	{5155,	5155,	5155,	4787,	4787,	4787,	4419,	4050,	4050,	3682,	3682,	3314,	3314,	2946,	2946,	2578,	2578,	2209,	2209,	1841,	1841},
	.MaxTorqueLutInfo=
	{
		.Length = 21,
		.Interval = 7.44581,
		.Min = 148.916204,
		.Scale = 0.00390625,
		.pTableStart = &MotorTable.MaxTorqueLutTable[0],
	},
	.MaxTorqueHeader=
	{
		TABLE_TYPE_LUT_INT16_1DIM,
		&MotorTable.MaxTorqueLutInfo,
	},
#endif

	//USER CODE BEGIN DATA
	//USER CODE END DATA

		.MotEncParamTableInfoArray =
		{
				// Property = {10^n[10:8], read location[7:6], size[5], decimal[4:3], Authority[2:0]}. See in ParamTable.h.
				// ============================ P4 MotorEncoderParams ============================
				//  Min,	Max,	Default,	Property,		*pAddr
				{		0,		0,		0,		0,		0},	//P4-00
				{		0,		0,		0,		0,		0},	//P4-01
				{		0,		0,		0,		0,		0},	//P4-02
				{		0,		0,		0,		0,		0},	//P4-03
				{		0,		0,		0,		0,		0},	//P4-04
				{		0,		0,		0,		0,		0},	//P4-05
				{		0,		0,		0,		0,		0},	//P4-06
				{		0,		0,		0,		0,		0},	//P4-07
				{		0,		0,		0,		0,		0},	//P4-08
				{		0,		0,		0,		0,		0},	//P4-09
				{		0,		0,		0,		0,		0},	//P4-10
				{		0,		0,		0,		0,		0},	//P4-11
				{		0,		0,		0,		0,		0},	//P4-12
				{		0,		0,		0,		0,		0},	//P4-13
				{		0,		0,		0,		0,		0},	//P4-14
				{		0,		0,		0,		0,		0},	//P4-15
				{		0,		0,		0,		0,		0},	//P4-16
				{		0,		0,		0,		0,		0},	//P4-17
				{		0,		0,		0,		0,		0},	//P4-18
				{		0,		0,		0,		0,		0},	//P4-19
				{		0,		0,		0,		0,		0},	//P4-20
				{		0,		0,		0,		0,		0},	//P4-21
				{		0,		0,		0,		0,		0},	//P4-22
				{		0,		0,		0,		0,		0},	//P4-23
				{		0,		0,		0,		0,		0},	//P4-24
				{		0,		0,		0,		0,		0},	//P4-25
				{		0,		0,		0,		0,		0},	//P4-26
				{		0,		0,		0,		0,		0},	//P4-27
				{		0,		0,		0,		0,		0},	//P4-28
				{		0,		0,		0,		0,		0},	//P4-29
				{		0,		0,		0,		0,		0},	//P4-30
				{		0,		0,		0,		0,		0},	//P4-31
				{		0,		0,		0,		0,		0},	//P4-32
				{		0,		0,		0,		0,		0},	//P4-33
				{		0,		0,		0,		0,		0},	//P4-34
				{		0,		0,		0,		0,		0},	//P4-35
				{		0,		0,		0,		0,		0},	//P4-36
				{		0,		0,		0,		0,		0},	//P4-37
				{		0,		0,		0,		0,		0},	//P4-38
				{		0,		0,		0,		0,		0},	//P4-39
				{		0,		0,		0,		0,		0},	//P4-40
				{		0,		0,		0,		0,		0},	//P4-41
				{		0,		0,		0,		0,		0},	//P4-42
				{		0,		0,		0,		0,		0},	//P4-43
				{		0,		0,		0,		0,		0},	//P4-44
				{		0,		0,		0,		0,		0},	//P4-45
				{		0,		0,		0,		0,		0},	//P4-46
				{		0,		0,		0,		0,		0},	//P4-47
				{		0,		0,		0,		0,		0},	//P4-48
				{		0,		0,		0,		0,		0},	//P4-49
				{		0,		0,		0,		0,		0},	//P4-50
				{		0,		0,		0,		0,		0},	//P4-51
				{		0,		0,		0,		0,		0},	//P4-52
				{		0,		0,		0,		0,		0},	//P4-53
				{		0,		0,		0,		0,		0},	//P4-54
				{		0,		0,		0,		0,		0},	//P4-55
				{		0,		0,		0,		0,		0},	//P4-56
				{		0,		0,		0,		0,		0},	//P4-57
				{		0,		0,		0,		0,		0},	//P4-58
				{		0,		0,		0,		0,		0},	//P4-59
				{		0,		0,		0,		0,		0},	//P4-60
				{		0,		0,		0,		0,		0},	//P4-61
				{		0,		0,		0,		0,		0},	//P4-62
				{		0,		0,		0,		0,		0},	//P4-63
				{		0,		0,		0,		0,		0},	//P4-64
				{		0,		0,		0,		0,		0},	//P4-65
				{		0,		0,		0,		0,		0},	//P4-66
				{		0,		0,		0,		0,		0},	//P4-67
				{		0,		0,		0,		0,		0},	//P4-68
				{		0,		0,		0,		0,		0},	//P4-69
				{		0,		0,		0,		0,		0},	//P4-70
				{		0,		0,		0,		0,		0},	//P4-71
				{		0,		0,		0,		0,		0},	//P4-72
				{		0,		0,		0,		0,		0},	//P4-73
				{		0,		0,		0,		0,		0},	//P4-74
				{		0,		0,		0,		0,		0},	//P4-75
				{		0,		0,		0,		0,		0},	//P4-76
				{		0,		0,		0,		0,		0},	//P4-77
				{		0,		0,		0,		0,		0},	//P4-78
				{		0,		0,		0,		0,		0},	//P4-79
				{		0,		0,		0,		0,		0},	//P4-80
				{		0,		0,		0,		0,		0},	//P4-81
				{		0,		0,		0,		0,		0},	//P4-82
				{		0,		0,		0,		0,		0},	//P4-83
				{		0,		0,		0,		0,		0},	//P4-84
				{		0,		0,		0,		0,		0},	//P4-85
				{		0,		0,		0,		0,		0},	//P4-86
				{		0,		0,		0,		0,		0},	//P4-87
				{		0,		0,		0,		0,		0},	//P4-88
				{		0,		0,		0,		0,		0},	//P4-89
				{		0,		0,		0,		0,		0},	//P4-90
				{		0,		0,		0,		0,		0},	//P4-91
				{		0,		0,		0,		0,		0},	//P4-92
				{		0,		0,		0,		0,		0},	//P4-93
				{		0,		0,		0,		0,		0},	//P4-94
				{		0,		0,		0,		0,		0},	//P4-95
				{		0,		0,		0,		0,		0},	//P4-96
				{		0,		0,		0,		0,		0},	//P4-97
				{		0,		0,		0,		0,		0},	//P4-98
				{		0,		0,		0,		0,		0},	//P4-99
				{		0,		0,		0,		0,		0},	//P5-00
				{		0,		0,		0,		0,		0},	//P5-01
				{		0,		0,		0,		0,		0},	//P5-02
				{		0,		0,		0,		0,		0},	//P5-03
				{		0,		0,		0,		0,		0},	//P5-04
				{		0,		0,		0,		0,		0},	//P5-05
				{		0,		0,		0,		0,		0},	//P5-06
				{		0,		0,		0,		0,		0},	//P5-07
				{		0,		0,		0,		0,		0},	//P5-08
				{		0,		0,		0,		0,		0},	//P5-09
				{		0,		0,		0,		0,		0},	//P5-10
				{		0,		0,		0,		0,		0},	//P5-11
				{		0,		0,		0,		0,		0},	//P5-12
				{		0,		0,		0,		0,		0},	//P5-13
				{		0,		0,		0,		0,		0},	//P5-14
				{		0,		0,		0,		0,		0},	//P5-15
				{		0,		0,		0,		0,		0},	//P5-16
				{		0,		0,		0,		0,		0},	//P5-17
				{		0,		0,		0,		0,		0},	//P5-18
				{		0,		0,		0,		0,		0},	//P5-19
				{		0,		0,		0,		0,		0},	//P5-20
				{		0,		0,		0,		0,		0},	//P5-21
				{		0,		0,		0,		0,		0},	//P5-22
				{		0,		0,		0,		0,		0},	//P5-23
				{		0,		0,		0,		0,		0},	//P5-24
				{		0,		0,		0,		0,		0},	//P5-25
				{		0,		0,		0,		0,		0},	//P5-26
				{		0,		0,		0,		0,		0},	//P5-27
				{		0,		0,		0,		0,		0},	//P5-28
				{		0,		0,		0,		0,		0},	//P5-29
				{		0,		0,		0,		0,		0},	//P5-30
				{		0,		0,		0,		0,		0},	//P5-31
				{		0,		0,		0,		0,		0},	//P5-32
				{		0,		0,		0,		0,		0},	//P5-33
				{		0,		0,		0,		0,		0},	//P5-34
				{		0,		0,		0,		0,		0},	//P5-35
				{		0,		0,		0,		0,		0},	//P5-36
				{		0,		0,		0,		0,		0},	//P5-37
				{		0,		0,		0,		0,		0},	//P5-38
				{		0,		0,		0,		0,		0},	//P5-39
				{		0,		0,		0,		0,		0},	//P5-40
				{		0,		0,		0,		0,		0},	//P5-41
				{		0,		0,		0,		0,		0},	//P5-42
				{		0,		0,		0,		0,		0},	//P5-43
				{		0,		0,		0,		0,		0},	//P5-44
				{		0,		0,		0,		0,		0},	//P5-45
				{		0,		0,		0,		0,		0},	//P5-46
				{		0,		0,		0,		0,		0},	//P5-47
				{		0,		0,		0,		0,		0},	//P5-48
				{		0,		0,		0,		0,		0},	//P5-49
				{		0,		0,		0,		0,		0},	//P5-50
				{		0,		0,		0,		0,		0},	//P5-51
				{		0,		0,		0,		0,		0},	//P5-52
				{		0,		0,		0,		0,		0},	//P5-53
				{		0,		0,		0,		0,		0},	//P5-54
				{		0,		0,		0,		0,		0},	//P5-55
				{		0,		0,		0,		0,		0},	//P5-56
				{		0,		0,		0,		0,		0},	//P5-57
				{		0,		0,		0,		0,		0},	//P5-58
				{		0,		0,		0,		0,		0},	//P5-59
				{		0,		0,		0,		0,		0},	//P5-60
				{		0,		0,		0,		0,		0},	//P5-61
				{		0,		0,		0,		0,		0},	//P5-62
				{		0,		0,		0,		0,		0},	//P5-63
				{		0,		0,		0,		0,		0},	//P5-64
				{		0,		0,		0,		0,		0},	//P5-65
				{		0,		0,		0,		0,		0},	//P5-66
				{		0,		0,		0,		0,		0},	//P5-67
				{		0,		0,		0,		0,		0},	//P5-68
				{		0,		0,		0,		0,		0},	//P5-69
				{		0,		0,		0,		0,		0},	//P5-70
				{		0,		0,		0,		0,		0},	//P5-71
				{		0,		0,		0,		0,		0},	//P5-72
				{		0,		0,		0,		0,		0},	//P5-73
				{		0,		0,		0,		0,		0},	//P5-74
				{		0,		0,		0,		0,		0},	//P5-75
				{		0,		0,		0,		0,		0},	//P5-76
				{		0,		0,		0,		0,		0},	//P5-77
				{		0,		0,		0,		0,		0},	//P5-78
				{		0,		0,		0,		0,		0},	//P5-79
				{		0,		0,		0,		0,		0},	//P5-80
				{		0,		0,		0,		0,		0},	//P5-81
				{		0,		0,		0,		0,		0},	//P5-82
				{		0,		0,		0,		0,		0},	//P5-83
				{		0,		0,		0,		0,		0},	//P5-84
				{		0,		0,		0,		0,		0},	//P5-85
				{		0,		0,		0,		0,		0},	//P5-86
				{		0,		0,		0,		0,		0},	//P5-87
				{		0,		0,		0,		0,		0},	//P5-88
				{		0,		0,		0,		0,		0},	//P5-89
				{		0,		0,		0,		0,		0},	//P5-90
				{		0,		0,		0,		0,		0},	//P5-91
				{		0,		0,		0,		0,		0},	//P5-92
				{		0,		0,		0,		0,		0},	//P5-93
				{		0,		0,		0,		0,		0},	//P5-94
				{		0,		0,		0,		0,		0},	//P5-95
				{		0,		0,		0,		0,		0},	//P5-96
				{		0,		0,		0,		0,		0},	//P5-97
				{		0,		0,		0,		0,		0},	//P5-98
				{		0,		0,		0,		0,		0},	//P5-99
		},

		.CheckWord = 0x4321
};
#endif
