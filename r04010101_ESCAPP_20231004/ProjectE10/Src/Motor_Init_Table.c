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
			{	0,	0,	0,	-1,	-5,	-40,	-310,	-712,	-1101,	-1319,	-1490,	-1647,	-1801,	-1953,	-2104,	-2256,	-2407,	-2560,	-2716,	-2902,	-3276,	-4365,	-5615,	-6597,	-7065,	-7405,	-7714,	-8014,	-8313,	-8611,	-8909,	-9207,	-9505,	-9803,	-10104,	-10441,	-10786,	-11118,	-11403,	-11538,	},
			{	0,	0,	0,	-1,	-5,	-41,	-311,	-714,	-1103,	-1321,	-1491,	-1648,	-1801,	-1953,	-2104,	-2256,	-2407,	-2560,	-2716,	-2902,	-3276,	-4365,	-5615,	-6597,	-7065,	-7405,	-7714,	-8014,	-8313,	-8611,	-8909,	-9207,	-9539,	-9910,	-10338,	-10804,	-11247,	-11665,	-12046,	-12276,	},
			{	0,	0,	0,	-1,	-5,	-41,	-312,	-715,	-1105,	-1322,	-1491,	-1648,	-1801,	-1967,	-2152,	-2358,	-2573,	-2796,	-3030,	-3300,	-3743,	-4716,	-5827,	-6660,	-7170,	-7570,	-7971,	-8383,	-8800,	-9194,	-9579,	-9970,	-10403,	-10888,	-11455,	-12058,	-12640,	-13146,	-13271,	-13379,	},
			{	-214,	-214,	-220,	-215,	-238,	-306,	-559,	-925,	-1253,	-1481,	-1679,	-1867,	-2068,	-2276,	-2522,	-2800,	-3094,	-3401,	-3726,	-4111,	-4620,	-5455,	-6396,	-7108,	-7682,	-8169,	-8666,	-9184,	-9712,	-10231,	-10743,	-11268,	-11801,	-12443,	-13150,	-13888,	-14587,	-15165,	-15137,	-15000,	},
			{	-877,	-877,	-921,	-971,	-1047,	-1188,	-1430,	-1762,	-2031,	-2280,	-2514,	-2736,	-2987,	-3260,	-3590,	-3943,	-4311,	-4694,	-5115,	-5596,	-6150,	-6811,	-7543,	-8280,	-8903,	-9523,	-10103,	-10707,	-11337,	-11948,	-12593,	-13252,	-13891,	-14574,	-15329,	-16089,	-16831,	-17079,	-16618,	-16618,	},
			{	-2054,	-2054,	-2084,	-2197,	-2313,	-2478,	-2673,	-2940,	-3218,	-3485,	-3742,	-4018,	-4314,	-4656,	-4999,	-5396,	-5777,	-6194,	-6666,	-7171,	-7745,	-8382,	-9093,	-9824,	-10465,	-11105,	-11747,	-12373,	-13023,	-13726,	-14442,	-15123,	-15849,	-16587,	-17373,	-18143,	-18546,	-18450,	-17514,	-17514,	},
			{	-3799,	-3799,	-3719,	-3808,	-3911,	-4066,	-4274,	-4519,	-4795,	-5063,	-5323,	-5630,	-5942,	-6323,	-6689,	-7114,	-7518,	-7973,	-8463,	-8997,	-9576,	-10212,	-10879,	-11551,	-12254,	-12931,	-13625,	-14275,	-14975,	-15735,	-16476,	-17240,	-17967,	-18757,	-19157,	-19574,	-19447,	-18910,	-18910,	-18910,	},
			{	-5561,	-5561,	-5505,	-5564,	-5609,	-5744,	-5942,	-6214,	-6475,	-6725,	-7013,	-7353,	-7692,	-8071,	-8457,	-8851,	-9300,	-9789,	-10294,	-10833,	-11432,	-12078,	-12707,	-13361,	-14074,	-14765,	-15494,	-16233,	-16959,	-17789,	-18573,	-19309,	-20047,	-20484,	-20474,	-20225,	-19575,	-19575,	-19575,	-19575,	},
			{	-7444,	-7444,	-7332,	-7318,	-7320,	-7415,	-7602,	-7851,	-8122,	-8394,	-8667,	-9017,	-9375,	-9754,	-10155,	-10567,	-11018,	-11529,	-12070,	-12625,	-13197,	-13842,	-14502,	-15153,	-15888,	-16638,	-17345,	-18079,	-18868,	-19699,	-20493,	-21277,	-21708,	-21808,	-21131,	-21131,	-21131,	-21131,	-21131,	-21131,	},
			{	-9245,	-9245,	-9056,	-8986,	-8955,	-8997,	-9177,	-9438,	-9671,	-9920,	-10206,	-10544,	-10943,	-11330,	-11714,	-12165,	-12622,	-13179,	-13697,	-14256,	-14868,	-15555,	-16255,	-16884,	-17578,	-18285,	-19037,	-19802,	-20694,	-21535,	-22376,	-22850,	-22937,	-22231,	-22231,	-22231,	-22231,	-22231,	-22231,	-22231,	},
			{	-10475,	-10475,	-10403,	-10274,	-10290,	-10393,	-10579,	-10815,	-11032,	-11235,	-11500,	-11871,	-12227,	-12622,	-13067,	-13539,	-13995,	-14576,	-15114,	-15669,	-16322,	-17014,	-17754,	-18420,	-19128,	-19851,	-20578,	-21363,	-22257,	-23195,	-23761,	-23906,	-23356,	-23356,	-23356,	-23356,	-23356,	-23356,	-23356,	-23356,	},
			{	-12452,	-12452,	-11825,	-11552,	-11595,	-11652,	-11908,	-12156,	-12342,	-12547,	-12824,	-13181,	-13557,	-13968,	-14431,	-14899,	-15418,	-15919,	-16528,	-17086,	-17738,	-18415,	-19176,	-19902,	-20646,	-21403,	-22101,	-22884,	-23786,	-24500,	-24753,	-24290,	-24290,	-24290,	-24290,	-24290,	-24290,	-24290,	-24290,	-24290,	},
			{	-14387,	-14387,	-13208,	-12752,	-12611,	-12761,	-12978,	-13234,	-13423,	-13666,	-13948,	-14311,	-14696,	-15131,	-15611,	-16073,	-16562,	-17074,	-17756,	-18271,	-18911,	-19600,	-20404,	-21178,	-21947,	-22672,	-23399,	-24190,	-25116,	-25500,	-25309,	-25309,	-25309,	-25309,	-25309,	-25309,	-25309,	-25309,	-25309,	-25309,	},
			{	-15795,	-15795,	-14431,	-14002,	-13783,	-13873,	-14075,	-14314,	-14541,	-14765,	-15088,	-15411,	-15804,	-16290,	-16712,	-17180,	-17711,	-18279,	-18896,	-19482,	-20131,	-20831,	-21636,	-22389,	-23223,	-23964,	-24745,	-25346,	-25915,	-25840,	-25840,	-25840,	-25840,	-25840,	-25840,	-25840,	-25840,	-25840,	-25840,	-25840,	},
			{	-16688,	-16688,	-15366,	-14981,	-14973,	-15064,	-15183,	-15337,	-15632,	-15845,	-16146,	-16472,	-16847,	-17304,	-17830,	-18285,	-18828,	-19410,	-20043,	-20639,	-21323,	-22061,	-22907,	-23662,	-24417,	-25214,	-25800,	-26177,	-26302,	-26060,	-26060,	-26060,	-26060,	-26060,	-26060,	-26060,	-26060,	-26060,	-26060,	-26060,	},
			{	-17410,	-17410,	-16346,	-16214,	-16065,	-16035,	-16135,	-16344,	-16590,	-16843,	-17119,	-17480,	-17858,	-18323,	-18859,	-19389,	-19926,	-20479,	-21099,	-21730,	-22477,	-23217,	-24032,	-24859,	-25697,	-26501,	-26889,	-26859,	-26859,	-26859,	-26859,	-26859,	-26859,	-26859,	-26859,	-26859,	-26859,	-26859,	-26859,	-26859,	},
			{	-18417,	-18417,	-17564,	-17182,	-17037,	-16914,	-17079,	-17232,	-17450,	-17738,	-18055,	-18392,	-18834,	-19310,	-19899,	-20385,	-20927,	-21485,	-22047,	-22755,	-23483,	-24398,	-25240,	-26013,	-26793,	-27415,	-27521,	-27521,	-27521,	-27521,	-27521,	-27521,	-27521,	-27521,	-27521,	-27521,	-27521,	-27521,	-27521,	-27521,	},
			{	-19464,	-19464,	-18811,	-17983,	-17759,	-17698,	-17801,	-18009,	-18320,	-18590,	-18950,	-19245,	-19603,	-20085,	-20684,	-21243,	-21754,	-22306,	-22963,	-23686,	-24510,	-25382,	-26171,	-27111,	-27898,	-28268,	-28182,	-28182,	-28182,	-28182,	-28182,	-28182,	-28182,	-28182,	-28182,	-28182,	-28182,	-28182,	-28182,	-28182,	},
			{	-20386,	-20386,	-19492,	-18768,	-18613,	-18526,	-18572,	-18780,	-19099,	-19416,	-19757,	-20052,	-20380,	-20930,	-21542,	-22135,	-22650,	-23205,	-23940,	-24620,	-25465,	-26356,	-27259,	-27952,	-28566,	-28545,	-28545,	-28545,	-28545,	-28545,	-28545,	-28545,	-28545,	-28545,	-28545,	-28545,	-28545,	-28545,	-28545,	-28545,	},
			{	-20804,	-20804,	-20086,	-19491,	-19468,	-19128,	-19271,	-19523,	-19858,	-20160,	-20413,	-20785,	-21177,	-21796,	-22338,	-22869,	-23451,	-24114,	-24776,	-25590,	-26358,	-27165,	-28156,	-28680,	-29016,	-28665,	-28665,	-28665,	-28665,	-28665,	-28665,	-28665,	-28665,	-28665,	-28665,	-28665,	-28665,	-28665,	-28665,	-28665,	},
			{	-21462,	-21462,	-20974,	-20419,	-19984,	-19847,	-20108,	-20231,	-20392,	-20770,	-21092,	-21561,	-21952,	-22607,	-23157,	-23662,	-24228,	-24930,	-25638,	-26460,	-27255,	-28165,	-28831,	-29111,	-29111,	-29111,	-29111,	-29111,	-29111,	-29111,	-29111,	-29111,	-29111,	-29111,	-29111,	-29111,	-29111,	-29111,	-29111,	-29111,	},
			{	-22058,	-22058,	-21664,	-21307,	-20650,	-20635,	-20709,	-21024,	-21248,	-21624,	-22024,	-22398,	-22871,	-23360,	-23897,	-24461,	-25132,	-25789,	-26606,	-27349,	-28242,	-28835,	-29193,	-29347,	-29347,	-29347,	-29347,	-29347,	-29347,	-29347,	-29347,	-29347,	-29347,	-29347,	-29347,	-29347,	-29347,	-29347,	-29347,	-29347,	},
			{	-22658,	-22658,	-22384,	-21832,	-21157,	-21261,	-21309,	-21587,	-22018,	-22415,	-22806,	-23169,	-23549,	-24003,	-24624,	-25110,	-25904,	-26560,	-27383,	-28236,	-29035,	-29440,	-29576,	-29576,	-29576,	-29576,	-29576,	-29576,	-29576,	-29576,	-29576,	-29576,	-29576,	-29576,	-29576,	-29576,	-29576,	-29576,	-29576,	-29576,	},
			{	-22865,	-22865,	-22979,	-22624,	-22135,	-21925,	-22081,	-22325,	-22610,	-23023,	-23414,	-23865,	-24295,	-24744,	-25265,	-25884,	-26580,	-27343,	-28182,	-29049,	-29639,	-29824,	-29824,	-29824,	-29824,	-29824,	-29824,	-29824,	-29824,	-29824,	-29824,	-29824,	-29824,	-29824,	-29824,	-29824,	-29824,	-29824,	-29824,	-29824,	},
			{	-23036,	-23036,	-23532,	-23249,	-22935,	-22600,	-22881,	-23111,	-23225,	-23486,	-23847,	-24417,	-24946,	-25369,	-25863,	-26513,	-27252,	-28029,	-28888,	-29617,	-30069,	-29955,	-29955,	-29955,	-29955,	-29955,	-29955,	-29955,	-29955,	-29955,	-29955,	-29955,	-29955,	-29955,	-29955,	-29955,	-29955,	-29955,	-29955,	-29955,	},
			{	-23160,	-23160,	-24050,	-23887,	-23453,	-23294,	-23542,	-23732,	-23928,	-24136,	-24469,	-25059,	-25586,	-25983,	-26477,	-27246,	-28011,	-28799,	-29689,	-30144,	-30224,	-30224,	-30224,	-30224,	-30224,	-30224,	-30224,	-30224,	-30224,	-30224,	-30224,	-30224,	-30224,	-30224,	-30224,	-30224,	-30224,	-30224,	-30224,	-30224,	},
			{	-23326,	-23326,	-24237,	-24428,	-24038,	-23894,	-24010,	-24334,	-24488,	-24642,	-25082,	-25573,	-25988,	-26498,	-27217,	-27990,	-28801,	-29604,	-30176,	-30441,	-30441,	-30441,	-30441,	-30441,	-30441,	-30441,	-30441,	-30441,	-30441,	-30441,	-30441,	-30441,	-30441,	-30441,	-30441,	-30441,	-30441,	-30441,	-30441,	-30441,	},
			{	-24906,	-24906,	-25139,	-24972,	-24598,	-24452,	-24290,	-24594,	-24866,	-25147,	-25471,	-25947,	-26473,	-27124,	-27883,	-28541,	-29481,	-30187,	-30693,	-30668,	-30668,	-30668,	-30668,	-30668,	-30668,	-30668,	-30668,	-30668,	-30668,	-30668,	-30668,	-30668,	-30668,	-30668,	-30668,	-30668,	-30668,	-30668,	-30668,	-30668,	},
			{	-26712,	-26712,	-26174,	-25548,	-25377,	-24937,	-24736,	-24793,	-25240,	-25688,	-26166,	-26590,	-27095,	-27753,	-28421,	-29244,	-30074,	-30692,	-30856,	-30856,	-30856,	-30856,	-30856,	-30856,	-30856,	-30856,	-30856,	-30856,	-30856,	-30856,	-30856,	-30856,	-30856,	-30856,	-30856,	-30856,	-30856,	-30856,	-30856,	-30856,	},
			{	-27676,	-27676,	-26348,	-25800,	-25605,	-25146,	-24985,	-24917,	-25411,	-25950,	-26503,	-26974,	-27442,	-27948,	-28640,	-29532,	-30279,	-30876,	-30931,	-30931,	-30931,	-30931,	-30931,	-30931,	-30931,	-30931,	-30931,	-30931,	-30931,	-30931,	-30931,	-30931,	-30931,	-30931,	-30931,	-30931,	-30931,	-30931,	-30931,	-30931,	},
			{	-27822,	-27822,	-26364,	-26052,	-25876,	-25419,	-24966,	-25052,	-25633,	-26337,	-26758,	-27143,	-27490,	-28127,	-28877,	-29683,	-30477,	-30755,	-30755,	-30755,	-30755,	-30755,	-30755,	-30755,	-30755,	-30755,	-30755,	-30755,	-30755,	-30755,	-30755,	-30755,	-30755,	-30755,	-30755,	-30755,	-30755,	-30755,	-30755,	-30755,	}

	},
	.IdCmdLutInfo=
	{
		.XLength = 40,
		.XInterval = 0.521911,
		.XMin = 0,
		.YLength = 31,
		.YInterval = 7.44581,
		.YMin = 148.916204,
		.Scale = 0.015625,
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
			{	0,	1322,	2050,	2751,	3453,	4200,	5016,	5827,	6629,	7454,	8288,	9124,	9961,	10798,	11635,	12472,	13309,	14146,	14982,	15813,	16607,	17233,	17761,	18327,	19062,	19843,	20635,	21431,	22228,	23024,	23821,	24618,	25414,	26212,	27016,	27917,	28840,	29728,	30487,	30849,	},
			{	0,	1322,	2050,	2751,	3453,	4200,	5016,	5827,	6629,	7454,	8288,	9124,	9961,	10797,	11635,	12472,	13309,	14146,	14982,	15813,	16607,	17233,	17761,	18327,	19062,	19843,	20635,	21431,	22228,	23024,	23821,	24618,	25402,	26172,	26964,	27824,	28713,	29534,	30247,	30568,	},
			{	0,	1322,	2050,	2751,	3453,	4200,	5016,	5827,	6628,	7454,	8288,	9124,	9961,	10795,	11626,	12453,	13278,	14101,	14922,	15735,	16509,	17141,	17693,	18304,	19022,	19780,	20538,	21322,	22104,	22896,	23660,	24420,	25161,	25883,	26648,	27451,	28276,	29018,	29735,	30088,	},
			{	0,	1399,	2073,	2773,	3504,	4278,	5058,	5830,	6604,	7424,	8252,	9082,	9909,	10735,	11552,	12361,	13166,	13967,	14764,	15543,	16285,	16921,	17526,	18199,	18919,	19642,	20358,	21097,	21833,	22602,	23341,	24069,	24760,	25436,	26160,	26893,	27611,	28323,	29008,	29437,	},
			{	0,	1812,	2389,	3069,	3778,	4545,	5248,	5962,	6701,	7477,	8263,	9052,	9862,	10668,	11456,	12237,	13011,	13779,	14534,	15266,	15967,	16622,	17272,	17946,	18666,	19352,	20021,	20711,	21425,	22148,	22847,	23528,	24185,	24857,	25507,	26202,	26867,	27602,	28384,	28384,	},
			{	0,	2155,	2672,	3335,	4017,	4707,	5359,	6040,	6761,	7496,	8240,	9009,	9773,	10549,	11297,	12049,	12779,	13522,	14241,	14944,	15643,	16314,	16981,	17642,	18315,	18975,	19629,	20294,	20987,	21683,	22369,	23034,	23664,	24327,	24943,	25573,	26265,	27082,	27953,	27953,	},
			{	0,	2348,	2819,	3459,	4111,	4776,	5437,	6108,	6802,	7475,	8188,	8943,	9670,	10390,	11091,	11817,	12531,	13245,	13943,	14616,	15297,	15951,	16593,	17236,	17887,	18513,	19154,	19832,	20520,	21209,	21841,	22478,	23106,	23722,	24379,	25095,	25924,	26795,	26795,	26795,	},
			{	0,	2296,	2840,	3489,	4139,	4783,	5443,	6115,	6768,	7431,	8138,	8846,	9531,	10215,	10902,	11579,	12292,	12981,	13664,	14327,	14987,	15624,	16245,	16846,	17470,	18113,	18758,	19407,	20081,	20746,	21380,	22005,	22624,	23271,	23980,	24881,	25826,	25826,	25826,	25826,	},
			{	0,	2121,	2680,	3330,	3983,	4629,	5301,	5950,	6612,	7269,	7931,	8642,	9321,	9974,	10648,	11318,	12000,	12676,	13330,	13979,	14622,	15245,	15859,	16450,	17081,	17714,	18352,	18974,	19638,	20271,	20896,	21514,	22144,	22867,	23769,	23769,	23769,	23769,	23769,	23769,	},
			{	0,	1924,	2487,	3142,	3812,	4460,	5118,	5774,	6422,	7085,	7754,	8410,	9093,	9749,	10409,	11086,	11735,	12371,	13018,	13665,	14310,	14929,	15538,	16108,	16739,	17380,	18019,	18617,	19260,	19888,	20507,	21126,	21825,	22686,	22686,	22686,	22686,	22686,	22686,	22686,	},
			{	0,	1689,	2266,	2915,	3599,	4263,	4934,	5596,	6237,	6885,	7544,	8215,	8877,	9529,	10170,	10837,	11455,	12104,	12740,	13357,	14006,	14630,	15239,	15817,	16434,	17063,	17691,	18313,	18957,	19570,	20170,	20808,	21644,	21644,	21644,	21644,	21644,	21644,	21644,	21644,	},
			{	0,	1597,	2121,	2748,	3421,	4093,	4760,	5425,	6072,	6723,	7376,	8033,	8678,	9327,	9961,	10599,	11237,	11860,	12496,	13125,	13743,	14358,	14949,	15548,	16171,	16805,	17433,	18068,	18675,	19261,	19890,	20671,	20671,	20671,	20671,	20671,	20671,	20671,	20671,	20671,	},
			{	0,	1519,	2013,	2628,	3277,	3956,	4618,	5278,	5922,	6560,	7219,	7879,	8527,	9154,	9780,	10393,	11028,	11645,	12277,	12889,	13518,	14123,	14719,	15325,	15938,	16573,	17214,	17842,	18448,	19021,	19640,	19640,	19640,	19640,	19640,	19640,	19640,	19640,	19640,	19640,	},
			{	0,	1414,	1918,	2540,	3185,	3847,	4495,	5137,	5785,	6421,	7080,	7718,	8365,	8980,	9613,	10224,	10845,	11449,	12086,	12700,	13320,	13912,	14520,	15130,	15768,	16398,	17017,	17618,	18253,	18866,	18866,	18866,	18866,	18866,	18866,	18866,	18866,	18866,	18866,	18866,	},
			{	0,	1334,	1848,	2459,	3107,	3769,	4409,	5035,	5669,	6304,	6957,	7601,	8236,	8857,	9483,	10078,	10692,	11290,	11919,	12529,	13155,	13755,	14368,	15004,	15620,	16224,	16800,	17425,	18145,	18713,	18713,	18713,	18713,	18713,	18713,	18713,	18713,	18713,	18713,	18713,	},
			{	0,	1271,	1785,	2404,	3044,	3694,	4320,	4947,	5568,	6199,	6847,	7482,	8119,	8740,	9364,	9959,	10553,	11158,	11768,	12388,	12993,	13615,	14240,	14873,	15479,	16067,	16609,	17201,	17201,	17201,	17201,	17201,	17201,	17201,	17201,	17201,	17201,	17201,	17201,	17201,	},
			{	0,	1226,	1754,	2356,	2985,	3631,	4261,	4880,	5496,	6116,	6757,	7384,	8023,	8644,	9251,	9842,	10432,	11033,	11642,	12254,	12870,	13496,	14118,	14751,	15353,	15902,	16423,	16423,	16423,	16423,	16423,	16423,	16423,	16423,	16423,	16423,	16423,	16423,	16423,	16423,	},
			{	0,	1183,	1720,	2312,	2942,	3587,	4210,	4824,	5444,	6060,	6689,	7299,	7931,	8552,	9155,	9738,	10343,	10942,	11559,	12166,	12773,	13397,	14025,	14652,	15244,	15748,	16183,	16183,	16183,	16183,	16183,	16183,	16183,	16183,	16183,	16183,	16183,	16183,	16183,	16183,	},
			{	0,	1153,	1687,	2280,	2911,	3541,	4159,	4779,	5403,	6007,	6627,	7239,	7865,	8479,	9078,	9670,	10277,	10878,	11485,	12094,	12695,	13313,	13940,	14521,	15119,	15639,	15639,	15639,	15639,	15639,	15639,	15639,	15639,	15639,	15639,	15639,	15639,	15639,	15639,	15639,	},
			{	0,	1130,	1656,	2248,	2883,	3506,	4126,	4734,	5363,	5963,	6585,	7199,	7821,	8425,	9019,	9620,	10222,	10826,	11426,	12047,	12640,	13251,	13870,	14397,	14991,	15509,	15509,	15509,	15509,	15509,	15509,	15509,	15509,	15509,	15509,	15509,	15509,	15509,	15509,	15509,	},
			{	0,	1105,	1630,	2240,	2858,	3472,	4094,	4706,	5325,	5921,	6536,	7151,	7766,	8376,	8972,	9564,	10155,	10767,	11378,	11995,	12596,	13202,	13767,	14271,	14271,	14271,	14271,	14271,	14271,	14271,	14271,	14271,	14271,	14271,	14271,	14271,	14271,	14271,	14271,	14271,	},
			{	0,	1080,	1614,	2228,	2833,	3449,	4068,	4674,	5288,	5890,	6500,	7114,	7722,	8335,	8933,	9515,	10104,	10725,	11335,	11951,	12547,	13117,	13658,	14148,	14148,	14148,	14148,	14148,	14148,	14148,	14148,	14148,	14148,	14148,	14148,	14148,	14148,	14148,	14148,	14148,	},
			{	0,	1055,	1598,	2206,	2817,	3427,	4042,	4645,	5251,	5856,	6462,	7072,	7687,	8295,	8890,	9474,	10067,	10680,	11287,	11898,	12491,	13016,	13502,	13502,	13502,	13502,	13502,	13502,	13502,	13502,	13502,	13502,	13502,	13502,	13502,	13502,	13502,	13502,	13502,	13502,	},
			{	0,	1045,	1578,	2184,	2803,	3410,	4023,	4620,	5221,	5827,	6436,	7041,	7646,	8257,	8858,	9449,	10035,	10641,	11245,	11852,	12407,	12868,	12868,	12868,	12868,	12868,	12868,	12868,	12868,	12868,	12868,	12868,	12868,	12868,	12868,	12868,	12868,	12868,	12868,	12868,	},
			{	0,	1036,	1557,	2171,	2792,	3392,	4005,	4603,	5205,	5811,	6414,	7017,	7616,	8222,	8816,	9416,	10012,	10617,	11217,	11768,	12301,	12746,	12746,	12746,	12746,	12746,	12746,	12746,	12746,	12746,	12746,	12746,	12746,	12746,	12746,	12746,	12746,	12746,	12746,	12746,	},
			{	0,	1028,	1535,	2156,	2771,	3369,	3982,	4581,	5183,	5789,	6390,	6993,	7584,	8183,	8780,	9384,	9979,	10583,	11185,	11687,	12154,	12154,	12154,	12154,	12154,	12154,	12154,	12154,	12154,	12154,	12154,	12154,	12154,	12154,	12154,	12154,	12154,	12154,	12154,	12154,	},
			{	0,	1059,	1528,	2140,	2763,	3367,	3962,	4557,	5159,	5764,	6363,	6969,	7563,	8160,	8748,	9342,	9948,	10552,	11116,	11566,	11566,	11566,	11566,	11566,	11566,	11566,	11566,	11566,	11566,	11566,	11566,	11566,	11566,	11566,	11566,	11566,	11566,	11566,	11566,	11566,	},
			{	0,	1093,	1535,	2125,	2754,	3366,	3953,	4544,	5136,	5735,	6342,	6949,	7540,	8134,	8719,	9307,	9909,	10523,	11029,	11417,	11417,	11417,	11417,	11417,	11417,	11417,	11417,	11417,	11417,	11417,	11417,	11417,	11417,	11417,	11417,	11417,	11417,	11417,	11417,	11417,	},
			{	0,	1092,	1544,	2112,	2738,	3360,	3951,	4535,	5125,	5723,	6334,	6934,	7521,	8106,	8692,	9284,	9888,	10456,	10902,	10902,	10902,	10902,	10902,	10902,	10902,	10902,	10902,	10902,	10902,	10902,	10902,	10902,	10902,	10902,	10902,	10902,	10902,	10902,	10902,	10902,	},
			{	0,	1103,	1561,	2118,	2733,	3355,	3951,	4532,	5119,	5715,	6326,	6920,	7506,	8093,	8680,	9270,	9876,	10414,	10795,	10795,	10795,	10795,	10795,	10795,	10795,	10795,	10795,	10795,	10795,	10795,	10795,	10795,	10795,	10795,	10795,	10795,	10795,	10795,	10795,	10795,	},
			{	0,	1141,	1593,	2134,	2728,	3349,	3949,	4529,	5114,	5710,	6312,	6913,	7497,	8081,	8670,	9268,	9864,	10334,	10334,	10334,	10334,	10334,	10334,	10334,	10334,	10334,	10334,	10334,	10334,	10334,	10334,	10334,	10334,	10334,	10334,	10334,	10334,	10334,	10334,	10334,	}

	},
	.IqCmdLutInfo=
	{
		.XLength = 40,
		.XInterval = 0.521911,
		.XMin = 0,
		.YLength = 31,
		.YInterval = 7.44581,
		.YMin = 148.916204,
		.Scale = 0.015625,
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
		{	0,	118,	236,	355,	473,	591,	709,	828,	946,	1063,	1181,	1299,	1416,	1533,	1650,	1767,	1883,	1999,	2114,	2229,	2343,	2457,	2569,	2681,	2792,	2901,	3072},
		{	0,	118,	236,	355,	473,	591,	709,	828,	946,	1063,	1181,	1299,	1416,	1533,	1650,	1767,	1883,	1999,	2114,	2229,	2343,	2457,	2569,	2681,	2792,	2901,	3072},
		{	0,	119,	236,	354,	473,	591,	710,	828,	946,	1063,	1181,	1299,	1416,	1533,	1650,	1767,	1883,	1998,	2114,	2229,	2344,	2457,	2568,	2674,	2774,	2868,	3072},
		{	0,	118,	237,	355,	473,	591,	709,	827,	946,	1064,	1181,	1299,	1416,	1533,	1650,	1767,	1884,	1999,	2112,	2222,	2327,	2427,	2522,	2613,	2700,	2785,	2944},
		{	0,	119,	236,	354,	473,	592,	710,	828,	945,	1063,	1181,	1299,	1417,	1534,	1649,	1761,	1869,	1974,	2074,	2171,	2264,	2354,	2442,	2526,	2608,	2686,	2816},
		{	0,	118,	237,	355,	473,	591,	709,	828,	946,	1064,	1181,	1295,	1406,	1514,	1618,	1719,	1817,	1912,	2005,	2096,	2184,	2269,	2351,	2430,	2507,	2581,	2688},
		{	0,	119,	236,	354,	473,	592,	710,	826,	939,	1050,	1157,	1261,	1363,	1462,	1559,	1654,	1747,	1838,	1927,	2014,	2098,	2179,	2259,	2336,	2411,	2483,	2560},
		{	0,	118,	237,	354,	469,	581,	690,	796,	901,	1004,	1105,	1204,	1301,	1396,	1489,	1581,	1670,	1758,	1844,	1928,	2010,	2089,	2167,	2243,	2316,	2386,	2560},
		{	0,	89,	207,	317,	425,	531,	636,	739,	841,	940,	1038,	1134,	1228,	1321,	1413,	1503,	1591,	1676,	1760,	1843,	1923,	2001,	2078,	2152,	2223,	2292,	2432},
		{	0,	0,	79,	226,	345,	454,	560,	665,	767,	866,	963,	1058,	1151,	1243,	1334,	1423,	1510,	1595,	1678,	1759,	1838,	1916,	1991,	2065,	2135,	2201,	2304},
		{	0,	0,	0,	56,	226,	355,	469,	577,	682,	784,	883,	979,	1073,	1164,	1254,	1343,	1430,	1515,	1597,	1678,	1757,	1833,	1908,	1981,	2052,	2118,	2176},
		{	0,	0,	0,	0,	36,	223,	362,	480,	589,	695,	798,	897,	992,	1085,	1175,	1264,	1351,	1436,	1519,	1600,	1678,	1754,	1828,	1901,	1972,	2039,	2176},
		{	0,	0,	0,	0,	0,	27,	226,	369,	490,	602,	710,	812,	910,	1003,	1093,	1181,	1268,	1355,	1441,	1525,	1605,	1680,	1753,	1824,	1896,	1965,	2048},
		{	0,	0,	0,	0,	0,	0,	38,	235,	382,	505,	617,	723,	825,	924,	1019,	1112,	1201,	1288,	1372,	1454,	1533,	1609,	1682,	1753,	1822,	1891,	2048},
		{	0,	0,	0,	0,	0,	0,	0,	58,	252,	398,	522,	635,	741,	844,	942,	1036,	1126,	1213,	1296,	1377,	1456,	1533,	1610,	1686,	1760,	1825,	1920},
		{	0,	0,	0,	0,	0,	0,	0,	0,	90,	274,	421,	545,	657,	762,	862,	959,	1054,	1144,	1230,	1312,	1391,	1465,	1538,	1610,	1682,	1752,	1920},
		{	0,	0,	0,	0,	0,	0,	0,	0,	0,	127,	307,	447,	569,	681,	787,	887,	983,	1074,	1161,	1245,	1327,	1408,	1487,	1561,	1629,	1694,	1792},
		{	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	174,	344,	480,	598,	709,	813,	913,	1007,	1097,	1182,	1264,	1344,	1422,	1499,	1572,	1640,	1792},
		{	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	224,	384,	515,	632,	740,	843,	940,	1033,	1121,	1204,	1284,	1363,	1439,	1513,	1581,	1664},
		{	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	83,	278,	427,	553,	668,	775,	876,	971,	1060,	1145,	1227,	1307,	1384,	1457,	1522,	1664},
		{	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	157,	334,	472,	594,	707,	812,	909,	1000,	1088,	1172,	1254,	1330,	1399,	1466,	1664},
		{	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	226,	389,	521,	638,	746,	848,	944,	1035,	1119,	1199,	1277,	1352,	1424,	1536},
		{	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	105,	296,	446,	571,	684,	788,	887,	980,	1068,	1150,	1228,	1302,	1373,	1536}
	},
	.AcCurrLimitLutInfo=
	{
		.XLength = 27,
		.XInterval = 10,
		.XMin = 0,
		.YLength = 23,
		.YInterval = 4.9288,
		.YMin = 88.7184,
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
		{	0,	1382,	1936,	2368,	2727,	3072,	3072,	3072,	3072,	3072,	3072,	3072,	3072,	3072,	3072,	3072,	3072,	3072,	3072,	3072,	3072,	3072,	3072,	3072,	3072,	3072},
		{	0,	1133,	1693,	2124,	2477,	2787,	3072,	3072,	3072,	3072,	3072,	3072,	3072,	3072,	3072,	3072,	3072,	3072,	3072,	3072,	3072,	3072,	3072,	3072,	3072,	3072},
		{	0,	939,	1481,	1902,	2256,	2566,	2839,	3072,	3072,	3072,	3072,	3072,	3072,	3072,	3072,	3072,	3072,	3072,	3072,	3072,	3072,	3072,	3072,	3072,	3072,	3072},
		{	0,	788,	1300,	1707,	2055,	2361,	2637,	2885,	3072,	3072,	3072,	3072,	3072,	3072,	3072,	3072,	3072,	3072,	3072,	3072,	3072,	3072,	3072,	3072,	3072,	3072},
		{	0,	670,	1147,	1538,	1875,	2176,	2448,	2697,	2925,	3072,	3072,	3072,	3072,	3072,	3072,	3072,	3072,	3072,	3072,	3072,	3072,	3072,	3072,	3072,	3072,	3072},
		{	0,	579,	1020,	1391,	1716,	2008,	2274,	2520,	2748,	2959,	3072,	3072,	3072,	3072,	3072,	3072,	3072,	3072,	3072,	3072,	3072,	3072,	3072,	3072,	3072,	3072},
		{	0,	506,	913,	1263,	1574,	1857,	2116,	2357,	2582,	2792,	2989,	3072,	3072,	3072,	3072,	3072,	3072,	3072,	3072,	3072,	3072,	3072,	3072,	3072,	3072,	3072},
		{	0,	448,	824,	1153,	1450,	1721,	1973,	2208,	2428,	2636,	2831,	3072,	3072,	3072,	3072,	3072,	3072,	3072,	3072,	3072,	3072,	3072,	3072,	3072,	3072,	3072},
		{	0,	402,	748,	1058,	1340,	1600,	1843,	2071,	2286,	2490,	2683,	2866,	3072,	3072,	3072,	3072,	3072,	3072,	3072,	3072,	3072,	3072,	3072,	3072,	3072,	3072},
		{	0,	363,	684,	975,	1243,	1492,	1726,	1946,	2155,	2354,	2544,	2724,	2897,	3072,	3072,	3072,	3072,	3072,	3072,	3072,	3072,	3072,	3072,	3072,	3072,	3072},
		{	0,	331,	629,	903,	1157,	1395,	1620,	1833,	2035,	2229,	2414,	2592,	2762,	2925,	3072,	3072,	3072,	3072,	3072,	3072,	3072,	3072,	3072,	3072,	3072,	3072},
		{	0,	304,	581,	839,	1080,	1308,	1524,	1729,	1925,	2113,	2294,	2468,	2635,	2795,	2950,	3072,	3072,	3072,	3072,	3072,	3072,	3072,	3072,	3072,	3072,	3072},
		{	0,	280,	540,	783,	1012,	1230,	1437,	1634,	1824,	2007,	2182,	2352,	2516,	2674,	2826,	2973,	3072,	3072,	3072,	3072,	3072,	3072,	3072,	3072,	3072,	3072},
		{	0,	260,	504,	733,	951,	1159,	1358,	1548,	1732,	1908,	2079,	2244,	2404,	2559,	2709,	2854,	2994,	3072,	3072,	3072,	3072,	3072,	3072,	3072,	3072,	3072},
		{	0,	243,	472,	689,	897,	1095,	1286,	1469,	1646,	1818,	1983,	2144,	2300,	2451,	2598,	2741,	2879,	3072,	3072,	3072,	3072,	3072,	3072,	3072,	3072,	3072},
		{	0,	228,	444,	650,	847,	1037,	1220,	1397,	1568,	1734,	1894,	2051,	2203,	2350,	2494,	2634,	2770,	2903,	3072,	3072,	3072,	3072,	3072,	3072,	3072,	3072},
		{	0,	214,	418,	614,	803,	985,	1161,	1331,	1496,	1656,	1812,	1964,	2112,	2256,	2396,	2533,	2667,	2798,	2925,	3072,	3072,	3072,	3072,	3072,	3072,	3072},
		{	0,	202,	396,	582,	763,	937,	1106,	1270,	1429,	1584,	1735,	1883,	2027,	2167,	2304,	2439,	2570,	2698,	2823,	2945,	3072,	3072,	3072,	3072,	3072,	3072},
		{	0,	191,	375,	553,	726,	893,	1055,	1214,	1367,	1518,	1664,	1807,	1947,	2084,	2218,	2349,	2477,	2603,	2726,	2846,	2963,	3072,	3072,	3072,	3072,	3072},
		{	0,	181,	357,	527,	692,	853,	1009,	1162,	1310,	1456,	1598,	1737,	1873,	2006,	2137,	2265,	2390,	2513,	2634,	2752,	2868,	2981,	3072,	3072,	3072,	3072},
		{	0,	173,	340,	503,	661,	816,	967,	1114,	1258,	1398,	1536,	1671,	1803,	1933,	2060,	2185,	2308,	2428,	2546,	2660,	2770,	2874,	3072,	3072,	3072,	3072},
		{	0,	165,	325,	481,	633,	782,	927,	1069,	1208,	1345,	1478,	1609,	1738,	1865,	1989,	2110,	2228,	2343,	2453,	2559,	2662,	2761,	2855,	2944,	2944,	2944},
		{	0,	157,	311,	461,	607,	750,	891,	1028,	1162,	1295,	1425,	1552,	1676,	1798,	1916,	2031,	2143,	2250,	2355,	2456,	2553,	2647,	2738,	2816,	2816,	2816},
		{	0,	151,	298,	442,	583,	721,	857,	990,	1120,	1248,	1372,	1494,	1613,	1728,	1841,	1950,	2056,	2159,	2259,	2355,	2448,	2538,	2625,	2688,	2688,	2688},
		{	0,	145,	286,	425,	561,	694,	825,	953,	1078,	1200,	1319,	1435,	1548,	1658,	1765,	1870,	1972,	2070,	2166,	2259,	2349,	2435,	2518,	2560,	2560,	2560},
		{	0,	139,	275,	409,	539,	667,	792,	914,	1034,	1150,	1264,	1376,	1484,	1590,	1693,	1793,	1891,	1986,	2078,	2168,	2255,	2338,	2419,	2560,	2560,	2560},
		{	0,	133,	264,	391,	516,	639,	759,	876,	990,	1102,	1211,	1318,	1422,	1524,	1624,	1720,	1815,	1907,	1996,	2082,	2166,	2247,	2326,	2432,	2432,	2432},
		{	0,	125,	250,	373,	492,	610,	725,	837,	947,	1055,	1160,	1263,	1364,	1462,	1558,	1651,	1743,	1831,	1918,	2002,	2083,	2162,	2238,	2304,	2304,	2304},
		{	0,	115,	235,	353,	468,	581,	692,	800,	906,	1010,	1111,	1211,	1308,	1403,	1496,	1586,	1675,	1761,	1845,	1926,	2005,	2082,	2156,	2176,	2176,	2176},
		{	0,	104,	220,	333,	444,	553,	659,	764,	866,	967,	1065,	1161,	1255,	1347,	1437,	1525,	1611,	1694,	1776,	1855,	1932,	2007,	2079,	2176,	2176,	2176},
		{	0,	93,	204,	314,	421,	525,	629,	730,	829,	926,	1020,	1113,	1204,	1293,	1381,	1467,	1551,	1632,	1712,	1788,	1864,	1937,	2007,	2048,	2048,	2048},
		{	0,	81,	189,	294,	398,	499,	599,	697,	792,	886,	979,	1069,	1158,	1245,	1330,	1414,	1495,	1574,	1651,	1726,	1799,	1870,	1940,	2048,	2048,	2048},
		{	0,	70,	174,	275,	376,	474,	571,	665,	758,	850,	939,	1027,	1113,	1198,	1280,	1361,	1439,	1516,	1592,	1667,	1740,	1810,	1870,	1920,	1920,	1920},
		{	0,	58,	159,	257,	354,	450,	544,	636,	726,	815,	902,	987,	1071,	1154,	1234,	1313,	1390,	1464,	1537,	1608,	1679,	1748,	1815,	1920,	1920,	1920},
		{	0,	47,	144,	240,	334,	427,	518,	607,	695,	782,	867,	950,	1031,	1111,	1190,	1267,	1343,	1417,	1490,	1560,	1628,	1694,	1760,	1792,	1792,	1792},
		{	0,	36,	131,	223,	315,	405,	494,	581,	666,	750,	833,	914,	994,	1072,	1149,	1224,	1297,	1370,	1441,	1511,	1578,	1643,	1792,	1792,	1792,	1792},
		{	0,	25,	117,	207,	296,	384,	470,	555,	639,	720,	801,	880,	958,	1035,	1110,	1183,	1255,	1326,	1395,	1463,	1530,	1593,	1664,	1664,	1664,	1664},
		{	0,	15,	104,	192,	279,	364,	448,	531,	612,	692,	771,	849,	925,	999,	1073,	1144,	1215,	1284,	1352,	1419,	1483,	1544,	1664,	1664,	1664,	1664},
		{	0,	4,	92,	178,	262,	346,	428,	508,	587,	666,	743,	818,	893,	966,	1037,	1108,	1177,	1245,	1312,	1376,	1438,	1500,	1664,	1664,	1664,	1664},
		{	0,	0,	80,	164,	247,	328,	408,	486,	564,	641,	716,	790,	862,	934,	1004,	1074,	1141,	1207,	1272,	1336,	1399,	1460,	1536,	1536,	1536,	1536},
		{	0,	0,	70,	151,	231,	311,	389,	466,	542,	616,	690,	763,	834,	904,	973,	1040,	1106,	1172,	1236,	1298,	1358,	1418,	1536,	1536,	1536,	1536}
	},
	.DcCurrLimitLutInfo=
	{
		.XLength = 26,
		.XInterval = 10,
		.XMin = 0,
		.YLength = 41,
		.YInterval = 4.9288,
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
	{5211,	5211,	5211,	5211,	5077,	5077,	4944,	4810,	4543,	4409,	4275,	4142,	4008,	3875,	3875,	3607,	3474,	3474,	3340,	3340,	3073,	3073,	2939,	2806,	2806,	2672,	2539,	2539,	2405,	2405,	2271},
	.MaxTorqueLutInfo=
	{
		.Length = 31,
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
