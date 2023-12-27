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
		{ 0,	0	    ,0	    , 0	    ,0	    ,-14	,-96	,-352	,-790	,-1134	,-1465	,-1645	,-1801	,-1953	,-2105	,-2256	,-2407	,-2559	,-2710	,-2862	,-3013	,-3164	,-3329	,-3612	,-4138	,-5431	,-6832	,-7800	,-8269	,-8603	,-8908	,-9207	,-9505	,-9803	,-10101	,-10399	,-10697	,-10995	,-11277	,-11414	},
		{ 0,	0	    ,0	    , 0	    ,0	    ,-14	,-96	,-352	,-790	,-1134	,-1465	,-1645	,-1801	,-1953	,-2105	,-2256	,-2407	,-2559	,-2710	,-2862	,-3013	,-3164	,-3329	,-3612	,-4138	,-5431	,-6832	,-7800	,-8269	,-8603	,-8908	,-9207	,-9505	,-9803	,-10101	,-10399	,-10697	,-10995	,-11277	,-11414	},
		{ 0,	-113	,-44	,-16	,-6	    ,-10	,-99	,-466	,-952	,-1332	,-1549	,-1715	,-1861	,-2017	,-2150	,-2294	,-2411	,-2559	,-2710	,-2862	,-3013	,-3173	,-3456	,-3980	,-5255	,-6651	,-7583	,-8009	,-8323	,-8611	,-8909	,-9201	,-9458	,-9772	,-10177	,-10515	,-10834	,-11138	,-11419	,-11558	},
		{ 0,	-105	,-42	,-15	,-6	    ,-12	,-107	,-477	,-955	,-1332	,-1548	,-1717	,-1872	,-2021	,-2154	,-2252	,-2412	,-2560	,-2710	,-2862	,-3014	,-3228	,-3583	,-4176	,-5409	,-6721	,-7615	,-7990	,-8278	,-8557	,-8902	,-9272	,-9722	,-10202	,-10669	,-11079	,-11539	,-11940	,-12400	,-12662	},
		{ 0,	-97	    ,-39	,-15	,-14	,-55	,-187	,-559	,-998	,-1337	,-1549	,-1739	,-1945	,-2156	,-2385	,-2655	,-2925	,-3212	,-3478	,-3749	,-4035	,-4442	,-4966	,-5579	,-6538	,-7509	,-8179	,-8642	,-9063	,-9569	,-10000	,-10573	,-11170	,-11771	,-12319	,-12944	,-13518	,-14163	,-14314	,-14428	},
		{ 0,	-1122	,-1165	,-1241	,-1326	,-1482	,-1665	,-1959	,-2325	,-2596	,-2863	,-3128	,-3449	,-3764	,-4133	,-4523	,-4930	,-5350	,-5751	,-6187	,-6650	,-7183	,-7733	,-8308	,-9009	,-9681	,-10349	,-10951	,-11601	,-12196	,-12809	,-13419	,-14100	,-14722	,-15368	,-16041	,-16885	,-16554	,-16239	,-16239	},
		{ 0,	-3059	,-3072	,-3181	,-3319	,-3453	,-3690	,-3929	,-4225	,-4507	,-4780	,-5130	,-5463	,-5879	,-6267	,-6747	,-7153	,-7625	,-8108	,-8621	,-9132	,-9656	,-10196	,-10779	,-11441	,-12127	,-12748	,-13404	,-14064	,-14755	,-15405	,-16085	,-16799	,-17452	,-18101	,-18421	,-18653	,-18392	,-18392	,-18392	},
		{ 0,	-4264	,-4333	,-4394	,-4520	,-4708	,-4900	,-5196	,-5447	,-5703	,-6001	,-6325	,-6723	,-7100	,-7578	,-7995	,-8458	,-8902	,-9424	,-9962	,-10509	,-11042	,-11582	,-12199	,-12855	,-13481	,-14102	,-14778	,-15450	,-16118	,-16793	,-17569	,-18297	,-18939	,-19158	,-19346	,-18914	,-18914	,-18914	,-18914	},
		{ 0,	-6381	,-6496	,-6492	,-6623	,-6762	,-6953	,-7176	,-7438	,-7703	,-8053	,-8362	,-8742	,-9156	,-9580	,-10086	,-10506	,-11023	,-11554	,-12147	,-12694	,-13206	,-13778	,-14397	,-15060	,-15730	,-16371	,-17042	,-17695	,-18412	,-19234	,-20015	,-20820	,-20618	,-20321	,-20321	,-20321	,-20321	,-20321	,-20321	},
		{ 0,	-8354	,-8346	,-8415	,-8420	,-8561	,-8707	,-8945	,-9203	,-9525	,-9859	,-10190	,-10511	,-10927	,-11417	,-11881	,-12429	,-12915	,-13474	,-14010	,-14568	,-15122	,-15680	,-16357	,-17059	,-17734	,-18407	,-19110	,-19825	,-20525	,-21352	,-22241	,-22520	,-22218	,-22218	,-22218	,-22218	,-22218	,-22218	,-22218	},
		{ 0,	-9909	,-9675	,-9621	,-9677	,-9776	,-9896	,-10157	,-10435	,-10781	,-11088	,-11422	,-11811	,-12199	,-12647	,-13168	,-13679	,-14194	,-14710	,-15322	,-15904	,-16445	,-17028	,-17664	,-18385	,-19116	,-19844	,-20610	,-21319	,-22103	,-22892	,-23301	,-23183	,-23183	,-23183	,-23183	,-23183	,-23183	,-23183	,-23183	},
		{ 0,	-12016	,-11937	,-11784	,-11803	,-11800	,-11922	,-12190	,-12575	,-12843	,-13265	,-13526	,-13905	,-14223	,-14682	,-15214	,-15776	,-16274	,-16827	,-17386	,-18064	,-18685	,-19234	,-19955	,-20673	,-21465	,-22169	,-22993	,-23803	,-24327	,-24326	,-23951	,-23951	,-23951	,-23951	,-23951	,-23951	,-23951	,-23951	,-23951	},
		{ 0,	-13301	,-13125	,-13206	,-13066	,-13012	,-13120	,-13468	,-13754	,-14192	,-14533	,-14858	,-15071	,-15484	,-15948	,-16539	,-17039	,-17653	,-18170	,-18795	,-19397	,-20061	,-20662	,-21372	,-22193	,-22912	,-23629	,-24383	,-25197	,-25401	,-25401	,-25401	,-25401	,-25401	,-25401	,-25401	,-25401	,-25401	,-25401	,-25401	},
		{ 0,	-15613	,-14815	,-14299	,-14286	,-14303	,-14441	,-14781	,-14974	,-15293	,-15594	,-15975	,-16417	,-16792	,-17265	,-17735	,-18353	,-18879	,-19461	,-20043	,-20733	,-21299	,-22034	,-22844	,-23606	,-24244	,-25010	,-25308	,-25715	,-25587	,-25587	,-25587	,-25587	,-25587	,-25587	,-25587	,-25587	,-25587	,-25587	,-25587	},
		{ 0,	-17580	,-16077	,-15613	,-15339	,-15427	,-15679	,-15904	,-16187	,-16369	,-16755	,-17165	,-17635	,-18069	,-18447	,-18940	,-19509	,-20092	,-20665	,-21318	,-21975	,-22756	,-23449	,-24263	,-24999	,-25719	,-26129	,-26268	,-26268	,-26268	,-26268	,-26268	,-26268	,-26268	,-26268	,-26268	,-26268	,-26268	,-26268	,-26268	},
		{ 0,	-18306	,-16950	,-16278	,-16127	,-16130	,-16329	,-16622	,-16941	,-17238	,-17533	,-17948	,-18392	,-18821	,-19307	,-19862	,-20491	,-21019	,-21604	,-22360	,-22960	,-23632	,-24442	,-25372	,-26172	,-26760	,-26942	,-26942	,-26942	,-26942	,-26942	,-26942	,-26942	,-26942	,-26942	,-26942	,-26942	,-26942	,-26942	,-26942	},
		{ 0,	-18922	,-18084	,-17776	,-17724	,-17769	,-17679	,-17948	,-18433	,-18659	,-18947	,-19338	,-19760	,-20220	,-20800	,-21501	,-22016	,-22604	,-23264	,-23895	,-24627	,-25405	,-26145	,-27031	,-27464	,-27776	,-27568	,-27568	,-27568	,-27568	,-27568	,-27568	,-27568	,-27568	,-27568	,-27568	,-27568	,-27568	,-27568	,-27568	},
		{ 0,	-19745	,-18866	,-19019	,-18894	,-18649	,-18561	,-18867	,-19343	,-19632	,-19832	,-20287	,-20736	,-21154	,-21848	,-22487	,-23048	,-23653	,-24228	,-24920	,-25790	,-26531	,-27270	,-27772	,-28051	,-28051	,-28051	,-28051	,-28051	,-28051	,-28051	,-28051	,-28051	,-28051	,-28051	,-28051	,-28051	,-28051	,-28051	,-28051	},
		{ 0,	-21450	,-20694	,-19703	,-19650	,-19459	,-19746	,-19892	,-20264	,-20536	,-20876	,-21285	,-21714	,-22250	,-22781	,-23418	,-23907	,-24682	,-25280	,-26060	,-26785	,-27566	,-28052	,-28100	,-28100	,-28100	,-28100	,-28100	,-28100	,-28100	,-28100	,-28100	,-28100	,-28100	,-28100	,-28100	,-28100	,-28100	,-28100	,-28100	},
		{ 0,	-22825	,-21511	,-20605	,-20455	,-20758	,-20742	,-20983	,-21148	,-21510	,-21795	,-22077	,-22629	,-23172	,-23732	,-24237	,-24865	,-25541	,-26359	,-26984	,-27715	,-28421	,-28765	,-28765	,-28765	,-28765	,-28765	,-28765	,-28765	,-28765	,-28765	,-28765	,-28765	,-28765	,-28765	,-28765	,-28765	,-28765	,-28765	,-28765	},
		{ 0,	-23090	,-22150	,-21474	,-21406	,-21250	,-21423	,-21549	,-21857	,-22032	,-22296	,-22720	,-23276	,-23836	,-24346	,-24864	,-25651	,-26381	,-27143	,-27789	,-28503	,-29013	,-29246	,-29246	,-29246	,-29246	,-29246	,-29246	,-29246	,-29246	,-29246	,-29246	,-29246	,-29246	,-29246	,-29246	,-29246	,-29246	,-29246	,-29246	},
		{ 0,	-23437	,-22976	,-22464	,-22209	,-22016	,-22325	,-22442	,-22725	,-23181	,-23615	,-24033	,-24471	,-24822	,-25305	,-25975	,-26685	,-27577	,-28225	,-28996	,-29396	,-29658	,-29658	,-29658	,-29658	,-29658	,-29658	,-29658	,-29658	,-29658	,-29658	,-29658	,-29658	,-29658	,-29658	,-29658	,-29658	,-29658	,-29658	,-29658	},
		{ 0,	-23631	,-23020	,-22832	,-22540	,-22499	,-22486	,-22845	,-23284	,-23716	,-24164	,-24605	,-24957	,-25299	,-25673	,-26340	,-27122	,-27916	,-28654	,-29297	,-29676	,-29676	,-29676	,-29676	,-29676	,-29676	,-29676	,-29676	,-29676	,-29676	,-29676	,-29676	,-29676	,-29676	,-29676	,-29676	,-29676	,-29676	,-29676	,-29676	}
	},
	.IdCmdLutInfo=
	{
		.XLength = 40,
		.XInterval = 0.564442,
		.XMin = 0,
		.YLength = 23,
		.YInterval = 7.44581,
		.YMin = 134.02458,
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
		{	0,	1303,	2063,	2895,	3743,	4593,	5427,	6185,	6852,	7568,	8304,	9126,	9961,	10798,	11635,	12472,	13309,	14146,	14983,	15820,	16657,	17494,	18329,	19141,	19904,	20472,	20944,	21510,	22244,	23027,	23822,	24618,	25414,	26211,	27008,	27804,	28601,	29398,	30150,	30519},
		{	0,	1303,	2063,	2895,	3743,	4593,	5427,	6185,	6852,	7568,	8304,	9126,	9961,	10798,	11635,	12472,	13309,	14146,	14983,	15820,	16657,	17494,	18329,	19141,	19904,	20472,	20944,	21510,	22244,	23027,	23822,	24618,	25414,	26211,	27008,	27804,	28601,	29398,	30150,	30519},
		{	0,	1419,	2097,	2905,	3745,	4594,	5443,	6277,	7081,	7883,	8707,	9535,	10347,	11091,	11788,	12537,	13318,	14147,	14983,	15820,	16657,	17498,	18417,	19323,	19997,	20490,	20992,	21598,	22299,	23034,	23822,	24621,	25491,	26412,	27328,	28171,	28987,	29785,	30533,	30904},
		{	0,	1426,	2101,	2906,	3746,	4594,	5443,	6276,	7081,	7883,	8708,	9538,	10360,	11167,	11872,	12575,	13329,	14149,	14983,	15820,	16658,	17544,	18473,	19370,	19991,	20495,	21065,	21697,	22348,	23066,	23827,	24651,	25518,	26399,	27187,	27974,	28722,	29477,	30200,	30542},
		{	0,	1429,	2102,	2907,	3746,	4593,	5440,	6269,	7075,	7883,	8709,	9536,	10358,	11163,	11900,	12615,	13390,	14182,	14989,	15799,	16611,	17457,	18293,	19052,	19672,	20239,	20879,	21525,	22170,	22893,	23682,	24489,	25260,	25973,	26691,	27404,	28137,	28830,	29515,	29895},
		{	0,	1921,	2515,	3255,	4016,	4801,	5539,	6329,	7105,	7868,	8671,	9478,	10265,	11044,	11795,	12535,	13274,	14017,	14772,	15514,	16252,	16970,	17683,	18378,	19097,	19777,	20438,	21069,	21718,	22459,	23220,	23946,	24626,	25278,	25955,	26667,	27380,	28133,	28775,	28775},
		{	0,	2045,	2650,	3372,	4080,	4771,	5543,	6279,	7030,	7741,	8501,	9273,	10009,	10790,	11498,	12241,	12940,	13644,	14346,	15082,	15778,	16462,	17139,	17848,	18567,	19277,	19963,	20617,	21272,	21971,	22706,	23370,	24047,	24713,	25381,	26115,	26795,	27484,	27484,	27484},
		{	0,	2051,	2675,	3354,	4073,	4792,	5503,	6256,	6941,	7656,	8401,	9138,	9885,	10597,	11346,	12037,	12742,	13411,	14134,	14859,	15582,	16259,	16932,	17616,	18302,	19005,	19703,	20399,	21060,	21731,	22404,	23102,	23787,	24471,	25111,	25829,	26647,	26647,	26647,	26647},
		{	0,	1954,	2617,	3308,	4023,	4721,	5426,	6134,	6852,	7553,	8287,	8977,	9691,	10401,	11089,	11822,	12515,	13183,	13849,	14584,	15260,	15950,	16591,	17259,	17961,	18665,	19341,	20036,	20700,	21358,	22044,	22728,	23440,	24100,	24755,	24755,	24755,	24755,	24755,	24755},
		{	0,	1698,	2325,	3022,	3717,	4435,	5145,	5879,	6589,	7298,	7994,	8700,	9414,	10099,	10797,	11525,	12242,	12887,	13535,	14217,	14944,	15604,	16258,	16945,	17643,	18308,	18969,	19659,	20349,	21004,	21676,	22385,	22999,	23638,	23638,	23638,	23638,	23638,	23638,	23638},
		{	0,	1651,	2245,	2939,	3655,	4374,	5082,	5796,	6520,	7211,	7907,	8614,	9335,	10014,	10697,	11394,	12091,	12724,	13408,	14085,	14792,	15476,	16119,	16793,	17470,	18145,	18816,	19520,	20165,	20858,	21530,	22168,	22787,	22787,	22787,	22787,	22787,	22787,	22787,	22787},
		{	0,	1454,	2079,	2772,	3497,	4192,	4904,	5632,	6362,	7049,	7772,	8454,	9154,	9815,	10496,	11184,	11845,	12516,	13186,	13870,	14543,	15225,	15855,	16551,	17231,	17896,	18551,	19242,	19910,	20517,	21215,	21914,	21914,	21914,	21914,	21914,	21914,	21914,	21914,	21914},
		{	0,	1348,	1970,	2669,	3371,	4063,	4768,	5500,	6214,	6937,	7631,	8332,	8990,	9682,	10350,	11041,	11727,	12396,	13058,	13740,	14402,	15045,	15709,	16390,	17059,	17714,	18372,	19064,	19730,	20227,	20227,	20227,	20227,	20227,	20227,	20227,	20227,	20227,	20227,	20227},
		{	0,	1368,	1957,	2625,	3327,	4035,	4747,	5471,	6166,	6865,	7561,	8255,	8950,	9611,	10318,	10987,	11654,	12320,	12978,	13644,	14319,	14972,	15637,	16335,	16990,	17616,	18293,	18942,	19592,	20062,	20062,	20062,	20062,	20062,	20062,	20062,	20062,	20062,	20062,	20062},
		{	0,	1345,	1926,	2584,	3257,	3964,	4679,	5395,	6101,	6778,	7462,	8160,	8851,	9532,	10196,	10870,	11532,	12197,	12872,	13535,	14203,	14879,	15561,	16249,	16884,	17515,	18099,	18648,	18648,	18648,	18648,	18648,	18648,	18648,	18648,	18648,	18648,	18648,	18648,	18648},
		{	0,	1323,	1906,	2563,	3242,	3943,	4657,	5363,	6056,	6739,	7430,	8131,	8808,	9487,	10158,	10812,	11489,	12177,	12852,	13512,	14175,	14845,	15519,	16193,	16841,	17462,	17946,	17946,	17946,	17946,	17946,	17946,	17946,	17946,	17946,	17946,	17946,	17946,	17946,	17946},
		{	0,	1278,	1858,	2512,	3204,	3913,	4622,	5315,	5997,	6684,	7371,	8063,	8729,	9418,	10074,	10731,	11410,	12098,	12768,	13422,	14073,	14752,	15398,	16055,	16657,	17267,	17657,	17657,	17657,	17657,	17657,	17657,	17657,	17657,	17657,	17657,	17657,	17657,	17657,	17657},
		{	0,	1265,	1832,	2496,	3182,	3893,	4584,	5272,	5961,	6649,	7330,	8012,	8690,	9356,	10032,	10678,	11364,	12035,	12688,	13332,	13993,	14648,	15310,	15869,	16376,	16376,	16376,	16376,	16376,	16376,	16376,	16376,	16376,	16376,	16376,	16376,	16376,	16376,	16376,	16376},
		{	0,	1230,	1826,	2482,	3174,	3880,	4576,	5253,	5949,	6636,	7318,	7983,	8653,	9325,	9985,	10645,	11313,	11976,	12646,	13298,	13967,	14617,	15182,	15694,	15694,	15694,	15694,	15694,	15694,	15694,	15694,	15694,	15694,	15694,	15694,	15694,	15694,	15694,	15694,	15694},
		{	0,	1215,	1824,	2471,	3154,	3855,	4542,	5234,	5911,	6599,	7268,	7940,	8603,	9260,	9926,	10585,	11237,	11918,	12593,	13253,	13898,	14539,	14963,	14963,	14963,	14963,	14963,	14963,	14963,	14963,	14963,	14963,	14963,	14963,	14963,	14963,	14963,	14963,	14963,	14963},
		{	0,	1213,	1810,	2460,	3160,	3846,	4531,	5210,	5884,	6573,	7247,	7919,	8584,	9243,	9894,	10552,	11223,	11904,	12561,	13217,	13859,	14403,	14759,	14759,	14759,	14759,	14759,	14759,	14759,	14759,	14759,	14759,	14759,	14759,	14759,	14759,	14759,	14759,	14759,	14759},
		{	0,	1205,	1783,	2446,	3140,	3828,	4502,	5178,	5855,	6531,	7206,	7876,	8551,	9202,	9852,	10509,	11180,	11858,	12512,	13162,	13720,	14158,	14158,	14158,	14158,	14158,	14158,	14158,	14158,	14158,	14158,	14158,	14158,	14158,	14158,	14158,	14158,	14158,	14158,	14158},
		{	0,	1258,	1805,	2457,	3125,	3813,	4493,	5159,	5833,	6501,	7174,	7855,	8516,	9174,	9831,	10489,	11158,	11834,	12486,	13119,	13587,	13587,	13587,	13587,	13587,	13587,	13587,	13587,	13587,	13587,	13587,	13587,	13587,	13587,	13587,	13587,	13587,	13587,	13587,	13587}
	},
	.IqCmdLutInfo=
	{
		.XLength = 40,
		.XInterval = 0.564442,
		.XMin = 0,
		.YLength = 23,
		.YInterval = 7.44581,
		.YMin = 134.02458,
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
	{5635,	5635,	5635,	5635,	5635,	5491,	5346,	5202,	4913,	4768,	4624,	4479,	4190,	4190,	3901,	3757,	3757,	3468,	3323,	3179,	3179,	3034,	2890},
	.MaxTorqueLutInfo=
	{
		.Length = 23,
		.Interval = 7.44581,
		.Min = 134.02458,
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
