/*
 * CurrentCalibration.c
 *
 *  Created on: 2020年9月4日
 *      Author: Hank.Chen.CHC
 */


#include "CurrentCalibration.h"

const MF_CURR_CALIB_SETUP PhaseOption[12] = {
		{ 0, 0, 1 },		//0
		{ 0, 0, 0 },		//1
		{ 0, 1, 1 },		//2
		{ 0, 1, 0 },		//3
		{ 0, 2, 1 },		//4
		{ 0, 2, 0 },		//5
		{ 1, 0, 1 },		//6
		{ 1, 0, 0 },		//7
		{ 1, 1, 1 },		//8
		{ 1, 1, 0 },		//9
		{ 1, 2, 1 },		//10
		{ 1, 2, 0 },		//11
};

void ReadCtrlModeAndAuthority( CURR_CALIB_DEFINE *v, uint16_t Mode, uint16_t Auth )
{
	v->Authority = Auth;
	v->CtrlMode  = Mode;
}

void ReadCmdInfo( CURR_CALIB_DEFINE *v, uint32_t Setup, uint32_t *Enable )
{
	if( ( v->Authority < Mfsa_LscMf ) && ( v->CtrlMode != MF_CALIB_CURR_CTRL ) )
		return;
	v->R_StartFlg  = *Enable;
	if( v->R_StartFlg == MF_ENABLE )
	{
		v->SelectAxis  = PhaseOption[ Setup - 1 ].AxisNum;
		v->SelecetSide = PhaseOption[ Setup - 1 ].POSorNEG;
		v->SelectPhase = PhaseOption[ Setup - 1 ].Phase;
	}
	else;
}


void ReadCurrentAdcValue( CURR_CALIB_DEFINE *v, AdcStation *a, uint32_t *Enable )
{
	if( ( v->Authority < Mfsa_LscMf ) && ( v->CtrlMode != MF_CALIB_CURR_CTRL ) )
		return;

	if( v->R_StartFlg == MF_ENABLE )
	{
		v->AccumulatorADC += a->AdcRawData.Inj[ v-> SelectPhase ].RawAdcValue;
		v->AccumulatorCnt++;

		if( v->AccumulatorCnt == ACC_NUM )
		{
			v->AdcVal[ v->SelectAxis ].Ise[v-> SelectPhase].Value[ v->SelecetSide ] = ( uint16_t )( v->AccumulatorADC >> SHIFT_BIT );
			v->AccumulatorADC = MF_CLEAR;
			v->AccumulatorCnt = MF_CLEAR;
			*Enable = MF_DISABLE;
			v->R_StartFlg = MF_DISABLE;
		}
		else;
	}
	else v->R_StartFlg = MF_DISABLE;
}

void CalbGainAndZeroPoint( CURR_CALIB_DEFINE *v, uint32_t *CmdValue )
{
	if( ( v->Authority < Mfsa_LscMf ) && ( v->CtrlMode != MF_CALIB_CURR_CTRL ) )
		return;

	uint16_t i = 0;


	float CurrentRange = ( float )( *CmdValue - 0x200 ) *2;
	uint16_t EnableFlg = ( ( *CmdValue & 0x200 ) == 0x200 )? MF_ENABLE: MF_DISABLE ;

	v->Calib_StartFlag = EnableFlg;

	if( v->Calib_StartFlag == MF_ENABLE )
	{
		for( i=0; i<2; i++ )
		{
			v->CalibrationGain[ i ].Ise[ Ise_U ].FDta = CurrentRange / ( ( ( float )v->AdcVal[ i ].Ise[ Ise_U ].Value[ MF_HIGH ] ) - ( ( float )v->AdcVal[ i ].Ise[ Ise_U ].Value[ MF_LOW ] ) );
			v->CalibrationGain[ i ].Ise[ Ise_U ].FDta = ( v->CalibrationGain[ i ].Ise[ Ise_U ].FDta < 0 )?		\
					                                    ( v->CalibrationGain[ i ].Ise[ Ise_U ].FDta * ( -1 ) ): v->CalibrationGain[ i ].Ise[ Ise_U ].FDta;

			v->CalibrationGain[ i ].Ise[ Ise_V ].FDta = CurrentRange / ( ( ( float )v->AdcVal[ i ].Ise[ Ise_V ].Value[ MF_HIGH ] ) - ( ( float )v->AdcVal[ i ].Ise[ Ise_V ].Value[ MF_LOW ] ) );
			v->CalibrationGain[ i ].Ise[ Ise_V ].FDta = ( v->CalibrationGain[ i ].Ise[ Ise_V ].FDta < 0 )?		\
					                                    ( v->CalibrationGain[ i ].Ise[ Ise_V ].FDta * ( -1 ) ): v->CalibrationGain[ i ].Ise[ Ise_V ].FDta;

			v->CalibrationGain[ i ].Ise[ Ise_W ].FDta = CurrentRange / ( ( ( float )v->AdcVal[ i ].Ise[ Ise_W ].Value[ MF_HIGH ] ) - ( ( float )v->AdcVal[ i ].Ise[ Ise_W ].Value[ MF_LOW ] ) );
			v->CalibrationGain[ i ].Ise[ Ise_W ].FDta = ( v->CalibrationGain[ i ].Ise[ Ise_W ].FDta < 0 )?		\
								                        ( v->CalibrationGain[ i ].Ise[ Ise_W ].FDta * ( -1 ) ): v->CalibrationGain[ i ].Ise[ Ise_W ].FDta;

			v->Zero[ i ].Iu = ( v->AdcVal[ i ].Ise[ Ise_U ].Value[ MF_HIGH ] + v->AdcVal[ i ].Ise[ Ise_U ].Value[ MF_LOW ] ) >> 1;
			v->Zero[ i ].Iv = ( v->AdcVal[ i ].Ise[ Ise_V ].Value[ MF_HIGH ] + v->AdcVal[ i ].Ise[ Ise_V ].Value[ MF_LOW ] ) >> 1;
			v->Zero[ i ].Iw = ( v->AdcVal[ i ].Ise[ Ise_W ].Value[ MF_HIGH ] + v->AdcVal[ i ].Ise[ Ise_W ].Value[ MF_LOW ] ) >> 1;
		}
//		Axis 1
		DriveParams.PCUParams.Axis1_Iu_Scale[ MF_LOW ]  = v->CalibrationGain[ 0 ].Ise[ Ise_U ].Uint16Type[ MF_LOW ];
		DriveParams.PCUParams.Axis1_Iu_Scale[ MF_HIGH ] = v->CalibrationGain[ 0 ].Ise[ Ise_U ].Uint16Type[ MF_HIGH ];
		DriveParams.PCUParams.Axis1_Iv_Scale[ MF_LOW ]  = v->CalibrationGain[ 0 ].Ise[ Ise_V ].Uint16Type[ MF_LOW ];
		DriveParams.PCUParams.Axis1_Iv_Scale[ MF_HIGH ] = v->CalibrationGain[ 0 ].Ise[ Ise_V ].Uint16Type[ MF_HIGH ];
		DriveParams.PCUParams.Axis1_Iw_Scale[ MF_LOW ]  = v->CalibrationGain[ 0 ].Ise[ Ise_W ].Uint16Type[ MF_LOW ];
		DriveParams.PCUParams.Axis1_Iw_Scale[ MF_HIGH ] = v->CalibrationGain[ 0 ].Ise[ Ise_W ].Uint16Type[ MF_HIGH ];

		DriveParams.PCUParams.Axis1_Iu_ZeroPoint  = v->Zero[ 0 ].Iu;
		DriveParams.PCUParams.Axis1_Iv_ZeroPoint  = v->Zero[ 0 ].Iv;
		DriveParams.PCUParams.Axis1_Iw_ZeroPoint  = v->Zero[ 0 ].Iw;

//		Axis 2
//		DriveParams.PCUParams.Axis2_Iu_Scale[ MF_LOW ]  = v->CalibrationGain[ 1 ].Ise[ Ise_U ].Uint16Type[ MF_LOW ];
//		DriveParams.PCUParams.Axis2_Iu_Scale[ MF_HIGH ] = v->CalibrationGain[ 1 ].Ise[ Ise_U ].Uint16Type[ MF_HIGH ];
//		DriveParams.PCUParams.Axis2_Iv_Scale[ MF_LOW ]  = v->CalibrationGain[ 1 ].Ise[ Ise_V ].Uint16Type[ MF_LOW ];
//		DriveParams.PCUParams.Axis2_Iv_Scale[ MF_HIGH ] = v->CalibrationGain[ 1 ].Ise[ Ise_V ].Uint16Type[ MF_HIGH ];
//		DriveParams.PCUParams.Axis2_Iw_Scale[ MF_LOW ]  = v->CalibrationGain[ 1 ].Ise[ Ise_W ].Uint16Type[ MF_LOW ];
//		DriveParams.PCUParams.Axis2_Iw_Scale[ MF_HIGH ] = v->CalibrationGain[ 1 ].Ise[ Ise_W ].Uint16Type[ MF_HIGH ];
//
//		DriveParams.PCUParams.Axis2_Iu_ZeroPoint  = v->Zero[ 1 ].Iu;
//		DriveParams.PCUParams.Axis2_Iv_ZeroPoint  = v->Zero[ 1 ].Iv;
//		DriveParams.PCUParams.Axis2_Iw_ZeroPoint  = v->Zero[ 1 ].Iw;
		*CmdValue = MF_DISABLE;
		v->Calib_StartFlag = MF_DISABLE;
	}
	else;
}
