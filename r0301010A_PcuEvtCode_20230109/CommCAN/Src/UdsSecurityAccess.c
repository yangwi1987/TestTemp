/*
 * UdsSecurityAccess.c
 *
 *  Created on: 2021年3月24日
 *      Author: Will.Yang.CYY
 */

#include "UdsSecurityAccess.h"

//uint8_t keymul[2][8]= {
//		{0x3D, 0x02, 0x83, 0xEE, 0xCB, 0xF4, 0x89, 0x71},
//		{0x31, 0xB2, 0x54, 0xE2, 0x25, 0x8D, 0x89, 0xB6}
//};
////									0			1			2			3			4			5			6			7
//uint32_t SecurityAccessMask[8]= { 	0, 			0x1030, 	0x1030, 	0x0809,		0x0809, 	0x02D670ED, 0x0505A0A0, 0xAA005500 };

uint8_t keymul[3][8]= {
		{0xDA, 0x24, 0x17, 0xA8, 0x0E, 0x39, 0x0B, 0x49},
		{0xAC, 0x0A, 0x78, 0x37, 0x7A, 0xEA, 0xDA, 0xCF},
		{0xEE, 0xFF, 0xAB, 0x35, 0x0A, 0xE2, 0x6A, 0xB0}
};
//									0		1			2			3			4			5			6			7
uint32_t SecurityAccessMask[8]= { 	0,		0x0920,		0x0920, 	0x0828,		0x0828,		0x0828,		0x0828,		0xAA005500 };

void UdsSecurityAccess_Init ( UdsSecurityAccessCtrl_t *p )
{
	p->SecureLvReq = 0;
//	p->SecureLvNow = DEFAULT_SECURITY_LEVEL; // read from external flash
	p->SecureState = UDS_SECURE_STATE_IDLE;
	p->SecureResult = UDS_SECURE_RESULT_IDLE;
	p->Seed = 0;
	p->TimeoutCnt = 0;
	p->SubFuncIn = 0;
}

void UdsSecurityAccess_Clear ( UdsSecurityAccessCtrl_t *p )
{
	p->SecureLvReq = 0;
	p->SecureState = UDS_SECURE_STATE_IDLE;
	p->Seed = 0;
	p->TimeoutCnt = 0;
	p->SubFuncIn = 0;
}

uint8_t UdsSecurityAccess_SeedReq ( UdsSecurityAccessCtrl_t *p, uint8_t *pDataOut )
{
	uint32_t SeedTemp = 0;
	uint8_t  ResultTemp = UDS_SECURE_RESULT_IDLE;
	//todo use random value or use timer counter value
	if ( p->SecureLvNow != p->SecureLvReq )
	{
		//new security level request is received, return specified Seed to client
		srand(HAL_GetTick());
		p->Seed = rand();
		if( (p->SecureLvReq== 1)|
			(p->SecureLvReq== 2)|
			(p->SecureLvReq== 3)|
			(p->SecureLvReq== 4)|
			(p->SecureLvReq== 5)|
			(p->SecureLvReq== 6) )
		{
			p->Seed&= 0x0000FFFF;
		}
		else
		{
			//new security level required is unlocked before, return all "0" seed to client
			SeedTemp = 0;
			ResultTemp =  UDS_SECURE_RESULT_FAIL_UNSUPPORT_SECURE_LEVEL;
		}
		p->keymulSel= p->Seed;
		p->KeyMulCal( p );
		SeedTemp = p->Seed;
		ByteSwap ( &SeedTemp, UDS_SECURE_SEED_SIZE );
	}
	else
	{
		//new security level required is unlocked before, return all "0" seed to client 
		SeedTemp = 0;
	}
	memcpy ( pDataOut, (uint8_t*)&SeedTemp, UDS_SECURE_SEED_SIZE );

	return ResultTemp;
}

void UdsSecurityAccess_KeymulCal( UdsSecurityAccessCtrl_t *p )
{
	IndexSelect_u Idx;
    if( (p->SecureLvReq == 1)|(p->SecureLvReq == 2) ) {
    	Idx.Byte.bit0= 0x00000001&( p->keymulSel >> 5 );
    	Idx.Byte.bit1= 0x00000001&( p->keymulSel >> 8 );
    	Idx.Byte.bit2= 0x00000001&( p->keymulSel >> 11 );
    	Idx.Byte.reserved= 0;
    	p->Keymul= keymul[0][Idx.All];
    } else if( (p->SecureLvReq == 3)|(p->SecureLvReq == 4) ) {
    	Idx.Byte.bit0= 0x00000001&( p->keymulSel >> 3 );
    	Idx.Byte.bit1= 0x00000001&( p->keymulSel >> 5 );
    	Idx.Byte.bit2= 0x00000001&( p->keymulSel >> 11 );
    	Idx.Byte.reserved= 0;
    	p->Keymul= keymul[1][Idx.All];
    } else if( (p->SecureLvReq == 5)|(p->SecureLvReq == 6) ) {
    	Idx.Byte.bit0= 0x00000001&( p->keymulSel >> 3 );
    	Idx.Byte.bit1= 0x00000001&( p->keymulSel >> 5 );
    	Idx.Byte.bit2= 0x00000001&( p->keymulSel >> 11 );
    	Idx.Byte.reserved= 0;
    	p->Keymul= keymul[2][Idx.All];
    } else;
}


uint8_t UdsSecurityAccess_KeySend ( UdsSecurityAccessCtrl_t *p, uint8_t *pDataIn )
{
	uint32_t KeyReceived;
	uint32_t KeyExpected;
	uint32_t InverseSeed= 0;
	KeyReceived = ( *pDataIn << 8 )  + *( pDataIn + 1 );
	switch ( p->SecureLvReq )
	{
		case 1:
		case 2:
		case 3:
		case 4:
		case 5:
		case 6:
			InverseSeed= 0x0000FFFF&(~(p->Seed));
			KeyExpected = (uint32_t)( 0x0000FFFF& (InverseSeed* p->Keymul/ 0x40 ));
			break;
		case 7:
			return UDS_SECURE_RESULT_FAIL_UNSUPPORT_SECURE_LEVEL;
			break;
		default:
			return UDS_SECURE_RESULT_FAIL_UNSUPPORT_SECURE_LEVEL;
			break;
	}

	if ( KeyExpected == KeyReceived )
	{
		p->SecureLvNow = p->SecureLvReq;
		return UDS_SECURE_RESULT_SUCCESS;
	}
	else
	{
		if ( ++(p->FalseAccessCNT) >= FALSE_ACCESS_EXCEED_LIMIT_NUMBER )
		{
			p->IsFalseAccessExceedLimit = 1;
			return UDS_SECURE_RESULT_FAIL_FALSE_ACCESS_EXCEED_LIMIT;
		}
		else
		{
		    return UDS_SECURE_RESULT_FAIL_WRONG_KEY;
		}
	}
}




uint8_t UdsSecurityAccess_SubFuncExecute ( UdsSecurityAccessCtrl_t *p, uint8_t *pDataIn, uint8_t *pDataOut )
{
	uint8_t SubFunc;
	SubFunc = p->SubFuncIn & 0x01;
	if ( SubFunc == 1 )	//odd value ,it's a "Request Seed" sub-function	
	{
		p->SecureState = UDS_SECURE_STATE_GET_REQUEST_SEED;
		p->SecureLvReq = ((p->SubFuncIn + 1) >> 1);
		p->SecureResult = UdsSecurityAccess_SeedReq ( p, pDataOut );
		p->SecureState = ( p->SecureResult == UDS_SECURE_RESULT_IDLE ) ? UDS_SECURE_STATE_SEND_SEED : UDS_SECURE_STATE_CHECKED;
		p->TimeoutCnt = 0;
	}
	else	//even value ,it's a "Send Key" sub-function
	{
		if ( p->SecureState == UDS_SECURE_STATE_WAIT_KEY )	//correct sequence
		{
			if ( (p->SubFuncIn >> 1) == p->SecureLvReq )
			{
				/*
				 * the sub-function value  of send key and seed request are matched ,
				 * for example if send key = 2*N-1 and then seed request should be 2*N where N = 1...50,
				 * otherwise ,report error
				 */
				p->SecureResult = UdsSecurityAccess_KeySend ( p, pDataIn );
				p->SecureState = UDS_SECURE_STATE_CHECKED;

			}
			else	 //wrong sub function value,report error
			{
				p->SecureResult = UDS_SECURE_RESULT_FAIL_SUB_FUNCTION_VALUE_MISMATCH;
			}
		}
		else //in-correct sequence, report error
		{
			p->SecureResult = UDS_SECURE_RESULT_FAIL_WRONG_SEQ;
		}

		if ( p->SecureResult != UDS_SECURE_RESULT_SUCCESS )
		{
			UdsSecurityAccess_Clear ( p );
		}
	}
	return p->SecureResult;
}

uint8_t UdsSecurityAccess_DoPlcLoop ( UdsSecurityAccessCtrl_t *p )
{
	if ( p->SecureState == UDS_SECURE_STATE_WAIT_KEY )
	{
		if ( p->TimeoutCnt >= UDS_SECURE_TIMEOUT_THRESHOLD_MS )
		{
			p->TimeoutCnt = 0;
			p->SecureResult = UDS_SECURE_RESULT_FAIL_TIMEOUT;
			UdsSecurityAccess_Clear ( p );
		}
		p->TimeoutCnt++;
	}
	return p->SecureResult;
}
