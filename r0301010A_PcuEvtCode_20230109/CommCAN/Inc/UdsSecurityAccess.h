/*
 * UdsSecurityAccess.h
 *
 *  Created on: 2021年3月24日
 *      Author: Will.Yang.CYY
 */

#ifndef INC_UDSSECURITYACCESS_H_
#define INC_UDSSECURITYACCESS_H_

#include "stm32g4xx_hal.h"
#include "string.h"
#include "stdlib.h"
#include "CANDrive.h"

#define UDS_SECURE_SEED_SIZE 2
#define UDS_SECURE_TIMEOUT_THRESHOLD_MS	60000 //60sec
#define FALSE_ACCESS_EXCEED_LIMIT_NUMBER 3 // 3 times
#define SECURITY_LEVEL_0  0
#define DEFAULT_SECURITY_LEVEL  SECURITY_LEVEL_0

typedef void (*funcTypeUdsSecurityAccess_Init) ( void* );
typedef void (*funcTypeUdsSecurityAccess_Clear) ( void* );
typedef void (*funcTypeUdsSecurityAccess_Keymul) ( void* );
typedef uint8_t (*funcTypeUdsSecurityAccess_SubFunctExcute) ( void*, uint8_t*, uint8_t* );
typedef uint8_t (*funcTypeUdsSecurityAccess_DoPlcLoop) ( void* );

typedef enum
{
	UDS_SECURE_STATE_IDLE,
	UDS_SECURE_STATE_GET_REQUEST_SEED,
	UDS_SECURE_STATE_SEND_SEED,
	UDS_SECURE_STATE_WAIT_KEY,
	UDS_SECURE_STATE_GET_KEY_SEND,
	UDS_SECURE_STATE_CHECKED,
} UdsSecureState_e;

typedef enum
{
	UDS_SECURE_RESULT_IDLE,
	UDS_SECURE_RESULT_SUCCESS,
	UDS_SECURE_RESULT_FAIL_SUB_FUNCTION_VALUE_MISMATCH,
	UDS_SECURE_RESULT_FAIL_WRONG_KEY,
	UDS_SECURE_RESULT_FAIL_WRONG_SEQ,
	UDS_SECURE_RESULT_FAIL_UNSUPPORT_SECURE_LEVEL,
	UDS_SECURE_RESULT_FAIL_TIMEOUT,
	UDS_SECURE_RESULT_FAIL_FALSE_ACCESS_EXCEED_LIMIT
} UdsSecureResult_e;

typedef struct{
	uint8_t bit0:1;
	uint8_t bit1:1;
	uint8_t bit2:1;
	uint8_t reserved:5;
} BitFields_t;

typedef union{
	uint8_t All;
	BitFields_t Byte;
}IndexSelect_u;

typedef struct
{
	uint8_t IsFalseAccessExceedLimit;
	uint8_t FalseAccessCNT;
	uint8_t SubFuncIn;
	uint8_t SecureLvReq;
	uint8_t SecureLvNow;
	uint8_t SecureState;
	uint8_t SecureResult;
	uint8_t Keymul;
	uint16_t TimeoutCnt;
	uint32_t Seed;
	uint16_t keymulSel;
	funcTypeUdsSecurityAccess_Init Init;
	funcTypeUdsSecurityAccess_SubFunctExcute SubFuncCheck;
	funcTypeUdsSecurityAccess_DoPlcLoop DoPlcLoop;
	funcTypeUdsSecurityAccess_Clear Clear;
	funcTypeUdsSecurityAccess_Keymul KeyMulCal;
}UdsSecurityAccessCtrl_t;

void UdsSecurityAccess_Init ( UdsSecurityAccessCtrl_t *p );
void UdsSecurityAccess_Clear ( UdsSecurityAccessCtrl_t *p );
uint8_t UdsSecurityAccess_SeedReq ( UdsSecurityAccessCtrl_t *p, uint8_t *pDataOut );
void UdsSecurityAccess_KeymulCal( UdsSecurityAccessCtrl_t *p );
uint8_t UdsSecurityAccess_KeySend ( UdsSecurityAccessCtrl_t *p, uint8_t *pDataIn );
uint8_t UdsSecurityAccess_SubFuncExecute ( UdsSecurityAccessCtrl_t *p, uint8_t *pDataIn, uint8_t *pDataOut );
uint8_t UdsSecurityAccess_DoPlcLoop ( UdsSecurityAccessCtrl_t *p );

#define UDS_SECURITY_ACCESS_CTRL_DEFAULT \
{\
	0,    /*IsFalseAccessExceedLimit*/\
	0,    /*FalseAccessCNT*/\
	0,    /*SubFuncIn*/\
	0,    /*SecureLvReq*/\
	DEFAULT_SECURITY_LEVEL,    /*SecureLvNow*/\
	0,    /*SecureState*/\
	0,    /*SecureResult*/\
	0,    /*Keymul*/\
	0,    /*TimeoutCnt*/\
	0,    /*Seed*/\
	0,  /*keymulSel*/\
	(funcTypeUdsSecurityAccess_Init)UdsSecurityAccess_Init,\
	(funcTypeUdsSecurityAccess_SubFunctExcute)UdsSecurityAccess_SubFuncExecute,\
	(funcTypeUdsSecurityAccess_DoPlcLoop)UdsSecurityAccess_DoPlcLoop,\
	(funcTypeUdsSecurityAccess_Clear)UdsSecurityAccess_Clear,\
	(funcTypeUdsSecurityAccess_Keymul)UdsSecurityAccess_KeymulCal,\
}\


#endif /* INC_UDSSECURITYACCESS_H_ */
