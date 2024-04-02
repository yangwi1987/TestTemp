/*
 * ParamMgr.h
 *
 *  Created on: Dec 4, 2019
 *      Author: MikeSFWen
 */

#ifndef INC_PARAMMGR_H_
#define INC_PARAMMGR_H_

#include "stdint.h"
#include "ParamData.h"
#include "ParamTable.h"
#include "PCUTableLinker.h"
#include "SystemTableLinker.h"
#include "Motor_Init_Table.h"
#include "ExtFlash.h"
#include "string.h"

#define PARAM_TOTAL_SIZE 		(sizeof(DriveParams_t)/sizeof(uint16_t))
#define PARAM_FN_REGS_SIZE		100

extern uint32_t DriveFnRegs[PARAM_FN_REGS_SIZE];

typedef void (*functypeParamMgr_Init)(void *,void*);
typedef void (*functypeParamMgr_OnParamValueChanged)(uint16_t, uint16_t);
typedef int32_t (*functypeParamMgr_ReadParam)(void *, uint16_t, uint16_t, uint8_t *);
typedef int32_t (*functypeParamMgr_WriteParam)(void *, uint16_t, uint16_t, int32_t, uint8_t *);
typedef int32_t (*functypeParamMgr_ReadFnRegs)(void *, uint16_t, uint8_t *);
typedef int32_t (*functypeParamMgr_WriteFnRegs)(void *, uint16_t, int32_t, uint8_t *);
typedef void (*functypeParamMgr_LoadCurrentCalibFromSecExtFlash)(void *);

typedef struct {
	uint32_t Min;
	uint32_t Max;
	uint16_t Security;
} DriveFnRegsInfo_t;


typedef enum 
{
	PARAM_ACCESS_SUCCESS = (uint8_t)0,
	PARAM_ACCESS_FAIL_NO_AUTH,
	PARAM_ACCESS_FAIL_PARAM_ID_NOT_DEFINED,
	PARAM_ACCESS_FAIL_VALUE_NOT_AVAILABLE,
	PARAM_ACCESS_FAIL_WORNG_AXIS_ID,
	PARAM_ACCESS_FAIL_WRITE_NOT_SUPPORTED,
}ParamAccessResult_e;


typedef struct {
	uint16_t Dirty;
	uint16_t Session; // original LSC manufacturing session. BRP session is DiagnosticSession in UDSserviceCtrl.h
	uint16_t NextSession; // original LSC manufacturing session. BRP session is DiagnosticSession in UDSserviceCtrl.h
	uint16_t Security;
	uint8_t ECUSoftResetEnable;
	uint8_t *pFlashParaReadEnableTable;
	ParamTableInfo_t *pParamTable;
	functypeParamMgr_Init Init;
	functypeParamMgr_OnParamValueChanged OnParamValueChanged;
	functypeParamMgr_ReadParam ReadParam; /* read data from DriveParams */
	functypeParamMgr_WriteParam WriteParam; /* write data into DriveParams */
	functypeParamMgr_ReadFnRegs ReadFnRegs; /* read data from drive function registers(DriveFnRegs) */
	functypeParamMgr_WriteFnRegs WriteFnRegs; /* write data into drive function registers(DriveFnRegs) */
	functypeParamMgr_LoadCurrentCalibFromSecExtFlash LoadCurrentCalibFromSecExtFlash;
} ParamMgr_t;

void ParamMgr_Init( ParamMgr_t *v, ExtFlash_t *pExtFlash );

// PN: parameter number. FN: function number
int32_t ParamMgr_ReadParam( ParamMgr_t *v, uint16_t AxisID, uint16_t PN, uint8_t *pError );
int32_t ParamMgr_WriteParam( ParamMgr_t *v, uint16_t AxisID, uint16_t PN, int32_t value,uint8_t *pError );
int32_t ParamMgr_ReadFnRegs( ParamMgr_t *v, uint16_t FN,uint8_t *pError );
int32_t ParamMgr_WriteFnRegs( ParamMgr_t *v, uint16_t FN, uint32_t value,uint8_t *pError );
void ParamMgr_LoadCurrentCalibFromSecExtFlash( ExtFlash_t *pExtFlash );
extern uint16_t ParamMgr_ParaGainHandler( DriveParams_t *v, uint16_t *Var, float *Out );

#if USE_INIT_HIGH_SECURITY
#define PARAM_MGR_DEFAULT { \
	0, \
	Session_0x01_Default, /* Session */ \
	Session_0x01_Default, /* NextSession */ \
	DEFAULT_SECURITY_LEVEL, /* Security */ \
	0, /* ECUSoftResetEnable */ \
	0, /* pFlashParaReadEnableTable */ \
	0, /* pParamTable */\
	(functypeParamMgr_Init)ParamMgr_Init, \
	0, \
	(functypeParamMgr_ReadParam)ParamMgr_ReadParam, \
	(functypeParamMgr_WriteParam)ParamMgr_WriteParam, \
	(functypeParamMgr_ReadFnRegs)ParamMgr_ReadFnRegs, \
	(functypeParamMgr_WriteFnRegs)ParamMgr_WriteFnRegs, \
	(functypeParamMgr_LoadCurrentCalibFromSecExtFlash)ParamMgr_LoadCurrentCalibFromSecExtFlash }
#else
#define PARAM_MGR_DEFAULT { \
	0, \
	Session_0x01_Default, /* Session */ \
	Session_0x01_Default, /* NextSession */ \
	0, /* Security */ \
	0, /* ECUSoftResetEnable */ \
	0, /* pFlashParaReadEnableTable */ \
	0, /* pParamTable */\
	(functypeParamMgr_Init)ParamMgr_Init, \
	0, \
	(functypeParamMgr_ReadParam)ParamMgr_ReadParam, \
	(functypeParamMgr_WriteParam)ParamMgr_WriteParam, \
	(functypeParamMgr_ReadFnRegs)ParamMgr_ReadFnRegs, \
	(functypeParamMgr_WriteFnRegs)ParamMgr_WriteFnRegs, \
	(functypeParamMgr_LoadCurrentCalibFromSecExtFlash)ParamMgr_LoadCurrentCalibFromSecExtFlash }
#endif

#endif /* INC_PARAMMGR_H_ */
