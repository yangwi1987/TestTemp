/*
 * DiagnosticTroubleCode.h
 *
 *  Created on: 2023年4月19日
 *      Author: Jeff.Chang
 */

#ifndef INC_DIAGNOSTICTROUBLECODE_H_
#define INC_DIAGNOSTICTROUBLECODE_H_

#include "ExtFlash.h"
#include "UdsDID.h"
#include "AlarmMgr.h"

#define DTCStatusAvailabilityMask  {\
	1, /*bit0 Test Failed*/\
    0, /*bit1 Test Failed This Operation Cycle*/\
	0, /*bit2 Pending DTC*/\
	0, /*bit3 Confirmed DTC*/\
    0, /*bit4 Test Not Completed Since Last Clear*/\
    0, /*bit5 Test Failed Since Last Clear*/\
    0, /*bit6 Test Not Completed This Operation Cycle*/\
    0, /*bit7 Warning Indicator Requested*/\
    }

typedef void (*functypeDTCStation_Init)          ( void * );
typedef void (*functypeDTCStation_DoHouseKeeping)( void *, void * );

typedef enum
{
	DTC_RecordNumber_P0562_System_voltage_low,
	DTC_RecordNumber_P0563_System_voltage_high ,
	DTC_RecordNumber_U0408_Invalid_data_received_from_RF_RC_module            ,
	DTC_RecordNumber_P1F01_ESC_Over_current                                   ,
	DTC_RecordNumber_P1F09_ESC_Internal_circuit_voltage_out_of_range          ,
	DTC_RecordNumber_P1F02_ESC_Mosfet_High_temperature                        ,
	DTC_RecordNumber_P1F03_ESC_Capacitor_High_temperature                     ,
	DTC_RecordNumber_P1F04_Motor_High_temperature                             ,
	DTC_RecordNumber_P1F05_ESC_current_sensor_abnormal                        ,
	DTC_RecordNumber_P1F06_System_voltage_sensor_abnormal                     ,
	DTC_RecordNumber_P1F0A_ESC_Internal_circuit_logical_failure               ,
	DTC_RecordNumber_P0C05_Motor_Phase_lost                                   ,
	DTC_RecordNumber_P0219_Motor_Overspeed                                    ,
	DTC_RecordNumber_P18A6_Foil_Position_sensor_abnormal                      ,
	DTC_RecordNumber_P0666_ESC_Mosfet_Temperature_sensor_abnormal             ,
	DTC_RecordNumber_P0667_ESC_Capacitor_Temperature_sensor_abnormal          ,
	DTC_RecordNumber_P0A2A_Motor_Temperature_sensor_abnormal                  ,
	DTC_RecordNumber_P1F00_Motor_Stalled                                      ,
	DTC_RecordNumber_U0111_Lost_communication_with_BMS                        ,
	DTC_RecordNumber_U0107_Lost_communication_with_RF                         ,
	DTC_RecordNumber_P060E_Throttle_position_performance                      ,
	DTC_RecordNumber_U0412_Invalid_data_received_from_BMS                     ,
	DTC_RecordNumber_P0605_Internal_Control_Module_ROM_Error                  ,
	DTC_RecordNumber_P1F12_ESC_Mosfet_High_temperature_warning                ,
	DTC_RecordNumber_P1F13_ESC_Capacitor_High_temperature_warning             ,
	DTC_RecordNumber_P1F14_Motor_High_temperature_warning                     ,
	DTC_RecordNumber_P021A_Motor_Reverse                     ,
	DTC_RecordNumber_Total
}DTC_RecordNumber_e;

typedef enum
{
	DTC_Code_P0562_System_voltage_low                                 =  0x0562,
	DTC_Code_P0563_System_voltage_high                                =  0x0563,
	DTC_Code_U0408_Invalid_data_received_from_RF_RC_module            =  0xC408,
	DTC_Code_P1F01_ESC_Over_current                                   =  0x1F01,
	DTC_Code_P1F09_ESC_Internal_circuit_voltage_out_of_range          =  0x1F09,
	DTC_Code_P1F02_ESC_Mosfet_High_temperature                        =  0x1F02,
	DTC_Code_P1F03_ESC_Capacitor_High_temperature                     =  0x1F03,
	DTC_Code_P1F04_Motor_High_temperature                             =  0x1F04,
	DTC_Code_P1F05_ESC_current_sensor_abnormal                        =  0x1F05,
	DTC_Code_P1F06_System_voltage_sensor_abnormal                     =  0x1F06,
	DTC_Code_P1F0A_ESC_Internal_circuit_logical_failure               =  0x1F0A,
	DTC_Code_P0C05_Motor_Phase_lost                                   =  0x0C05,
	DTC_Code_P0219_Motor_Overspeed                                    =  0x0219,
	DTC_Code_P18A6_Foil_Position_sensor_abnormal                      =  0x18A6,
	DTC_Code_P0666_ESC_Mosfet_Temperature_sensor_abnormal             =  0x0666,
	DTC_Code_P0667_ESC_Capacitor_Temperature_sensor_abnormal          =  0x0667,
	DTC_Code_P0A2A_Motor_Temperature_sensor_abnormal                  =  0x0A2A,
	DTC_Code_P1F00_Motor_Stalled                                      =  0x1F00,
	DTC_Code_U0111_Lost_communication_with_BMS                        =  0xC111,
	DTC_Code_U0107_Lost_communication_with_RF                         =  0xC107,
	DTC_Code_P060E_Throttle_position_performance                      =  0x060E,
	DTC_Code_U0412_Invalid_data_received_from_BMS                     =  0xC412,
	DTC_Code_P0605_Internal_Control_Module_ROM_Error                  =  0x0605,
	DTC_Code_P1F12_ESC_Mosfet_High_temperature_warning                =  0x1F12,
	DTC_Code_P1F13_ESC_Capacitor_High_temperature_warning             =  0x1F13,
	DTC_Code_P1F14_Motor_High_temperature_warning                     =  0x1F14,
	DTC_Code_P021A_Motor_Reverse                                      =  0x021A,
}DTC_Code_e;

typedef enum
{
	DTC_Store_State_None,
	DTC_Store_State_Confirmed_and_wait_for_Store,
	DTC_Store_State_Check_Store_Valid,
	DTC_Store_State_Has_Stored_this_cycle
}DTCStoreState_e;

typedef enum
{
	DTC_Process_State_Idle,
	DTC_Process_State_Read,
	DTC_Process_State_Write,
	DTC_Process_State_Clear,
	DTC_Process_State_Clear_Failed
}DTC_Process_State_e;

typedef struct
{
	uint8_t Test_Failed                             :1;
	uint8_t Test_Failed_This_Operation_Cycle        :1;
	uint8_t Pending_DTC                             :1;
	uint8_t Confirmed_DTC                           :1;
	uint8_t Test_Not_Completed_Since_Last_Clear     :1;
	uint8_t Test_Failed_Since_Last_Clear            :1;
	uint8_t Test_Not_Completed_This_Operation_Cycle :1;
	uint8_t Warning_Indicator_Requested             :1;

}DTCStatusOfDTC_t;

typedef struct
{
	uint32_t DTCStoredDataRecordNumber : 8;
	uint32_t DTCCodeHi                 : 8;
	uint32_t DTCCodeLow                : 8;
	uint32_t FTB                       : 8;
}DTCIdentifier_t;

typedef struct
{
	DTCIdentifier_t DTCId;
	DTCStatusOfDTC_t StatusOfDTC;
	uint8_t DTCStoredDataRecordNumberOfIdentifiers ;
	uint8_t DataIdentifierHi            ;
	uint8_t DataIdentifierLow           ;
	UdsDTCFreezeFrameEnvironmentalData_t DTCStoredData;
}DTCRespondPackge_t;

_Static_assert( sizeof(DTCRespondPackge_t) == (DATA_LENGTH_EACH_DTC_STORE - 1),"error");

typedef struct
{
	DTCIdentifier_t DTCId;
	uint8_t DTCStoredDataRecordNumberOfIdentifiers ;
	uint8_t DataIdentifierHi            ;
	uint8_t DataIdentifierLow           ;
	UdsDTCFreezeFrameEnvironmentalData_t DTCStoredData;
}DTCStoreContent_t;
_Static_assert( sizeof(DTCStoreContent_t) == ( DATA_LENGTH_EACH_DTC_STORE - 1 ),"error");

typedef struct
{
	DTCStoreState_e DTC_Store_State;
	DTCStoreContent_t StoreContent;
}DTCStorePackge_t;

typedef struct
{
	uint8_t DTC_Initial_Read_Finishied;
	uint16_t DTC_Code[DTC_RecordNumber_Total];
	DTCRespondPackge_t DTCRespondPackgeFirst[DTC_RecordNumber_Total];
	DTCRespondPackge_t DTCRespondPackgeLast[DTC_RecordNumber_Total];
	DTCStorePackge_t DTCStorePackge[DTC_RecordNumber_Total];
	DTCStatusOfDTC_t StatusOfDTC_Realtime[DTC_RecordNumber_Total];
	DTC_Process_State_e State ;
	DTCStatusOfDTC_t StatusAvailabilityMask;
	functypeDTCStation_Init Init;
	functypeDTCStation_DoHouseKeeping  DoHouseKeeping ;
}DTCStation_t;

void DTC_Init( DTCStation_t *v );
void DTC_DoHouseKeeping ( DTCStation_t *v, ExtFlash_t *p );

#define DTC_CODE_DEFFAULT { \
		DTC_Code_P0562_System_voltage_low                       ,\
		DTC_Code_P0563_System_voltage_high                      ,\
		DTC_Code_U0408_Invalid_data_received_from_RF_RC_module  ,\
		DTC_Code_P1F01_ESC_Over_current                         ,\
		DTC_Code_P1F09_ESC_Internal_circuit_voltage_out_of_range,\
		DTC_Code_P1F02_ESC_Mosfet_High_temperature              ,\
		DTC_Code_P1F03_ESC_Capacitor_High_temperature           ,\
		DTC_Code_P1F04_Motor_High_temperature                   ,\
		DTC_Code_P1F05_ESC_current_sensor_abnormal              ,\
		DTC_Code_P1F06_System_voltage_sensor_abnormal           ,\
		DTC_Code_P1F0A_ESC_Internal_circuit_logical_failure     ,\
		DTC_Code_P0C05_Motor_Phase_lost                         ,\
		DTC_Code_P0219_Motor_Overspeed                          ,\
		DTC_Code_P18A6_Foil_Position_sensor_abnormal            ,\
		DTC_Code_P0666_ESC_Mosfet_Temperature_sensor_abnormal   ,\
		DTC_Code_P0667_ESC_Capacitor_Temperature_sensor_abnormal,\
		DTC_Code_P0A2A_Motor_Temperature_sensor_abnormal        ,\
		DTC_Code_P1F00_Motor_Stalled                            ,\
		DTC_Code_U0111_Lost_communication_with_BMS              ,\
		DTC_Code_U0107_Lost_communication_with_RF               ,\
		DTC_Code_P060E_Throttle_position_performance            ,\
		DTC_Code_U0412_Invalid_data_received_from_BMS           ,\
		DTC_Code_P0605_Internal_Control_Module_ROM_Error        ,\
		DTC_Code_P1F12_ESC_Mosfet_High_temperature_warning      ,\
		DTC_Code_P1F13_ESC_Capacitor_High_temperature_warning   ,\
		DTC_Code_P1F14_Motor_High_temperature_warning           ,\
		DTC_Code_P021A_Motor_Reverse            \
		}\

#define DTC_STATION_DEFFAULT { \
		0,                     /*DTC_Initial_Read_Finishied*/\
		DTC_CODE_DEFFAULT,     /*DTC_Code*/\
		{ {{0},{0},0,0,0,{0}} }, /*DTCRespondPackgeFirst*/\
		{ {{0},{0},0,0,0,{0}} }, /*DTCRespondPackgeLast*/\
		{ {0, {{0},0,0,0,{0}}} },  /*DTCStorePackge*/\
		{ {0} },                 /*StatusOfDTC_Realtime*/\
		DTC_Process_State_Idle, /*State*/\
		DTCStatusAvailabilityMask, /*StatusAvailabilityMask*/\
		(functypeDTCStation_Init)DTC_Init,   /**/\
		(functypeDTCStation_DoHouseKeeping)DTC_DoHouseKeeping   /**/\
        }

#endif /* INC_DIAGNOSTICTROUBLECODE_H_ */
