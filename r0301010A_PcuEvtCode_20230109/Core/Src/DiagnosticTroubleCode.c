/*
 * DiagnosticTroubleCode.c
 *
 *  Created on: 2023年4月19日
 *      Author: Jeff.Chang
 */

#include "DiagnosticTroubleCode.h"
#include "string.h"


void DTC_Init( DTCStation_t *v )
{
	for (uint16_t i = 0; i < DTC_RecordNumber_Total; i++)
	{
		v->DTCStorePackge[i].StoreContent.DTCId.DTCStoredDataRecordNumber = i + 1;
		v->DTCStorePackge[i].StoreContent.DTCId.DTCCodeHi = v->DTC_Code[i] >> 8;
		v->DTCStorePackge[i].StoreContent.DTCId.DTCCodeLow = v->DTC_Code[i] & 0xFF;
	}
}
void DTC_DoHouseKeeping ( DTCStation_t *v, ExtFlash_t *p )
{
/*Read DTC*/
	if ( v->State == DTC_Process_State_Read || v->DTC_Initial_Read_Finishied == 0 )
	{
        for (uint8_t i = 0; i < DTC_RecordNumber_Total; i++)
        {
        	DTCStoreContent_t tempDTCStoreContent0 = {0};
        	DTCStoreContent_t tempDTCStoreContent1 = {0};
            p->DTC_Store.Read_DTC_Data( p, i, (uint8_t*)&tempDTCStoreContent0, (uint8_t*)&tempDTCStoreContent1 );

            if ( tempDTCStoreContent0.DTCId.DTCStoredDataRecordNumber != 0 )
            {
                memcpy( &v->DTCRespondPackgeFirst[i].DTCId, &tempDTCStoreContent0.DTCId, 4 );
                memcpy( &v->DTCRespondPackgeFirst[i].StatusOfDTC, &v->StatusOfDTC_Realtime[i], 1);
                memcpy( &v->DTCRespondPackgeFirst[i].DTCStoredDataRecordNumberOfIdentifiers, &tempDTCStoreContent0.DTCStoredDataRecordNumberOfIdentifiers, 1 );
                memcpy( &v->DTCRespondPackgeFirst[i].DataIdentifierHi, &tempDTCStoreContent0.DataIdentifierHi, 2 );
                memcpy( &v->DTCRespondPackgeFirst[i].DTCStoredData, &tempDTCStoreContent0.DTCStoredData, UdsDTCFreezeFrameEnvironmentalDataLength );
            }
            if ( tempDTCStoreContent1.DTCId.DTCStoredDataRecordNumber != 0 )
            {
                memcpy( &v->DTCRespondPackgeLast[i].DTCId, &tempDTCStoreContent1.DTCId, 4 );
                memcpy( &v->DTCRespondPackgeLast[i].StatusOfDTC, &v->StatusOfDTC_Realtime[i], 1);
                memcpy( &v->DTCRespondPackgeLast[i].DTCStoredDataRecordNumberOfIdentifiers, &tempDTCStoreContent1.DTCStoredDataRecordNumberOfIdentifiers, 1 );
                memcpy( &v->DTCRespondPackgeLast[i].DataIdentifierHi, &tempDTCStoreContent1.DataIdentifierHi, 2 );
                memcpy( &v->DTCRespondPackgeLast[i].DTCStoredData, &tempDTCStoreContent1.DTCStoredData, UdsDTCFreezeFrameEnvironmentalDataLength );
            }
        }
        if ( v->DTC_Initial_Read_Finishied == 1 )
        {
            v->State = DTC_Process_State_Idle;
        }
        else
        {
            v->DTC_Initial_Read_Finishied = 1;
        }
	}

/*Write DTC*/
	else if( v->State == DTC_Process_State_Write )
	{
        for (uint8_t i = 0; i < DTC_RecordNumber_Total; i++)
        {
        	DTCStoreContent_t tempDTCStoreContent0 = {0};
        	DTCStoreContent_t tempDTCStoreContent1 = {0};
            if ( v->DTCStorePackge[i].DTC_Store_State == DTC_Store_State_Confirmed_and_wait_for_Store )
            {
            	if ( v->DTCRespondPackgeLast[i].DTCStoredData.Error_Occurred_Counter >= 2)
            	{
                	v->DTCStorePackge[i].StoreContent.DTCStoredData.Error_Occurred_Counter = v->DTCRespondPackgeLast[i].DTCStoredData.Error_Occurred_Counter + 1;
            	}
            	else if ( v->DTCRespondPackgeFirst[i].DTCStoredData.Error_Occurred_Counter == 1 )
            	{
            		v->DTCStorePackge[i].StoreContent.DTCStoredData.Error_Occurred_Counter = 2;
            	}
            	else
            	{
            		v->DTCStorePackge[i].StoreContent.DTCStoredData.Error_Occurred_Counter = 1;
            	}

            	p->DTC_Store.Write_DTC_Data( p, i, (uint8_t*)&v->DTCStorePackge[i].StoreContent );
            	v->DTCStorePackge[i].DTC_Store_State = DTC_Store_State_Check_Store_Valid;

                //  Read back and check store valid
                p->DTC_Store.Read_DTC_Data( p, i, (uint8_t*)&tempDTCStoreContent0, (uint8_t*)&tempDTCStoreContent1 );

                if ( v->DTCStorePackge[i].StoreContent.DTCStoredData.Error_Occurred_Counter == 1 )
                {
                	if ( tempDTCStoreContent0.DTCId.DTCStoredDataRecordNumber != 0 )
                	{
                    	v->DTCStorePackge[i].DTC_Store_State = DTC_Store_State_Has_Stored_this_cycle;
                        memcpy( &v->DTCRespondPackgeFirst[i].DTCId, &tempDTCStoreContent0.DTCId, 4 );
                        memcpy( &v->DTCRespondPackgeFirst[i].StatusOfDTC, &v->StatusOfDTC_Realtime[i], 1);
                        memcpy( &v->DTCRespondPackgeFirst[i].DTCStoredDataRecordNumberOfIdentifiers, &tempDTCStoreContent0.DTCStoredDataRecordNumberOfIdentifiers, 1 );
                        memcpy( &v->DTCRespondPackgeFirst[i].DataIdentifierHi, &tempDTCStoreContent0.DataIdentifierHi, 2 );
                        memcpy( &v->DTCRespondPackgeFirst[i].DTCStoredData, &tempDTCStoreContent0.DTCStoredData, UdsDTCFreezeFrameEnvironmentalDataLength );
                	}
                	else
                	{
                    	v->DTCStorePackge[i].DTC_Store_State = DTC_Store_State_None;
                	}
                }
                else
                {
                	if ( tempDTCStoreContent1.DTCId.DTCStoredDataRecordNumber != 0 )
                	{
                    	v->DTCStorePackge[i].DTC_Store_State = DTC_Store_State_Has_Stored_this_cycle;
                        memcpy( &v->DTCRespondPackgeLast[i].DTCId, &tempDTCStoreContent1.DTCId, 4 );
                        memcpy( &v->DTCRespondPackgeLast[i].StatusOfDTC, &v->StatusOfDTC_Realtime[i], 1);
                        memcpy( &v->DTCRespondPackgeLast[i].DTCStoredDataRecordNumberOfIdentifiers, &tempDTCStoreContent1.DTCStoredDataRecordNumberOfIdentifiers, 1 );
                        memcpy( &v->DTCRespondPackgeLast[i].DataIdentifierHi, &tempDTCStoreContent1.DataIdentifierHi, 2 );
                        memcpy( &v->DTCRespondPackgeLast[i].DTCStoredData, &tempDTCStoreContent1.DTCStoredData, UdsDTCFreezeFrameEnvironmentalDataLength );
                	}
                	else
                	{
                    	v->DTCStorePackge[i].DTC_Store_State = DTC_Store_State_None;
                	}
                }
            }

        }

        v->State = DTC_Process_State_Idle;
	}

/*Clear DTC*/

	else if ( v->State == DTC_Process_State_Clear )
	{
		p->DTC_Store.Clear_DTC_Data( p );
        for (uint8_t i = 0; i < DTC_RecordNumber_Total; i++)
        {
        	DTCStoreContent_t tempDTCStoreContent0 = {0};
        	DTCStoreContent_t tempDTCStoreContent1 = {0};
            p->DTC_Store.Read_DTC_Data( p, i, (uint8_t*)&tempDTCStoreContent0, (uint8_t*)&tempDTCStoreContent1 );

            if (( tempDTCStoreContent0.DTCId.DTCStoredDataRecordNumber != 0 ) || ( tempDTCStoreContent1.DTCId.DTCStoredDataRecordNumber != 0 ))
            {
            	 v->State = DTC_Process_State_Clear_Failed;
            	 break;
            }
            else
            {
            	memset(&(v->StatusOfDTC_Realtime[i]), 0, sizeof(DTCStatusOfDTC_t));
            	memset(&(v->DTCRespondPackgeFirst[i]), 0, sizeof(DTCRespondPackge_t));
            	memset(&(v->DTCRespondPackgeLast[i]), 0, sizeof(DTCRespondPackge_t));
            	memset(&(v->DTCStorePackge[i].StoreContent.DTCStoredData), 0, sizeof(UdsDTCFreezeFrameEnvironmentalData_t));
            }

        }
	}

}
