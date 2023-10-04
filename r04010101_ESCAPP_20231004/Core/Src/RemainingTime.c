/*
 * RemainingTime.c
 *
 *  Created on: 2023年7月14日
 *      Author: Jeff
 */
#include "RemainingTime.h"

void RemainingTime_Do1secLoop ( RemainingTime_t *p, uint16_t FCC, uint8_t Related_SoC, uint16_t Insta_Power, uint8_t Limphome_flag )
{
	if ( Limphome_flag == 0 )
	{
		if (( FCC > MIN_FULL_CHARGED_CAPACITY ) && ( Related_SoC > MIN_RELATED_SOC ))
		{
			uint32_t temp_Remaining_Time_Min = 0;
			// calculate remaining capacity (Wh)
            p->Remaining_Capacity_100Wh = (uint32_t)FCC * ( (uint32_t)Related_SoC - 20 );   //Wh * 100

            //  calculate power consumption during 120 sec
            p->power_sum_120Sec_Ws = p->power_sum_120Sec_Ws - (uint32_t)p->power_record[p->power_record_index];
            p->power_record[p->power_record_index] = Insta_Power;
            p->power_sum_120Sec_Ws = p->power_sum_120Sec_Ws + (uint32_t)p->power_record[p->power_record_index];
            p->power_record_index = (( p->power_record_index == ( POWER_RECORD_NUMBER - 1 )) ? 0 : p->power_record_index + 1 );



            // Calculate remain time (min)
            /*
             *   1. Remaining_Capacity_Wh  = Remaining_Capacity_100Wh / 100
             *   2. power_consumption_Wh = power_sum_120Sec_Ws / 3600 / 120
             *   3. Remaining_Time_Sec = Remaining_Capacity_Wh / power_consumption_Wh
             *   4. Remaining_Time_Min = Remaining_Time_Sec / 60
             *   combine:
             *   Remaining_Time_Min = (( Remaining_Capacity_100Wh / 100 ) / ( power_sum_120Sec_Ws / 3600 / 120 )) / 60
             *                      = ( Remaining_Capacity_100Wh * 72 ) / power_sum_120Sec_Ws
             */
            if ( p->power_sum_120Sec_Ws >= MIN_POWER_SUM_120SEC_WS )
            {
                temp_Remaining_Time_Min = ( p->Remaining_Capacity_100Wh * 72 ) / p->power_sum_120Sec_Ws;
                temp_Remaining_Time_Min = ( temp_Remaining_Time_Min > MAX_REMAINING_TIME ) ? MAX_REMAINING_TIME : temp_Remaining_Time_Min ;
            }
            else
            {
            	temp_Remaining_Time_Min = MAX_REMAINING_TIME;
            }

            p->Remaining_Time_Min = (uint8_t)temp_Remaining_Time_Min;
		}
		else
		{
			;;
		}
	}
	else
	{
        // Clean remaining time data
		p->Remaining_Capacity_100Wh = 0;
		p->power_sum_120Sec_Ws = 0;
		memset(p->power_record, 0, sizeof(p->power_record));
		p->power_record_index = 0;
		p->Remaining_Time_Min = 0;
	}
}
