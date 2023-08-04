/*
 * RemainingTime.h
 *
 *  Created on: 2023年7月14日
 *      Author: Jeff
 */

#ifndef INC_REMAININGTIME_H_
#define INC_REMAININGTIME_H_

#include "Constant.h"
#include "string.h"

#define MIN_FULL_CHARGED_CAPACITY 100   // Uint: Wh
#define MIN_RELATED_SOC           1     // Uint: %
#define MAX_REMAINING_TIME       255    // Uint:Min
#define MIN_POWER_SUM_120SEC_WS   1     // Uint: Ws * 120Sec

#define Power_record_number       120



typedef void (*functypeRemainingTime_Do1secLoop)( void*, uint16_t, uint8_t, uint16_t, uint8_t );

typedef struct
{
	uint32_t Remaining_Capacity_100Wh;
	uint16_t power_record[Power_record_number];
	uint8_t power_record_index;
	uint8_t Remaining_Time_Min;
	uint32_t power_sum_120Sec_Ws;
	functypeRemainingTime_Do1secLoop Do1secLoop;
}RemainingTime_t;

void RemainingTime_Do1secLoop ( RemainingTime_t *p, uint16_t FCC, uint8_t Related_SoC, uint16_t Insta_Power, uint8_t Limphome_flag );

#define REMAININGTIME_DEFAULT  \
{                              \
	0, /*Remaining_Capacity_100Wh;*/\
    {0}, /*power_record[Power_record_number]; */\
    0, /*power_record_index; */\
	0, /*Remaining_Time_Min;*/\
    0, /*power_sum_120Sec_Ws*/\
    (functypeRemainingTime_Do1secLoop)RemainingTime_Do1secLoop \
}
#endif /* INC_REMAININGTIME_H_ */
