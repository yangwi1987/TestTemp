/*
 * GlobalAlarmDetect.h
 *
 *  Created on: 2021年4月16日
 *      Author: Kevin.Kuo
 */

#ifndef GLOBALALARMDETECT_H_

void GlobalAlarmDetect_init( void );
void GlobalAlarmDetect_DoHouseKeeping( void );
void GlobalAlarmDetect_Accumulation( PROTECT_POLLING_TYPE *p, int Signal, int TargetID );
void GlobalAlarmDetect_ConfigAlarmSystem( void );

#define GLOBALALARMDETECT_H_

#endif /* GLOBALALARMDETECT_H_ */
