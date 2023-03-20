/*
 * PWM_RC.h
 *
 *  Created on: Feb 10, 2022
 *      Author: Hank.Chen.CHC
 */

#ifndef INC_PWM_RC_H_
#define INC_PWM_RC_H_

#define MAX_CNT_OF_PWM_PERIOD	65536
#define MAX_CNT_ALARM_TRIG		21

typedef void ( *functypePwmRcAlarmDetect )( void* );

typedef struct{
	uint32_t PeriodEdgeValue;
	uint32_t DutyEdgeValue;
	uint32_t PrevPeriodEdgeValue;
	uint16_t StartFlag;
	uint16_t AbnormalCnt;
	float DutyRaw;
	float DutyCalc;
	functypePwmRcAlarmDetect AlarmDet;
}PWM_RC_TYPE;

void PWM_RC_Abnormal_Detect( PWM_RC_TYPE *p );

#define PWM_RC_TYPE_DEFAULT { \
	0, \
	0, \
	0, \
	0, \
	0, \
	0.0, \
	0.0, \
	(functypePwmRcAlarmDetect) PWM_RC_Abnormal_Detect,	\
}

#endif /* INC_PWM_RC_H_ */
