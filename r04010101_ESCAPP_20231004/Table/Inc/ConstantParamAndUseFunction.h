/*
 * ConstantParamAndUseFunction.h
 *
 *  Created on: 2021年11月9日
 *      Author: Fernando.Wang.HHW
 */

#ifndef INC_CONSTANTPARAMANDUSEFUNCTION_H_
#define INC_CONSTANTPARAMANDUSEFUNCTION_H_

#define USE_FUNCTION 1
#define UNUSE_FUNCTION 0

//Constant Parameter
#define DRIVE_PROPULSION_TOLERANCE	1.0f
#define GUARANTEED_VBUS_FOR_PERFORMANCE 40.0f
#define EEMF_CALC_SPEED 125.663706144f //300RPM/60*4*2PI=125.663706144
#define EEMF_START_SPEED 251.327412287f  //600RPM/60*4*2PI=251.327412287
#define SENSORLESS_BASE_TEMP 20.0f
#define Factor_to_cal_power_from_dq 1.5f
#define Root_of_One_Third 0.577350269f

#define POC UNUSE_FUNCTION
#define EVT USE_FUNCTION

#define JUDGE_FUNCTION_DELAY UNUSE_FUNCTION
#define MEASURE_CPU_LOAD USE_FUNCTION
#define MEASURE_CPU_LOAD_ADC_INJ USE_FUNCTION
#define MEASURE_CPU_LOAD_CURRENTLOOP USE_FUNCTION
#define MEASURE_CPU_LOAD_PLCLOOP USE_FUNCTION

#define MOTOR_P1 	0
#define MOTOR_P1_3 	1
#define MOTOR_STAGE MOTOR_P1_3

//Function status of use
#define USE_EEMF UNUSE_FUNCTION
#define USE_CALC_SUM_ROOT UNUSE_FUNCTION
#define USE_THERMAL_DERATING USE_FUNCTION
#define USE_DATA_RECORD UNUSE_FUNCTION
#define USE_HFI_SIN UNUSE_FUNCTION
#define USE_UART_COMMUNICATION UNUSE_FUNCTION
#define USE_THROTTLE_CALIBRATION UNUSE_FUNCTION
#define USE_CURRENT_CALIBRATION USE_FUNCTION
#define USE_VOLTAGE_CALIBRATION USE_FUNCTION
#define USE_PWM_RC_FUNCTION	USE_FUNCTION
#define USE_BME060_MOSV_BUG USE_FUNCTION
#define USE_ACTIVE_CODE_VERIFY	UNUSE_FUNCTION
#define USE_FOIL_ABNORMAL_DETECT USE_FUNCTION
#define USE_ANALOG_FOIL_SENSOR_FUNC USE_FUNCTION

#endif /* INC_CONSTANTPARAMANDUSEFUNCTION_H_ */
