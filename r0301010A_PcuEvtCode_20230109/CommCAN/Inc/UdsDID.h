/*
 * IdsDID.h
 *
 *  Created on: 2023年3月24日
 *      Author: User
 */

#ifndef INC_UDSDID_H_
#define INC_UDSDID_H_


typedef enum
{
	DID_0xF190_Vehicle_Identification_Number        =  0xF190,
	DID_0xF180_Boot_Version                         =  0xF180,
	DID_0xF185_Fingerprint                          =  0xF185,
	DID_0xF186_Active_Diagnostic_Session            =  0xF186,
	DID_0xF188_BRP_Sw_Number                        =  0xF188,
	DID_0xF191_BRP_Hw_Number                        =  0xF191,
	DID_0xF192_Manufacturer_Hw_Number               =  0xF192,
	DID_0xF194_Manufacturer_Sw_Number               =  0xF194,
	DID_0xF18B_ECU_Build_Date                       =  0xF18B,
	DID_0xF18C_ECU_Serial_Number                    =  0xF18C,
	DID_0xF1A6_Vehicle_Model                        =  0xF1A6,
	DID_0xF1A0_Customer_Name                        =  0xF1A0,
	DID_0xF1A1_Delivery_Date                        =  0xF1A1,
	DID_0xF1CD_CDID_Raw_Value                       =  0xF1CD,
	DID_0xF100_Diagnostic_Code                      =  0xF100,
	DID_0xF1F0_Manufacturer_Calibration_Number      =  0xF1F0,
	DID_0xF1F1_Internal_Flash_Write_Times           =  0xF1F1,

	DID_0xC000_Battery_Voltage                                      = 0xC000,
	DID_0xC001_Throttle_Raw                                         = 0xC001,
	DID_0xC002_Throttle_Position                                    = 0xC002,
	DID_0xC003_Odometer                                             = 0xC003,
	DID_0xC004_Vehicle_Hour                                         = 0xC004,
	DID_0xC005_Dc_Bus_Voltage                                       = 0xC005,
	DID_0xC006_Motor_Current                                        = 0xC006,
	DID_0xC007_Motor_Input_Power                                    = 0xC007,
	DID_0xC008_Motor_Temperature                                    = 0xC008,
	DID_0xC009_Motor_Temperature_Minimum                            = 0xC009,
	DID_0xC00A_Motor_Temperature_Maximum                            = 0xC00A,
	DID_0xC00B_Motor_Speed                                          = 0xC00B,
	DID_0xC00C_Torque_Reference                                     = 0xC00C,

	DID_0xC00E_ESC_Mosfets_Center_Temperature                       = 0xC00E,
	DID_0xC00F_ESC_Mosfets_Center_Temperature_Minimum               = 0xC00F,
	DID_0xC010_ESC_Mosfets_Center_Temperature_Maximum               = 0xC010,
	DID_0xC011_ESC_Mosfets_Side_Temperature                         = 0xC011,
	DID_0xC012_ESC_Mosfets_Side_Temperature_Minimum                 = 0xC012,
	DID_0xC013_ESC_Mosfets_Side_Temperature_Maximum                 = 0xC013,
	DID_0xC014_ESC_Capacitor_Temperature                            = 0xC014,
	DID_0xC015_ESC_Capacitor_Temperature_Minimum                    = 0xC015,
	DID_0xC016_ESC_Capacitor_Temperature_Maximum                    = 0xC016,
	DID_0xC017_Sensorless_State                                     = 0xC017,
	DID_0xC018_ESC_Operation_State                                  = 0xC018,
	DID_0xC019_Current_Limit                                        = 0xC019,
	DID_0xC01A_Motor_Phase_U_Current                                = 0xC01A,
	DID_0xC01B_Motor_Phase_V_Current                                = 0xC01B,
	DID_0xC01C_Motor_Phase_W_Current                                = 0xC01C,
	DID_0xC01D_Motor_Direct_Axis_Current                            = 0xC01D,
	DID_0xC01E_Motor_Quadrature_Axis_Current                        = 0xC01E,
	DID_0xC01F_Motor_Stator_Current_Is                              = 0xC01F,
	DID_0xC020_Set_Point_For_Id                                     = 0xC020,
	DID_0xC021_Set_Point_For_Iq                                     = 0xC021,
	DID_0xC022_PWM_Frequency                                        = 0xC022,
	DID_0xC023_Set_Point_For_Vd                                     = 0xC023,
	DID_0xC024_Set_Point_For_Vq                                     = 0xC024,
	DID_0xC025_Modulation_Index                                     = 0xC025,
	DID_0xC026_Motor_Pole_Pairs                                     = 0xC026,
	DID_0xC027_Motor_R                                              = 0xC027,
	DID_0xC028_Motor_Rs_Max                                         = 0xC028,
	DID_0xC029_Motor_Ld                                             = 0xC029,
	DID_0xC02A_Motor_Lq                                             = 0xC02A,
	DID_0xC02B_Motor_Flux                                           = 0xC02B,
	DID_0xC02C_ESC_Internal_circuit_voltage                         = 0xC02C,
	DID_0xC02D_Current_Command_Limit                                = 0xC02D,
	DID_0xC02E_Temperature_Max                                      = 0xC02E,
	DID_0xC02F_Foil_Position_State                                  = 0xC02F,
	DID_0xC030_Tether_Cord_State                                    = 0xC030,
	DID_0xC031_Session_Time                                         = 0xC031,
	DID_0xC032_Session_Distance                                     = 0xC032,
	DID_0xC033_ESC_Mosfet_Center_NTC_Status                         = 0xC033,
	DID_0xC034_ESC_Mosfet_Side_NTC_Status                           = 0xC034,
	DID_0xC035_ESC_Mosfet_Cap_NTC_Status                            = 0xC035,
	DID_0xC036_Motor_NTC_Status                                     = 0xC036,
	DID_0xC037_RC_connetion_status                                  = 0xC037,
	DID_0xC038_Error_Code_from_RF                                   = 0xC038,
	DID_0xC039_BMS_State_Read_by_ESC                                = 0xC039,
	DID_0xC03A_Estimated_Time_Remaining                             = 0xC03A,
	DID_0xC03B_Foil_Position_Voltage                                = 0xC03B,

	DID_0xC200_Last_Driving_Cycle_Information_1                     = 0xC200,
	DID_0xC201_Last_Driving_Cycle_Information_2                     = 0xC201,
	DID_0xC202_Last_Driving_Cycle_Information_3                     = 0xC202,
	DID_0xC203_Last_Driving_Cycle_Information_4                     = 0xC203,
	DID_0xC204_Last_Driving_Cycle_Information_5                     = 0xC204,
	DID_0xC205_Time_Spent_Over_Speed_Derating_Occurred_during_Vehicle_Life            = 0xC205,

	DID_0xC2FF_Environmental_Data                                   = 0xC2FF,

}UdsDIDParameter_e;

typedef enum
{
	WDBI_DID_0xF190_Vehicle_Identification_Number,
	WDBI_DID_0xF18B_ECU_Build_Date,
	WDBI_DID_0xF18C_ECU_Serial_Number,
	WDBI_DID_0xF1A6_Vehicle_Model ,
	WDBI_DID_0xF1A0_Customer_Name ,
	WDBI_DID_0xF1A1_Delivery_Date ,
	WDBI_DID_0xF1CD_CDID_Raw_Value,
	WDBI_DID_0xF100_Diagnostic_Code,
	WDBI_DID_0xF1F0_Manufacturer_Calibration_Number,
	WDBI_DID_0xF188_BRP_Sw_Number,
	WDBI_DID_0xF191_BRP_Hw_Number,
	WDBI_DID_Total_Number
}UdsWDBI_DID_e;

typedef enum
{
	None,
	Initial_Angle_Align,
	Using_HFI_Algorithm,
	Using_EEMF_Algorithm,
	Switching_from_HFI_to_EEMF,
	Switching_from_EEMF_to_HFI
}UdsDIDSensorlessState_e;

typedef enum
{
	Initial_ESC,
	Standby_ESC,
	Surf_Mode,
	Foil_Mode,
	Paddle_Mode,
	Limp_Home_Mode,
	Fault_Mode,
	Dealer_Test_Mode,
	Manufacturer_Test_Mode,
	Power_Off_ESC,
}UdsDIDESCOperationState_e;

typedef enum
{
	Foil_Position_Paddle,
	Foil_Position_Surf,
	Foil_Position_Foil,
	Foil_Position_Circuit_Break,
	Foil_Position_Circuit_Short
}UdsDIDFoilPositionState_e;

typedef enum
{
	Tether_Cord_Connected,
	Tether_Cord_Disconnected
}UdsDIDTetherCordState_e;

typedef enum
{
	NTC_Normal,
	NTC_Short,
	NTC_Break
}UdsDIDNTCStatus_e;

typedef enum
{
	No_Valid_RC_is_connected,
	Valid_RC_is_connected
}UdsDIDRCConnectionStatus_e;

typedef enum
{
	BMS_SM_OFF,
	BMS_SM_INIT,
	BMS_SM_IDLE,
	BMS_SM_ACTIVE,
	BMS_SM_ERROR,
	BMS_SM_PROGRAMMING,
	BMS_SM_SHUTDOWN
}UdsDIDBMSStateReadbyESC_e;

typedef enum
{
	ESC_Mosfet_Center,
	ESC_Mosfet_Side,
	ESC_Capacitor,
	Motor
}UdsDIDTemperatureMaxItem_e;

typedef struct
{
	UdsDIDTemperatureMaxItem_e Temperature_Max_Item;
	float   Temperature_Max_Value;
}UdsDIDTemperatureMax_e;

typedef struct
{
	uint16_t Battery_Level_Over80pct;
	uint16_t Battery_Level_between_60_and_80_pct;
	uint16_t Battery_Level_between_40_and_60_pct;
	uint16_t Battery_Level_between_20_and_40_pct;
	uint16_t Battery_Level_Under20pct;
}UdsDIDTimeSpentRecordforBatteryperCycle_t;

typedef struct
{
	uint32_t Battery_Level_Over80pct;
	uint32_t Battery_Level_between_60_and_80_pct;
	uint32_t Battery_Level_between_40_and_60_pct;
	uint32_t Battery_Level_between_20_and_40_pct;
	uint32_t Battery_Level_Under20pct;
}UdsDIDTimeSpentRecordforBatteryduringVehicleLife_t;

typedef struct
{
	uint16_t System_Under_Voltage_Occurred :1;
	uint16_t ESC_Over_Temperature_Occurred :1;
	uint16_t Motor_Over_Temperature_Occurred :1;
	uint16_t Motor_Over_Speed_Occurred     :1;
	uint16_t Battery_Over_Temperature_Occurred : 1;
	uint16_t Reserved                      :11;
}UdsDIDErrorFlags_t;

typedef struct
{
    uint32_t Session_Time;
    uint8_t  Battery_Level_at_Start;
    uint8_t  Battery_Level_at_Stop;
    UdsDIDTimeSpentRecordforBatteryperCycle_t Time_Spent_Record_for_Battery_this_Cycle;
    uint16_t Time_Spent_Over_Speed_derating_Occurred_this_Cycle;
    UdsDIDErrorFlags_t Error_Flags;
    float    Maximum_Torque;
    float    Average_Torque;
}UdsDIDLastDrivingCycleInfo_t;



typedef struct
{
    float Battery_Voltage;                ;                     //
    float Dc_Bus_Voltage                  ;                     //
    float Motor_Current                   ;                     //
    float Motor_Input_Power               ;                     //
    float Motor_Direct_Axis_Current       ;                     //
    float Motor_Quadrature_Axis_Current   ;                     //
    float Set_Point_For_Id                ;                     //
    float Set_Point_For_Iq                ;                     //
    float Set_Point_For_Vd                ;                     //
    float Set_Point_For_Vq                ;                     //
    float Modulation_Index                ;                     //
    float DC_Current_Limit                ;                     //
    float Electrical_Angle                ;                     //
    float ESC_Internal_circuit_voltage    ;                     //
    float Throttle_Position               ;                     //
    float Motor_Speed                     ;                     //
    float Torque_Reference                ;                     //
    float Motor_Temperature               ;                     //
    float ESC_Mosfets_Center_Temperature  ;                     //
    float ESC_Mosfets_Side_Temperature    ;                     //
    float ESC_Capacitor_Temperature       ;                     //
    float Foil_Position_Voltage           ;                     //
    UdsDIDNTCStatus_e ESC_Mosfet_Center_NTC_Status;             //
    UdsDIDNTCStatus_e ESC_Mosfet_Side_NTC_Status;               //
    UdsDIDNTCStatus_e ESC_Cap_NTC_Status;                       //
    UdsDIDNTCStatus_e Motor_NTC_Status;                         //
    UdsDIDSensorlessState_e Sensorless_State  ;                 //
    UdsDIDESCOperationState_e ESC_Operation_State;              //
    UdsDIDRCConnectionStatus_e RC_Connection_Status         ;   //
    UdsDIDBMSStateReadbyESC_e BMS_Status_Read_By_ESC       ;    //
    uint32_t Session_Time                 ;                     //
    uint32_t Vehicle_Hour                 ;                     //
    uint32_t Error_Occurred_Counter       ;                     //  108 bytes
}UdsDTCFreezeFrameEnvironmentalData_t;

#define UdsDTCFreezeFrameEnvironmentalDataLength 108
_Static_assert( sizeof(UdsDTCFreezeFrameEnvironmentalData_t) == UdsDTCFreezeFrameEnvironmentalDataLength, "error");

typedef struct
{
	UdsWDBI_DID_e WDBI_DID_Number;
	uint8_t Valid_Session;
	int16_t Valid_Length;
	uint8_t ValidSecurityLv;
}UdsWDBI_DID_Info_t;

#endif /* INC_UDSDID_H_ */
