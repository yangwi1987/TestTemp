/*
 * UtilityBase.h
 *
 *  Created on: 2021年3月10日
 *      Author: Kevin.Kuo
 */

#ifndef INC_UTILITYBASE_H_
#define INC_UTILITYBASE_H_

// Define app version, mask and check word
#define PROJECT_CODE_X1					0x0003
#define PRODUCT_AND_BIN_TYPE_X2			0x0001
#define MAIN_RELEASE_NUMBER_YY			0x0001
#define MINOR_RELEASE_NUMBER_ZZ			0x000D
#define RELEASE_CANDIDATE_RC			0x0000
#define APP_CHECK_WORD					0x4321
#define PRODUCT_TYPE_MASK				0x00F0
#define BIN_TYPE_MASK					0x000F

// Define offset of version array
#define VERSION_PROJECT_CODE_OFFSET		0
#define VERSION_BINTYPE_OFFSET			1
#define VERSION_MAIN_RELEASE_OFFSET		2
#define VERSION_MINOR_RELEASE_OFFSET	3
#define VERSION_RC_OFFSET				4

// app version define
#define APP_VERSION	{ \
	PROJECT_CODE_X1, \
	PRODUCT_AND_BIN_TYPE_X2, \
	MAIN_RELEASE_NUMBER_YY, \
	MINOR_RELEASE_NUMBER_ZZ, \
	RELEASE_CANDIDATE_RC \
}

// Define external flash version
#define EXT_FLASH_MAIN_RELEASE_NN		0x0001
#define EXT_FLASH_MINOR_RELEASE_MM		0x0004

// Define Function Enable/ Disable
#define ENABLE_FUNC_CPU_LOADING_MEASREMENT	0

// Define global parameter setting
#define ACTIVE_AXIS_NUM 1

// Check if it is MF code or not, by checking PRODUCT_AND_BIN_TYPE_X2.
#define IS_MF_CODE_BIN_FILE	( (PRODUCT_AND_BIN_TYPE_X2 & BIN_TYPE_MASK) == 0x0E || \
							  (PRODUCT_AND_BIN_TYPE_X2 & BIN_TYPE_MASK) == 0x0F )
// Define System Suppler Identify
#define ELEMENT_1ST						'L'
#define ELEMENT_2ND						'S'
#define ELEMENT_3RD						'C'
#define ELEMENT_4TH						0x00
#define ELEMENT_5TH						0x00
#define ELEMENT_6TH						0x00
#define ELEMENT_7TH						0x00
#define ELEMENT_8TH						0x00

#define NUMBER_00						'0'
#define NUMBER_01						'0'
#define NUMBER_02						'0'
#define NUMBER_03						'0'
#define NUMBER_04						'0'
#define NUMBER_05						'0'
#define NUMBER_06						'0'
#define NUMBER_07						'0'

#define NUMBER_08						'0'
#define NUMBER_09						'0'
#define NUMBER_10						'0'
#define NUMBER_11						'0'
#define NUMBER_12						'8'
#define NUMBER_13						'0'
#define NUMBER_14						'0'
#define NUMBER_15						'A'

#define NUMBER_16						'0'
#define NUMBER_17						'0'
#define NUMBER_18						'0'
#define NUMBER_19						'0'
#define NUMBER_20						'0'
#define NUMBER_21						'1'
#define NUMBER_22						'0'
#define NUMBER_23						'0'

#define BOOT_VER_BYTE_0					0x0001
#define BOOT_VER_BYTE_1					0x000B
#define BOOT_VER_BYTE_2					0x0001
#define BOOT_VER_BYTE_3					0x0001


enum CCS_MODE_ENUM {
	CCS_MODE_CMD_NOT_CCS = 0,
	CCS_MODE_CMD_CCS,
};

#define FERNANDO_TEST 0
#define BRP 1

#endif /* INC_UTILITYBASE_H_ */
