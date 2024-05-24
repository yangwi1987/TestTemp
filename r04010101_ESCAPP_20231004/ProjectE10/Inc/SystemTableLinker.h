/*
 * SystemTableLinker.h
 *
 *  Created on: 2020年4月24日
 *      Author: Mike.Wen.SFW
 */
 
#if E10
#ifndef INC_SYSTEMTABLELINKER_H_
#define INC_SYSTEMTABLELINKER_H_

#include "UtilityBase.h"
#include "stdint.h"

typedef struct /*__attribute__((__packed__))*/
{
	uint16_t Version[4];
} System_Table_t_Linker;

extern const System_Table_t_Linker SystemTable;
extern uint16_t SystemTable_FunEnable(uint16_t FunID);

#endif /* INC_SYSTEMTABLELINKER_H_ */
#endif /* E10 */
