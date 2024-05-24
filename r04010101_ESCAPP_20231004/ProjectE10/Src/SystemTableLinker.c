/*
 * SystemTableLinker.c
 *
 *  Created on: 2020年5月5日
 *      Author: Mike.Wen.SFW
 */
#if E10
#include "SystemTableLinker.h"
#include "ConstantParamAndUseFunction.h"

__attribute__((__section__(".SystemBin"),used)) const System_Table_t_Linker SystemTable = {0};


#endif
