/*************************************************************************
 *
 *    Used with ICCARM and AARM.
 *
 *    Delays
 *    Centeye,Inc
 *    Alison Leonard
 *    May 29, 2011
 **************************************************************************/

#ifndef DELAYS_H
#define DELAYS_H

#include "arm_comm.h"

#define DLY_100US       450
#define DLY_10US        45
#define DLY_1US         5

void DelayResolution100us(Int32U Dly);
void DelayResolution10us(Int32U Dly);
void DelayResolution1us(Int32U Dly);

#endif