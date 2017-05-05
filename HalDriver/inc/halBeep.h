#ifndef __HALBEEP_H
#define __HALBEEP_H

#include "stm32f10x.h"

extern void halBeepInit(void);
extern void halBeepOn(uint32_t freq);
extern void halBeepOff(void);

#endif

