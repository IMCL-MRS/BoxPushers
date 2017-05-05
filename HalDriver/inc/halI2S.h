#ifndef __HALI2S_H
#define __HALI2S_H

#include "stm32f10x.h"

#define I2S_DMA_BUF_SIZE  (1024)


extern void halI2SInit(void);


extern void halI2SStartSample(void);
extern void halI2SStopSample(void);
extern uint16_t* halI2SReadMicroPhone(void);

#endif


