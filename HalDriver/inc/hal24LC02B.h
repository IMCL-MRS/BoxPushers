#ifndef __HAL24LC02B_H
#define __HAL24LC02B_H

#include "stm32f10x.h"
#include "halI2C.h"


#define CONTROL_CODE      (0xA0)
#define CHIP_SELECT_ADDR  (0)
#define R_W_WRITE         (0)
#define R_W_READ          (1)


extern void hal24LC02BInit(void);
extern bool hal24LC02BByteWrite(uint8_t addr, uint8_t data);
extern bool hal24LC02BPageWrite(uint8_t addr, uint8_t* data, uint8_t len);
extern uint8_t hal24LC02BCurAddrRead(void);
extern bool hal24LC02BRandomRead(uint8_t addr, uint8_t* data);
extern bool hal24LC02BSeqRead(uint8_t addr, uint8_t* data, uint8_t len);


#endif

