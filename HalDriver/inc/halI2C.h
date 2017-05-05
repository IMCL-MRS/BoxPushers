#ifndef __HALI2C_H
#define __HALI2C_H

#include "stm32f10x.h"
#include "stdbool.h"
#include "halMCUWait.h"

////////////////////////////////////////////////////////////////////////////////
//SCLK->PB1 
//SDA->PB0
////////////////////////////////////////////////////////////////////////////////
#define   I2C_SDA_HIGH()       GPIO_SetBits(GPIOA, GPIO_Pin_9); halMCUWaitUS(1)
#define   I2C_SDA_LOW()        GPIO_ResetBits(GPIOA, GPIO_Pin_9);halMCUWaitUS(1)
#define   I2C_SCL_HIGH()       GPIO_SetBits(GPIOA, GPIO_Pin_8);halMCUWaitUS(2)
#define   I2C_SCL_LOW()        GPIO_ResetBits(GPIOA, GPIO_Pin_8);halMCUWaitUS(2)
#define   I2C_SDA()            GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_9)

extern bool halI2CStart(void);
extern void halI2CStop(void);
extern void halI2CWriteAck(void);
extern void halI2CWriteNAck(void);
extern bool halI2CReadAck(void);
extern uint8_t halI2CReadByte(void);
extern void halI2CWriteByte(uint8_t data);

extern bool halI2CBusy(void);

extern void halI2CInit(void);

#endif