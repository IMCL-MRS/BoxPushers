#ifndef __BOTTOMBOARD_H
#define __BOTTOMBOARD_H

#include "stm32f10x.h"
#include "halSPI.h"

#define CMD_SET_SPEED   (0x81)
#define CMD_GET_SPEED   (0x01)

#define CMD_SET_ROTATE  (0x82)
#define CMD_DUMMY_BYTE  (0xCC)

#define CMD_NOP         (0x00)

#define HIGH8BIT(x)  (x>>8)&0x00FF
#define LOW8BIT(x)   x&0x00FF

extern uint8_t SetRobotSpeed(int16_t sL, int16_t sR);
extern uint8_t GetRobotBStatus(uint8_t* infSensor, int16_t* sL, int16_t* sR, uint8_t* bat1248);

extern int16_t GetRobotSpeedRight(void);
extern int16_t GetRobotSpeedLeft(void);

#endif