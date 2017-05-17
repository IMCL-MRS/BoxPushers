#ifndef __BCASTTASK_H
#define __BCASTTASK_H

#include "stm32f10x.h"
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"

#include "systemConfig.h"
#include "bottomBoard.h"
#include "CRC16.h"
#include "halLed.h"

#pragma pack(push)
#pragma pack(1)
typedef struct type_RFPacket{
  uint16_t id;
  int16_t  speedL;
  int16_t  speedR;
  int16_t  dir;
  int16_t  locationX;
  int16_t  locationY;
  uint8_t  infSensor;
  //int16_t  status;
  uint16_t crc16Res;
}type_RFPacket;
#pragma pack(pop)


extern void BCASTTask( void *pvParameters );

#endif

