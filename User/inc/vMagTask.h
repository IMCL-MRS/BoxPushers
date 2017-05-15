#ifndef __MAGTASK_H
#define __MAGTASK_H
#include <stdint.h>
#include "FreeRTOS.h"
#include "task.h"

#pragma pack(push)
#pragma pack(1)
typedef struct type_magPacket{
  uint16_t id;
  int16_t  magX;
  int16_t  magY;
  int16_t  maxX;
  int16_t  minX;
  int16_t  maxY;
  uint8_t  minY;
  uint16_t crc16Res;
}type_magPacket;
#pragma pack(pop)

typedef struct typeMagSensor{
  int16_t magX;
  int16_t magY;
  int16_t magZ;
}typeMagSensor;

extern typeMagSensor ReadMagSensor();
extern void vMagTask(void *pvParameters);

#endif