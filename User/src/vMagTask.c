
#include "vMagTask.h"
#include "halMPU9250.h"
#include "basicmotion.h"
#include <stdint.h>
#include "FreeRTOS.h"
#include "task.h"
#include "CRC16.h"

static typeMagSensor magSensor;
typeMagSensor ReadMagSensor() {  
  while(halMPU9250RdCompassX(&magSensor.magX) == 0){ //cmopass data update every 10ms
    vTaskDelay(5);
  }  
  while(halMPU9250RdCompassY(&magSensor.magY) == 0){ 
    vTaskDelay(5);
  }  
  while(halMPU9250RdCompassZ(&magSensor.magZ) == 0){ 
    vTaskDelay(5);
  }

  return magSensor;
}

typeMagSensor data[100], nowdata;
int16_t minX, minY,maxX,maxY;
int32_t magXTmp[2],mag_sensor_x;
int32_t magYTmp[2],mag_sensor_y;

void vMagTask(void *pvParameters){
  int datan= 0;
  int i = 0;
  minX = minY = 10000;
  maxX = maxY = -10000;
  vTaskDelay(200);
  RobotRotate(15, 5);
  while(1){
    portTickType xLastWakeTime; 
    xLastWakeTime = xTaskGetTickCount(); 
    for (i = 0; i < 2; i++){
      for (datan = 0; datan < 48; ++ datan) {
        nowdata = ReadMagSensor();
        if (minX > nowdata.magX) minX = nowdata.magX;
        if (minY > nowdata.magY) minY = nowdata.magY;
        if (maxX < nowdata.magX) maxX = nowdata.magX;
        if (maxY < nowdata.magY) maxY = nowdata.magY;
        data[datan] = nowdata;
        RobotRotate(15, 5);
        vTaskDelay(500);
      }
      magXTmp[i] = ((minX + maxX) / 2);
      magYTmp[i] = ((minY + maxY) / 2);  
    }
    mag_sensor_x = (int32_t)((magXTmp[0] + magXTmp[1]) / 2);
    mag_sensor_y = (int32_t)((magYTmp[0] + magYTmp[1]) / 2);
    
    uint8_t tx[32];
    type_magPacket* p = (type_magPacket*)tx;
    
    p->id = 1;
    p->magX = mag_sensor_x;
    p->magY = mag_sensor_y;
    p->maxX = maxX;
    p->minX = minX;
    p->maxY = maxY;
    p->minY = minY;  
    p->crc16Res = CRC16(tx, sizeof(type_magPacket)-2);
    
    extern xQueueHandle xQueueHandleRFTx, xQueueHandleRFRx;
    while (xQueueSendToBack(xQueueHandleRFTx, tx, portMAX_DELAY)!=pdPASS){
      vTaskDelay(5);
    }
    
    vTaskDelayUntil(&xLastWakeTime, 200);  
  }
}