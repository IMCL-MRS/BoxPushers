#include "BCASTTask.h"
#include "halBeep.h"
#include "basicMotion.h"


extern xQueueHandle xQueueHandleRFTx;

void BCASTTask( void *pvParameters ){
  while (1) {
    uint8_t tx[32];
    
    portTickType xLastWakeTime; 
    xLastWakeTime = xTaskGetTickCount(); 
    
    type_RFPacket* p = (type_RFPacket*)tx;
    
    p->id = ROBOT_ID;
    uint8_t infSensor, bat1248;
    int16_t sL, sR;
    while (GetRobotBStatus(&infSensor, &sL, &sR, &bat1248) == 0) {
      vTaskDelay(20);
    }
    type_coordinate tp = RobotGetPosition();
    //p->speedL = GetRobotSpeedLeft();
    //p->speedR = GetRobotSpeedRight();
    
    //if (infSensor ==  0) {
    //  halBeepOn(2100);
    //}
    //else {
    //  halBeepOff();
    //}
    p->speedL = sL;
    p->speedR = sR;
    p->infSensor = infSensor;
    p->dir    = RobotAngle2North();
    p->locationX = tp.x;
    p->locationY = tp.y;
    p->crc16Res = CRC16(tx, sizeof(type_RFPacket)-2);
    
    while (xQueueSendToBack(xQueueHandleRFTx, tx, portMAX_DELAY)!=pdPASS)  {
      vTaskDelay(5);
    }
    
    halSetLedStatus(LED_RED, LED_TOGGLE);
    
    vTaskDelayUntil(&xLastWakeTime, 200);  
  }
}