#include "RADIOTask.h"


extern xQueueHandle xQueueHandleRFTx, xQueueHandleRFRx;

void RADIORxTask(void *pvParameters ){
  portBASE_TYPE xStatus;

  while(1) {
    uint8_t RFRxData[32];
    xStatus = xQueueReceive(xQueueHandleRFRx, RFRxData ,portMAX_DELAY);
    if (xStatus==pdPASS) {
      //USER START.....
      halSetLedStatus(LED_GREEN, LED_TOGGLE);
      
      //USER END.......
    }
  }
}

extern uint8_t RFTxDone;
void RADIOTxTask( void *pvParameters ){
  portBASE_TYPE xStatus;

  while(1) {
    uint8_t RFTxData[32];
    xStatus = xQueueReceive(xQueueHandleRFTx, RFTxData ,portMAX_DELAY);
    if (xStatus==pdPASS) {
      RFTxDone = 0;
      while (halnRF24L01PBroadcastPacket(RFTxData, 32)==0) {
        vTaskDelay(3);
      }
      while (RFTxDone==0) {
        vTaskDelay(3);
      }
      halnRF24L01PRxPacket();
    }
  }
}

