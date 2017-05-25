#include "halLed.h"
#include "halBeep.h"
#include "halADC12.h"
#include "halSPI.h"
#include "halMPU9250.h"
#include "halButton.h"
#include "halI2C.h"
#include "hal24LC02B.h"
#include "halTIM3.h"
#include "halUltraSound.h"
#include "halInfrared.h"
#include "halnRF24L01P.h"
#include "halI2S.h"

#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"

#include "DEMOTask.h"
#include "RADIOTask.h"
#include "BCASTTask.h"

#include "bottomBoard.h"

xQueueHandle xQueueHandleRFTx, xQueueHandleRFRx;
xSemaphoreHandle xBSB1,xBSB2;

int main(void) {
  
  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);
  
  SystemInit();
  halMCUWaitMS(100);
  halLedInit();
  halSPI1Init();
  halBeepInit();
  
  halADC12Init();
  halButtonInit();
  halI2CInit();
  hal24LC02BInit();
  halI2SInit();
  
  halMPU9250Init();
  
  SetRobotSpeed(0,0);
  
  //º”‘ÿ≤‚æ‡«˝∂Ø
  halTM3Init();
  halUltraSoundInit();
  halnRF24L01PInit();
  
  //used by button
  vSemaphoreCreateBinary(xBSB1);
  vSemaphoreCreateBinary(xBSB2);
  //used by radio
  xQueueHandleRFTx = xQueueCreate(10,  32);   
  xQueueHandleRFRx = xQueueCreate(10,  32);
  
  //KEEP THESE TASKS FOR LOCALIZATION£¨RF COMMUNICATION AND BASIC DEMO 
  xTaskCreate( RADIOTxTask, ( signed portCHAR * ) "RADIO TX", configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY+3, NULL );    
  xTaskCreate( RADIORxTask, ( signed portCHAR * ) "RADIO RX", configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY+3, NULL );  
  xTaskCreate( BCASTTask,   ( signed portCHAR * ) "BCAST",    configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY+3, NULL );  
  
  //extern void COMPASS_Task( void *pvParameters ) ;
  //xTaskCreate(COMPASS_Task,    ( signed portCHAR * ) "LED",      configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY+3, NULL );   
  //xTaskCreate( vPushLineTask,    ( signed portCHAR * ) "LED",      configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY+3, NULL );
  //xTaskCreate( IPusherShanTask,    ( signed portCHAR * ) "LED",      configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY+3, NULL );  
  //xTaskCreate( vMagCalTask,    ( signed portCHAR * ) "LED",      configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY+3, NULL );  
  //xTaskCreate( UPusherShanTask,    ( signed portCHAR * ) "LED",      configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY+3, NULL );  
  //xTaskCreate( ROBOT2TASK,    ( signed portCHAR * ) "ROBOT2TASK",      configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY+3, NULL );  
  xTaskCreate( ROBOT3TASK,    ( signed portCHAR * ) "ROBOT3TASK",      configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY+3, NULL );  
  //xTaskCreate( ROBOT4TASK,    ( signed portCHAR * ) "ROBOT4TASK",      configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY+3, NULL );  
  //xTaskCreate( TestTask,    ( signed portCHAR * ) "LED",      configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY+3, NULL );  
  //xTaskCreate( gotoPointTest,    ( signed portCHAR * ) "LED",      configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY+3, NULL );  
  vTaskStartScheduler();
  
  return 0;
}


