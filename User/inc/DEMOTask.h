#ifndef __DEMOTASK_H
#define __DEMOTASK_H

#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"

#include "halLed.h"
#include "vMagTask.h"

extern void UPusher3Task( void *pvParameters );
extern void UPusher2Task( void *pvParameters );
extern void vPushLineTask( void *pvParameters);
extern void BEEP_DEMOTask( void *pvParameters );
extern void COMPASSCALITask(void *pvParameters);
extern void COORDINATECALITask(void *pvParameters);
extern void IPusherShanTask(void *pvParameters);
extern void vMagCalTask(void *pvParameters);
extern void TestTask(void *pvParameters);
extern void BOTTOMTask(void *pvParameters);
extern void ROBOT2TASK(void *pvParameters); // H bottom (no black)
extern void ROBOT3TASK(void *pvParameters); // H top (with black)
extern void ROBOT4TASK(void *pvParameters); // I
extern void UPusherAllTask(void *pvParameters);

extern void gotoPointTest(void *pvParameters);
#endif