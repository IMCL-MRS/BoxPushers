#ifndef __DEMOTASK_H
#define __DEMOTASK_H

#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"

#include "halLed.h"
#include "vMagTask.h"

extern void UPusherShanTask( void *pvParameters );
extern void vPushLineTask( void *pvParameters);
extern void BEEP_DEMOTask( void *pvParameters );
extern void COMPASSCALITask(void *pvParameters);
extern void COORDINATECALITask(void *pvParameters);
extern void IPusherShanTask(void *pvParameters);
extern void vMagCalTask(void *pvParameters);

extern void gotoPointTest(void *pvParameters);
#endif