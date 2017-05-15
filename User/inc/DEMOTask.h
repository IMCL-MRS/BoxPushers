#ifndef __DEMOTASK_H
#define __DEMOTASK_H

#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"

#include "halLed.h"

extern void DEMOTask( void *pvParameters );
extern void vPushLineTask( void *pvParameters);
extern void BEEP_DEMOTask( void *pvParameters );
extern void COMPASSCALITask(void *pvParameters);
extern void COORDINATECALITask(void *pvParameters) ;

extern void infSensorTest( void *pvParameters);
extern void gotoPointTest(void *pvParameters);
#endif