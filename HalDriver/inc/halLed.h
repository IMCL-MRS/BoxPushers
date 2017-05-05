#ifndef __HALLED_H
#define __HALLED_H

#include "stm32f10x.h"


#define LED1        (1)
#define LED2        (2)
#define LED3        (3)

#define LED_RED        LED1
#define LED_GREEN      LED2
#define LED_YELLOW     LED3

#define LED_ON      (0)
#define LED_OFF     (1)
#define LED_TOGGLE  (2)

extern void halLedInit(void);
extern void halSetLedStatus(unsigned char led, unsigned char status);

#endif