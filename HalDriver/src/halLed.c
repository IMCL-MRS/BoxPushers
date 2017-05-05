#include "halLed.h"

////////LED使用的IO口/////////////////////////////////////////////////////////
//(GPIOF, GPIO_Pin_6 | GPIO_Pin_7 | GPIO_Pin_8 | GPIO_Pin_9)
//////////////////////////////////////////////////////////////////////////////
void halLedInit(void){
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOE, ENABLE);
  
  GPIO_InitTypeDef  GPIO_InitStructure;
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9 | GPIO_Pin_11 | GPIO_Pin_13 ;  
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;                                
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;                               
  GPIO_Init(GPIOE, &GPIO_InitStructure);                                         
  GPIO_SetBits(GPIOE, GPIO_Pin_9 | GPIO_Pin_11 | GPIO_Pin_13);   
}

void halSetLedStatus(unsigned char led, unsigned char status){
  if (led==LED1) {
    if (status==LED_ON) {
      GPIO_ResetBits(GPIOE,GPIO_Pin_9);
    }
    else if (status==LED_OFF){
      GPIO_SetBits(GPIOE,GPIO_Pin_9);
    }
    else if (status==LED_TOGGLE){
      (GPIOE->ODR)^=(1<<9);
    }
  }
  else if (led==LED2) {
    if (status==LED_ON) {
      GPIO_ResetBits(GPIOE,GPIO_Pin_11);
    }
    else if (status==LED_OFF){
      GPIO_SetBits(GPIOE,GPIO_Pin_11);
    }
    else if (status==LED_TOGGLE){
      (GPIOE->ODR)^=(1<<11);
    }
  }
  else if (led==LED3) {
    if (status==LED_ON) {
      GPIO_ResetBits(GPIOE,GPIO_Pin_13);
    }
    else if (status==LED_OFF){
      GPIO_SetBits(GPIOE,GPIO_Pin_13);
    }
    else if (status==LED_TOGGLE){
      (GPIOE->ODR)^=(1<<13);
    }
  }
}

unsigned char halGetLedStatus(unsigned char led) {
  unsigned int ledODR=(GPIOE->ODR);
  if (led==LED1) {
    return !(((ledODR>>9)+0x01)&0x01);
  }
  else if (led==LED2) {
    return !(((ledODR>>11)+0x01)&0x01);
  }
  else if (led==LED3) {
    return !(((ledODR>>13)+0x01)&0x01);
  }
  return 0;
}


