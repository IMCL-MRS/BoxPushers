#include "halUltraSound.h"


void halUltraSoundInit(void){
  //1. RCC
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
  
  //2. GPIO
  GPIO_InitTypeDef  GPIO_InitStructure;
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;  
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;                                
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;                               
  GPIO_Init(GPIOC, &GPIO_InitStructure); 
  //3. EXIT
  EXTI_InitTypeDef EXTI_InitStructure;
  EXTI_InitStructure.EXTI_Line    = GPIO_Pin_9;                            //外部中断线
  EXTI_InitStructure.EXTI_Mode    = EXTI_Mode_Interrupt;                   //中断模式
  EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;                  //中断触发方式
  EXTI_InitStructure.EXTI_LineCmd = ENABLE;                                //打开中断
  EXTI_Init(&EXTI_InitStructure);
  
  GPIO_EXTILineConfig(GPIO_PortSourceGPIOC, GPIO_PinSource9);
  
  //NVIC
  NVIC_InitTypeDef NVIC_InitStructure;
  NVIC_InitStructure.NVIC_IRQChannel = EXTI9_5_IRQn;           
  NVIC_InitStructure.NVIC_IRQChannelSubPriority=2;         
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);  
  
}

__weak void EXIT9ISR(void) {
  
}