#include "halInfrared.h"
#include "halLed.h"

////////////////////////////////////////////////////////////////////////////////
//PA3->INFRARED SENSOR INT
////////////////////////////////////////////////////////////////////////////////

void halInfraredInit(void){
  //1. RCC
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
  //2. GPIO
  GPIO_InitTypeDef  GPIO_InitStructure;
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;  
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;                                
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;                               
  GPIO_Init(GPIOA, &GPIO_InitStructure); 
  //3. EXIT
  EXTI_InitTypeDef EXTI_InitStructure;
  EXTI_InitStructure.EXTI_Line    = GPIO_Pin_3;                            //外部中断线
  EXTI_InitStructure.EXTI_Mode    = EXTI_Mode_Interrupt;                   //中断模式
  EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising_Falling;           //中断触发方式
  EXTI_InitStructure.EXTI_LineCmd = ENABLE;                                //打开中断
  EXTI_Init(&EXTI_InitStructure);
  
  GPIO_EXTILineConfig(GPIO_PortSourceGPIOA, GPIO_PinSource3);
  
  //NVIC
  NVIC_InitTypeDef NVIC_InitStructure;
  NVIC_InitStructure.NVIC_IRQChannel = EXTI3_IRQn;           
  NVIC_InitStructure.NVIC_IRQChannelSubPriority=2;         
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
}


__weak void EXIT3ISR_INFRARED_FALLING(void){

}
__weak void EXIT3ISR_INFRARED_RISING(void){
  halSetLedStatus(LED_YELLOW, LED_TOGGLE);
}
