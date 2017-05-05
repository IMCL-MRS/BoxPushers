#include "halButton.h"

////////////////////////////////////////////////////////////////////////////////
//B1 -> PG3  B2 -> PG4  B3 -> PG5
////////////////////////////////////////////////////////////////////////////////
void halButtonInit(void){
  //1. RCC
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOG, ENABLE);
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
  //2. GPIO
  GPIO_InitTypeDef  GPIO_InitStructure;
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3 | GPIO_Pin_4 | GPIO_Pin_5 ;  
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;                                
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;                               
  GPIO_Init(GPIOG, &GPIO_InitStructure); 
  //3. EXIT
  EXTI_InitTypeDef EXTI_InitStructure;
  EXTI_InitStructure.EXTI_Line    = GPIO_Pin_3 | GPIO_Pin_4 ;              //外部中断线
  EXTI_InitStructure.EXTI_Mode    = EXTI_Mode_Interrupt;                   //中断模式
  EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;                  //中断触发方式
  EXTI_InitStructure.EXTI_LineCmd = ENABLE;                                //打开中断
  EXTI_Init(&EXTI_InitStructure);
  
  GPIO_EXTILineConfig(GPIO_PortSourceGPIOG, GPIO_PinSource3);
  GPIO_EXTILineConfig(GPIO_PortSourceGPIOG, GPIO_PinSource4);
  
  //NVIC
  NVIC_InitTypeDef NVIC_InitStructure;
  
  EXTI_ClearFlag(EXTI_Line3);
  NVIC_InitStructure.NVIC_IRQChannel = EXTI3_IRQn;              
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=2;        
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
  
  EXTI_ClearFlag(EXTI_Line4);
  NVIC_InitStructure.NVIC_IRQChannel = EXTI4_IRQn;              
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=2;        
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure); 
  
}

__weak void EXTI3ISR_BUTTON(void){
  
}

__weak void EXTI4ISR(void){
  
}

__weak void EXTI5ISR_BUTTON(void){
  
}
