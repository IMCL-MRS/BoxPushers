#include "halBeep.h"

////////////////////////////////////////////////////////////////////////////////
//BEEP->PE14全映射->TIM1_CH4 
////////////////////////////////////////////////////////////////////////////////
void halBeepInit(void){
  //1. RCC
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOE, ENABLE);
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1,  ENABLE);
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO,  ENABLE);
  //2. GPIO  
  GPIO_InitTypeDef GPIO_InitStructure;
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_14;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOE, &GPIO_InitStructure);
  //3. TIM1
  GPIO_PinRemapConfig(GPIO_FullRemap_TIM1, ENABLE);
  
  TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
  TIM_DeInit(TIM1);
  TIM_InternalClockConfig(TIM1);
  TIM_TimeBaseStructure.TIM_Period=((uint32_t)72000000/2000);                         //ARR的值周期10K, 自动重装载寄存器周期值
  TIM_TimeBaseStructure.TIM_Prescaler=0;                      //TIM1时钟频率的预分频值
  TIM_TimeBaseStructure.TIM_ClockDivision = 0x00;             //时钟分割值
  TIM_TimeBaseStructure.TIM_CounterMode=TIM_CounterMode_Up;   //向上计数模式
  TIM_TimeBaseStructure.TIM_RepetitionCounter = 0x0;
  TIM_TimeBaseInit(TIM1, &TIM_TimeBaseStructure);
  
  TIM_OCInitTypeDef TIMOCInitStructure;
  TIMOCInitStructure.TIM_OCMode = TIM_OCMode_PWM2;             //PWM模式1输出
  
  TIMOCInitStructure.TIM_Pulse = ((uint32_t)72000000/8000) ;   //占空比=(CCRx/ARR)*100%
  TIMOCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;     //TIM输出比较极性低有效
  TIMOCInitStructure.TIM_OutputState = TIM_OutputState_Enable; //使能输出状态选择输出比较状态
  TIMOCInitStructure.TIM_OutputNState = TIM_OutputState_Disable;
  
  TIM_OC4Init(TIM1, &TIMOCInitStructure);                      //TIM1的CH4输出
  
  TIM_CtrlPWMOutputs(TIM1,ENABLE);                             //设置TIM1的PWM输出为使能
  TIM_SetCounter(TIM1, 0);
  TIM_Cmd(TIM1, DISABLE);
}

void halBeepOn(uint32_t freq){
  TIM_CtrlPWMOutputs(TIM1,DISABLE);                            //设置TIM1的PWM输出为使能
  TIM_Cmd(TIM1, DISABLE);
  TIM_SetCounter(TIM1, 0);
  
  TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
  TIM_DeInit(TIM1);
  TIM_InternalClockConfig(TIM1);
  TIM_TimeBaseStructure.TIM_Period=((uint32_t)72000000/freq); //ARR的值周期10K, 自动重装载寄存器周期值
  TIM_TimeBaseStructure.TIM_Prescaler=0;                      //TIM1时钟频率的预分频值
  TIM_TimeBaseStructure.TIM_ClockDivision = 0x00;             //时钟分割值
  TIM_TimeBaseStructure.TIM_CounterMode=TIM_CounterMode_Up;   //向上计数模式
  TIM_TimeBaseStructure.TIM_RepetitionCounter = 0x0;
  TIM_TimeBaseInit(TIM1, &TIM_TimeBaseStructure);
  
  TIM_OCInitTypeDef TIMOCInitStructure;
  TIMOCInitStructure.TIM_OCMode = TIM_OCMode_PWM2;                 //PWM模式1输出
  
  TIMOCInitStructure.TIM_Pulse = ((uint32_t)(72000000/(freq*2))) ; //占空比=(CCRx/ARR)*100%
  TIMOCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;         //TIM输出比较极性高有效
  TIMOCInitStructure.TIM_OutputState = TIM_OutputState_Enable;     //使能输出状态选择输出比较状态
  TIMOCInitStructure.TIM_OutputNState = TIM_OutputState_Disable;     //使能输出状态选择输出比较状态
  TIM_OC4Init(TIM1, &TIMOCInitStructure);                          //TIM1的CH4输出

  TIM_SetCounter(TIM1, 0);  
  TIM_CtrlPWMOutputs(TIM1,ENABLE);                                 //设置TIM1的PWM输出为使能
  TIM_Cmd(TIM1, ENABLE);
}
void halBeepOff(void){
  TIM_SetCounter(TIM1, 0);
  TIM_CtrlPWMOutputs(TIM1,DISABLE);
  TIM_Cmd(TIM1, DISABLE);
  TIM_SetCounter(TIM1, 0);
}


