#include "halADC12.h"

uint16_t adcBuf[ADC_NUMBER_PER_CH][ADC_CH_NUM];

void halADC12Init(void) {
 //1. 时钟
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1,  ENABLE);
  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);
 
  //2. GPIO 初始化
  GPIO_InitTypeDef GPIO_InitStructure;
  GPIO_InitStructure.GPIO_Pin  = GPIO_Pin_0|GPIO_Pin_1|GPIO_Pin_2|GPIO_Pin_3;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
  GPIO_Init(GPIOC, &GPIO_InitStructure);
  
  //3. ADC 初始化
  ADC_DeInit(ADC1);
  
  ADC_InitTypeDef ADC_InitStructure;     
  ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;                   //ADC1和ADC2工作在独立模式
  ADC_InitStructure.ADC_ScanConvMode = ENABLE;                         //使能扫描
  ADC_InitStructure.ADC_ContinuousConvMode = ENABLE;                   //ADC转换工作在连续模式
  ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;  //有软件控制转换
  ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;               //转换数据右对齐
  ADC_InitStructure.ADC_NbrOfChannel = ADC_CH_NUM;                     //连续转换通道数为8
  ADC_Init(ADC1, &ADC_InitStructure);                                  //初始化ADC

  //3. 2. 4. 1
  RCC_ADCCLKConfig(RCC_PCLK2_Div8);
  ADC_RegularChannelConfig(ADC1, ADC_Channel_13, 1, ADC_SampleTime_239Cycles5 );
  ADC_RegularChannelConfig(ADC1, ADC_Channel_12, 2, ADC_SampleTime_239Cycles5 );
  ADC_RegularChannelConfig(ADC1, ADC_Channel_11, 3, ADC_SampleTime_239Cycles5 );
  ADC_RegularChannelConfig(ADC1, ADC_Channel_10, 4, ADC_SampleTime_239Cycles5 );

  ADC_DMACmd(ADC1, ENABLE);                                            //使能ADC1模块DMA
  ADC_Cmd(ADC1, ENABLE);                                               //使能ADC1
  
  ADC_ResetCalibration(ADC1);                                          //重置ADC1校准寄存器
  while(ADC_GetResetCalibrationStatus(ADC1));                          //等待ADC1校准重置完成
  ADC_StartCalibration(ADC1);                                          //开始ADC1校准
  while(ADC_GetCalibrationStatus(ADC1));                               //等待ADC1校准完成
  ADC_SoftwareStartConvCmd(ADC1, ENABLE);                              //使能ADC1软件开始转换x 
  
  //4. DMA配置
  DMA_InitTypeDef DMA_InitStructure;
  DMA_DeInit(DMA1_Channel1);                                                  //将DMA的通道1寄存器重设为缺省值
  DMA_InitStructure.DMA_PeripheralBaseAddr = (u32)(&ADC1->DR);                //DMA外设ADC基地址
  DMA_InitStructure.DMA_MemoryBaseAddr = (u32)adcBuf;                         //DMA内存基地址
  DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;                          //内存作为数据传输的目的地
  DMA_InitStructure.DMA_BufferSize = ADC_BUF_SIZE;                            //DMA通道的DMA缓存的大小
  DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;            //外设地址寄存器不变
  DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;                     //内存地址寄存器递增
  DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord; //数据宽度为16位
  DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;         //数据宽度为16位
  DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;                             //工作在循环缓存模式
  DMA_InitStructure.DMA_Priority = DMA_Priority_High;                         //DMA通道 x拥有高优先级
  DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;                                //DMA通道x没有设置为内存到内存传输
  DMA_Init(DMA1_Channel1, &DMA_InitStructure);                                //根据DMA_InitStruct中指定的参数初始化DMA的通道  
  DMA_Cmd(DMA1_Channel1, ENABLE);
}

////////////////////////////////////////////////////////////////////////////////
//开启DMA 开始采样
////////////////////////////////////////////////////////////////////////////////
void halADC1Start(void) {
  ADC_SoftwareStartConvCmd(ADC1, ENABLE);
  DMA_Cmd(DMA1_Channel1, ENABLE);
}

////////////////////////////////////////////////////////////////////////////////
//获得滤波后数据
////////////////////////////////////////////////////////////////////////////////
uint16_t halADC1GetData(uint8_t ch) {
  uint16_t j;
  uint32_t sum;
  sum = 0;
  DMA_Cmd(DMA1_Channel1, DISABLE);
  for (j=0;j<ADC_NUMBER_PER_CH;j++) {
    sum += adcBuf[j][ch];
  }
  DMA_Cmd(DMA1_Channel1, ENABLE);
  return sum/ADC_NUMBER_PER_CH;
}


