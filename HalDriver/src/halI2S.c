#include "halI2S.h"
#include "halMCUWait.h"

uint16_t I2SDMARxBuf[I2S_DMA_BUF_SIZE];



void halI2SInit(void){
  //1. RCC
  RCC_APB2PeriphClockCmd( RCC_APB2Periph_GPIOB | RCC_APB2Periph_GPIOD | RCC_APB2Periph_AFIO , ENABLE);
  RCC_APB1PeriphClockCmd( RCC_APB1Periph_SPI2, ENABLE);
  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);
  
  //2. GPIO
  GPIO_InitTypeDef  GPIO_InitStructure;
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12 | GPIO_Pin_13 | GPIO_Pin_15;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
  GPIO_Init(GPIOB, &GPIO_InitStructure);    
  
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8 | GPIO_Pin_9;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_Init(GPIOD, &GPIO_InitStructure);

  GPIO_SetBits(GPIOD,GPIO_Pin_9);      //ENABLE MICROPHONE
  GPIO_ResetBits(GPIOD,GPIO_Pin_8);    //SET LEFT CHANNEL
  //GPIO_SetBits(GPIOD,GPIO_Pin_8);    //SET LEFT CHANNEL
  
  //3. I2S  
  I2S_InitTypeDef I2S_InitStructure; 
  I2S_Cmd( SPI2, DISABLE );
  SPI_I2S_DeInit(SPI2); 
  I2S_StructInit(&I2S_InitStructure);
  I2S_InitStructure.I2S_AudioFreq = I2S_AudioFreq_8k;      //Flat frequency response from 60 Hz to 15 kHz, It should be higher
  I2S_InitStructure.I2S_Standard = I2S_Standard_Phillips;
  //I2S_InitStructure.I2S_Standard = I2S_Standard_MSB;
  
  I2S_InitStructure.I2S_DataFormat = I2S_DataFormat_24b;
  I2S_InitStructure.I2S_CPOL = I2S_CPOL_High;
  I2S_InitStructure.I2S_Mode = I2S_Mode_MasterRx;
  I2S_InitStructure.I2S_MCLKOutput = I2S_MCLKOutput_Enable;
  
  I2S_Init(SPI2, &I2S_InitStructure);
  //4. DMA
  
  DMA_InitTypeDef DMA_InitStructure;
  DMA_DeInit(DMA1_Channel4);
  DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)(&SPI2->DR);
  DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)(I2SDMARxBuf);
  DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;
  DMA_InitStructure.DMA_BufferSize = I2S_DMA_BUF_SIZE;
  DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
  DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
  DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;  
  DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;
  DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
  //DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
  
  DMA_InitStructure.DMA_Priority = DMA_Priority_High;
  DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
  DMA_Init(DMA1_Channel4, &DMA_InitStructure);
  SPI_I2S_DMACmd(SPI2, SPI_I2S_DMAReq_Rx, ENABLE);
  //DMA_Cmd(DMA1_Channel4, ENABLE);
  //I2S_Cmd( SPI2, ENABLE );
}

void halI2SStartSample(void) {
  DMA_SetCurrDataCounter(DMA1_Channel4, I2S_DMA_BUF_SIZE);
  DMA_Cmd(DMA1_Channel4, ENABLE);
  I2S_Cmd(SPI2, ENABLE );
}

void halI2SStopSample(void) {
  I2S_Cmd(SPI2, DISABLE );
  DMA_Cmd(DMA1_Channel4, DISABLE);
}

uint16_t* halI2SReadMicroPhone(void) {
  
  if ((I2SDMARxBuf[0] == 0) && (I2SDMARxBuf[1] != 0)) {
    return (I2SDMARxBuf+1);
  }
  else if ((I2SDMARxBuf[0] == 0) && (I2SDMARxBuf[1] == 0)) {
    return (I2SDMARxBuf+2);
  }
  else if ((I2SDMARxBuf[0] != 0) && (I2SDMARxBuf[1] != 0)) {
    return (I2SDMARxBuf+0);
  }
  else {
    return (I2SDMARxBuf+1);
  }
  
}


