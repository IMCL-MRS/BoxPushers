#include "halSPI.h"

void halSPI1Init(void) {
  //1. RCC
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_SPI1,ENABLE);
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD, ENABLE);
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOG, ENABLE);
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);
  //2. GPIO
  GPIO_InitTypeDef GPIO_InitStructure;
  //SPI1_CS_RF -> PD4
  GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_4;
  GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_Out_PP;          
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOD, &GPIO_InitStructure);
  GPIO_SetBits(GPIOD, GPIO_Pin_4);
  //SPI1_CS_MPU -> PG9
  GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_9;
  GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_Out_PP;          
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOG, &GPIO_InitStructure);
  GPIO_SetBits(GPIOG, GPIO_Pin_9);
  //SPI1_CS_EXT -> PC12
  GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_12;
  GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_Out_PP;          
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOC, &GPIO_InitStructure);
  GPIO_SetBits(GPIOC, GPIO_Pin_12);
  //SCLK PIN -> PA5
  GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_5;
  GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF_PP;          
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOA,   &GPIO_InitStructure);
  //MISO PIN -> PA6
  GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_6;
  GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_IPU;              
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOA,   &GPIO_InitStructure);
  //MOSI PIN -> PA7
  GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_7;
  GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF_PP;           
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOA,   &GPIO_InitStructure);
  
  //3. SPI1配置  
  SPI_InitTypeDef SPI_InitStruct;
  SPI_InitStruct.SPI_Direction = SPI_Direction_2Lines_FullDuplex;    //全双工模式
  SPI_InitStruct.SPI_Mode = SPI_Mode_Master;
  SPI_InitStruct.SPI_DataSize = SPI_DataSize_8b;
  SPI_InitStruct.SPI_CPOL = SPI_CPOL_Low;                           //SPI_CPOL_Low->空闲模式下SCK为高电平
  SPI_InitStruct.SPI_CPHA = SPI_CPHA_1Edge;                          //SPI_CPHA_2Edge->数据采样从第二个时钟沿开始
  SPI_InitStruct.SPI_NSS =  SPI_NSS_Soft;                            //SPI_NSS_Hard
  SPI_InitStruct.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_16;  //
  SPI_InitStruct.SPI_FirstBit =  SPI_FirstBit_MSB;                   //SPI_FirstBit_LSB
  SPI_Init(SPI1, &SPI_InitStruct);
  SPI_Cmd(SPI1, ENABLE);                                             //使能SPI  
  SPI_I2S_ClearITPendingBit(SPI1, SPI_I2S_IT_RXNE);
  SPI_I2S_ClearFlag(SPI1, SPI_I2S_IT_RXNE);
}

uint8_t halSPI1RdByte(void) {
  SPI_I2S_SendData(SPI1, 0);
  while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_RXNE) == RESET);
  return SPI_I2S_ReceiveData(SPI1);
}

void halSPI1WrByte(uint8_t data){
  SPI_I2S_SendData(SPI1, data);
  while(SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_BSY) == SET);
  SPI_I2S_ReceiveData(SPI1);
}

void halSPI1RdPacket(uint8_t* data, uint8_t len){
  uint8_t i;
  for (i=0;i<len;i++) {
    SPI_I2S_SendData(SPI1, 0);
    while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_RXNE) == RESET);
    data[i] = SPI_I2S_ReceiveData(SPI1);  
  }
}

void halSPI1WrPacket(uint8_t* data, uint8_t len) {
  uint8_t i;
  for (i=0;i<len;i++) {
    SPI_I2S_SendData(SPI1, data[i]);
    while(SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_BSY) == SET);
    SPI_I2S_ReceiveData(SPI1);
  }
}

uint8_t halSPI1WrRdByte(uint8_t data){
  SPI_I2S_SendData(SPI1, data);
  while(SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_BSY) == SET);
  return SPI_I2S_ReceiveData(SPI1);
}


uint8_t halSPI1BusBusy(void) {
  if((GPIO_ReadOutputDataBit(GPIOD,GPIO_Pin_4) ==Bit_RESET) || 
     (GPIO_ReadOutputDataBit(GPIOG,GPIO_Pin_9) ==Bit_RESET) || 
     (GPIO_ReadOutputDataBit(GPIOC,GPIO_Pin_12)==Bit_RESET) ) {
     
     return 1;
  }
  else {
    return 0;
  }
  
}




