#include "halI2C.h"


////////////////////////////////////////////////////////////////////////////////
//400K I2C PROTOCOL   PA8->SCL PA9->SDA
////////////////////////////////////////////////////////////////////////////////
void halI2CInit(void) {
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
  //1. 初始化GPIO PB0->I2CSDA PB1->I2CSCL
  GPIO_InitTypeDef GPIO_InitStruct;
  GPIO_InitStruct.GPIO_Pin = GPIO_Pin_8|GPIO_Pin_9; 
  GPIO_InitStruct.GPIO_Mode = GPIO_Mode_Out_OD;                                 
  GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;   
  GPIO_Init(GPIOA, &GPIO_InitStruct);
  //2. 输出高电平
  GPIO_SetBits(GPIOA, GPIO_Pin_8);  //I2CSCL
  GPIO_SetBits(GPIOA, GPIO_Pin_9);  //I2CSDA
}

bool halI2CBusy(void){
  I2C_SCL_HIGH();
  halMCUWaitUS(2);
  I2C_SDA_HIGH();
  halMCUWaitUS(2);

  if (I2C_SDA()==Bit_SET) {
    return false;
  }
  return true;
}

bool halI2CStart(void) {
  I2C_SCL_HIGH();
  halMCUWaitUS(2);
  I2C_SDA_HIGH();
  halMCUWaitUS(2);
  I2C_SDA_LOW();
  halMCUWaitUS(2);
  I2C_SCL_LOW();
  halMCUWaitUS(2);
  return true;
}

void halI2CStop(void) {
  I2C_SCL_LOW();
  halMCUWaitUS(2);
  I2C_SDA_LOW();
  halMCUWaitUS(2);
  I2C_SCL_HIGH();
  halMCUWaitUS(2);
  I2C_SDA_HIGH();
  halMCUWaitUS(2);
}

void halI2CWriteAck(void) {
  I2C_SDA_LOW();
  I2C_SCL_HIGH();
  I2C_SCL_LOW();
}

void halI2CWriteNAck(void) {
  I2C_SDA_HIGH();
  I2C_SCL_HIGH();
  I2C_SCL_LOW();
}

bool halI2CReadAck(void) {
  I2C_SDA_HIGH();
  I2C_SCL_HIGH();
  if (I2C_SDA()==Bit_RESET) {
    halMCUWaitUS(2);
    I2C_SCL_LOW();
    return true;
  }
  halMCUWaitUS(2);
  I2C_SCL_LOW();
  return false;
}

uint8_t halI2CReadByte(void) {
  uint8_t i;
  uint8_t data=0;
  I2C_SDA_HIGH();
  for (i=0;i<8;i++) {
    data<<=1;
    I2C_SCL_HIGH();
    if (I2C_SDA()==Bit_SET) {
      data+=1;
    }
    else {
      asm("NOP");
    }
    halMCUWaitUS(2);
    I2C_SCL_LOW();
  }
  return data;
}

void halI2CWriteByte(uint8_t data) {
  uint8_t i;
  for (i=0;i<8;i++) {
    if ((data&0x80)==0x80) {
      I2C_SDA_HIGH();
    }
    else {
      I2C_SDA_LOW();
    }
    halMCUWaitUS(2);
    data<<=1;
    I2C_SCL_HIGH();
    I2C_SCL_LOW();
  }
}



