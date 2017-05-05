#include "halMPU9250.h"
#include "halSPI.h"
#include "halMCUWait.h"
#include "halLed.h"


uint8_t spi1MPUBroken=0;
////////////////////////////////////////////////////////////////////////////////
//读取寄存器
////////////////////////////////////////////////////////////////////////////////
void halMPU9250RdReg(uint8_t addr, uint8_t* data, uint8_t len){
  uint8_t i;
  
  SPI_MPU_ENABLE();
  halSPI1WrByte(0x80 + addr);
  for (i=0;i<len;i++) {
    data[i] = halSPI1RdByte();
  }
  SPI_MPU_DISABLE();
}

////////////////////////////////////////////////////////////////////////////////
//写入寄存器
////////////////////////////////////////////////////////////////////////////////
void halMPU9250WrReg(uint8_t addr, uint8_t data){
  SPI_MPU_ENABLE();
  halSPI1WrByte(addr);
  halSPI1WrByte(data);
  SPI_MPU_DISABLE();
}

void halMPU9250I2CMasterWait(void) {
  uint8_t status=0x00;
  while((status & 0x40) != 0x40 ) {
    halMPU9250RdReg(I2C_MST_STATUS, &status, 1);
    halMCUWaitMS(2);
  }  
}

////////////////////////////////////////////////////////////////////////////////
//初始化
////////////////////////////////////////////////////////////////////////////////
void halMPU9250Init(void){
  halMPU9250WrReg(USER_CTRL,  0x30);         //使能 MPU9250 SPI 工作模式和 I2C 主机 I/F 模式 
  halMPU9250WrReg(CONFIG,     0x06);
  halMPU9250WrReg(PWR_MGMT_1, 0);
  halMPU9250WrReg(PWR_MGMT_2, 0);
  
  halMPU9250WrReg(I2C_SLV4_ADDR,0x0C);
  halMPU9250WrReg(I2C_SLV4_REG,0x0A);
  halMPU9250WrReg(I2C_SLV4_DO, 0x00);
  halMPU9250WrReg(I2C_SLV4_CTRL,0x81);
  halMPU9250I2CMasterWait();
  halMPU9250WrReg(I2C_SLV4_ADDR,0x0C);
  halMPU9250WrReg(I2C_SLV4_REG,0x0A);
  halMPU9250WrReg(I2C_SLV4_DO, 0x16);        //连续采样100HZ
  halMPU9250WrReg(I2C_SLV4_CTRL,0x81);
  halMPU9250I2CMasterWait();
  asm("NOP");
  
  uint8_t wmi=0;
  halMPU9250RdReg(REG_WHO_AM_I ,&wmi, 1);
  if( wmi== 0x71){
    halSetLedStatus(LED_RED, LED_ON);
    halMCUWaitMS(10);
    halSetLedStatus(LED_RED, LED_OFF);
  }
  
  while (wmi != 0x71) {
    halMCUWaitMS(10);
  }
  
  halMCUWaitMS(10);
}

////////////////////////////////////////////////////////////////////////////////
//input : cx  -> pionter to data store addr
//return: 0: fail  1: success
////////////////////////////////////////////////////////////////////////////////
uint8_t halMPU9250RdCompassX(int16_t* cx){ 
  uint8_t regValue[2]={0,0};
  uint8_t st;
  //读取状态寄存器ST1
  
  spi1MPUBroken=0;
  
  halMPU9250WrReg(I2C_SLV4_ADDR,0x8C);
  halMPU9250WrReg(I2C_SLV4_REG,0x02);
  halMPU9250WrReg(I2C_SLV4_CTRL,0x81);
  halMPU9250I2CMasterWait();
  halMPU9250RdReg(I2C_SLV4_DI, &st, 1);
  
  if ((st&0x01) == 0x00) {
    return 0;
  }
  
  asm ("NOP");
  //读取COMPASS XLOW
  halMPU9250WrReg(I2C_SLV4_ADDR,0x8C);
  halMPU9250WrReg(I2C_SLV4_REG,0x03);
  halMPU9250WrReg(I2C_SLV4_CTRL,0x81);
  halMPU9250I2CMasterWait();
  halMPU9250RdReg(I2C_SLV4_DI, &(regValue[0]), 1);
  //读取COMPASS XHIGH
  halMPU9250WrReg(I2C_SLV4_ADDR,0x8C);
  halMPU9250WrReg(I2C_SLV4_REG,0x04);
  halMPU9250WrReg(I2C_SLV4_CTRL,0x81);
  halMPU9250I2CMasterWait();
  halMPU9250RdReg(I2C_SLV4_DI, &(regValue[1]), 1);
  //读取状态寄存器ST1 读取结束
  halMPU9250WrReg(I2C_SLV4_ADDR,0x8C);
  halMPU9250WrReg(I2C_SLV4_REG,0x09);
  halMPU9250WrReg(I2C_SLV4_CTRL,0x81);
  halMPU9250I2CMasterWait();
  halMPU9250RdReg(I2C_SLV4_DI, &st, 1);
  asm ("NOP");
  
  *cx = *(int16_t*)(regValue);
  
  if (spi1MPUBroken==1) {
    return 0;
  }
  
  return 1;
}

uint8_t halMPU9250RdCompassY(int16_t* cx){
  uint8_t regValue[2]={0,0};
  uint8_t st;
  
  spi1MPUBroken=0;
  
  
  //读取状态寄存器ST1
  halMPU9250WrReg(I2C_SLV4_ADDR,0x8C);
  halMPU9250WrReg(I2C_SLV4_REG,0x02);
  halMPU9250WrReg(I2C_SLV4_CTRL,0x81);
  halMPU9250I2CMasterWait();
  halMPU9250RdReg(I2C_SLV4_DI, &st, 1);
  
  if ((st&0x01) == 0x00) {
    return 0;
  }
  
  asm ("NOP");
  //读取COMPASS XLOW
  halMPU9250WrReg(I2C_SLV4_ADDR,0x8C);
  halMPU9250WrReg(I2C_SLV4_REG,0x05);
  halMPU9250WrReg(I2C_SLV4_CTRL,0x81);
  halMPU9250I2CMasterWait();
  halMPU9250RdReg(I2C_SLV4_DI, &(regValue[0]), 1);
  //读取COMPASS XHIGH
  halMPU9250WrReg(I2C_SLV4_ADDR,0x8C);
  halMPU9250WrReg(I2C_SLV4_REG,0x06);
  halMPU9250WrReg(I2C_SLV4_CTRL,0x81);
  halMPU9250I2CMasterWait();
  halMPU9250RdReg(I2C_SLV4_DI, &(regValue[1]), 1);
  //读取状态寄存器ST2 读取结束
  halMPU9250WrReg(I2C_SLV4_ADDR,0x8C);
  halMPU9250WrReg(I2C_SLV4_REG,0x09);
  halMPU9250WrReg(I2C_SLV4_CTRL,0x81);
  halMPU9250I2CMasterWait();
  halMPU9250RdReg(I2C_SLV4_DI, &st, 1);
  asm ("NOP");
  
  *cx = *(int16_t*)(regValue);
  
  if (spi1MPUBroken==1) {
    return 0;
  }
  
  return 1;
}

uint8_t halMPU9250RdCompassZ(int16_t* cx){
  uint8_t regValue[2]={0,0};
  uint8_t st;
  
  
  spi1MPUBroken=0;
  
  //读取状态寄存器ST1
  halMPU9250WrReg(I2C_SLV4_ADDR,0x8C);
  halMPU9250WrReg(I2C_SLV4_REG,0x02);
  halMPU9250WrReg(I2C_SLV4_CTRL,0x81);
  halMPU9250I2CMasterWait();
  halMPU9250RdReg(I2C_SLV4_DI, &st, 1);
  
  if ((st&0x01) == 0x00) {
    return 0;
  }
  
  asm ("NOP");
  //读取COMPASS XLOW
  halMPU9250WrReg(I2C_SLV4_ADDR,0x8C);
  halMPU9250WrReg(I2C_SLV4_REG,0x07);
  halMPU9250WrReg(I2C_SLV4_CTRL,0x81);
  halMPU9250I2CMasterWait();
  halMPU9250RdReg(I2C_SLV4_DI, &(regValue[0]), 1);
  //读取COMPASS XHIGH
  halMPU9250WrReg(I2C_SLV4_ADDR,0x8C);
  halMPU9250WrReg(I2C_SLV4_REG,0x08);
  halMPU9250WrReg(I2C_SLV4_CTRL,0x81);
  halMPU9250I2CMasterWait();
  halMPU9250RdReg(I2C_SLV4_DI, &(regValue[1]), 1);
  //读取状态寄存器ST2 读取结束
  halMPU9250WrReg(I2C_SLV4_ADDR,0x8C);
  halMPU9250WrReg(I2C_SLV4_REG,0x09);
  halMPU9250WrReg(I2C_SLV4_CTRL,0x81);
  halMPU9250I2CMasterWait();
  halMPU9250RdReg(I2C_SLV4_DI, &st, 1);
  asm ("NOP");
  
  *cx = *(int16_t*)(regValue);
  
  if (spi1MPUBroken==1) {
    return 0;
  }
  
  return 1;
}

float halMPU9250RdAccelX(void){
  uint8_t regValue[2];
  spi1MPUBroken=0;
  halMPU9250RdReg(ACCEL_XOUT_L, &regValue[0], 1);
  halMPU9250RdReg(ACCEL_XOUT_H, &regValue[1], 1);
  
  if (spi1MPUBroken==1) {
    return 0;
  }
  
  return ((float)(*(int16_t*)(regValue))/16384.0);
}

float halMPU9250RdAccelY(void){
  uint8_t regValue[2];
  spi1MPUBroken=0;
  halMPU9250RdReg(ACCEL_YOUT_L, &regValue[0], 1);
  halMPU9250RdReg(ACCEL_YOUT_H, &regValue[1], 1);
  
  if (spi1MPUBroken==1) {
    return 0;
  }
  
  return ((float)(*(int16_t*)(regValue))/16384.0);  
}

float halMPU9250RdAccelZ(void){
  uint8_t regValue[2];
  spi1MPUBroken=0;
  halMPU9250RdReg(ACCEL_ZOUT_L, &regValue[0], 1);
  halMPU9250RdReg(ACCEL_ZOUT_H, &regValue[1], 1);
  
  if (spi1MPUBroken==1) {
    return 0;
  }
  
  return ((float)(*(int16_t*)(regValue))/16384.0);  
}

float halMPU9250RdGyroX(void){
  uint8_t regValue[2];
  spi1MPUBroken=0;
  halMPU9250RdReg(GYRO_XOUT_L, &regValue[0], 1);
  halMPU9250RdReg(GYRO_XOUT_H, &regValue[1], 1);
  
  if (spi1MPUBroken==1) {
    return 0;
  }
  
  return ((float)(*(int16_t*)(regValue))/131.0);  
}

float halMPU9250RdGyroY(void){
  uint8_t regValue[2];
  spi1MPUBroken=0;
  halMPU9250RdReg(GYRO_YOUT_L, &regValue[0], 1);
  halMPU9250RdReg(GYRO_YOUT_H, &regValue[1], 1);
  
  if (spi1MPUBroken==1) {
    return 0;
  }
  return ((float)(*(int16_t*)(regValue))/131.0);  
}

float halMPU9250RdGyroZ(void){
  uint8_t regValue[2];
  spi1MPUBroken=0;
  halMPU9250RdReg(GYRO_ZOUT_L, &regValue[0], 1);
  halMPU9250RdReg(GYRO_ZOUT_H, &regValue[1], 1);
  if (spi1MPUBroken==1) {
    return 0;
  }
  return ((float)(*(int16_t*)(regValue))/131.0);  
}


