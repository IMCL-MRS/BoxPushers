#include "bottomBoard.h"
#include "CRC8.h"
#include "halMCUWait.h"
////////////////////////////////////////////////////////////////////////////////
//读取红外传感器和速度
////////////////////////////////////////////////////////////////////////////////
uint8_t  infraredSensorStatus=0;
static int16_t  sl=0, sr=0;
static uint8_t  batV1248=0;

uint8_t spi1EXTBroken=0;

uint8_t GetRobotBStatus(uint8_t* infSensor, int16_t* sL, int16_t* sR, uint8_t* bat1248) {
  uint8_t slL,slH,srL,srH;
  uint8_t txBuf[7];
  
  spi1EXTBroken = 0;
  
  txBuf[0] = CMD_DUMMY_BYTE;
  txBuf[1] = CMD_NOP;
  txBuf[2] = 0;
  txBuf[3] = 0;
  txBuf[4] = 0;
  txBuf[5] = 0;
  txBuf[6] = CRC8(txBuf, 6);
  
  SPI1_EXT_ENABLE();
  
  halSPI1WrByte(txBuf[0]);
  halMCUWaitUS(10);
  *infSensor = halSPI1WrRdByte(txBuf[1]);
  halMCUWaitUS(10);
  slL = halSPI1WrRdByte(txBuf[2]);
  halMCUWaitUS(10);
  slH = halSPI1WrRdByte(txBuf[3]);
  halMCUWaitUS(10);
  srL = halSPI1WrRdByte(txBuf[4]);
  halMCUWaitUS(10);
  srH = halSPI1WrRdByte(txBuf[5]);
  halMCUWaitUS(10);
  *sL = (int16_t)(((uint16_t)slH<<8) + slL);
  *sR = (int16_t)(((uint16_t)srH<<8) + srL);
  
  *bat1248 = halSPI1WrRdByte(txBuf[6]);
  
  SPI1_EXT_DISABLE();
  
  if (spi1EXTBroken ==1) {
    return 0;
  }
  
  return 1;
}
////////////////////////////////////////////////////////////////////////////////
//设定速度
////////////////////////////////////////////////////////////////////////////////
int16_t robotSpeedL=0, robotSpeedR=0;

int16_t GetRobotSpeedLeft(void){
  return robotSpeedL;
}
int16_t GetRobotSpeedRight(void){
  return robotSpeedR;
}

uint8_t SetRobotSpeed(int16_t sL, int16_t sR){
  uint8_t txBuf[7];
  uint8_t slL,slH,srL,srH;
  
  robotSpeedL = sL;
  robotSpeedR = sR;

  txBuf[0] = CMD_DUMMY_BYTE;
  txBuf[1] = CMD_SET_SPEED;
  txBuf[2] = LOW8BIT(sL);
  txBuf[3] = HIGH8BIT(sL);
  txBuf[4] = LOW8BIT(sR);
  txBuf[5] = HIGH8BIT(sR);
  txBuf[6] = CRC8(txBuf, 6);
  
  spi1EXTBroken = 0;

  SPI1_EXT_ENABLE();
  halSPI1WrByte(txBuf[0]);
  halMCUWaitUS(10);
  infraredSensorStatus = halSPI1WrRdByte(txBuf[1]);
  halMCUWaitUS(10);
  slL = halSPI1WrRdByte(txBuf[2]);
  halMCUWaitUS(10);
  slH = halSPI1WrRdByte(txBuf[3]);
  halMCUWaitUS(10);
  srL = halSPI1WrRdByte(txBuf[4]);
  halMCUWaitUS(10);
  srH = halSPI1WrRdByte(txBuf[5]);
  halMCUWaitUS(10);
  sl = (int16_t)(((uint16_t)slH<<8) + slL);
  sr = (int16_t)(((uint16_t)srH<<8) + srL);
  batV1248 = halSPI1WrRdByte(txBuf[6]);
  halMCUWaitUS(10);
  SPI1_EXT_DISABLE();
  
  if (spi1EXTBroken ==1) {
    return 0;
  }

  return 1;
}

int8_t SetRobotRotate(int16_t s, int16_t angle){
  
  return 1;
}